"""The crash report on the robot: the analysis job and the report routes (BRAIN4_SPEC 6.9, S17), installed on the
agent's app like brain_api:

    GET  /api/runs/{id}/report        200 the report JSON | 404 none | 409 {state, progress} while queued / analysing
    GET  /api/runs/{id}/report.html   the standalone report page (text/html; ?theme= is read by the page itself)
    POST /api/runs/{id}/analyze       {redo: bool} -> {job, state}; 409 while a DRIVES program runs
    GET  /api/analyze                 {state, run_id, progress, stage, queue, last}

AnalyzeJob (the brain_api.FitJob pattern): a queue of <= 4 run ids, ONE `python -m bluewave.analyze <run> --out-dir
<runs> --json-progress` process at a time, nice 10, BW_MOCK removed from its environment.  It never starts while a
DRIVES program runs, and one that starts KILLS it (state cancelled; the run goes back to the head of the queue once,
analysed when the Runner is idle again) -- the car's CPU is the race's.  The Runner enqueues every run when it ends
(params an.auto).  Nothing here parses a run file on the agent's interpreter: a report's verdict and counts are read
from `<id>.report.json` (small, cached by mtime) for the GET /api/runs rows.  Nothing here moves the car.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from collections import OrderedDict, deque

from fastapi import Body, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
QUEUE_MAX = 4
KEEP_STATES = 200


def _safe(run_id: str) -> str:
    rid = os.path.basename(str(run_id or ""))
    if not rid or rid.startswith(".") or rid != str(run_id):
        raise HTTPException(404, str(run_id))
    return rid


def report_paths(run_dir: str, rid: str) -> tuple:
    return os.path.join(run_dir, rid + ".report.json"), os.path.join(run_dir, rid + ".report.html")


class AnalyzeJob:
    """One analyzer process at a time over a small queue.  `drives_fn()` is True while a DRIVES program runs."""

    module = "bluewave.analyze"          # tests point these at a stand-in analyzer
    extra_path = ""

    def __init__(self, run_dir: str, drives_fn, note=None):
        self.run_dir, self.drives_fn, self.note = run_dir, drives_fn, note or (lambda *_a: None)
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.queue = deque()                              # run ids, oldest first
        self.requeued = set()                             # cancelled once already: never twice
        self.proc = None
        self.run_id, self.progress, self.stage, self.started = None, 0.0, None, 0.0
        self.states = OrderedDict()                       # run id -> {state, progress, error, t}
        self.last = None
        self._cache = {}                                  # path -> (mtime, summary)
        self._th = threading.Thread(target=self._loop, name="analyze-job", daemon=True)
        self._th.start()

    # ---- state
    def _set(self, rid: str, **kw) -> None:
        st = dict(self.states.get(rid) or {}, **kw, t=round(time.time(), 3))
        self.states[rid] = st
        self.states.move_to_end(rid)
        while len(self.states) > KEEP_STATES:
            self.states.popitem(last=False)

    def state_of(self, rid: str) -> dict | None:
        with self.lock:
            s = self.states.get(rid)
            if s and s.get("state") == "running" and rid == self.run_id:
                s = dict(s, progress=round(self.progress, 3), stage=self.stage)
            return dict(s) if s else None

    def pending(self, rid: str) -> bool:
        """Queued or running (a run cancelled for a drive and queued again is pending too)."""
        with self.lock:
            return rid == self.run_id or rid in self.queue

    def status(self) -> dict:
        with self.lock:
            return dict(state="running" if self.run_id else ("queued" if self.queue else "idle"), run_id=self.run_id,
                        progress=round(self.progress, 3) if self.run_id else None, stage=self.stage if self.run_id
                        else None, queue=list(self.queue), last=self.last,
                        elapsed_s=round(time.time() - self.started, 1) if self.run_id else None)

    # ---- queue
    def enqueue(self, rid: str, redo: bool = False, by: str = "auto") -> dict:
        """Queue `rid` (409 when the queue is full); a run already queued or running is not queued twice."""
        jp, _hp = report_paths(self.run_dir, rid)
        if not os.path.isfile(os.path.join(self.run_dir, rid + ".jsonl")):
            raise HTTPException(404, "no run %s" % rid)
        with self.lock:
            if rid == self.run_id or rid in self.queue:
                return dict(job=rid, state="running" if rid == self.run_id else "queued")
            if os.path.isfile(jp) and not redo:
                return dict(job=None, state="ready")
            if len(self.queue) >= QUEUE_MAX:
                raise HTTPException(409, "the analysis queue is full (%d): %s" % (QUEUE_MAX, ", ".join(self.queue)))
            self.queue.append(rid)
            self._set(rid, state="queued", progress=0.0, error=None, by=by)
        self.wake.set()
        return dict(job=rid, state="queued")

    def cancel_for_drive(self) -> bool:
        """A DRIVES program starts: kill the running analysis; it goes back to the head of the queue once."""
        with self.lock:
            p, rid = self.proc, self.run_id
            if p is None or rid is None:
                return False
            self._set(rid, state="cancelled", error="a program that drives started")
            if rid not in self.requeued:
                self.requeued.add(rid)
                self.queue.appendleft(rid)
            self.proc, self.run_id = None, None
        try:
            p.kill()
        except Exception:
            pass
        return True

    # ---- the worker
    def _loop(self):
        while True:
            self.wake.wait(1.0)
            self.wake.clear()
            while True:
                if self.drives_fn():                     # S17: never while a DRIVES program runs
                    break
                with self.lock:
                    if not self.queue or self.proc is not None:
                        break
                    rid = self.queue.popleft()
                try:
                    self._run(rid)
                except Exception as e:                   # the job thread must survive anything
                    with self.lock:
                        self._set(rid, state="failed", error="%s: %s" % (type(e).__name__, e))
                        self.proc, self.run_id = None, None

    def _cmd(self, path: str) -> list:
        return [sys.executable, "-m", self.module, path, "--out-dir", self.run_dir, "--json-progress"]

    def _run(self, rid: str) -> None:
        path = os.path.join(self.run_dir, rid + ".jsonl")
        if self.module == "bluewave.analyze" and not os.path.isfile(os.path.join(ROOT, "bluewave", "analyze.py")):
            with self.lock:
                self._set(rid, state="failed", error="the analyzer is not installed (bluewave/analyze.py): bw deploy")
                self.last = dict(run_id=rid, state="failed")
            return
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(p for p in (ROOT, self.extra_path,
                                                                     os.environ.get("PYTHONPATH", "")) if p))
        env.pop("BW_MOCK", None)                          # the analyzer never opens a robot
        kw = {}
        if hasattr(os, "nice"):
            kw["preexec_fn"] = lambda: os.nice(10)
        err = tempfile.TemporaryFile()                  # stderr to a file: a chatty analyzer never fills a pipe
        with self.lock:
            if self.drives_fn():                         # a program started between the pop and here
                self.queue.appendleft(rid)
                err.close()
                return
            proc = subprocess.Popen(self._cmd(path), cwd=ROOT, env=env, stdout=subprocess.PIPE,
                                    stderr=err, text=True, **kw)
            self.proc, self.run_id, self.progress, self.stage, self.started = proc, rid, 0.0, None, time.time()
            self._set(rid, state="running", progress=0.0, error=None)
        result = None
        for line in proc.stdout:
            try:
                m = json.loads(line)
            except ValueError:
                continue
            with self.lock:
                if proc is not self.proc:
                    break
                if "progress" in m:
                    try:
                        self.progress = float(m["progress"])
                    except (TypeError, ValueError):
                        pass
                    self.stage = m.get("stage")
                if isinstance(m.get("result"), dict):
                    result = m["result"]
        rc = proc.wait()
        try:
            err.seek(0)
            tail = err.read()[-4000:].decode(errors="replace").strip().splitlines()[-3:]
        except Exception:
            tail = []
        finally:
            err.close()
        jp, _hp = report_paths(self.run_dir, rid)
        with self.lock:
            if proc is not self.proc:                    # cancelled for a drive: its state is already set
                return
            self.proc, self.run_id = None, None
            ok = os.path.isfile(jp) and (rc == 0 or rc == 3)
            err = None if ok and rc == 0 else ("exit %s: %s" % (rc, " / ".join(tail) or "no output"))
            self._set(rid, state="ready" if ok else "failed", progress=1.0 if ok else self.progress, error=err)
            self.last = dict(run_id=rid, state="ready" if ok else "failed", error=err,
                             ms=int((time.time() - self.started) * 1000))
        if ok:
            s = self.summary(rid) or {}
            v = s.get("verdict") or (result or {}).get("verdict")
            level = "bad" if v == "CONTACT" else "ok" if v == "CLEAN" else "warn"
            self.note(level, "analysis", "%s: %s %s, near %s -> RUNS" % (
                rid, v, s.get("contact") if s.get("contact") is not None else "", s.get("near")))
        else:
            self.note("warn", "analysis", "%s: the analysis failed (%s)" % (rid, err))

    # ---- reading reports
    def summary(self, rid: str) -> dict | None:
        """{verdict, contact, near, lost} from <id>.report.json, cached by mtime; None when there is none."""
        jp, _hp = report_paths(self.run_dir, rid)
        try:
            mt = os.path.getmtime(jp)
        except OSError:
            return None
        hit = self._cache.get(jp)
        if hit and hit[0] == mt:
            return dict(hit[1])
        try:
            with open(jp, encoding="utf-8") as f:
                rep = json.load(f)
            c = rep.get("counts") or {}
            s = dict(verdict=rep.get("verdict"), contact=c.get("contact"), near=c.get("near"),
                     lost=c.get("loc_loss"), error=rep.get("error"))
        except (OSError, ValueError, AttributeError):
            s = dict(verdict=None, contact=None, near=None, lost=None, error="unreadable report")
        if len(self._cache) > 400:
            self._cache.clear()
        self._cache[jp] = (mt, s)
        return dict(s)

    def report_state(self, rid: str) -> dict:
        """The `report` block of a GET /api/runs row: {state: ready|running|queued|failed|cancelled|none, verdict,
        contact, near, lost}."""
        st = self.state_of(rid)
        if st and st.get("state") in ("running", "queued"):
            return dict(state=st["state"], verdict=None, contact=None, near=None, lost=None)
        s = self.summary(rid)
        if s is not None:
            return dict(state="failed" if s.get("error") else "ready", verdict=s.get("verdict"),
                        contact=s.get("contact"), near=s.get("near"), lost=s.get("lost"))
        if st:
            return dict(state=st.get("state") or "none", verdict=None, contact=None, near=None, lost=None)
        return dict(state="none", verdict=None, contact=None, near=None, lost=None)


def install(app, robot, runner, hub, run_dir: str, by=lambda request: "?"):
    """Register the routes; returns the AnalyzeJob (the Runner enqueues each ended run and kills the job when a DRIVES
    program starts)."""
    job = AnalyzeJob(run_dir, lambda: runner.busy() and bool(runner.drives), note=hub.note)

    @app.get("/api/runs/{run_id}/report")
    def api_report(run_id: str):
        rid = _safe(run_id)
        st = job.state_of(rid)
        if st and (st.get("state") in ("queued", "running") or job.pending(rid)):
            # a run cancelled for a drive and queued again reads `cancelled` here until it runs again: still coming
            return JSONResponse(dict(state=st["state"], progress=st.get("progress", 0.0), stage=st.get("stage"),
                                     detail="%s: %s" % (rid, st["state"])), status_code=409)
        jp, _hp = report_paths(run_dir, rid)
        if os.path.isfile(jp):
            return FileResponse(jp, media_type="application/json", headers={"Cache-Control": "no-store"})
        raise HTTPException(404, json.dumps(dict(state=(st or {}).get("state") or "none",
                                                 error=(st or {}).get("error"))))

    @app.get("/api/runs/{run_id}/report.html")
    def api_report_html(run_id: str):
        rid = _safe(run_id)
        _jp, hp = report_paths(run_dir, rid)
        if not os.path.isfile(hp):
            raise HTTPException(404, "no report page for %s" % rid)
        return FileResponse(hp, media_type="text/html", headers={"Cache-Control": "no-store"})

    @app.post("/api/runs/{run_id}/analyze")
    def api_analyze_run(run_id: str, request: Request, body: dict = Body(default={})):
        rid = _safe(run_id)
        if runner.busy() and runner.drives:
            raise HTTPException(409, "%s is running: the analysis waits until it ends" % runner.name)
        return job.enqueue(rid, redo=bool((body or {}).get("redo")), by=by(request))

    @app.get("/api/analyze")
    def api_analyze():
        return job.status()

    return job
