"""REST for the obstacle brain and the calibrate flow (BRAIN_SPEC 8.5), installed on the agent's app like map_api:

    GET  /api/perc?pts=0|1        the console's `perc` message body (the program's pose-aware perception when it is
                                  this scan's, else the pose-free one); pts=1 adds X, Y (flat int mm, car frame)
    GET  /api/tf                  cfg.sensors + the live pose as base_link, each sensor's rate and age from tel
    GET  /api/recs                runs/rec/*: id, frames, scans (W9 lines), bytes, mtime -- newest first
    POST /api/calib/fit           {rec: "latest" | id, kind: "replay" | "lidar"} -> {job}: bluewave.calib in a SEPARATE
                                  process (nice 10: the agent's interpreter and the program keep their core), one at a
                                  time, refused while a DRIVES program runs and killed when one starts
    GET  /api/calib/fit           {state: idle | running | done | failed, progress 0-1, result, rec, kind, error}
    POST /api/calib/fit/cancel

Nothing here moves the car.  The fit's result is a suggestion: the console applies its `diff` through PUT /api/params
with the reason calib:fit (history, undo), never this module.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import time

from fastapi import Body, HTTPException, Request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def rec_list(dirs) -> list:
    """Every recording under `dirs` (runs/rec of the agent and of the programs' tree; deduplicated by id), newest
    first: id, frames (JPEGs), scans (steps.jsonl lines with a lidar revolution), bytes, mtime."""
    out, seen = [], set()
    for base in dirs:
        if not base or not os.path.isdir(base):
            continue
        for rid in os.listdir(base):
            d = os.path.join(base, rid)
            if rid in seen or not os.path.isfile(os.path.join(d, "steps.jsonl")):
                continue
            seen.add(rid)
            frames = nbytes = 0
            with os.scandir(d) as it:
                for e in it:
                    if e.is_file():
                        nbytes += e.stat().st_size
                        frames += e.name.endswith(".jpg")
            scans = 0
            with open(os.path.join(d, "steps.jsonl"), "rb") as f:
                for line in f:
                    scans += line.startswith(b'{"scan"')
            out.append(dict(id=rid, frames=frames, scans=scans, bytes=nbytes, mtime=round(os.path.getmtime(d), 1),
                            dir=d))
    out.sort(key=lambda r: (r["mtime"], r["id"]), reverse=True)
    return out


class FitJob:
    """One `python -m bluewave.calib fit` process at a time; its JSON lines -> progress / result / error."""

    def __init__(self, dirs_fn, drives_fn, note=None, params_fn=None):
        self.dirs_fn, self.drives_fn, self.note = dirs_fn, drives_fn, note or (lambda *_a: None)
        self.params_fn = params_fn                       # the robot's params: for a recording without its own
        self.lock = threading.Lock()
        self.proc = None
        self.state, self.progress, self.result, self.error = "idle", 0.0, None, None
        self.rec = self.kind = self.job = self.by = None
        self.started = 0.0

    def status(self) -> dict:
        with self.lock:
            return dict(state=self.state, progress=round(self.progress, 3), result=self.result, rec=self.rec,
                        kind=self.kind, error=self.error, job=self.job, by=self.by,
                        elapsed_s=round(time.time() - self.started, 1) if self.started else None)

    def start(self, rec: str, kind: str, by: str) -> dict:
        if kind not in ("replay", "lidar"):
            raise HTTPException(400, 'kind must be "replay" or "lidar"')
        recs = rec_list(self.dirs_fn())
        if not recs:
            raise HTTPException(404, "no recording: run wro_next with record=1 first")
        r = recs[0] if rec in ("", "latest", None) else next((q for q in recs if q["id"] == rec), None)
        if r is None:
            raise HTTPException(404, "no recording %s" % rec)
        with self.lock:
            if self.state == "running":
                raise HTTPException(409, "a fit is running: %s" % self.rec)
            if self.drives_fn():
                raise HTTPException(409, "a program that drives is running: the fit waits until it ends")
            env = dict(os.environ, PYTHONPATH=ROOT + os.pathsep + os.environ.get("PYTHONPATH", ""))
            env.pop("BW_MOCK", None)                     # the fit never opens a robot
            kw = {}
            if hasattr(os, "nice"):
                kw["preexec_fn"] = lambda: os.nice(10)
            cmd = [sys.executable, "-m", "bluewave.calib", "fit", r["dir"], "--kind", kind, "--json"]
            if self.params_fn is not None and not os.path.isfile(os.path.join(r["dir"], "robot_params.json")):
                # a recording from before wro_next saved its params: the robot's current ones, never the DEFAULTS
                pf = os.path.join(r["dir"], "..", ".fit_params.json")
                with open(pf, "w", encoding="utf-8") as f:
                    json.dump(self.params_fn(), f, default=str)
                cmd += ["--params", pf]
            self.proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                         text=True, **kw)
            self.state, self.progress, self.result, self.error = "running", 0.0, None, None
            self.rec, self.kind, self.by, self.started = r["id"], kind, by, time.time()
            self.job = "fit-%d" % int(self.started)
            proc = self.proc
        threading.Thread(target=self._read, args=(proc,), name="calib-fit", daemon=True).start()
        self.note("info", "calib_fit", "camera fit on %s (%s) started by %s" % (r["id"], kind, by))
        return dict(job=self.job, rec=r["id"], kind=kind)

    def _read(self, proc):
        err_tail = []
        for line in proc.stdout:
            try:
                m = json.loads(line)
            except ValueError:
                continue
            with self.lock:
                if proc is not self.proc:
                    return
                if "progress" in m:
                    self.progress = float(m["progress"])
                if "result" in m:
                    self.result = m["result"]
                if "error" in m:
                    self.error = str(m["error"])
        rc = proc.wait()
        try:
            err_tail = proc.stderr.read().strip().splitlines()[-3:]
        except Exception:
            pass
        with self.lock:
            if proc is not self.proc or self.state != "running":
                return
            if rc == 0 and self.result is not None:
                self.state, self.progress = "done", 1.0
            else:
                self.state = "failed"
                self.error = self.error or ("exit %s: %s" % (rc, " / ".join(err_tail) or "no output"))
                self.result = dict(error=self.error)
        if self.state == "done":
            self.note("ok", "calib_fit", "camera fit done: score %s -> %s mm" % (
                self.result.get("score_start"), self.result.get("score_best")))
        else:
            self.note("warn", "calib_fit", "camera fit failed: %s" % self.error)

    def cancel(self, why: str = "cancelled") -> bool:
        with self.lock:
            p = self.proc
            if p is None or self.state != "running":
                return False
            self.state, self.error = "failed", why
            self.result = dict(error=why)
        try:
            p.kill()
        except Exception:
            pass
        return True


def install(app, robot, runner, hub, mapper, rec_dirs, by=lambda request: "?"):
    """Register the routes; returns the FitJob (the agent cancels it when a DRIVES program starts)."""
    fit = FitJob(lambda: rec_dirs, lambda: runner.busy() and bool(runner.drives), note=hub.note,
                 params_fn=lambda: robot.p)

    @app.get("/api/perc")
    def api_perc(pts: int = 0):
        body = mapper.perc_body(pts=bool(pts)) if mapper is not None else None
        if body is None:
            raise HTTPException(409, "no lidar scan")
        return body

    @app.get("/api/tf")
    def api_tf():
        cfg, tel = hub.cfg(), hub.tel()
        lv = robot.live.read(pts=False, trail=False) if getattr(robot, "live", None) is not None else {}
        frames = []
        for s in cfg.get("sensors", []):
            f = dict(s)
            if s["name"] == "base_link":
                f["parent"] = lv.get("frame") or "field"
                if lv.get("pose"):
                    f["pose"], f["age_ms"] = lv["pose"], lv.get("age_ms")
            elif s["name"] == "lidar":
                f.update(rpm=tel.get("lidar_rpm"), bins=tel.get("lidar_valid"), age_ms=tel.get("lidar_age_ms"))
            elif s["name"] == "camera":
                f.update(fps=tel.get("cam_fps"), age_ms=tel.get("cam_age_ms"))
            elif s["name"] == "depth" and tel.get("depth"):
                f.update(fps=tel["depth"].get("fps"), age_ms=tel["depth"].get("age_ms"))
            elif s["name"] == "imu":
                f.update(hz=tel.get("imu_hz"), age_ms=tel.get("imu_age_ms"))
            frames.append(f)
        return dict(t=round(time.time(), 3), frames=frames)

    @app.get("/api/recs")
    def api_recs():
        return [{k: v for k, v in r.items() if k != "dir"} for r in rec_list(rec_dirs)]

    @app.post("/api/calib/fit")
    def api_fit(request: Request, body: dict = Body(default={})):
        return fit.start(str(body.get("rec") or "latest"), str(body.get("kind") or "replay"), by(request))

    @app.get("/api/calib/fit")
    def api_fit_get():
        return fit.status()

    @app.post("/api/calib/fit/cancel")
    def api_fit_cancel():
        return {"cancelled": fit.cancel()}

    return fit
