"""BlueWave dev agent -- runs ON the robot in DEV mode (never in race mode: WRO 11.10 wants the radios off).

One process owns the hardware (hw.Robot) and serves, on port 8000 (the contract: docs/CONSOLE_SPEC.md):
    /                           the team's console (ui/index.html)
    /ws                         protocol v2: tel 20 Hz, scan <= 10 Hz, live 10 Hz, vis, sys 1 Hz, log, note; accepts
                                hello / sub / drive / stop / estop / ping (bluewave/hub.py)
    /api/status                 snapshot + program state + host health
    /api/estop  /api/arm        latched emergency stop (idempotent; one beep per latch) / release it
    /api/drive                  {v, steer, ttl_ms} -- manual drive under the hub's rules (one driver, dead-man)
    /api/servo /api/motor       raw actuation for calibration
    /api/scan                   latest lidar revolution (?compact=1&res=1|2: the WS `scan` message)
    /api/camera.jpg|.mjpg       the camera (?w= &q= &fps= &overlay=1); every part carries X-Frame-Time / X-Frame-Seq
    /api/live                   what the running program believes: pose, path, signs, lot, camera + lidar points
    /api/sys  /api/sys/action   the robot's health (sysinfo.py); confirmed system actions
    /api/tests[/{name}|/last]   component tests (tests.py) -> measured numbers + suggested params
    /api/programs[...]          list (typed params from DEFAULTS) / upload / run (?preset=) / stop / log / presets
    /api/library[/{name}]       the PROGRAMS list: note / rating / status / tags / challenge (PUT) + each one's last run
    /api/runs[/{id}[/summary|/replay]]   every program run recorded as JSON lines
    /api/params[...]            read / save (history backup first) / history / restore / undo / meta
    /api/profiles[...]          named params patches: stock_a1, bluewave_body, custom
    /api/notes                  the last 50 console notes (E-STOP, arm, program start/end, params, ...)
    /api/map[.png|.ros] /api/maps /api/cloud[.ply|.png]   SLAM's / the known-pose map and the depth cloud (map_api.py,
                                over mapper.py; the WS channels map / cloud)
    /api/perc /api/tf /api/recs /api/calib/fit[/cancel]   lidar perception, the sensor tree, the recordings, the camera
                                fit from a recorded drive in its own process (brain_api.py, BRAIN_SPEC 8.5 / 12.4)
    /api/body[/fingerprint]     which chassis this is against the applied profile (bodyid.py, BRAIN4_SPEC 8.3-8.6)
    /api/runs/{id}/report[.html] /api/runs/{id}/analyze /api/analyze   every run's crash report, made by a niced
                                analyzer process after the run (runs_api.py, BRAIN4_SPEC 6.9)
    /api/programs/{p}/run?wait=button&wait_s=120   the program starts only on the car's START button (7.5, S16)
Auth: set BW_TOKEN and send it as ?token= or the X-BW-Token header (off by default on the team's private network).
POST /api/estop never needs it; /docs and /openapi.json are not served.
Clients name themselves with X-BW-Client (`console/<device>`, `bw/<hostname>`); notes say who did what.

    uvicorn bluewave.agent:app --host 0.0.0.0 --port 8000      (BW_MOCK=1 for the simulator on a laptop)
"""
from __future__ import annotations

import asyncio
import copy
import importlib.util
import json
import os
import platform
import shutil
import threading
import time
import traceback
from collections import deque

from fastapi import Body, FastAPI, File, HTTPException, Request, UploadFile, WebSocket
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse

from . import bodyid
from . import brain_api
from . import live
from . import map_api
from . import paramstore as PS
from . import params as P
from . import progmeta
from . import runlog
from . import runs_api
from . import sysinfo
from . import tests as T
from .hub import Hub, scan_body
from .hw import Robot
from .mapper import Mapper

ROOT =os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROG_DIR = os.environ.get("BW_PROGRAMS", os.path.join(ROOT, "programs"))
RUN_DIR = os.environ.get("BW_RUNS", os.path.join(ROOT, "runs"))
UI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui", "index.html")
TOKEN = os.environ.get("BW_TOKEN", "")

app = FastAPI(title="BlueWave agent", docs_url=None, redoc_url=None, openapi_url=None)   # no unauthenticated map
robot = Robot()
robot.live = live.Bus()
os.makedirs(RUN_DIR, exist_ok=True)
# the params the running Robot was BUILT with: a save that changes one of paramstore.RESTART_KEYS (the drive backend,
# the lidar port, the camera, ...) is on disk at once but on the car only after a restart -- the console says so
BOOT_P = copy.deepcopy(robot.p)


def restart_pending() -> list:
    """The saved keys the running Robot does not use yet (PS.needs_restart of params vs BOOT_P); [] = none."""
    try:
        return PS.needs_restart(PS.diff(BOOT_P, robot.p))
    except Exception:
        return []


# wro_next's exit plans (6-9 s each on the Pi) computed before anyone presses Start: at agent start and after every
# params save, on a thread, never while a program runs (wro_next.warm_exit_plans; bluewave/park.py holds one lock)
_warm = dict(state="idle", ms=None, t=None, error=None)
_warm_lock = threading.Lock()


def warm_plans(why: str = "") -> None:
    if robot.mock and os.environ.get("BW_WARM_PLANS", "0") != "1":
        return                                           # the simulator: tests and sims plan when they need to

    def body():
        if not _warm_lock.acquire(blocking=False):
            return
        try:
            if runner.busy():
                return
            path = os.path.join(PROG_DIR, "wro_next.py")
            spec = importlib.util.spec_from_file_location("bwwarm_wro_next", path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            fn = getattr(mod, "warm_exit_plans", None)
            if fn is None:
                return
            _warm.update(state="computing", t=time.time(), error=None)
            res = fn(robot.p, (robot.p.get("race") or {}).get("params") or {})
            _warm.update(state="ready" if all(ms is not None for _m, ms in res) else "no plan",
                         ms=[ms for _m, ms in res], t=time.time())
            if any(ms and ms > 200 for _m, ms in res):
                hub.note("info", "exit_plans", "exit plans ready (%s ms%s)" % (
                    ", ".join(str(ms) for _m, ms in res), ", " + why if why else ""))
        except Exception as e:
            _warm.update(state="error", error="%s: %s" % (type(e).__name__, e))
        finally:
            _warm_lock.release()
    threading.Thread(target=body, name="warm-plans", daemon=True).start()


def _ip(request: Request) -> str:
    return request.client.host if request.client else "?"


def _by(request: Request) -> str:
    return getattr(request.state, "by", None) or "http %s" % _ip(request)


@app.middleware("http")
async def auth(request: Request, call_next):
    if request.url.path.startswith("/api"):
        estop = request.method == "POST" and request.url.path == "/api/estop"      # stopping never needs the token
        if TOKEN and not estop and TOKEN not in (request.query_params.get("token"), request.headers.get("x-bw-token")):
            return JSONResponse({"error": "token", "detail": "token missing or wrong: open the console with ?token="},
                                status_code=401)
        hub.start()
        label = "".join(ch for ch in (request.headers.get("x-bw-client") or "http") if ch.isprintable())[:40]
        request.state.by = "%s %s" % (label, _ip(request))
        # S13 (BRAIN_SPEC 8.1): a passive client -- the ROS bridge, a script polling on the robot -- never renews the
        # link watchdog, or a DRIVES program would drive on with every console closed
        if request.headers.get("x-bw-passive") != "1" and request.url.path != "/api/stream":
            hub.touch(request.state.by)
    return await call_next(request)


# ---------------------------------------------------------------------------------------------------- programs
class Runner:
    """One program at a time, in a thread, with a stop Event; its log, 20 Hz telemetry and 5 Hz live records go to
    runs/<id>.jsonl.  Run refuses to start a DRIVES program while the E-STOP is latched and never arms the robot:
    pressing Run used to clear a latched E-STOP (D1).

    Start is serialised: every check and the thread start happen under `start_lock` (tests_run takes it too), with
    `starting` set so busy() is already true.  Four Run requests fired together through a barrier were once all
    accepted, and the first program then drove on unseen by Stop, by tel.busy and by the link watchdog (measured
    2026-09-23).  Every program thread still alive is kept with its stop Event: stop() sets them all and busy() is
    true while any of them lives."""

    def __init__(self):
        self.th = None
        self.stop_ev = threading.Event()
        self.name, self.run_id, self.started, self.error = None, None, 0.0, None
        self.drives, self.manual_ok, self.by = True, False, None
        self.log = deque(maxlen=400)                     # (n, t, record), n increasing per run
        self.n = 0
        self._f = None
        self._lock = threading.Lock()
        self.start_lock = threading.Lock()               # one owner of the car: a program start or a moving test
        self.starting = False
        self._tl = threading.Lock()
        self._live = []                                  # [(thread, stop_ev)] of every program thread not yet dead
        self.waiting = False                             # wait=button: the program thread waits for the START button
        self.wait_until = 0.0                            # ...until this Unix time (the console's countdown)
        self._imu_last = 0.0                             # the newest IMU sample already in the run file (monotonic)

    def busy(self) -> bool:
        with self._tl:
            self._live = [(t, e) for t, e in self._live if t.is_alive()]
            return self.starting or bool(self._live)

    def _write(self, rec: dict):
        line = json.dumps(rec, default=str) + "\n"
        with self._lock:
            if self._f:
                self._f.write(line)

    def emit(self, x):
        t = time.time()
        line = json.dumps(dict(t=t, log=x), default=str) + "\n"
        with self._lock:
            self.n += 1
            self.log.append((self.n, t, x))
            if self._f:
                self._f.write(line)

    def records(self, since_n: int = 0):
        """(run_id, [(n, t, record)] with n > since_n) of the current or last run."""
        with self._lock:
            return self.run_id, [r for r in self.log if r[0] > since_n]

    def start(self, name: str, prm: dict, by: str = "?", wait: str = "", wait_s: float = 0.0):
        """`wait="button"` (BRAIN4_SPEC 7.5, S16): the run file opens now, the program runs only after the car's
        START button; a Stop, an E-STOP or `wait_s` (default mat.wait_s) ends the wait with nothing moved."""
        if wait not in ("", "button"):
            raise HTTPException(400, "wait must be empty or button")
        with self.start_lock:
            if self.busy():
                raise HTTPException(409, "a program is running: %s" % self.name)
            path = os.path.join(PROG_DIR, name + ".py")
            if not os.path.isfile(path):
                raise HTTPException(404, "no program %s" % name)
            meta = progmeta.meta(path)
            drives = bool(meta["drives"])
            if robot.estopped and drives:
                raise HTTPException(409, "E-STOP latched: ARM first")
            if hub.testing:
                raise HTTPException(409, "a test owns the car: %s" % hub.testing)
            drv = hub.driver() if drives else None
            if drv:
                raise HTTPException(409, "%s is driving by hand" % drv)
            if drives:
                # S15: the strap proves the applied profile is for the other chassis (the WLtoys servo limits on the
                # A1's servo, the A1's RRC motor ports on the MD13S car).  A fingerprint alone warns, never refuses
                try:
                    chk = bodyid.check(robot)
                except Exception as e:                   # a broken check never blocks, and never passes silently
                    chk = dict(ok=None, why="body check failed: %s: %s" % (type(e).__name__, e))
                if bodyid.refuses(chk, robot.p):
                    raise HTTPException(409, chk.get("why") or "body mismatch")
                if chk.get("ok") is False:
                    hub.note("warn", "body", "%s starts on a fingerprint mismatch: %s" % (name, chk.get("why")))
            if drives and fit_job.cancel("a program that drives started: fit cancelled"):
                hub.note("warn", "calib_fit", "camera fit cancelled: %s starts" % name)   # the car's CPU is the race's
            if drives and analyze_job.cancel_for_drive():                             # S17: the same for a report
                hub.note("info", "analysis", "analysis paused: %s starts (it runs again after)" % name)
            old = (self.name, self.drives, self.manual_ok)
            stop_ev = threading.Event()
            with self._tl:
                self.starting = True
                self.stop_ev = stop_ev
                self.name, self.drives, self.manual_ok = name, drives, bool(meta["manual_ok"])
            ok = False
            try:
                self._launch(name, path, prm, by, stop_ev, wait, wait_s)
                ok = True
            finally:
                with self._tl:
                    self.starting = False
                    if not ok:
                        self.name, self.drives, self.manual_ok = old

    def _launch(self, name: str, path: str, prm: dict, by: str, stop_ev: threading.Event, wait: str = "",
                wait_s: float = 0.0):
        spec = importlib.util.spec_from_file_location("bwprog_" + name, path)
        mod = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(mod)                 # a syntax error surfaces here, before anything moves
        except Exception as e:
            raise HTTPException(400, "cannot load %s: %s: %s" % (name, type(e).__name__, e))
        if not callable(getattr(mod, "run", None)):
            raise HTTPException(400, "%s has no run(robot, params, log, stop)" % name)
        if stop_ev.is_set() or (self.drives and robot.estopped):
            raise HTTPException(409, "E-STOP pressed while %s was starting" % name)
        self.error, self.started, self.by = None, time.time(), by
        now = time.time()                                # milliseconds: two runs never share a file
        run_id = "%s.%03d-%s" % (time.strftime("%Y%m%d-%H%M%S", time.localtime(now)), int(now * 1000) % 1000, name)
        f = open(os.path.join(RUN_DIR, run_id + ".jsonl"), "w", encoding="utf-8")
        with self._lock:                                 # the id and the cleared log change together: the hub's log
            self.run_id, self._f = run_id, f             # tick once read the new id with the old run's last n and
            self.log.clear()                             # then skipped every record of the new run
            self.n = 0
        self._write(dict(t=time.time(), meta=dict(program=name, params=prm, robot_params=robot.p, drives=self.drives,
                                                  manual_ok=self.manual_ok, by=by, wait=wait or None)))
        robot.run_id = run_id                            # wro_next's Recorder header names the run (W19)
        self._imu_last = time.monotonic()
        self._clock()                                    # ties the recording's monotonic time to this file (2.3, G6)
        robot.live.clear(src=name, run_id=self.run_id)
        robot.reset_yaw()
        hub.note("info", "prog_start", "%s %s by %s" % (name, "waits for START" if wait else "started", by))

        def body():
            try:
                if wait == "button" and not self._wait_button(stop_ev, wait_s):
                    return
                mod.run(robot, prm, self.emit, stop_ev)
            except Exception:
                self.error = traceback.format_exc()
                self.emit({"error": self.error})
            finally:
                if self.drives or hub.driver() is None:
                    try:
                        robot.stop()                      # a still program never cuts a hand driver's command
                    except Exception as e:                # never skip the rest: the log, the file, the end note
                        self.emit({"error": "robot.stop() at the end: %s: %s" % (type(e).__name__, e)})
                self.emit("-- program ended --")
                with self._lock:
                    fh, self._f = self._f, None
                if fh:
                    fh.close()
                robot.live.set_running(False)
                self._end_note(name)
                self._auto_analyze(run_id)

        th = threading.Thread(target=body, name="program", daemon=True)
        th.start()                                       # `starting` covers busy() until the thread is registered
        with self._tl:
            self._live.append((th, stop_ev))
            self.th = th
        threading.Thread(target=self._telemetry, name="run-telemetry", daemon=True).start()

    def _auto_analyze(self, run_id: str):
        """params an.auto (BRAIN4_SPEC 6.9): every ended run goes to the analysis queue; the job starts it once no
        DRIVES program runs (never on this thread, never in this interpreter)."""
        try:
            if int((robot.p.get("an") or {}).get("auto", 1)):
                analyze_job.enqueue(run_id, by="auto")
        except HTTPException as e:
            hub.note("warn", "analysis", "%s not analysed: %s" % (run_id, e.detail))
        except Exception as e:
            hub.note("warn", "analysis", "%s not analysed: %s: %s" % (run_id, type(e).__name__, e))

    def _end_note(self, name: str):
        if self.error:
            last = [l for l in self.error.strip().splitlines() if l.strip()]
            hub.note("bad", "prog_error", "%s failed: %s" % (name, last[-1].strip() if last else "error"))
            return
        _rid, recs = self.records(0)
        end = next((x for _n, _t, x in reversed(recs) if isinstance(x, dict) and x.get("ev") == "end"), None)
        if end:
            parts = [str(end.get("reason"))]
            if end.get("laps") is not None:
                parts.append("laps %s" % end["laps"])
            if end.get("seconds") is not None:
                parts.append("%s s" % end["seconds"])
            hub.note("info", "prog_end", "%s ended: %s" % (name, ", ".join(parts)))
        else:
            hub.note("info", "prog_end", "%s ended after %.1f s" % (name, time.time() - self.started))

    def _clock(self):
        """The `clock` record (BRAIN4_SPEC 2.3): this file's Unix time and the process's monotonic time read together,
        so a recording's frame (monotonic, steps.jsonl) is found at a run-file event without guessing (G6)."""
        t, mono = time.time(), time.monotonic()
        self._write(dict(t=t, clock=dict(mono=round(mono, 4), run_id=self.run_id,
                                         agent_started=round(hub.agent_started, 3))))

    def _wait_button(self, stop_ev: threading.Event, wait_s: float) -> bool:
        """wait=button (7.5, S16), in the program thread before mod.run: stop, drain old key events, two beeps (READY,
        as race_main), then poll KEY1 / the start GPIO every 10 ms (KEY_CLICK, KEY_RELEASE_FROM_SP or KEY_PRESSED, as
        race_main accepts); after the press 0.8 s for the hand to leave, the yaw zeroed, True.  The stop event, an
        E-STOP or the timeout end the wait with an `end` event and False: nothing has moved."""
        from . import rrc
        p = robot.p
        try:
            wait_s = float(wait_s) if wait_s else float((p.get("mat") or {}).get("wait_s", 120.0))
        except (TypeError, ValueError):
            wait_s = 120.0
        wait_s = max(5.0, min(600.0, wait_s))
        keys = {1}
        try:
            keys.add(int((p.get("buttons") or {}).get("start_key", 1)))
        except (TypeError, ValueError):
            pass
        accept = (rrc.KEY_CLICK, rrc.KEY_RELEASE_FROM_SP, rrc.KEY_PRESSED)
        if self.drives:
            robot.stop()
        for _ in range(200):                             # a press from before the wait never starts the run
            if robot.button() is None:
                break
        t0 = time.monotonic()
        self.wait_until = time.time() + wait_s
        self.waiting = True
        self.emit(dict(ev="wait_button", timeout_s=round(wait_s, 1)))

        def ended() -> str:
            if robot.estopped and self.drives:
                return "estop"
            return "stopped" if stop_ev.is_set() else ""

        try:
            try:
                robot.beep(1800)
                time.sleep(0.15)
                robot.beep(2400)
            except Exception:
                pass
            while True:
                why = ended()
                if why:
                    self.emit(dict(ev="end", reason=why, waited_s=round(time.monotonic() - t0, 1), laps=0))
                    return False
                if time.monotonic() - t0 >= wait_s:
                    self.emit(dict(ev="end", reason="no_button", waited_s=round(wait_s, 1), laps=0))
                    return False
                ev = robot.button()
                if ev and ev[0] in keys and ev[1] in accept:
                    break
                time.sleep(0.01)
            self.emit(dict(ev="button", after_s=round(time.monotonic() - t0, 2)))
            hand = time.monotonic() + 0.8                # the hand leaves the car
            while time.monotonic() < hand:
                why = ended()
                if why:
                    self.emit(dict(ev="end", reason=why, waited_s=round(time.monotonic() - t0, 1), laps=0))
                    return False
                time.sleep(0.01)
        finally:
            self.waiting = False
            self.wait_until = 0.0
        robot.reset_yaw()                                # the car may have been turned by hand while it waited
        self.started = time.time()
        return True

    def _telemetry(self):
        """tel at 20 Hz; every 4th tick (5 Hz) the program's published pose, and its path / signs / lot when they
        change, and the board's IMU samples since the last `imu` record (blackbox.take, 2.3); every 200th (10 s) a
        `clock` record.  Reading the bus here also keeps it active for every run the agent starts."""
        k = 0
        last = dict(pose=None, path_v=None, pillars=None, lot=None)
        try:
            from . import blackbox
        except Exception:                                # the analyzer's encoder missing: the run goes on without it
            blackbox = None
        while self.busy():
            self._write(dict(t=time.time(), tel=robot.snapshot()))
            k += 1
            if k % 4 == 0:
                try:
                    self._live_records(robot.live.read(pts=False, trail=False), last)
                except Exception:
                    pass
                if blackbox is not None:
                    try:
                        rec, self._imu_last = blackbox.take(robot, self._imu_last)
                        if rec:
                            self._write(dict(t=time.time(), imu=rec))
                    except Exception:
                        pass
            if k % 200 == 0:
                self._clock()
            time.sleep(0.05)

    def _live_records(self, d: dict, last: dict):
        t = time.time()
        pz = d.get("pose")
        if pz:
            rec = dict(pose=[pz["x"], pz["y"], pz["th"], pz["sxy"], pz["sth"]], state=d.get("state"),
                       laps=d.get("laps"), section=d.get("section"))
            if rec != last["pose"]:
                last["pose"] = rec
                self._write(dict(t=t, live=rec))
        if d.get("path_v") is not None and d["path_v"] != last["path_v"]:
            last["path_v"] = d["path_v"]
            self._write(dict(t=t, live=dict(path_v=d["path_v"], path=d.get("path"))))
        if d.get("pillars") and d["pillars"] != last["pillars"]:
            last["pillars"] = d["pillars"]
            self._write(dict(t=t, live=dict(pillars=d["pillars"])))
        if d.get("lot") and d["lot"] != last["lot"]:
            last["lot"] = d["lot"]
            self._write(dict(t=t, live=dict(lot=d["lot"])))

    def stop(self):
        with self._tl:
            evs = [e for _t, e in self._live] + [self.stop_ev]
        for e in evs:
            e.set()
        try:
            robot.stop()
        except Exception as e:
            print("runner stop: robot.stop() failed:", type(e).__name__, e, flush=True)


runner = Runner()
sampler = sysinfo.Sampler(RUN_DIR, robot.mock, battery=robot.battery_v)
sampler.program = lambda: (runner.busy(), bool(runner.drives))

# The latch survives the agent: systemd restarts a crashed agent (Restart=on-failure), and a Robot() comes up
# un-latched, so a latch pressed before a crash would silently vanish.  The file marks it; Arm removes it.
LATCH = os.path.join(RUN_DIR, "estop.latched")


def _latch_file(on: bool):
    try:
        if on:
            with open(LATCH, "w", encoding="utf-8") as f:
                f.write("%.3f\n" % time.time())
        elif os.path.exists(LATCH):
            os.remove(LATCH)
    except OSError as e:
        print("estop latch file:", e)


if os.path.exists(LATCH):
    with robot._lock:
        robot.estopped = True
    robot.stop()


def do_estop(by: str):
    """LATCHES first, then stops the program (and a moving test).  Idempotent: a second press stops again but does not
    beep again (D4).  Each step in its own try: the program's stop used to come first, and when the RRC's serial had
    died its robot.stop() raised before the latch was ever set (the WLtoys motor then ran on)."""
    for step in (lambda: robot.estop() if not robot.estopped else robot.stop(), runner.stop, hub.test_stop.set,
                 lambda: _latch_file(True)):
        try:
            step()
        except Exception as e:
            print("E-STOP step failed (the others still ran):", type(e).__name__, e, flush=True)
    hub.note("bad", "estop", "E-STOP by %s" % by)


hub = Hub(robot, runner, sampler, on_estop=do_estop)
hub.restart_fn = restart_pending
hub.body_fn = lambda: bodyid.check(robot)               # cfg.body and the `body` note (BRAIN4_SPEC 8.6)
if robot.estopped:
    hub.note("bad", "estop", "E-STOP still latched from before the agent restarted: Arm to drive")
# perc / map / cloud for the Map page and bw (BRAIN_SPEC 8.3-8.5): one thread that works only while subscribed
mapper = Mapper(robot, robot.live, drives=lambda: runner.busy() and bool(runner.drives))
hub.mapper = mapper
map_api.install(app, robot, runner, hub, mapper, by=_by)
# /api/perc, /api/tf, /api/recs, /api/calib/fit (BRAIN_SPEC 8.5, 12.4).  Recordings: the agent's runs/rec and the one
# wro_next writes next to its programs (the same directory on the robot; two on the laptop's mock agent)
fit_job = brain_api.install(app, robot, runner, hub, mapper,
                            [os.path.join(RUN_DIR, "rec"), os.path.join(os.path.dirname(PROG_DIR), "runs", "rec")],
                            by=_by)
# the crash report of every run: /api/runs/{id}/report[.html], /api/runs/{id}/analyze, /api/analyze (BRAIN4_SPEC 6.9)
analyze_job = runs_api.install(app, robot, runner, hub, RUN_DIR, by=_by)


@app.on_event("startup")
async def _startup():
    hub.start()
    warm_plans("agent start")
    if os.environ.get("BW_STALLWATCH", "1") != "0":
        from . import stallwatch                         # who holds the GIL in a whole-agent freeze -> runs/stalls.txt
        stallwatch.start(os.path.join(RUN_DIR, "stalls.txt"))


# ---------------------------------------------------------------------------------------------------- routes
@app.get("/")
def index():
    return FileResponse(UI, headers={"Cache-Control": "no-cache"})


def _host() -> dict:
    out = dict(host=platform.node())
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            out["cpu_c"] = int(f.read()) / 1000.0
        out["load"] = os.getloadavg()[0]
    except Exception:
        pass
    return out


@app.get("/api/status")
def status():
    # host.agent_started: when this agent process started (bw preflight: a camera verified before it = maybe moved)
    return dict(robot=robot.snapshot(), program=dict(name=runner.name, busy=runner.busy(), run_id=runner.run_id,
                                                     error=runner.error, drives=bool(runner.drives),
                                                     waiting=bool(runner.waiting)),
                host=dict(_host(), agent_started=round(hub.agent_started, 3)), restart=restart_pending(),
                exit_plans=dict(_warm))


@app.post("/api/estop")
def estop(request: Request):
    do_estop(_by(request))
    return {"estop": True}


@app.post("/api/arm")
def arm(request: Request):
    """Refused while the stopped DRIVES program's thread is still alive or a moving test still runs: arming then
    would hand the car back to code that has not seen its stop yet (an uploaded program that ignores `stop`)."""
    if runner.busy() and runner.drives:
        raise HTTPException(409, "%s has not stopped yet: wait for it to end, then Arm" % runner.name)
    if hub.testing:
        raise HTTPException(409, "the test %s has not ended yet" % hub.testing)
    robot.arm()
    _latch_file(False)
    hub.note("ok", "arm", "Armed by %s" % _by(request))
    return {"estop": False}


@app.post("/api/beep")
def beep():
    robot.beep()
    return {"ok": True}


@app.post("/api/drive")
def drive(request: Request, v: float = Body(0.0), steer: float = Body(0.0), ttl_ms: int = Body(400)):
    key = "rest %s" % _ip(request)
    ok, why = hub.drive(key, key, v, steer, ttl_ms)
    if not ok:
        raise HTTPException(409, why)
    return {"accepted": True}


def _actuation_free(request: Request):
    """The raw routes obey the same owners as driving: the latch, a program, a moving test, another console's lock."""
    if robot.estopped:
        raise HTTPException(409, "E-STOP latched")
    if runner.busy() and not runner.manual_ok:
        raise HTTPException(409, "a program owns the car")
    if hub.testing:
        raise HTTPException(409, "a test owns the car: %s" % hub.testing)
    drv = hub.driver()
    if drv and drv != _by(request):
        raise HTTPException(409, "another console is driving: %s" % drv)


@app.post("/api/servo")
def servo(request: Request, deg: float | None = Body(None), us: int | None = Body(None)):
    _actuation_free(request)
    s = robot.p["steer"]
    if us is not None:
        with robot._lock:                                # the latch checked at the moment of the write
            if robot.estopped:
                raise HTTPException(409, "E-STOP latched")
            robot.board.set_pwm_servo(s["servo_id"], max(500, min(2500, int(us))), 0.1)
        return {"us": us}
    if not robot.steer_only(deg or 0.0):
        raise HTTPException(409, "E-STOP latched")
    return {"deg": deg}


@app.post("/api/motor")
def motor(request: Request, id: int = Body(...), rps: float = Body(...), ms: int = Body(500)):
    _actuation_free(request)
    if not robot.motor_only(id, rps):
        raise HTTPException(409, "E-STOP latched or no drive motor")
    end = time.monotonic() + max(0, min(ms, 3000)) / 1000.0
    while time.monotonic() < end and not robot.estopped:
        time.sleep(0.02)
    if hasattr(robot, "motor_stop"):
        robot.motor_stop(id)                             # RRC wheel or the gpio_pwm motor, whichever drives
    else:
        robot.board.set_motors_rps({int(id): 0.0})
    return {"ok": not robot.estopped}


@app.get("/api/scan")
def scan(compact: int = 0, res: int = 1):
    sc = robot.scan
    if sc is None:
        return {"ok": False}
    if compact:
        return dict(scan_body(sc, 2.0 if res == 2 else 1.0), ok=True)
    return dict(ok=True, t_age_ms=round((time.monotonic() - sc.t) * 1000), res=sc.res, rpm=sc.rpm,
                dist=sc.dist.tolist(), conf=sc.conf.tolist())


# ---------------------------------------------------------------------------------------------------- camera
_enc_lock = threading.Lock()
_enc_cache: dict = {}                                    # (w, q, overlay) -> (frame t_mono, jpeg): one encode per frame


def _unix(t_mono: float) -> float:
    return time.time() - (time.monotonic() - t_mono)


def _encode(f, t: float, w: int, q: int, overlay: bool) -> bytes:
    key = (w, q, overlay)
    with _enc_lock:
        hit = _enc_cache.get(key)
        if hit and hit[0] == t:
            return hit[1]
    import cv2
    from . import camera as C
    img = C.overlay(f, robot.p["hsv"]) if overlay else f
    H, W = img.shape[:2]
    if w and w < W:
        img = cv2.resize(img, (w, max(1, int(round(H * w / W)))), interpolation=cv2.INTER_AREA)
    j = C.jpeg(img, q)
    with _enc_lock:
        if len(_enc_cache) > 16:
            _enc_cache.clear()
        _enc_cache[key] = (t, j)
    return j


@app.get("/api/camera.jpg")
async def camera_jpg(w: int = 0, q: int = 70, overlay: int = 0):
    f, t = robot.frame()
    if f is None:
        return JSONResponse({"error": "no camera"}, 503)
    w = max(160, min(int(w), 1280)) if w else f.shape[1]
    j = await asyncio.to_thread(_encode, f, t, w, max(30, min(int(q), 95)), bool(overlay))
    if not j:
        return JSONResponse({"error": "encode failed"}, 503)
    return Response(j, media_type="image/jpeg", headers={"X-Frame-Time": "%.3f" % _unix(t),
                                                          "Cache-Control": "no-store"})


@app.get("/api/camera.mjpg")
async def camera_mjpg(fps: int = 10, w: int = 640, q: int = 70, overlay: int = 0, n: int = 0):
    """multipart/x-mixed-replace, boundary f; a part only for a NEW frame; `n` > 0 ends after n parts."""
    fps, w, q = max(1, min(int(fps), 30)), max(160, min(int(w), 1280)), max(30, min(int(q), 95))

    async def gen():
        last, sent, nxt = None, 0, time.monotonic()
        while True:
            f, t = robot.frame()
            if f is None or t == last:
                await asyncio.sleep(0.01)
                continue
            last = t
            j = await asyncio.to_thread(_encode, f, t, w, q, bool(overlay))
            if j:
                seq = int(getattr(robot.camera, "frames", 0) or 0)
                yield (b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\nX-Frame-Time: %.3f\r\n"
                       b"X-Frame-Seq: %d\r\n\r\n" % (len(j), _unix(t), seq)) + j + b"\r\n"
                sent += 1
                if n and sent >= n:
                    return
            nxt += 1.0 / fps
            now = time.monotonic()
            if now - nxt > 0.5:
                nxt = now
            await asyncio.sleep(max(0.0, nxt - now))

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=f",
                             headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})


# ---------------------------------------------------------------------------------------------------- live / sys
@app.get("/api/live")
def live_get(pts: int = 1, path: int = 1):
    d = robot.live.read(scan=robot.scan, lidar_cfg=robot.p.get("lidar", {}), pts=bool(pts), trail=bool(path))
    if not path:
        d.pop("path", None)
        d.pop("trail", None)
    return dict(type="live", t=round(time.time(), 3), **d)


@app.get("/api/sys")
def sys_get():
    sampler.want()
    if not (runner.busy() and runner.drives):
        sampler.refresh_slow(10.0)                       # `bw sys` after an idle spell: sample now, not stale nulls
    return dict(type="sys", t=round(time.time(), 3), **sampler.snapshot())


@app.post("/api/sys/action")
def sys_action(request: Request, body: dict = Body(default={})):
    name = str(body.get("action") or "")
    if name not in sysinfo.ACTIONS:
        raise HTTPException(404, "unknown action %s (one of %s)" % (name, ", ".join(sysinfo.ACTIONS)))
    if body.get("confirm") != name:
        raise HTTPException(400, "confirm must equal the action name")
    busy = runner.busy()
    if busy and sysinfo.ACTIONS[name][1]:
        raise HTTPException(409, "a program is running: stop it first")
    r = sysinfo.action(name, body, robot.mock, busy, sampler.card)
    hub.note("ok" if r["ok"] else "warn", "sys_action",
             "%s by %s: %s" % (sysinfo.ACTIONS[name][0], _by(request), r["detail"]))
    return r


@app.get("/api/notes")
def notes():
    return list(hub.notes)


# ---------------------------------------------------------------------------------------------------- tests
@app.get("/api/tests")
def tests_list():
    return [dict(name=n, how=T.HOW.get(n, ""), moves=n in T.MOVES) for n in T.ALL]


@app.get("/api/tests/last")
def tests_last():
    out = {}
    try:
        with open(os.path.join(RUN_DIR, "tests.jsonl"), encoding="utf-8") as f:
            for line in f:
                try:
                    r = json.loads(line)
                    out[r["test"]] = dict(t=r.get("t"), args=r.get("args"), result=r.get("result"))
                except (ValueError, KeyError, TypeError):
                    pass
    except OSError:
        pass
    return out


@app.post("/api/tests/stop")
def tests_stop(request: Request):
    """Ends a moving test at its next check (<= 20 ms); the car stops."""
    name = hub.testing
    hub.test_stop.set()
    if name:
        robot.stop()
        hub.note("warn", "test_stop", "test %s stopped by %s" % (name, _by(request)))
    return {"stopped": name}


@app.post("/api/tests/{name}")
def tests_run(name: str, request: Request, args: dict = Body(default={})):
    """A moving test owns the car like a program does (the same start lock), and gets a stop Event that E-STOP,
    POST /api/tests/stop, its console's disconnect and the link watchdog set."""
    if name not in T.ALL:
        raise HTTPException(404, name)
    moves = name in T.MOVES
    args = {k: v for k, v in (args or {}).items() if k != "stop"}
    stop = threading.Event()
    with runner.start_lock:
        if runner.busy():
            raise HTTPException(409, "a program owns the car")
        if moves and robot.estopped:
            raise HTTPException(409, "E-STOP latched: ARM first")
        if hub.testing:
            raise HTTPException(409, "a test is running: %s" % hub.testing)
        if moves:
            drv = hub.driver()
            if drv:
                raise HTTPException(409, "%s is driving by hand" % drv)
            hub.testing, hub.test_by, hub.test_stop = name, _by(request), stop
    try:
        r = T.ALL[name](robot, stop=stop, **args)
    except Exception as e:
        r = dict(ok=False, summary="the test raised %s: %s" % (type(e).__name__, e),
                 data=dict(traceback=traceback.format_exc()[-1500:]), suggest={})
    finally:
        if moves:
            hub.testing = hub.test_by = None
            robot.stop()
    with open(os.path.join(RUN_DIR, "tests.jsonl"), "a", encoding="utf-8") as f:
        f.write(json.dumps(dict(t=time.time(), test=name, args=args, result=r), default=str) + "\n")
    hub.note("info" if r.get("ok") else "warn", "test",
             "%s: %s -- %s" % (name, "pass" if r.get("ok") else "check", r.get("summary")))
    return r


# ---------------------------------------------------------------------------------------------------- programs
def _prog_path(name: str) -> str:
    return os.path.join(PROG_DIR, os.path.basename(name) + ".py")


def _car_defaults(m: dict) -> dict:
    """The form starts from THIS car's numbers: params prog.<program> (a profile, e.g. wltoys) over the program's
    DEFAULTS, marked car=True.  A copy: progmeta's cached dict stays the program's own.  `race`: the challenges race
    mode runs this program for (params race.programs; the one race.challenge picks first) -- nothing on the page said
    which program is the obstacle one."""
    race = robot.p.get("race") or {}
    forced = race.get("program") or ""
    ch = [k for k, v in sorted((race.get("programs") or {}).items(), key=lambda kv: kv[0] != race.get("challenge"))
          if v == m.get("name") and not forced] + (["forced"] if forced and forced == m.get("name") else [])
    car = (robot.p.get("prog") or {}).get(m.get("name")) or {}
    out = dict(m, race=ch)
    if not car:
        return out
    return dict(out, params=[dict(q, default=car[q["key"]], car=True) if q["key"] in car else q for q in m["params"]])


def _prog_meta(fn: str):
    """progmeta.meta of programs/<fn>, or None (the file went away).  Any other failure is a row with the reason:
    one bad file (a cp1256 comment raised UnicodeDecodeError here) once emptied the whole PROGRAMS page."""
    path = os.path.join(PROG_DIR, fn)
    try:
        return _car_defaults(progmeta.meta(path))
    except FileNotFoundError:
        return None
    except Exception as e:
        return dict(progmeta.meta_error(path, e), race=[])


def _prog_names() -> set:
    """The listed programs' exact names (os.path.isfile ignores case on Windows: PUT OPEN_FAST wrote a phantom
    library entry and then failed)."""
    try:
        return {fn[:-3] for fn in os.listdir(PROG_DIR) if fn.endswith(".py") and not fn.startswith("_")}
    except OSError:
        return set()


@app.get("/api/programs")
def programs():
    out = []
    for fn in sorted(os.listdir(PROG_DIR)):
        if fn.endswith(".py") and not fn.startswith("_"):
            m = _prog_meta(fn)
            if m is not None:
                out.append(m)
    return out


@app.post("/api/programs/upload")
async def program_upload(request: Request, file: UploadFile = File(...)):
    name = os.path.basename(file.filename or "")
    if not name.endswith(".py") or "/" in name or "\\" in name:
        raise HTTPException(400, "a .py file")
    if not progmeta.name_ok(name[:-3]):
        raise HTTPException(400, "program names are 1-48 of A-Z a-z 0-9 _ -, starting with a letter or digit "
                                 "(%r is not one)" % name[:-3][:60])
    if name[:-3] in progmeta.frozen_names(PROG_DIR):
        raise HTTPException(409, "%s is FROZEN (kept as it ran): upload it under a new name, e.g. %s_v2.py"
                                 % (name[:-3], name[:-3]))
    data = await file.read()
    try:
        src = progmeta.decode_source(data)               # compile(bytes) accepts a latin-1 / cp1256 comment
    except ValueError as e:
        raise HTTPException(400, str(e))
    try:
        compile(src, name, "exec")                       # refuse a file that does not even parse
    except SyntaxError as e:
        raise HTTPException(400, "syntax error line %s: %s" % (e.lineno, e.msg))
    except ValueError as e:                              # null bytes
        raise HTTPException(400, "cannot parse: %s" % e)
    if runner.busy() and runner.name == name[:-3]:
        raise HTTPException(409, "%s is running: stop it before replacing it" % name[:-3])
    dst = os.path.join(PROG_DIR, name)
    bak = None
    if os.path.exists(dst):                              # a replaced program (the race one too) is kept, never lost
        bdir = os.path.join(PROG_DIR, ".bak")
        os.makedirs(bdir, exist_ok=True)
        bak = "%s-%s.py" % (name[:-3], time.strftime("%Y%m%d-%H%M%S"))
        shutil.copy2(dst, os.path.join(bdir, bak))
    with open(dst, "wb") as f:
        f.write(data)
    hub.note("info", "upload", "%s uploaded by %s (%d bytes)%s" % (name, _by(request), len(data),
                                                                  "; the old file is programs/.bak/" + bak if bak else ""))
    return {"saved": name[:-3], "bytes": len(data), "backup": bak}


@app.get("/api/programs/log")
def program_log(since: float = 0.0, kinds: str = ""):
    want = {k.strip() for k in kinds.split(",") if k.strip()}
    rid, recs = runner.records(0)
    lines = []
    for n, t, x in recs:
        if t <= since:
            continue
        kind, ev = runlog.classify(x)
        if want and kind not in want:
            continue
        lines.append(dict(n=n, t=t, kind=kind, ev=ev, log=x))
    return dict(busy=runner.busy(), name=runner.name, run_id=rid, error=runner.error, lines=lines)


@app.get("/api/programs/{name}/source")
def program_source(name: str):
    path = _prog_path(name)
    if not os.path.isfile(path):
        raise HTTPException(404, name)
    return FileResponse(path, media_type="text/plain")


@app.post("/api/programs/{name}/run")
def program_run(name: str, request: Request, prm: dict = Body(default={}), preset: str = "", wait: str = "",
                wait_s: float = 0.0):
    """?wait=button&wait_s=120: the run starts only on the car's START button (bw mat, BRAIN4_SPEC 7.5)."""
    name = os.path.basename(name)
    if not progmeta.name_ok(name):                       # the list hides _name: the API runs only what it lists
        raise HTTPException(404, "no program %s" % name)
    base = {}
    if preset:
        base = progmeta.preset(name, preset)
        if base is None:
            raise HTTPException(409, "no preset %s for %s" % (preset, name))
    runner.start(name, dict(base, **(prm or {})), _by(request), wait=wait, wait_s=wait_s)
    return {"run_id": runner.run_id, "wait": wait or None}


@app.post("/api/programs/stop")
def program_stop():
    runner.stop()
    return {"stopped": True}


@app.get("/api/programs/{name}/presets")
def presets_get(name: str):
    return progmeta.presets(os.path.basename(name))


@app.put("/api/programs/{name}/presets/{preset}")
def preset_put(name: str, preset: str, values: dict = Body(default={})):
    try:
        progmeta.preset_put(os.path.basename(name), preset, values)
    except ValueError as e:
        raise HTTPException(400, str(e))
    return {"saved": preset}


@app.delete("/api/programs/{name}/presets/{preset}")
def preset_delete(name: str, preset: str):
    if not progmeta.preset_delete(os.path.basename(name), preset):
        raise HTTPException(404, "no preset %s" % preset)
    return {"deleted": preset}


# ---------------------------------------------------------------------------------------------------- library
def _library_rows(only: str = "") -> list:
    """One row per programs/*.py: what the file declares (progmeta.meta, without params and docstring: GET
    /api/programs has them), the team's library entry (seed + store) and its newest run.  While a DRIVES program runs
    a run file not parsed before is not parsed now (the same rule as /api/runs/{id}/summary: parsing is pure Python
    in the program's own process) -- its row says parsed False and the console asks again after the run."""
    lib = progmeta.library(PROG_DIR)
    frozen = progmeta.frozen_names(PROG_DIR)
    idx = progmeta.run_index(RUN_DIR)
    busy = runner.busy()
    parse = not (busy and runner.drives)
    rows = []
    for fn in sorted(os.listdir(PROG_DIR)):
        if not fn.endswith(".py") or fn.startswith("_") or (only and fn != only + ".py"):
            continue
        m = _prog_meta(fn)
        if m is None:
            continue
        name, e, ids = m["name"], lib.get(m["name"], {}), idx.get(m["name"], [])
        auto = progmeta.challenge_auto(m, m.get("race"))
        last = None
        if ids:
            rid = ids[-1]
            if busy and runner.run_id == rid:
                last = dict(id=rid, started=runlog.parse_id(rid).get("started"), reason="running", parsed=True,
                            laps=None, seconds=None, corners=None, error=None, ended=False)
            else:
                last = progmeta.last_run(os.path.join(RUN_DIR, rid + ".jsonl"), rid, parse=parse)
                try:                                     # the report verdict reads a JSON file too: not while driving
                    last["report"] = analyze_job.report_state(rid) if parse else None
                except Exception:
                    last["report"] = None
        rows.append(dict(name=name, summary=m.get("summary") or "", bytes=m.get("bytes"), mtime=m.get("mtime"),
                         drives=bool(m.get("drives")), manual_ok=bool(m.get("manual_ok")), race=m.get("race") or [],
                         error=m.get("error"), challenge=e.get("challenge") or auto, challenge_auto=auto,
                         challenge_set=bool(e.get("challenge")), note=e.get("note") or "",
                         rating=int(e.get("rating") or 0), rank=progmeta.rank_of(e), status=e.get("status") or "", tags=list(e.get("tags") or []),
                         updated=e.get("updated"), note_updated=e.get("note_updated"), by=e.get("by"),
                         frozen=name in frozen, runs=len(ids), last_run=last,
                         running=bool(busy and runner.name == name)))
    return rows


@app.get("/api/library")
def library_get():
    """The console's PROGRAMS list: every program with its note / rating / status / tags / challenge and last run."""
    busy = runner.busy()
    return dict(programs=_library_rows(), statuses=list(progmeta.STATUSES), challenges=list(progmeta.CHALLENGES),
                running=runner.name if busy else None, t=round(time.time(), 3))


@app.get("/api/library/{name}")
def library_one(name: str):
    rows = _library_rows(os.path.basename(name))
    if not rows:
        raise HTTPException(404, "no program %s" % name)
    return rows[0]


@app.put("/api/library/{name}")
def library_put(name: str, request: Request, patch: dict = Body(...), if_note: float | None = None):
    """{note?, rating? 0-5, status? best|testing|old|broken|"", tags? [..] or "a, b", challenge? open|obstacle|tool|""}
    merged into the program's entry (atomic write beside params.json); the program file itself is never touched.
    ?if_note=<note_updated the editor loaded, 0 = none>: 409 {entry} when another device saved the note since."""
    name = os.path.basename(name)
    if name not in _prog_names():
        raise HTTPException(404, "no program %s" % name)
    try:
        progmeta.library_put(PROG_DIR, name, patch, _by(request), if_note=if_note)
    except progmeta.Conflict as e:
        raise HTTPException(409, dict(conflict="note", message=str(e), note=e.entry.get("note") or "",
                                      note_updated=e.entry.get("note_updated"), by=e.entry.get("by")))
    except ValueError as e:
        raise HTTPException(400, str(e))
    except OSError as e:
        raise HTTPException(500, "library not saved: %s" % e)
    rows = _library_rows(name)
    if not rows:
        raise HTTPException(404, "no program %s" % name)
    return rows[0]


# ---------------------------------------------------------------------------------------------------- runs
def _run_path(run_id: str) -> str:
    path = os.path.join(RUN_DIR, os.path.basename(run_id) + ".jsonl")
    if not os.path.isfile(path):
        raise HTTPException(404, run_id)
    return path


@app.get("/api/runs")
def runs():
    out = []
    for fn in sorted(os.listdir(RUN_DIR), reverse=True):
        if fn.endswith(".jsonl") and fn != "tests.jsonl":
            st = os.stat(os.path.join(RUN_DIR, fn))
            out.append(dict(id=fn[:-6], bytes=st.st_size, mtime=st.st_mtime, **runlog.parse_id(fn[:-6])))
    out = out[:200]
    for r in out:                                        # the report's verdict (runs_api: <id>.report.json by mtime)
        r["report"] = analyze_job.report_state(r["id"])
    return out


def _parse_allowed(path: str, kind: str):
    """Parsing a run is pure Python in this process: an uncached 5 MB replay stretched a 60 Hz program loop to a
    155 ms period on the laptop (Pi 5 ~3x slower).  So while a DRIVES program runs, only cached results are served."""
    if runner.busy() and runner.drives and not runlog.cached(path, kind):
        raise HTTPException(409, "%s is running: open runs after it ends" % runner.name)


@app.get("/api/runs/{run_id}/summary")
def run_summary(run_id: str):
    path = _run_path(run_id)
    _parse_allowed(path, "summary")
    return runlog.summary(path)


@app.get("/api/runs/{run_id}/replay")
def run_replay(run_id: str):
    path = _run_path(run_id)
    _parse_allowed(path, "replay")
    return runlog.replay(path)


@app.get("/api/runs/{run_id}")
def run_file(run_id: str):
    path = _run_path(run_id)
    return FileResponse(path, media_type="application/x-ndjson", filename=os.path.basename(path))


# ---------------------------------------------------------------------------------------------------- params
def _params_changed(before: dict, by: str, what: str = ""):
    ch = PS.diff(before, robot.p)
    if ch:
        hub.note("info", "params", "%s%s by %s" % (what, PS.change_text(ch), by))
        rs = PS.needs_restart(ch)
        if rs:
            hub.note("warn", "restart", "saved, but the car uses %s only after SYSTEM -> Restart agent" % (
                ", ".join(rs[:4]) + (" (+%d)" % (len(rs) - 4) if len(rs) > 4 else "")))
        hub.cfg_changed()
        warm_plans("params")
    return ch


@app.get("/api/params")
def params_get():
    return robot.p


# the saves that prove the camera pose this session (BRAIN4_SPEC 8.5): a camera fit applied through Review, the
# preflight's pitch from the pillar, TESTS -> cam_pitch -> Apply, and a still field_check that fitted the standard map
CAMERA_PROOF = ("calib:fit", "preflight:pitch", "preflight:field", "test:cam_pitch")


def _camera_stamp(patch: dict, reason: str) -> dict:
    """A proof save that names the camera also stamps camera.verified_t with the ROBOT's clock (the laptop's may be
    off: the Pi has no RTC).  Any other save passes camera.verified_t through as sent."""
    if reason not in CAMERA_PROOF or not any(k == "camera" or k.startswith("camera.") for k in (patch or {})):
        return patch
    return dict(patch, **{"camera.verified_t": round(time.time(), 3)})


@app.put("/api/params")
def params_put(request: Request, patch: dict = Body(...), reason: str = "save"):
    """Merge a partial dict, or apply dotted keys {"steer.center_us": 1512}; history backup first, then saved
    atomically and applied at once -- IN PLACE, so a running loop that holds a sub-dict sees the change."""
    patch = _camera_stamp(patch, reason)
    before = copy.deepcopy(robot.p)
    trial = copy.deepcopy(robot.p)
    PS.patch(trial, patch)
    changed = {c["key"] for c in PS.diff(before, trial)}
    bad = [why for key, why in PS.validate(trial) if key in changed]
    if bad:
        raise HTTPException(400, "; ".join(bad))
    if changed:
        PS.backup(robot.p, reason or "save")
        PS.patch(robot.p, patch)
        P.save(robot.p)
        _params_changed(before, _by(request))
    return robot.p


@app.get("/api/params/history")
def params_history():
    return PS.history()


@app.get("/api/params/history/{hid}")
def params_history_entry(hid: str):
    e = PS.entry(hid)
    if e is None:
        raise HTTPException(404, hid)
    return e


@app.post("/api/params/restore/{hid}")
def params_restore(hid: str, request: Request):
    before = copy.deepcopy(robot.p)
    try:
        PS.restore(robot.p, hid)
    except KeyError:
        raise HTTPException(404, hid)
    _params_changed(before, _by(request), "restore %s: " % hid)
    return robot.p


@app.post("/api/params/undo")
def params_undo(request: Request):
    before = copy.deepcopy(robot.p)
    try:
        _p, hid = PS.undo(robot.p)
    except KeyError:
        raise HTTPException(404, "no params history yet")
    _params_changed(before, _by(request), "undo: ")
    return JSONResponse(robot.p, headers={"X-BW-Restored": hid})


@app.get("/api/params/meta")
def params_meta():
    return PS.meta()


@app.get("/api/profiles")
def profiles():
    return PS.profiles()


@app.put("/api/profiles/{name}")
def profile_put(name: str, body: dict = Body(...)):
    if not progmeta.PRESET_NAME.match(name):
        raise HTTPException(400, "profile names are 1-32 of A-Z a-z 0-9 _ . -")
    try:
        PS.profile_put(name, str(body.get("desc", "")), dict(body.get("patch") or {}))
    except PermissionError:
        raise HTTPException(409, "%s is built in (read-only)" % name)
    return {"saved": name}


@app.post("/api/profiles/{name}/apply")
def profile_apply(name: str, request: Request):
    before = copy.deepcopy(robot.p)
    try:
        PS.profile_apply(robot.p, name)
    except KeyError:
        raise HTTPException(404, "no profile %s" % name)
    _params_changed(before, _by(request), "profile %s: " % name)
    bodyid.clear_cache()                                 # the next check reads the new profile's expectation at once
    return robot.p


# ---------------------------------------------------------------------------------------------------- body (8.3-8.6)
@app.get("/api/body")
def body_get(fresh: int = 0):
    """bodyid.check: {ok, applied, expect, detected, by, why, evidence, shell, t} (ok None = unknown)."""
    return bodyid.check(robot, force=bool(fresh))


def _fingerprint_now() -> dict:
    sc = bodyid.fresh_scan(robot)
    if sc is None:
        raise HTTPException(409, "no fresh lidar scan: is the lidar spinning?  (bw status: lidar rpm)")
    fp = bodyid.fingerprint(sc)
    b = robot.p.get("body") or {}
    applied = str(b.get("profile") or "")
    ref = bodyid.load_store().get(applied) if applied else None
    return dict(fp=fp, profile=applied, informative=bodyid.informative(fp), clear_ring_bins=bodyid.clear_ring(sc),
                ref_score=bodyid.fp_match(fp, ref["fp"]) if ref and isinstance(ref.get("fp"), list) else None,
                ref_t=ref.get("t") if ref else None)


@app.get("/api/body/fingerprint")
def body_fingerprint_get():
    """The lidar's near-return fingerprint now and its score against the applied profile's reference (no save)."""
    return _fingerprint_now()


@app.post("/api/body/fingerprint")
def body_fingerprint_post(request: Request, body: dict = Body(default={})):
    """Record the fingerprint as the applied profile's reference: {confirm: "fingerprint"}, the car still, nothing
    within 30 cm, no program driving (it must be the body alone)."""
    if body.get("confirm") != "fingerprint":
        raise HTTPException(400, "confirm must be \"fingerprint\" (the car still, nothing within 30 cm)")
    if runner.busy() and runner.drives:
        raise HTTPException(409, "%s is running: stop it first" % runner.name)
    if abs(robot.v_cmd) > 0.0:
        raise HTTPException(409, "the car is commanded to move: stop it first")
    b = robot.p.get("body") or {}
    applied = str(b.get("profile") or "")
    if not applied:
        raise HTTPException(409, "no profile stamp: bw body NAME first, then record its fingerprint")
    now = _fingerprint_now()
    if now["clear_ring_bins"] > bodyid.CLEAR_BINS:
        raise HTTPException(409, "something is 20-30 cm from the lidar (%d bins): clear the car's surroundings"
                            % now["clear_ring_bins"])
    exp, _eb = bodyid.expected(robot.p)
    e = bodyid.save_fingerprint(applied, now["fp"], exp, str(b.get("shell") or ""))
    hub.note("info", "body", "body fingerprint of %s recorded by %s%s" % (
        applied, _by(request), "" if now["informative"] else " (it shows none of the car's own parts: the strap decides)"))
    hub.cfg_changed()
    return dict(now, saved=True, entry=e)


# ---------------------------------------------------------------------------------------------------- websocket
@app.websocket("/ws")
async def ws(sock: WebSocket):
    if TOKEN and sock.query_params.get("token") != TOKEN:
        await sock.close(code=4401)
        return
    await hub.serve(sock)


@app.on_event("shutdown")
def _shutdown():
    runner.stop()
    mapper.close()
    sampler.close()
    robot.close()
