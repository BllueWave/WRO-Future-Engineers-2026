"""The console's WebSocket hub: /ws protocol v2 (docs/CONSOLE_SPEC.md section 5), the drive lock and the link watchdog.

    producer   ONE coroutine, a deadline-scheduled 20 Hz tick (asyncio.sleep(0.05) alone gave 16.3 msg/s on Windows:
               the sleep rounds up to the 15.6 ms timer).  Each tick builds the messages that are due, serialises each
               ONCE, and keeps only the latest per kind -- a slow phone never gets a backlog of stale telemetry
    sender     one coroutine per client: the queued events (pong, refused, note, log, cfg), then the newest of every
               stream it subscribes to
    reader     one coroutine per client: hello / sub / drive / stop / estop / ping

Blocking work stays off the event loop: the camera overlay data (`vis`) is computed on its own thread at <= 5 Hz and
only while a client subscribes; system info has its sampler thread (sysinfo.py).

Safety (section 10): one driver at a time (the lock, released on stop, 1.0 s of silence or disconnect); a disconnect
stops the car ONLY when that client held the lock -- any other disconnect never touches a running program; the link
watchdog stops a DRIVES program when no WS message and no /api request arrived for safety.link_timeout_ms.
"""
from __future__ import annotations

import asyncio
import base64
import hashlib
import json
import math
import platform
import threading
import time
import traceback
from collections import deque

import numpy as np

from . import field as F
from . import runlog

DEFAULT_SUBS = dict(tel=True, scan=1, live=True, vis=False, sys=True, log=True,
                    perc=False, map=0, cloud=0)      # BRAIN_SPEC 8.4 (v3): off by default -- a v2 console pays nothing
CLOUD_PERIOD_S = 0.5          # <= 2 cloud chunks per second per client (<= 192 KB/s while a cloud fills)
LOCK_SILENCE_S = 1.0
HOUSE_STALE_S = 0.25          # the dead-man thread's heartbeat older than this: no drive command is accepted.  Not
                              # 0.1 s: one GIL stall of 155 ms was measured on the laptop while a run file parsed
MAX_CLIENTS = 8               # consoles at once: each costs ~1 % of a core (5 measured at 5 %), and E-STOP has REST


def dumps(obj) -> str:
    """Compact JSON; a NaN / inf anywhere becomes null (the browser's JSON.parse rejects NaN)."""
    try:
        return json.dumps(obj, separators=(",", ":"), allow_nan=False)
    except (ValueError, TypeError):
        return json.dumps(_clean(obj), separators=(",", ":"), allow_nan=False, default=str)


def _clean(x):
    if isinstance(x, dict):
        return {str(k): _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    if isinstance(x, np.generic):
        x = x.item()
    if isinstance(x, np.ndarray):
        return _clean(x.tolist())
    if isinstance(x, float) and not math.isfinite(x):
        return None
    return x


def _num(x, nd=None):
    """float/int or None for missing / non-finite."""
    if x is None:
        return None
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v):
        return None
    return int(round(v)) if nd == 0 else (round(v, nd) if nd is not None else v)


def params_digest(p: dict) -> str:
    return hashlib.sha1(json.dumps(p, sort_keys=True, default=str).encode()).hexdigest()[:6]


def scan_body(sc, res_want: float = 1.0, seq: int = 0) -> dict:
    """The `scan` message: n little-endian uint16 mm (0 = no return) and n uint8 confidences, base64.  Doubling the
    resolution keeps each pair's SMALLEST non-zero distance (the conservative choice for obstacles) with its
    confidence.  Bin k is at -180 + (k + 0.5) * res degrees, CCW +, 0 = the car's nose."""
    d = np.asarray(sc.dist).astype(np.int64)
    q = np.asarray(sc.conf).astype(np.int64)
    res = float(sc.res)
    k = max(1, int(round(float(res_want) / res)))
    if k > 1 and len(d) % k == 0:
        dd, qq = d.reshape(-1, k), q.reshape(-1, k)
        big = np.where(dd > 0, dd, 1 << 30)
        i = big.argmin(axis=1)
        r = np.arange(len(big))
        dmin = big[r, i]
        d = np.where(dmin >= 1 << 30, 0, dmin)
        q = np.where(d > 0, qq[r, i], 0)
        res *= k
    d16 = np.clip(d, 0, 65535).astype("<u2")
    return dict(type="scan", t=round(time.time(), 3), seq=seq, age_ms=int((time.monotonic() - sc.t) * 1000),
                res=res, n=int(len(d16)), rpm=_num(sc.rpm, 0), valid=int(np.count_nonzero(np.asarray(sc.dist) > 0)),
                d=base64.b64encode(d16.tobytes()).decode("ascii"),
                q=base64.b64encode(np.clip(q, 0, 255).astype(np.uint8).tobytes()).decode("ascii"))


def vis_payload(bgr, t_mono: float, p: dict) -> dict:
    """The camera overlay data (CONSOLE_SPEC 5.4 `vis`): per sampled column the base row and its class, the
    horizon, the program's pillar detections with range and bearing, and the HSV blobs the old overlay drew."""
    from . import camera as C
    from . import vision as V
    H, W = bgr.shape[:2]
    g = V.Ground(p["camera"], W, H)
    vp = p.get("vision")
    e = V.edges(bgr, g, vp)
    cols = []
    for u, v, c, clip in zip(e.u, e.v, e.cls, e.clip):
        cols += [int(u), int(v) if v == v else -1, int(c), int(bool(clip))]
    pil = []
    for q in V.pillars(bgr, g, vp):
        X, Y = float(q["X"]), float(q["Y"])
        dx, dy = X - float(g.pos[0]), Y - float(g.pos[1])
        pil.append(dict(c=q["colour"], x=int(round(q["u"] - q["w_px"] / 2.0)), y=int(round(q["v_base"] - q["h_px"])),
                        w=int(q["w_px"]), h=int(q["h_px"]), X=_num(X, 0), Y=_num(Y, 0), r=_num(math.hypot(dx, dy), 0),
                        brg=_num(math.degrees(math.atan2(dy, dx)), 1), est=q.get("est", "base")))
    blobs = []
    ms = C.masks(bgr, p.get("hsv", {}))
    for c in ("red", "green", "magenta", "orange", "blue"):
        if c in ms:
            for b in C.blobs(ms[c])[:4]:
                blobs.append(dict(c=c, x=b["x"], y=b["y"], w=b["w"], h=b["h"], a=b["area"]))
    return dict(type="vis", t=round(time.time(), 3), frame_t=round(time.time() - (time.monotonic() - t_mono), 3),
                W=int(W), H=int(H), horizon=round(float(g.horizon_row()), 1), cols=cols, pillars=pil, blobs=blobs)


class Client:
    def __init__(self, sock, ip: str, key: str):
        self.sock, self.ip, self.key = sock, ip, key
        self.label = "console/? %s" % ip
        self.subs = dict(DEFAULT_SUBS)
        self.sent = {}
        self.q = deque(maxlen=500)
        self.wake = asyncio.Event()
        self.live_epoch, self.live_path_v, self.need_trail = None, None, True
        self.refused_t = 0.0
        self.cloud_state, self.cloud_t = None, 0.0     # (epoch, ver) of the last cloud chunk sent, and when


class Hub:
    TICK = 0.05

    def __init__(self, robot, runner, sampler, on_estop=None):
        self.robot, self.runner, self.sampler = robot, runner, sampler
        self.on_estop = on_estop
        self.clients: set = set()
        self.loop = None
        self._task = None
        self._n = 0
        self._lock = threading.Lock()                    # the drive lock's state (WS on the loop, REST on threads)
        self.driver_key = self.driver_label = None
        self.driver_t = 0.0
        self.testing = None                              # name of a MOVES test in progress (the agent sets it)
        self.test_by = None                              # who started it: its console's disconnect stops it
        self.test_stop = threading.Event()               # the running test's stop (E-STOP, Stop, disconnect, watchdog)
        self.last_touch, self.touch_label = time.monotonic(), None
        self._wd_run = None                              # run id the watchdog already stopped
        self._pending = deque(maxlen=50)                 # notes raised before the loop exists
        self.agent_started = time.time()
        self.seq = 0
        self.latest = {}                                 # kind -> (seq, text)
        self.live = None                                 # (seq, dict, text, path, trail)
        self._live_n = 0
        self._scan_t = None
        self._scan_seq = 0
        self._log_rid, self._log_n = None, 0
        self._cfg_digest = None
        self._cfg_dirty = False
        self.vis_wanted = False
        self._vis = (0, None)
        self._vis_th = None
        self._err_t = 0.0
        self.notes = deque(maxlen=50)                    # the last notes, for GET /api/notes and new clients' toasts
        self.mapper = None                               # bluewave/mapper.Mapper (the agent sets it): perc, map, cloud
        self.restart_fn = None                           # () -> saved keys the running Robot does not use yet (agent)
        self.body_fn = None                              # () -> bodyid.check(robot) (agent): cfg.body, the `body` note
        self._body_seen = None                           # (ok, detected, applied) of the last body check
        self._msh = None                                 # the manual shield (key, Shield, Odo, state): _manual_filter
        self.manual_shield = None                        # its last verdict, for tel.shield_manual

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        """Start the producer on the running loop (idempotent; restarts after the loop it ran on is gone)."""
        loop = asyncio.get_running_loop()
        if self._task is not None and not self._task.done() and self.loop is loop:
            return
        self.loop = loop
        self._task = loop.create_task(self._producer())
        while self._pending:
            self._broadcast(self._pending.popleft())
        if self._vis_th is None or not self._vis_th.is_alive():
            self._vis_th = threading.Thread(target=self._vis_loop, name="vis", daemon=True)
            self._vis_th.start()

    def touch(self, label=None) -> None:
        self.last_touch = time.monotonic()
        if label:
            self.touch_label = label

    def note(self, level: str, what: str, text: str) -> None:
        """Broadcast a toast to every console; thread-safe."""
        msg = dict(type="note", t=round(time.time(), 3), level=level, what=what, text=text)
        self.notes.append(msg)
        txt = dumps(msg)
        loop = self.loop
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if loop is not None and running is loop:
            self._broadcast(txt)
        elif loop is not None and not loop.is_closed():
            try:
                loop.call_soon_threadsafe(self._broadcast, txt)
            except RuntimeError:
                self._pending.append(txt)
        else:
            self._pending.append(txt)

    def cfg_changed(self) -> None:
        self._cfg_dirty = True

    def _broadcast(self, txt: str, only=None) -> None:
        for c in list(self.clients):
            if only is None or c.subs.get(only):
                c.q.append(txt)
                c.wake.set()

    # ------------------------------------------------------------------ drive rules (5.6)
    def drive(self, key: str, label: str, v, steer, ttl_ms=None):
        """(accepted, reason).  E-STOP, a program that owns the car, a moving test, another driver: refused."""
        r, run = self.robot, self.runner
        try:
            v, steer = float(v), float(steer)
        except (TypeError, ValueError):
            return False, "v and steer must be numbers"
        if not (math.isfinite(v) and math.isfinite(steer)):
            return False, "v and steer must be finite"
        with self._lock:
            if r.estopped:
                return False, "E-STOP latched"
            if run.busy() and not run.manual_ok:
                return False, "a program owns the car"
            if self.testing:
                return False, "a test owns the car: %s" % self.testing
            now = time.monotonic()
            if now - getattr(r, "house_t", now) > HOUSE_STALE_S:
                return False, "the dead-man thread is not running: restart the agent"
            if self.driver_key and self.driver_key != key and now - self.driver_t < LOCK_SILENCE_S:
                return False, "another console is driving: %s" % self.driver_label
            self.driver_key, self.driver_label, self.driver_t = key, label, now
            try:
                dm = int(max(100, min(1000, float(r.p.get("safety", {}).get("deadman_ms", 400)))))
            except (TypeError, ValueError):
                dm = 400
            ttl = min(int(ttl_ms), dm) if ttl_ms else dm           # REST may shorten the dead-man, never lengthen it
            try:
                v, steer = self._manual_filter(v, steer, now)
            except Exception:                                     # a shield bug must never take the stop away
                if time.monotonic() - self._err_t > 5.0:
                    self._err_t = time.monotonic()
                    traceback.print_exc()
                v = 0.0
            ok = r.drive(v, steer, ttl_ms=max(50, ttl))
        return ok, None if ok else "E-STOP latched"

    def _manual_filter(self, v: float, steer: float, now: float):
        """BRAIN_SPEC 8.7, params shield.manual (-1 = on iff lidar.loc = 1, the WLtoys build): the joystick passes the
        lidar shield in "manual" mode -- it slows or brakes, it never steers for the driver.  Without it the Map
        page's drive held 0.25 m/s against a pillar while its own perception read free 9 mm (console review
        2026-09-24: 7 contacts in 25 s).  Its own odometry (the gyro, v_odo) carries the scan memory over the body's
        blind rear.  Under self._lock."""
        r = self.robot
        p = r.p
        sh_p = p.get("shield") or {}
        try:
            m = int(sh_p.get("manual", -1))
        except (TypeError, ValueError):
            m = -1
        on = bool(int((p.get("lidar") or {}).get("loc", 0))) if m < 0 else bool(m)
        if not on or v == 0.0:
            return v, steer
        from . import lidar_perc as LP
        from . import loc as LOC
        from . import shield as SH
        key = json.dumps([sh_p, (p.get("lidar") or {}).get("loc"), p.get("car"), p.get("chassis"), p.get("steer")],
                         sort_keys=True, default=str)
        if self._msh is None or self._msh[0] != key:
            self._msh = (key, SH.Shield(p, {"on": 1}), LOC.Odo(), dict(t=now, yaw=r.yaw, scan_t=None))
        _k, sh, odo, st = self._msh
        dt = min(max(now - st["t"], 0.0), 0.1)
        dth = math.radians(((r.yaw - st["yaw"] + 180.0) % 360.0) - 180.0)
        ds = float(r.v_odo()) * 1000.0 * dt
        st.update(t=now, yaw=r.yaw)
        sh.odom(ds, dth)
        odo.add(now, ds, dth)
        sc = r.scan
        if sc is not None and sc.t != st["scan_t"]:
            st["scan_t"] = sc.t
            sh.scan(LP.of(r), odo.since)
        v2, st2 = sh.filter(v, steer, now, "manual")
        self.manual_shield = dict(t=now, act=sh.act, asked=round(v, 2), v=round(v2, 2),
                                  free=None if sh.free is None else int(round(min(sh.free, 9999.0))),
                                  need=None if sh.need is None else int(round(sh.need)))
        return v2, st2

    def manual_stop(self, key: str) -> None:
        """A client's `stop`: stops the car if it holds the lock (or nobody does) and no program owns the car."""
        with self._lock:
            if self.driver_key is None or self.driver_key == key:
                self.driver_key = self.driver_label = None
                if not (self.runner.busy() and not self.runner.manual_ok) and not self.testing:
                    self.robot.stop()

    def _release_on_disconnect(self, key: str) -> None:
        with self._lock:
            if self.driver_key == key:
                self.driver_key = self.driver_label = None
                if not (self.runner.busy() and not self.runner.manual_ok):
                    self.robot.stop()                    # losing the page never leaves the car driving

    def driver(self):
        with self._lock:
            return self.driver_label

    # ------------------------------------------------------------------ messages
    def cfg(self, you: str | None = None) -> dict:
        p = self.robot.p
        cam = p.get("camera", {})
        W, H = int(cam.get("width", 640)), int(cam.get("height", 480))
        try:
            fx = float(cam["intrinsics"][0]) * W / float(cam.get("calib_size", [640, 480])[0])
            hfov = round(math.degrees(2 * math.atan(W / 2.0 / fx)), 1)
        except (KeyError, IndexError, TypeError, ValueError, ZeroDivisionError):
            hfov = None
        car = dict(dict(rear_mm=34.0, front_mm=179.0, half_w_mm=80.5, lot_len_mm=320.0), **p.get("car", {}))
        li = p.get("lidar", {})
        st = p.get("steer", {})
        sf = p.get("safety", {})
        ch, sh, dp, im = p.get("chassis", {}), p.get("shield", {}), p.get("depth", {}), p.get("imu", {})
        wb = float(ch.get("wheelbase_m", 0.145))
        try:
            r_min = round(wb * 1000.0 / math.tan(math.radians(float(st.get("max_deg", 29.0)))), 1)
        except (TypeError, ValueError, ZeroDivisionError):
            r_min = None
        sh_on = int(sh.get("on", -1))
        pos = [float(v) for v in (li.get("pos_mm") or [0.0, 0.0])[:2]]
        h_li = li.get("h_mm", 145.7)
        # proto stays 2: v3 (BRAIN_SPEC 8.4) only ADDS blocks and subscriptions, a v2 console reads this unchanged
        d = dict(type="cfg", t=round(time.time(), 3), proto=2, host=platform.node(), mock=bool(self.robot.mock),
                 card=getattr(self.sampler, "card", None), mode=getattr(self.sampler, "mode", None),
                 agent_started=round(self.agent_started, 3), digest=params_digest(p),
                 car={k: car[k] for k in ("rear_mm", "front_mm", "half_w_mm", "lot_len_mm")},
                 lidar=dict(pos_mm=pos, block_deg=li.get("block_deg", []),
                            res_deg=li.get("res_deg", 1.0), loc=li.get("loc", 0), h_mm=h_li, cw=li.get("cw"),
                            offset_deg=li.get("offset_deg"), scan_lag_s=li.get("scan_lag_s", 0.05)),
                 # BRAIN_SPEC 8.4 (v3): the effective and the geometric wheelbase, the shield, the maps, the TF tree
                 chassis=dict(wheelbase_mm=round(wb * 1000.0, 1),
                              wheelbase_geom_mm=round(float(ch.get("wheelbase_geom_m", wb)) * 1000.0, 1),
                              track_mm=round(float(ch.get("track_m", 0.133)) * 1000.0, 1),
                              wheel_d_mm=round(float(p.get("drive", {}).get("wheel_d_m", 0.067)) * 1000.0, 1),
                              r_min_mm=r_min),
                 shield=dict(on=bool(li.get("loc", 0)) if sh_on < 0 else bool(sh_on),
                             margin_mm=sh.get("margin_mm", 40.0), side_mm=sh.get("side_mm", 15.0),
                             park_margin_mm=sh.get("park_margin_mm", 8.0), horizon_mm=sh.get("horizon_mm", 900.0)),
                 map=dict(res_mm=p.get("map", {}).get("res_mm", 20.0), n=p.get("map", {}).get("size", 512)),
                 depth=dict(on=bool(int(dp.get("on", 0))), registered=int(dp.get("registered", 1))),
                 sensors=[
                     dict(name="base_link", parent="field", xyz=[0, 0, 0], rpy=[0, 0, 0],
                          note="rear-axle centre on the floor"),
                     dict(name="lidar", parent="base_link", xyz=[pos[0], pos[1], h_li], rpy=[0, 0, 0], model="LD19",
                          raw_offset_deg=li.get("offset_deg"), cw=li.get("cw")),
                     dict(name="camera", parent="base_link", xyz=[cam.get("x_mm", 150.0), cam.get("y_mm", 0.0),
                                                                 cam.get("h_mm")],
                          rpy=[cam.get("roll_deg", 0.0), cam.get("pitch_deg"), 0.0], hfov_deg=hfov,
                          latency_ms=round(float(cam.get("latency_s", 0.12)) * 1000.0)),
                     dict(name="depth", parent="camera", xyz=[0, 0, 0], rpy=[0, 0, 0],
                          registered=int(dp.get("registered", 1)), on=bool(int(dp.get("on", 0)))),
                     dict(name="imu", parent="base_link", xyz=None, rpy=None, axis=im.get("gyro_axis"),
                          sign=im.get("gyro_sign"), bias_dps=im.get("gyro_bias"))],
                 camera=dict(width=W, height=H, fps=cam.get("fps"), hfov_deg=hfov, x_mm=cam.get("x_mm", 150.0),
                             y_mm=cam.get("y_mm", 0.0), h_mm=cam.get("h_mm"), pitch_deg=cam.get("pitch_deg"),
                             latency_s=cam.get("latency_s", 0.12)),
                 steer=dict(max_deg=st.get("max_deg"), center_us=st.get("center_us"), us_per_deg=st.get("us_per_deg"),
                            invert=st.get("invert")),
                 drive=dict(max_mps=p.get("drive", {}).get("max_mps"), backend=p.get("drive", {}).get("backend", "rrc")),
                 safety=dict(deadman_ms=sf.get("deadman_ms", 400), link_timeout_ms=sf.get("link_timeout_ms", 3000)),
                 field=dict(half=F.HALF, isl=F.ISL, wall_h=F.WALL_H, pillar=F.PILLAR, lot_depth=F.LOT_DEPTH,
                            limit=list(F.LIMIT), seats=[[int(s[0]), int(s[1]), int(s[2]), int(round(s[3])),
                                                         int(round(s[4]))] for s in F.SEATS]))
        try:
            d["restart"] = list(self.restart_fn()) if self.restart_fn else []
        except Exception:
            d["restart"] = []
        if self.body_fn is not None:                     # BRAIN4_SPEC 8.6: the console's BODY chip
            try:
                from . import bodyid
                d["body"] = bodyid.cfg_block(self.body_fn())
            except Exception as e:
                d["body"] = dict(profile="", expect="", shell="", detected=None, ok=None, by=None,
                                 why="body check failed: %s" % e)
        if you:
            d["you"] = you
        return d

    def _body_tick(self) -> None:
        """Every 5 s with a console connected: a changed body verdict notes `body` (bad = mismatch, warn = unknown,
        ok = consistent) and re-sends cfg.  The first verdict after the start notes only a mismatch."""
        if self.body_fn is None:
            return
        chk = self.body_fn()
        seen = (chk.get("ok"), chk.get("detected"), chk.get("applied"))
        first = self._body_seen is None
        if seen == self._body_seen:
            return
        self._body_seen = seen
        self._cfg_dirty = True
        if first and chk.get("ok") is not False:
            return
        level = "ok" if chk.get("ok") else "bad" if chk.get("ok") is False else "warn"
        self.note(level, "body", str(chk.get("why") or "body check changed"))

    def tel(self) -> dict:
        r, run = self.robot, self.runner
        s = r.snapshot()
        now = time.monotonic()
        im = r.p.get("imu", {})
        gz = acc = tilt = None
        try:
            imu = r.imu_raw()
        except Exception:
            imu = None
        if imu:
            try:
                gz = round(float(imu[int(im.get("gyro_axis", 5))]) * float(im.get("gyro_sign", 1.0))
                           - float(im.get("gyro_bias", 0.0)), 2)
                ax, ay, az = (float(a) for a in imu[:3])
                if math.sqrt(ax * ax + ay * ay + az * az) > 4.0:        # m/s^2, not g
                    ax, ay, az = ax / 9.80665, ay / 9.80665, az / 9.80665
                acc = [round(ax, 3), round(ay, 3), round(az, 3)]
                tilt = [round(math.degrees(math.atan2(ax, math.hypot(ay, az))), 1),
                        round(math.degrees(math.atan2(ay, az)), 1)]
            except (TypeError, ValueError, IndexError):
                pass
        sc = r.scan
        _f, tf = r.frame()
        busy = run.busy()
        drives = bool(run.drives) if busy else True
        manual_ok = bool(run.manual_ok) if busy else False
        estop = bool(r.estopped)
        self.seq += 1
        out: dict = dict(type="tel", t=round(time.time(), 3), seq=self.seq, mock=bool(r.mock), estop=estop,
                   v=_num(s.get("v"), 2), steer=_num(s.get("steer"), 1), yaw=_num(s.get("yaw"), 1), gz=gz, acc=acc,
                   tilt=tilt, imu_hz=_num(s.get("imu_hz"), 1), imu_age_ms=_num(s.get("imu_age_ms"), 0),
                   battery_v=_num(s.get("battery_v"), 2),
                   lidar_rpm=_num(sc.rpm, 0) if sc is not None else None,
                   lidar_age_ms=int((now - sc.t) * 1000) if sc is not None else None,
                   lidar_valid=int(np.count_nonzero(np.asarray(sc.dist) > 0)) if sc is not None else None,
                   cam_fps=_num(s.get("cam_fps"), 1), cam_age_ms=int((now - tf) * 1000) if tf else None,
                   front_mm=_num(s.get("front_mm"), 0), left_mm=_num(s.get("left_mm"), 0),
                   right_mm=_num(s.get("right_mm"), 0), back_mm=_num(s.get("back_mm"), 0),
                   busy=busy, prog=run.name if busy else None, run_id=run.run_id if busy else None,
                   elapsed_s=round(time.time() - run.started, 1) if busy else None, drives=drives, manual_ok=manual_ok,
                   drive_ok=(not estop) and (not busy or manual_ok) and not self.testing, driver=self.driver(),
                   testing=self.testing,
                   house_ms=int((now - r.house_t) * 1000) if isinstance(getattr(r, "house_t", None), float) else None)
        if busy and getattr(run, "waiting", False):     # wait=button (BRAIN4_SPEC 7.5): the strip's countdown
            out["waiting"] = True
            out["wait_left_s"] = round(max(0.0, float(getattr(run, "wait_until", 0.0) or 0.0) - time.time()), 1)
        ms = self.manual_shield                          # the joystick's shield acted in the last 0.5 s (BRAIN_SPEC 8.7)
        if ms and now - ms["t"] < 0.5 and ms["act"] not in ("ok", "idle", "off"):
            out["shield_manual"] = {k: ms[k] for k in ("act", "asked", "v", "free", "need")}
        sim = getattr(r, "sim", None)
        if sim is not None:
            w = sim.world
            out["sim"] = dict(x=int(round(w.x)), y=int(round(w.y)), th=round(math.degrees(w.th), 1),
                              contacts=int(w.contacts), touching=str(w.touching or ""))
        dv = s.get("drive")                              # G6: the encoderless drive (gpio_pwm only), BRAIN_SPEC 8.4
        if isinstance(dv, dict):
            out["drive"] = dict(backend=dv.get("backend"), duty=_num(dv.get("duty"), 4), v_est=_num(dv.get("v_est"), 3),
                                v_lidar=_num(dv.get("v_lidar"), 3), fresh=bool(dv.get("lidar_fresh")),
                                stall=bool(dv.get("stall")), stalls=int(getattr(r, "stalls", 0) or 0),
                                error=dv.get("error"))
        dd = s.get("depth")                              # the HP60C stream (depth.on = 1 only)
        if isinstance(dd, dict):
            age = dd.get("age_ms")
            if age is None:                              # the stream object has no age: its newest frame's exposure
                try:
                    _dimg, td = r.depth()
                    age = (now - td) * 1000.0 if td else None
                except Exception:
                    age = None
            out["depth"] = dict(fps=_num(dd.get("fps"), 1), frames=dd.get("frames"), ok=bool(dd.get("ok")),
                                age_ms=_num(age, 0), error=dd.get("error"))
        return out

    def log_items(self, recs) -> list:
        out = []
        for n, t, x in recs:
            kind, ev = runlog.classify(x)
            out.append(dict(n=n, t=round(t, 3), kind=kind, ev=ev, rec=x))
        return out

    # ------------------------------------------------------------------ producer
    async def _producer(self):
        nxt = time.monotonic()
        tick = 0
        while True:
            tick += 1
            try:
                self._tick(tick)
            except Exception:
                if time.monotonic() - self._err_t > 5.0:           # one traceback per 5 s, never a dead hub
                    self._err_t = time.monotonic()
                    traceback.print_exc()
            nxt += self.TICK
            now = time.monotonic()
            if now - nxt > 0.2:
                nxt = now                                          # fell behind: re-anchor, do not burst
            await asyncio.sleep(max(0.0, nxt - now))

    def _tick(self, tick: int):
        r, run = self.robot, self.runner
        now_m = time.monotonic()
        cl = list(self.clients)
        # drive lock: released after 1.0 s of silence.  The dead-man stopped the car at deadman_ms already -- unless
        # its thread died, so the release stops it too (a released lock left no holder whose disconnect would)
        with self._lock:
            if self.driver_key and now_m - self.driver_t > LOCK_SILENCE_S:
                self.driver_key = self.driver_label = None
                if not (run.busy() and not run.manual_ok) and not self.testing:
                    r.stop()
        self._watchdog(now_m)
        if cl and tick % 100 == 50:
            try:
                self._body_tick()
            except Exception:
                pass
        if self.sampler is not None and hasattr(self.sampler, "want") and any(c.subs["sys"] for c in cl):
            self.sampler.want()
        # cfg: on any params change, whoever made it
        if self._cfg_dirty or tick % 20 == 1:
            dg = params_digest(r.p)
            if self._cfg_dirty or dg != self._cfg_digest:
                first = self._cfg_digest is None
                self._cfg_dirty, self._cfg_digest = False, dg
                if not first:
                    for c in cl:
                        c.q.append(dumps(self.cfg(c.label)))
        if any(c.subs["tel"] for c in cl):
            t = self.tel()
            self.latest["tel"] = (t["seq"], dumps(t))
        # scan, per resolution, when the revolution changed
        sc = r.scan
        want = {c.subs["scan"] for c in cl if c.subs["scan"]}
        if sc is not None and want and sc.t != self._scan_t:
            self._scan_t = sc.t
            self._scan_seq += 1
            for res in want:
                self.latest["scan%d" % res] = (self._scan_seq, dumps(scan_body(sc, float(res), self._scan_seq)))
        # live: 10 Hz while a program runs, else 1 Hz (a heartbeat, or the ended program's last state); points in
        # every 2nd message
        bus = getattr(r, "live", None)
        if bus is not None and any(c.subs["live"] for c in cl) and tick % (2 if bus.src and bus.running else 20) == 0:
            self._live_n += 1
            need_trail = any(c.subs["live"] and (c.need_trail or c.live_epoch != bus.epoch) for c in cl)
            d = bus.read(scan=sc, lidar_cfg=r.p.get("lidar", {}), pts=self._live_n % 2 == 1, trail=need_trail)
            d = dict(type="live", t=round(time.time(), 3), **d)
            path, trail = d.pop("path", None), d.pop("trail", None)
            self.live = (self._live_n, d, dumps(d), path, trail)
        if tick % 20 == 0 and any(c.subs["sys"] for c in cl) and self.sampler is not None:
            self.latest["sys"] = (tick, dumps(dict(type="sys", t=round(time.time(), 3), **self.sampler.snapshot())))
        self.vis_wanted = any(c.subs["vis"] for c in cl)
        if self._vis[1] is not None:
            self.latest["vis"] = self._vis
        mp = self.mapper                                 # perc / map / cloud: built on the mapper's thread
        if mp is not None:
            lv = 0
            for c in cl:
                lv |= int(c.subs.get("map") or 0) & 3
            mp.want(any(c.subs.get("perc") for c in cl), lv, any(c.subs.get("cloud") for c in cl))
            for k in ("perc", "map1", "map2"):
                if k in mp.latest:
                    self.latest[k] = mp.latest[k]
                else:
                    self.latest.pop(k, None)             # cleared (a new program, Clear): no stale map to a new client
        if tick % 2 == 0:
            self._log_tick(cl)
        for c in cl:
            c.wake.set()

    def _log_tick(self, cl):
        rid = self.runner.run_id
        since = self._log_n if rid == self._log_rid else 0
        rid2, recs = self.runner.records(since)
        if rid2 != rid:
            return
        if rid != self._log_rid:                         # a new run counts n from 1 again: keeping the old run's
            self._log_rid, self._log_n = rid, 0          # last n skipped every record up to it (hello's whole log)
        if recs:
            self._log_n = recs[-1][0]
            txt = dumps(dict(type="log", run_id=rid, items=self.log_items(recs)))
            for c in cl:
                if c.subs["log"]:
                    c.q.append(txt)

    def _watchdog(self, now_m: float):
        """A DRIVES program, or a moving test, stops when no WS message and no /api request came for
        safety.link_timeout_ms (0 = off)."""
        run = self.runner
        try:
            lim = float(self.robot.p.get("safety", {}).get("link_timeout_ms", 3000)) / 1000.0
        except (TypeError, ValueError):
            lim = 3.0
        if lim <= 0 or now_m - self.last_touch <= lim:
            return
        if self.testing and not self.test_stop.is_set():
            self.test_stop.set()
            self.note("bad", "link_stop", "link lost for %.1f s -- test %s stopped" % (now_m - self.last_touch,
                                                                                     self.testing))
        if not run.busy() or not run.drives or self._wd_run == run.run_id:
            return
        self._wd_run = run.run_id
        run.stop()
        self.note("bad", "link_stop", "link lost for %.1f s -- program stopped" % (now_m - self.last_touch))

    # ------------------------------------------------------------------ vis thread
    def _vis_loop(self):
        last = 0.0
        seq = 0
        while True:
            try:
                if not self.vis_wanted:
                    time.sleep(0.2)
                    continue
                period = 0.5 if self.runner.busy() else 0.2        # 2 Hz under a program, 5 Hz idle
                t0 = time.monotonic()
                f, tf = self.robot.frame()
                if f is None or tf == last:
                    time.sleep(0.03)
                    continue
                last = tf
                d = vis_payload(f, tf, self.robot.p)
                seq += 1
                self._vis = (seq, dumps(d))
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
            except Exception:
                if time.monotonic() - self._err_t > 5.0:
                    self._err_t = time.monotonic()
                    traceback.print_exc()
                time.sleep(0.5)

    # ------------------------------------------------------------------ per client
    def _unique(self, base: str, me) -> str:
        taken = {c.label for c in self.clients if c is not me}
        label, k = base, 2
        while label in taken:
            label = "%s #%d" % (base, k)
            k += 1
        return label

    async def serve(self, sock) -> None:
        """One /ws connection, from accept to close.  The caller has checked the token."""
        self.start()
        await sock.accept()
        if len(self.clients) >= MAX_CLIENTS:
            await sock.close(code=1013, reason="too many consoles open (%d)" % MAX_CLIENTS)
            return
        self._n += 1
        ip = sock.client.host if sock.client else "?"
        c = Client(sock, ip, "ws%d" % self._n)
        c.label = self._unique("console/? %s" % ip, c)
        self.clients.add(c)
        self.touch(c.label)
        c.q.append(dumps(self.cfg(c.label)))
        rid, recs = self.runner.records(0)
        # the backlog stops where the log tick has got to; everything later reaches this client through the tick
        recs = [r for r in recs if r[0] <= self._log_n] if rid == self._log_rid else []
        c.q.append(dumps(dict(type="log", run_id=rid, items=self.log_items(recs[-200:]))))
        c.wake.set()
        sender = asyncio.ensure_future(self._sender(c))
        sender.add_done_callback(lambda t: t.cancelled() or t.exception())     # a failed send ends quietly
        try:
            await self._reader(c)                                              # returns on disconnect
        except Exception:
            pass
        finally:
            self.clients.discard(c)
            self._release_on_disconnect(c.key)
            if self.testing and self.test_by == c.label and not self.test_stop.is_set():
                self.test_stop.set()                     # losing the page stops a moving test too
                self.note("warn", "test_stop", "test %s stopped: %s disconnected" % (self.testing, c.label))
            sender.cancel()

    async def _sender(self, c: Client):
        sock = c.sock
        while True:
            await c.wake.wait()
            c.wake.clear()
            while c.q:
                await sock.send_text(c.q.popleft())
            for kind in ("tel", "sys", "vis"):
                if c.subs.get(kind) and kind in self.latest:
                    seq, txt = self.latest[kind]
                    if c.sent.get(kind) != seq:
                        c.sent[kind] = seq
                        await sock.send_text(txt)
            res = c.subs.get("scan")
            if res and "scan%d" % res in self.latest:
                seq, txt = self.latest["scan%d" % res]
                if c.sent.get("scan") != seq:
                    c.sent["scan"] = seq
                    await sock.send_text(txt)
            if c.subs.get("perc") and "perc" in self.latest:
                seq, txt = self.latest["perc"]
                if c.sent.get("perc") != seq:
                    c.sent["perc"] = seq
                    await sock.send_text(txt)
            lv = c.subs.get("map")
            if lv and "map%d" % lv in self.latest:
                seq, txt = self.latest["map%d" % lv]
                if c.sent.get("map") != seq:
                    c.sent["map"] = seq
                    await sock.send_text(txt)
            if c.subs.get("cloud") and self.mapper is not None and time.monotonic() - c.cloud_t >= CLOUD_PERIOD_S:
                try:
                    nxt = self.mapper.cloud_next(c.cloud_state, 2000 if c.subs["cloud"] == 2 else 8000)
                except Exception:                        # a cloud bug must never end this client's tel stream
                    nxt, c.cloud_t = None, time.monotonic() + 5.0
                if nxt is not None:                      # one chunk per wake: tel never waits behind a burst
                    c.cloud_state, c.cloud_t = nxt[0], time.monotonic()
                    await sock.send_text(nxt[1])
            if c.subs.get("live") and self.live is not None and c.sent.get("live") != self.live[0]:
                seq, d, txt, path, trail = self.live
                c.sent["live"] = seq
                extra = {}
                if d.get("epoch") != c.live_epoch or c.need_trail:
                    if trail is not None:
                        extra["trail"] = trail
                        c.need_trail = False
                        c.live_epoch = d.get("epoch")
                        c.live_path_v = None
                if d.get("path_v") is not None and d.get("path_v") != c.live_path_v and path is not None:
                    extra["path"] = path
                    c.live_path_v = d.get("path_v")
                await sock.send_text(dumps(dict(d, **extra)) if extra else txt)

    async def _reader(self, c: Client):
        sock = c.sock
        while True:
            m = await sock.receive()
            if m.get("type") == "websocket.disconnect":
                return
            txt = m.get("text")
            if txt is None and m.get("bytes") is not None:
                txt = m["bytes"].decode("utf-8", "replace")
            self.touch(c.label)
            try:
                msg = json.loads(txt or "")
            except ValueError:
                continue
            if not isinstance(msg, dict):
                continue
            t = msg.get("type")
            if t == "ping":
                c.q.append(dumps(dict(type="pong", c=msg.get("c"), t=round(time.time(), 3))))
                c.wake.set()
            elif t == "drive":
                ok, why = self.drive(c.key, c.label, msg.get("v", 0.0), msg.get("steer", 0.0))
                if not ok and time.monotonic() - c.refused_t >= 1.0:
                    c.refused_t = time.monotonic()
                    c.q.append(dumps(dict(type="refused", what="drive", reason=why)))
                    c.wake.set()
            elif t == "stop":
                self.manual_stop(c.key)
            elif t == "estop":
                if self.on_estop is not None:
                    self.on_estop(c.label)
            elif t in ("hello", "sub"):
                if t == "hello":
                    lab = "".join(ch for ch in str(msg.get("label") or "console/?") if ch.isprintable())[:40]
                    c.label = self._unique("%s %s" % (lab.strip() or "console/?", c.ip), c)
                self._subs(c, msg)
                if t == "hello":
                    c.q.append(dumps(self.cfg(c.label)))
                    c.wake.set()

    @staticmethod
    def _subs(c: Client, m: dict):
        was_live = c.subs["live"]
        for k in ("tel", "live", "vis", "sys", "log"):
            if k in m:
                c.subs[k] = bool(m[k])
        if "scan" in m:
            s = m["scan"]
            c.subs["scan"] = s if s in (0, 1, 2) and not isinstance(s, bool) else (1 if s else 0)
            c.sent.pop("scan", None)
        if "perc" in m:
            c.subs["perc"] = bool(m["perc"])
            c.sent.pop("perc", None)
        if "map" in m:                                   # 0 off, 1 full resolution, 2 half (the phone)
            s = m["map"]
            c.subs["map"] = s if s in (0, 1, 2) and not isinstance(s, bool) else (1 if s else 0)
            c.sent.pop("map", None)
        if "cloud" in m:                                 # 0 off, 1 = 8000-point chunks, 2 = 2000 (the phone's Load 3D)
            s = m["cloud"]
            c.subs["cloud"] = s if s in (0, 1, 2) and not isinstance(s, bool) else (1 if s else 0)
            if not c.subs["cloud"]:
                c.cloud_state = None                     # re-subscribing starts the cloud over (reset=true)
        if c.subs["live"] and not was_live:
            c.need_trail, c.live_path_v = True, None
        for k in ("tel", "sys", "vis", "live"):
            if not c.subs.get(k):
                c.sent.pop(k, None)
