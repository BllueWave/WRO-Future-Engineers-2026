"""The console's live view of a running program: what it believes (pose, plan, signs, lot) next to what it sees.

Two layers.  The dev agent attaches a `Bus` as `robot.live`; a program publishes through `pub()`, which is a no-op when
`robot.live` does not exist (tools/sim_run.py, race/race_main.py -- race mode stays radio-free and quiet) and when
nobody has read the bus for READ_TTL_S.  `pub_on()` tells a program whether to build the arguments at all, so an
unwatched run pays one attribute lookup per loop.

Units in: mm, radians (the programs' own).  Units out (`read()`): int mm, degrees with 1 decimal -- the wire format
of docs/CONSOLE_SPEC.md 5.4.  Field frame: origin at the field centre, x right, y up.  Robot frame: the rear-axle
centre, x forward, y left.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque

import numpy as np

from . import field as F

SECTIONS = {("straight", 0): "S", ("straight", 1): "E", ("straight", 2): "N", ("straight", 3): "W",
            ("corner", 0): "SE", ("corner", 1): "NE", ("corner", 2): "NW", ("corner", 3): "SW"}


def section(x: float, y: float) -> str:
    """S/E/N/W for a straight, SE/NE/NW/SW for a corner, `out` inside the island or beyond the walls."""
    return SECTIONS.get(F.where(x, y), "out")


def _scalar(v):
    if isinstance(v, (bool, str)) or v is None:
        return v
    if isinstance(v, np.generic):
        v = v.item()
    if isinstance(v, int):
        return v
    if isinstance(v, float):
        return round(v, 3) if math.isfinite(v) else None
    return None


def _flat(X, Y, cap: int):
    """Two coordinate arrays -> [x0, y0, x1, y1, ...] int mm, finite points only, decimated by stride to <= cap."""
    X, Y = np.asarray(X, float).ravel(), np.asarray(Y, float).ravel()
    ok = np.isfinite(X) & np.isfinite(Y)
    X, Y = X[ok], Y[ok]
    if len(X) > cap:
        k = np.linspace(0, len(X) - 1, cap).astype(int)
        X, Y = X[k], Y[k]
    return np.rint(np.stack([X, Y], 1)).astype(int).ravel().tolist()


class Bus:
    READ_TTL_S = 3.0          # active() while a reader (hub 10 Hz, Runner 5 Hz, GET /api/live) read within this
    MAX_CAM, MAX_LIDAR, MAX_PATH, MAX_TRAIL = 120, 180, 400, 900

    def __init__(self):
        self._lock = threading.Lock()
        self._read_t = -1e9
        self.epoch = 0
        self.src, self.run_id, self.running = None, None, False
        self._reset()

    def _reset(self):
        self.pose = None                          # (x, y, th_rad, sxy, sth_rad)
        self.pose_t = 0.0                         # monotonic stamp of the pose
        self.fit = self.state = self.laps = self.dir = None
        self.cam = None                           # (X, Y) float arrays, robot frame, <= MAX_CAM
        self.seen = self.pillars = self.lot = None
        self.path_v, self.path = None, None       # path: flat int list, <= MAX_PATH points
        self.trail = deque(maxlen=self.MAX_TRAIL)  # (x, y) int
        self._trail_last = None                   # (x, y, th) of the last trail point
        self._frames = deque(maxlen=64)           # (monotonic, frames)
        self.prog_fps = self.loop_ms = None
        self.extra = None
        # BRAIN_SPEC 8.2 (the map / cloud side): which frame the pose is in, SLAM's grid, and the pose history the
        # mapper places a scan or a depth frame with (pose_at)
        self.frame = "field"                      # "field" | "map" (programs/slam.py)
        self.grid = None                          # a gridmap.GridMap (slam.py): the Map page serves it
        self.scan_pose = deque(maxlen=32)         # (t_mono_scan, x, y, th): the lidar-corrected pose AT a scan
        self.pose_hist = deque(maxlen=64)         # (pose_t, x, y, th) of every accepted pose
        # BRAIN_SPEC 8.2 (the obstacle brain's side): the program's pose-aware perception (the mapper reuses it for the
        # console's `perc` instead of computing another), the lidar-confirmed signs, the shield's state
        self.perc = None                          # a lidar_perc.Perception (object reference; not in read())
        self.lpil = None                          # [[x, y, colour | None, hits, conf_pct, seat]] field / pose frame
        self.shield = None                        # Shield.state(): act, free, need, v_cap, n_slow, n_steer, n_brake, age_ms

    # ------------------------------------------------------------------ writers
    def active(self) -> bool:
        return time.monotonic() - self._read_t < self.READ_TTL_S

    def clear(self, src: str | None, run_id: str | None = None) -> None:
        """A new program (or none): epoch += 1, every published item dropped."""
        with self._lock:
            self.epoch += 1
            self._reset()
            self.src, self.run_id, self.running = src, run_id, src is not None

    def set_running(self, running: bool) -> None:
        with self._lock:
            self.running = bool(running)

    def update(self, **kw) -> None:
        """Thread-safe; the keys of CONSOLE_SPEC 4.1, unknown keys ignored.  Never raises into the program."""
        now = time.monotonic()
        with self._lock:
            for k, v in kw.items():
                try:
                    self._set(k, v, now)
                except Exception:
                    pass

    def _set(self, k, v, now):
        if k == "pose":
            if v is None:
                self.pose = None
                return
            x, y, th, sxy, sth = (float(a) for a in v)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(th)):
                return
            self.pose, self.pose_t = (x, y, th, sxy, sth), now
            self.pose_hist.append((now, x, y, th))
            last = self._trail_last
            if last is None or math.hypot(x - last[0], y - last[1]) >= 10.0 or \
                    abs((th - last[2] + math.pi) % (2 * math.pi) - math.pi) >= math.radians(2.0):
                self.trail.append((int(round(x)), int(round(y))))
                self._trail_last = (x, y, th)
        elif k == "fit":
            self.fit = None if v is None or not math.isfinite(float(v)) else float(v)
        elif k == "state":
            self.state = None if v is None else str(v)
        elif k == "laps":
            self.laps = None if v is None else round(float(v), 2)
        elif k == "dir":
            self.dir = {1: "ccw", -1: "cw"}.get(int(v) if v is not None else 0)
        elif k == "cam":
            if v is None:
                self.cam = None
                return
            X, Y = np.asarray(v[0], float).ravel(), np.asarray(v[1], float).ravel()
            if len(X) > self.MAX_CAM:
                s = int(math.ceil(len(X) / self.MAX_CAM))
                X, Y = X[::s], Y[::s]
            self.cam = (X.copy(), Y.copy())
        elif k == "seen":
            self.seen = None if v is None else [[int(round(q[0])), int(round(q[1])), str(q[2])] for q in list(v)[:12]]
        elif k == "pillars":
            if v is None:
                self.pillars = None
                return
            out = []
            for q in v:
                row = [int(round(q[0])), int(round(q[1])), str(q[2])]
                if len(q) > 3 and q[3] is not None:
                    row.append(int(q[3]))
                out.append(row)
            self.pillars = out
        elif k == "lot_x":
            if v is None:
                self.lot = None
            else:
                a, b = sorted((float(v[0]), float(v[1])))
                self.lot = [int(round(a)), -1500, int(round(b)), int(round(-1500.0 + F.LOT_DEPTH))]
        elif k == "path":
            if v is None:
                return
            ver, xy = v
            if ver == self.path_v:
                return
            self.path_v = int(ver)
            if xy is None:
                self.path = None
                return
            xy = np.asarray(xy, float)
            self.path = _flat(xy[:, 0], xy[:, 1], self.MAX_PATH) if len(xy) else []
        elif k == "frames":
            n = int(v)
            self._frames.append((now, n))
            old = None
            for t, f in self._frames:
                if now - t >= 1.0:
                    old = (t, f)
                else:
                    break
            if old is not None and now > old[0]:
                self.prog_fps = round((n - old[1]) / (now - old[0]), 1)
        elif k == "loop_ms":
            x = float(v)
            if math.isfinite(x):
                self.loop_ms = x if self.loop_ms is None else 0.8 * self.loop_ms + 0.2 * x
        elif k == "extra":
            if v is None:
                self.extra = None
            else:
                self.extra = {str(a): _scalar(b) for a, b in list(dict(v).items())[:12]}
        elif k == "frame":
            self.frame = "map" if v == "map" else "field"
        elif k == "grid":
            self.grid = v                         # an object reference: the mapper reads it under the grid's lock
        elif k == "scan_pose":
            t, x, y, th = (float(a) for a in v)
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(th):
                self.scan_pose.append((t, x, y, th))
        elif k == "perc":
            self.perc = v                         # the mapper checks its `t` against the current scan
        elif k == "lpil":
            if v is None:
                self.lpil = None
                return
            out = []
            for q in list(v)[:24]:
                col = q[2] if q[2] in ("red", "green") else None
                out.append([int(round(float(q[0]))), int(round(float(q[1]))), col, int(q[3]),
                            int(round(100.0 * float(q[4]))), int(q[5])])
            self.lpil = out
        elif k == "shield":
            self.shield = None if v is None else {str(a): _scalar(b) for a, b in dict(v).items()}

    def pose_at(self, t_mono: float):
        """(x, y, th) at monotonic t: a scan_pose within 20 ms of t if one exists, else linear interpolation in the
        pose history; None when t lies outside the history by > 0.15 s (a stale pose never places a new scan)."""
        with self._lock:
            sp = list(self.scan_pose)
            ph = list(self.pose_hist)
        best = min(sp, key=lambda q: abs(q[0] - t_mono)) if sp else None
        if best is not None and abs(best[0] - t_mono) <= 0.02:
            return best[1], best[2], best[3]
        if not ph or t_mono < ph[0][0] - 0.15 or t_mono > ph[-1][0] + 0.15:
            return None
        if t_mono <= ph[0][0]:
            return ph[0][1:]
        if t_mono >= ph[-1][0]:
            return ph[-1][1:]
        for a, b in zip(ph, ph[1:]):
            if a[0] <= t_mono <= b[0]:
                f = (t_mono - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
                dth = (b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi
                return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dth
        return ph[-1][1:]

    # ------------------------------------------------------------------ reader
    def read(self, scan=None, lidar_cfg: dict | None = None, pts: bool = True, trail: bool = True) -> dict:
        """The `live` payload (CONSOLE_SPEC 5.4) without `type` and `t`.  Marks the bus as read (active)."""
        now = time.monotonic()
        with self._lock:
            self._read_t = now
            out: dict = dict(epoch=self.epoch, src=self.src, run_id=self.run_id, running=self.running)
            if self.src is None:
                out["pose"] = None
                return out
            pose = self.pose
            out["age_ms"] = int((now - self.pose_t) * 1000) if pose is not None else None
            if pose is not None:
                x, y, th, sxy, sth = pose
                out["pose"] = dict(x=int(round(x)), y=int(round(y)), th=round(math.degrees(th), 1),
                                   sxy=int(round(sxy)) if math.isfinite(sxy) else None,
                                   sth=round(math.degrees(sth), 1) if math.isfinite(sth) else None,
                                   fit=int(round(self.fit)) if self.fit is not None else None)
                out["section"] = section(x, y) if self.frame == "field" else None
            else:
                out["pose"] = None
                out["section"] = None
            out.update(state=self.state, laps=self.laps, dir=self.dir, seen=self.seen, pillars=self.pillars,
                       lot=self.lot, path_v=self.path_v, path=self.path,
                       prog_fps=self.prog_fps, loop_ms=round(self.loop_ms, 1) if self.loop_ms is not None else None,
                       extra=self.extra, frame=self.frame, lpil=self.lpil, shield=self.shield)
            if self.grid is not None:
                out["grid_v"] = int(getattr(self.grid, "version", 0))
                out["grid_src"] = "slam"
            if trail:
                out["trail"] = [c for q in self.trail for c in q]
            cam = self.cam
        if pts:
            out["cam_car"] = _flat(cam[0], cam[1], self.MAX_CAM) if cam is not None else None
            out["cam"] = out["lidar"] = None
            if pose is not None:
                x, y, th = pose[0], pose[1], pose[2]
                c, s = math.cos(th), math.sin(th)
                if cam is not None:
                    X, Y = cam
                    out["cam"] = _flat(x + c * X - s * Y, y + s * X + c * Y, self.MAX_CAM)
                if scan is not None and now - self.pose_t < 1.0:    # never the current scan at a stale pose
                    out["lidar"] = self._lidar_field(scan, lidar_cfg or {}, x, y, c, s)
        return out

    def _lidar_field(self, scan, cfg, x, y, c, s):
        """The latest revolution in the field frame: bins with 90 < d < 4000 mm at a 2 deg stride, shifted by the
        lidar's mount (params lidar.pos_mm, robot frame), then placed by the pose -- <= 180 points."""
        try:
            d = np.asarray(scan.dist, float)
            a = np.radians(scan.angles())
            stride = max(1, int(round(2.0 / max(float(scan.res), 1e-3))))
            d, a = d[::stride], a[::stride]
            ok = (d > 90.0) & (d < 4000.0)
            px, py = (cfg.get("pos_mm") or (0.0, 0.0))[:2]
            X = float(px) + d[ok] * np.cos(a[ok])
            Y = float(py) + d[ok] * np.sin(a[ok])
            return _flat(x + c * X - s * Y, y + s * X + c * Y, self.MAX_LIDAR)
        except Exception:
            return None


def pub(robot, **kw) -> None:
    """Publish to the console's live view.  No-op without a bus (sim_run, race mode) or with nobody watching."""
    bus = getattr(robot, "live", None)
    if bus is None:
        return
    try:
        if bus.active():
            bus.update(**kw)
    except Exception:
        pass


def pub_on(robot) -> bool:
    """True when pub() would store something: a program skips building its arguments otherwise."""
    bus = getattr(robot, "live", None)
    try:
        return bus is not None and bus.active()
    except Exception:
        return False
