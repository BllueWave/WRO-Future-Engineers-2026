"""The console's background worker (docs/BRAIN_SPEC.md 8.3): perception for the MAP view, the known-pose field map,
SLAM's map, the depth cloud -- one daemon thread (<= 10 Hz) that does only what a console subscribes to.  Created by
the agent, so nothing of it exists in race mode (WRO 11.10: radio-free, and not one cycle spent on a viewer).

    perc   a new scan -> the `perc` message: bus.perc when the running program computed it for THIS scan (src
           "program"), else lidar_perc.of(robot) (src "idle"; cached on the Scan, so the program shares it)
    map    bus.grid (programs/slam.py) -> served as is (src "slam", frame "map").  Else, while the bus has a confident
           pose in the field frame, robot.scan is integrated at bus.pose_at(scan.t - lag) into the mapper's own
           GridMap(frame="field") -- 5 Hz while a DRIVES program runs (map.live_hz), 10 Hz otherwise; a new bus epoch
           (a new program) clears it.  `map1` (full) <= 1 Hz, `map2` (scale 2) <= 0.5 Hz, only when the version changed
    cloud  each new depth frame (<= 5 Hz) with its pose pose_at(t_exp) -> cloud.Cloud.add; no pose -> the car frame
           (the latest frame only).  A bus epoch or frame change clears it.  Chunks are cut per client on request
           (cloud_next), <= 8000 points, so a late subscriber gets the whole cloud from its start, paced by the hub

The GIL rule (BRAIN_SPEC 3): every per-scan result is computed once; a PNG is encoded here, never on the hub's event
loop; a map request over REST (bw map) keeps the known-pose map alive for 60 s without a console.
"""
from __future__ import annotations

import base64
import math
import threading
import time
import traceback

import numpy as np

from . import gridmap as GM

REST_KEEP_S = 60.0          # a REST map request keeps the known-pose map integrating this long (bw map, the bridge)
WANT_TTL_S = 1.0            # the hub renews want() every 50 ms tick; older = nobody subscribes


def _dumps(obj) -> str:
    from .hub import dumps
    return dumps(obj)


class Mapper:
    PERIOD = 0.1

    def __init__(self, robot, bus, drives=None):
        """drives: a callable -> True while a DRIVES program runs (the known-pose map then integrates at
        map.live_hz)."""
        self.robot, self.bus = robot, bus
        self.drives = drives or (lambda: False)
        self.latest = {}                     # "perc" | "map1" | "map2" -> (seq, text), built on this thread
        self._seq = 0
        self._want = (False, 0, False)
        self._want_t = -1e9
        self._rest_t = -1e9
        self.field = None                    # the known-pose GridMap(frame="field")
        self._field_epoch = None
        self._int_t = 0.0
        self._int_scan_t = None
        self._ignore_grid = None             # a cleared SLAM grid (the object: an id() can be reused): not served
        self._map_last = {}                  # level -> ((id, version), t)
        self._perc_t = None
        self._perc_msg = None
        self.cloud = None
        self._cloud_cfg = None
        self._cloud_key = None               # (bus epoch, frame) the cloud belongs to
        self._depth_t = None
        self._depth_add_t = 0.0
        self._dg = None                      # (key, DepthGeom)
        self._chunks = {}                    # slice key -> ((epoch, ver_to), text), see cloud_next
        self.err = None
        self._err_t = 0.0
        self.stats = dict(integrated=0, perc=0, png=0, cloud_frames=0)
        self._run = True
        self._th = threading.Thread(target=self._loop, name="mapper", daemon=True)
        self._th.start()

    # ------------------------------------------------------------------ the hub's side
    def want(self, perc: bool, map_level: int, cloud: bool) -> None:
        """The hub sets it every tick: perc, map_level (a bit mask: 1 = full resolution, 2 = half), cloud."""
        self._want, self._want_t = (bool(perc), int(map_level or 0), bool(cloud)), time.monotonic()

    def rest_touch(self) -> None:
        """A REST map request: keep the known-pose map integrating for REST_KEEP_S."""
        self._rest_t = time.monotonic()

    def _wanted(self, now: float):
        perc, lvl, cloud = self._want if now - self._want_t <= WANT_TTL_S else (False, 0, False)
        return perc, lvl, cloud, lvl > 0 or now - self._rest_t <= REST_KEEP_S

    def close(self):
        self._run = False

    # ------------------------------------------------------------------ loop
    def _loop(self):
        while self._run:
            t0 = time.monotonic()
            try:
                self.tick(t0)
            except Exception as e:
                self.err = "%s: %s" % (type(e).__name__, e)
                if t0 - self._err_t > 5.0:               # one traceback per 5 s, never a dead mapper
                    self._err_t = t0
                    traceback.print_exc()
            time.sleep(max(0.01, self.PERIOD - (time.monotonic() - t0)))

    def tick(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        perc, lvl, cloud, mapping = self._wanted(now)
        if perc:
            self._perc_tick()
        if mapping:
            self._map_tick(now)
            if lvl:
                self._map_msgs(now, lvl)
        if cloud:
            self._cloud_tick(now)

    # ------------------------------------------------------------------ perc
    def _perception(self):
        """(Perception, src) of the latest scan or (None, None): the program's own when it is for this scan."""
        sc = getattr(self.robot, "scan", None)
        if sc is None:
            return None, None
        try:
            from . import lidar_perc as LP
        except ImportError:
            return None, None
        bp = getattr(self.bus, "perc", None)
        if bp is not None and getattr(bp, "t", None) == sc.t:
            return bp, "program"
        return LP.of(self.robot), "idle"

    def perc_body(self, pts: bool = False) -> dict | None:
        """The `perc` message for GET /api/perc (pts: + X, Y flat int mm, car frame); None without a scan."""
        pc, src = self._perception()
        if pc is None:
            return None
        d = dict(type="perc", t=round(time.time(), 3), age_ms=int((time.monotonic() - pc.t) * 1000), src=src,
                 **pc.wire())
        if pts:
            d["X"] = np.rint(pc.X).astype(int).tolist()
            d["Y"] = np.rint(pc.Y).astype(int).tolist()
        return d

    def _perc_tick(self):
        sc = getattr(self.robot, "scan", None)
        if sc is None or sc.t == self._perc_t:
            return
        bp = getattr(self.bus, "perc", None)
        if (bp is not None and getattr(bp, "t", None) != sc.t and getattr(self.bus, "running", False)
                and time.monotonic() - sc.t < 0.1):
            return          # a running program publishes THIS scan's perception within a loop tick: take it next
            #                 tick rather than computing a second, pose-free one on the same interpreter
        d = self.perc_body()
        if d is None:
            return
        self._perc_t = sc.t
        self._seq += 1
        self.latest["perc"] = (self._seq, _dumps(d))
        self.stats["perc"] += 1

    # ------------------------------------------------------------------ map
    def map_obj(self):
        """The served GridMap (BRAIN_SPEC 8.3) or None."""
        return self.served()[0]

    def served(self):
        """(GridMap, src) served: SLAM's grid ("slam"), else the known-pose field map ("field"), else (None, None)."""
        g = getattr(self.bus, "grid", None)
        if g is not None and g is not self._ignore_grid:
            return g, "slam"
        if self.field is not None:
            return self.field, "field"
        return None, None

    def _cfg(self, sec: str) -> dict:
        return dict((self.robot.p.get(sec) or {}))

    def _points(self, sc, max_mm: float):
        """The scan's returns in the car frame without the car's own body: lidar_perc's non-clutter points (computed
        once per scan and shared), or the raw bins outside the blind sectors."""
        try:
            from . import lidar_perc as LP
            pc = getattr(self.bus, "perc", None)                    # the running program's, when it is this scan's
            if pc is None or getattr(pc, "t", None) != sc.t:
                pc = LP.of(self.robot)
            if pc is not None and pc.t == sc.t:
                keep = pc.cls != LP.CLS_CLUTTER
                return pc.X[keep].astype(np.float64), pc.Y[keep].astype(np.float64)
        except ImportError:
            pass
        from . import loc as L
        lc = self._cfg("lidar")
        X, Y, _s = L.Localizer.lidar_points(sc, tuple((lc.get("pos_mm") or (0.0, 0.0))[:2]),
                                            [tuple(b) for b in lc.get("block_deg", [])], 1.0,
                                            float(lc.get("min_mm", 90.0)), max_mm)
        return X, Y

    def _map_tick(self, now: float):
        bus = self.bus
        if bus.epoch != self._field_epoch:           # a new program: the last one's map is not this one's
            self.field, self._field_epoch, self._map_last, self._ignore_grid = None, bus.epoch, {}, None
            self.latest.pop("map1", None)
            self.latest.pop("map2", None)
        g = getattr(bus, "grid", None)
        if g is not None and g is not self._ignore_grid:
            return                                   # SLAM runs (or ran): its map is served, nothing to integrate
        if getattr(bus, "frame", "field") != "field" or not hasattr(bus, "pose_at"):
            return
        mp = self._cfg("map")
        hz = float(mp.get("live_hz", 5.0)) if self.drives() else 10.0
        if now - self._int_t < 1.0 / max(0.5, hz) - 0.005:
            return
        sc = getattr(self.robot, "scan", None)
        if sc is None or sc.t == self._int_scan_t:
            return
        pose = bus.pose
        if pose is None or not (pose[3] < 60.0 and pose[4] < math.radians(4.0)) or bus.state == "LOST":
            return                                   # a lost or uncertain pose would smear the map
        lag = float(self._cfg("lidar").get("scan_lag_s", 0.05))
        pz = bus.pose_at(sc.t - lag)
        if pz is None:
            return
        max_mm = float(mp.get("max_mm", 4000.0))
        X, Y = self._points(sc, max_mm)
        if len(X) < 10:
            return
        if self.field is None:
            self.field = GM.GridMap(float(mp.get("res_mm", 20.0)), int(mp.get("size", 512)), None, "field")
        lc = self._cfg("lidar")
        self.field.integrate(pz, X, Y, tuple((lc.get("pos_mm") or (0.0, 0.0))[:2]), max_mm)
        self._int_t, self._int_scan_t = now, sc.t
        self.stats["integrated"] += 1

    def map_meta(self) -> dict | None:
        g, src = self.served()
        if g is None:
            return None
        m = g.meta()
        m.update(src=src, epoch=self.bus.epoch, known_m2=round(g.known_m2(), 2), t=round(time.time(), 3))
        return m

    def map_message(self, level: int) -> str | None:
        """The `map` message text at level 1 (full) or 2 (scale 2), built now."""
        g, src = self.served()
        if g is None:
            return None
        png, meta = g.png(True, 2 if level == 2 else 1)
        self.stats["png"] += 1
        d = dict(type="map", t=round(time.time(), 3), src=src, frame=g.frame, v=meta["v"], epoch=self.bus.epoch,
                 res_mm=meta["res_mm"], n=meta["n"], origin_mm=meta["origin_mm"], crop=meta["crop"],
                 scale=meta["scale"], scans=meta["scans"], known_m2=round(g.known_m2(), 2),
                 png=base64.b64encode(png).decode("ascii"))
        return _dumps(d)

    def _map_msgs(self, now: float, lvl: int):
        g, _src = self.served()
        if g is None:
            return
        key = (id(g), g.version)
        for level, period in ((1, 1.0), (2, 2.0)):
            if not lvl & level:
                continue
            last = self._map_last.get(level)
            if last is not None and (last[0] == key or now - last[1] < period):
                continue
            txt = self.map_message(level)
            if txt is None:
                continue
            self._seq += 1
            self.latest["map%d" % level] = (self._seq, txt)
            self._map_last[level] = (key, now)

    def clear_map(self) -> None:
        """Drop the known-pose map and stop serving the current SLAM grid (a new SLAM run serves its own)."""
        g = getattr(self.bus, "grid", None)
        if g is not None:
            self._ignore_grid = g
        self.field, self._map_last = None, {}
        self.latest.pop("map1", None)
        self.latest.pop("map2", None)

    def save_map(self, name: str) -> list:
        """The served map -> runs/maps/<name>.{png,json,npy} (name-2, -3 ... when taken: a save never replaces a
        map); [] when there is none."""
        g, src = self.served()
        if g is None:
            return []
        base = GM.unique_base(GM.safe_name(name) or time.strftime("%Y%m%d-%H%M%S"))
        return g.save(base, extra=dict(src=src, epoch=self.bus.epoch))

    def overlay_png(self, scale: int = 0) -> bytes | None:
        """The served map in colour for offline review: grey map (black occupied, white free, grey unknown), the true field
        walls in blue when the frame is "field", the trail (grey), the latest scan at the pose (range rainbow), the
        car's footprint and heading (blue), lidar pillars (lpil, sign colours), a 1 m scale bar and a caption.  The
        crop is scaled to ~800 px (scale 0 = auto)."""
        import cv2
        g, src = self.served()
        if g is None:
            return None
        i0, j0, h, w, L, v = g._crop(True, 1)
        T = np.flipud(GM.GridMap._tri(L))
        f = int(scale) if scale else max(1, min(6, int(800 // max(h, w))))
        img = cv2.cvtColor(cv2.resize(T, (w * f, h * f), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
        res, (ox, oy) = g.res, g.origin

        def px(x, y):
            return (int(round(((np.asarray(x) - ox) / res - j0) * f)),
                    int(round((h - ((np.asarray(y) - oy) / res - i0)) * f)))

        def pxa(X, Y):
            u = np.rint(((np.asarray(X, float) - ox) / res - j0) * f).astype(np.int32)
            vv = np.rint((h - ((np.asarray(Y, float) - oy) / res - i0)) * f).astype(np.int32)
            return u, vv

        bus = self.bus
        same = getattr(bus, "frame", "field") == g.frame
        if g.frame == "field":
            from . import field as F
            for x1, y1, x2, y2 in F.WALLS:
                cv2.line(img, px(x1, y1), px(x2, y2), (200, 120, 40), 1, cv2.LINE_AA)
        if same and len(bus.trail) > 1:
            tr = np.array(list(bus.trail), float)
            u, vv = pxa(tr[:, 0], tr[:, 1])
            cv2.polylines(img, [np.stack([u, vv], 1)], False, (150, 150, 150), 1, cv2.LINE_AA)
        pose = bus.pose if same else None
        sc = getattr(self.robot, "scan", None)
        if pose is not None and sc is not None and hasattr(bus, "pose_at"):
            # the scan at the pose OF the scan: the published pose is up to two scans newer, and a car turning at
            # 44 deg/s drew its scan 9 deg off the walls with it (simulator, 2026-09-23)
            pz = bus.pose_at(sc.t - float(self._cfg("lidar").get("scan_lag_s", 0.05)))
            pose = pose if pz is None else tuple(pz) + tuple(pose[3:])
        if pose is not None:
            x, y, th = pose[0], pose[1], pose[2]
            if sc is not None:
                X, Y = self._points(sc, 6000.0)
                if len(X):
                    c, s = math.cos(th), math.sin(th)
                    u, vv = pxa(x + c * X - s * Y, y + s * X + c * Y)
                    rr = np.hypot(X, Y)
                    hue = np.clip(rr / 4000.0 * 135.0, 0, 135).astype(np.uint8)       # OpenCV hue 0-179: red..violet
                    hsv = np.stack([hue, np.full_like(hue, 230), np.full_like(hue, 255)], 1)[None]
                    col = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0]
                    ok = (u >= 0) & (u < img.shape[1]) & (vv >= 0) & (vv < img.shape[0])
                    for a, b, cc in zip(u[ok], vv[ok], col[ok]):
                        cv2.circle(img, (int(a), int(b)), max(1, f // 2), tuple(int(k) for k in cc), -1)
            car = dict(dict(rear_mm=34.0, front_mm=179.0, half_w_mm=80.5), **(self.robot.p.get("car") or {}))
            c, s = math.cos(th), math.sin(th)
            cs = [(-car["rear_mm"], -car["half_w_mm"]), (car["front_mm"], -car["half_w_mm"]),
                  (car["front_mm"], car["half_w_mm"]), (-car["rear_mm"], car["half_w_mm"])]
            poly = np.array([px(x + c * a - s * b, y + s * a + c * b) for a, b in cs], np.int32)
            cv2.polylines(img, [poly], True, (255, 90, 0), 2, cv2.LINE_AA)
            cv2.arrowedLine(img, px(x, y), px(x + c * 250.0, y + s * 250.0), (255, 90, 0), 2, cv2.LINE_AA,
                            tipLength=0.25)
        lp = getattr(bus, "lpil", None) if same else None
        for q in lp or []:
            colour = {"red": (40, 40, 220), "green": (40, 170, 40)}.get(q[2], (160, 160, 160))
            cv2.circle(img, px(q[0], q[1]), max(3, int(25.0 / res * f)), colour, -1 if q[2] else 2, cv2.LINE_AA)
        H = img.shape[0]
        bar = int(round(1000.0 / res * f))
        cv2.line(img, (10, H - 12), (10 + bar, H - 12), (0, 0, 0), 3)
        cv2.putText(img, "1 m", (14 + bar, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, "%s  frame %s  v%d  %.1f m2  %d mm cells" % (src, g.frame, v, g.known_m2(), int(res)),
                    (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        ok, buf = cv2.imencode(".png", img)
        return buf.tobytes() if ok else None

    # ------------------------------------------------------------------ cloud
    def _cloud_tick(self, now: float):
        robot, bus = self.robot, self.bus
        if getattr(robot, "depth_cam", None) is None or now - self._depth_add_t < 0.2:
            return
        d, t_exp = robot.depth()
        if d is None or t_exp == self._depth_t:
            return
        from . import cloud as CL
        cc = dict(dict(voxel_mm=25.0, max_points=200000, stride=8, max_mm=3000.0, rgb=0), **self._cfg("cloud"))
        key = tuple(sorted(cc.items()))
        if self.cloud is None or self._cloud_cfg != key:
            old = self.cloud
            self.cloud = CL.Cloud(float(cc["voxel_mm"]), int(cc["max_points"]), int(cc["stride"]),
                                  float(self._cfg("depth").get("min_mm", 200.0)), float(cc["max_mm"]),
                                  bool(int(cc["rgb"])))
            if old is not None:                      # a new cloud (cloud.* changed) never reuses an epoch: every
                self.cloud.epoch = old.epoch + 1     # client starts over with reset=true
            self._cloud_cfg, self._cloud_key, self._chunks = key, None, {}
        frame = getattr(bus, "frame", "field")
        ck = (bus.epoch, frame)
        if ck != self._cloud_key:
            self.cloud.clear(frame)
            self._cloud_key, self._chunks = ck, {}
        pose = None
        pz = bus.pose if bus.src is not None else None
        if pz is not None and pz[3] < 80.0 and hasattr(bus, "pose_at"):
            pose = bus.pose_at(t_exp)
        H, W = d.shape[:2]
        dgk = (H, W, repr(sorted((self.robot.p.get("camera") or {}).items())),
               repr(sorted((self.robot.p.get("depth") or {}).items())))
        if self._dg is None or self._dg[0] != dgk:
            from . import depth as D
            from . import vision as V
            self._dg = (dgk, D.DepthGeom(V.Ground(self.robot.p["camera"], W, H), D.config(self.robot.p), W, H))
        bgr = None
        if self.cloud.rgb:
            f, tf = robot.frame()
            lat = float(self._cfg("camera").get("latency_s", 0.12))
            if f is not None and abs((tf - lat) - t_exp) <= float(self._cfg("depth").get("pair_s", 0.08)):
                bgr = f
        self.cloud.add(d, t_exp, self._dg[1], pose, frame if pose is not None else "car", bgr)
        self._depth_t, self._depth_add_t = t_exp, now
        self.stats["cloud_frames"] += 1

    def cloud_next(self, last, cap: int = 8000):
        """((epoch, ver_to), text) of the next `cloud` chunk for a client whose last state is `last` ((epoch, ver) or
        None), or None when it has everything.  A client from another epoch (the cloud was cleared, the frame changed)
        starts over with reset=true.  Chunks are cached per slice (epoch, ver, cap, reset, end): clients at the same
        place share one."""
        cl = self.cloud
        if cl is None:
            return None
        epoch = cl.epoch
        reset = last is None or last[0] != epoch
        ver = cl.base if reset else int(last[1])
        if not reset and ver >= cl.version:
            return None
        ck = (epoch, ver, int(cap), reset, min(cl.version, ver + int(cap)))    # the same slice of the same cloud
        hit = self._chunks.get(ck)
        if hit is not None:
            return hit
        ver_to, xyz, rgb = cl.since(ver, int(cap))
        d = dict(type="cloud", t=round(time.time(), 3), frame=cl.frame, ver=int(ver_to), reset=bool(reset),
                 n=int(len(xyz)), total=int(cl.count), full=bool(cl.full), voxel_mm=cl.voxel,
                 xyz=base64.b64encode(np.ascontiguousarray(xyz, "<i2").tobytes()).decode("ascii"),
                 rgb=base64.b64encode(np.ascontiguousarray(rgb, np.uint8).tobytes()).decode("ascii")
                 if rgb is not None else None)
        out = ((epoch, int(ver_to)), _dumps(d))
        if len(self._chunks) > 32:
            self._chunks.clear()
        self._chunks[ck] = out
        return out

    def cloud_since(self, ver, cap: int = 8000) -> list:
        """BRAIN_SPEC 8.3's name: [(ver, text)] -- the next chunk after `ver` ((epoch, ver) state), at most one."""
        nxt = self.cloud_next(ver, cap)
        return [] if nxt is None else [nxt]

    def clear_cloud(self) -> None:
        if self.cloud is not None:
            self.cloud.clear(self.cloud.frame)
        self._chunks = {}
