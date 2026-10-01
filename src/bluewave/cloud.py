"""The HP60C depth as a 3-D point cloud for the console's 3D view and for offline review (docs/BRAIN_SPEC.md section 7).

    c = Cloud(voxel_mm=25.0, max_points=200000, stride=8)
    c.add(depth_u16, t_exp, dg, pose, "field")      # one depth frame at a known pose -> the NEW voxels
    ver, xyz, rgb = c.since(ver)                     # what a client has not seen yet (int16 mm, <= cap points)
    c.ply()                                          # the whole cloud, binary PLY (any viewer: MeshLab, CloudCompare)

Nothing exists unless depth.on = 1 and a console subscribes to `cloud` (bluewave/mapper.py): zero cost otherwise.

Every `stride`-th pixel both ways (80 x 60 = 4800 samples at 640 x 480) -> x depth.scale -> range gate -> the car frame
through the camera's pose (depth.DepthGeom: registered depth = the RGB intrinsics and pose) -> the map / field frame
through the rear-axle pose at the EXPOSURE -> a 25 mm voxel key -> a Python set for membership; the point stored is the
voxel centre, so a surface seen from ten poses is stored once.  Frames: z up from the floor, mm.

What the real HP60C gave on the team's field (2026-09-22): no return from the white mat, most of the black walls
missed, the pillars seen -- so on the field the cloud is mostly the signs; in a room it is the room.
"""
from __future__ import annotations

import threading

import numpy as np

_B = 1 << 20                                   # voxel index offset: +-2^20 voxels of 25 mm = +-26 km, never reached
_M = (1 << 21) - 1


class Cloud:
    """HP60C depth frames -> voxel-filtered 3-D points in the field / map frame.  Nothing exists unless depth.on = 1
    and a console subscribes to `cloud`: zero cost otherwise.

    Points get a sequence number as they are added (`version` = the next one, monotonic across clears); `base` = the
    first one still stored.  since(ver) returns the points numbered >= ver, so a client that remembers the last `ver`
    it got continues exactly where it stopped; ver < base means the cloud was cleared (or the frame changed) since:
    the client starts over (`epoch` counts the clears)."""

    def __init__(self, voxel_mm=25.0, max_points=200000, stride=8, min_mm=200.0, max_mm=3000.0, rgb=False):
        self.voxel = float(voxel_mm)
        self.max_points = int(max_points)
        self.stride = max(1, int(stride))
        self.min_mm, self.max_mm = float(min_mm), float(max_mm)
        self.rgb = bool(rgb)
        self.lock = threading.Lock()
        self.frame = None                       # "field" | "map" | "car" | None (empty)
        self.version = 0
        self.base = 0
        self.epoch = 0
        self.count = 0
        self.full = False
        self.frames = 0                         # depth frames added
        self._keys = set()
        self._xyz = np.zeros((0, 3), np.int16)
        self._rgb = np.zeros((0, 3), np.uint8)

    # ------------------------------------------------------------------ store
    def clear(self, frame=None) -> None:
        with self.lock:
            self._clear(frame)

    def _clear(self, frame):
        self.frame = frame
        self.base = self.version
        self.epoch += 1
        self.count = 0
        self.full = False
        self._keys = set()
        self._xyz = np.zeros((0, 3), np.int16)
        self._rgb = np.zeros((0, 3), np.uint8)

    def _append(self, xyz: np.ndarray, rgb: np.ndarray):
        n = len(xyz)
        if self.count + n > len(self._xyz):
            cap = max(4096, 2 * len(self._xyz), self.count + n)
            nx = np.zeros((cap, 3), np.int16)
            nx[:self.count] = self._xyz[:self.count]
            nr = np.zeros((cap, 3), np.uint8)
            nr[:self.count] = self._rgb[:self.count]
            self._xyz, self._rgb = nx, nr
        self._xyz[self.count:self.count + n] = xyz
        self._rgb[self.count:self.count + n] = rgb
        self.count += n
        self.version += n

    # ------------------------------------------------------------------ add
    def points(self, depth_u16, dg, pose=None):
        """(P (3, k) float32 mm in the pose's frame -- the car frame for pose None --, (rows, cols) of each point)
        of the sampled pixels inside [min_mm, max_mm]."""
        d = np.asarray(depth_u16)
        H, W = d.shape[:2]
        s = self.stride
        rows = np.arange(s // 2, H, s)
        cols = np.arange(s // 2, W, s)
        z = d[np.ix_(rows, cols)].astype(np.float32) * np.float32(getattr(dg, "scale", 1.0))
        ok = (z >= self.min_mm) & (z <= self.max_mm)
        ri, ci = np.nonzero(ok)
        z = z[ri, ci]
        u, v = cols[ci].astype(np.float32), rows[ri].astype(np.float32)
        xp = (u - np.float32(dg.cx)) / np.float32(dg.fx)
        yp = (v - np.float32(dg.cy)) / np.float32(dg.fy)
        R = np.asarray(dg.R, np.float32)
        p = np.stack([xp * z, yp * z, z])                     # camera frame (x right, y down, z forward)
        P = R @ p + np.asarray(dg.pos, np.float32)[:, None]   # car frame, mm
        if pose is not None:
            x, y, th = (float(a) for a in pose[:3])
            c, s_ = np.float32(np.cos(th)), np.float32(np.sin(th))
            X = x + c * P[0] - s_ * P[1]
            Y = y + s_ * P[0] + c * P[1]
            P = np.stack([X, Y, P[2]]).astype(np.float32)
        return P, (rows[ri], cols[ci])

    def add(self, depth_u16, t_exp, dg, pose, frame, bgr=None) -> int:
        """One depth frame.  dg: depth.DepthGeom (the camera pose; registered depth = RGB intrinsics); pose (x, y, th)
        at t_exp in `frame`, or None -> the "car" frame, and then only this frame is kept (reset each call).  A frame
        change clears the cloud.  bgr: the RGB frame paired with it (colour, cloud.rgb = 1).  Returns the number of NEW
        voxels."""
        if depth_u16 is None:
            return 0
        P, (ri, ci) = self.points(depth_u16, dg, pose)
        if pose is None:
            frame = "car"
        vx = self.voxel
        k = np.floor(P / np.float32(vx)).astype(np.int64)                 # (3, m) voxel indices
        keys = (((k[0] + _B) & _M) << 42) | (((k[1] + _B) & _M) << 21) | ((k[2] + _B) & _M)
        keys, first = np.unique(keys, return_index=True)
        with self.lock:
            if frame != self.frame or pose is None:
                self._clear(frame)
            self.frames += 1
            if self.full:
                return 0
            room = self.max_points - self.count
            fresh = [i for i, kk in enumerate(keys.tolist()) if kk not in self._keys]
            if len(fresh) > room:
                fresh = fresh[:room]
                self.full = True
            if not fresh:
                return 0
            sel = first[np.asarray(fresh, np.int64)]
            self._keys.update(keys[np.asarray(fresh, np.int64)].tolist())
            ctr = np.clip((k[:, sel].T.astype(np.float64) + 0.5) * vx, -32767, 32767).astype(np.int16)
            if self.rgb and bgr is not None:
                bh, bw = bgr.shape[:2]
                dh, dw = np.asarray(depth_u16).shape[:2]
                rr = np.clip((ri[sel] * bh) // dh, 0, bh - 1)
                cc = np.clip((ci[sel] * bw) // dw, 0, bw - 1)
                col = bgr[rr, cc][:, ::-1]                                  # BGR -> RGB
            else:
                col = _height_rgb(ctr[:, 2])
            self._append(ctr, col.astype(np.uint8))
            if self.count >= self.max_points:
                self.full = True
            return len(fresh)

    # ------------------------------------------------------------------ read
    def since(self, ver, cap=8000) -> tuple:
        """(ver_to, xyz int16 (k, 3) mm, rgb uint8 (k, 3) | None): the points numbered >= ver (from `base` when ver
        is older), at most `cap`; ver_to = the number to ask from next time.  rgb is None unless cloud.rgb."""
        with self.lock:
            v0 = max(int(ver or 0), self.base)
            i0 = v0 - self.base
            i1 = min(self.count, i0 + int(cap))
            xyz = self._xyz[i0:i1].copy()
            rgb = self._rgb[i0:i1].copy() if self.rgb else None
            return self.base + max(i1, i0), xyz, rgb

    def snapshot(self):
        """(frame, xyz int16 (n, 3), rgb uint8 (n, 3)) of the whole cloud, copied."""
        with self.lock:
            return self.frame, self._xyz[:self.count].copy(), self._rgb[:self.count].copy()

    def ply(self) -> bytes:
        """Binary little-endian PLY: float32 x y z (m), uchar red green blue (the RGB frame's, or a height ramp)."""
        frame, xyz, rgb = self.snapshot()
        n = len(xyz)
        head = ("ply\nformat binary_little_endian 1.0\ncomment BlueWave HP60C cloud, frame %s, voxel %g mm, z up\n"
                "element vertex %d\nproperty float x\nproperty float y\nproperty float z\n"
                "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
                % (frame or "none", self.voxel, n)).encode("ascii")
        rec = np.zeros(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")])
        rec["x"], rec["y"], rec["z"] = xyz[:, 0] / 1000.0, xyz[:, 1] / 1000.0, xyz[:, 2] / 1000.0
        rec["r"], rec["g"], rec["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        return head + rec.tobytes()

    def render_png(self, view="top", size=800) -> bytes:
        """An orthographic picture for bw and the team: "top" = x right, y up; "side" = x right, z up.  1 m grid lines,
        2 px points coloured by height over this cloud's 5th-95th percentile (the field's 0-100 mm would all be one
        colour on a fixed ramp); the caption says the view, the frame and the count."""
        import cv2
        frame, xyz, rgb = self.snapshot()
        size = max(200, min(int(size), 2000))
        img = np.full((size, size, 3), 24, np.uint8)
        if len(xyz):
            a = xyz[:, 0].astype(np.float32)
            b = (xyz[:, 1] if view != "side" else xyz[:, 2]).astype(np.float32)
            lo_a, hi_a = float(np.percentile(a, 1)), float(np.percentile(a, 99))
            lo_b, hi_b = float(np.percentile(b, 1)), float(np.percentile(b, 99))
            span = max(hi_a - lo_a, hi_b - lo_b, 500.0) * 1.1
            ca, cb = (lo_a + hi_a) / 2.0, (lo_b + hi_b) / 2.0
            k = (size - 20) / span                                        # px per mm
            for g in np.arange(np.floor((ca - span / 2) / 1000.0), np.ceil((ca + span / 2) / 1000.0) + 1) * 1000.0:
                u = int(round(size / 2 + (g - ca) * k))
                if 0 <= u < size:
                    img[:, u] = 48
            for g in np.arange(np.floor((cb - span / 2) / 1000.0), np.ceil((cb + span / 2) / 1000.0) + 1) * 1000.0:
                v = int(round(size / 2 - (g - cb) * k))
                if 0 <= v < size:
                    img[v, :] = 48
            u = np.round(size / 2 + (a - ca) * k).astype(np.int64)
            v = np.round(size / 2 - (b - cb) * k).astype(np.int64)
            z = xyz[:, 2].astype(np.float32)
            z5, z95 = float(np.percentile(z, 5)), float(np.percentile(z, 95))
            col = _height_rgb((z - z5) / max(z95 - z5, 50.0) * 1000.0)   # the ramp over THIS cloud's heights
            order = np.argsort(z) if view != "side" else np.arange(len(xyz))          # the top wins from above
            for du in (0, 1):                                                          # 2 x 2 px dots
                for dv in (0, 1):
                    uu, vv = u + du, v + dv
                    ok = (uu >= 0) & (uu < size) & (vv >= 0) & (vv < size)
                    o = order[ok[order]]
                    img[vv[o], uu[o]] = col[o][:, ::-1]                               # RGB -> BGR for cv2
            cv2.putText(img, "%s  %s  %d pts  grid 1 m" % (view, frame or "-", len(xyz)), (8, size - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
        ok, buf = cv2.imencode(".png", img)
        return buf.tobytes() if ok else b""


def _height_rgb(z_mm) -> np.ndarray:
    """A blue -> green -> yellow -> red ramp over 0 .. 1000 mm of height (the field's walls and signs are 100 mm, a
    room's furniture up to ~1 m)."""
    t = np.clip(np.asarray(z_mm, np.float32) / 1000.0, 0.0, 1.0)
    r = np.clip(2.0 * t - 0.5, 0, 1)
    g = np.clip(1.5 - np.abs(3.0 * t - 1.5), 0, 1)
    b = np.clip(1.0 - 2.5 * t, 0, 1)
    return (np.stack([r, g, b], 1) * 255.0 + 0.5).astype(np.uint8)
