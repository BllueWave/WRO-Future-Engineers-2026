"""Occupancy grid, a correlative scan matcher and a light SLAM (docs/BRAIN_SPEC.md section 6).

    g = GridMap(res_mm=20.0, size=512)                 # log-odds, float32 [iy, ix], 10.24 m square, 1 MB
    g.integrate(pose, X, Y, sensor_xy)                 # one scan at a known pose (the field map, SLAM's key scans)
    pose, score, info = match(g, X, Y, prior)          # the pose that puts the scan on the map's walls
    s = Slam(cfg); s.step(X, Y, t_scan, yaw_deg)       # predict (gyro + constant velocity) -> match -> integrate

Frames: `pose` = (x, y, th) of the REAR AXLE in the grid's frame (mm, rad); X, Y = returns in the CAR frame (rear axle,
x forward, y left, mm); `sensor_xy` = the lidar in the car frame.  Grid indexing G[iy, ix]: cell (0, 0) has its lower-
left corner at `origin`; images are flipud (the top row is the highest y).

Why no encoder in the prior: the WLtoys build has none and the stock A1's board never reports its wheels, so the motion
prior is the gyro for heading and a constant velocity from the last two accepted poses -- never the command (a stalled
car with a command integrated its pose 17 m out of the field on 2026-09-23).  No loop closure: this maps a room or the
field for the console and for offline review; it is not a race localiser.

Measured (laptop, 2026-09-23, mock scans of the 3 x 3 m field, lidar at [152, 0], 264 returns; tests_console/
test_map.py asserts them; the laptop shared with other work, so ranges): integrate 0.45-1.0 ms, the matcher's
distance + score fields 1.1-1.9 ms (rebuilt every 5th scan at most), match (1458 poses x 120 points) 1.0-1.8 ms,
Slam.step median 1.0-2.3 ms wall, 1.3-2.4 ms thread CPU.  Matching a scan 100-150 mm / 3-6 deg off its prior: <= 9
mm / 0.6 deg.  A 7.5 m lap at 0.4 m/s offline: pose error median 5-16 mm, heading 0.1-0.8 deg; the same lap driven
through the agent: 14-17 mm, 0.6-0.9 deg, 95 % of the occupied cells within 30 mm of a true wall.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time

import numpy as np

L_OCC, L_FREE = 0.8, -0.8                          # |log-odds| past which a cell is occupied / free
T_OCC, T_FREE, T_UNK = 0, 254, 205                 # the trinary image (ROS map_server's grey levels)
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def maps_dir() -> str:
    """runs/maps of the agent (BW_RUNS when set, as agent.RUN_DIR), created on first use by save()."""
    return os.path.join(os.environ.get("BW_RUNS") or os.path.join(ROOT, "runs"), "maps")


def safe_name(name: str) -> str:
    """A map name for a file: 1-40 of A-Z a-z 0-9 _ . - (anything else becomes _); '' stays ''."""
    s = "".join(ch if ch.isalnum() or ch in "_.-" else "_" for ch in str(name or ""))[:40].strip(".")
    return s


def unique_base(name: str, d: str | None = None) -> str:
    """maps_dir()/name, or name-2, name-3 ... when a saved map already has that name: a save never replaces a map."""
    d = d or maps_dir()
    base, k = os.path.join(d, name), 2
    while os.path.exists(base + ".json") or os.path.exists(base + ".npy"):
        base = os.path.join(d, "%s-%d" % (name, k))
        k += 1
    return base


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class GridMap:
    """Log-odds occupancy, float32 [iy, ix], bounded: size x size cells of res_mm (512 x 20 mm = 10.24 m, 1 MB).

    A cell is occupied at L >= 0.8 (one hit: 0.85), free at L <= -0.8 (two misses), unknown between; |L| <= clamp so
    a moved object is forgotten after a few scans instead of never."""

    def __init__(self, res_mm=20.0, size=512, origin_mm=None, frame="map", hit=0.85, miss=-0.4, clamp=4.0):
        self.res = float(res_mm)
        self.n = int(size)
        if origin_mm is None:
            origin_mm = (-self.n * self.res / 2.0, -self.n * self.res / 2.0)
        self.origin = (float(origin_mm[0]), float(origin_mm[1]))
        self.frame = str(frame)
        self.hit, self.miss, self.clamp = float(hit), float(miss), float(clamp)
        self.L = np.zeros((self.n, self.n), np.float32)
        self.version = 0                            # +1 per integrate
        self.scans = 0
        self.lock = threading.Lock()
        self._box = None                            # [i_min, i_max, j_min, j_max] touched, inclusive
        self._dist = None                           # (version, i0, j0, D crop) -- likelihood cache
        self._score = None                          # (version, sigma, i0, j0, S, S2) -- the matcher's fields

    # ------------------------------------------------------------------ cells
    def cell(self, x, y):
        """Field / map mm -> (iy, ix) int arrays (floor), possibly outside the grid."""
        ix = np.floor((np.asarray(x, float) - self.origin[0]) / self.res).astype(np.int64)
        iy = np.floor((np.asarray(y, float) - self.origin[1]) / self.res).astype(np.int64)
        return iy, ix

    def centre(self, iy, ix):
        """(x, y) mm of cell centres."""
        return (self.origin[0] + (np.asarray(ix, float) + 0.5) * self.res,
                self.origin[1] + (np.asarray(iy, float) + 0.5) * self.res)

    # ------------------------------------------------------------------ integrate
    def integrate(self, pose, X, Y, sensor_xy=(0.0, 0.0), max_mm=4000.0) -> None:
        """One scan: every beam's cells from the sensor up to one cell before its return get `miss` ONCE per scan (a
        unique mask: near the sensor many beams cross one cell), its end cell `hit` (once, and never also `miss`);
        beams longer than max_mm clear up to max_mm and leave no hit.  Vectorised over all beams' samples (np.repeat:
        no rectangular beams x samples array, so a scan of short beams costs little)."""
        X = np.asarray(X, np.float64).ravel()
        Y = np.asarray(Y, np.float64).ravel()
        if not len(X):
            return
        x, y, th = (float(v) for v in pose[:3])
        c, s = math.cos(th), math.sin(th)
        px, py = float(sensor_xy[0]), float(sensor_xy[1])
        gsx, gsy = x + c * px - s * py, y + s * px + c * py
        dx = x + c * X - s * Y - gsx
        dy = y + s * X + c * Y - gsy
        r = np.hypot(dx, dy)
        ok = r > 1.0
        dx, dy, r = dx[ok], dy[ok], r[ok]
        if not len(r):
            return
        ux, uy = dx / r, dy / r
        res = self.res
        has_hit = r <= max_mm
        # grazing beams: a beam meeting a surface at incidence a runs within one cell of it for the last res / sin(a)
        # of its path, so clearing up to "one cell before the return" erodes the surface's cells ahead of the car and
        # the wall drifts outward there (a 0.3 deg heading bias in the matcher, simulator 2026-09-23).  The surface
        # direction comes from the neighbouring return in scan order (the nearer of the two, when it is on the same
        # surface: <= max(60 mm, 35 % of the range) away -- at 1 deg bins consecutive returns on a wall met at 3 deg
        # are 0.33 r apart); the free part stops res / sin(a) before the return, capped at 8 cells and half the beam.
        # Lap in the simulator: pose error median 16 -> 13 mm, map cells > 40 mm off the true walls 24 -> 0.
        stop = np.full(len(r), res)
        if len(r) >= 3:
            ex, ey = dx, dy
            fx_, fy_ = np.roll(ex, -1) - ex, np.roll(ey, -1) - ey          # to the next return
            bx_, by_ = ex - np.roll(ex, 1), ey - np.roll(ey, 1)            # from the previous one
            lf, lb = np.hypot(fx_, fy_), np.hypot(bx_, by_)
            use_f = lf <= lb
            tx, ty = np.where(use_f, fx_, bx_), np.where(use_f, fy_, by_)
            tl = np.where(use_f, lf, lb)
            near = (tl > 1e-6) & (tl <= np.maximum(60.0, 0.35 * r))
            sin_a = np.abs(ux * ty - uy * tx) / np.maximum(tl, 1e-6)
            k = np.clip(1.0 / np.maximum(sin_a, 1e-3), 1.0, 8.0)
            stop = np.where(near, np.minimum(res * k, 0.5 * r), res)
        free_len = np.clip(np.where(has_hit, r - np.maximum(stop, res), max_mm), 0.0, max_mm)
        step = res                                            # one sample per cell length: a corner-cut cell stays
        k = (free_len // step).astype(np.int64) + 1           #   unknown for this scan, the next scans fill it
        tot = int(k.sum())
        beam = np.repeat(np.arange(len(k)), k)
        first = np.repeat(np.cumsum(k) - k, k)
        t = (np.arange(tot) - first) * step
        sx = gsx + ux[beam] * t
        sy = gsy + uy[beam] * t
        fiy, fix = self.cell(sx, sy)
        hx, hy = gsx + dx[has_hit], gsy + dy[has_hit]
        hiy, hix = self.cell(hx, hy)
        n = self.n
        inf = (fix >= 0) & (fix < n) & (fiy >= 0) & (fiy < n)
        inh = (hix >= 0) & (hix < n) & (hiy >= 0) & (hiy < n)
        fix, fiy, hix, hiy = fix[inf], fiy[inf], hix[inh], hiy[inh]
        if not len(fix) and not len(hix):
            return
        allx = np.concatenate([fix, hix])
        ally = np.concatenate([fiy, hiy])
        j0, j1 = int(allx.min()), int(allx.max())
        i0, i1 = int(ally.min()), int(ally.max())
        h, w = i1 - i0 + 1, j1 - j0 + 1
        fm = np.zeros((h, w), bool)
        fm[fiy - i0, fix - j0] = True
        hm = np.zeros((h, w), bool)
        hm[hiy - i0, hix - j0] = True
        fm &= ~hm
        with self.lock:
            sub = self.L[i0:i1 + 1, j0:j1 + 1]                # a view: the masked updates land in L
            sub[fm] += np.float32(self.miss)
            sub[hm] += np.float32(self.hit)
            np.clip(sub, -self.clamp, self.clamp, out=sub)
            b = self._box
            self._box = [i0, i1, j0, j1] if b is None else [min(b[0], i0), max(b[1], i1), min(b[2], j0), max(b[3], j1)]
            self.version += 1
            self.scans += 1

    # ------------------------------------------------------------------ read
    def bbox(self, pad_cells=50) -> tuple:
        """(i0, j0, h, w) of the touched area + padding, clipped to the grid; nothing touched: a pad box at the
        centre."""
        n = self.n
        b = self._box
        if b is None:
            c = n // 2
            b = [c, c, c, c]
        i0, j0 = max(0, b[0] - pad_cells), max(0, b[2] - pad_cells)
        i1, j1 = min(n - 1, b[1] + pad_cells), min(n - 1, b[3] + pad_cells)
        return i0, j0, i1 - i0 + 1, j1 - j0 + 1

    @staticmethod
    def _tri(L) -> np.ndarray:
        T = np.full(L.shape, T_UNK, np.uint8)
        T[L >= L_OCC] = T_OCC
        T[L <= L_FREE] = T_FREE
        return T

    def trinary(self) -> np.ndarray:
        """uint8 [iy, ix]: 0 occupied (L >= 0.8), 254 free (L <= -0.8), 205 unknown."""
        with self.lock:
            L = self.L.copy()
        return self._tri(L)

    def known_m2(self) -> float:
        """Known (free or occupied) area, m^2."""
        with self.lock:
            i0, j0, h, w = self.bbox(0)
            sub = self.L[i0:i0 + h, j0:j0 + w]
            k = int(np.count_nonzero((sub >= L_OCC) | (sub <= L_FREE))) if self._box is not None else 0
        return k * self.res * self.res / 1e6

    def _crop(self, crop: bool, scale: int):
        """(i0, j0, h, w, L copy) under the lock; h and w even for scale 2 (grown by one cell, or trimmed at the
        border)."""
        with self.lock:
            i0, j0, h, w = self.bbox() if crop else (0, 0, self.n, self.n)
            if scale == 2:
                if h % 2:
                    if i0 + h < self.n:
                        h += 1
                    elif i0 > 0:
                        i0 -= 1
                        h += 1
                    else:
                        h -= 1
                if w % 2:
                    if j0 + w < self.n:
                        w += 1
                    elif j0 > 0:
                        j0 -= 1
                        w += 1
                    else:
                        w -= 1
            return i0, j0, h, w, self.L[i0:i0 + h, j0:j0 + w].copy(), self.version

    def meta(self, crop=None, scale=1) -> dict:
        return dict(res_mm=self.res, n=self.n, origin_mm=[round(self.origin[0], 3), round(self.origin[1], 3)],
                    crop=list(crop) if crop is not None else list(self.bbox()), scale=int(scale), frame=self.frame,
                    v=int(self.version), scans=int(self.scans))

    def png(self, crop=True, scale=1) -> tuple:
        """(PNG bytes, meta): flipud(trinary) of the crop; scale 2 = 2 x 2 cells per pixel, min-pooled (occupied beats
        unknown beats free).  PNG pixel (row r, col c) = cell (iy = i0 + h - 1 - r, ix = j0 + c) at scale 1."""
        import cv2
        scale = 2 if int(scale) == 2 else 1
        i0, j0, h, w, L, v = self._crop(crop, scale)
        img = np.flipud(self._tri(L))
        if scale == 2 and h >= 2 and w >= 2:
            img = img.reshape(h // 2, 2, w // 2, 2).min(axis=(1, 3))
        ok, buf = cv2.imencode(".png", np.ascontiguousarray(img), [cv2.IMWRITE_PNG_COMPRESSION, 6])
        meta = self.meta((i0, j0, h, w), scale)
        meta["v"] = int(v)
        return (buf.tobytes() if ok else b""), meta

    def ros(self, crop=True) -> tuple:
        """(meta, int8 bytes) for nav_msgs/OccupancyGrid: -1 unknown / 0 free / 100 occupied, row-major, row 0 = the
        LOWEST y (no flip).  meta: res_mm, width, height, origin_mm (the crop's lower-left corner), frame, v."""
        i0, j0, h, w, L, v = self._crop(crop, 1)
        out = np.full(L.shape, -1, np.int8)
        out[L >= L_OCC] = 100
        out[L <= L_FREE] = 0
        meta = dict(res_mm=self.res, width=int(w), height=int(h),
                    origin_mm=[round(self.origin[0] + j0 * self.res, 3), round(self.origin[1] + i0 * self.res, 3)],
                    frame=self.frame, v=int(v), crop=[i0, j0, h, w])
        return meta, out.tobytes()

    # ------------------------------------------------------------------ distance fields
    def _dist_crop(self):
        """(i0, j0, D): mm from each cell of the padded touched box to the nearest occupied cell (cv2
        distanceTransform, exact L2), cached per version; D = inf everywhere when nothing is occupied."""
        import cv2
        with self.lock:
            dc = self._dist
            if dc is not None and dc[0] == self.version:
                return dc[1], dc[2], dc[3]
            i0, j0, h, w = self.bbox(50)
            occ = self.L[i0:i0 + h, j0:j0 + w] >= L_OCC
            v = self.version
        if occ.any():
            src = np.where(occ, 0, 255).astype(np.uint8)
            D = cv2.distanceTransform(src, cv2.DIST_L2, cv2.DIST_MASK_PRECISE).astype(np.float32) * np.float32(self.res)
        else:
            D = np.full(occ.shape, np.inf, np.float32)
        with self.lock:
            self._dist = (v, i0, j0, D)
        return i0, j0, D

    def likelihood(self) -> np.ndarray:
        """float32 [iy, ix] mm to the nearest occupied cell, cached per version (cells beyond the padded touched box
        are > 1 m from anything: inf there)."""
        i0, j0, D = self._dist_crop()
        out = np.full((self.n, self.n), np.inf, np.float32)
        out[i0:i0 + D.shape[0], j0:j0 + D.shape[1]] = D
        return out

    def score_field(self, sigma_mm=30.0):
        """(i0, j0, S, S2): exp(-d^2 / 2 sigma^2) of the distance field over the padded box, with a one-cell zero
        border (S[1 + iy - i0, 1 + ix - j0]; an index clipped to the border scores 0), and S2 = the same upsampled 2x
        (bilinear, 10 mm cells at res 20, one-cell zero border) for the matcher's fine stage: one gather per point
        instead of a four-cell bilinear (fine stage 2 -> 0.4 ms, laptop).  Cached per (version, sigma)."""
        import cv2
        sc = self._score
        if sc is not None and sc[0] == self.version and sc[1] == sigma_mm:
            return sc[2:]
        v = self.version
        i0, j0, D = self._dist_crop()
        S = np.zeros((D.shape[0] + 2, D.shape[1] + 2), np.float32)
        with np.errstate(over="ignore", invalid="ignore"):
            S[1:-1, 1:-1] = np.exp(-(np.minimum(D, 1e4) ** 2) / np.float32(2.0 * sigma_mm * sigma_mm))
        h, w = D.shape
        S2 = np.zeros((2 * h + 2, 2 * w + 2), np.float32)
        S2[1:-1, 1:-1] = cv2.resize(S[1:-1, 1:-1], (2 * w, 2 * h), interpolation=cv2.INTER_LINEAR)
        self._score = (v, sigma_mm, i0, j0, S, S2)
        return i0, j0, S, S2

    # ------------------------------------------------------------------ files
    def save(self, base: str, extra: dict | None = None) -> list:
        """base.png (trinary of the touched box, top row = max y), base.json (meta: res, n, origin, crop, frame, v,
        scans, known_m2, saved), base.npy (the log-odds of the crop, float32).  Returns the three paths."""
        import cv2
        d = os.path.dirname(os.path.abspath(base))
        os.makedirs(d, exist_ok=True)
        i0, j0, h, w, L, v = self._crop(True, 1)
        ok, buf = cv2.imencode(".png", np.ascontiguousarray(np.flipud(self._tri(L))))
        meta = self.meta((i0, j0, h, w), 1)
        meta.update(v=int(v), known_m2=round(self.known_m2(), 3), saved=round(time.time(), 3),
                    hit=self.hit, miss=self.miss, clamp=self.clamp)
        if extra:
            meta["extra"] = extra
        paths = [base + ".png", base + ".json", base + ".npy"]
        with open(paths[0], "wb") as f:
            f.write(buf.tobytes() if ok else b"")
        with open(paths[1], "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=1)
        np.save(paths[2], L)
        return paths

    @classmethod
    def load(cls, base: str) -> "GridMap":
        if base.endswith((".json", ".png", ".npy")):
            base = base[:-4] if not base.endswith(".json") else base[:-5]
        with open(base + ".json", encoding="utf-8") as f:
            m = json.load(f)
        g = cls(m["res_mm"], m["n"], m["origin_mm"], m.get("frame", "map"), m.get("hit", 0.85), m.get("miss", -0.4),
                m.get("clamp", 4.0))
        i0, j0, h, w = m["crop"]
        L = np.load(base + ".npy").astype(np.float32)
        g.L[i0:i0 + h, j0:j0 + w] = L[:h, :w]
        nz = np.nonzero(np.abs(g.L) > 1e-6)
        if len(nz[0]):
            g._box = [int(nz[0].min()), int(nz[0].max()), int(nz[1].min()), int(nz[1].max())]
        g.version, g.scans = int(m.get("v", 1)), int(m.get("scans", 0))
        return g


# ------------------------------------------------------------------ de-skew
def deskew(X, Y, sensor_xy, rpm, offset_deg, cw, lag_s, v_mmps, w_radps):
    """Car-frame returns of one revolution -> the car frame at the scan's mean time t - lag, each moved by the car's
    motion between its own bin's time and that reference.  A bin's time is t - (1 - raw / 360) * 60 / rpm: the
    revolution ends at the raw wrap (lidar.ScanAssembler), raw = offset_deg -+ its bearing from the lidar (cw).
    v_mmps = (forward, left) of the rear axle, w_radps the yaw rate (the gyro).  At 100 deg/s a revolution spans 10 deg:
    uncorrected, a wall 1.5 m away bends by up to 26 mm across the scan."""
    X = np.asarray(X, np.float64)
    Y = np.asarray(Y, np.float64)
    if not len(X) or (abs(v_mmps[0]) + abs(v_mmps[1]) < 1.0 and abs(w_radps) < 1e-3):
        return X, Y
    T = 60.0 / rpm if rpm and rpm > 1.0 else 0.1
    a = np.degrees(np.arctan2(Y - sensor_xy[1], X - sensor_xy[0]))
    raw = (float(offset_deg) + (-a if cw else a)) % 360.0
    dt = (1.0 - raw / 360.0) * T - float(lag_s)                      # t_ref - t_bin: > 0 = the bin is older
    dth = float(w_radps) * dt
    c2, s2 = np.cos(dth / 2.0), np.sin(dth / 2.0)
    dx = (c2 * v_mmps[0] - s2 * v_mmps[1]) * dt
    dy = (s2 * v_mmps[0] + c2 * v_mmps[1]) * dt
    c, s = np.cos(dth), np.sin(dth)
    Xs, Ys = X - dx, Y - dy
    return c * Xs + s * Ys, -s * Xs + c * Ys


# ------------------------------------------------------------------ the correlative scan matcher
def subsample(X, Y, max_pts=120):
    """<= max_pts points evenly along the scan order."""
    X = np.asarray(X, np.float64).ravel()
    Y = np.asarray(Y, np.float64).ravel()
    if len(X) > max_pts:
        k = np.linspace(0, len(X) - 1, max_pts).astype(np.int64)
        X, Y = X[k], Y[k]
    return X, Y


def _eval(field, grid, X, Y, x0, y0, ths, offs_x, offs_y, bilinear=False):
    """Scores (len(ths), len(offs_y), len(offs_x)) of the rear-axle poses (x0 + ox, y0 + oy, th): the mean of the
    score field at every point: the nearest cell (coarse), or the nearest cell of the 2x-upsampled field (fine: a
    20 mm cell would otherwise quantise a 10 mm step; the upsampling is bilinear, so this IS the bilinear value at
    10 mm resolution)."""
    i0, j0, S = field[:3]
    k = 1.0
    if bilinear and len(field) > 3:
        S, k = field[3], 2.0
    H, W = S.shape
    res, (ox, oy) = grid.res, grid.origin
    c, s = np.cos(ths)[:, None], np.sin(ths)[:, None]
    fx = ((x0 + c * X[None, :] - s * Y[None, :] - ox) / res - j0) * k + 1.0     # (a, m) in the padded field
    fy = ((y0 + s * X[None, :] + c * Y[None, :] - oy) / res - i0) * k + 1.0
    kx = np.asarray(offs_x, float) / res * k
    ky = np.asarray(offs_y, float) / res * k
    jx = np.clip(np.floor(fx[:, None, :] + kx[None, :, None]).astype(np.int64), 0, W - 1)    # (a, kx, m)
    jy = np.clip(np.floor(fy[:, None, :] + ky[None, :, None]).astype(np.int64), 0, H - 1)    # (a, ky, m)
    idx = jy[:, :, None, :] * W + jx[:, None, :, :]                                          # (a, ky, kx, m)
    return S.ravel()[idx].mean(axis=3)


def _vertex(lo, mid, hi) -> float:
    """The parabola vertex through (-1, lo), (0, mid), (1, hi), in steps, clamped to +-0.5; 0 at an edge."""
    if lo is None or hi is None:
        return 0.0
    den = float(lo) - 2.0 * float(mid) + float(hi)
    if den >= -1e-12:
        return 0.0
    return max(-0.5, min(0.5, 0.5 * (float(lo) - float(hi)) / den))


def match(grid: GridMap, X, Y, prior, win_xy=160.0, win_th_deg=6.0, sigma_mm=30.0, max_pts=120, prior_k=0.02,
          field=None, gyro_k=0.0, gyro_deg=1.0):
    """(pose, score, info).  Points in the car frame (<= max_pts, evenly subsampled); coarse 40 mm / 1.5 deg over the
    window, fine 10 mm / 0.4 deg over +-40 mm / +-1.6 deg around the best; score = mean exp(-d^2 / 2 sigma^2) of the
    likelihood-field distances, bilinear in the fine stage, then a parabola's vertex on each axis.  The objective adds
    two small pulls to the prior: prior_k x the squared window fraction (breaks ties where the scan does not constrain
    a direction -- a corridor's two parallel walls -- so the match stays at the prediction instead of sliding), and
    gyro_k x (dth / gyro_deg)^2 (the gyro's heading change over 0.1 s is better than a scan's heading on a sparse map:
    lap in the simulator 0.82 -> 0.72 deg median with gyro_k 0.005).  info = dict(n_eval, ms, second_ratio, edge):
    second_ratio = the best coarse score >= 80 mm / 3 deg away / the best (near 1 = ambiguous); edge = the result lies
    beyond the window (the truth may be outside it).  `field` = a score_field() result to use instead of the grid's
    current one (Slam keeps an older one)."""
    t0 = time.perf_counter()
    X, Y = subsample(X, Y, max_pts)
    x0, y0, th0 = (float(v) for v in prior[:3])
    if len(X) < 3:
        return (x0, y0, th0), 0.0, dict(n_eval=0, ms=0.0, second_ratio=1.0, edge=False)
    if field is None:
        field = grid.score_field(sigma_mm)
    wt = math.radians(win_th_deg)
    cs_xy = 40.0
    offs = np.arange(-win_xy, win_xy + 1e-6, cs_xy)
    ths = th0 + np.radians(np.arange(-win_th_deg, win_th_deg + 1e-6, 1.5))
    sc = _eval(field, grid, X, Y, x0, y0, ths, offs, offs)                  # (a, y, x)
    gr = math.radians(gyro_deg)
    pa = prior_k * ((ths - th0) / wt) ** 2 + gyro_k * ((ths - th0) / gr) ** 2
    po = prior_k * (offs / win_xy) ** 2
    obj = sc - (pa[:, None, None] + po[None, :, None] + po[None, None, :])
    a, iy, ix = np.unravel_index(int(np.argmax(obj)), obj.shape)
    n_eval = sc.size
    best_c = float(sc[a, iy, ix])
    far = (np.abs(offs[None, :, None] - offs[iy]) >= 80.0) | (np.abs(offs[None, None, :] - offs[ix]) >= 80.0) | \
          (np.abs(np.degrees(ths - ths[a]))[:, None, None] >= 3.0)
    second = float(sc[far].max()) if far.any() else 0.0
    bx, by, bth = x0 + offs[ix], y0 + offs[iy], float(ths[a])
    # fine: 10 mm / 0.4 deg over +-40 mm / +-1.6 deg around the coarse best
    fo = np.arange(-40.0, 40.0 + 1e-6, 10.0)
    fth = bth + np.radians(np.arange(-1.6, 1.6 + 1e-6, 0.4))
    sf = _eval(field, grid, X, Y, bx, by, fth, fo, fo, bilinear=True)
    pa = prior_k * ((fth - th0) / wt) ** 2 + gyro_k * ((fth - th0) / gr) ** 2
    pox = prior_k * ((bx + fo - x0) / win_xy) ** 2
    poy = prior_k * ((by + fo - y0) / win_xy) ** 2
    objf = sf - (pa[:, None, None] + poy[None, :, None] + pox[None, None, :])
    a2, iy2, ix2 = np.unravel_index(int(np.argmax(objf)), objf.shape)
    n_eval += sf.size
    score = float(sf[a2, iy2, ix2])
    # sub-step: a parabola through the best and its two neighbours on each axis (the fine grid is 10 mm / 0.4 deg;
    # the vertex moves at most half a step)
    ddx = _vertex(objf[a2, iy2, ix2 - 1] if ix2 > 0 else None, objf[a2, iy2, ix2],
                  objf[a2, iy2, ix2 + 1] if ix2 + 1 < len(fo) else None)
    ddy = _vertex(objf[a2, iy2 - 1, ix2] if iy2 > 0 else None, objf[a2, iy2, ix2],
                  objf[a2, iy2 + 1, ix2] if iy2 + 1 < len(fo) else None)
    dda = _vertex(objf[a2 - 1, iy2, ix2] if a2 > 0 else None, objf[a2, iy2, ix2],
                  objf[a2 + 1, iy2, ix2] if a2 + 1 < len(fth) else None)
    pose = (bx + fo[ix2] + 10.0 * ddx, by + fo[iy2] + 10.0 * ddy, float(_wrap(fth[a2] + math.radians(0.4) * dda)))
    edge = abs(pose[0] - x0) > win_xy + 1e-6 or abs(pose[1] - y0) > win_xy + 1e-6 or \
        abs(_wrap(pose[2] - th0)) > wt + 1e-9
    info = dict(n_eval=int(n_eval), ms=round((time.perf_counter() - t0) * 1000.0, 3),
                second_ratio=round(second / best_c, 3) if best_c > 1e-9 else 1.0, edge=bool(edge))
    return pose, score, info


# ------------------------------------------------------------------ SLAM
SLAM_DEFAULTS = dict(res_mm=20.0, size=512, max_mm=6000.0, min_score=0.35, win_xy=160.0, win_th=6.0, key_mm=40.0,
                     key_deg=2.0, sensor_xy=(0.0, 0.0), sigma_mm=30.0, max_pts=120, field_every=5, init_scans=3,
                     gyro_k=0.005, gyro_deg=1.0)


class Slam:
    """No encoder: the motion prior is the gyro for heading and a constant velocity from the last two accepted poses
    (decaying 0.5 per second of silence), not the command.

    step(): predict -> the first scan integrates at the prediction (the map is empty) -> then match; accept when
    score >= min_score and the correction lies inside the window; otherwise keep the prediction and count `weak`;
    integrate an accepted scan when the pose moved >= key_mm or turned >= key_deg since the last integration, or every
    10th scan, and always until init_scans are in.  The matcher's distance field is rebuilt at most every
    field_every-th scan.

    Why only ONE scan at the prediction (BRAIN_SPEC 6.2 said three): with no accepted pair yet the velocity is 0, so a
    car already rolling when SLAM starts put scans 2-3 40-80 mm off (400 mm/s, simulator 2026-09-23) and every later
    match inherited that smear; matched against the first scan they land where they belong."""

    def __init__(self, cfg: dict | None = None):
        c = dict(SLAM_DEFAULTS, **(cfg or {}))
        self.cfg = c
        self.grid = GridMap(c["res_mm"], int(c["size"]), None, "map")
        self.pose = (0.0, 0.0, 0.0)
        self.sensor = (float(c["sensor_xy"][0]), float(c["sensor_xy"][1]))
        self.weak = 0                       # scans not accepted (cumulative)
        self.weak_run = 0                   # ... in a row
        self.n = 0                          # scans stepped
        self.accepted = 0
        self.integrated = 0
        self._t = None                      # the previous step's scan time
        self._yaw = None                    # ... and gyro yaw (deg)
        self._acc = []                      # [(t, pose)] of the last two accepted poses
        self._v = (0.0, 0.0)                # car-frame velocity mm/s (forward, left)
        self._key = None                    # pose at the last integration
        self._since_key = 0
        self._field = None                  # the matcher's field (i0, j0, S), and when it was built
        self._field_n = -10**9
        self._field_v = -1
        self.cpu_s = 0.0                    # thread CPU time spent in step(), for the cost per scan

    def _predict(self, t_scan: float, yaw_deg: float):
        x, y, th = self.pose
        if self._t is None:
            return x, y, th
        dt = max(0.0, t_scan - self._t)
        dth = math.radians(((yaw_deg - self._yaw + 180.0) % 360.0) - 180.0)
        silence = max(0.0, t_scan - self._acc[-1][0] - 0.15) if self._acc else 0.0
        k = 0.5 ** silence
        vx, vy = self._v[0] * k, self._v[1] * k
        tm = th + dth / 2.0
        c, s = math.cos(tm), math.sin(tm)
        return x + (c * vx - s * vy) * dt, y + (s * vx + c * vy) * dt, _wrap(th + dth)

    def _accept(self, t_scan: float, pose):
        self._acc.append((t_scan, pose))
        if len(self._acc) > 2:
            self._acc.pop(0)
        if len(self._acc) == 2:
            (ta, pa), (tb, pb) = self._acc
            dt = tb - ta
            if 0.02 < dt < 1.0:
                dth = _wrap(pb[2] - pa[2])
                tm = pa[2] + dth / 2.0
                c, s = math.cos(tm), math.sin(tm)
                ddx, ddy = pb[0] - pa[0], pb[1] - pa[1]
                self._v = ((c * ddx + s * ddy) / dt, (-s * ddx + c * ddy) / dt)

    def predict(self, t: float, yaw_deg: float):
        """The pose at time t (after the last scan), dead-reckoned from it with the gyro's yaw and the constant
        velocity: what the console shows between scans.  The state is not changed."""
        return self._predict(t, yaw_deg) if self._t is not None and t >= self._t else self.pose

    def velocity(self, t: float):
        """(forward, left) mm/s of the constant-velocity prior at time t (for deskew), decayed like the prediction."""
        silence = max(0.0, t - self._acc[-1][0] - 0.15) if self._acc else 0.0
        k = 0.5 ** silence
        return self._v[0] * k, self._v[1] * k

    def _integrate(self, X, Y):
        self.grid.integrate(self.pose, X, Y, self.sensor, float(self.cfg["max_mm"]))
        self._key = self.pose
        self._since_key = 0
        self.integrated += 1

    def step(self, X, Y, t_scan: float, yaw_deg: float) -> dict:
        """One scan (car-frame returns, the scan's time, the gyro yaw at that time) -> dict(pose, score, accepted,
        integrated, ms, cpu_ms, n, second_ratio)."""
        t0, c0 = time.perf_counter(), time.thread_time()
        c = self.cfg
        X = np.asarray(X, np.float64).ravel()
        Y = np.asarray(Y, np.float64).ravel()
        pred = self._predict(t_scan, yaw_deg)
        self._t, self._yaw = t_scan, yaw_deg
        self.n += 1
        self._since_key += 1
        out = dict(score=None, accepted=False, integrated=False, second_ratio=None, n=int(len(X)))
        if self.grid.version == 0:
            # the map is empty: this scan IS the map's first content, at the prediction (the origin for the first)
            self.pose = pred
            self._accept(t_scan, pred)
            self.accepted += 1
            if len(X):
                self._integrate(X, Y)
                out["integrated"] = True
            out["accepted"] = True
        elif len(X) < 20:
            self.pose = pred
            self.weak += 1
            self.weak_run += 1
        else:
            if self._field is None or (self.n - self._field_n >= int(c["field_every"])
                                       and self.grid.version != self._field_v):
                self._field = self.grid.score_field(float(c["sigma_mm"]))
                self._field_n, self._field_v = self.n, self.grid.version
            pose, score, info = match(self.grid, X, Y, pred, float(c["win_xy"]), float(c["win_th"]),
                                      float(c["sigma_mm"]), int(c["max_pts"]), field=self._field,
                                      gyro_k=float(c["gyro_k"]), gyro_deg=float(c["gyro_deg"]))
            out["score"], out["second_ratio"] = round(score, 3), info["second_ratio"]
            if score >= float(c["min_score"]) and not info["edge"]:
                self.pose = pose
                self._accept(t_scan, pose)
                self.accepted += 1
                self.weak_run = 0
                out["accepted"] = True
                kp = self._key
                moved = kp is None or math.hypot(pose[0] - kp[0], pose[1] - kp[1]) >= float(c["key_mm"]) or \
                    abs(math.degrees(_wrap(pose[2] - kp[2]))) >= float(c["key_deg"]) or self._since_key >= 10
                if moved or self.integrated < int(c["init_scans"]):    # the first few matched scans densify the
                    #                                                     start map (one scan is noisy and sparse)
                    self._integrate(X, Y)
                    out["integrated"] = True
            else:
                self.pose = pred
                self.weak += 1
                self.weak_run += 1
        dc = time.thread_time() - c0
        self.cpu_s += dc
        out.update(pose=self.pose, ms=round((time.perf_counter() - t0) * 1000.0, 3), cpu_ms=round(dc * 1000.0, 3))
        return out
