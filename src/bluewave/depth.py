"""The HP60C's depth on the host: ranges that do not depend on the camera pitch.

The RGB floor model (vision.py) ranges a wall base from its image ROW, so a 1 deg pitch error moves a base 1 m away by
~130 mm and broke the map program in the sim.  A depth pixel IS a range: a pitch
error only tilts the point up or down (1 deg at 1 m = 17 mm of height, < 5 mm of horizontal range).

What the camera did on the team's field (2026-09-22): depth0 16UC1 mm, 640 x 480 at 14 Hz, registered to rgb0 (same
K); no return from the white mat, most of the black walls missed, the pillars seen, one red pillar at ~1.25 m read
1.43 m.  So every use below is behind its own param and off until measured (tests.depth measures depth.scale):

    dg = DepthGeom(g, cfg, w, h)            # the RGB camera's pose (vision.Ground) + the depth lens' registration
    dc = columns(depth_u16, dg, cfg)        # per sampled column: the nearest returns in the 30-90 mm band
    retime(dc, odo, t_depth, t_rgb)         # carried into the RGB frame's car frame (the two frames differ in time)
    classify(dc, e, g)                      # the RGB colour decides what each column hit (wall / red / green / magenta)
    X, Y, k, sig = wall_points(dc, cfg)     # WALL columns -> Localizer.update(..., sig); k = the RGB columns replaced
    q = fix_pillar(q, dc, cfg, g)           # a sign's centre from its depth columns instead of its base row

Car frame as vision.py: origin at the rear axle on the floor, X forward, Y left, Z up, mm.
"""
from __future__ import annotations

import math

import numpy as np

from . import loc as L
from . import vision as V

PILLAR_HALF = 25.0                                  # mm: a sign is 50 x 50, its front face is 25 mm before its centre
_CLS = {"red": V.RED, "green": V.GREEN, "magenta": V.MAGENTA}


def config(p: dict) -> dict:
    """params["depth"] over the defaults (a params dict from an older params.json has no "depth" section)."""
    from . import params as P
    return dict(P.DEFAULTS["depth"], **(p.get("depth") or {}))


class DepthGeom:
    """Depth pixels -> car-frame points.  Registered (the vendor driver aligns depth0 to rgb0, measured): the depth
    image has the RGB camera's intrinsics and pose.  Unregistered: its own `intrinsics` and its lens at `offset_mm` in
    the RGB camera frame (x right, y down, z forward), axes parallel to the RGB camera's (one board, one housing).
    Built from the program's Ground, so a fitted pitch carries over; rebuild when `g` changes."""

    def __init__(self, g: V.Ground, cfg: dict, width: int, height: int):
        self.g = g
        cam = g.cam
        if int(cfg.get("registered", 1)):
            fx, fy, cx, cy = cam["intrinsics"]
            cal = cam.get("calib_size", [640, 480])
            off = (0.0, 0.0, 0.0)
        else:
            fx, fy, cx, cy = cfg["intrinsics"]
            cal = cfg.get("calib_size", [640, 480])
            off = cfg.get("offset_mm", (0.0, 0.0, 0.0))
        sx, sy = width / float(cal[0]), height / float(cal[1])   # a frame the bridge subsampled keeps pixel = 2k
        self.fx, self.fy, self.cx, self.cy = fx * sx, fy * sy, cx * sx, cy * sy
        self.W, self.H = int(width), int(height)
        self.shape = (self.H, self.W)
        self.R = g.R
        self.pos = g.pos + g.R @ np.asarray(off, float)          # the depth lens in the car frame
        self.scale = float(cfg.get("scale", 1.0))


class DepthCols:
    """Per sampled depth column: u (depth pixel column), X, Y, Z (car frame, mm; the mean of the returns within tol of
    the k-th nearest), r (horizontal range from the depth lens), n (returns in the band), good (n >= k).  After
    classify(): ur (the RGB column it projects to), k (index into the RGB Edges, -1 = outside), cls, r_rgb (that RGB
    column's floor-model range, NaN if none), clip (the RGB base was below the image)."""
    __slots__ = ("u", "X", "Y", "Z", "r", "n", "good", "pos", "ur", "k", "cls", "r_rgb", "clip")

    def __init__(self, u, X, Y, Z, n, good, pos):
        self.u, self.X, self.Y, self.Z, self.n, self.good, self.pos = u, X, Y, Z, n, good, pos
        self.r = np.hypot(X - pos[0], Y - pos[1])
        m = len(u)
        self.ur = np.full(m, np.nan)
        self.k = np.full(m, -1, np.int32)
        self.cls = np.zeros(m, np.int8)
        self.r_rgb = np.full(m, np.nan)
        self.clip = np.zeros(m, bool)


def columns(d: np.ndarray, dg: DepthGeom, cfg: dict) -> DepthCols:
    """The nearest return of each sampled column inside the wall / sign band.

    A return counts when its range is within [min_mm, max_mm] and its car-frame height within band_mm: the mat (0) and
    the tops (100) never do, and a 1 deg pitch error moves a point 1.5 m away by only 26 mm in height.  Per column the
    k-th nearest horizontal range (k_near: a few flying pixels at a sign's edge are not the column's range), then the
    mean of every return within tol_mm of it (a vertical face gives many at one range: the noise averages out)."""
    H, W = d.shape[:2]
    step = max(1, int(cfg.get("col_step", 4)) * W // 640)
    rstep = max(1, int(cfg.get("row_step", 2)) * H // 480)
    cols = np.arange(step // 2, W, step)
    rows = np.arange(rstep // 2, H, rstep)
    z = d[np.ix_(rows, cols)].astype(np.float32) * np.float32(dg.scale)
    ok = (z >= float(cfg.get("min_mm", 200.0))) & (z <= float(cfg.get("max_mm", 2000.0)))
    xp = ((cols - dg.cx) / dg.fx).astype(np.float32)[None, :]
    yp = ((rows - dg.cy) / dg.fy).astype(np.float32)[:, None]
    R, pos = dg.R.astype(np.float32), dg.pos
    X = z * (R[0, 0] * xp + R[0, 1] * yp + R[0, 2]) + np.float32(pos[0])
    Y = z * (R[1, 0] * xp + R[1, 1] * yp + R[1, 2]) + np.float32(pos[1])
    Z = z * (R[2, 0] * xp + R[2, 1] * yp + R[2, 2]) + np.float32(pos[2])
    lo, hi = cfg.get("band_mm", (30.0, 90.0))
    band = ok & (Z >= lo) & (Z <= hi)
    rh = np.where(band, np.hypot(X - np.float32(pos[0]), Y - np.float32(pos[1])), np.inf)
    n = band.sum(0)
    k = max(1, min(int(cfg.get("k_near", 3)), len(rows)))
    rk = np.sort(rh, axis=0)[k - 1]
    good = (n >= k) & np.isfinite(rk)
    sel = band & (np.abs(rh - rk[None, :]) <= float(cfg.get("tol_mm", 30.0)))   # the k-1 nearer ones stay out
    cnt = np.maximum(sel.sum(0), 1)
    Xm = np.where(sel, X, 0.0).sum(0) / cnt
    Ym = np.where(sel, Y, 0.0).sum(0) / cnt
    Zm = np.where(sel, Z, 0.0).sum(0) / cnt
    nan = np.float32(np.nan)
    return DepthCols(cols.astype(float), np.where(good, Xm, nan).astype(float), np.where(good, Ym, nan).astype(float),
                     np.where(good, Zm, nan).astype(float), n, good, pos)


def retime(dc: DepthCols, odo: L.Odo, t_from: float, t_to: float) -> DepthCols:
    """Carry the columns from the car frame at t_from (the depth exposure) to the car frame at t_to (the RGB
    exposure) with the program's odometry: to now, then back to t_to."""
    if abs(t_to - t_from) < 1e-4:
        return dc
    Xn, Yn = L.to_now(dc.X, dc.Y, odo.since(t_from))
    dc.X, dc.Y = L.from_now(Xn, Yn, odo.since(t_to))
    dc.r = np.hypot(dc.X - dc.pos[0], dc.Y - dc.pos[1])
    return dc


def classify(dc: DepthCols, e: V.Edges, g: V.Ground) -> DepthCols:
    """What each column hit, by the RGB: its point projected into the RGB image (the same instant after retime), the
    nearest sampled RGB column's class.  Projection, not the pixel index, so an unregistered lens and the time between
    the two frames (a turning car moves 2-3 columns in 40 ms) land on the right RGB column."""
    if not len(e.u) or not len(dc.u):
        return dc
    u, _v, zc = g.project(np.stack([dc.X, dc.Y, np.nan_to_num(dc.Z, nan=50.0)]))
    step = float(e.u[1] - e.u[0]) if len(e.u) > 1 else 1.0
    with np.errstate(invalid="ignore"):
        k = np.rint((u - e.u[0]) / step)
        inside = dc.good & np.isfinite(k) & (zc > 0) & (k >= 0) & (k < len(e.u))
    ki = np.where(inside, k, 0).astype(np.int32)
    dc.ur = np.where(inside, u, np.nan)
    dc.k = np.where(inside, ki, -1).astype(np.int32)
    dc.cls = np.where(inside, e.cls[ki], 0).astype(np.int8)
    dc.r_rgb = np.where(inside, np.hypot(e.X[ki] - g.pos[0], e.Y[ki] - g.pos[1]), np.nan)
    dc.clip = inside & e.clip[ki]
    return dc


def wall_points(dc: DepthCols, cfg: dict, gate: float | None = None):
    """The WALL columns as localiser measurements: (X, Y, k, sigma) with k the RGB column each one replaces.

    The gate: the depth range must agree with the same column's RGB floor range within `gate` (a share of it; 0 =
    off).  It rejects a sign's edge column the RGB calls wall, and -- on the real field, where most of the black wall
    gives no return -- the room seen OVER a near wall, which lands in the band 0.7-1 m behind it.  A clipped RGB base
    (below the image) is only a far bound.  1 deg of pitch error moves the RGB range 15 % at 1 m and 30 % at 2 m."""
    gate = float(cfg.get("gate", 0.35)) if gate is None else float(gate)
    m = dc.good & (dc.cls == V.WALL) & (dc.k >= 0)
    if gate > 0:
        with np.errstate(invalid="ignore", divide="ignore"):
            rel = (dc.r - dc.r_rgb) / dc.r_rgb
            m &= np.where(dc.clip, rel <= gate, np.abs(rel) <= gate)
    a, b = cfg.get("sig_mm", (12.0, 8e-6))
    r = dc.r[m]
    return dc.X[m], dc.Y[m], dc.k[m], float(a) + float(b) * r * r


def fix_pillar(q: dict, dc: DepthCols, cfg: dict, g: V.Ground) -> dict:
    """A red / green sign's centre from depth: the median range of its central columns (the RGB colour says which
    columns are this sign) + half a sign along the ray.  Returns q unchanged when depth has no say (too few columns,
    or it disagrees with the base row by more than the gate -- then something else is in the band there)."""
    want = _CLS.get(q.get("colour"))
    if want is None or want == V.MAGENTA:
        return q
    half = max(1.0, 0.3 * float(q.get("w_px", 0)))
    with np.errstate(invalid="ignore"):
        m = dc.good & (dc.cls == want) & (np.abs(dc.ur - float(q["u"])) <= half)
    if int(m.sum()) < int(cfg.get("pillar_cols", 1)):
        return q
    o = dc.pos
    r = float(np.median(dc.r[m]))
    az = float(np.median(np.arctan2(dc.Y[m] - o[1], dc.X[m] - o[0])))
    gate = float(cfg.get("gate", 0.35))
    if gate > 0 and not q.get("touches_bottom"):
        r_rgb = math.hypot(q["X"] - g.pos[0], q["Y"] - g.pos[1]) - PILLAR_HALF
        if r_rgb > 0 and abs(r - r_rgb) > gate * r_rgb:
            return q
    rc = r + PILLAR_HALF
    return dict(q, X=float(o[0] + rc * math.cos(az)), Y=float(o[1] + rc * math.sin(az)), est="depth",
                X_rgb=q["X"], Y_rgb=q["Y"], depth_cols=int(m.sum()))
