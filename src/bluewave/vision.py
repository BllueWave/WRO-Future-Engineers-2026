"""Camera -> floor geometry.  The stock LD19 scans above the 100 mm walls (measured 2026-09-22), so the camera IS the
range sensor: every column's lowest non-mat pixel is where a wall or a pillar stands on the mat, and a calibrated
pinhole + flat-floor model turns that pixel into millimetres.

    g = Ground(params["camera"])                   # intrinsics + camera height / pitch / roll / mounting
    e = edges(bgr, g, params["vision"])           # per column: base row, class, floor point (car frame)
    ps = pillars(bgr, g, params["vision"])        # red / green blobs with their floor position

Car frame: origin at the REAR AXLE centre on the floor, X forward, Y left, mm.
Camera frame: x right, y down, z forward (OpenCV).  pitch + = looking DOWN, roll + = image turns clockwise.

Why a dark-run and not a colour threshold for the wall: under the hall light the mat reads V~190 S~12, the black walls
V 20-35, the pillars saturated; the orange and blue floor lines read almost white (S 8-32, V 110-190) and are only a few
pixels tall, so "V < v_dark or saturated, for K rows" finds walls and pillars and never a floor line.  Measured on
the real camera 2026-09-22: see memory mentorpi-a1-platform.
"""
from __future__ import annotations

import math

import numpy as np

WALL, RED, GREEN, MAGENTA = 1, 2, 3, 4
CLASS_NAMES = {0: "none", WALL: "wall", RED: "red", GREEN: "green", MAGENTA: "magenta"}

DEFAULT_VISION = {
    "v_dark": 70, "s_col": 90, "v_col": 40, "K": 6, "col_step": 4,
    "red_h": [10, 165], "green_h": [38, 92], "magenta_h": [135, 165],
    "max_range_mm": 3000.0, "min_blob": 60,
}


class Ground:
    def with_pitch(self, pitch_deg: float) -> "Ground":
        """The same camera tilted to another pitch (for self-calibration against the map)."""
        return Ground(dict(self.cam, pitch_deg=float(pitch_deg)), self.W, self.H)

    def __init__(self, cam: dict, width: int = 640, height: int = 480):
        self.cam = dict(cam)
        fx, fy, cx, cy = cam["intrinsics"]
        sx, sy = width / cam.get("calib_size", [640, 480])[0], height / cam.get("calib_size", [640, 480])[1]
        self.fx, self.fy, self.cx, self.cy = fx * sx, fy * sy, cx * sx, cy * sy
        self.W, self.H = width, height
        self.h = float(cam["h_mm"])
        self.pos = np.array([float(cam.get("x_mm", 150.0)), float(cam.get("y_mm", 0.0)), self.h])
        th, ph = math.radians(cam["pitch_deg"]), math.radians(cam.get("roll_deg", 0.0))
        z = np.array([math.cos(th), 0.0, -math.sin(th)])          # optical axis in the car frame
        y = np.array([-math.sin(th), 0.0, -math.cos(th)])         # image down
        x = np.array([0.0, -1.0, 0.0])                            # image right
        self.xc = x * math.cos(ph) + y * math.sin(ph)
        self.yc = -x * math.sin(ph) + y * math.cos(ph)
        self.zc = z
        self.R = np.stack([self.xc, self.yc, self.zc], 1)         # camera -> car: P_car = R @ p_cam + pos

    def rays(self, u, v):
        """Unit-z camera rays of pixels, in the car frame (3, n)."""
        u = np.asarray(u, float).ravel()
        v = np.asarray(v, float).ravel()
        xp, yp = (u - self.cx) / self.fx, (v - self.cy) / self.fy
        return self.R @ np.stack([xp, yp, np.ones_like(xp)])

    def floor(self, u, v, z: float = 0.0):
        """Pixels -> (X, Y) where their rays meet the plane at height z; NaN above the horizon."""
        d = self.rays(u, v)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (z - self.pos[2]) / d[2]
        t = np.where(t > 0, t, np.nan)
        return self.pos[0] + t * d[0], self.pos[1] + t * d[1]

    def project(self, P):
        """Car-frame points (3, n) -> (u, v, depth); depth <= 0 means behind the camera."""
        p = self.R.T @ (np.asarray(P, float) - self.pos[:, None])
        with np.errstate(divide="ignore", invalid="ignore"):
            return self.cx + self.fx * p[0] / p[2], self.cy + self.fy * p[1] / p[2], p[2]

    def horizon_row(self) -> float:
        """Highest image row (over all columns) that can still see the floor."""
        us = np.array([0.0, self.W / 2, self.W - 1.0])
        best = self.H
        for u in us:
            lo, hi = 0.0, self.H - 1.0                        # rays go down as v grows
            for _ in range(30):
                m = (lo + hi) / 2
                if self.rays([u], [m])[2, 0] < 0:
                    hi = m
                else:
                    lo = m
            best = min(best, hi)
        return best


def _hsv(bgr):
    import cv2
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)


def _classes(hsv, vp):
    Hh, S, V = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    # the blue floor line reads saturated dark blue under some light (H~111 S~144 V 60-110, measured 2026-09-22): its
    # darkest pixels must not pass as black wall -- a black wall is dark at ANY hue, the line only at blue hues
    blue_line = (Hh >= 95) & (Hh <= 130) & (S > 90) & (V > 40)
    dark = (V < vp["v_dark"]) & ~blue_line
    col = (S > vp["s_col"]) & (V > vp["v_col"])
    red = col & ((Hh <= vp["red_h"][0]) | (Hh >= vp["red_h"][1]))
    green = col & (Hh >= vp["green_h"][0]) & (Hh <= vp["green_h"][1])
    mag = col & (Hh >= vp["magenta_h"][0]) & (Hh < vp["magenta_h"][1])
    return dark, red, green, mag


class Edges:
    """Per sampled column: u, v (base row, NaN = mat all the way to the horizon), cls, X, Y (car frame, mm), clip
    (the obstacle reaches the image bottom: its base is out of view and nearer than X, Y)."""
    __slots__ = ("u", "v", "cls", "X", "Y", "clip", "t", "behind", "vv", "behind_v")

    def __init__(self, u, v, cls, X, Y, clip, t=0.0, vv=None):
        self.u, self.v, self.cls, self.X, self.Y, self.clip, self.t = u, v, cls, X, Y, clip, t
        self.vv = vv                                               # sub-pixel base rows (continuous image rows)
        self.behind = (np.zeros(0), np.zeros(0), np.zeros(0))       # wall bases seen over a magenta limitation
        self.behind_v = np.zeros(0)

    def wall_rows(self, max_mm: float = 3000.0):
        """(u, row) of the wall bases points() would return -- to re-project them with another camera pitch."""
        m = (self.cls == WALL) & np.isfinite(self.X) & ~self.clip & (np.hypot(self.X, self.Y) < max_mm)
        u, v = self.u[m], self.vv[m]
        if len(self.behind[0]):
            k = np.hypot(self.behind[0], self.behind[1]) < max_mm
            u, v = np.concatenate([u, self.behind[2][k]]), np.concatenate([v, self.behind_v[k]])
        return u.astype(float), v.astype(float)

    def points(self, classes=(WALL,), max_mm: float = 3000.0):
        """Wall base points with a SEEN base -- what the localiser may use -- including those seen over a limitation."""
        m = np.isin(self.cls, classes) & np.isfinite(self.X) & ~self.clip & (np.hypot(self.X, self.Y) < max_mm)
        X, Y, u = self.X[m], self.Y[m], self.u[m]
        if WALL in classes and len(self.behind[0]):
            bx, by, bu = self.behind
            k = np.hypot(bx, by) < max_mm
            X, Y, u = np.concatenate([X, bx[k]]), np.concatenate([Y, by[k]]), np.concatenate([u, bu[k]])
        return X, Y, u

    def front(self, half_width_mm: float = 90.0, classes=(WALL, RED, GREEN, MAGENTA)) -> float:
        """Nearest obstacle straight ahead in a corridor of +-half_width (mm from the rear axle), inf if none seen."""
        m = np.isin(self.cls, classes) & np.isfinite(self.X) & (np.abs(self.Y) < half_width_mm)
        return float(self.X[m].min()) if m.any() else math.inf


def edges(bgr, g: Ground, vp: dict | None = None, t: float = 0.0) -> Edges:
    vp = dict(DEFAULT_VISION, **(vp or {}))
    H, W = bgr.shape[:2]
    K, step = int(vp["K"]), int(vp["col_step"])
    v0 = max(0, int(g.horizon_row()) - K)
    band = bgr[v0:, step // 2::step]
    hsv = _hsv(band)
    dark, red, green, mag = _classes(hsv, vp)
    obst = dark | red | green | mag
    c = np.cumsum(obst, axis=0, dtype=np.int32)
    win = c.copy()
    win[K:] -= c[:-K]
    cond = (win >= K - 1) & obst                                  # the base row itself must be obstacle
    cond[:K] = False
    n = cond.shape[0]
    rev = cond[::-1]
    has = rev.any(axis=0)
    r = (n - 1) - rev.argmax(axis=0)                              # lowest obstacle row with an obstacle run above
    u = np.arange(band.shape[1]) * step + step // 2
    v = np.where(has, r + v0, np.nan)
    cls = np.zeros(len(u), np.int8)
    edge = np.where(has, r + 0.5, n).astype(float)               # pixel k covers [k - .5, k + .5]
    cols = np.nonzero(has)[0]
    if len(cols):
        rows = r[cols][:, None] - np.arange(3 * K)[None, :]      # 3K rows above (and at) the base
        rows = np.clip(rows, 0, n - 1)
        cc = np.broadcast_to(cols[:, None], rows.shape)
        counts = np.stack([dark[rows, cc].sum(1), red[rows, cc].sum(1), green[rows, cc].sum(1), mag[rows, cc].sum(1)])
        cls[cols] = np.array([WALL, RED, GREEN, MAGENTA])[counts.argmax(0)]
        # sub-pixel wall edge: rows r-2 .. r+3 each contribute their darkness fraction between the wall level (rows
        # r-6..r-3) and the mat level (rows r+4..r+7); for averaging pixels their sum is exactly the edge position
        wc = cols[cls[cols] == WALL]
        if len(wc):
            Vb = hsv[..., 2].astype(np.float32)
            rr = r[wc]
            ok = (rr - 6 >= 0) & (rr + 7 < n)
            wc, rr = wc[ok], rr[ok]
            if len(wc):
                D = np.median(Vb[rr[:, None] + np.arange(-6, -2)[None, :], wc[:, None]], axis=1)
                M = np.median(Vb[rr[:, None] + np.arange(4, 8)[None, :], wc[:, None]], axis=1)
                good = (M - D) > 60
                seg = Vb[rr[:, None] + np.arange(-2, 4)[None, :], wc[:, None]]
                frac = np.clip((M[:, None] - seg) / np.maximum(M - D, 1.0)[:, None], 0.0, 1.0)
                e_sub = (rr - 2 - 0.5) + frac.sum(1)
                edge[wc[good]] = e_sub[good]
    vv = np.where(has, edge + v0, g.H)                            # base = the wall / floor boundary, continuous rows
    vv = np.minimum(vv, g.H - 0.5)
    X, Y = g.floor(u, vv)
    X = np.where(has, X, np.nan)
    Y = np.where(has, Y, np.nan)
    clip = has & (r >= n - 2)
    e = Edges(u, v, cls, X, Y, clip, t, vv)
    # behind a magenta limitation (100 mm tall, the camera 123 mm up) the mat and the wall beyond stay in view: from
    # inside the parking lot the whole bottom of the image is the front limitation, and without this second look the
    # localiser would get no wall at all
    mg = np.nonzero(has & (cls == MAGENTA))[0]
    if len(mg):
        mat = ~obst
        X2, Y2, u2, v2 = [], [], [], []
        for ci in mg:
            rr = int(r[ci])
            while rr > K and obst[rr, ci]:
                rr -= 1                                           # up through the limitation
            top = rr
            while rr > K and mat[rr, ci]:
                rr -= 1                                           # up through the mat behind it
            if top - rr < 3 or rr <= K:
                continue
            if obst[rr - K + 1:rr + 1, ci].sum() >= K - 1 and dark[rr - K + 1:rr + 1, ci].sum() >= K - 2:
                xx, yy = g.floor([u[ci]], [rr + 0.5 + v0])
                if np.isfinite(xx[0]):
                    X2.append(xx[0])
                    Y2.append(yy[0])
                    u2.append(u[ci])
                    v2.append(rr + 0.5 + v0)
        e.behind = (np.array(X2), np.array(Y2), np.array(u2))
        e.behind_v = np.array(v2)
    return e


def pillars(bgr, g: Ground, vp: dict | None = None):
    """Red / green / magenta blobs standing on the mat: [{colour, u, v_base, X, Y, w_px, touches_bottom}] with X, Y the
    floor point under the blob's bottom-centre, pushed 25 mm (half a pillar) further along the ray."""
    import cv2
    vp = dict(DEFAULT_VISION, **(vp or {}))
    v0 = max(0, int(g.horizon_row()) - 60)
    band = bgr[v0:]
    _dark, red, green, mag = _classes(_hsv(band), vp)
    out = []
    for name, m in (("red", red), ("green", green), ("magenta", mag)):
        n, _lab, st, cen = cv2.connectedComponentsWithStats(m.astype(np.uint8), connectivity=8)
        for i in range(1, n):
            x, y, w, h, a = st[i]
            if a < vp["min_blob"] or h < 4:
                continue
            vb = y + h + v0
            uc = x + w / 2.0
            est = "base"
            if vb >= g.H - 2 and name != "magenta":
                # base below the image: range it from the TOP edge instead -- the blob's top row is the far edge of the
                # sign's 100 mm top face, and the camera is only ~23 mm above it, so up close that row moves a lot per
                # millimetre (9 mm/px at 350 mm)
                X, Y = g.floor([uc], [y + v0 + 0.5], z=100.0)
                X, Y = float(X[0]), float(Y[0])
                if not math.isfinite(X) or math.hypot(X - g.pos[0], Y - g.pos[1]) > 800.0:
                    continue
                rr = math.hypot(X - g.pos[0], Y - g.pos[1])
                if rr > 0:                                         # far top edge -> the sign's centre
                    X -= 25.0 * (X - g.pos[0]) / rr
                    Y -= 25.0 * (Y - g.pos[1]) / rr
                out.append(dict(colour=name, u=float(uc), v_base=float(vb), X=X, Y=Y, w_px=int(w), h_px=int(h),
                                area=int(a), touches_bottom=True, est="top"))
                continue
            X, Y = g.floor([uc], [vb])
            X, Y = float(X[0]), float(Y[0])
            if not math.isfinite(X):
                continue
            rr = math.hypot(X - g.pos[0], Y - g.pos[1])
            touches = vb >= g.H - 2 or y + v0 <= 1
            if not touches and h < 0.45 * g.fy * 100.0 / max(rr, 1.0):
                continue                                          # a fragment (highlight split), not a whole pillar
            if rr > 0:
                X += 25.0 * (X - g.pos[0]) / rr
                Y += 25.0 * (Y - g.pos[1]) / rr
            out.append(dict(colour=name, u=float(uc), v_base=float(vb), X=X, Y=Y, w_px=int(w), h_px=int(h),
                            area=int(a), touches_bottom=bool(vb >= g.H - 2), est=est))
    return sorted(out, key=lambda b: b["X"])


def draw(bgr, e: Edges, ps=None):
    """Debug overlay: base points coloured by class, pillar boxes."""
    import cv2
    out = bgr.copy()
    colour = {WALL: (0, 255, 255), RED: (0, 0, 255), GREEN: (0, 255, 0), MAGENTA: (255, 0, 255)}
    for u, v, c in zip(e.u, e.v, e.cls):
        if v == v:
            cv2.circle(out, (int(u), int(v)), 2, colour.get(int(c), (255, 255, 255)), -1)
    for p in ps or []:
        cv2.putText(out, "%s %.0f,%.0f" % (p["colour"][0], p["X"], p["Y"]), (int(p["u"]) - 30, int(p["v_base"]) + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    return out
