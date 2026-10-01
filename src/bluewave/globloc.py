"""START ANYWHERE (BRAIN4_SPEC 4): where is the car, from any pose on the mat, before it moves?

    v = camera_view(robot, g, vp)                     # a STILL view: 5 frames, per column the median base row
    v = lidar_view(robot)                             # (lidar.loc = 1) 3 revolutions, per bin the median range
    fix = search([v])                                 # the whole field, every heading: classes with an honest p
    fix = resolve_frame(fix, [v], "auto", needed, expect=[x, y, deg], survey=s)   # which twin is the field frame
    if fix.confident: seed(loc, fix)                  # else Look: short moves, one more still view each

Why it exists: every start search before this one looked only in the start straight (x +-560, y -1460..-540, headings
0 / pi +-14 deg) or at the four wall headings +-14-30 deg; a car put down at 45 deg in a corner was in no window, and the
only confidence was a hypothesis count.  The mat 2026-09-23 also showed what a moving view does to the fit (field_check
1.2-1.5 on a moving car, 0.009 still: M3) -- every view here is a STILL view.

THE SEARCH (4.4).  The field's walls are axis-aligned, so the dominant direction of the wall points (pairs of
neighbouring points, mod 90 deg) gives the heading up to the field's 4-fold symmetry; a 4-fold symmetric map needs one
quadrant of headings (every other hypothesis is a rotation of one found there).  Positions: the whole field on a 40 mm
grid, kept where the car could stand at every view.  Cost: the ray-cast cost of loc.Localizer.beam_cost (it knows a
near wall hides a far one), computed here by a slab ray-cast against the outer square and the island box -- the same
numbers (tests_console/test_globloc.py T4) at a fraction of the work.  Candidates: grid-cell minima, non-maximum
suppression, refined in two stages; a lidar view's best class is then refined on the likelihood field (the ray-cast
finds the basin, the distance field sets the millimetres).

HONEST CONFIDENCE (4.5).  C_i = the total ray-cast cost of class i; T = max(1, cols / temper_k) is the filter's own
temper for correlated columns (neighbouring columns see one wall); p_i = exp(-(C_i - C_best) / 2T) normalised over the
distinct classes found.  A view of one long wall leaves a line of classes of equal cost: p falls, and the car LOOKs.
`confident` needs p, the per-column cost, enough columns, enough bearing span, and the frame when the run needs it.

THE TWIN (4.6).  The empty field is 4-fold symmetric: the walls alone never say which straight is S.  The lot (magenta
bases or the lidar's off-wall returns in S's lot band), a saved survey's signs, the team's start mark (`expect`) or the
rules' start straight decide; two sources that disagree make the fix not confident.

LOOK (4.7).  Short forward moves (reverse only with the lidar shield: S14), each checked against what the sensor sees
AND, on the stock A1, against the map around EVERY plausible hypothesis (walls + every seat not seen empty): the A1's
camera sees nothing within 213 mm of its nose nor beside the car, so a turn into an unseen wall would pass the camera
shield alone.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field as _dfield

import numpy as np

from . import field as F
from . import lidar_perc as LP
from . import loc as L

# ------------------------------------------------------------------------------------------------------------ params
# BRAIN4_SPEC 2.1: the loc.* / look.* params with their defaults (params.py DEFAULTS gains them: B4); every read here is
# .get(key, default), so nothing waits for that
CFG = dict(min_p=0.95, good=0.30, bad=0.80, min_cols=24, min_cov_deg=40.0, temper_k=8.0, step_mm=40.0,
           heading_win_deg=3.0, full_step_deg=2.0, nms_mm=150.0, nms_deg=6.0, max_hyps=8, lot_min_pts=8,
           expect_mm=150.0, expect_deg=15.0, locate_s=8.0)
LOOK = dict(v=0.15, step_mm=150.0, max_mm=900.0, views=6, still_s=0.6)
MAX_PTS = 48                      # columns / points per view in the search (the start search's own 48)
LOT_Y = (-1500.0, -1290.0)        # straight S's lot band [RULE p.8]: the limitations stand here
LOT_X = 1400.0
SIGN_MATCH_MM = 120.0             # a view's sign vs a survey's sign


def cfg_of(robot_p: dict | None) -> dict:
    """The loc.* params of a robot params dict over CFG."""
    return dict(CFG, **{k: v for k, v in ((robot_p or {}).get("loc") or {}).items() if k in CFG})


def look_of(robot_p: dict | None) -> dict:
    return dict(LOOK, **{k: v for k, v in ((robot_p or {}).get("look") or {}).items() if k in LOOK})


# ------------------------------------------------------------------------------------------------------------ records
@dataclass
class View:
    t: float                    # monotonic time of the view (the newest frame / scan in it)
    odo: tuple                  # (x, y, th) the car in the LOCATE frame at this view
    src: str                    # "camera" | "lidar" | "depth"
    sensor_x: float             # the range sensor on the car's axis (camera.x_mm or lidar.pos_mm[0]); y must be 0
    X: np.ndarray               # WALL points, car frame at the view, mm
    Y: np.ndarray
    sig: np.ndarray             # per-point sigma, mm
    magenta: np.ndarray = _dfield(default_factory=lambda: np.zeros((0, 2)))   # (k, 2) the lot's limitations, car frame
    signs: list = _dfield(default_factory=list)          # [(X, Y, colour | None)] car frame
    raw: object = None                                   # lidar: the median scan; camera: None
    clip_bearings: np.ndarray = _dfield(default_factory=lambda: np.zeros(0))   # deg, car frame, clipped wall columns
    edges: object = None                                 # camera: the last frame's vision.Edges (the camshield's feed)
    blind: tuple = ()                                    # lidar: lidar.block_deg (the body's own sector) for `raw`


@dataclass
class Fix:
    pose: tuple                 # (x, y, th) rear axle NOW, field frame: the chosen twin of the best class
    sxy: float                  # honest spread of the class's good set, mm / rad (floors 10 mm, 0.5 deg)
    sth: float
    p: float                    # probability of the best class among the distinct classes found
    cost: float                 # per-column ray-cast cost of the best
    hyps: list                  # [(x, y, th, p, cost)] distinct classes (twin 0, now), best first, <= max_hyps
    twins: list                 # [(x, y, th)] R_k of the best (now), k over the layout's symmetry
    frame_k: int | None         # index into twins chosen as the field frame; None = unresolved
    frame_by: str               # "lot" | "survey" | "expect" | "start" | "any" | "none"
    layout: str                 # "standard" | "open-S-E-N-W" | "unknown"
    #                             (not_standard with layout "standard": no single Open layout fits clearly -- `layouts`
    #                             holds each layout's untrimmed per-column cost)
    cols: int
    cov_deg: float
    views: int
    src: str
    ms: float
    confident: bool
    not_standard: bool
    why: str
    # ---- the builder's (not in the event): what resolve_frame / seed / Look need
    p0: list = _dfield(default_factory=list)       # [(x, y, th)] LOCATE-origin pose of every class (twin 0)
    ps: list = _dfield(default_factory=list)       # p of every class
    odo_last: tuple = (0.0, 0.0, 0.0)
    sym: int = 4
    island: tuple = (-F.ISL, F.ISL, -F.ISL, F.ISL)
    base_ok: bool = False                          # every criterion but the frame
    basins: list = _dfield(default_factory=list)   # per class: how far (mm) along x / y it fits as well (0 = sharp)
    guard: list = _dfield(default_factory=list)    # [(x, y, th)] LOCATE-origin poses of the coarse good set (<= 40,
    #                                                twin 0): LOOK's map check must hold at every pose the car may be
    base_why: str = ""
    start_check: dict | None = None                # expect: {expect, found, d_mm, d_deg, ok}
    layouts: dict = _dfield(default_factory=dict)   # layout name -> per-column cost of its best (P1: the Open layouts)

    def event(self) -> dict:
        return dict(ev="fix", x=round(self.pose[0]), y=round(self.pose[1]), th=round(math.degrees(self.pose[2]), 1),
                    sxy=round(self.sxy, 1), sth=round(math.degrees(self.sth), 2), p=round(self.p, 3),
                    cost=round(self.cost, 3),
                    hyps=[[round(h[0]), round(h[1]), round(math.degrees(h[2]), 1), round(h[3], 3), round(h[4], 3)]
                          for h in self.hyps],
                    frame_by=self.frame_by, frame_k=self.frame_k, cols=int(self.cols), cov_deg=round(self.cov_deg, 1),
                    views=int(self.views), src=self.src, layout=self.layout, confident=bool(self.confident),
                    not_standard=bool(self.not_standard), why=self.why, ms=round(self.ms))


# ------------------------------------------------------------------------------------------------------------ geometry
def compose(a, b):
    """a (+) b: the pose b given in the frame of pose a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _wrap(a[2] + b[2]))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _compose_many(P, o):
    """P (n, 3) (+) the fixed pose o -> (n, 3)."""
    c, s = np.cos(P[:, 2]), np.sin(P[:, 2])
    return np.stack([P[:, 0] + c * o[0] - s * o[1], P[:, 1] + s * o[0] + c * o[1], P[:, 2] + o[2]], 1)


def direction_at(pose) -> int:
    """+1 (ccw) when the car faces the counter-clockwise tangent at its position, else -1 (BRAIN4_SPEC 2)."""
    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    return 1 if math.cos(th - (math.atan2(y, x) + math.pi / 2)) >= 0 else -1


def layouts(kind: str = "obstacle") -> dict:
    """name -> (walls N x 4, sym, island): "standard" always; with kind "open" also the 15 other corridor sets of the
    Open Challenge (600 or 1000 mm per straight [RULE]), named open-S-E-N-W."""
    out = {"standard": (F.WALLS, 4, (-F.ISL, F.ISL, -F.ISL, F.ISL))}
    if kind == "open":
        for cs in range(16):
            corr = tuple(600.0 if (cs >> i) & 1 else 1000.0 for i in range(4))
            if corr == (1000.0, 1000.0, 1000.0, 1000.0):
                continue
            s, e, n, w = corr
            sym = 4 if s == e == n == w else 2 if (s == n and e == w) else 1
            out["open-%d-%d-%d-%d" % corr] = (F.layout_walls(corr), sym, F.island_of(corr))
    return out


def ray_aa(cx, cy, ang, island):
    """Distance along each ray to the first face of the outer square (inner faces +-1500) or the island box: the
    slab method, exact for an axis-aligned field (loc.Localizer.beam_cost's walls).  cx, cy broadcast against ang."""
    ix = 1.0 / (np.cos(ang) + 1e-12)                # 1e-12: no 0 / 0 (and no NaN) on an exactly vertical ray
    iy = 1.0 / (np.sin(ang) + 1e-12)
    H = F.HALF
    l, r, b, t = island
    # the outer square from inside: each slab's far root
    out = np.minimum(np.maximum((H - cx) * ix, (-H - cx) * ix), np.maximum((H - cy) * iy, (-H - cy) * iy))
    a1, a2 = (l - cx) * ix, (r - cx) * ix
    b1, b2 = (b - cy) * iy, (t - cy) * iy
    tmin = np.maximum(np.minimum(a1, a2), np.minimum(b1, b2))
    tmax = np.minimum(np.maximum(a1, a2), np.maximum(b1, b2))
    hit = (tmax >= tmin) & (tmax > 0)
    ti = np.where(tmin > 0, tmin, tmax)             # from inside the island (not a valid pose) its far face
    return np.where(hit, np.minimum(out, ti), out)


def wall_dist(xf, yf, island=(-F.ISL, F.ISL, -F.ISL, F.ISL)):
    """Exact distance (mm) of field points to the nearest wall face of an axis-aligned layout (the outer square's
    inner faces, the island box's outer faces): the likelihood field without field.DistField's 10 mm cells."""
    x, y = np.asarray(xf, float), np.asarray(yf, float)
    outer = np.abs(F.HALF - np.maximum(np.abs(x), np.abs(y)))
    l, r, b, t = island
    ex = np.maximum(np.maximum(l - x, x - r), 0.0)
    ey = np.maximum(np.maximum(b - y, y - t), 0.0)
    inside = (x > l) & (x < r) & (y > b) & (y < t)
    isl = np.where(inside, np.minimum(np.minimum(x - l, r - x), np.minimum(y - b, t - y)), np.hypot(ex, ey))
    return np.minimum(outer, isl)


def cost_poses(P, vw, island, chunk: int = 4000) -> np.ndarray:
    """Total ray-cast cost of LOCATE-origin poses P (n, 3) over the prepared views vw (each: az, r, sig, sx, odo).
    Per column min(((ray - r) / sig capped at 3)^2, 9) -- loc.Localizer.beam_cost's formula."""
    P = np.asarray(P, float)
    out = np.zeros(len(P))
    for v in vw:
        az, rng, sig, sx, o = v["az"], v["r"], v["sig"], v["sx"], v["odo"]
        for i in range(0, len(P), chunk):
            Q = _compose_many(P[i:i + chunk], o) if o != (0.0, 0.0, 0.0) else P[i:i + chunk]
            cx = (Q[:, 0] + sx * np.cos(Q[:, 2]))[:, None]
            cy = (Q[:, 1] + sx * np.sin(Q[:, 2]))[:, None]
            t = ray_aa(cx, cy, Q[:, 2][:, None] + az[None, :], island)
            rr = np.minimum((t - rng[None, :]) / sig[None, :], 3.0)
            out[i:i + chunk] += np.minimum(rr * rr, 9.0).sum(axis=1)
    return out


def sign_cost(p0, views, max_mm: float = 1300.0):
    """(cost, n) of the views' sign detections at LOCATE-origin pose p0: per sign min(((d - 30) / sig)^2, 9), d its
    distance to the nearest seat, sig = 30 + 0.04 r (the floor model's range error grows with r)."""
    tot, n = 0.0, 0
    for v in views:
        if not v.signs:
            continue
        pv = compose(p0, v.odo)
        for X, Y, _c in v.signs:
            r = math.hypot(X - float(v.sensor_x), Y)
            if r > max_mm:
                continue
            fx, fy = LP.to_field(np.array([X]), np.array([Y]), pv)
            d = float(np.min(np.hypot(F.SEAT_XY[:, 0] - fx[0], F.SEAT_XY[:, 1] - fy[0])))
            tot += min((max(d - 30.0, 0.0) / (30.0 + 0.04 * r)) ** 2, 9.0)
            n += 1
    return tot, n


def col_costs(p0, vw, island) -> np.ndarray:
    """Every view's per-column ray-cast cost at ONE LOCATE-origin pose (cost_poses' terms, concatenated)."""
    out = []
    for v in vw:
        Q = np.asarray(compose(tuple(p0), v["odo"]), float)
        cx = Q[0] + v["sx"] * math.cos(Q[2])
        cy = Q[1] + v["sx"] * math.sin(Q[2])
        t = ray_aa(cx, cy, Q[2] + v["az"], island)
        rr = np.minimum((t - v["r"]) / v["sig"], 3.0)
        out.append(np.minimum(rr * rr, 9.0))
    return np.concatenate(out) if out else np.zeros(0)


def fit_cost(p0, vw, island, trim: float = 0.10) -> float:
    """The per-column cost that says whether a view FITS the layout (loc.good / loc.bad): the mean without the worst
    `trim` share of columns -- a limitation, a sign the pose-free lidar did not name, a foot on the mat is no wall of
    the map and costs 9 in its 2-5 columns at the true pose (a lidar fix beside the lot read 0.43, good 0.30)."""
    c = np.sort(col_costs(p0, vw, island))
    if not len(c):
        return math.inf
    k = max(1, int(round(len(c) * (1.0 - trim))))
    return float(c[:k].mean())


def _valid_all(P, vw, island) -> np.ndarray:
    ok = np.ones(len(P), bool)
    for v in vw:
        o = v["odo"]
        Q = _compose_many(P, o) if o != (0.0, 0.0, 0.0) else P
        ok &= L.Localizer.valid(Q, island=island)
    return ok


# ------------------------------------------------------------------------------------------------------------ views
def _prep(views, max_pts: int = MAX_PTS) -> list:
    """Each view as the search's arrays: <= max_pts wall points evenly by bearing (from the sensor), their bearing,
    range and sigma, the sensor's x and the view's LOCATE-frame odometry."""
    out = []
    for v in views:
        X, Y = np.asarray(v.X, float), np.asarray(v.Y, float)
        if not len(X):
            continue
        sig = np.asarray(v.sig, float) if v.sig is not None and len(v.sig) == len(X) else L.floor_sigma(X, Y)
        sx = float(v.sensor_x)
        az = np.arctan2(Y, X - sx)
        o = np.argsort(az, kind="stable")
        X, Y, sig, az = X[o], Y[o], sig[o], az[o]
        m = np.unique(np.linspace(0, len(X) - 1, min(len(X), max_pts)).round().astype(int))
        out.append(dict(az=az[m], r=np.hypot(X[m] - sx, Y[m]), sig=sig[m], sx=sx,
                        odo=tuple(float(q) for q in v.odo), X=X, Y=Y, src=v.src))
    return out


def _bearings_locate(views) -> np.ndarray:
    """Every wall point's bearing (deg) from its view's sensor, in the LOCATE frame."""
    out = []
    for v in views:
        if not len(v.X):
            continue
        out.append(np.degrees(np.arctan2(np.asarray(v.Y, float), np.asarray(v.X, float) - float(v.sensor_x)))
                   + math.degrees(float(v.odo[2])))
    return np.concatenate(out) if out else np.zeros(0)


def coverage_deg(views) -> float:
    """The bearing span of the wall points (LOCATE frame): 360 minus the largest empty gap of a 1 deg histogram."""
    b = _bearings_locate(views)
    if not len(b):
        return 0.0
    h = np.zeros(360, bool)
    h[(np.floor(b) % 360).astype(int)] = True
    if h.all():
        return 360.0
    k = int(np.argmin(h))                          # start the scan at an empty bin
    hh = np.roll(h, -k)
    gaps, run = [], 0
    for x in hh:
        if not x:
            run += 1
        else:
            if run:
                gaps.append(run)
            run = 0
    if run:
        gaps.append(run)
    return float(360 - max(gaps)) if gaps else 360.0


def _heading(views, bin_deg: float = 0.5):
    """(alpha_deg in [-45, 45), strength 0..1, total pair length mm): the dominant wall direction mod 90 in the
    LOCATE frame, from pairs of wall points on one continuous run (every bearing-neighbour step < 60 mm), weighted
    by length.  Pairs k = 1, 2, 4, 8 points apart, not only neighbours: two lidar returns 17-50 mm apart with 6 mm of
    range noise give a direction +-16 deg (the histogram's strength fell to 0.13 on a clean lidar view and the search
    ran the whole quadrant); 8 apart it is +-2 deg."""
    nb = int(round(90.0 / bin_deg))
    hist = np.zeros(nb)
    for v in views:
        X, Y = np.asarray(v.X, float), np.asarray(v.Y, float)
        if len(X) < 3:
            continue
        az = np.arctan2(Y, X - float(v.sensor_x))
        o = np.argsort(az, kind="stable")
        X, Y = X[o], Y[o]
        gap = np.hypot(np.diff(X), np.diff(Y))
        brk = np.concatenate([[0], np.cumsum(gap >= 60.0)])          # run id of each point
        for kk in (1, 2, 4, 8):
            if len(X) <= kk:
                break
            same = brk[kk:] == brk[:-kk]
            dx, dy = X[kk:] - X[:-kk], Y[kk:] - Y[:-kk]
            ln = np.hypot(dx, dy)
            m = same & (ln > 1.0) & (ln < 400.0) & ((ln >= 30.0) | (kk == 1))
            if not m.any():
                continue
            d = np.degrees(np.arctan2(dy[m], dx[m])) + math.degrees(float(v.odo[2]))
            d = (d + 45.0) % 90.0 - 45.0
            k = (np.floor((d + 45.0) / bin_deg).astype(int)) % nb
            np.add.at(hist, k, ln[m] / (1.0 + math.log2(kk)))
    tot = float(hist.sum())
    if tot <= 0:
        return 0.0, 0.0, 0.0
    ker = np.array([1.0, 2.0, 3.0, 2.0, 1.0])
    sm = np.zeros(nb)
    for i, w in enumerate(ker):
        sm += w * np.roll(hist, i - 2)
    j = int(np.argmax(sm))
    half = int(round(2.0 / bin_deg))
    idx = (np.arange(j - half, j + half + 1)) % nb
    w = hist[idx]
    centres = -45.0 + (np.arange(j - half, j + half + 1) + 0.5) * bin_deg
    alpha = float((w * centres).sum() / max(w.sum(), 1e-9))
    alpha = (alpha + 45.0) % 90.0 - 45.0
    return alpha, float(w.sum() / tot), tot


def heading_mod90(views) -> tuple:
    """(alpha_deg in [-45, 45): the dominant wall direction in the LOCATE frame, strength 0..1)."""
    a, s, _t = _heading(views)
    return a, s


def _still_cam(robot, prev_thumb, bgr, gyro_dps=0.8, img_diff=1.0):
    from .tests_cal import _gyro_dps, _thumb
    th = _thumb(bgr)
    g = _gyro_dps(robot)
    diff = float(np.mean(np.abs(th - prev_thumb))) if prev_thumb is not None else 0.0
    moving = abs(float(robot.v_odo())) >= 0.02 or abs(float(getattr(robot, "v_cmd", 0.0))) > 0.0
    ok = (g is None or g <= gyro_dps) and diff <= img_diff and not moving
    return ok, th, diff, g


def view_from_edges(es, g, odo=(0.0, 0.0, 0.0), t: float = 0.0, bgr=None, vp=None, max_mm: float = 2600.0,
                    agree_px: float = 2.0) -> View:
    """A still camera view from several frames' vision.Edges of the SAME camera: per sampled column the median
    sub-pixel base row over the frames that see a base there, kept when >= min(3, frames) of them agree within
    agree_px (a flickering column is dropped); re-projected once with the camera model."""
    from . import vision as V
    n = len(es)
    VV = np.stack([np.where(np.isfinite(e.v), e.vv, np.nan) for e in es])
    CL = np.stack([e.cls for e in es]).astype(int)
    CP = np.stack([e.clip for e in es])
    with np.errstate(all="ignore"):
        med = np.nanmedian(VV, axis=0)
    agree = np.abs(VV - med[None, :]) <= agree_px
    need = min(3, n)
    okc = np.isfinite(med) & (agree.sum(0) >= need)
    u = es[-1].u.astype(float)
    cls = np.zeros(len(u), int)
    clip = np.zeros(len(u), bool)
    for j in np.flatnonzero(okc):
        a = agree[:, j]
        cls[j] = int(np.bincount(CL[a, j], minlength=5).argmax())
        clip[j] = bool(CP[a, j].sum() * 2 > a.sum())
    X, Y = g.floor(u, np.where(okc, med, g.H - 0.5))
    fin = okc & np.isfinite(X)
    r_cam = np.hypot(X - g.pos[0], Y - g.pos[1])
    wall = fin & (cls == V.WALL) & ~clip & (np.hypot(X, Y) < max_mm)
    Xw, Yw = X[wall], Y[wall]
    last = es[-1]
    if len(last.behind[0]):                          # wall bases seen over a limitation (from inside the lot)
        bx, by = last.behind[0], last.behind[1]
        k = np.hypot(bx, by) < max_mm
        Xw, Yw = np.concatenate([Xw, bx[k]]), np.concatenate([Yw, by[k]])
    mag = fin & (cls == V.MAGENTA) & ~clip & (r_cam < 1600.0)
    cb = fin & (cls == V.WALL) & clip
    signs = []
    if bgr is not None:
        for q in V.pillars(bgr, g, vp):
            if q["colour"] == "magenta" or (q["touches_bottom"] and q.get("est") != "top") or q["X"] > 1900.0:
                continue
            signs.append((float(q["X"]), float(q["Y"]), q["colour"]))
    return View(t=float(t), odo=tuple(float(q) for q in odo), src="camera", sensor_x=float(g.pos[0]),
                X=Xw, Y=Yw, sig=L.floor_sigma(Xw, Yw), magenta=np.stack([X[mag], Y[mag]], 1),
                signs=signs, raw=None,
                clip_bearings=np.degrees(np.arctan2(Y[cb] - g.pos[1], X[cb] - g.pos[0])), edges=last)


def camera_view(robot, g, vp, odo=(0.0, 0.0, 0.0), frames: int = 5, max_wait_s: float = 4.0, stop=None,
                still_s: float = 0.3, max_mm: float = 2600.0, tick=None):
    """A STILL camera view (4.3): wait until the car has been still for still_s (gyro < 0.8 deg/s, thumbnail
    difference < 1.0, no drive command, v_odo ~ 0), then `frames` NEW frames; a frame that is not still restarts the
    view.  None after max_wait_s ("the car is moving -- hold it still").  g: the program's vision.Ground (its fitted
    pitch), built from the first frame when None.  tick: called every pass (the caller's recorder)."""
    from . import vision as V
    t0 = time.monotonic()
    last_t, prev = None, None
    still_since = None
    es, bgr_last, t_last = [], None, 0.0
    while time.monotonic() - t0 < max_wait_s:
        if stop is not None and stop.is_set():
            return None
        if tick is not None:
            tick()
        bgr, tf = robot.frame()
        if bgr is None or tf == last_t:
            time.sleep(0.01)
            continue
        last_t = tf
        ok, prev, _d, _g = _still_cam(robot, prev, bgr)
        now = time.monotonic()
        if not ok:
            still_since, es = None, []
            continue
        still_since = still_since or now
        if now - still_since < still_s:
            continue
        if g is None:
            g = V.Ground(robot.p["camera"], bgr.shape[1], bgr.shape[0])
        es.append(V.edges(bgr, g, vp, tf))
        bgr_last, t_last = bgr, tf
        if len(es) >= frames:
            return view_from_edges(es, g, odo, t_last, bgr_last, vp, max_mm)
    return None


class _MedScan:
    """A lidar.Scan-alike: the per-bin median of several revolutions."""

    def __init__(self, t, res, dist, conf, rpm=600.0):
        self.t, self.res, self.dist, self.conf, self.rpm = t, res, dist, conf, rpm

    def angles(self):
        return -180.0 + (np.arange(len(self.dist)) + 0.5) * self.res


def lidar_view(robot, lidar_cfg=None, odo=(0.0, 0.0, 0.0), scans: int = 3, max_wait_s: float = 2.0, stop=None,
               start_near_mm: float = 300.0, still_s: float = 0.3, tick=None):
    """A STILL lidar view (4.3, lidar.loc = 1): `scans` revolutions while still, per bin the median range; the
    pose-free perception's wall points (not signs, clutter or pose-free limitations); returns within start_near_mm AHEAD
    dropped (the lot's front limitation when the car starts parked, as wro_next SETTLE).  lidar_cfg: lidar_perc.config
    (None = from robot.p).  None after max_wait_s."""
    from .tests_cal import _gyro_dps
    cfg = lidar_cfg if (lidar_cfg and "pos" in lidar_cfg) else LP.config(robot.p)
    t0 = time.monotonic()
    got, last_t, still_since = [], None, None
    while time.monotonic() - t0 < max_wait_s:
        if stop is not None and stop.is_set():
            return None
        if tick is not None:
            tick()
        sc = robot.scan
        g = _gyro_dps(robot)
        still = (g is None or g <= 0.8) and abs(float(robot.v_odo())) < 0.02 and \
            abs(float(getattr(robot, "v_cmd", 0.0))) == 0.0
        now = time.monotonic()
        if not still:
            got, still_since = [], None
        else:
            still_since = still_since or now
        if sc is not None and sc.t != last_t:
            last_t = sc.t
            if still and now - still_since >= still_s:
                got.append(sc)
                if len(got) >= scans:
                    break
        time.sleep(0.01)
    if len(got) < scans:
        return None
    D = np.stack([np.asarray(s.dist, float) for s in got])
    Dm = np.where(D > 0, D, np.nan)
    with np.errstate(all="ignore"):
        med = np.nanmedian(Dm, axis=0)
    valid = (D > 0).sum(0) * 2 > len(got)
    dist = np.where(valid & np.isfinite(med), np.rint(med), 0).astype(np.int32)
    raw = _MedScan(float(got[-1].t), float(got[-1].res), dist, np.asarray(got[-1].conf), float(got[-1].rpm or 600.0))
    return view_from_scan(raw, cfg, odo, start_near_mm)


def view_from_scan(scan, cfg, odo=(0.0, 0.0, 0.0), start_near_mm: float = 300.0) -> View:
    """A lidar View from one (median) scan -- lidar_view's second half, for tests and replays."""
    pr = LP.perceive(scan, cfg)
    px, py = cfg["pos"]
    if pr is None:
        X = Y = np.zeros(0)
        signs = []
    else:
        m = ~np.isin(pr.cls, (LP.CLS_PILLAR, LP.CLS_CLUTTER, LP.CLS_LIMITER))
        # a cluster spanning < 120 mm is no wall the search can use: a sign the pose-free classifier did not name
        # (one return, a half-hidden face) costs up to 9 per column at the TRUE pose (0.26 / column on a correct
        # WLtoys fix, the good threshold 0.30) -- walls seen through the corridor are longer
        lab = pr.lab
        for c_ in np.unique(lab[m]):
            if c_ < 0:
                continue
            k_ = np.flatnonzero(lab == c_)              # (a cluster may wrap round the scan: its bounding box)
            if math.hypot(float(np.ptp(pr.X[k_])), float(np.ptp(pr.Y[k_]))) < 120.0:
                m[k_] = False
        X, Y = pr.X[m].astype(float), pr.Y[m].astype(float)
        signs = [(float(q[0]), float(q[1]), None) for q in pr.pillars]
    r = np.hypot(X - px, Y - py)
    b = np.degrees(np.arctan2(Y - py, X - px))
    k = ~((r < start_near_mm) & (np.abs(b) < 75.0)) if start_near_mm > 0 else np.ones(len(X), bool)
    X, Y, r = X[k], Y[k], r[k]
    return View(t=float(scan.t), odo=tuple(float(q) for q in odo), src="lidar", sensor_x=float(px), X=X, Y=Y,
                sig=12.0 + 0.012 * r, magenta=np.zeros((0, 2)), signs=signs, raw=scan, clip_bearings=np.zeros(0),
                blind=tuple(tuple(b) for b in cfg["blind"]))


# ------------------------------------------------------------------------------------------------------------ search
def _grid_nms(G, C, limit, cfg, sym, keep_n):
    """Candidate LOCATE-origin poses: the cheapest pose per (nms_mm x nms_mm x nms_deg) cell, then greedy non-maximum
    suppression in cost order, <= keep_n, cost <= limit."""
    nm, nd = float(cfg["nms_mm"]), math.radians(float(cfg["nms_deg"]))
    sel = np.flatnonzero(C <= limit)
    if not len(sel):
        sel = np.array([int(np.argmin(C))])
    Gs, Cs = G[sel], C[sel]
    key = (np.floor((Gs[:, 0] + 2000.0) / nm).astype(np.int64) * 100000
           + np.floor((Gs[:, 1] + 2000.0) / nm).astype(np.int64) * 100
           + np.floor((Gs[:, 2] + 4.0) / nd).astype(np.int64))
    o = np.lexsort((Cs, key))
    first = np.ones(len(o), bool)
    first[1:] = key[o][1:] != key[o][:-1]
    cand = o[first]
    cand = cand[np.argsort(Cs[cand], kind="stable")]
    kept = []
    for i in cand.tolist():
        q = Gs[i]
        if any(math.hypot(q[0] - k[0], q[1] - k[1]) < nm and abs(_wrap(q[2] - k[2])) < nd for k, _c in kept):
            continue
        kept.append((q.copy(), float(Cs[i])))
        if len(kept) >= keep_n:
            break
    return kept


def _canon(p, sym):
    """The twin of pose p whose heading lies in the canonical range ([-45, 45) for sym 4, [-90, 90) for sym 2)."""
    if sym == 4:
        k = int(math.floor((math.degrees(p[2]) + 45.0) / 90.0)) % 4
        return F.rot_pose(p, -k) if k else tuple(p)
    if sym == 2:
        return F.rot_pose(p, 2) if abs(math.degrees(p[2])) >= 90.0 else tuple(p)
    return tuple(p)


def _refine(P0, vw, island, step, sx_stage=None):
    """Two-stage local grid refinement of one LOCATE-origin pose: +-step at step/4 and +-1 deg at 0.25, then +-10 mm
    at 2 and +-0.3 deg at 0.1.  Returns (pose, cost, evaluated poses, their costs)."""
    evP, evC = [], []
    best = np.asarray(P0, float)
    for half, dxy, hth, dth in ((step, step / 4.0, 1.0, 0.25), (10.0, 2.0, 0.3, 0.1)):
        a = np.arange(-half, half + 1e-6, dxy)
        b = np.radians(np.arange(-hth, hth + 1e-9, dth))
        R = np.array(np.meshgrid(best[0] + a, best[1] + a, best[2] + b, indexing="ij")).reshape(3, -1).T
        ok = _valid_all(R, vw, island)
        if not ok.any():
            break
        R = R[ok]
        c = cost_poses(R, vw, island)
        evP.append(R)
        evC.append(c)
        best = R[int(np.argmin(c))]
    if not evP:
        return best, float(cost_poses(best[None, :], vw, island)[0]), np.zeros((0, 3)), np.zeros(0)
    Pall, Call = np.vstack(evP), np.concatenate(evC)
    return best, float(Call.min()), Pall, Call


def _lidar_refine(P0, vw, walls, name, island):
    """A lidar class's millimetres (4.4 step 7): the mean truncated squared likelihood-field distance of every lidar
    wall point over a 5 mm / 0.2 deg local grid (+-15 mm, +-0.6 deg).  The ray-cast found the basin.  Not the median:
    with most returns on the walls of one direction the median ignores the other and slid a correct fix 9 mm along
    it (ray-cast cost 0.07 -> 0.33 per column, sim)."""
    pts = [(v["X"], v["Y"], v["odo"]) for v in vw if v["src"] == "lidar"]

    def df(fx, fy):
        return wall_dist(fx, fy, island)
    if not pts:
        return np.asarray(P0, float)
    a = np.arange(-15.0, 15.1, 5.0)
    b = np.radians(np.arange(-0.6, 0.61, 0.2))
    R = np.array(np.meshgrid(P0[0] + a, P0[1] + a, P0[2] + b, indexing="ij")).reshape(3, -1).T
    R = R[_valid_all(R, vw, island)]
    if not len(R):
        return np.asarray(P0, float)
    cost = np.zeros(len(R))
    for X, Y, o in pts:
        k = np.linspace(0, len(X) - 1, min(len(X), 240)).astype(int)
        Xs, Ys = X[k], Y[k]
        Q = _compose_many(R, o)
        c, s = np.cos(Q[:, 2])[:, None], np.sin(Q[:, 2])[:, None]
        fx = Q[:, 0:1] + c * Xs[None, :] - s * Ys[None, :]
        fy = Q[:, 1:2] + s * Xs[None, :] + c * Ys[None, :]
        cost += np.mean(np.minimum(df(fx, fy), 30.0) ** 2, axis=1)
    return R[int(np.argmin(cost))]


def _search_layout(vw, name, walls, sym, island, cfg, heading, cols):
    """The whole search on one layout -> dict(classes=[(p0, C)], spread=(sxy, sth), ...)."""
    T = max(1.0, cols / float(cfg["temper_k"]))
    step = float(cfg["step_mm"])
    # ---- 2 headings
    if heading is not None:
        ths = heading + np.arange(-float(cfg["heading_win_deg"]), float(cfg["heading_win_deg"]) + 1e-6, 0.5)
        mode = "given"
    else:
        alpha, strength, plen = _heading_from_prepped(vw)
        if strength >= 0.35 and plen >= 400.0:
            ths = -alpha + np.arange(-float(cfg["heading_win_deg"]), float(cfg["heading_win_deg"]) + 1e-6, 0.5)
            mode = "walls"
        else:
            ths = np.arange(-45.0, 45.0, float(cfg["full_step_deg"]))
            mode = "full"
    ths = (np.asarray(ths) + 45.0) % 90.0 - 45.0                         # one quadrant
    if sym == 2:
        ths = np.concatenate([ths, ths + 90.0])
    elif sym == 1:
        ths = np.concatenate([ths + 90.0 * k for k in range(4)])
    ths = np.radians((ths + 180.0) % 360.0 - 180.0)
    # ---- 3 positions, 4 cost
    g = np.arange(-1440.0, 1440.0 + 1e-6, step)
    G = np.array(np.meshgrid(g, g, ths, indexing="ij")).reshape(3, -1).T
    G = G[_valid_all(G, vw, island)]
    if not len(G):
        return None
    C = cost_poses(G, vw, island)
    cb = float(C.min())
    # ---- 5 candidates: generous before the refinement (an off-grid basin reads up to ~0.5 / column worse)
    cands = _grid_nms(G, C, cb + 12.0 * T + 0.5 * cols, cfg, sym, 2 * int(cfg["max_hyps"]))
    # LOOK's guard: the good set wider than max_hyps (a view of one wall leaves a LINE of equal poses: the true one
    # was the 11th of them at a corner start, sim seed 1) -- coarse poses are enough for a footprint check
    guard = [tuple(float(v) for v in q) for q, _c in _grid_nms(G, C, cb + 12.0 * T + 0.5 * cols, cfg, sym, 40)]
    # ---- 6 refine, canonical quadrant, suppress again
    ref = []
    evals = []
    for q, _c in cands:
        b, cst, Pe, Ce = _refine(q, vw, island, step)
        if any(v["src"] == "lidar" for v in vw):
            b2 = _lidar_refine(b, vw, walls, name, island)
            c2 = float(cost_poses(b2[None, :], vw, island)[0])
            if c2 <= cst + T:                       # the likelihood field's millimetres, unless it left the basin
                b, cst = b2, c2
        ref.append((np.asarray(_canon(tuple(b), sym), float), cst))
        evals.append((Pe, Ce, np.asarray(b, float), cst))
    order = sorted(range(len(ref)), key=lambda i: ref[i][1])
    nm, nd = float(cfg["nms_mm"]), math.radians(float(cfg["nms_deg"]))
    classes, cls_ev = [], []
    for i in order:
        p0, c0 = ref[i]
        if c0 > ref[order[0]][1] + 12.0 * T:
            break
        dup = False
        for k in range(len(classes)):
            q = classes[k][0]
            for kk in range(sym):                   # a class found at +44.9 and at -45.1 deg is one
                qq = F.rot_pose(q, kk) if kk else q
                if math.hypot(p0[0] - qq[0], p0[1] - qq[1]) < nm and abs(_wrap(p0[2] - qq[2])) < nd:
                    dup = True
                    break
            if dup:
                break
        if dup:
            continue
        classes.append((tuple(float(v) for v in p0), c0))
        cls_ev.append(evals[i])
        if len(classes) >= int(cfg["max_hyps"]):
            break
    # ---- spread of the best class: every evaluated pose within 2 T of its cost (weights exp(-dC / 2T)), measured
    #      around the refined pose in the frame it was refined in (canonicalising the grid would split it at +-45 deg)
    Pe, Ce, braw, craw = cls_ev[0]
    c0 = classes[0][1]
    sxy, sth = 10.0, math.radians(0.5)
    if len(Pe):
        dC = Ce - craw
        m = dC <= 2.0 * T
        if m.sum() >= 2:
            w = np.exp(-dC[m] / (2.0 * T))
            w /= w.sum()
            dx, dy = Pe[m, 0] - braw[0], Pe[m, 1] - braw[1]
            dt = (Pe[m, 2] - braw[2] + math.pi) % (2 * math.pi) - math.pi
            sxy = max(10.0, math.sqrt(float(w @ (dx * dx + dy * dy))))
            sth = max(math.radians(0.5), math.sqrt(float(w @ (dt * dt))))
    # the basin beyond the refinement's +-40 mm: a view of ONE wall at an angle is flat along it, and the grid above
    # only measures +-40 mm of that -- a confident fix (p 1.0, sxy 29) stood 62 mm off (sim, A1, seed 3 marked).
    # The field's walls are axis-aligned, so the flat directions are x and y: probe them.  The tolerance is a FIXED
    # cost (2T of one 48-column view), not 2T: T grows with every view, and a direction only one view pins then read
    # flat (7-15 mm fixes refused).  Measured on 120 single camera views (sim, 2026-09-25): the 22 fixes with
    # p >= 0.95 were 1-18 mm off and fit within 12 at most +-60 mm -- "wide" starts at +-120
    tol_b = min(2.0 * T, 12.0)

    def basin_of(q, cq):
        bw = 0.0
        for dd in (60.0, 120.0, 240.0):
            pr = np.array([[q[0] + dd, q[1], q[2]], [q[0] - dd, q[1], q[2]], [q[0], q[1] + dd, q[2]],
                           [q[0], q[1] - dd, q[2]]])
            okp = _valid_all(pr, vw, island)
            if not okp.any():
                break
            if (cost_poses(pr[okp], vw, island) <= cq + tol_b).any():
                bw = dd
            else:
                break
        return bw
    basin = basin_of(braw, craw)
    basins = [basin] + [basin_of(np.asarray(q, float), cq) for q, cq in classes[1:]]   # per class (the mark may
    #                                                                                     pick another: resolve_frame)
    if basin > 0.0:
        sxy = max(sxy, basin / math.sqrt(3.0))                   # a flat run of +-basin: at least its uniform spread
    cc_all = col_costs(classes[0][0], vw, island)
    return dict(classes=classes, T=T, sxy=sxy, sth=sth, mode=mode, n_eval=int(len(G)), sym=sym, island=island,
                basin=basin, basins=basins,
                guard=guard, name=name, cost_col=fit_cost(classes[0][0], vw, island),
                cost_all=float(cc_all.mean()) if len(cc_all) else math.inf)


def _heading_from_prepped(vw):
    vs = [View(t=0.0, odo=v["odo"], src=v["src"], sensor_x=v["sx"], X=v["X"], Y=v["Y"], sig=np.zeros(0)) for v in vw]
    return _heading(vs)


def search(views, layout: str = "standard", cfg: dict | None = None, heading=None) -> Fix:
    """The whole field, every heading (4.4) -> a Fix with the frame unresolved (frame_by "none", the twin k = 0).
    layout "standard" | "auto" (P1: the standard, then the 16 Open layouts when the standard does not fit).
    heading: a known LOCATE-origin heading (deg) to search +- heading_win_deg around, instead of the walls'."""
    c = dict(CFG, **(cfg or {}))
    t0 = time.perf_counter()
    views = [v for v in views if v is not None]
    vw = _prep(views)
    cols = int(sum(len(v["az"]) for v in vw))
    cov = coverage_deg(views)
    src = "+".join(sorted({v.src for v in views})) or "none"
    odo_last = tuple(float(q) for q in views[-1].odo) if views else (0.0, 0.0, 0.0)
    lays = layouts("open" if layout in ("auto", "open") else "obstacle")
    empty = Fix(pose=(0.0, 0.0, 0.0), sxy=math.inf, sth=math.inf, p=0.0, cost=math.inf, hyps=[], twins=[],
                frame_k=None, frame_by="none", layout="unknown", cols=cols, cov_deg=cov, views=len(views), src=src,
                ms=0.0, confident=False, not_standard=False, why="no wall points in view", odo_last=odo_last)
    if cols < 6:
        empty.ms = (time.perf_counter() - t0) * 1000.0
        empty.why = "no view" if not views else "too few wall points: %d" % cols
        return empty
    names = ["standard"]
    res = {}
    r0 = _search_layout(vw, "standard", *lays["standard"], c, heading, cols)
    if r0 is not None:
        res["standard"] = r0
    if layout in ("auto", "open") and (r0 is None or r0["cost_all"] > float(c["good"])):
        for nm_ in lays:
            if nm_ == "standard":
                continue
            rr = _search_layout(vw, nm_, *lays[nm_], c, heading, cols)
            if rr is not None:
                res[nm_] = rr
            names.append(nm_)
    if not res:
        empty.ms = (time.perf_counter() - t0) * 1000.0
        empty.why = "no pose where the car could stand fits the view"
        return empty
    ranked = sorted(res.values(), key=lambda r: r["cost_all"])
    best = ranked[0]
    chosen = best
    if best["name"] != "standard" and "standard" in res:
        runner = ranked[1]["cost_all"] if len(ranked) > 1 else math.inf
        if not (best["cost_all"] <= float(c["good"]) and best["cost_all"] <= 0.6 * runner):
            chosen = res["standard"]            # no Open layout wins clearly: judge the standard
    classes, T = chosen["classes"], chosen["T"]
    # the signs stand only on the 24 seats [RULE]: a class that puts a detected sign far from every seat is less likely
    # (one column's worth per sign, capped at 9 like a wall column); twins map seats to seats, so the frame is untouched
    Cs = np.array([cc + sign_cost(pc, views)[0] for pc, cc in classes])
    order = np.argsort(Cs, kind="stable")
    sxy_c, sth_c = chosen["sxy"], chosen["sth"]
    if int(order[0]) != 0:                  # the spread was measured for the ray-cast's best: not this class's
        sxy_c, sth_c = max(sxy_c, 25.0), max(sth_c, math.radians(1.0))
    classes = [classes[i] for i in order]
    Cs = Cs[order]
    w = np.exp(-(Cs - Cs.min()) / (2.0 * T))
    ps = (w / w.sum()).tolist()
    p0s = [p for p, _cc in classes]
    classes = [(pc, float(c_)) for (pc, _cc), c_ in zip(classes, Cs)]
    now = [compose(p, odo_last) for p in p0s]
    sym, isl = chosen["sym"], chosen["island"]
    b = now[0]
    twins = [F.rot_pose(b, k * (4 // sym)) for k in range(sym)]
    cost_col = fit_cost(p0s[0], vw, isl)
    cc_all = col_costs(p0s[0], vw, isl)
    cost_all = float(cc_all.mean()) if len(cc_all) else math.inf
    enough = cols >= int(c["min_cols"]) and cov >= float(c["min_cov_deg"])
    std_all = res["standard"]["cost_all"] if "standard" in res else math.inf
    opens = sorted((r["cost_all"], k) for k, r in res.items() if k != "standard")
    # not the standard field: enough view and the standard fits nowhere (untrimmed >= bad), or an Open layout fits
    # (<= good) where the standard does not (> good, and at least twice as costly) -- several Open layouts may fit one
    # view equally (a corridor out of sight): the field is still not the standard one
    not_std = enough and (std_all >= float(c["bad"]) or chosen["name"] != "standard" or
                          (bool(opens) and opens[0][0] <= float(c["good"]) and std_all > float(c["good"])
                           and std_all >= 2.0 * opens[0][0]))
    why = []
    if cols < int(c["min_cols"]):
        why.append("too few wall columns: %d < %d" % (cols, int(c["min_cols"])))
    if cov < float(c["min_cov_deg"]):
        why.append("one wall only: %.0f deg" % cov)
    if cost_col > float(c["good"]):
        why.append("cost %.2f > %.2f" % (cost_col, float(c["good"])))
    elif cost_all > 2.0 * float(c["good"]):
        # the trim forgives a few unmapped objects (a limitation: <= ~0.56 untrimmed); more columns off the map is a
        # map that does not fit -- an Open field read 0.24 trimmed and 1.16 untrimmed, and drove lost (sim, 2026-09-25)
        why.append("cost %.2f with every column > %.2f" % (cost_all, 2.0 * float(c["good"])))
    if ps[0] < float(c["min_p"]):
        why.append("%d hypotheses: p %.2f" % (len(classes), ps[0]))
    basins = [chosen["basins"][i] for i in order] if chosen.get("basins") else []
    if basins and basins[0] >= 120.0:
        why.append("basin wide: +-%.0f mm fit as well" % basins[0])
    base_ok = not why and chosen["name"] == "standard"
    if chosen["name"] != "standard":
        why.insert(0, "the walls fit %s, not the standard field" % chosen["name"])
    fx = Fix(pose=twins[0], sxy=sxy_c, sth=sth_c, p=float(ps[0]), cost=cost_col,
             hyps=[(q[0], q[1], q[2], float(pp), float(cc) / max(1, cols)) for q, pp, (_p, cc) in zip(now, ps, classes)],
             twins=twins, frame_k=None, frame_by="none", layout=chosen["name"], cols=cols, cov_deg=cov,
             views=len(views), src=src, ms=(time.perf_counter() - t0) * 1000.0, confident=base_ok,
             not_standard=bool(not_std), why="; ".join(why), p0=p0s, ps=ps, odo_last=odo_last, sym=sym, island=isl,
             base_ok=base_ok, base_why="; ".join(why), layouts={k: round(v["cost_all"], 3) for k, v in res.items()},
             guard=list(chosen.get("guard") or []), basins=basins)
    return fx


# ------------------------------------------------------------------------------------------------------------ frame
def _twin_p0(fix, k_idx):
    """The LOCATE-origin pose of the best class in twin k_idx."""
    return F.rot_pose(fix.p0[0], k_idx * (4 // max(1, fix.sym)))


def _lot_votes_camera(fix, views, cfg):
    """Per twin: (in the lot band, total) magenta bases."""
    out = []
    for ki in range(len(fix.twins)):
        p0 = _twin_p0(fix, ki)
        n_in = n_all = 0
        for v in views:
            if v.magenta is None or not len(v.magenta):
                continue
            pv = compose(p0, v.odo)
            fx, fy = LP.to_field(v.magenta[:, 0], v.magenta[:, 1], pv)
            n_all += len(fx)
            n_in += int(np.sum((fy > LOT_Y[0]) & (fy < LOT_Y[1]) & (np.abs(fx) < LOT_X)))
        out.append((n_in, n_all))
    return out


def _lot_votes_lidar(fix, views, cfg):
    """Per twin: the lidar's returns > 45 mm off every map wall that land in S's lot band >= 15 mm off the wall face."""
    out = []
    for ki in range(len(fix.twins)):
        p0 = _twin_p0(fix, ki)
        n = 0
        for v in views:
            if v.src != "lidar" or v.raw is None:
                continue
            X, Y, _s = L.Localizer.lidar_points(v.raw, (v.sensor_x, 0.0), v.blind or (), 1.0, rmin=60.0,
                                                rmax=3000.0)
            pv = compose(p0, v.odo)
            fx, fy = LP.to_field(X, Y, pv)
            off = wall_dist(fx, fy, fix.island) > 45.0
            n += int(np.sum(off & (fy > LOT_Y[0] + LP.LOT_OFF_WALL) & (fy < LOT_Y[1]) & (np.abs(fx) < LOT_X)))
        out.append(n)
    return out


def _survey_votes(fix, views, survey):
    """Per twin: the survey's confirmed signs matched by the views' sign detections (120 mm, colour agrees)."""
    sg = [(float(s["x"]), float(s["y"]), s.get("colour")) for s in (survey or {}).get("seats", [])
          if s.get("exists") is True]
    out = []
    for ki in range(len(fix.twins)):
        p0 = _twin_p0(fix, ki)
        n = 0
        for v in views:
            pv = compose(p0, v.odo)
            for X, Y, col in v.signs:
                fx, fy = LP.to_field(np.array([X]), np.array([Y]), pv)
                for sx, sy, scol in sg:
                    if math.hypot(fx[0] - sx, fy[0] - sy) < SIGN_MATCH_MM and (col is None or scol is None or
                                                                                col == scol):
                        n += 1
                        break
        out.append(n)
    return out


def at_mark(p0, expect, sym: int = 4, mm: float = CFG["expect_mm"], deg: float = CFG["expect_deg"]) -> bool:
    """Is some twin of the LOCATE-origin pose p0 within mm / deg of the start mark expect [x, y, deg]?"""
    ex = (float(expect[0]), float(expect[1]), math.radians(float(expect[2])))
    for k in range(sym):
        q = F.rot_pose(p0, k * (4 // max(1, sym)))
        if math.hypot(q[0] - ex[0], q[1] - ex[1]) <= mm and abs(math.degrees(_wrap(q[2] - ex[2]))) <= deg:
            return True
    return False


def _decide_max(votes, need, lead):
    """The index whose vote reaches `need` and beats every other by `lead`; None."""
    if not votes:
        return None
    i = int(np.argmax(votes))
    if votes[i] < need:
        return None
    others = [v for j, v in enumerate(votes) if j != i]
    return i if not others or votes[i] >= max(others) + lead else None


def resolve_frame(fix: Fix, views, policy: str = "auto", needed: bool = True, expect=None, survey=None,
                  cfg: dict | None = None) -> Fix:
    """Which twin is the field frame (4.6).  Sources: lot (camera magenta bases / lidar off-wall returns in S's lot
    band), survey (its signs; its lot), expect (the start mark [x, y, deg]), start (the pose in a straight -> the twin
    that puts it in S).  Policy "auto": lot > survey > expect, and two of them deciding differently = not confident
    ("frame sources disagree"); start only when none of the three decides.  needed=False: laps do not care -- the
    evidence is still used when it decides, else the start twin (in a straight) or k = 0, frame_by "any"."""
    c = dict(CFG, **(cfg or {}))
    if not fix.twins:
        return fix
    views = [v for v in views if v is not None]
    if expect is not None and len(expect) >= 3 and len(fix.p0) > 1 and fix.ps[0] < 0.5 and \
            not at_mark(fix.p0[0], expect, fix.sym, float(c["expect_mm"]), float(c["expect_deg"])):
        # the camera cannot tell its classes apart (one wall in view: 8 near-equal classes, the "best" one 180 deg
        # off the mark C -- sim 2026-09-25) and the car was PUT on the mark: the best class AT the mark leads.  A camera
        # sure of another place (p >= 0.5) is not overruled: that is a start_mismatch
        at = [i for i, q in enumerate(fix.p0) if at_mark(q, expect, fix.sym, float(c["expect_mm"]),
                                                          float(c["expect_deg"]))]
        if at:
            i = at[0]
            order = [i] + [j for j in range(len(fix.p0)) if j != i]
            nf = Fix(**{k: getattr(fix, k) for k in fix.__dataclass_fields__})
            nf.p0 = [fix.p0[j] for j in order]
            nf.ps = [fix.ps[j] for j in order]
            nf.hyps = [fix.hyps[j] for j in order]
            if fix.basins:
                nf.basins = [fix.basins[j] for j in order]
                keep_w = [w_ for w_ in fix.base_why.split("; ") if w_ and not w_.startswith("basin wide")]
                if nf.basins[0] >= 120.0:
                    keep_w.append("basin wide: +-%.0f mm fit as well" % nf.basins[0])
                nf.base_why = "; ".join(keep_w)
                nf.base_ok = not keep_w and fix.layout == "standard"
            b = compose(nf.p0[0], fix.odo_last)
            nf.twins = [F.rot_pose(b, k * (4 // max(1, fix.sym))) for k in range(len(fix.twins))]
            nf.pose, nf.p = nf.twins[0], nf.ps[0]
            nf.sxy, nf.sth = max(fix.sxy, 25.0), max(fix.sth, math.radians(1.0))   # the spread was the old best's
            fix = nf
    n = len(fix.twins)
    dec = []                                                   # [(source, k)]
    # lot, camera
    lc = _lot_votes_camera(fix, views, c)
    ins = [a for a, _t in lc]
    kc = _decide_max(ins, int(c["lot_min_pts"]), 1)
    if kc is not None and ins[kc] >= 0.8 * max(1, lc[kc][1]):
        dec.append(("lot", kc))
    # lot, lidar
    if any(v.src == "lidar" for v in views):
        ll = _lot_votes_lidar(fix, views, c)
        kl = _decide_max(ll, int(c["lot_min_pts"]), int(c["lot_min_pts"]) // 2)
        if kl is not None and not any(s == "lot" for s, _k in dec):
            dec.append(("lot", kl))
        elif kl is not None and any(s == "lot" and k != kl for s, k in dec):
            dec.append(("lot", kl))                             # camera and lidar lot disagree: reported below
    # survey
    if survey:
        sv = _survey_votes(fix, views, survey)
        ks = _decide_max(sv, 2, 2)
        if ks is not None:
            dec.append(("survey", ks))
    # expect
    sc = None
    if expect is not None and len(expect) >= 3:
        ex = (float(expect[0]), float(expect[1]), math.radians(float(expect[2])))
        best = None
        for ki in range(n):
            q = _twin_p0(fix, ki)
            dd = math.hypot(q[0] - ex[0], q[1] - ex[1])
            dg = abs(math.degrees(_wrap(q[2] - ex[2])))
            if best is None or dd + 10.0 * dg < best[1] + 10.0 * best[2]:
                best = (ki, dd, dg, q)
        ki, dd, dg, q = best
        ok = dd <= float(c["expect_mm"]) and dg <= float(c["expect_deg"])
        sc = dict(ev="start_check", expect=[round(ex[0]), round(ex[1]), round(math.degrees(ex[2]), 1)],
                  found=[round(q[0]), round(q[1]), round(math.degrees(q[2]), 1)], d_mm=round(dd), d_deg=round(dg, 1),
                  ok=bool(ok))
        if ok:
            dec.append(("expect", ki))
    ks_ = [k for _s, k in dec]
    disagree = len(set(ks_)) > 1
    frame_k, frame_by, fwhy = None, "none", ""
    pol = str(policy or "auto")
    if pol in ("lot", "survey", "expect"):
        mine = [k for s, k in dec if s == pol]
        if mine and len(set(mine)) == 1:
            frame_k, frame_by = mine[0], pol
        else:
            fwhy = "frame: %s not seen" % pol if pol != "expect" else "frame: no start mark"
    elif pol == "any":
        frame_k, frame_by = 0, "any"
    elif disagree:
        fwhy = "frame sources disagree (%s)" % ", ".join("%s k%d" % (s, k) for s, k in dec)
    elif dec:
        frame_k, frame_by = dec[0][1], dec[0][0]
    if frame_k is None and not disagree and pol in ("auto", "start"):
        p0s = [_twin_p0(fix, ki) for ki in range(n)]
        ins_s = [ki for ki, q in enumerate(p0s) if F.where(q[0], q[1]) == ("straight", 0)]
        if len(ins_s) == 1:
            frame_k, frame_by = ins_s[0], "start"
            fwhy = ""
        elif pol == "start":
            fwhy = "frame: the start pose is not in a straight"
    if frame_k is None and not needed and not disagree:
        frame_k, frame_by, fwhy = 0, "any", ""
    if frame_k is None and not fwhy:
        fwhy = "frame: lot not seen" if needed else ""
    if sc is not None and not sc["ok"]:
        fwhy = "start mark %d mm / %.0f deg off" % (sc["d_mm"], sc["d_deg"])
    out = Fix(**{k: getattr(fix, k) for k in fix.__dataclass_fields__})
    out.start_check = sc
    if sc is not None and sc["ok"] and len(fix.p0) > 1:
        # the car was PUT on the mark: a class no twin of which stands at the mark contradicts that -- p over the
        # classes at the mark only (the camera's best must be one of them: else start_check failed above)
        keep = [at_mark(q, expect, fix.sym, float(c["expect_mm"]), float(c["expect_deg"])) for q in fix.p0]
        ps2 = [pp if kp else 0.0 for pp, kp in zip(fix.ps, keep)]
        tot = sum(ps2)
        if tot > 0 and keep[0]:
            out.ps = [pp / tot for pp in ps2]
            out.p = out.ps[0]
            out.hyps = [(h[0], h[1], h[2], pp, h[4]) for h, pp in zip(fix.hyps, out.ps)]
            fix = out
            other = [w_ for w_ in fix.base_why.split("; ") if w_ and "hypotheses" not in w_]
            if out.p < float(c["min_p"]):
                other.append("%d hypotheses at the mark: p %.2f" % (sum(keep), out.p))
            out.base_ok = not other and fix.layout == "standard"
            out.base_why = "; ".join(other)
    out.frame_k, out.frame_by = frame_k, frame_by
    kk = frame_k if frame_k is not None else 0
    rot = kk * (4 // max(1, fix.sym))
    out.pose = fix.twins[kk]
    out.hyps = [(*F.rot_pose(h[:3], rot), h[3], h[4]) for h in fix.hyps]
    frame_ok = frame_k is not None and not disagree and (sc is None or sc["ok"])
    if not needed and not disagree and (sc is None or sc["ok"]):
        frame_ok = True
    out.confident = bool(fix.base_ok and frame_ok)
    reasons = [r for r in (fix.base_why, fwhy if (needed or disagree or (sc is not None and not sc["ok"])) else "")
               if r]
    out.why = "; ".join(reasons)
    return out


def seed(loc, fix: Fix, alt: bool = True) -> None:
    """The filter's particles on the chosen twin (sigma = the fix's spread, >= 15 mm / 1 deg); with alt, also every
    other class with p >= 0.02, weighted by p, in the same frame: a genuinely ambiguous fix stays ambiguous there."""
    rot = (fix.frame_k or 0) * (4 // max(1, fix.sym))
    parts = []
    for i, (p0, pp) in enumerate(zip(fix.p0, fix.ps)):
        if i and (not alt or pp < 0.02):
            continue
        q = F.rot_pose(compose(p0, fix.odo_last), rot)
        s = max(15.0, float(fix.sxy)) if i == 0 else 20.0
        sth = max(math.radians(1.0), float(fix.sth)) if i == 0 else math.radians(1.0)
        parts.append((q[0], q[1], q[2], s, s, sth, max(float(pp), 1e-3)))
    if not parts:
        return
    loc.init(parts)


# ------------------------------------------------------------------------------------------------------------ LOOK
def map_points(pose, island=(-F.ISL, F.ISL, -F.ISL, F.ISL), seats=(), reach_mm: float = 1100.0):
    """The map around a field pose as CAR-frame points: every wall face within reach every 20 mm, and each (x, y) in
    `seats` as a 50 x 50 sign outline."""
    from . import shield as SH
    walls = F.layout_walls() if tuple(island) == (-F.ISL, F.ISL, -F.ISL, F.ISL) else None
    if walls is None:
        l, r, b, t = island
        walls = np.array([[-F.HALF, -F.HALF, F.HALF, -F.HALF], [F.HALF, -F.HALF, F.HALF, F.HALF],
                          [F.HALF, F.HALF, -F.HALF, F.HALF], [-F.HALF, F.HALF, -F.HALF, -F.HALF],
                          [l, b, r, b], [r, b, r, t], [r, t, l, t], [l, t, l, b]], float)
    pts = []
    for x1, y1, x2, y2 in walls:
        nn = max(2, int(math.hypot(x2 - x1, y2 - y1) / 20.0) + 1)
        tt = np.linspace(0.0, 1.0, nn)
        pts.append(np.stack([x1 + (x2 - x1) * tt, y1 + (y2 - y1) * tt], 1))
    W = np.vstack(pts)
    x, y = float(pose[0]), float(pose[1])
    W = W[(np.abs(W[:, 0] - x) < reach_mm) & (np.abs(W[:, 1] - y) < reach_mm)]
    parts = [W]
    for sx, sy in seats:
        if abs(sx - x) < reach_mm and abs(sy - y) < reach_mm:
            parts.append(SH.box_pts(sx, sy, F.PILLAR / 2.0, F.PILLAR / 2.0))
    P_ = np.vstack(parts) if parts else np.zeros((0, 2))
    return LP.field_to_car(P_[:, 0], P_[:, 1], pose)


def seen_empty_seats(view: View, pose, near_mm: float = 240.0, hfov_deg: float = 58.6) -> set:
    """Seat indices a camera view saw the mat at (in the field of view, nearer than the wall base behind them by
    60 mm, and no sign detected within 150 mm) -- at the field pose `pose` of that view."""
    out = set()
    if view is None or view.src != "camera" or not len(view.X):
        return out
    Xs, Ys = LP.field_to_car(F.SEAT_XY[:, 0], F.SEAT_XY[:, 1], pose)
    sx = float(view.sensor_x)
    bs = np.degrees(np.arctan2(Ys, Xs - sx))
    rs = np.hypot(Xs - sx, Ys)
    bw = np.degrees(np.arctan2(view.Y, view.X - sx))
    rw = np.hypot(view.X - sx, view.Y)
    for j in range(len(Xs)):
        if abs(bs[j]) > hfov_deg / 2.0 - 2.0 or rs[j] < near_mm:
            continue
        k = np.abs(bw - bs[j]) < 1.5
        if not k.any() or float(np.min(rw[k])) < rs[j] + 60.0:
            continue
        if any(math.hypot(X - Xs[j], Y - Ys[j]) < 150.0 for X, Y, _c in view.signs):
            continue
        out.add(j)
    return out


CLEAR_AHEAD, CLEAR_BESIDE = 550.0, 250.0        # a taped start mark's clear zone [RULE-like, plans/*.json]: the team
#                                                 keeps this far ahead of the nose and beside the body free of signs


def clear_zone_seats(p0, car) -> set:
    """Seat indices inside a start MARK's clear zone (nothing within 550 mm ahead of the nose or 250 mm beside the
    body, as the judges place a car) at the LOCATE origin p0 = where the car stood when it was put on the mark.  Only
    with `expect` (bw mat): without a mark nothing is promised."""
    Xs, Ys = LP.field_to_car(F.SEAT_XY[:, 0], F.SEAT_XY[:, 1], p0)
    rear, front, half = (float(v) for v in car)
    m = (Xs > -rear - CLEAR_BESIDE) & (Xs < front + CLEAR_AHEAD) & (np.abs(Ys) < half + CLEAR_BESIDE)
    return set(np.flatnonzero(m).tolist())


def under_car(pose, car, pad: float = 25.0) -> set:
    """Seat indices whose 50 x 50 sign would overlap the car's footprint (rear, front, half_w) at pose: the car stands
    there, so they are empty."""
    Xs, Ys = LP.field_to_car(F.SEAT_XY[:, 0], F.SEAT_XY[:, 1], pose)
    rear, front, half = (float(v) for v in car)
    m = (Xs > -rear - pad) & (Xs < front + pad) & (np.abs(Ys) < half + pad)
    return set(np.flatnonzero(m).tolist())


class Look:
    """LOOK (4.7): one short move, stop, one more still view -- until the fix is confident, look.max_mm is driven,
    look.views are taken or nothing is free.  Forward on every car; reverse only with the lidar shield (S14: the A1's
    reversing is blind, M9).  A candidate is feasible when the swept footprint (grown by the shield's margins) has
    step + need(v) free among the shield's points AND, when a fix is given, among the map points around each of its
    plausible hypotheses (walls + every seat not seen empty) -- the A1's camera does not see beside the car nor within
    213 mm of its nose."""

    def __init__(self, robot, bot, rp: dict, sensor: str):
        self.robot, self.bot, self.sensor = robot, bot, str(sensor)
        self.rp = rp or {}
        self.lc = look_of(self.rp)
        self.mx = float((self.rp.get("steer") or {}).get("max_deg", 29.0))
        self.wb = float((self.rp.get("chassis") or {}).get("wheelbase_m", 0.145)) * 1000.0
        self.shield = getattr(bot, "_shield", None)
        self.driven_mm, self.moves = 0.0, 0
        self._cur, self._t0, self._dist, self._brk = None, None, 0.0, None
        self.last_plan = None

    def need(self, v: float) -> float:
        if self.shield is not None:
            return float(self.shield.need_mm(v))
        return LP.need_mm(v, 2.2, 0.25, 25.0)

    def _footprint(self):
        sh = self.shield
        if sh is not None:
            return LP.grow(sh.car, float(sh.c["margin_mm"]), float(sh.c["side_mm"]), float(sh.c["margin_mm"])), sh.car
        car = LP.config(self.rp)["car"]
        return LP.grow(car, 30.0, 10.0, 30.0), car

    def plan(self, views, fix: Fix | None = None, now_pose=None, marked: bool = False, mark=None) -> dict | None:
        """The next move {"v", "steer", "mm", "why", "free", "gain"} or None (boxed in / budget spent).  now_pose: the
        car in the LOCATE frame now (default: the last view's).  marked: the car was put on a taped mark (`expect`):
        the seats of the mark's clear zone are empty (clear_zone_seats); mark [x, y, deg]: the poses no twin of which
        stands at the mark are ruled out (at_mark) -- unless none is left (then the mark or the view is wrong)."""
        lc = self.lc
        if self.moves >= int(lc["views"]) - 1 or self.driven_mm >= float(lc["max_mm"]) - 1e-6:
            return None
        v = float(lc["v"])
        step = min(float(lc["step_mm"]), float(lc["max_mm"]) - self.driven_mm)
        need = step + self.need(v)
        fp, body = self._footprint()
        X, Y = (self.shield.points_now() if self.shield is not None else (np.zeros(0), np.zeros(0)))
        groups = [(np.asarray(X, float), np.asarray(Y, float))]
        if fix is not None and fix.p0:
            last = next((v_ for v_ in reversed(views) if v_.src == "camera"), None)
            poses = [p0 for i, (p0, pp) in enumerate(zip(fix.p0, fix.ps)) if i == 0 or pp >= 0.02]
            poses += [q for q in fix.guard if all(math.hypot(q[0] - r[0], q[1] - r[1]) > 60.0 or
                                                  abs(_wrap(q[2] - r[2])) > math.radians(4.0) for r in poses)]
            if mark is not None and len(mark) >= 3:
                at = [q for q in poses if at_mark(q, mark, fix.sym)]
                if at:
                    poses = at
            for p0 in poses:
                q = compose(p0, fix.odo_last)
                empty = seen_empty_seats(last, compose(p0, last.odo)) if last is not None else set()
                empty |= under_car(q, body)
                if marked:
                    empty |= clear_zone_seats(p0, body)
                seats = [(F.SEAT_XY[j, 0], F.SEAT_XY[j, 1]) for j in range(len(F.SEATS)) if j not in empty]
                groups.append(map_points(q, fix.island, seats))
        dirs = [1, -1] if self.sensor == "lidar" else [1]
        st = [-self.mx, -self.mx / 2.0, 0.0, self.mx / 2.0, self.mx]
        # coverage table: 36 x 10 deg of LOCATE-frame bearings the views' wall points already cover
        cov = np.zeros(36, bool)
        b = _bearings_locate(views)
        if len(b):
            cov[(np.floor(b / 10.0) % 36).astype(int)] = True
        th_now = float((now_pose or (views[-1].odo if views else (0.0, 0.0, 0.0)))[2])
        half = 29.3 if self.sensor != "lidar" else 132.0
        best = None
        for d in dirs:
            ks = [math.tan(math.radians(s)) / max(self.wb, 1e-6) for s in st]
            fr = np.full(len(st), np.inf)
            for gx, gy in groups:
                if len(gx):
                    fr = np.minimum(fr, LP.swept_many(gx, gy, fp, ks, d, need + 50.0, body=body))
                else:
                    fr = np.minimum(fr, need + 50.0)
            for s, k, f in zip(st, ks, fr.tolist()):
                if f < need:
                    continue
                th_end = th_now + d * step * k
                lo = math.degrees(th_end) - half
                bins = (np.floor((lo + np.arange(0.0, 2 * half + 1e-6, 5.0)) / 10.0) % 36).astype(int)
                gain = int(np.sum(~cov[np.unique(bins)]))
                keyv = (gain, f, -abs(s))
                if best is None or keyv > best[0]:
                    best = (keyv, dict(v=d * v, steer=float(s), mm=step, free=round(float(f)), gain=gain,
                                       why="+%d x 10 deg unseen" % gain))
        self.last_plan = best[1] if best else None
        return self.last_plan

    def start(self, move: dict) -> None:
        self._cur, self._t0, self._dist, self._brk = dict(move), None, 0.0, None

    def tick(self, now: float, ds_mm: float) -> str:
        """Drive the started move through `bot`: "moving" | "done" | "blocked" (braked 0.3 s by the shield)."""
        m = self._cur
        if m is None:
            return "done"
        if self._t0 is None:
            self._t0 = now
        self._dist += abs(float(ds_mm))
        if self._dist >= float(m["mm"]) or now - self._t0 > float(m["mm"]) / (1000.0 * abs(float(m["v"]))) + 1.5:
            return self._end("done")
        self.bot.drive(float(m["v"]), float(m["steer"]))
        if getattr(self.bot, "last_act", "") in ("brake", "blind"):
            self._brk = self._brk or now
            if now - self._brk >= 0.3:
                return self._end("blocked")
        else:
            self._brk = None
        return "moving"

    def _end(self, what: str) -> str:
        self.bot.drive(0.0, 0.0)
        self.driven_mm += self._dist
        self.moves += 1
        self._cur = None
        return what
