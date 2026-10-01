"""Lidar perception: one LD19 revolution -> wall segments, sign (pillar) candidates, parking limitations and the free
space around the car's body; plus the geometry the safety layer and the fusion share (the swept arc, the braking
distance, the camera-to-lidar association by bearing).

Built for the WLtoys build's 50 mm scan plane (3d/bluewave_body/wl/layout_wl.json: LD19 upright at [152, 0], the body
hides 131.95-228.1 deg), which sees the 100 mm walls, the 50 x 50 signs and the 200 x 20 limitations.  The stock A1's
LD19 at 145.7 mm scans ABOVE the walls (measured 2026-09-22): there the output describes the room, which the console and
SLAM still use; nothing here assumes the field unless a pose is given.

    cfg = config(robot.p)                                  # the numbers perceive() reads
    pr = perceive(robot.scan, cfg)                         # pose-free: clusters, lines, pillar candidates, clearances
    pr = perceive(sc, cfg, pose=(x, y, th), df=loc.df)     # + the field prior: wall / limiter classes, seat support
    pr = of(robot)                                         # the latest scan's pose-free result, computed once per scan

Frames and units (BRAIN_SPEC 2): every output is in the CAR frame -- rear-axle centre on the floor, x forward, y left,
mm; bins as lidar.Scan (bin k at -180 + (k + 0.5) res deg, CCW +, bearings from the lidar).  `pose` is the rear-axle
FIELD pose at the scan's mean time (Scan.t - lidar.scan_lag_s).  Deterministic (no RANSAC, no random draw): a logged
scan gives the same result on the laptop as on the car.

The pipeline (BRAIN_SPEC 4.2; vectorised -- the Python loops run per segment or per candidate, never per point):
    1 valid returns     min_mm < d < perc.max_mm, conf >= lidar.min_conf, bearing outside lidar.block_deg
    7 clutter           a return within 5 mm of the car's own outline (a cable, a hand, the body) -> CLS_CLUTTER, out
    2 clusters          adaptive breakpoint (Borges & Aldon 2004): break when the gap between two consecutive returns
                        exceeds  r sin(dphi) / sin(lambda - dphi) + break_mm  with r the NEARER of the two ranges (the
                        same break whichever way the lidar spins), or when more than 2 bins separate them; the last
                        cluster joins the first when the scan wraps without a gap
    4 pillars           a small cluster (lateral span + half a beam spacing and radial depth <= pillar_max_mm) that is
                        not a grazing wall piece (radial <= 2.5 x lateral + 20 mm), both neighbour bins farther by
                        pillar_gap_mm or empty; one lone intermediate return (the LD19's mixed "ghost" between an edge
                        and the wall behind) is looked past
    3 lines             split-and-merge per cluster of >= 4 points, total-least-squares fits; two runs of a split
                        that meet at > 15 deg end at their lines' intersection (the corner, not the nearest scan
                        point); a <= 5-point run straddling a corner is given back to the two walls; neighbours merge
                        when their joint line holds (a wall cut by a sign's shadow is one wall)
    5 with a pose       field wall band -> CLS_WALL; the lot band's 200 x 20 limitations -> CLS_LIMITER and
                        `limiters` (side views and end-on views); a pillar on a map wall or in the lot band is
                        dropped, one on a seat gains confidence, may be a single return, and may be half hidden
    6 without a pose    segments >= 300 mm are walls; a 150-230 mm segment standing on a >= 400 mm one within 25 deg
                        of perpendicular, toward the lidar, with a convex free end, is a limitation (conf 0.5)
    8 clearances        36 sectors of body clearance around the body centre, the straight free run ahead and behind
    9 de-skew (motion)  every bin moved by the car's motion between its own time and the scan's mean time t - lag

Measured (laptop under the CAD workflow's load, numpy 2.3, 2026-09-23; mock.World scans at 1 deg, sigma 6 mm, the
WLtoys mount; tests_console/test_lidar_perc.py asserts each):
    signs, pose-free      0.5 / 0.7 / 1.0 / 1.3 m: 60 of 60 each (0, 150, 300 mm off the axis, the car jittered);
                          1.6 m: 57 of 60 -- a face-on sign gives ONE return 1 time in 5 there
    signs on seats + pose 1.6 m 20 of 20; 95 of 95 visible seat signs from random poses at 0.3-1.6 m (single
                          returns, half-hidden signs); centre error median 4-6 mm to 1.3 m, 8.6 at 1.6 m
    false signs           0 in 200 bare-wall scans at 40 random poses, pose-free and with the pose
    corners               endpoint error median 1.5 mm; max 2.7 (island's convex corner) / 12.5 (outer corner at a
                          grazing view, where a 3-point run joins two clusters)
    limitations + pose    96 of 96 hit with >= 3 returns found; the face x within 9.8 mm; none reported as a sign
    free_fwd              within 15.5 mm of the true run at headings to 10 deg off the wall's normal
    de-skew               0.45 m/s at 2 rad/s: wall distance median 11.3 mm raw -> 0.20 mm
    cost                  perceive 0.43 ms pose-free, 0.64 ms with the pose (264 returns, 11 segments; CPU 44 % busy,
                          up to 0.75 / 1.1 ms at 52 %); swept 0.085 ms for one arc, 0.28 ms for 21 arcs.  x3-4.3 on
                          the Pi 5 (BRAIN_SPEC 13): ~2-3 ms per scan at 10 Hz = 2-3 % of a core; the command's arc at
                          40 Hz ~1 %
"""
from __future__ import annotations

import base64
import math
import time
from dataclasses import dataclass, field

import numpy as np

from . import field as F

CLS_NONE, CLS_WALL, CLS_PILLAR, CLS_LIMITER, CLS_CLUTTER = 0, 1, 2, 3, 4
CLS_NAMES = {CLS_NONE: "none", CLS_WALL: "wall", CLS_PILLAR: "pillar", CLS_LIMITER: "limiter", CLS_CLUTTER: "clutter"}
NO_RETURN = 255                        # the wire's per-bin class for a bin without a return
N_SECTORS = 36
DEFAULT_BLIND = ((130.0, 230.0),)      # what wro_next / watch assume when lidar.block_deg is absent
CLUTTER_MM = 5.0                       # body clearance below this = the car's own parts
PUSH_MM = F.PILLAR / 2.0               # a sign's centre lies half a sign behind the face the lidar sees
SEAT_N1_MM = 60.0                      # a single-return pillar needs a seat this close (pose only)
LOT_BAND = (-1500.0, -1290.0, 1400.0)  # straight S: y from, y to, |x| <= -- where the limitations stand [RULE p.8]
LOT_OFF_WALL = 15.0                    # a limitation return lies this far off the outer wall face at least
AWAY_MM = 2.0                          # swept(): a point within reach that the first 2 mm of motion do not bring
#                                        closer to the BODY does not block (a wall behind the tail while pulling away)
CLOSE_EPS = 1e-6                       # mm: "closer" is strictly closer -- a wall the body runs parallel to is not

computed = 0                           # perceptions of(robot) computed (tests read it: once per scan)


# ------------------------------------------------------------------------------------------------------------ config
def config(p: dict) -> dict:
    """The numbers perceive() reads, from a robot params dict, each with its BRAIN_SPEC 2.2 default: lidar.pos_mm,
    block_deg, min_mm, min_conf, scan_lag_s, cw, offset_deg; perc.*; car.rear_mm / front_mm / half_w_mm;
    chassis.wheelbase_m; steer.max_deg.  `key` identifies the values (the of() cache key)."""
    p = p or {}
    li, pc, car, ch, st = (p.get(k) or {} for k in ("lidar", "perc", "car", "chassis", "steer"))
    pos = li.get("pos_mm") or (0.0, 0.0)
    c: dict = dict(
        pos=(float(pos[0]), float(pos[1])),
        blind=tuple((float(lo), float(hi)) for lo, hi in li.get("block_deg", DEFAULT_BLIND)),
        min_mm=float(li.get("min_mm", 60.0)),
        min_conf=int(li.get("min_conf", 0)),
        lag=float(li.get("scan_lag_s", 0.05)),
        cw=bool(li.get("cw", True)),
        offset=float(li.get("offset_deg", 0.0)),
        max_mm=float(pc.get("max_mm", 3000.0)),
        break_deg=float(pc.get("break_deg", 10.0)),
        break_mm=float(pc.get("break_mm", 30.0)),
        line_tol_mm=float(pc.get("line_tol_mm", 20.0)),
        line_tol_pct=float(pc.get("line_tol_pct", 1.0)),
        pillar_max_mm=float(pc.get("pillar_max_mm", 80.0)),
        pillar_gap_mm=float(pc.get("pillar_gap_mm", 100.0)),
        wall_band_mm=float(pc.get("wall_band_mm", 45.0)),
        seat_snap_mm=float(pc.get("seat_snap_mm", 120.0)),
        car=(float(car.get("rear_mm", 34.0)), float(car.get("front_mm", 179.0)), float(car.get("half_w_mm", 80.5))),
        wheelbase_mm=float(ch.get("wheelbase_m", 0.145)) * 1000.0,
        steer_max_deg=float(st.get("max_deg", 29.0)),
    )
    c["key"] = tuple(c[k] for k in sorted(c))
    return c


# ------------------------------------------------------------------------------------------------------------ result
@dataclass(eq=False)
class Perception:
    t: float                 # Scan.t (monotonic, end of the revolution)
    lag_s: float             # lidar.scan_lag_s: the points hold for t - lag_s on average
    X: np.ndarray            # float32 (n,) valid returns in the CAR frame, mm (lidar pos added)
    Y: np.ndarray
    b: np.ndarray            # int16 (n,) scan bin of each point
    cls: np.ndarray          # int8  (n,) CLS_*
    lab: np.ndarray          # int16 (n,) cluster id (angular order), -1 = clutter
    segs: np.ndarray         # float32 (k, 7): x1, y1, x2, y2, n, rms_mm, cls   (endpoints on the fitted line)
    pillars: np.ndarray      # float32 (m, 6): x, y (centre), width_mm, conf 0..1, n, range_mm (centre, from the lidar)
    limiters: np.ndarray     # float32 (q, 6): x1, y1, x2, y2, conf, n
    sectors: np.ndarray      # float32 (36,): min body clearance, mm, per 10 deg around the body centre; inf = none
    free_fwd: float          # mm the body can drive straight ahead before touching a point (perc.max_mm if none)
    free_back: float         # the same backwards (the blind sector is not in it: see `blind`)
    blind: list              # [(lo, hi)] lidar.block_deg, echoed
    pose: tuple | None       # (x, y, th) field pose at the scan used for the prior, or None
    ms: float                # compute time
    res: float = 1.0         # the scan's bin width, deg
    bcls: np.ndarray = field(default_factory=lambda: np.zeros(0, np.uint8))
    #                          uint8 per scan bin: CLS_* of its return, 255 no return, 0 dropped (too near/far, blind)
    pillar_seat: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int16))
    #                          int16 (m,) the nearest field.SEATS index within perc.seat_snap_mm, -1 (pose only)
    lidar_xy: tuple = (0.0, 0.0)          # lidar.pos_mm
    _car: tuple = (34.0, 179.0, 80.5)     # the body (rear, front, half_w) the clearances used

    def points(self, exclude=(CLS_PILLAR, CLS_LIMITER, CLS_CLUTTER)):
        """(X, Y) float64 of the points whose class is not in `exclude` -- by default what the localiser takes (W1:
        walls, and unknown returns, which are what corrects a pose that drifted off the map)."""
        m = ~np.isin(self.cls, exclude)
        return self.X[m].astype(np.float64), self.Y[m].astype(np.float64)

    def nearest(self):
        """(body clearance mm, bearing deg from the body centre) of the nearest non-clutter return; (inf, nan)."""
        m = self.cls != CLS_CLUTTER
        if not m.any():
            return math.inf, math.nan
        rear, front, _half = self._car
        X, Y = self.X[m].astype(np.float64), self.Y[m].astype(np.float64)
        c = body_clearance(X, Y, self._car)
        i = int(np.argmin(c))
        return float(c[i]), math.degrees(math.atan2(Y[i], X[i] - (front - rear) / 2.0))

    def wire(self) -> dict:
        """The body of the console's `perc` message (BRAIN_SPEC 8.4) without type / t / age_ms / src: int mm, conf in
        %, `sec` -1 = no return, `bcls` base64 of one byte per bin."""
        sg = self.segs
        seg = np.column_stack([np.rint(sg[:, :4]), sg[:, 6]]).astype(np.int64).ravel().tolist() if len(sg) else []
        pl = self.pillars
        pil = [[int(round(r[0])), int(round(r[1])), int(round(r[2])), int(round(100 * r[3])), int(r[4])] for r in pl]
        lim = [[int(round(v)) for v in r[:4]] + [int(round(100 * r[4]))] for r in self.limiters]
        sec = [int(round(v)) if math.isfinite(v) else -1 for v in self.sectors.tolist()]
        return dict(ms=round(float(self.ms), 2), n=int(len(self.X)), seg=seg, pil=pil, lim=lim, sec=sec,
                    free=dict(fwd=int(round(self.free_fwd)), back=int(round(self.free_back))),
                    bcls=base64.b64encode(self.bcls.tobytes()).decode("ascii"), blind=[[lo, hi] for lo, hi in self.blind])


# ------------------------------------------------------------------------------------------------------------ frames
def to_field(X, Y, pose):
    """Car-frame points -> field frame through pose (x, y, th): x + c X - s Y, y + s X + c Y."""
    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = math.cos(th), math.sin(th)
    X, Y = np.asarray(X, np.float64), np.asarray(Y, np.float64)
    return x + c * X - s * Y, y + s * X + c * Y


def field_to_car(Xf, Yf, pose):
    """The inverse of to_field: field points -> the car frame at pose."""
    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = math.cos(th), math.sin(th)
    dx, dy = np.asarray(Xf, np.float64) - x, np.asarray(Yf, np.float64) - y
    return c * dx + s * dy, -s * dx + c * dy


def field_wall_dist(xf, yf):
    """Distance (mm) from field points to the nearest wall face of the FIELD (outer square +-1500, island +-500),
    analytic: the lot's limitations and known signs a program adds to its DistField are not in it."""
    ax, ay = np.abs(np.asarray(xf, np.float64)), np.abs(np.asarray(yf, np.float64))
    m = np.maximum(ax, ay)
    outer = np.abs(F.HALF - m)
    isl = np.hypot(np.maximum(ax - F.ISL, 0.0), np.maximum(ay - F.ISL, 0.0))
    isl = np.where(m < F.ISL, F.ISL - m, isl)
    return np.minimum(outer, isl)


# ------------------------------------------------------------------------------------------------------------ bodies
def body_clearance(X, Y, car) -> np.ndarray:
    """Distance (mm) of car-frame points to the body rectangle car = (rear, front, half_w) from the rear axle
    ([-rear, front] x [-half_w, half_w]); 0 inside."""
    rear, front, half = (float(v) for v in car)
    X, Y = np.asarray(X, np.float64), np.asarray(Y, np.float64)
    ex = np.maximum(np.maximum(-rear - X, X - front), 0.0)
    ey = np.maximum(np.abs(Y) - half, 0.0)
    return np.hypot(ex, ey)


def body_sclearance(X, Y, car) -> np.ndarray:
    """body_clearance, SIGNED: negative inside the rectangle (minus the depth to its nearest face)."""
    rear, front, half = (float(v) for v in car)
    return _rect_sdist(np.asarray(X, np.float64), np.asarray(Y, np.float64), rear, front, half)


def grow(car, ahead: float, side: float, behind: float | None = None) -> tuple:
    """The footprint grown by `ahead` at the nose, `behind` (default = ahead) at the tail and `side` each side --
    the shield's stopping margins (BRAIN_SPEC 5.3: 40 / 15 driving, 8 all round parking)."""
    rear, front, half = (float(v) for v in car)
    return rear + float(ahead if behind is None else behind), front + float(ahead), half + float(side)


def curvature(steer_deg: float, wheelbase_mm: float) -> float:
    """1/mm, + = left: the arc the rear axle follows at `steer_deg` on the EFFECTIVE wheelbase (chassis.wheelbase_m,
    the one the car really turns on: the stock A1 turned R ~510 mm at 29 deg, not the geometric 262)."""
    return math.tan(math.radians(float(steer_deg))) / max(float(wheelbase_mm), 1e-6)


def need_mm(v_mps: float, decel: float, latency_s: float, stop_mm: float) -> float:
    """The distance (mm) a command at |v| needs before anything may be in its way: the braked stop v^2 / (2 decel),
    the distance covered while the next scan and loop notice (|v| latency), and a fixed stop margin."""
    v = abs(float(v_mps))
    if v == 0.0:
        return 0.0
    return 1000.0 * (v * v / (2.0 * max(float(decel), 1e-3)) + v * float(latency_s)) + float(stop_mm)


def v_allowed(free_mm: float, decel: float, latency_s: float, stop_mm: float) -> float:
    """The largest |v| (m/s) with need_mm(v) <= free_mm: the root of 500 v^2 / decel + 1000 latency v + stop = free."""
    a, lat = max(float(decel), 1e-3), float(latency_s)
    room = (float(free_mm) - float(stop_mm)) / 1000.0
    if room <= 0.0:
        return 0.0
    return a * (-lat + math.sqrt(lat * lat + 2.0 * room / a))


# ------------------------------------------------------------------------------------------------------------ swept
def swept(X, Y, car, curv, direction, horizon, step: float = 20.0, inflate: float = 0.0, body=None) -> float:
    """Arc length (mm, >= 0) the rear axle can travel on curvature `curv` (1/mm, + left) in `direction` (+1 / -1)
    before the footprint car = (rear, front, half_w), inflated by `inflate`, comes within reach of a point; `horizon`
    when nothing is hit.  See swept_many."""
    return float(swept_many(X, Y, car, [curv], direction, horizon, step, inflate, body)[0])


def arc_free(X, Y, car, v_mps: float, steer_deg: float, wheelbase_mm: float, horizon: float = 900.0,
             inflate: float = 0.0, body=None) -> float:
    """The free run (mm) of a drive command: swept() on the arc of `steer_deg` (curvature(), the effective wheelbase)
    in the direction of `v_mps`; `horizon` for a stop.  Compare with need_mm(v): BRAIN_SPEC 5.3's shield test."""
    if v_mps == 0.0:
        return float(horizon)
    return swept(X, Y, car, curvature(steer_deg, wheelbase_mm), 1 if v_mps > 0 else -1, horizon, inflate=inflate,
                 body=body)


def swept_many(X, Y, car, curvs, direction, horizon, step: float = 20.0, inflate: float = 0.0,
               body=None) -> np.ndarray:
    """swept() for several curvatures at once -> (len(curvs),) mm.

    Exact, not sampled (`step` is accepted for the BRAIN_SPEC 4.1 signature and unused).  Straight (|k| < 1e-9): a
    point in the path's band enters at x - front - sqrt(inflate^2 - ey^2).  Arc: in the car frame every point turns
    about the instantaneous centre (0, 1/k) by -k s, so its entry is the first crossing of that circle with the
    inflated body (4 offset edges and 4 corner arcs) at which it goes INSIDE -- closed form for every (curvature,
    point) pair whose radius and angle can reach the body within k * horizon.  A 20 mm sampling of the arc missed a
    point that only clips an inflated corner (2 of 400 random clouds: 109 mm read as the 1500 mm horizon, 2026-09-23).
    Reversing is driving forward in the mirror image (x -> -x, rear <-> front); a right turn is a left turn mirrored
    in y.

    A point already within reach (inside `car` grown by `inflate`) blocks at 0 only when the first AWAY_MM of the
    motion bring it strictly closer to `body` -- the car's REAL rectangle (default: `car` itself), on the SIGNED
    distance (negative inside): stopping must never be the only way out of a touch, and a point the body only runs
    parallel to or leaves behind is let go.  The test used to be `not farther` on the grown footprint's unsigned
    distance, which is 0 before and after for every point inside it: one return inside the 40 / 15 mm driving margin
    blocked forward AND reverse at every steering, and a car stopped 20 mm from a wall, or started parallel to one
    10 mm off, could never move again (sim: 104 brakes and LOST at 5.2 s; 43 LOST cycles beside a sign)."""
    k = np.atleast_1d(np.asarray(curvs, np.float64)).ravel()
    H = max(float(horizon), 0.0)
    out = np.full(len(k), H)
    X = np.asarray(X, np.float64).ravel()
    Y = np.asarray(Y, np.float64).ravel()
    if not len(X) or not len(k) or H <= 0.0:
        return out
    rear, front, half = (float(v) for v in car)
    br, bf, bh = (float(v) for v in (body if body is not None else car))
    if direction < 0:
        X, rear, front, br, bf = -X, front, rear, bf, br
    bd = (br, bf, bh)
    infl = max(float(inflate), 0.0)
    straight = np.abs(k) < 1e-9
    if straight.any():
        out[straight] = _swept_straight(X, Y, rear, front, half, infl, H, bd)
    arc = np.flatnonzero(~straight)
    if len(arc):
        out[arc] = _swept_arcs(X, Y, rear, front, half, infl, H, k[arc], bd)
    return out


def _rect_dist(a, b, rear, front, half):
    ex = np.maximum(np.maximum(-rear - a, a - front), 0.0)
    ey = np.maximum(np.abs(b) - half, 0.0)
    return np.hypot(ex, ey)


def _rect_sdist(a, b, rear, front, half):
    """_rect_dist, signed: inside the rectangle minus the depth to its nearest face."""
    ex = np.maximum(-rear - a, a - front)
    ey = np.abs(b) - half
    out = np.hypot(np.maximum(ex, 0.0), np.maximum(ey, 0.0))
    return np.where((ex <= 0.0) & (ey <= 0.0), np.maximum(ex, ey), out)


def _swept_straight(X, Y, rear, front, half, infl, H, body=None) -> float:
    ey = np.maximum(np.abs(Y) - half, 0.0)
    d0 = np.hypot(np.maximum(np.maximum(-rear - X, X - front), 0.0), ey)
    near = d0 <= infl
    if near.any():                            # within reach now: blocking only if the first AWAY_MM bring it closer
        br, bf, bh = body if body is not None else (rear, front, half)
        Xn, Yn = X[near], Y[near]
        s0 = _rect_sdist(Xn, Yn, br, bf, bh)
        s1 = _rect_sdist(Xn - AWAY_MM, Yn, br, bf, bh)
        if (s1 < s0 - CLOSE_EPS).any():
            return 0.0
    band = (ey <= infl) & (X > front) & ~near
    if not band.any():
        return H
    s = X[band] - front - np.sqrt(np.maximum(infl * infl - ey[band] ** 2, 0.0))
    return float(min(H, max(0.0, float(s.min()))))


def _swept_arcs(X, Y, rear, front, half, infl, H, k, body=None) -> np.ndarray:
    nk = len(k)
    out = np.full(nk, H)
    sg = np.where(k < 0, -1.0, 1.0)
    ka = np.abs(k)
    R = 1.0 / ka
    Yj = sg[:, None] * Y[None, :]                                  # (nk, n): every arc turned into a left turn
    Xj = np.broadcast_to(X[None, :], Yj.shape)
    # --- points within reach now: blocking only if the first AWAY_MM bring them strictly closer to the body
    d0 = _rect_dist(Xj, Yj, rear, front, half)
    near = d0 <= infl
    if near.any():
        br, bf, bh = body if body is not None else (rear, front, half)
        jj, ii = np.nonzero(near)
        th = ka[jj] * AWAY_MM
        fx, fy = np.sin(th) / ka[jj], (1.0 - np.cos(th)) / ka[jj]
        dx, dy = X[ii] - fx, Yj[jj, ii] - fy
        c, s = np.cos(th), np.sin(th)
        s0 = _rect_sdist(X[ii], Yj[jj, ii], br, bf, bh)
        s1 = _rect_sdist(c * dx + s * dy, -s * dx + c * dy, br, bf, bh)
        closing = s1 < s0 - CLOSE_EPS
        out[np.unique(jj[closing])] = 0.0
    # --- (curvature, point) pairs that the arc can reach: radius band and angular window about the centre
    rho = np.hypot(Xj, Yj - R[:, None])
    rmin = np.maximum(R - half - infl, 0.0)
    rmax = np.hypot(max(rear, front) + infl, R + half + infl)
    cand = ~near & (rho >= rmin[:, None]) & (rho <= rmax[:, None])
    inside = R <= half + infl                                      # the centre inside the (inflated) body: no window
    psi = np.arctan2(Xj, R[:, None] - Yj)                          # angle about the centre from the rear axle's ray
    den = np.maximum(R - half - infl, 1e-6)
    p_hi = np.arctan2(front + infl, den)
    p_lo = np.arctan2(-(rear + infl), den)
    rot = np.where(psi > p_hi[:, None], psi - p_hi[:, None], np.where(psi >= p_lo[:, None], 0.0,
                                                                   psi - p_hi[:, None] + 2.0 * math.pi))
    cand &= (rot <= (ka * H)[:, None] + 1e-9) | inside[:, None] | near     # near + moving away: may come round
    cand &= (out > 0.0)[:, None]
    if not cand.any():
        return out
    # --- exact: the point runs on the circle (0, R), rho at angle psi = atan2(x, R - y), psi(s) = psi0 - k s; the
    # inflated body is 4 edges offset by `inflate` (each valid over the rectangle's own extent) and 4 corner arcs
    jj, ii = np.nonzero(cand)
    r, Rp = rho[jj, ii][:, None], R[jj][:, None]
    psi0 = psi[jj, ii]
    # nose / tail edges x = const (valid for |y| <= half): sin psi = x / rho, two roots each
    sv = np.array([front + infl, -(rear + infl)])[None, :] / r
    a1 = np.arcsin(np.clip(sv, -1.0, 1.0))
    ax = np.hstack([a1, math.pi - a1])
    yv = Rp - r * np.cos(ax)
    okx = (np.abs(np.hstack([sv, sv])) <= 1.0) & (yv >= -half) & (yv <= half)
    # side edges y = const (valid for -rear <= x <= front): cos psi = (R - y) / rho
    cv = (Rp - np.array([-(half + infl), half + infl])[None, :]) / r
    a2 = np.arccos(np.clip(cv, -1.0, 1.0))
    ay = np.hstack([a2, -a2])
    xv = r * np.sin(ay)
    oky = (np.abs(np.hstack([cv, cv])) <= 1.0) & (xv >= -rear) & (xv <= front)
    A, ok = [ax, ay], [okx, oky]
    if infl > 0.0:                                                 # the 4 corner arcs: circle-circle intersections
        ccx = np.array([front, front, -rear, -rear])[None, :]
        ccy = np.array([-half, half, -half, half])[None, :]
        dc = np.hypot(ccx, Rp - ccy)
        pc = np.arctan2(ccx, Rp - ccy)
        ca = (r * r + dc * dc - infl * infl) / np.maximum(2.0 * r * dc, 1e-9)
        al = np.arccos(np.clip(ca, -1.0, 1.0))
        A.append(np.hstack([pc + al, pc - al]))
        okc = np.abs(ca) <= 1.0
        ok.append(np.hstack([okc, okc]))
    A, fin = np.hstack(A), np.hstack(ok)                           # (P, C) crossing angles, valid
    ae = A - 0.05 / r                                              # 0.05 mm past the crossing: inside = an entry
    entry = fin & (_rect_dist(r * np.sin(ae), Rp - r * np.cos(ae), rear, front, half) <= infl)
    rot = np.where(entry, (psi0[:, None] - A) % (2.0 * math.pi), np.inf).min(axis=1)
    best = np.full(nk, np.inf)
    np.minimum.at(best, jj, rot / ka[jj])
    return np.minimum(out, best)


# ------------------------------------------------------------------------------------------------------------ fusion
def associate(det_xy, tgt_xy, cam_xy=(0.0, 0.0), bearing_tol_deg: float = 4.0, range_gate: float = 0.35,
              half_w_mm: float = 40.0) -> list:
    """Camera detections -> lidar pillars (or seats) by BEARING from the camera (BRAIN_SPEC 5.2, P0).

    det_xy: (m, 2) the detections' floor points in the car frame at the camera exposure (vision.pillars X, Y); tgt_xy:
    (q, 2) the targets in the SAME frame (lidar pillars carried there, or seats through field_to_car at the pose of
    the exposure); cam_xy: the lens (camera.x_mm, y_mm).  A pair matches when |phi - phi_t| <= max(bearing_tol_deg,
    atan(half_w_mm / r_t)) and |r - r_t| <= range_gate * r_t: the bearing of a blob is pitch-free to first order, its
    floor range is not (1 deg of pitch moved the mat-day range by ~10 % at 1 m).  One-to-one, smallest |dphi| first,
    so of two targets at one bearing the range gate picks, and of two detections of one target the better aligned
    wins.  Returns [(det index, target index, dphi_deg)] sorted by det index."""
    D = np.asarray(det_xy, np.float64).reshape(-1, 2)
    T = np.asarray(tgt_xy, np.float64).reshape(-1, 2)
    if not len(D) or not len(T):
        return []
    cx, cy = float(cam_xy[0]), float(cam_xy[1])
    pd, rd = np.arctan2(D[:, 1] - cy, D[:, 0] - cx), np.hypot(D[:, 0] - cx, D[:, 1] - cy)
    pt, rt = np.arctan2(T[:, 1] - cy, T[:, 0] - cx), np.hypot(T[:, 0] - cx, T[:, 1] - cy)
    dphi = np.degrees((pd[:, None] - pt[None, :] + math.pi) % (2.0 * math.pi) - math.pi)
    tol = np.maximum(float(bearing_tol_deg), np.degrees(np.arctan2(float(half_w_mm), np.maximum(rt, 1.0))))
    ok = (np.abs(dphi) <= tol[None, :]) & (np.abs(rd[:, None] - rt[None, :]) <= float(range_gate) * rt[None, :])
    ii, jj = np.nonzero(ok)
    if not len(ii):
        return []
    order = np.lexsort((jj, ii, np.abs(dphi[ii, jj])))
    used_d, used_t, out = set(), set(), []
    for q in order.tolist():
        i, j = int(ii[q]), int(jj[q])
        if i in used_d or j in used_t:
            continue
        used_d.add(i)
        used_t.add(j)
        out.append((i, j, float(dphi[i, j])))
    return sorted(out)


def dets_xy(dets, colours=("red", "green")):
    """vision.pillars() output -> ((m, 2) car-frame floor points, [colour], [index into dets]) of the wanted colours."""
    keep = [i for i, q in enumerate(dets or []) if q.get("colour") in colours]
    xy = np.array([[float(dets[i]["X"]), float(dets[i]["Y"])] for i in keep], np.float64).reshape(-1, 2)
    return xy, [dets[i]["colour"] for i in keep], keep


def seen_through(scan, cfg: dict, X, Y, margin_mm: float = 60.0, rmin: float = 150.0, rmax: float = 2500.0):
    """Per car-frame point: 1 = the scan saw PAST it (every bin across a sign's half width at its bearing returned
    farther than its range + margin_mm: nothing stands there), 0 = something at or before it, -1 = no evidence (blind
    sector, no return, out of rmin..rmax, below lidar.min_mm).  The SeatMap's negative evidence (BRAIN_SPEC 5.2)."""
    X, Y = np.asarray(X, np.float64).ravel(), np.asarray(Y, np.float64).ravel()
    out = np.full(len(X), -1, np.int8)
    if scan is None or not len(X):
        return out
    d = np.asarray(scan.dist)
    n, res = len(d), float(scan.res)
    _a, _c, _s, blind = _tables(n, res, cfg["blind"])
    px, py = cfg["pos"]
    r = np.hypot(X - px, Y - py)
    ang = np.degrees(np.arctan2(Y - py, X - px))
    half = np.degrees(np.arctan2(PUSH_MM, np.maximum(r, 1.0)))
    for i in range(len(X)):                                        # <= 24 seats: a few us each
        if not (rmin <= r[i] <= rmax):
            continue
        k0 = int(math.floor((ang[i] - half[i] + 180.0) / res))
        k1 = int(math.floor((ang[i] + half[i] + 180.0) / res))
        ks = np.arange(k0, k1 + 1) % n
        dk = d[ks]
        if blind[ks].any() or (dk == 0).any() or (dk <= cfg["min_mm"]).any():
            continue
        out[i] = 1 if (dk > r[i] + float(margin_mm)).all() else 0
    return out


# ------------------------------------------------------------------------------------------------------------ perceive
_TABLES: dict = {}


def _tables(n: int, res: float, blind: tuple):
    """(bin angles deg, cos, sin, blind mask) per scan layout, computed once."""
    key = (n, round(res, 9), blind)
    t = _TABLES.get(key)
    if t is None:
        a = -180.0 + (np.arange(n) + 0.5) * res
        rad = np.radians(a)
        aa = a % 360.0
        bl = np.zeros(n, bool)
        for lo, hi in blind:
            if hi - lo >= 360.0:                                   # (0, 360): the whole circle, not an empty arc
                bl[:] = True
                continue
            lo, hi = lo % 360.0, hi % 360.0
            bl |= ((aa >= lo) & (aa <= hi)) if lo <= hi else ((aa >= lo) | (aa <= hi))
        if len(_TABLES) > 32:
            _TABLES.clear()
        t = _TABLES[key] = (a, np.cos(rad), np.sin(rad), bl)
    return t


def _empty(scan, cfg, pose, t0, bcls):
    """A scan with no valid return (a covered lidar, everything blind or out of range): nothing seen, all free."""
    z = np.zeros(0, np.float32)
    return Perception(
        t=float(scan.t), lag_s=cfg["lag"], X=z, Y=z, b=np.zeros(0, np.int16), cls=np.zeros(0, np.int8),
        lab=np.zeros(0, np.int16), segs=np.zeros((0, 7), np.float32), pillars=np.zeros((0, 6), np.float32),
        limiters=np.zeros((0, 6), np.float32), sectors=np.full(N_SECTORS, np.inf, np.float32),
        free_fwd=cfg["max_mm"], free_back=cfg["max_mm"], blind=list(cfg["blind"]),
        pose=None if pose is None else tuple(float(v) for v in pose[:3]), ms=(time.perf_counter() - t0) * 1000.0,
        res=float(scan.res), bcls=bcls, pillar_seat=np.zeros(0, np.int16), lidar_xy=cfg["pos"], _car=cfg["car"])


def _deskew(X, Y, a, scan, cfg, motion):
    """Move each return by the car's motion between its bin's time and the scan's mean time t - lag.  A bin's time is
    t - (1 - raw / 360) * 60 / rpm: the revolution ends at the raw wrap (lidar.ScanAssembler), raw = offset -+ a."""
    v, w = float(motion[0]), float(motion[1])
    rpm = float(scan.rpm) if scan.rpm and scan.rpm > 0 else 600.0
    raw = (cfg["offset"] + (-a if cfg["cw"] else a)) % 360.0
    dt = (1.0 - raw / 360.0) * (60.0 / rpm) - cfg["lag"]           # t_ref - t_i
    dth = w * dt
    ds = v * 1000.0 * dt
    dx, dy = ds * np.cos(dth / 2.0), ds * np.sin(dth / 2.0)
    c, s = np.cos(dth), np.sin(dth)
    Xs, Ys = X - dx, Y - dy
    return c * Xs + s * Ys, -s * Xs + c * Ys


def _fit(xs, ys, starts, cnt):
    """Total-least-squares lines of runs laid end to end (reduceat): centroid, unit direction, rms (mm)."""
    sx, sy = np.add.reduceat(xs, starts), np.add.reduceat(ys, starts)
    mx, my = sx / cnt, sy / cnt
    dx, dy = xs - np.repeat(mx, cnt), ys - np.repeat(my, cnt)
    sxx = np.add.reduceat(dx * dx, starts) / cnt
    syy = np.add.reduceat(dy * dy, starts) / cnt
    sxy = np.add.reduceat(dx * dy, starts) / cnt
    th = 0.5 * np.arctan2(2.0 * sxy, sxx - syy)
    ux, uy = np.cos(th), np.sin(th)
    lam_min = 0.5 * (sxx + syy) - np.sqrt(0.25 * (sxx - syy) ** 2 + sxy * sxy)
    return mx, my, ux, uy, np.sqrt(np.maximum(lam_min, 0.0))


def _line(xs, ys):
    mx, my, ux, uy, _r = _fit(xs, ys, np.array([0]), np.array([len(xs)]))
    return float(mx[0]), float(my[0]), float(ux[0]), float(uy[0])


def _absorb_chamfers(runs, xw, yw):
    """A run of <= 5 points between two runs of its own cluster whose lines cross within 40 mm of it is a chamfer
    across a corner, not a wall: a split point one sample off a concave corner leaves a sub-run holding returns of
    both walls, and its fit cut the corner by 16-30 mm (outer corner seen from the straight, 3 of 40 scans,
    2026-09-23).  Its points go to the neighbour whose line they fit (one cut, least total distance)."""
    i = 1
    while i < len(runs) - 1:
        a, s, c = runs[i - 1], runs[i], runs[i + 1]
        if not (s[1] - s[0] + 1 <= 5 and a[2] == s[2] == c[2] and a[1] == s[0] and s[1] == c[0]
                and a[1] - a[0] >= 2 and c[1] - c[0] >= 2):
            i += 1
            continue
        ma = _line(xw[a[0]:a[1]], yw[a[0]:a[1]])
        mc = _line(xw[c[0] + 1:c[1] + 1], yw[c[0] + 1:c[1] + 1])
        cr = ma[2] * mc[3] - ma[3] * mc[2]
        if abs(cr) < math.sin(math.radians(15.0)):
            i += 1
            continue
        t = ((mc[0] - ma[0]) * mc[3] - (mc[1] - ma[1]) * mc[2]) / cr
        ix, iy = ma[0] + t * ma[2], ma[1] + t * ma[3]
        xs, ys = xw[s[0]:s[1] + 1], yw[s[0]:s[1] + 1]
        if np.hypot(xs - ix, ys - iy).min() > 40.0:
            i += 1
            continue
        da = np.abs((xs - ma[0]) * ma[3] - (ys - ma[1]) * ma[2])
        dc = np.abs((xs - mc[0]) * mc[3] - (ys - mc[1]) * mc[2])
        cost = np.array([da[:m + 1].sum() + dc[m:].sum() for m in range(len(xs))])
        m = s[0] + int(np.argmin(cost))
        runs[i - 1] = (a[0], m, a[2])
        runs[i + 1] = (m, c[1], c[2])
        del runs[i]
    return runs


def _segments(runs, xw, yw, rw, cls_w, with_pose: bool, tol_mm: float, tol_k: float):
    """Split runs -> segments (k, 7) and each segment's point ranges [(i0, i1)] in the working arrays.

    A split point is shared by the two runs it separates; it is left out of both fits (it belongs to one face only)
    and, where the two lines meet at > 15 deg within 60 mm of it, the corner is their intersection -- the true corner
    instead of the scan point nearest it (one beam spacing off: 12-25 mm at 0.7-1.4 m).  Then neighbours of different
    clusters merge when their directions differ < 5 deg, the gap is < 150 mm and the joint rms stays under the
    split tolerance (BRAIN_SPEC 4.2 step 3, P1)."""
    runs = _absorb_chamfers(list(runs), xw, yw)
    nr = len(runs)
    sh_s = [i > 0 and runs[i - 1][2] == runs[i][2] and runs[i - 1][1] == runs[i][0] for i in range(nr)]
    sh_e = [i + 1 < nr and runs[i + 1][2] == runs[i][2] and runs[i + 1][0] == runs[i][1] for i in range(nr)]
    idx = []
    for i, (i0, i1, _c) in enumerate(runs):
        a, z = i0 + int(sh_s[i]), i1 - int(sh_e[i])
        idx.append(np.arange(a, z + 1) if z - a >= 2 else np.arange(i0, i1 + 1))
    cnt = np.array([len(q) for q in idx])
    allq = np.concatenate(idx)
    mx, my, ux, uy, rms = _fit(xw[allq], yw[allq], np.r_[0, np.cumsum(cnt)[:-1]], cnt)
    i0s = np.array([rn[0] for rn in runs])
    i1s = np.array([rn[1] for rn in runs])
    t1 = (xw[i0s] - mx) * ux + (yw[i0s] - my) * uy
    t2 = (xw[i1s] - mx) * ux + (yw[i1s] - my) * uy
    p1 = np.column_stack([mx + t1 * ux, my + t1 * uy])
    p2 = np.column_stack([mx + t2 * ux, my + t2 * uy])
    for i in range(nr - 1):                                        # corners of one cluster's split
        if not sh_e[i]:
            continue
        cr = ux[i] * uy[i + 1] - uy[i] * ux[i + 1]
        if abs(cr) < math.sin(math.radians(15.0)):
            continue
        qx, qy = mx[i + 1] - mx[i], my[i + 1] - my[i]
        s_ = (qx * uy[i + 1] - qy * ux[i + 1]) / cr
        cx_, cy_ = mx[i] + s_ * ux[i], my[i] + s_ * uy[i]
        k = runs[i][1]
        if math.hypot(cx_ - xw[k], cy_ - yw[k]) <= 60.0:
            p2[i] = (cx_, cy_)
            p1[i + 1] = (cx_, cy_)
    # ---- merge: neighbours of one cluster whose joint line holds every point within the split tolerance (a noise
    # point > tol from a chord split a straight wall and tilted a half by 6 deg); neighbours of different clusters
    # when their directions differ < 5 deg, the gap is < 150 mm and the joint rms stays under the tolerance
    out, pieces = [], []
    cur = dict(q=idx[0], p1=p1[0], p2=p2[0], r=[(runs[0][0], runs[0][1])], c=runs[0][2], u=(ux[0], uy[0]),
               m=(mx[0], my[0]), rms=rms[0])
    cos20, cos5 = math.cos(math.radians(20.0)), math.cos(math.radians(5.0))
    for i in range(1, nr):
        nxt = dict(q=idx[i], p1=p1[i], p2=p2[i], r=[(runs[i][0], runs[i][1])], c=runs[i][2], u=(ux[i], uy[i]),
                   m=(mx[i], my[i]), rms=rms[i])
        same = nxt["c"] == cur["c"] and cur["r"][-1][1] == nxt["r"][0][0]
        cosd = abs(cur["u"][0] * nxt["u"][0] + cur["u"][1] * nxt["u"][1])
        gap = math.hypot(nxt["p1"][0] - cur["p2"][0], nxt["p1"][1] - cur["p2"][1])
        if (same and cosd > cos20) or (not same and cosd > cos5 and gap < 150.0):
            q = np.unique(np.concatenate([np.arange(a_, z_ + 1) for a_, z_ in cur["r"] + nxt["r"]])) if same \
                else np.concatenate([cur["q"], nxt["q"]])
            jmx, jmy, jux, juy, jrms = _fit(xw[q], yw[q], np.array([0]), np.array([len(q)]))
            tol_q = tol_mm + tol_k * rw[q]
            if same:
                ok = bool((np.abs((xw[q] - jmx[0]) * juy[0] - (yw[q] - jmy[0]) * jux[0]) <= tol_q).all())
            else:
                ok = jrms[0] < float(tol_q.mean())
            if ok:
                u, m_ = (jux[0], juy[0]), (jmx[0], jmy[0])

                def proj(p):
                    t = (p[0] - m_[0]) * u[0] + (p[1] - m_[1]) * u[1]
                    return np.array([m_[0] + t * u[0], m_[1] + t * u[1]])
                cur = dict(q=q, p1=proj(cur["p1"]), p2=proj(nxt["p2"]), r=cur["r"] + nxt["r"], c=nxt["c"], u=u, m=m_,
                           rms=jrms[0])
                continue
        out.append(cur)
        cur = nxt
    out.append(cur)
    segs = np.zeros((len(out), 7), np.float32)
    for si, s_ in enumerate(out):
        rg = s_["r"]
        npts = sum(z - a + 1 for a, z in rg) - sum(rg[q][1] == rg[q + 1][0] for q in range(len(rg) - 1))
        if with_pose:
            cw_ = np.concatenate([cls_w[a:z + 1] for a, z in rg])
            nw, nl = int((cw_ == CLS_WALL).sum()), int((cw_ == CLS_LIMITER).sum())
            nn = len(cw_) - nw - nl
            c_ = CLS_WALL if nw >= max(nn, nl) else (CLS_LIMITER if nl > nn else CLS_NONE)
        else:
            c_ = CLS_WALL if math.hypot(s_["p2"][0] - s_["p1"][0], s_["p2"][1] - s_["p1"][1]) >= 300.0 else CLS_NONE
        segs[si] = (s_["p1"][0], s_["p1"][1], s_["p2"][0], s_["p2"][1], npts, s_["rms"], c_)
        pieces.append(rg)
    return segs, pieces


def _lot_limiters(xw, yw, xf, yf, cw_, dfv) -> list:
    """With a pose: the 200 x 20 limitations in straight S's lot band (BRAIN_SPEC 4.2 step 5).  Candidates are the
    band's returns more than LOT_OFF_WALL off the wall face that are not signs, grouped by FIELD x (gaps > 40 mm;
    then within 25 mm of the group's median x, which sheds the odd wall return that noise lifts off the face).  A group
    is a limitation seen
        from the side   >= 3 returns, >= 50 mm of face along y, <= 40 mm across x   -> conf 0.9 (>= 120 mm) / 0.7
        end on          >= 2 returns at its free end (y within 30 mm of -1300), <= 40 mm across  -> conf 0.6
    Grouped by x, not by cluster: the side face and the end face often break into two clusters at the corner, and a
    face half inside the blind sector shows 50-115 mm of its 200 (11 of 60 lot scans missed a visible limitation with
    the spec's per-cluster 120-230 mm rule, 2026-09-23).  Rows are the measured extents (TLS endpoints, car frame).
    Marks the group CLS_LIMITER, or CLS_WALL when the program's map already holds it (lidar.lot_map: the localiser
    may then use it) -- read off `dfv`, the map distance of every point: on a mapped limitation its returns are ~0 mm
    from the map, while the plain field puts them >= LOT_OFF_WALL from the outer wall.  Edits cw_ in place."""
    y0, y1, xm = LOT_BAND
    q = np.flatnonzero((cw_ != CLS_PILLAR) & (yf > y0 + LOT_OFF_WALL) & (yf < y1) & (np.abs(xf) < xm))
    out = []
    if len(q) < 2:
        return out
    q = q[np.argsort(xf[q], kind="stable")]
    for grp in np.split(q, np.flatnonzero(np.diff(xf[q]) > 40.0) + 1):
        if len(grp) < 2:
            continue
        grp = grp[np.abs(xf[grp] - np.median(xf[grp])) <= 25.0]
        if len(grp) < 2:
            continue
        ey_ = float(yf[grp].max() - yf[grp].min())
        ex_ = float(xf[grp].max() - xf[grp].min())
        if ex_ > 40.0:
            continue
        if ey_ >= 50.0 and len(grp) >= 3:
            conf = 0.9 if ey_ >= 120.0 else 0.7
        elif ey_ < 50.0 and float(yf[grp].max()) >= -1330.0:
            conf = 0.6
        else:
            continue
        gx, gy = xw[grp], yw[grp]
        lmx, lmy, lux, luy, _rms = _fit(gx, gy, np.array([0]), np.array([len(grp)]))
        tt = (gx - lmx[0]) * lux[0] + (gy - lmy[0]) * luy[0]
        out.append((lmx[0] + tt.min() * lux[0], lmy[0] + tt.min() * luy[0],
                    lmx[0] + tt.max() * lux[0], lmy[0] + tt.max() * luy[0], conf, len(grp)))
        in_map = dfv is not None and float(np.median(dfv[grp])) < 0.5 * LOT_OFF_WALL
        cw_[grp] = CLS_WALL if in_map else CLS_LIMITER
    return out


def _free_limiters(segs, pieces, bu, d, n, lidar_xy, gap: float) -> list:
    """Without a pose (P1): a 150-230 mm segment with one end within 40 mm of a >= 400 mm segment, within 25 deg of
    perpendicular to it, is a 200 x 20 limitation standing on the wall (conf 0.5) -- if it also stands out TOWARD
    the lidar (its free end on the lidar's side of the wall line) and its free end is convex (the next bin beyond it
    returns >= `gap` farther, or nothing: the wall behind the slab).  Without those two, a mostly hidden wall meeting
    another at an outer corner and the island's convex corner passed: 19 of 300 bare-wall scans, 2026-09-23.  Marks
    it CLS_LIMITER in segs."""
    if len(segs) < 2:
        return []
    L = np.hypot(segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1])
    shorts, longs = np.flatnonzero((L >= 150.0) & (L <= 230.0)), np.flatnonzero(L >= 400.0)
    out = []
    if not len(shorts) or not len(longs):
        return out
    lx, ly = lidar_xy
    sin25 = math.sin(math.radians(25.0))
    S = segs.astype(np.float64).tolist()
    for i in shorts.tolist():
        ax1, ay1, ax2, ay2 = S[i][:4]
        uax, uay = (ax2 - ax1) / L[i], (ay2 - ay1) / L[i]
        for j in longs.tolist():
            cx1, cy1, cx2, cy2 = S[j][:4]
            Lj = float(L[j])
            ucx, ucy = (cx2 - cx1) / Lj, (cy2 - cy1) / Lj
            if abs(uax * ucx + uay * ucy) > sin25:
                continue
            dd = []
            for ex, ey in ((ax1, ay1), (ax2, ay2)):
                t = min(max((ex - cx1) * ucx + (ey - cy1) * ucy, 0.0), Lj)
                dd.append(math.hypot(ex - cx1 - t * ucx, ey - cy1 - t * ucy))
            if min(dd) > 40.0:
                continue
            fe = 0 if dd[0] > dd[1] else 1                         # the free end: 0 = first point, 1 = last
            fx, fy = (ax1, ay1) if fe == 0 else (ax2, ay2)
            side_l = (lx - cx1) * -ucy + (ly - cy1) * ucx
            side_f = (fx - cx1) * -ucy + (fy - cy1) * ucx
            if side_l * side_f <= 0.0:                             # the slab must stand toward the lidar
                continue
            rg = pieces[i]
            k = int(bu[rg[-1][1]]) + 1 if fe == 1 else int(bu[rg[0][0]]) - 1
            dn = float(d[k % n])
            if not (dn == 0.0 or dn >= math.hypot(fx - lx, fy - ly) + gap):
                continue
            segs[i, 6] = CLS_LIMITER
            out.append((ax1, ay1, ax2, ay2, 0.5, S[i][4]))
            break
    return out


def perceive(scan, cfg: dict, pose=None, df=None, motion=None) -> Perception | None:
    """One revolution -> Perception (module docstring).  None when scan is None.  pose: the rear-axle FIELD pose at
    the scan's mean time (t - lag) -> the prior; df: a field.DistField (the program's, so lot_map / known signs count
    as walls for the point classes); motion: (v_mps, w_radps) -> de-skew (P1)."""
    if scan is None:
        return None
    t0 = time.perf_counter()
    d = np.asarray(scan.dist)
    n, res = len(d), float(scan.res)
    a_deg, ca, sa, blind = _tables(n, res, cfg["blind"])
    conf = np.asarray(scan.conf)
    max_mm, min_mm = cfg["max_mm"], cfg["min_mm"]
    valid = (d > min_mm) & (d < max_mm) & ~blind
    if cfg["min_conf"] > 0:
        valid &= conf >= cfg["min_conf"]
    bcls = np.where(d == 0, NO_RETURN, 0).astype(np.uint8)
    b = np.flatnonzero(valid)
    if not len(b):
        return _empty(scan, cfg, pose, t0, bcls)
    r = d[b].astype(np.float64)
    px, py = cfg["pos"]
    X, Y = px + r * ca[b], py + r * sa[b]
    if motion is not None:
        X, Y = _deskew(X, Y, a_deg[b], scan, cfg, motion)
    rear, front, half = car = cfg["car"]
    bc = body_clearance(X, Y, car)
    npt = len(b)
    cls = np.zeros(npt, np.int8)
    lab = np.full(npt, -1, np.int16)
    clut = bc < CLUTTER_MM
    cls[clut] = CLS_CLUTTER
    w = np.flatnonzero(~clut)                                      # the working set, angular order
    m = len(w)
    segs = np.zeros((0, 7), np.float32)
    pil = np.zeros((0, 6), np.float32)
    pil_seat = np.zeros(0, np.int16)
    lims = []
    if m:
        bu = b[w].astype(np.int64)
        rw, xw, yw = r[w], X[w], Y[w]
        # ---- 2 clusters: adaptive breakpoints (the nearer range: the same break in either spin direction)
        lam = math.radians(cfg["break_deg"])
        rr = math.radians(res)

        def breaks(db, r1, r2, gap):
            dphi = db * rr
            with np.errstate(divide="ignore", invalid="ignore"):
                dmax = np.minimum(r1, r2) * np.sin(dphi) / np.sin(lam - dphi) + cfg["break_mm"]
            return (db > 2) | (dphi >= lam) | (gap > dmax)

        brk = breaks(np.diff(bu), rw[:-1], rw[1:], np.hypot(np.diff(xw), np.diff(yw))) if m > 1 else np.zeros(0, bool)
        if m > 2 and brk.any():                                    # the scan wraps: join the last cluster to the first
            wrap_b = breaks(np.array([bu[0] + n - bu[-1]]), rw[-1:], rw[:1],
                            np.array([math.hypot(xw[0] - xw[-1], yw[0] - yw[-1])]))[0]
            if not wrap_b:
                f = int(np.argmax(brk)) + 1
                order = np.r_[f:m, 0:f]
                w, rw, xw, yw = w[order], rw[order], xw[order], yw[order]
                bu = np.r_[bu[f:], bu[:f] + n]
                brk = np.r_[brk[f:], False, brk[:f - 1]]
        starts = np.r_[0, np.flatnonzero(brk) + 1]
        ends = np.r_[starts[1:] - 1, m - 1]
        cnt = ends - starts + 1
        labels = np.repeat(np.arange(len(starts)), cnt)
        lab[w] = labels
        # ---- 4 pillar candidates
        mx = np.add.reduceat(xw, starts) / cnt
        my = np.add.reduceat(yw, starts) / cnt
        rng_c = np.hypot(mx - px, my - py)
        # size: the silhouette ACROSS the mean ray and the depth ALONG it.  The returns' lateral span `ls` bounds the
        # true silhouette to ls .. ls + 2 beam spacings, so `width` = ls + one beam (the unbiased estimate) and the
        # test is ls + HALF a beam <= pillar_max_mm: a 50 mm square's silhouette and depth are <= 70.7 at any angle,
        # and seen diagonally at 0.95-1.6 m its 3-5 returns span 55-69 mm, which ls + one beam (83-87) rejected
        # (6 of 118 seat signs missed, 2026-09-23).  Not the plain chord either: 5-10 deg off a face one beam lands
        # anywhere along the 50 mm side face (chord + beam 82-90 mm for a real sign at 1.0 and 1.3 m).
        cdx, cdy = xw[ends] - xw[starts], yw[ends] - yw[starts]
        ux, uy = (mx - px) / np.maximum(rng_c, 1e-6), (my - py) / np.maximum(rng_c, 1e-6)
        radial, lateral = np.abs(cdx * ux + cdy * uy), np.abs(cdx * uy - cdy * ux)
        beam = rng_c * rr
        width = lateral + beam
        small = ((lateral + 0.5 * beam <= cfg["pillar_max_mm"]) & (radial <= cfg["pillar_max_mm"])
                 & (radial <= 2.5 * lateral + 20.0))
        pc = np.flatnonzero(small)
        nc_ = len(starts)
        lok, rok, both = np.zeros(nc_, bool), np.zeros(nc_, bool), np.zeros(nc_, bool)
        locc, rocc = np.zeros(nc_, bool), np.zeros(nc_, bool)
        if len(pc):
            # per bin: 1 clear (farther, empty, beyond max), 2 unknown (blind, too near, low conf, clutter), else 0
            gap = cfg["pillar_gap_mm"]
            unk = blind | ((d > 0) & (d <= min_mm))
            if cfg["min_conf"] > 0:
                unk = unk | ((conf < cfg["min_conf"]) & (d > 0))
            if clut.any():
                unk = unk.copy()
                unk[b[clut]] = True
            csz = np.zeros(n, np.int64)
            csz[bu % n] = cnt[labels]

            def state(bins, redge):
                bins = bins % n
                dn = d[bins].astype(np.float64)
                st_ = np.where((dn == 0) | (dn >= max_mm) | (dn >= redge + gap), 1, 0)
                return np.where(unk[bins] & ~((dn == 0) & ~blind[bins]), 2, st_), bins

            def side(e_idx, step_):
                """(ok, clear with evidence, occluded: the neighbour is NEARER by the gap -- an edge in front)"""
                re = rw[e_idx]
                s1, b1 = state(bu[e_idx] + step_, re)
                s2, _ = state(bu[e_idx] + 2 * step_, re)
                ghost = (s1 == 0) & (csz[b1] == 1) & (s2 != 0)
                d1 = d[b1].astype(np.float64)
                return (s1 != 0) | ghost, (s1 == 1) | (ghost & (s2 == 1)), (s1 == 0) & (d1 > 0) & (d1 <= re - gap)

            lok[pc], lev, locc[pc] = side(starts[pc], -1)
            rok[pc], rev, rocc[pc] = side(ends[pc], +1)
            both[pc] = lev & rev
        n1 = cnt == 1
        cand = small & lok & rok
        cxs = mx + ux * PUSH_MM
        cys = my + uy * PUSH_MM
        conf_c = (0.25 * (cnt >= 2) + 0.25 * (cnt >= 4) + 0.25 * both
                  + 0.25 * ((width >= 35.0) & (width <= 75.0)))
        seat_c = np.full(len(starts), -1, np.int64)
        wb = cfg["wall_band_mm"]
        if pose is not None:
            cxf, cyf = to_field(cxs, cys, pose)
            wall_d = field_wall_dist(cxf, cyf)
            sd = np.hypot(F.SEAT_XY[None, :, 0] - cxf[:, None], F.SEAT_XY[None, :, 1] - cyf[:, None])
            j = sd.argmin(axis=1)
            sdist = sd[np.arange(len(j)), j]
            drop = wall_d < wb
            if df is not None:
                drop |= (np.asarray(df(cxf, cyf)) < wb) & (sdist >= cfg["seat_snap_mm"])
            # nothing in the lot band is a sign (seats stand >= 400 mm off the wall): a limitation's 20 mm end face
            # seen from the lane looks exactly like one (5 of 60 lot scans, 2026-09-23)
            drop |= (cyf < LOT_BAND[1]) & (cyf > LOT_BAND[0]) & (np.abs(cxf) < LOT_BAND[2])
            support = (wall_d > wb) & (sdist < cfg["seat_snap_mm"])
            conf_c = conf_c + 0.25 * support
            seat_c = np.where(sdist < cfg["seat_snap_mm"], j, -1)
            # a sign half hidden behind a nearer edge (the island's corner, another sign) has a NEARER neighbour on
            # that side: pose-free that is a wall's visible end, at a seat off the walls it is the sign (2 of 118
            # seat signs, 2026-09-23)
            cand = small & (lok | (locc & support)) & (rok | (rocc & support))
            cand &= ~drop & (~n1 | (sdist < SEAT_N1_MM))
            conf_c = np.where(n1, np.minimum(conf_c, 0.5), conf_c)
        else:
            cand &= ~n1
        conf_c = np.minimum(conf_c, 1.0)
        pcl = np.flatnonzero(cand)
        if len(pcl):
            rng_p = np.hypot(cxs[pcl] - px, cys[pcl] - py)
            o = np.argsort(rng_p, kind="stable")
            pcl, rng_p = pcl[o], rng_p[o]
            pil = np.column_stack([cxs[pcl], cys[pcl], width[pcl], conf_c[pcl], cnt[pcl], rng_p]).astype(np.float32)
            pil_seat = seat_c[pcl].astype(np.int16)
            is_p = np.zeros(len(starts), bool)
            is_p[pcl] = True
            cls[w[is_p[labels]]] = CLS_PILLAR
        # ---- 5 with a pose: the wall band, then the lot's limitations
        if pose is not None:
            xf, yf = to_field(xw, yw, pose)
            dfv = np.asarray(df(xf, yf)) if df is not None else field_wall_dist(xf, yf)
            cw_ = cls[w]
            cw_ = np.where((cw_ == CLS_NONE) & (dfv < wb), CLS_WALL, cw_).astype(np.int8)
            lims += _lot_limiters(xw, yw, xf, yf, cw_, dfv if df is not None else None)
            cls[w] = cw_
        # ---- 3 lines: split-and-merge on the clusters that are not pillars
        lc = np.flatnonzero((cnt >= 4) & ~cand)
        runs = []
        tol_mm, tol_k = cfg["line_tol_mm"], cfg["line_tol_pct"] * 0.01
        for ci in lc.tolist():
            stack = [(int(starts[ci]), int(ends[ci]), ci)]
            while stack:
                i0, i1, _ = stack.pop()
                if i1 - i0 < 2:
                    runs.append((i0, i1, ci))
                    continue
                xs, ys = xw[i0:i1 + 1], yw[i0:i1 + 1]
                ddx, ddy = xs[-1] - xs[0], ys[-1] - ys[0]
                L = math.hypot(ddx, ddy)
                if L < 1e-6:
                    dist = np.hypot(xs - xs[0], ys - ys[0])
                else:
                    dist = np.abs(ddx * (ys - ys[0]) - ddy * (xs - xs[0])) / L
                kk = int(np.argmax(dist[1:-1])) + 1
                if dist[kk] > tol_mm + tol_k * rw[i0 + kk]:
                    stack.append((i0 + kk, i1, ci))
                    stack.append((i0, i0 + kk, ci))
                else:
                    runs.append((i0, i1, ci))
        runs = [rn for rn in runs if rn[1] - rn[0] >= 2]           # >= 3 points
        if runs:
            segs, pieces = _segments(runs, xw, yw, rw, cls[w], pose is not None, tol_mm, tol_k)
            if pose is None:
                # ---- 6 without a pose: long segments are walls, a short one standing on a wall is a limitation
                lims += _free_limiters(segs, pieces, bu, d, n, (px, py), cfg["pillar_gap_mm"])
                for si, pc_ in enumerate(pieces):
                    c_ = int(segs[si, 6])
                    if c_ in (CLS_WALL, CLS_LIMITER):
                        sel = w[np.concatenate([np.arange(i0, i1 + 1) for i0, i1 in pc_])]
                        cls[sel] = np.where(cls[sel] == CLS_NONE, c_, cls[sel])
    # ---- 8 sectors and the straight free runs
    nc = ~clut
    Xn, Yn = X[nc], Y[nc]
    sectors = np.full(N_SECTORS, np.inf)
    if len(Xn):
        k = ((np.degrees(np.arctan2(Yn, Xn - (front - rear) / 2.0)) + 180.0) // 10.0).astype(np.int64)
        np.minimum.at(sectors, np.clip(k, 0, N_SECTORS - 1), bc[nc])
    free_fwd = _swept_straight(Xn, Yn, rear, front, half, 0.0, max_mm) if len(Xn) else max_mm
    free_back = _swept_straight(-Xn, Yn, front, rear, half, 0.0, max_mm) if len(Xn) else max_mm
    bcls[b] = cls.astype(np.uint8)
    return Perception(
        t=float(scan.t), lag_s=cfg["lag"], X=X.astype(np.float32), Y=Y.astype(np.float32), b=b.astype(np.int16),
        cls=cls, lab=lab, segs=segs, pillars=pil,
        limiters=np.array(lims, np.float32).reshape(-1, 6), sectors=sectors.astype(np.float32),
        free_fwd=float(free_fwd), free_back=float(free_back), blind=list(cfg["blind"]),
        pose=None if pose is None else tuple(float(v) for v in pose[:3]), ms=(time.perf_counter() - t0) * 1000.0,
        res=res, bcls=bcls, pillar_seat=pil_seat, lidar_xy=cfg["pos"], _car=car)


def of(robot) -> Perception | None:
    """The latest scan's POSE-FREE perception, computed once per scan: cached on the Scan instance as `_perc` with the
    config key (a params change recomputes).  Thread-safe: two threads racing on a new scan both compute it and one
    result wins the attribute -- harmless, and no lock on the hub's or the program's path."""
    global computed
    sc = getattr(robot, "scan", None)
    if sc is None:
        return None
    cfg = config(getattr(robot, "p", None) or {})
    hit = getattr(sc, "_perc", None)
    if hit is not None and hit[0] == cfg["key"]:
        return hit[1]
    pr = perceive(sc, cfg)
    computed += 1
    try:
        sc._perc = (cfg["key"], pr)
    except AttributeError:                                        # a Scan type with __slots__: no cache, still right
        pass
    return pr
