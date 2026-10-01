"""Where the car is on the field: a particle filter over (x, y, heading) in field.py's frame.

    predict  -- every loop, from the gyro's heading change and the commanded speed (the board regulates wheel speed on
                its encoders but never reports them, so v_cmd * dt * speed_scale IS the odometry)
    update   -- every camera frame, from the wall-base points vision.edges() found: a particle is as likely as its
                points are close to a known wall face (a likelihood field, field.DistField), with a per-point sigma that
                grows with range the way the floor model's error does (1 pixel = r^2 / (fy * h) mm).  With the HP60C
                depth on (params depth.walls, bluewave/depth.py) the columns it returned carry their depth range
                instead, with the depth sigma: the same update, pitch-free points

Why a filter and not a wall-follower: the camera sees +-29 deg ahead only, so no single frame shows both side walls
near the car; the map ties every wall piece the camera ever sees to one pose.  Pose = the REAR AXLE centre.
"""
from __future__ import annotations

import math

import numpy as np

from . import field as F


def wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def floor_sigma(X, Y):
    """The camera floor model's per-point sigma (mm): one pixel of base row = r^2 / (fy h) mm, + 15 mm.  A mixed
    update (depth wall points with their own sigma, bluewave/depth.py) gives the RGB points this one."""
    X, Y = np.asarray(X, float), np.asarray(Y, float)
    return 15.0 + 4.5e-5 * (X * X + Y * Y)


class Localizer:
    def __init__(self, n: int = 400, seed: int | None = None, dist: F.DistField | None = None):
        self.n = n
        self.rng = np.random.default_rng(seed)
        self.df = dist or F.DistField()
        self.P = np.zeros((n, 3))
        self.w = np.full(n, 1.0 / n)
        self.updates = 0
        self.last_fit = float("nan")                    # median |distance| of the last frame's points, mm

    # ------------------------------------------------------------------ initialisation
    def init(self, parts):
        """parts: [(x, y, th, sx, sy, sth, weight)] -- Gaussian blobs, or uniform boxes when a sigma is negative
        (then |sigma| is the half-width)."""
        tot = sum(p[6] for p in parts)
        rows = []
        for x, y, th, sx, sy, sth, wt in parts:
            k = max(1, int(round(self.n * wt / tot)))
            r = self.rng
            px = x + (r.uniform(-1, 1, k) * -sx if sx < 0 else r.normal(0, sx, k))
            py = y + (r.uniform(-1, 1, k) * -sy if sy < 0 else r.normal(0, sy, k))
            pt = th + (r.uniform(-1, 1, k) * -sth if sth < 0 else r.normal(0, sth, k))
            rows.append(np.stack([px, py, pt], 1))
        P = np.concatenate(rows)
        self.P = P[self.rng.choice(len(P), self.n, replace=len(P) < self.n)] if len(P) != self.n else P
        self.P[:, 2] = wrap(self.P[:, 2])
        self.w = np.full(self.n, 1.0 / self.n)
        self.updates = 0

    def init_start(self, heading: float | None = None):
        """Anywhere in the starting straight S, parallel to the walls within +-12 deg; both headings unless given."""
        hs = [heading] if heading is not None else [0.0, math.pi]
        self.init([(0.0, -1000.0, h, -560.0, -440.0, -math.radians(12), 1.0) for h in hs])

    # ------------------------------------------------------------------ filter
    def predict(self, ds: float, dth: float, s_ds: float = 0.10, s_th: float = 0.0015):
        n = self.n
        dsi = ds * (1.0 + self.rng.normal(0, s_ds, n)) + self.rng.normal(0, 0.6, n)
        thm = self.P[:, 2] + dth / 2
        self.P[:, 0] += dsi * np.cos(thm)
        self.P[:, 1] += dsi * np.sin(thm)
        self.P[:, 2] = wrap(self.P[:, 2] + dth + self.rng.normal(0, s_th + 0.02 * abs(dth), n))

    def update(self, X, Y, sig=None, temper: float | None = None):
        """X, Y: wall-base points in the car frame (mm, rear axle origin).  Returns the effective sample size."""
        X = np.asarray(X, float)
        Y = np.asarray(Y, float)
        m = len(X)
        if m < 4:
            return float(1.0 / np.sum(self.w ** 2))
        if sig is None:
            sig = floor_sigma(X, Y)
        c, s = np.cos(self.P[:, 2])[:, None], np.sin(self.P[:, 2])[:, None]
        px = self.P[:, 0:1] + c * X[None, :] - s * Y[None, :]
        py = self.P[:, 1:2] + s * X[None, :] + c * Y[None, :]
        d = self.df(px, py)
        ll = -0.5 * np.minimum((d / sig[None, :]) ** 2, 9.0).sum(1) / (temper or max(1.0, m / 8.0))
        lw = np.log(self.w + 1e-300) + ll
        lw -= lw.max()
        w = np.exp(lw)
        self.w = w / w.sum()
        best = int(np.argmax(self.w))
        self.last_fit = float(np.median(d[best]))
        self.updates += 1
        neff = float(1.0 / np.sum(self.w ** 2))
        if neff < self.n / 2:
            self.resample()
        return neff

    def resample(self):
        n = self.n
        pos = (self.rng.random() + np.arange(n)) / n
        idx = np.minimum(np.searchsorted(np.cumsum(self.w), pos), n - 1)
        self.P = self.P[idx].copy()
        self.P[:, 0] += self.rng.normal(0, 4.0, n)
        self.P[:, 1] += self.rng.normal(0, 4.0, n)
        self.P[:, 2] = wrap(self.P[:, 2] + self.rng.normal(0, math.radians(0.3), n))
        self.w = np.full(n, 1.0 / n)

    # ------------------------------------------------------------------ global search (the start)
    @staticmethod
    def beam_cost(poses, cam_x, az, rng_m, walls=F.WALLS, chunk=4000, sig=None):
        """Ray-cast cost of each pose (n, 3): per camera column, the map's first wall along the column's azimuth vs
        the measured range (both from the camera).  Unlike the likelihood field this KNOWS that a wall in front hides
        everything behind it, which is what separates "the island ends 1 m ahead" from "the outer wall continues".
        sig: per-column sigma (mm); None = the camera floor model's (a lidar view passes 12 + 0.012 r)."""
        sig = 15.0 + 4.5e-5 * rng_m * rng_m if sig is None else np.asarray(sig, float)
        out = np.empty(len(poses))
        x1, y1, x2, y2 = walls[:, 0], walls[:, 1], walls[:, 2], walls[:, 3]
        ex, ey = x2 - x1, y2 - y1
        for i in range(0, len(poses), chunk):
            P = poses[i:i + chunk]
            cx = P[:, 0] + cam_x * np.cos(P[:, 2])
            cy = P[:, 1] + cam_x * np.sin(P[:, 2])
            ang = P[:, 2][:, None] + az[None, :]                                  # (p, m)
            dx, dy = np.cos(ang)[..., None], np.sin(ang)[..., None]               # (p, m, 1)
            den = dx * ey - dy * ex                                              # (p, m, w)
            qx, qy = (x1[None, None, :] - cx[:, None, None]), (y1[None, None, :] - cy[:, None, None])
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (qx * ey - qy * ex) / den
                u = (qx * dy - qy * dx) / den
            t = np.where((den != 0) & (t > 0) & (u >= 0) & (u <= 1), t, np.inf).min(axis=2)
            r = np.minimum((t - rng_m[None, :]) / sig[None, :], 3.0)
            out[i:i + chunk] = np.sum(np.minimum(r * r, 9.0), axis=1)
        return out

    @staticmethod
    def valid(poses, margin: float = 85.0, island=None):
        """Rear-axle poses whose car could physically stand there: inside the outer walls, outside the island
        (the body centre is ~70 mm ahead of the rear axle).  island: (left, right, bottom, top) of a non-standard
        field's island (field.island_of); None = the standard +-500."""
        cx = poses[:, 0] + 70.0 * np.cos(poses[:, 2])
        cy = poses[:, 1] + 70.0 * np.sin(poses[:, 2])
        inside = (np.abs(cx) < F.HALF - margin) & (np.abs(cy) < F.HALF - margin)
        if island is None:
            isl = (np.abs(cx) < F.ISL + margin) & (np.abs(cy) < F.ISL + margin)
        else:
            l, r, b, t = (float(v) for v in island)
            isl = (cx > l - margin) & (cx < r + margin) & (cy > b - margin) & (cy < t + margin)
        return inside & ~isl

    def global_start(self, X, Y, cam_x: float, box=(-560.0, 560.0, -1460.0, -540.0), headings=(0.0, math.pi),
                     spread_deg: float = 14.0, keep: float = 2.0, step_xy: float = 40.0, step_th: float = 1.5):
        """Grid-search the start straight (25 mm, 1 deg; then 8 mm, 0.3 deg around the best) with the ray-cast cost,
        then seed the filter on every pose within `keep` cost units of the best -- so a genuinely ambiguous start
        stays ambiguous instead of being decided by chance.  Returns (best pose, cost, n_hypotheses)."""
        X, Y = np.asarray(X, float), np.asarray(Y, float)
        m = np.arange(len(X))[:: max(1, len(X) // 48)]
        az = np.arctan2(Y[m], X[m] - cam_x)
        rng_m = np.hypot(X[m] - cam_x, Y[m])
        gx, gy = np.arange(box[0], box[1] + 1, step_xy), np.arange(box[2], box[3] + 1, step_xy)
        ths = np.concatenate([h + np.radians(np.arange(-spread_deg, spread_deg + 0.1, step_th)) for h in headings])
        G = np.array(np.meshgrid(gx, gy, ths, indexing="ij")).reshape(3, -1).T
        G = G[self.valid(G)]
        c = self.beam_cost(G, cam_x, az, rng_m)
        best = G[int(np.argmin(c))]
        fx = np.arange(-step_xy, step_xy + 0.1, step_xy / 4)
        ft = np.radians(np.arange(-step_th, step_th + 0.01, step_th / 4))
        R = np.array(np.meshgrid(best[0] + fx, best[1] + fx, best[2] + ft, indexing="ij")).reshape(3, -1).T
        cr = self.beam_cost(R, cam_x, az, rng_m)
        best, cbest = R[int(np.argmin(cr))], float(cr.min())
        norm = cbest / max(1, len(m))
        good = G[c / max(1, len(m)) <= norm + keep]
        # one hypothesis per distinct heading family (their best), within `keep`
        hyps = [best]
        for h in headings:
            sel = good[np.abs(wrap(good[:, 2] - h)) < math.radians(spread_deg + 1)]
            if len(sel) and abs(wrap(best[2] - h)) > math.radians(spread_deg + 1):
                cs = self.beam_cost(sel, cam_x, az, rng_m)
                hyps.append(sel[int(np.argmin(cs))])
        self.init([(hp[0], hp[1], hp[2], 20.0, 20.0, math.radians(1.0), 1.0) for hp in hyps])
        return best, norm, len(hyps)

    def local_search(self, X, Y, cam_x: float, centre, half_xy: float = 450.0, half_th_deg: float = 25.0,
                     step_xy: float = 30.0, step_th: float = 1.5, prior_mm: float = 180.0, prior_deg: float = 10.0,
                     seed: bool = True):
        """Recovery: the ray-cast grid search around a remembered pose, then (seed) the filter re-seeded on the best.
        A Gaussian prior around `centre` (the odometry's guess) breaks the ties a one-wall view leaves: far from
        everything else, one straight wall fits a whole line of poses equally well.  Returns (best pose, cost/column).
        seed=False leaves the filter alone: a caller that may REJECT the pose seeds it itself (seed_at) -- seeding a
        rejected pose made it the filter's, and the shield's, confident pose until the next search."""
        X, Y = np.asarray(X, float), np.asarray(Y, float)
        m = np.arange(len(X))[:: max(1, len(X) // 48)]
        az = np.arctan2(Y[m], X[m] - cam_x)
        rng_m = np.hypot(X[m] - cam_x, Y[m])
        cx, cy, cth = centre
        G = np.array(np.meshgrid(np.arange(cx - half_xy, cx + half_xy + 1, step_xy),
                                 np.arange(cy - half_xy, cy + half_xy + 1, step_xy),
                                 cth + np.radians(np.arange(-half_th_deg, half_th_deg + 0.01, step_th)),
                                 indexing="ij")).reshape(3, -1).T
        G = G[self.valid(G)]
        if not len(G):                       # no pose a car could stand in near `centre`: nothing to search
            return np.asarray(centre, float), float("inf")

        def prior(Q):
            return (((Q[:, 0] - cx) ** 2 + (Q[:, 1] - cy) ** 2) / prior_mm ** 2
                    + (wrap(Q[:, 2] - cth) / math.radians(prior_deg)) ** 2) * len(m) / 8.0

        c = self.beam_cost(G, cam_x, az, rng_m) + prior(G)
        best = G[int(np.argmin(c))]
        R = np.array(np.meshgrid(best[0] + np.arange(-step_xy, step_xy + 0.1, step_xy / 4),
                                 best[1] + np.arange(-step_xy, step_xy + 0.1, step_xy / 4),
                                 best[2] + np.radians(np.arange(-step_th, step_th + 0.01, step_th / 4)),
                                 indexing="ij")).reshape(3, -1).T
        cr = self.beam_cost(R, cam_x, az, rng_m)
        k = int(np.argmin(cr + prior(R)))
        best = R[k]
        if seed:
            self.seed_at(best)
        return best, float(cr[k]) / max(1, len(m))

    def seed_at(self, pose):
        """The filter re-seeded tight around one pose (a relocalisation the caller accepted)."""
        self.init([(float(pose[0]), float(pose[1]), float(pose[2]), 20.0, 20.0, math.radians(1.0), 1.0)])

    # ------------------------------------------------------------------ lidar
    @staticmethod
    def lidar_points(scan, pos_mm=(0.0, 0.0), block_deg=((130.0, 230.0),), step_deg: float = 3.0,
                     rmin: float = 90.0, rmax: float = 2800.0):
        """A lidar revolution as wall points in the car frame (rear-axle origin) + per-point sigma, for update().

        Only meaningful when the scan plane is BELOW the 100 mm walls -- the stock LD19 at 145.7 mm sees the room
        behind the field, the BLUE WAVE body's inverted nose lidar scans at 50 mm.  `block_deg` are bearings the
        car's own body hides (the rear sector behind a nose-mounted lidar), in the car frame, degrees CCW from ahead."""
        if scan is None:
            return np.zeros(0), np.zeros(0), np.zeros(0)
        a = scan.angles()
        d = scan.dist.astype(float)
        keep = (d > rmin) & (d < rmax)
        aa = (a + 360.0) % 360.0
        for lo, hi in block_deg:
            keep &= ~((aa >= lo % 360.0) & (aa <= hi % 360.0)) if lo % 360.0 <= hi % 360.0 else \
                ~((aa >= lo % 360.0) | (aa <= hi % 360.0))
        idx = np.nonzero(keep)[0]
        if not len(idx):
            return np.zeros(0), np.zeros(0), np.zeros(0)
        stride = max(1, int(round(step_deg / max(scan.res, 1e-3))))
        idx = idx[::stride]
        t = np.radians(a[idx])
        X = pos_mm[0] + d[idx] * np.cos(t)
        Y = pos_mm[1] + d[idx] * np.sin(t)
        sig = 12.0 + 0.012 * d[idx]                           # LD19: ~1 % range noise + mounting / timing
        return X, Y, sig

    # ------------------------------------------------------------------ camera self-calibration
    @staticmethod
    def pitch_search(u, v, g, pose, span_deg: float = 5.0, step_deg: float = 0.25, pose_mm: float = 60.0,
                     pose_deg: float = 3.0):
        """The camera pitch that makes the wall bases fit the map best, searched jointly with a small pose window.

        A 1 deg pitch error wrecked the map program in the sim (116 mm pose error, 47-71 contacts per race) and the
        real mount moved ~10 deg in a day, so the pitch is fitted on the car, not trusted from params: the same pixels
        re-projected with each candidate pitch, the ray-cast cost at the best nearby pose.
        Returns (pitch_deg, pose, cost per column)."""
        u, v = np.asarray(u, float), np.asarray(v, float)
        m = np.arange(len(u))[:: max(1, len(u) // 48)]
        u, v = u[m], v[m]
        x0, y0, t0 = pose
        dx = np.arange(-pose_mm, pose_mm + 0.1, pose_mm / 2)
        dt = np.radians(np.arange(-pose_deg, pose_deg + 0.01, pose_deg / 2))
        G = np.array(np.meshgrid(x0 + dx, y0 + dx, t0 + dt, indexing="ij")).reshape(3, -1).T
        best = (math.inf, None, None)
        base = float(g.cam["pitch_deg"])
        for p in base + np.arange(-span_deg, span_deg + 1e-6, step_deg):
            gp = g.with_pitch(p)
            X, Y = gp.floor(u, v)
            ok = np.isfinite(X) & (np.hypot(X, Y) < 2800.0)
            if ok.sum() < 8:
                continue
            az = np.arctan2(Y[ok], X[ok] - gp.pos[0])
            rng = np.hypot(X[ok] - gp.pos[0], Y[ok])
            c = Localizer.beam_cost(G, gp.pos[0], az, rng) / ok.sum()
            i = int(np.argmin(c))
            if c[i] < best[0]:
                best = (float(c[i]), float(p), G[i])
        return best[1], best[2], best[0]

    def pitch_step(self, u, v, g, step_deg: float = 0.25):
        """One tracking step while driving: the likelihood-field fit of this frame at the filter's pose for pitch
        -step / 0 / +step; returns the direction (-1, 0, +1) that fits better by a clear margin."""
        x, y, th, _s, _t = self.estimate()
        c, s = math.cos(th), math.sin(th)
        costs = []
        for d in (-step_deg, 0.0, step_deg):
            gp = g.with_pitch(float(g.cam["pitch_deg"]) + d)
            X, Y = gp.floor(u, v)
            ok = np.isfinite(X) & (np.hypot(X, Y) < 2600.0)
            if ok.sum() < 12:
                return 0
            fx, fy = x + c * X[ok] - s * Y[ok], y + s * X[ok] + c * Y[ok]
            costs.append(float(np.median(self.df(fx, fy))))
        lo, mid, hi = costs
        if lo < mid - 2.0 and lo < hi:
            return -1
        if hi < mid - 2.0 and hi < lo:
            return 1
        return 0

    # ------------------------------------------------------------------ output
    def estimate(self):
        """(x, y, th, sxy, sth): weighted mean and spread (mm, rad)."""
        w = self.w
        x, y = float(w @ self.P[:, 0]), float(w @ self.P[:, 1])
        th = math.atan2(float(w @ np.sin(self.P[:, 2])), float(w @ np.cos(self.P[:, 2])))
        sxy = math.sqrt(float(w @ ((self.P[:, 0] - x) ** 2 + (self.P[:, 1] - y) ** 2)))
        sth = math.sqrt(float(w @ wrap(self.P[:, 2] - th) ** 2))
        return x, y, th, sxy, sth


# ------------------------------------------------------------------ latency
class Odo:
    """Dead-reckoning increments with times, to carry a delayed camera frame to the present."""

    def __init__(self):
        self.h = []                                    # (t, ds, dth)

    def add(self, t, ds, dth):
        self.h.append((t, ds, dth))
        if len(self.h) > 200:
            del self.h[:50]

    def since(self, t0):
        """(dx, dy, dth) of the car since t0, in the car frame AT t0."""
        x = y = th = 0.0
        for t, ds, dth in self.h:
            if t > t0:
                x += ds * math.cos(th + dth / 2)
                y += ds * math.sin(th + dth / 2)
                th += dth
        return x, y, th


def to_now(X, Y, motion):
    dx, dy, dth = motion
    c, s = math.cos(dth), math.sin(dth)
    Xs, Ys = X - dx, Y - dy
    return c * Xs + s * Ys, -s * Xs + c * Ys


def from_now(X, Y, motion):
    """The inverse of to_now: points in the car frame now -> the car frame at t0 (motion = Odo.since(t0))."""
    dx, dy, dth = motion
    c, s = math.cos(dth), math.sin(dth)
    return c * X - s * Y + dx, s * X + c * Y + dy
