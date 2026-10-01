"""The 24 legal sign seats and what each holds (BRAIN_SPEC 5.2): the LIDAR says whether a sign stands on a seat and
where (mm-accurate, pitch-free), the CAMERA says its colour.  Remembered for the whole run: signs do not move, so
laps 2-3 get every seat for free.

    seats = SeatMap(robot.p)
    seats.lidar(perc, pose_scan, lidar_pos, scan=sc, confident=True)   # every scan (perc WITH the pose prior)
    seats.camera(dets, pose_frame, cam_xy)                             # every frame's vision.pillars()
    if seats.to_planner(plan): plan.rebuild()                          # the planner's seat dict, written only here

Evidence per seat j (field.SEATS order):
    hits    a lidar sign (perc.pillars, conf >= 0.5) whose field centre lies within perc.seat_snap_mm of the seat;
            its running mean is the seat's measured position (the planner's lane off the wall, sign_d)
    empty   the lidar saw PAST the seat: every bin across the seat's bearing window returned farther than the seat +
            60 mm.  The window is the sign's half width + the pose's own uncertainty (sxy + r sth), not the half width
            alone: 1 deg of heading error is 26 mm at 1.5 m, more than half a sign, and a narrow window then counted a
            real sign's seat as seen through.  Only with sth / sxy small (the caller's `confident`), 150-2000 mm
    red / green   camera votes.  A detection is matched to an EXISTING seat by bearing from the camera (lidar_perc.
            associate: pitch-free to first order, one-to-one, range gate 35 %); a detection that matches none falls back
            to the stock rule -- its floor position snapped to the nearest seat within cam_snap_mm -- a camera-only vote

A seat EXISTS when hits >= hits_min and hits > empty_ratio x empty.  Its colour = the majority of >= votes_min votes.
The planner's key is (section, along index): the row (400 / 600 mm off the wall) never changes the pass lane, so the
two rows of one key are merged -- the row with the most hits gives the position, the votes of both are summed, and the
key's `empty` is 0 when a row exists (the empties of an empty neighbour row must not outvote a real sign's colour).
"""
from __future__ import annotations

import math

import numpy as np

from . import field as F
from . import lidar_perc as LP

DEFAULTS = dict(hits_min=2, votes_min=2, cam_snap_mm=200.0, bearing_tol_deg=4.0, range_gate=0.35, empty_ratio=2.0,
                empty_rmin=150.0, empty_rmax=2000.0, empty_margin_mm=60.0, pillar_conf=0.5)
N = len(F.SEATS)


class SeatMap:
    def __init__(self, p: dict, overrides: dict | None = None):
        p = p or {}
        c = dict(DEFAULTS)
        c.update({k: v for k, v in (overrides or {}).items() if k in DEFAULTS})
        c["snap_mm"] = float((p.get("perc") or {}).get("seat_snap_mm", 120.0))
        self.c = c
        self.cfg = LP.config(p)
        self.hits = np.zeros(N, np.int64)
        self.sx, self.sy = np.zeros(N), np.zeros(N)           # sums of the lidar centres (running mean = s / hits)
        self.empty = np.zeros(N, np.int64)
        self.red = np.zeros(N, np.int64)
        self.green = np.zeros(N, np.int64)
        self.cam_only = np.zeros(N, np.int64)                  # votes that came by snapping, not by association
        self.cx, self.cy, self.cn = np.zeros(N), np.zeros(N), np.zeros(N, np.int64)   # camera floor positions
        self._planned = None                                   # what to_planner wrote last (colours, existence)
        self._obs, self._obs_min = [], 3                       # off-seat things: [sum x, sum y, hits] (off_seat)

    # ------------------------------------------------------------------------------------------------ evidence
    def exists(self, j: int) -> bool:
        h = int(self.hits[j])
        return h >= int(self.c["hits_min"]) and h > float(self.c["empty_ratio"]) * int(self.empty[j])

    def all_resolved(self, empty_min: int = 2) -> bool:
        """Every seat settled: a sign the lidar confirmed AND its colour, or the lidar saw past it `empty_min` times
        and no sign -- laps 2-3 on the lidar build.  Then the camera's colour search adds nothing: wro_next skips
        vision.pillars, its biggest stage (60-65 ms/s of laptop CPU, ~3-4x that on the Pi: BRAIN_SPEC 13).  A seat
        that changes (a sign found, a colour lost) makes it False again at once."""
        for j in range(N):
            if self.exists(j):
                if self.colour(j) is None:
                    return False
            elif int(self.empty[j]) < empty_min:
                return False
        return True

    def colour(self, j: int):
        r, g = int(self.red[j]), int(self.green[j])
        if max(r, g) < int(self.c["votes_min"]) or r == g:
            return None
        return "red" if r > g else "green"

    def xy(self, j: int):
        if self.hits[j] > 0:
            return float(self.sx[j] / self.hits[j]), float(self.sy[j] / self.hits[j])
        if self.cn[j] > 0:
            return float(self.cx[j] / self.cn[j]), float(self.cy[j] / self.cn[j])
        return None

    def lidar(self, perc, pose_scan, lidar_pos=None, scan=None, confident: bool = True, sxy: float = 0.0,
              sth: float = 0.0) -> list:
        """One scan's evidence.  perc: its Perception (with the pose prior: signs on map walls already dropped);
        pose_scan: the rear-axle field pose at the scan's mean time; scan: the lidar.Scan (for the empties); sxy mm /
        sth rad: the pose's spread (widens the empties' window).  Returns the seats whose existence changed."""
        if perc is None or pose_scan is None:
            return []
        before = [self.exists(j) for j in range(N)]
        P_ = perc.pillars
        if len(P_):
            P_ = P_[P_[:, 3] >= float(self.c["pillar_conf"])]
        if len(P_):
            xf, yf = LP.to_field(P_[:, 0], P_[:, 1], pose_scan)
            d = np.hypot(F.SEAT_XY[None, :, 0] - xf[:, None], F.SEAT_XY[None, :, 1] - yf[:, None])
            j = d.argmin(axis=1)
            ok = d[np.arange(len(j)), j] < self.c["snap_mm"]
            for jj, x_, y_ in zip(j[ok].tolist(), xf[ok].tolist(), yf[ok].tolist()):
                self.hits[jj] += 1
                self.sx[jj] += x_
                self.sy[jj] += y_
        if confident and scan is not None:
            lp = lidar_pos if lidar_pos is not None else self.cfg["pos"]
            Xs, Ys = LP.field_to_car(F.SEAT_XY[:, 0], F.SEAT_XY[:, 1], pose_scan)
            self.empty += self._seen_past(scan, Xs, Ys, lp, sxy, sth)
        return [j for j in range(N) if self.exists(j) != before[j]]

    def _seen_past(self, scan, X, Y, lidar_pos, sxy, sth):
        """1 per seat the scan saw past (module docstring), else 0."""
        out = np.zeros(len(X), np.int64)
        d = np.asarray(scan.dist)
        n, res = len(d), float(scan.res)
        _a, _c, _s, blind = LP._tables(n, res, self.cfg["blind"])
        px, py = float(lidar_pos[0]), float(lidar_pos[1])
        r = np.hypot(X - px, Y - py)
        ang = np.degrees(np.arctan2(Y - py, X - px))
        c = self.c
        for i in np.flatnonzero((r >= c["empty_rmin"]) & (r <= c["empty_rmax"])).tolist():
            half = math.degrees(math.atan2(LP.PUSH_MM + sxy + r[i] * sth, r[i]))
            k0 = int(math.floor((ang[i] - half + 180.0) / res))
            k1 = int(math.floor((ang[i] + half + 180.0) / res))
            ks = np.arange(k0, k1 + 1) % n
            dk = d[ks]
            if blind[ks].any() or (dk == 0).any() or (dk <= self.cfg["min_mm"]).any():
                continue
            if (dk > r[i] + float(c["empty_margin_mm"])).all():
                out[i] = 1
        return out

    def camera(self, dets, pose_frame, cam_xy=(0.0, 0.0)) -> list:
        """One frame's vision.pillars() output (car frame AT THE EXPOSURE, floor points) at pose_frame, the rear-axle
        field pose at that exposure; cam_xy: the lens in the car frame.  Returns the seats whose colour changed."""
        if pose_frame is None:
            return []
        xy, cols, _idx = LP.dets_xy(dets)
        if not len(xy):
            return []
        before = [self.colour(j) for j in range(N)]
        ex = np.array([j for j in range(N) if self.exists(j)], np.int64)
        used = set()
        if len(ex):
            tx, ty = LP.field_to_car(F.SEAT_XY[ex, 0], F.SEAT_XY[ex, 1], pose_frame)
            for di, ti, _dphi in LP.associate(xy, np.stack([tx, ty], 1), cam_xy, float(self.c["bearing_tol_deg"]),
                                              float(self.c["range_gate"])):
                self._vote(int(ex[ti]), cols[di])
                used.add(di)
        rest = [i for i in range(len(xy)) if i not in used]
        if rest:
            fx, fy = LP.to_field(xy[rest, 0], xy[rest, 1], pose_frame)
            for i, x_, y_ in zip(rest, fx.tolist(), fy.tolist()):
                dd = np.hypot(F.SEAT_XY[:, 0] - x_, F.SEAT_XY[:, 1] - y_)
                j = int(np.argmin(dd))
                if dd[j] < float(self.c["cam_snap_mm"]):
                    self._vote(j, cols[i])
                    self.cam_only[j] += 1
                    self.cx[j] += x_
                    self.cy[j] += y_
                    self.cn[j] += 1
        return [j for j in range(N) if self.colour(j) != before[j]]

    def _vote(self, j: int, colour: str):
        if colour == "red":
            self.red[j] += 1
        elif colour == "green":
            self.green[j] += 1

    # ------------------------------------------------------------------------------------------------ outputs
    def state(self, j: int) -> dict:
        h, e = int(self.hits[j]), int(self.empty[j])
        return dict(xy=self.xy(j), hits=h, empty=e, red=int(self.red[j]), green=int(self.green[j]),
                    colour=self.colour(j), exists=self.exists(j), conf=round(self.conf(j), 2),
                    cam_only=int(self.cam_only[j]))

    def conf(self, j: int) -> float:
        h, e = int(self.hits[j]), int(self.empty[j])
        if h <= 0:
            return 0.0
        return min(1.0, h / 6.0) * h / (h + e)

    def confirmed(self) -> list:
        """[(j, x, y, colour | None)] of the seats that exist (the lidar's own centre)."""
        out = []
        for j in range(N):
            xy = self.xy(j) if self.exists(j) else None
            if xy is not None:
                out.append((j, xy[0], xy[1], self.colour(j)))
        return out

    def uncoloured_pts(self) -> list:
        """Existing seats without a colour yet, as their 50 mm square's corners + centre: the guard and the corner
        fillets must avoid them before the colour is known."""
        out = []
        for j, x, y, col in self.confirmed():
            if col is None:
                out += [(x + ox, y + oy) for ox in (-25.0, 0.0, 25.0) for oy in (-25.0, 0.0, 25.0)]
        return out

    def to_planner(self, plan) -> bool:
        """plan.seat from the evidence (module docstring: the key merge).  True when any key's colour or any seat's
        existence changed since the last call -- the caller then refreshes plan.extra_pts and rebuilds the path."""
        new = {}
        for k in range(4):
            for ai in range(len(F.SEAT_ALONG)):
                rows = [k * 6 + ai * 2 + ci for ci in range(2)]
                red = int(sum(self.red[j] for j in rows))
                green = int(sum(self.green[j] for j in rows))
                ex = [j for j in rows if self.exists(j)]
                if ex:
                    jb = max(ex, key=lambda j: int(self.hits[j]))
                    empty = 0
                else:
                    jb = max(rows, key=lambda j: (int(self.red[j] + self.green[j]), -int(self.empty[j])))
                    empty = int(self.empty[jb])
                if not ex and red + green == 0:
                    continue
                v: dict = {"red": red, "green": green, "empty": empty}
                xy = self.xy(jb)
                if xy is not None:
                    v["xy"] = (xy[0], xy[1], 1)
                new[(k, ai)] = v
        old_cols = {q: plan.colour(q) for q in plan.seat}
        plan.seat = new
        cols = {q: plan.colour(q) for q in new}
        key = (tuple(sorted((q, c_) for q, c_ in cols.items() if c_)), tuple(self.exists(j) for j in range(N)))
        changed = key != self._planned or {q: c_ for q, c_ in old_cols.items() if c_} != dict(key[0])
        self._planned = key
        return changed

    def off_seat(self, perc, pose_scan, min_conf: float = 0.5, snap_mm: float = 60.0, hits_min: int = 3,
                 max_n: int = 12) -> None:
        """Sign-sized things the lidar sees AWAY from every seat (a dropped part, a misplaced sign -- nothing the
        planner's lanes know), remembered in the FIELD frame for the whole run: `obstacles()` gives them to the
        shield's map points (the sector behind the nose lidar is blind, and the 1.5 s scan memory forgets a thing the
        car passed before it has to back off past it) and to the planner's guard.  Measured failure it fixes (sim,
        WLtoys, --obstacles 3 seed 3 cw): after 30 LOST cycles in front of an off-seat block the car had turned round,
        backed off blind into it and touched it 4 times."""
        if perc is None or pose_scan is None or not len(perc.pillars):
            return
        P_ = perc.pillars[perc.pillars[:, 3] >= min_conf]
        if not len(P_):
            return
        xf, yf = LP.to_field(P_[:, 0], P_[:, 1], pose_scan)
        d = np.hypot(F.SEAT_XY[None, :, 0] - xf[:, None], F.SEAT_XY[None, :, 1] - yf[:, None]).min(axis=1)
        for x_, y_ in zip(xf[d >= self.c["snap_mm"]].tolist(), yf[d >= self.c["snap_mm"]].tolist()):
            best = None
            for ob in self._obs:
                dd = math.hypot(ob[0] / ob[2] - x_, ob[1] / ob[2] - y_)
                if dd < snap_mm and (best is None or dd < best[0]):
                    best = (dd, ob)
            if best is not None:
                best[1][0] += x_
                best[1][1] += y_
                best[1][2] += 1
            elif len(self._obs) < 4 * max_n:
                self._obs.append([x_, y_, 1])
        self._obs.sort(key=lambda ob: -ob[2])
        self._obs_min = hits_min

    def clear_obstacles(self) -> None:
        """Forget the off-seat things (the pose they were placed with was corrected: a relocalisation, the lot fit)."""
        self._obs = []

    def obstacles(self) -> list:
        """[(x, y)] field centres of the off-seat things seen >= hits_min times (the most-seen first, <= 12)."""
        return [(ob[0] / ob[2], ob[1] / ob[2]) for ob in self._obs if ob[2] >= self._obs_min][:12]

    # ------------------------------------------------------------------------------------------------ BRAIN4_SPEC 5.3
    def export(self) -> dict:
        """Every seat's evidence, JSON-able: {"seats": [{j, hits, sx, sy, empty, red, green, cam_only, cx, cy, cn}],
        "obstacles": [[x, y, n]]} -- a survey file's seats and wro_next's handover (import_ restores it exactly)."""
        rows = []
        for j in range(N):
            rows.append(dict(j=j, hits=int(self.hits[j]), sx=round(float(self.sx[j]), 1), sy=round(float(self.sy[j]), 1),
                             empty=int(self.empty[j]), red=int(self.red[j]), green=int(self.green[j]),
                             cam_only=int(self.cam_only[j]), cx=round(float(self.cx[j]), 1),
                             cy=round(float(self.cy[j]), 1), cn=int(self.cn[j])))
        return dict(seats=rows, obstacles=[[round(ob[0], 1), round(ob[1], 1), int(ob[2])] for ob in self._obs])

    def import_(self, d: dict) -> None:
        """The evidence of an export() (a survey's, the survey lap's handover): replaces this map's."""
        for r in (d or {}).get("seats", []):
            j = int(r["j"])
            if not 0 <= j < N:
                continue
            self.hits[j], self.sx[j], self.sy[j] = int(r.get("hits", 0)), float(r.get("sx", 0.0)), float(r.get("sy", 0.0))
            self.empty[j], self.red[j], self.green[j] = int(r.get("empty", 0)), int(r.get("red", 0)), int(r.get("green", 0))
            self.cam_only[j] = int(r.get("cam_only", 0))
            self.cx[j], self.cy[j], self.cn[j] = float(r.get("cx", 0.0)), float(r.get("cy", 0.0)), int(r.get("cn", 0))
        self._obs = [[float(a), float(b), int(n)] for a, b, n in (d or {}).get("obstacles", [])]
        self._planned = None

    def rotate(self, k: int) -> None:
        """The whole map in the frame rotated by k x 90 deg CCW (field.rot_seat / field.rot): a frame found to be one
        of the field's twins after the fact (W21)."""
        k %= 4
        if not k:
            return
        idx = np.array([F.rot_seat(j, k) for j in range(N)])
        for name in ("hits", "empty", "red", "green", "cam_only", "cn"):
            a = getattr(self, name)
            b = np.zeros_like(a)
            b[idx] = a
            setattr(self, name, b)
        for xs, ys in (("sx", "sy"), ("cx", "cy")):
            X, Y = getattr(self, xs), getattr(self, ys)
            RX, RY = np.zeros(N), np.zeros(N)
            for j in range(N):
                RX[idx[j]], RY[idx[j]] = F.rot(k, float(X[j]), float(Y[j]))
            setattr(self, xs, RX)
            setattr(self, ys, RY)
        self._obs = [[*F.rot(k, ob[0], ob[1]), ob[2]] for ob in self._obs]     # sums rotate as their mean does
        self._planned = None

    def live_rows(self) -> list:
        """[(x, y, colour | None, hits, conf, j)] for pub(lpil=...): every existing seat, and every seat the camera
        alone has coloured (hits 0)."""
        out = []
        for j in range(N):
            ex, col = self.exists(j), self.colour(j)
            if not ex and col is None:
                continue
            xy = self.xy(j)
            if xy is None:
                continue
            out.append((xy[0], xy[1], col, int(self.hits[j]), self.conf(j), j))
        return out
