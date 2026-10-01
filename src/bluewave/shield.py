"""The lidar safety layer (BRAIN_SPEC 5.3): below the planner, every drive command is checked against what the lidar
sees NOW, what it saw in the last `shield.memory_s` (for the sector the body hides), and the map around the car; the
command stands, slows, steers away, or brakes.  It never delays a stop and never widens a command (S10, S11).

Why it exists (mat 2026-09-23, stock A1, run 185306-wro_next): the pose was lost beside a pillar, two bad
relocalisations were accepted and the car then drove pressed against a black wall for 100 s -- the planner and the
map guard only know the map, so a wrong pose, an unknown obstacle or the wall the car is already touching are
invisible to them.  The lidar on the WLtoys build scans at 50 mm and sees walls, signs and limitations directly.

    sh = Shield(robot.p)                     # shield.on: -1 = on iff lidar.loc = 1
    bot = sh.wrap(robot)                     # drive() filtered, pulse() checked, the rest delegated
    loop:  sh.odom(ds, dth)                  # the same odometry step the localiser predicts with
           sh.scan(perc, odo.since)          # every new lidar_perc.Perception
           sh.virtual(pts_car)               # <= 5 Hz, pose confident: map walls, known signs, the lot (car frame)
           bot.mode = "park" | "drive"
           bot.drive(v, steer)

Every remembered point lives in the shield's own ODOMETRY frame (integrated from odom()), so carrying 15 scans to
"now" is one rigid transform of one array per filter() call instead of a Python loop over the odometry history.

The test (BRAIN_SPEC 5.3 step 3): need(v) = v^2 / (2 decel) + |v| latency + stop_mm; free = the arc length the rear
axle can travel on curvature tan(steer) / L_eff (L_eff = chassis.wheelbase_m, the EFFECTIVE wheelbase the car really
turns on) before the footprint -- grown `margin_mm` ahead / behind and `side_mm` to the sides while driving,
`park_margin_mm` all round in "park" -- touches a point (lidar_perc.swept, exact).  free >= need: ok; else the largest
|v'| with need(v') <= free if >= v_floor: slow; else (mode "drive" only) the nearest steering, every 3 deg, whose arc
is free for v_floor: steer; else brake.

Deviations from BRAIN_SPEC 5.3, each with the reason:
  * "park" mode never steers (spec: only "manual" does not).  A park leg is an arc planned to the millimetre and ended
    on the gyro / the lidar pose; a different lock ends it somewhere the executor does not measure.
  * pulse_ok() needs the pulse's real travel (the step table's pick, <= 24 mm) + shield.pulse_stop_mm (0) free at
    shield.pulse_margin_mm (0) inflation -- not mm + stop_mm (25) at park_margin_mm (8): every exit / park leg ENDS
    `car.park_margin_mm` (22 WLtoys, 13 stock) from a limitation or the wall by plan, and the nearest of ~20 noisy
    returns (sigma 6 mm) lies ~12 mm short of the face.  At 8 mm the check cut a correct PARK leg 13 mm short and the
    legs after it touched the limitation (sim, WLtoys lot seed 2 cw: 1 contact; 0 without the shield).  A pulse is
    refused only when it would run the body INTO a return: a wrong pose, not the plan's own margin.
  * "park" mode adds shield.park_stop_mm (10) to the braking distance, not stop_mm (25): at v_leg 0.12 the spec's
    need was 3 + 18 + 25 = 46 mm, and the continuous part of a park leg ends ~43 mm of axle arc before a planned end
    that is itself only park_margin_mm from the lot -- the sim braked a correct PARK leg at free 36-38 and ended the run
    park_blocked (WLtoys, lot, seed 1 cw).  The plan's margin IS the park's stop margin; the shield there guards a
    wrong pose.  A braked park leg hands over to the step pulses (park.Executor), each checked by pulse_ok.
"""
from __future__ import annotations

import math
import time
from collections import deque

import numpy as np

from . import field as F
from . import lidar_perc as LP

DEFAULTS = dict(on=-1, margin_mm=40.0, side_mm=15.0, park_margin_mm=8.0, stop_mm=25.0, latency_s=0.15, decel=-1.0,
                horizon_mm=900.0, memory_s=1.5, v_floor=0.12, brake_hold_s=0.6, virtual=1, manual=-1,
                pulse_stop_mm=0.0, pulse_margin_mm=0.0, park_stop_mm=10.0, steer_step_deg=3.0, steer_hold_s=0.2,
                blind_s=0.5, dead_s=1.0)

# map wall samples every 20 mm, field frame: the virtual points' walls (computed once)
_WALL_PTS = None


def _wall_pts():
    global _WALL_PTS
    if _WALL_PTS is None:
        out = []
        for x1, y1, x2, y2 in F.WALLS:
            n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 20.0) + 1)
            t = np.linspace(0.0, 1.0, n)
            out.append(np.stack([x1 + (x2 - x1) * t, y1 + (y2 - y1) * t], 1))
        _WALL_PTS = np.vstack(out)
    return _WALL_PTS


def box_pts(cx, cy, hx, hy, step=10.0):
    """The outline of an axis-aligned box (field frame) sampled every `step` mm: a sign (25, 25), a limitation."""
    nx, ny = max(2, int(2 * hx / step) + 1), max(2, int(2 * hy / step) + 1)
    xs, ys = np.linspace(cx - hx, cx + hx, nx), np.linspace(cy - hy, cy + hy, ny)
    return np.vstack([np.stack([xs, np.full(nx, cy - hy)], 1), np.stack([xs, np.full(nx, cy + hy)], 1),
                      np.stack([np.full(ny, cx - hx), ys], 1), np.stack([np.full(ny, cx + hx), ys], 1)])


def virtual_points(pose, signs=(), boxes=(), reach_mm: float = 1200.0):
    """The map around a CONFIDENT field pose as car-frame points: wall faces every 20 mm within `reach_mm`, each known
    sign (x, y) as its 50 x 50 outline, each extra box (cx, cy, hx, hy) -- the lot's limitations -- as its outline."""
    W = _wall_pts()
    x, y = float(pose[0]), float(pose[1])
    near = (np.abs(W[:, 0] - x) < reach_mm) & (np.abs(W[:, 1] - y) < reach_mm)
    parts = [W[near]]
    for sx, sy in signs:
        if abs(sx - x) < reach_mm and abs(sy - y) < reach_mm:
            parts.append(box_pts(sx, sy, F.PILLAR / 2.0, F.PILLAR / 2.0))
    for cx, cy, hx, hy in boxes:
        if abs(cx - x) < reach_mm + hx and abs(cy - y) < reach_mm + hy:
            parts.append(box_pts(cx, cy, hx, hy))
    P_ = np.vstack(parts) if parts else np.zeros((0, 2))
    return LP.field_to_car(P_[:, 0], P_[:, 1], pose)


class Shield:
    """See the module docstring.  Not thread-safe: one program thread owns it (the hub's manual shield, P1, would own
    its own instance)."""

    def __init__(self, robot_p: dict, overrides: dict | None = None):
        rp = robot_p or {}
        c = dict(DEFAULTS)
        c.update({k: v for k, v in (rp.get("shield") or {}).items() if k in DEFAULTS})
        c.update({k: v for k, v in (overrides or {}).items() if k in DEFAULTS and v is not None})
        self.c = c
        on = int(c["on"])
        self.enabled = bool(int((rp.get("lidar") or {}).get("loc", 0))) if on < 0 else bool(on)
        pc = LP.config(rp)
        self.car = pc["car"]                                   # (rear, front, half_w) from the rear axle
        self.lidar_xy = pc["pos"]
        self.blind = pc["blind"]
        self.min_mm, self.max_mm = pc["min_mm"], pc["max_mm"]
        self.wb = pc["wheelbase_mm"]                           # the EFFECTIVE wheelbase (chassis.wheelbase_m)
        self.mx = pc["steer_max_deg"]
        d = float(c["decel"])
        self.decel = d if d > 0 else float((rp.get("drive") or {}).get("stop_decel", 2.2))
        # a gpio_pwm pulse travels the step table's biggest row that fits (hw.Robot.pulse), not the distance asked
        self._steps = None
        if str((rp.get("drive") or {}).get("backend", "rrc")) == "gpio_pwm":
            from .speed import StepTable
            try:
                self._steps = StepTable((rp.get("step") or {}).get("table") or [])
                self._steps.pick(1.0)
            except (ValueError, TypeError, IndexError):
                self._steps = None
        self.reach = float(c["horizon_mm"]) + 400.0            # points farther than this from the rear axle never matter
        # the odometry frame: the car's pose in it, and every remembered point
        self.ox = self.oy = self.oth = 0.0
        self._field = False                                    # the frame is the field (set_pose), not odometry
        self.mem = deque()                                     # (t_scan, xo, yo) per scan, newest last
        self.t_scan = None                                     # Scan.t of the newest scan (monotonic)
        self.vpts = None                                       # (xo, yo) of the virtual points, odometry frame
        self.vt = 0.0
        # the outcome of the last filter() and the counters
        self.act, self.free, self.need, self.v_cap = "idle", None, None, None
        self.n_slow = self.n_steer = self.n_brake = 0
        self.blocked = False
        self._brake_since = None
        self._hold = None                                      # (steer, until): a steering the shield chose
        self.cpu_s = 0.0                                       # thread CPU spent in filter / pulse_ok (Shielded)
        self.tight = False                                     # the last footprint had a margin cut (fit_margins)
        self._blind = self._blind_table()

    # ------------------------------------------------------------------------------------------------ inputs
    def _blind_table(self):
        return [(lo % 360.0, hi % 360.0, hi - lo >= 360.0) for lo, hi in self.blind]

    def _in_blind(self, bearing_deg):
        a = np.asarray(bearing_deg) % 360.0
        out = np.zeros(a.shape, bool)
        for lo, hi, full in self._blind:
            if full:
                out[:] = True
            elif lo <= hi:
                out |= (a >= lo) & (a <= hi)
            else:
                out |= (a >= lo) | (a <= hi)
        return out

    def odom(self, ds: float, dth: float) -> None:
        """One odometry step of the car (mm along its heading, rad): the program's own (the one loc.predict gets)."""
        th = self.oth + dth / 2.0
        self.ox += ds * math.cos(th)
        self.oy += ds * math.sin(th)
        self.oth = (self.oth + dth + math.pi) % (2.0 * math.pi) - math.pi
        self._field = False

    def set_pose(self, x: float, y: float, th: float) -> None:
        """The program's CONFIDENT field pose as the shield's frame (instead of odom()): stored points then live in
        the field, each placed with the pose its own scan just corrected.  Dead reckoning on the drive's speed ran ahead
        of the car over a few park pulses (the duty model's steady speed for a 60 ms kick is 1.4 m/s), carried the
        lot's limitation 37 mm toward the nose, and the shield refused a correct exit leg (sim, WLtoys --lot-start).
        Coming from odom(), everything stored is first re-expressed in the field through this pose."""
        if not self._field and (self.mem or self.vpts is not None):
            c0, s0 = math.cos(self.oth), math.sin(self.oth)
            c1, s1 = math.cos(th), math.sin(th)

            def move(xo, yo):
                dx, dy = xo - self.ox, yo - self.oy                    # odometry frame -> car frame now
                X, Y = c0 * dx + s0 * dy, -s0 * dx + c0 * dy
                return x + c1 * X - s1 * Y, y + s1 * X + c1 * Y       # car frame now -> field

            self.mem = deque((t, *move(xo, yo)) for t, xo, yo in self.mem)
            if self.vpts is not None:
                self.vpts = move(*self.vpts)
        self.ox, self.oy, self.oth = float(x), float(y), float(th)
        self._field = True

    def _to_odo(self, X, Y):
        c, s = math.cos(self.oth), math.sin(self.oth)
        return self.ox + c * X - s * Y, self.oy + s * X + c * Y

    def scan(self, perc, motion_fn=None) -> None:
        """A new perception: its non-clutter points, carried from the scan's mean time to now (motion_fn(t) = the car's
        motion since t, loc.Odo.since; one call per scan), stored in the odometry frame for `memory_s`."""
        if perc is None:
            return
        m = perc.cls != LP.CLS_CLUTTER
        X, Y = perc.X[m].astype(np.float64), perc.Y[m].astype(np.float64)
        if motion_fn is not None and len(X):
            dx, dy, dth = motion_fn(float(perc.t) - float(perc.lag_s))
            c, s = math.cos(dth), math.sin(dth)
            Xs, Ys = X - dx, Y - dy
            X, Y = c * Xs + s * Ys, -s * Xs + c * Ys
        keep = np.hypot(X, Y) < self.reach
        xo, yo = self._to_odo(X[keep], Y[keep])
        self.t_scan = float(perc.t)
        self.mem.append((self.t_scan, xo, yo))
        while self.mem and self.t_scan - self.mem[0][0] > float(self.c["memory_s"]):
            self.mem.popleft()

    def virtual(self, pts_car) -> None:
        """The map's points around the car (car frame now) while the pose is confident; None clears them."""
        if pts_car is None or not int(self.c["virtual"]):
            self.vpts = None
            return
        X, Y = (np.asarray(v, np.float64).ravel() for v in pts_car)
        self.vpts = self._to_odo(X, Y)
        self.vt = time.monotonic()

    def reset(self) -> None:
        """Forget the brake (a relocalised program starts over); the memory stays."""
        self._brake_since, self.blocked, self._hold = None, False, None

    def forget(self) -> None:
        """Drop the scan memory and the map points: the field pose they were placed with was just corrected (a
        relocalisation, the lot fit), so in the field frame they now sit where nothing is."""
        self.mem.clear()
        self.vpts = None

    # ------------------------------------------------------------------------------------------------ geometry
    def points_now(self):
        """(X, Y) car frame now: the newest scan, + older scans and the virtual points where the lidar cannot see
        now (the blind sector, nearer than lidar.min_mm, farther than perc.max_mm)."""
        if not self.mem and self.vpts is None:
            return np.zeros(0), np.zeros(0)
        c, s = math.cos(self.oth), math.sin(self.oth)

        def car(xo, yo):
            dx, dy = xo - self.ox, yo - self.oy
            return c * dx + s * dy, -s * dx + c * dy

        parts_x, parts_y = [], []
        if self.mem:
            X, Y = car(self.mem[-1][1], self.mem[-1][2])
            parts_x.append(X)
            parts_y.append(Y)
        old = [(q[1], q[2]) for q in list(self.mem)[:-1]]
        if self.vpts is not None:
            old.append(self.vpts)
        if old:
            X, Y = car(np.concatenate([q[0] for q in old]), np.concatenate([q[1] for q in old]))
            px, py = self.lidar_xy
            r = np.hypot(X - px, Y - py)
            unseen = self._in_blind(np.degrees(np.arctan2(Y - py, X - px))) | (r <= self.min_mm) | (r >= self.max_mm)
            parts_x.append(X[unseen])
            parts_y.append(Y[unseen])
        return np.concatenate(parts_x), np.concatenate(parts_y)

    def footprint(self, mode: str, X=None, Y=None):
        """The body grown by the mode's margins -- each margin cut to what the points (X, Y) now leave free
        (fit_margins); `self.tight` says whether one was cut."""
        if mode == "park":
            a = s = b = float(self.c["park_margin_mm"])
        else:
            a = b = float(self.c["margin_mm"])
            s = float(self.c["side_mm"])
        self.tight = False
        if X is not None and len(X):
            a2, s2, b2 = self.fit_margins(X, Y, a, s, b)
            self.tight = (a2, s2, b2) != (a, s, b)
            a, s, b = a2, s2, b2
        return LP.grow(self.car, a, s, b)

    def fit_margins(self, X, Y, a: float, s: float, b: float, gap: float = 1.0):
        """(ahead, side, behind) margins no point lies inside of: a return nearer than a margin cuts that margin to
        its own clearance - `gap`.  Ahead of the nose it cuts `ahead`, behind the tail `behind`, beside the body
        `side`; beyond a corner (still inside with the margins so far) the side gives way, so a wall the car runs
        parallel to 10 mm off never shortens the stopping margin ahead.

        Why: a car ALREADY closer to something than the margin -- after a touch, a late brake, started parallel to a
        wall as TEST_DAY.md asks -- had the thing inside its footprint for every candidate, forward and reverse: the
        swept test then either braked everything or (the grown band beside a wall) left no arc free, and the car stood
        until the time ran out (sim: wro_wl_start10 104 brakes, lapcam 639; wro_wl_pose20 43 LOST cycles).  With the
        margins cut to the clearance, a motion that does not close on anything is free and one that does is braked as
        before; filter() then creeps (v_floor) until the margins are whole again."""
        rear, front, half = self.car
        inside = (X > -rear - b) & (X < front + a) & (np.abs(Y) < half + s)
        if not inside.any():
            return a, s, b
        x, ay = X[inside], np.abs(Y[inside]) - half
        ax, bx = x - front, -rear - x
        m = (ax > 0.0) & (ay <= 0.0)
        if m.any():
            a = min(a, max(0.0, float(ax[m].min()) - gap))
        m = (bx > 0.0) & (ay <= 0.0)
        if m.any():
            b = min(b, max(0.0, float(bx[m].min()) - gap))
        m = (ay > 0.0) & (ax <= 0.0) & (bx <= 0.0)
        if m.any():
            s = min(s, max(0.0, float(ay[m].min()) - gap))
        m = (ay > 0.0) & (ay < s) & (((ax > 0.0) & (ax < a)) | ((bx > 0.0) & (bx < b)))
        if m.any():
            s = min(s, max(0.0, float(ay[m].min()) - gap))
        return a, s, b

    def _stop(self, mode: str) -> float:
        return float(self.c["park_stop_mm"] if mode == "park" else self.c["stop_mm"])

    def need_mm(self, v: float, mode: str = "drive") -> float:
        return LP.need_mm(v, self.decel, float(self.c["latency_s"]), self._stop(mode))

    def v_allowed(self, free: float, mode: str = "drive") -> float:
        return LP.v_allowed(free, self.decel, float(self.c["latency_s"]), self._stop(mode))

    # ------------------------------------------------------------------------------------------------ the filter
    def filter(self, v: float, steer: float, now: float | None = None, mode: str = "drive") -> tuple:
        """(v, steer) to send instead of (v, steer).  mode "drive" | "park" | "manual"."""
        if v == 0.0 or not self.enabled:
            if v != 0.0:
                self.act = "off"
            else:
                # the program itself stops: a brake episode ends here.  Kept across stops, the brake of one park leg
                # plus the next leg's first braked command read as "braked 0.6 s" and ended a good park
                # (park_blocked, sim WLtoys lot seed 2 ccw)
                self._brake_since, self.blocked = None, False
                if self.act not in ("brake", "blind"):
                    self.act = "idle"
            return v, steer
        now = time.monotonic() if now is None else now
        sign = 1 if v > 0 else -1
        c = self.c
        age = math.inf if self.t_scan is None else now - self.t_scan
        cap = abs(v)
        act = "ok"
        if age > float(c["dead_s"]):                           # S12: no fresh scan for 1 s while moving
            return self._out("blind", 0.0, steer, None, None, now, brake=True)
        if age > float(c["blind_s"]):
            cap = min(cap, float(c["v_floor"]))
            act = "blind"
        X, Y = self.points_now()
        fp = self.footprint(mode, X, Y)
        if self.tight and cap > float(c["v_floor"]):
            # closer to something than the margins: only a creep, and only where it does not close on anything
            cap, act = float(c["v_floor"]), "slow"
        H = float(c["horizon_mm"])
        k = LP.curvature(steer, self.wb)
        free = LP.swept(X, Y, fp, k, sign, H, body=self.car)
        need = self.need_mm(cap, mode)
        if self._hold is not None and free < need and now < self._hold[1] and mode == "drive":
            st_h = self._hold[0]
            fh = LP.swept(X, Y, fp, LP.curvature(st_h, self.wb), sign, H, body=self.car)
            if fh >= self.need_mm(float(c["v_floor"])):
                va = min(cap, self.v_allowed(fh))
                return self._out("steer", sign * va, st_h, fh, self.need_mm(va), now)
        self._hold = None
        if free >= need:
            return self._out(act, sign * cap, steer, free, need, now)
        va = self.v_allowed(free, mode)
        if va >= float(c["v_floor"]):
            return self._out("slow", sign * min(cap, va), steer, free, need, now)
        if mode == "drive":
            step = float(c["steer_step_deg"])
            half = np.append(np.arange(0.0, self.mx - 1e-6, step), self.mx)   # 0, 3, ... and the full lock itself
            cand = np.unique(np.concatenate([-half, half]))
            cand = cand[np.argsort(np.abs(cand - steer), kind="stable")]
            ks = np.tan(np.radians(cand)) / max(self.wb, 1e-6)
            fr = LP.swept_many(X, Y, fp, ks, sign, H, body=self.car)
            ok = np.flatnonzero(fr >= self.need_mm(float(c["v_floor"])))
            if len(ok):
                i = int(ok[0])
                st2, f2 = float(cand[i]), float(fr[i])
                self._hold = (st2, now + float(c["steer_hold_s"]))
                va = min(cap, self.v_allowed(f2))
                return self._out("steer", sign * va, st2, f2, self.need_mm(va), now)
        return self._out("brake", 0.0, steer, free, need, now, brake=True)

    def _out(self, act, v, steer, free, need, now, brake=False):
        if act == "slow":
            self.n_slow += 1
        elif act == "steer":
            self.n_steer += 1
        elif brake:
            self.n_brake += 1
        if brake:
            self._brake_since = self._brake_since or now
            self.blocked = now - self._brake_since >= float(self.c["brake_hold_s"])
        else:
            self._brake_since, self.blocked = None, False
        self.act, self.free, self.need, self.v_cap = act, free, need, abs(v)
        return v, steer

    def pulse_travel(self, mm: float) -> float:
        """What a pulse asked for `mm` really travels: the step table's pick on the gpio_pwm drive (<= 24 mm), `mm` on
        the closed-loop board (a timed drive of exactly that)."""
        return float(self._steps.pick(abs(float(mm)))[0]) if self._steps is not None else abs(float(mm))

    def pulse_ok(self, sign: int, steer: float, mm: float, now: float | None = None) -> bool:
        """A step-mode pulse asked for `mm` in direction `sign` on this steering: may it run?  The footprint grown by
        park_margin_mm all round must travel the pulse's REAL travel + pulse_stop_mm on the arc (module docstring: why
        not + stop_mm).  Checking the distance asked instead refused a 24 mm kick because 94 mm were left (sim)."""
        if not self.enabled:
            return True
        now = time.monotonic() if now is None else now
        if self.t_scan is None or now - self.t_scan > float(self.c["dead_s"]):
            return False
        X, Y = self.points_now()
        m = float(self.c["pulse_margin_mm"])
        want = self.pulse_travel(mm) + float(self.c["pulse_stop_mm"])
        free = LP.swept(X, Y, LP.grow(self.car, m, m), LP.curvature(steer, self.wb), 1 if sign > 0 else -1,
                        want + 50.0)
        ok = free >= want
        if not ok:
            self.n_brake += 1
            self.act, self.free, self.need = "brake", free, want
        return ok

    def state(self) -> dict:
        """For pub(shield=...) and the run log: JSON scalars."""
        age = None if self.t_scan is None else int(round((time.monotonic() - self.t_scan) * 1000.0))
        return dict(act=self.act if self.enabled else "off",
                    free=None if self.free is None else int(round(min(self.free, 99999.0))),
                    need=None if self.need is None else int(round(self.need)),
                    v_cap=None if self.v_cap is None else round(self.v_cap, 3),
                    n_slow=self.n_slow, n_steer=self.n_steer, n_brake=self.n_brake, age_ms=age,
                    blocked=bool(self.blocked))

    def wrap(self, robot) -> "Shielded":
        return Shielded(self, robot)


def command(robot, shield: "Shield | None" = None, ttl_ms: int | None = None) -> "Shielded":
    """What a race program drives through: the shield when it is enabled (else commands pass unchanged), and a dead-man
    `ttl_ms` on every drive() (robot.prog_ttl_ms(), params safety.prog_ttl_ms): a program that stops renewing its
    command -- a stalled thread, a skipped control step -- stops the car instead of leaving the last command running
    unguarded (sim: a 1.0 s program stall at 0.50 m/s drove the WLtoys ~0.5 m, a camera stall in lapcam ran its last
    command into the wall at 0.55 m/s)."""
    s = Shielded(shield if shield is not None and shield.enabled else None, robot)
    s.ttl_ms = ttl_ms
    return s


class Shielded:
    """The robot as a program's motion commands should see it: drive() through Shield.filter in `mode`, pulse() only
    when Shield.pulse_ok, everything else (stop, yaw, scan, frame, ...) the robot's own.  A stop is never delayed.
    shield None (command() with the shield off, the stock A1): commands pass unchanged, with the ttl."""

    def __init__(self, shield: "Shield | None", robot):
        self._shield, self._robot = shield, robot
        self.mode = "drive"
        self.ttl_ms = None                                     # the dead-man every drive() carries (command())
        self.last_act = "idle"                                 # the shield's verdict on the last drive() (park.Executor
        #                                                        hands a braked leg to its step pulses)

    def drive(self, v_mps: float, steer_deg: float, ttl_ms: int | None = None) -> bool:
        ttl = self.ttl_ms if ttl_ms is None else ttl_ms
        if self._shield is None:
            return self._robot.drive(float(v_mps), float(steer_deg), ttl)
        c0 = time.thread_time()
        v, st = self._shield.filter(float(v_mps), float(steer_deg), time.monotonic(), self.mode)
        self._shield.cpu_s += time.thread_time() - c0
        self.last_act = "brake" if (v_mps != 0.0 and v == 0.0) else self._shield.act
        return self._robot.drive(v, st, ttl)

    def pulse(self, sign: int, steer_deg: float, mm: float) -> float:
        if self._shield is None:
            return self._robot.pulse(sign, steer_deg, mm)
        c0 = time.thread_time()
        ok = self._shield.pulse_ok(sign, steer_deg, mm)
        self._shield.cpu_s += time.thread_time() - c0
        if not ok:
            return 0.0
        return self._robot.pulse(sign, steer_deg, mm)

    def __getattr__(self, name):
        return getattr(self._robot, name)
