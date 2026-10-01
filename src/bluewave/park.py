"""Parallel-parking legs for the WRO parking lot, planned on the car's true footprint.

The lot [RULE p.8, 13.25-13.29]: flush against the outer wall, 200 mm deep, 1.5 x the robot's length long (320 mm for
the 213 mm MentorPi A1), between two 200 x 20 mm magenta limitations.  The car has 53 mm of play at each end and its
minimum turning circle cannot swing the nose past the front limitation in one move (the front outer corner would cross
it ~46 mm from the wall), so leaving is a wiggle: reverse straight to the rear limitation, then forward on full lock
toward the field / reverse on full opposite lock, each leg stopped `margin` mm before anything, until one forward leg
runs free.

LOT FRAME: x along the wall in the travel direction, y = distance from the wall face into the field, heading 0 = +x.
Counter-clockwise rounds face +x in the start straight with the wall on the right, which is exactly this frame (y_lot
= y_field + 1500); clockwise rounds are its mirror image (x -> -x, steering sign flipped).

    legs = plan_exit(car)            # [(direction, steer_sign, dtheta_rad, dist_mm)], parked -> out
    legs_in = reverse(legs)          # the same path driven backwards: out -> parked

Kinematics are exact arcs of the rear axle (bicycle model); a leg is executed on the GYRO angle (turning legs) or on
distance (straight legs), because the car has no wheel encoders and the gyro is its most precise sensor.
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass

import numpy as np

LOT_LEN, LOT_DEPTH = 320.0, 200.0
LIMIT_T = 20.0


@dataclass
class Car:
    rear: float = 34.0          # body behind the rear axle, mm
    front: float = 179.0        # body ahead of the rear axle
    half_w: float = 80.5        # half width (159 mm + 1 mm per side)
    wheelbase: float = 145.0
    steer_max_deg: float = 26.0  # the planning lock -- keep a few degrees below the servo's for the controller

    @property
    def r_min(self) -> float:
        return self.wheelbase / math.tan(math.radians(self.steer_max_deg))

    def corners(self, x, y, th):
        c, s = math.cos(th), math.sin(th)
        return np.array([(x + c * a - s * b, y + s * a + c * b)
                         for a, b in ((-self.rear, -self.half_w), (self.front, -self.half_w),
                                      (self.front, self.half_w), (-self.rear, self.half_w))])


def obstacles(lot_len: float = LOT_LEN):
    """Convex polygons in the lot frame: the wall (y <= 0) and the two limitations."""
    wall = np.array([(-2000.0, -500.0), (2000.0, -500.0), (2000.0, 0.0), (-2000.0, 0.0)])
    a = np.array([(-LIMIT_T, 0.0), (0.0, 0.0), (0.0, LOT_DEPTH), (-LIMIT_T, LOT_DEPTH)])
    b = np.array([(lot_len, 0.0), (lot_len + LIMIT_T, 0.0), (lot_len + LIMIT_T, LOT_DEPTH), (lot_len, LOT_DEPTH)])
    return [wall, a, b]


def _seg_dist(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-9), 0.0, 1.0)
    return float(np.hypot(*(a + t * ab - p)))


def _overlap(A, B) -> bool:
    for P in (A, B):
        for i in range(len(P)):
            e = P[(i + 1) % len(P)] - P[i]
            n = np.array([-e[1], e[0]])
            pa, pb = A @ n, B @ n
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def poly_dist(A, B) -> float:
    if _overlap(A, B):
        return 0.0
    d = min(_seg_dist(p, B[i], B[(i + 1) % len(B)]) for p in A for i in range(len(B)))
    return min(d, min(_seg_dist(p, A[i], A[(i + 1) % len(A)]) for p in B for i in range(len(A))))


def clearance(car: Car, pose, obs) -> float:
    C = car.corners(*pose)
    return min(poly_dist(C, O) for O in obs)


def step(pose, direction: int, steer_sign: int, ds: float, car: Car):
    """Advance the rear axle by ds mm (>0) in `direction` with the wheels at +-steer_max (0 = straight)."""
    x, y, th = pose
    d = direction * ds
    if steer_sign == 0:
        return x + d * math.cos(th), y + d * math.sin(th), th
    R = car.r_min * steer_sign                    # + = centre to the LEFT
    th2 = th + d / R
    return x + R * (math.sin(th2) - math.sin(th)), y - R * (math.cos(th2) - math.cos(th)), th2


def parked_pose(car: Car, lot_len: float = LOT_LEN):
    """Rear-axle pose of the car centred in the lot, parallel to the wall."""
    return (lot_len / 2 - (car.front - car.rear) / 2, LOT_DEPTH / 2, 0.0)


def lot_check(car: Car, pose_lot, lot_len: float = LOT_LEN, tol_mm: float = 15.0) -> dict:
    """Is the car's footprint at `pose_lot` (rear axle, lot frame) inside the lot interior [0, lot_len] x [0,
    LOT_DEPTH], each corner within tol_mm?  -> {ok, corners_in, worst_mm (how far the worst corner is outside, 0 =
    inside), pose}.  wro_next reports "parked" only when ok (review 2026-09-25: "parked" with 0 / 4 corners in)."""
    C = car.corners(*pose_lot)
    out = [max(-px, px - lot_len, -py, py - LOT_DEPTH, 0.0) for px, py in C]
    n_in = sum(1 for o in out if o <= tol_mm)
    return dict(ok=n_in == 4, corners_in=n_in, worst_mm=round(float(max(out)), 1),
                pose=[round(float(pose_lot[0]), 1), round(float(pose_lot[1]), 1),
                      round(math.degrees(float(pose_lot[2])), 1)])


def run_leg(pose, direction, steer_sign, car, obs, margin, max_mm=600.0, res=1.0):
    """Drive until the next step would come closer than `margin` to anything; returns (pose, mm driven, min clr)."""
    done, clr = 0.0, clearance(car, pose, obs)
    while done < max_mm:
        nxt = step(pose, direction, steer_sign, res, car)
        c = clearance(car, nxt, obs)
        if c < margin:
            break
        pose, done, clr = nxt, done + res, c
    return pose, done, clr


def plan_exit(car: Car | None = None, lot_len: float = LOT_LEN, margin: float = 12.0, free_mm: float = 350.0,
              max_legs: int = 12):
    """Legs from the parked pose until a forward full-lock leg runs `free_mm` without touching.  Each leg:
    dict(dir, steer, dist_mm, dth_rad, start, end, clearance)."""
    car = car or Car()
    obs = obstacles(lot_len)
    pose = parked_pose(car, lot_len)
    legs = []
    # 1. straight back to the rear limitation
    end, dist, clr = run_leg(pose, -1, 0, car, obs, margin)
    legs.append(dict(dir=-1, steer=0, dist_mm=round(dist, 1), dth_rad=0.0, start=pose, end=end, clearance=round(clr, 1)))
    pose = end
    for _ in range(max_legs):
        end, dist, clr = run_leg(pose, +1, +1, car, obs, margin, max_mm=free_mm + 1)
        legs.append(dict(dir=1, steer=1, dist_mm=round(dist, 1), dth_rad=end[2] - pose[2], start=pose, end=end,
                         clearance=round(clr, 1)))
        pose = end
        if dist >= free_mm:
            return legs
        end, dist, clr = run_leg(pose, -1, -1, car, obs, margin)
        legs.append(dict(dir=-1, steer=-1, dist_mm=round(dist, 1), dth_rad=end[2] - pose[2], start=pose, end=end,
                         clearance=round(clr, 1)))
        pose = end
    raise RuntimeError("no exit within %d legs" % max_legs)


def plan_exit_to_lane(car: Car | None = None, lot_len: float = LOT_LEN, margin: float = 12.0, max_legs: int = 40):
    """The wiggle out of `plan_exit` (without its free leg), then an S back to heading 0: forward full-left by alpha,
    forward full-right until parallel to the wall again.  alpha is the smallest that clears everything, so the car
    ends as close to the lot as it can: pose Q = (x, y, 0).  Reversed, these legs ARE the parallel park from Q.

    `max_legs` 40, not plan_exit's 12: the stock A1's MEASURED radius (0.28 m effective wheelbase, mat 2026-09-23)
    needs 18 wiggle legs at 29 deg and 22 at the 26 deg park lock (R 574), and 12 raised "no exit" -- wro_next died
    at its start with park or lot_start on.  The WLtoys needs 4 (R <= 201, margin 22), 6 at R 267-308."""
    car = car or Car()
    obs = obstacles(lot_len)
    legs = plan_exit(car, lot_len, margin, max_legs=max_legs)[:-1]
    pose = legs[-1]["end"]
    for alpha_deg in np.arange(0.0, 80.0, 1.0):
        # leg A: forward full-left by alpha (checked every mm)
        a = math.radians(alpha_deg)
        ds_a = a * car.r_min
        p, ok, clr_a = pose, True, 1e9
        for _ in range(int(ds_a)):
            p = step(p, 1, 1, 1.0, car)
            c = clearance(car, p, obs)
            clr_a = min(clr_a, c)
            if c < margin:
                ok = False
                break
        if not ok:
            continue
        pa = p
        # leg B: forward full-right until the heading is back to 0
        th_b = pa[2]
        ds_b = th_b * car.r_min
        clr_b = 1e9
        for _ in range(int(ds_b)):
            p = step(p, 1, -1, 1.0, car)
            c = clearance(car, p, obs)
            clr_b = min(clr_b, c)
            if c < margin:
                ok = False
                break
        if not ok:
            continue
        p = (p[0], p[1], 0.0)
        legs.append(dict(dir=1, steer=1, dist_mm=round(ds_a, 1), dth_rad=a, start=pose, end=pa, clearance=round(clr_a, 1)))
        legs.append(dict(dir=1, steer=-1, dist_mm=round(ds_b, 1), dth_rad=-th_b, start=pa, end=p, clearance=round(clr_b, 1)))
        return legs
    raise RuntimeError("no S-curve clears the lot")


_PLANS: dict = {}
_PLAN_LOCK = threading.Lock()     # one plan at a time: a program asking while the agent warms the
#                                                 cache waits for that plan instead of computing it a second time


def plan_exit_to_lane_cached(car: Car, lot_len: float = LOT_LEN, margin: float = 12.0, cache_dir: str | None = None):
    """plan_exit_to_lane, remembered per (footprint, wheelbase, lock, lot, margin): in memory and, with `cache_dir`,
    in cache_dir/exit_plans.json.  The plan is pure Python per millimetre -- 1.7-2.2 s on the laptop for the WLtoys
    (x3-4.3 on the Pi: 6-9 s each, a lot start needs two) and wro_next computed it at every start, before the car
    moved: a race start sat still that long.  The first run after a params change pays it once; a cached plan loads
    in milliseconds; race_main and the agent warm the cache before a start (wro_next.warm_exit_plans)."""
    with _PLAN_LOCK:
        return _plan_cached(car, lot_len, margin, cache_dir)


def _plan_cached(car: Car, lot_len: float, margin: float, cache_dir: str | None):
    import json
    import os
    key = json.dumps([round(float(v), 3) for v in (car.rear, car.front, car.half_w, car.wheelbase, car.steer_max_deg,
                                                   lot_len, margin)] + [1])
    if key in _PLANS:
        return _PLANS[key]
    path = os.path.join(cache_dir, "exit_plans.json") if cache_dir else None
    disk = {}
    if path and os.path.exists(path):
        try:
            with open(path, encoding="utf-8") as f:
                disk = json.load(f)
        except (OSError, ValueError):
            disk = {}
    if key in disk:
        legs = [dict(lg, start=tuple(lg["start"]), end=tuple(lg["end"])) for lg in disk[key]]
    else:
        legs = plan_exit_to_lane(car, lot_len=lot_len, margin=margin)
        if path:
            try:
                disk[key] = [dict(lg, start=[float(v) for v in lg["start"]], end=[float(v) for v in lg["end"]],
                                  dth_rad=float(lg["dth_rad"])) for lg in legs]
                tmp = path + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(dict(list(disk.items())[-16:]), f)
                os.replace(tmp, path)
            except OSError:
                pass
    _PLANS[key] = legs
    return legs


def reverse(legs):
    """The same path driven backwards: every leg in reverse order with its direction flipped."""
    out = []
    for lg in reversed(legs):
        out.append(dict(dir=-lg["dir"], steer=lg["steer"], dist_mm=lg["dist_mm"], dth_rad=-lg["dth_rad"],
                        start=lg["end"], end=lg["start"], clearance=lg["clearance"]))
    return out


class Executor:
    """Drives a list of legs, one call per control step; non-blocking so the caller keeps localising.

        ex = Executor(legs, direction, p)   # p: v_leg, steer_deg, servo_s, coast_mm, coast_deg, leg_timeout, stop_s
        while not ex.done: ex.tick(robot, now, yaw_deg, ds_mm)

    A leg: wheels to the leg's lock with the car stopped (servo_s), then `v_leg` in the leg's direction until the gyro
    has turned the leg's angle (turning legs) or the odometry has run its distance (straight legs), minus what the
    car coasts after the stop command.  Clockwise rounds mirror the lot: the steering sign flips.

    Between legs the finished leg's lock is HELD for `stop_s` while the car coasts to a stop, and only then do the
    wheels swing: swung at once, the ~10 mm coast ran on the next leg's opposite lock and turned the car ~2.5 deg the
    wrong way, so every leg started ~5 deg off plan; legs end on absolute headings, so each then ran 5-30 mm too short
    or long and the wiggle walked into the limitation (sim: 5 contacts, 2/4 corners in, from an exact Q)."""

    def __init__(self, legs, direction: int, p: dict, yaw_lot0: float | None = None, pose_fn=None, pulse_ok=None):
        """yaw_lot0: the gyro yaw (deg) that corresponds to lot heading 0; given, turning legs end on the leg's
        ABSOLUTE end heading instead of its angle, so an error in the starting heading is absorbed.

        pulse_ok(sign, steer_deg, mm) -> bool (the lidar shield, BRAIN_SPEC 5.3): asked before every step-mode pulse;
        False ends the leg where it stands (log `blocked`) instead of kicking the car into what the lidar sees.

        STEP mode (p["step"] and a pose_fn): for a car whose speed is open loop (the WLtoys build) a leg is driven
        continuously only until `step_near_mm` + the braked stop remain; then STOP - MEASURE - PULSE - repeat: the car
        settles, `step_scans` fresh lidar poses arrive, the remaining distance to the leg's planned END is measured, and
        robot.pulse() gives one measured kick toward it (backwards after an overshoot), until it is within
        `step_done_mm`.  Straight legs measure on the lidar pose in the lot frame -- pose_fn() -> (time of the newest
        lidar-corrected pose, (x, y, th) in the lot frame) -- turning legs on the gyro's absolute heading (the arc left
        = the angle left x r_min).  research/dynamics_wl.md 6.3."""
        self.legs, self.dir, self.p = list(legs), direction, p
        self.yaw_lot0 = yaw_lot0
        self.pose_fn = pose_fn
        self.pulse_ok = pulse_ok
        self.blocked = 0                                     # pulses the shield refused (legs ended early)
        self.step = bool(p.get("step")) and pose_fn is not None
        self.i, self.phase, self.t0 = 0, "steer", None
        self.yaw0, self.mm = 0.0, 0.0
        self.pulses, self.t_still, self.t_leg = 0, 0.0, 0.0
        self.done = not self.legs
        self.log = []

    # ---------------------------------------------------------------------------------------------- step mode
    def remaining(self, lg, yaw_deg: float):
        """mm still to go along leg `lg` (negative = overshot), or None when it cannot be measured now."""
        if lg["steer"]:
            if self.yaw_lot0 is not None:
                want = self.yaw_lot0 + self.dir * math.degrees(lg["end"][2])
                togo = ((want - yaw_deg + 180.0) % 360.0) - 180.0
                sense = 1.0 if self.dir * lg["dth_rad"] > 0 else -1.0
                return math.radians(togo * sense) * self.p["r_min"]
            turned = abs(((yaw_deg - self.yaw0 + 180.0) % 360.0) - 180.0)
            return math.radians(abs(math.degrees(lg["dth_rad"])) - turned) * self.p["r_min"]
        got = self.pose_fn()
        if got is None or got[1] is None:
            return None
        x, y, _th = got[1]
        ex, ey, eth = lg["end"]
        return ((ex - x) * math.cos(eth) + (ey - y) * math.sin(eth)) * lg["dir"]

    def _finish_leg(self, robot, steer, **extra):
        robot.drive(0.0, steer)
        self.log.append(dict(leg=self.i, want_mm=self.legs[self.i]["dist_mm"], **extra))
        self.i += 1
        self.phase, self.t0 = "stop", None
        if self.i >= len(self.legs):
            self.done = True

    def _tick_step(self, robot, now: float, yaw_deg: float, lg, steer: float, ds_mm: float = 0.0):
        p = self.p
        if self.phase == "drive":
            self.mm += abs(ds_mm)
            rem = self.remaining(lg, yaw_deg)
            if rem is None:                              # no pose (never expected in the lot): the odometry
                rem = lg["dist_mm"] - self.mm
            v = p["v_leg"]
            brake = v * v / (2.0 * max(0.3, p.get("stop_decel", 2.2))) * 1000.0
            if rem is not None and rem <= p.get("step_near_mm", 40.0) + brake or now - self.t0 > p["leg_timeout"]:
                robot.drive(0.0, steer)
                self.phase, self.t_still = "settle", now
                return
            robot.drive(lg["dir"] * v, steer)
            if getattr(robot, "last_act", None) == "brake":
                # the lidar shield stopped the continuous part short (bluewave/shield.py): the measured pulses take
                # over from here, each asked of pulse_ok -- instead of pushing the braked command until leg_timeout
                self.phase, self.t_still = "settle", now
            return
        if self.phase == "pulse":
            if not robot.pulse_busy():
                self.phase, self.t_still = "settle", now
            return
        # settle: still, then fresh lidar poses, then measure
        robot.drive(0.0, steer)
        if now - self.t_still < p.get("step_settle_s", 0.25):
            return
        rem = self.remaining(lg, yaw_deg)
        if not lg["steer"]:
            got = self.pose_fn()
            fresh = got is not None and got[0] >= self.t_still + p.get("step_settle_s", 0.25) \
                + 0.1 * (p.get("step_scans", 2) - 1)
            if not fresh and now - self.t_still < 1.5:
                return                                   # wait for the lidar poses taken after the car stood still
        done_mm = p.get("step_done_mm", 6.0)
        if rem is None or abs(rem) <= done_mm or self.pulses >= p.get("step_max", 10) \
                or now - self.t_leg > p.get("step_timeout", 8.0):
            self._finish_leg(robot, steer, mode="step", pulses=self.pulses,
                             rem_mm=None if rem is None else round(rem, 1),
                             timeout=bool(rem is not None and abs(rem) > done_mm))
            return
        sign = lg["dir"] if rem > 0 else -lg["dir"]
        if self.pulse_ok is not None and not self.pulse_ok(sign, steer, abs(rem)):
            self.blocked += 1
            self._finish_leg(robot, steer, mode="step", pulses=self.pulses, rem_mm=round(rem, 1), timeout=False,
                             blocked=True)
            return
        if robot.pulse(sign, steer, abs(rem)) <= 0.0:
            return                                       # refused (E-STOP): stay here
        self.pulses += 1
        self.phase = "pulse"

    # ---------------------------------------------------------------------------------------------- tick
    def tick(self, robot, now: float, yaw_deg: float, ds_mm: float):
        if self.done:
            return
        lg = self.legs[self.i]
        steer = lg["steer"] * self.dir * self.p["steer_deg"]
        if self.phase == "stop":
            if self.t0 is None:
                self.t0 = now
            robot.drive(0.0, self.legs[self.i - 1]["steer"] * self.dir * self.p["steer_deg"])
            if now - self.t0 >= self.p.get("stop_s", 0.3):
                self.phase, self.t0 = "steer", None
            return
        if self.phase == "steer":
            if self.t0 is None:
                self.t0 = now
            robot.drive(0.0, steer)
            if now - self.t0 >= self.p["servo_s"]:
                self.phase, self.t0, self.yaw0, self.mm = "drive", now, yaw_deg, 0.0
                self.pulses, self.t_leg = 0, now
            return
        if self.step:
            self._tick_step(robot, now, yaw_deg, lg, steer, ds_mm)
            return
        self.mm += abs(ds_mm)
        turned = abs(((yaw_deg - self.yaw0 + 180.0) % 360.0) - 180.0)
        if lg["steer"] and self.yaw_lot0 is not None:
            # yaw grows CCW; in the mirrored (clockwise) lot the lot heading runs the other way
            want = self.yaw_lot0 + self.dir * math.degrees(lg["end"][2])
            togo = ((want - yaw_deg + 180.0) % 360.0) - 180.0
            sense = 1.0 if self.dir * lg["dth_rad"] > 0 else -1.0
            finished = togo * sense <= self.p["coast_deg"]
        elif lg["steer"]:
            finished = turned >= abs(math.degrees(lg["dth_rad"])) - self.p["coast_deg"]
        else:
            finished = self.mm >= lg["dist_mm"] - self.p["coast_mm"]
        if finished or now - self.t0 > self.p["leg_timeout"]:
            robot.drive(0.0, steer)
            self.log.append(dict(leg=self.i, mm=round(self.mm), turned=round(turned, 1),
                                 want_deg=round(abs(math.degrees(lg["dth_rad"])), 1), want_mm=lg["dist_mm"],
                                 timeout=not finished))
            self.i += 1
            self.phase, self.t0 = "stop", None
            if self.i >= len(self.legs):
                self.done = True
            return
        robot.drive(lg["dir"] * self.p["v_leg"], steer)


def to_field(pose_lot, lot_x0: float, direction: int):
    """Lot frame -> field frame for a lot whose rear limitation's inner face (in travel direction) is at field x
    `lot_x0` in the start straight S."""
    x, y, th = pose_lot
    if direction > 0:
        return lot_x0 + x, -1500.0 + y, th
    return lot_x0 - x, -1500.0 + y, math.pi - th


def to_lot(pose_field, lot_x0: float, direction: int):
    """Field frame -> lot frame (the inverse of to_field)."""
    x, y, th = pose_field
    if direction > 0:
        return x - lot_x0, y + 1500.0, th
    return lot_x0 - x, y + 1500.0, (math.pi - th + math.pi) % (2 * math.pi) - math.pi


if __name__ == "__main__":
    car = Car()
    legs = plan_exit(car)
    print("r_min %.0f mm, %d legs" % (car.r_min, len(legs)))
    for lg in legs:
        print("  %s steer %+d  %6.1f mm  %+6.1f deg  clearance %5.1f  -> (%.0f, %.0f, %.1f deg)" % (
            "fwd" if lg["dir"] > 0 else "rev", lg["steer"], lg["dist_mm"], math.degrees(lg["dth_rad"]), lg["clearance"],
            lg["end"][0], lg["end"][1], math.degrees(lg["end"][2])))
