"""OBS_V17 = obs_v16 + ONE change, bat_hold: the drive behaves as at 7.0 V whatever the pack.  The drive's bat_comp
(duty x v_ref / V_bat) over-corrects this car (mat plant fit vbat_exp 0): +31 % at 6.1 V (mat 20261001-204736: the
park legs and the Q creep overshot 30-150 mm, park_off; SIM true speed +38 %), -13 % at 8.1 V.  For the run v_ref
follows the pack (v_ref0 x V_bat / bat_hold_v), restored at the end.

OBS_V16 = obs_v13 (mat 20261001-203331: 57.6 s, 18/18, full park) + ONE change, q_brake: the stop at Q fires at
the real braking distance (v^2 / 2a + v x latency on the lidar speed, a = 2.4 m/s^2 fitted to that run: it came off
the last corner at 0.8-0.9 m/s coasting and overshot Q by 159 mm with the old 50 mm lead -> a 4.5 s realign).
A stop a little SHORT is crept forward.  obs_v14's quicker legs and v15's short back are out (both lost the park
in SIM).

OBS_V13 = obs_v12 (mat 20261001-202156: 68.2 s, full park, 7 legs) + the two slow parts gone:
  exit_smooth  the S's last leg is not driven: the car leaves the lot already turned (~62 deg) toward the corner and
               the laps take over -- no straightening to Q, no exit_back reverse (it was 360 mm east then 420 back).
  q_line       the final approach rides Q's line straight (the lap path bends into the next corner before Q: every
               park reached Q 40-50 deg off, ~10 s of realigns).

OBS_V12 = obs_v11 (mat 20261001-201055: 73.3 s, 18/18 signs, full park) + ONE change, park_asym: the lot exit
and the park are planned AND driven on the real lock per side -- left 30 deg true (R 237 at the slow wheelbase),
right 22 (R 339) -- instead of 22 both ways.  Same lot model (337.4), same margin 28, same Q (483 / 369 vs 484 / 363):
ccw 7 legs each way instead of 9 (cw stays 9).  obs_v6's planner (plan_exit6), not its leg driver (v6 failed on
the drive): the legs run on obs_v11's run_legs, each with its side's lock and radius.

OBS_V11 = obs_v7 + ONE change, q_creep_after: after each realign toward Q the same try creeps x onto Q at
v_leg_slow.  Today every realign ended 35-76 mm short of Q (the forward line drive stops 53 mm early at 0.2 m/s and
the braked car stays there) and the creep only ran when the line and heading were already in tolerance, which never
happened: the park started 40-70 mm short and 4 of 8 parks were full.  Laps / exit / park legs are obs_v7's (mat
20261001-195904: 3 laps, 18/18 signs).

OBS_V7 = obs_v5 (mat 72.6 s, full park, 18/18 signs right; frozen) + ONE mechanism fixed, the MAP READ: a sign the
lidar map holds (hits >= 2) whose colour is unread is (1) GLANCED at -- just past the camera's +-29 deg edge, the nose
turns toward it (glance) -- and (2) LOOKED at where the map puts it: its 50 mm face projected into the frame, the strip
counted pixel by pixel with looser colour limits (map_look).  Mat 2026-10-01 17:46 (obs_v5, run 20261001-174628.799)
and SIM seed 1 alike: the green after corner 2 (seat 13, key 2.0) was a lidar seat in all 3 laps and never got one
camera vote -- on the straight before corner 2 the car aims right of it (the red 1.2 is passed on its right) and it
stayed 30-60 deg off the camera's axis, then came into view ~250 mm away, beside the car; with no colour the car took
the default lane and passed it on the wrong side 3 times.  The good runs only got a lucky glimpse at the edge.  Every
look is saved (runs/rec/<stamp>-look/, capped) so a sign that still fails can be SEEN.  The rest is obs_v5's.

OBS_V1 -- the Obstacle Challenge on the WLtoys car (lidar at 50 mm + camera): lot start, 3 laps past the signs, the
parallel park.  Built on the mat-proven Open core (programs/open_fast_v2.py, never edited): the wheels' straight-ahead
at steer_trim (the _Trimmed wrapper), the effective wheelbase at speed (wb_eff: R x tan(true angle) = 137 mm at 0.2 m/s,
~210-230 at 0.6-1.0 m/s), the 22 deg true lock both ways (steer_cap), the lidar shield on every command with the
braked-stop decel shield_decel, and its board watchdogs (steer_dead / steer_wrong / board_hung -> robot.board_reopen()).
What is new, each with its mechanism:

  POSE      the lidar sees the 100 mm walls below 2.2 m: every scan's wall points (lidar_perc: pillars, limitations and
            clutter out) are fitted to the field's walls (the outer square +-1500, the island +-500, and the lot's two
            limitations once the lot is known) by a damped point-to-line ICP from the odometry prior (the gyro yaw +
            the drive's lidar-corrected speed).  A direction the walls do not constrain (no wall across the corridor in
            range) keeps the odometry.  The field frame is the rules': S = the start straight (the lot's), x right.
  START     in the lot: the outer wall ~45 mm beside the body says the direction (on the right = counter-clockwise),
            the wall and the island face give y and the heading.  x is not observable from inside the lot (the front
            limitation hides the wall ahead): a nominal x, the lot tied to it, the ICP on the lot's faces and the
            outer wall only; once out, a 1-D search on the wall ahead moves the car AND the lot (x_fix), and the
            limitations seen on the last lap's approach refine the lot (lot_refine).
  EXIT      park.py's wiggle + an S-curve to a Q >= q_lane_mm (360) off the wall (plan_exit_q: at the plan's own
            minimum, 303, the body passed the limitations 47 mm off and the shield held it), on the SLOW wheelbase
            (137 mm) at the true lock 22 deg (R 339), margin 28; frozen in PLAN_SEED for wltoys_bw2 (8-10 s of Python
            on the Pi).  Driven by run_legs on MEASURED quantities: a turning leg ends on the gyro's absolute heading,
            a straight one on the lidar pose in the lot frame; a fast part, a stop leg_slow_mm short, then creeps; a
            leg closing on a limitation nearer than leg_clear_stop ends there.  park.Executor's step pulses are not
            used: the fitted plant's drive lag (tau 0.56 s) moves a 30-60 ms kick ~1 mm.  After the exit, when the
            first corner's turn-in lies behind Q or the next straight's first sign is seen but not read, the car backs
            along S (exit_back, the nose exit_look_deg toward the island: the camera's +-29 deg then sees that sign).
  SIGNS     lidar_perc's pillar clusters snapped to the 24 rule seats (field.SEATS, seat_snap_mm); their colour from
            the camera (vision.pillars) paired BY BEARING (lidar_perc.associate: pitch-free) at the exposure's pose
            (camera.latency_s).  Red -> pass on its right, green -> on its left [RULE].  Seats persist over the laps.
  PATH      wro_next's Planner (copied here, the car's own half width): a lane per sign on the side it demands with
            sign_clear to the body, neighbours pulled within max_step, fillets between straights whose radius is
            checked against the known signs, the island and the lot; followed by pure pursuit from the rear axle with
            the SPEED'S wheelbase, and a collision guard (swept footprint on candidate arcs) under it.  Two signs 500
            mm apart asking opposite sides (a 330-530 mm shift) are tried; relax_holds shield holds beside such a pair
            relax it from the next straight on (the second sign then passed on the first one's lane: the wrong side,
            280+ mm clear) -- this car's R 339 cannot always make them.
  SPEED     a profile along the path: v_top (lap 1: v1, the signs unseen; laps 2-3: v, the map known), sqrt(ay_plan R)
            in curves, the true lock's reach at the speed's wheelbase, braking at a_brake; v_unknown near a sign whose
            colour is not read yet.
  PARK      after 3 laps (field progress) the start straight's lane moves to Q's line (the exit plan's end; a sign
            within q_zone before Q is passed on that line), the car stops at Q on the lidar pose (q_align: off the line
            -> back along it q_back_mm and in again), and drives the exit plan REVERSED with run_legs.  lot_check on
            the final pose; reason "parked" only when every corner is in.
  U-TURN    NOT implemented: the repo's rules notes (3d/bluewave_body/research/rules_oldchassis.md, field.py) do not
            carry a third-round direction change and wro_next has none; the 2024-season competitor notes do.  Confirm
            the 2026 rule book before the event.

The mat is the judge; SIM numbers are SIM.  Presets: obs1 (DEFAULTS) / safe (slower) / fast (SIM only).  DEFAULTS <
PRESET < the robot's prog.obs_v1 < the run's own params.  Returns {reason, laps, seconds, direction, parked}.
"""
import json
import math
import os
import time
from collections import deque

import numpy as np

from bluewave import field as FLD
from bluewave import lidar_perc as LP
from bluewave import park as K
from bluewave import shield as SH
from bluewave import vision as V

PRESETS = {
    # obs1: lap 1 at v1 (the camera reads every sign), laps 2-3 on the map at v
    "obs1": dict(v1=0.50, v=0.80, ay_plan=1.1, a_brake=1.0, r_corner=560.0),
    # safe: the first mat run of the ladder
    "safe": dict(v1=0.40, v=0.55, ay_plan=0.9, a_brake=0.8, r_corner=480.0),
    # fast: SIM only until obs1 is clean on the mat
    "fast": dict(v1=0.60, v=1.00, ay_plan=1.4, a_brake=1.2, r_corner=600.0),
}

DEFAULTS = dict(
    preset="obs1",
    laps=3, seconds=175.0, dir=0,             # dir: 0 = from the lot's wall side, 1 = ccw, -1 = cw
    lot_start=1, park=1,                      # start parked in the lot / park after the laps
    v1=0.50, v=0.80, v_min=0.20, v_unknown=0.30, v_approach=0.22, v_creep=0.20,   # v2: 0.14 never moved the mat car
    ay_plan=1.1, a_brake=1.0, lag_s=0.25,     # m/s^2 at the CG in curves; the planned braking; command-to-motion lag
    look_min=260.0, look_t=0.45,              # pure pursuit lookahead: max(look_min, v x look_t)
    wb_v=None, wb_mm=None,                    # the effective wheelbase (R x tan(true angle)) against |v|: None = the mat
    #                                           fit, profiles/plant_mat1001.json plant.wb_curve, its slow end the profile's
    #                                           chassis.wheelbase_m; no fit file: wb_slow at 0.2 m/s, wb_eff from 0.8
    wb_eff=210.0,                             # mm at 0.8-1.0 m/s (mat 2026-09-30, open_fast_v2's wb_eff): the fallback
    steer_cap=22.0,                           # (v3: unused -- see steer_cap_left / steer_cap_right)
    steer_cap_left=30.0, steer_cap_right=22.0,  # v3: deg TRUE per side: the servo's +-26 with the -4 trim reach 30 LEFT and
    #                                           22 right (mat 2026-10-01: every obstacle stop asked 34-44 deg at a corner
    #                                           while the program capped 22 -- R 339 instead of the measured 237)
                              # deg TRUE: the linkage's lock both ways (mat 2026-09-30)
    reach_margin=6.0,                         # v5: deg of lock the speed profile keeps in reserve (1.5 let lap 2 enter
    #                                           corner 2 at 0.69 m/s and pass the green after it on the wrong side;
    #                                           6 -> every sign right in 2 mat runs, laps ~1 s slower)
    steer_trim=-4.0,                          # deg added to every servo command: the wheels' straight-ahead (mat)
    wall_band_mm=270.0,                       # v2: a "sign" nearer than this to an OUTER wall is the parking lot's
    #                                           limitation (it stands 200 mm out; signs sit 400 / 600 mm out): ignored
    #                                           (mat 2026-10-01: the lot read as two red signs -> the route went into it)
    stand_max_deg=12.0,
    # the Planner (wro_next's, the car's own half width)
    slalom_tol=60.0, relax_mm=900.0, relax_holds=2,          # a sign pair needing more than max_step + this; a hold this near relaxes it
    lane_mid=500.0, sign_clear=85.0, wall_clear=45.0, max_step=170.0, hold_front=215.0, hold_back=170.0, ramp=330.0,
    r_corner=560.0, r_cands=[480.0, 430.0, 400.0, 340.0, 300.0, 640.0, 720.0], corner_ok=90.0, seat_votes=2, lot_band=200.0,
    # seats
    seat_snap=120.0, cam_snap=200.0, bearing_tol=4.0, range_gate=0.35, pillar_max_mm=1900.0, cam_hz=12.0,
    unknown_ahead=1100.0,
    # v7: the map look (map_look) -- an unread lidar sign looked at where the map puts it
    look=1,                                   # 0 = obs_v5's camera exactly
    look_min_mm=280.0, look_max_mm=1750.0,    # the lens-to-sign range it runs in (the reads came at 0.5-1.9 m)
    look_z_mm=[20.0, 80.0],                   # the strip: this band of the 100 mm sign's face (clear of mat and top)
    look_w=0.6,                               # the counted window: this share of the sign's width in pixels
    look_slack_px=10.0,                       # +- search around the projection (pose / latency): >= 0.8 sign widths
    look_s=70.0, look_v=26.0,                 # looser than vision's s_col 90 / v_col 40 (a shadowed, half-glared sign)
    look_frac=0.30,                           # one colour must fill this share of the window ...
    look_dom=3.0,                             # ... and outnumber the other this many times (two frames: seat_votes)
    look_save=60, look_save_gap_s=0.2,        # frames kept for the review (runs/rec/<stamp>-look/), one per gap
    # v7: the glance (glance) -- an unread map sign just past the camera's edge turns the nose to it
    glance=1,                                 # 0 = obs_v5's path exactly
    glance_fov_deg=27.0,                      # the frame's usable edge (HFOV +-29.3; map_look's box needs ~2 deg)
    glance_in=6.0,                            # turn until the sign sits this far inside the edge (21 deg)
    glance_deg=16.0,                          # signs up to edge + this are glanced at; the largest nose bias
    glance_min_mm=600.0, glance_max_mm=1750.0,  # lens range: nearer is too late to change lane, farther reads poorly
    glance_max_s=1.5,                         # per sign
    glance_block_x=900.0, glance_block_y=300.0,  # no glance across a sign this near ahead / this near the axis
    # the guard on pure pursuit
    guard=1, guard_mm=30.0, guard_h0=250.0, guard_ht=0.5,
    # the pose
    icp_gate=70.0, icp_lam=6.0, icp_th_gain=0.25, x_search=850.0,
    # the lot: exit / park (park.py)
    v_leg_exit=0.16,                          # v4: the lot EXIT's legs (the park keeps v_leg 0.11: it parked perfect)
    park_lock_deg=22.0, park_margin=28.0, exit_margin=28.0, v_leg=0.11, v_leg_slow=0.06, leg_slow_mm=22.0,
    servo_dps=85.0, servo_extra_s=0.05,       # the servo's slew (mat fit 85.6 deg/s): lock to lock 44 deg = 0.52 s
    leg_coast_s=0.05, leg_coast_line_s=0.25, leg_tol_mm=6.0, leg_timeout=5.0, leg_stop_s=0.10, leg_settle_s=0.15,
    q_brake=1, q_brake_decel=2.4, q_brake_lat=0.08,   # v16: the stop at Q on the braking distance -- 2.4 from the mat:
    #                                           203331 overshot 159 mm with a 50 mm lead at ~0.85 m/s -> ~210 needed
    q_near_y=0.0, q_near_deg=0.0, q_back_short=380.0,   # v16: off (v15's 150 mm back could not fix 13-15 mm aside: SIM)
    leg_tries=4, leg_clear_stop=12.0,
    q_lane_mm=360.0,                          # Q (the park's start, the exit's end) at least this far off the wall
    q_zone=900.0, exit_back_max=420.0, exit_look_deg=14.0, exit_look_s=0.8,
    q_tol_mm=8.0, q_tol_y=12.0, q_tol_deg=2.5, q_tries=3, q_back_mm=380.0, q_look=200.0, lot_refine=1,
    q_creep_after=1,                          # v11: creep x onto Q after every realign (0 = obs_v7)
    bat_hold=1, bat_hold_v=7.0,               # v17: drive as at this pack voltage whatever the pack (0 = bat_comp as is)
    exit_smooth=1,                            # v13: leave the lot turned (no S straightening, no exit_back / look)
    q_line=1, q_line_deg=35.0,                # v13: the final approach rides Q's line once within this of its heading
    q_decel=0.3,                              # v13: m/s^2 of the approach's braking profile to Q (obs_v12: 0.6)
    park_asym=1,                              # v12: exit / park legs planned AND driven on the real lock per side
    park_lock_left=30.0, park_lock_right=22.0,  # v12: deg TRUE (mat: servo +-26 with trim -4); R = wb_slow / tan
    #                                           ccw 7 legs (obs_v11: 9) at the same margin 28 and Q; cw stays 9
    park_prep=2.2,                            # quarter laps before the end: S's lane moves to Q's line
    # shield / safety (open_fast_v2's)
    shield=1, shield_h=1600.0, shield_decel=1.0, shield_park_margin=0.0, shield_park_stop=0.0,
    hold_s=1.0, holds_max=6, back_mm=150.0,
    steer_fault_s=0.8, steer_fault_dps=25.0, dead_imu_s=0.35, reopen_gap_s=2.5,
    follow_dead_mm=150.0, follow_dead_deg=8.0, wrong_dps=25.0, wrong_s=0.35,
    loop_hz=40.0, log_every=0.5,
)


class LookSaver:
    """v7: the frames map_look judged, for the review: <BW_RUNS or the tree's runs>/rec/<stamp>-look/ (f%05d.jpg +
    look.jsonl, one row per frame: the seat, the box counted, the counts, the verdict, the pose).  At most `cap`
    frames, one per gap_s; the JPEG writes run on a thread, never in the control loop."""

    def __init__(self, cap: int, gap_s: float):
        self.cap, self.gap, self.n, self.last = int(cap), float(gap_s), 0, -1e9
        self.q = self.th = None

    def put(self, bgr, meta):
        if self.n >= self.cap or meta["t"] - self.last < self.gap:
            return
        if self.q is None:
            import queue
            import threading
            root = os.path.join(os.environ.get("BW_RUNS") or os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "runs"), "rec")
            self.dir = os.path.join(root, time.strftime("%Y%m%d-%H%M%S") + "-look")
            os.makedirs(self.dir, exist_ok=True)
            self.q = queue.Queue(maxsize=8)
            self.th = threading.Thread(target=self._loop, daemon=True)
            self.th.start()
        try:
            self.q.put_nowait((self.n, bgr.copy(), meta))
        except Exception:
            return
        self.n += 1
        self.last = meta["t"]

    def _loop(self):
        import cv2
        with open(os.path.join(self.dir, "look.jsonl"), "a", newline="\n") as f:
            while True:
                it = self.q.get()
                if it is None:
                    break
                n, bgr, meta = it
                cv2.imwrite(os.path.join(self.dir, "f%05d.jpg" % n), bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
                f.write(json.dumps(dict(meta, frame=n)) + "\n")
                f.flush()

    def close(self):
        if self.q is not None:
            self.q.put(None)
            self.th.join(timeout=3.0)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrapd(a):
    return (a + 180.0) % 360.0 - 180.0


def _r(x):
    return None if x is None or x != x else round(x)


# ------------------------------------------------------------------------------------------ guard (from wro_next)
def pose_clearance(poses, pts, rear, front, half):
    """Per pose (N x 3: rear-axle x, y, heading): the smallest distance from the car's rectangle to any of the points,
    to the outer walls and to the island (negative = inside a wall)."""
    c, s_ = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    out = np.full(len(poses), 1e9)
    if len(pts):
        P_ = np.asarray(pts, float)
        dx = P_[None, :, 0] - poses[:, None, 0]
        dy = P_[None, :, 1] - poses[:, None, 1]
        a = c * dx + s_ * dy
        b = -s_ * dx + c * dy
        ex = np.maximum(np.maximum(-rear - a, a - front), 0.0)
        ey = np.maximum(np.abs(b) - half, 0.0)
        out = np.hypot(ex, ey).min(axis=1)
    ca = np.array([-rear, front, front, -rear])[None, :]
    cb = np.array([-half, -half, half, half])[None, :]
    cx = poses[:, 0:1] + c * ca - s_ * cb
    cy = poses[:, 1:2] + s_ * ca + c * cb
    m = np.maximum(np.abs(cx), np.abs(cy))
    return np.minimum(out, np.minimum((FLD.HALF - m).min(axis=1), (m - FLD.ISL).min(axis=1)))


def guard_steer(x, y, th, steer, pts, car, L_wb, mx, keep, horizon):
    """wro_next's collision guard: the footprint swept along the arc of each candidate steering for `horizon` mm
    against the known pillars, the lot's limitations, the island and the outer walls; the commanded steering stands if
    it keeps `keep` mm, else the nearest one that does, else the one that keeps the most."""
    cands = np.concatenate([[steer], np.arange(-mx, mx + 0.01, 2.0)])
    s = np.arange(25.0, horizon + 1.0, 25.0)
    k = np.tan(np.radians(cands))[:, None] / L_wb
    ks = k * s[None, :]
    straight = np.abs(k) < 1e-7
    kk = np.where(straight, 1.0, k)
    fx = np.where(straight, s[None, :], np.sin(ks) / kk)
    fy = np.where(straight, 0.0, (1.0 - np.cos(ks)) / kk)
    c, sn = math.cos(th), math.sin(th)
    poses = np.stack([x + c * fx - sn * fy, y + sn * fx + c * fy, th + ks], axis=-1).reshape(-1, 3)
    clr = pose_clearance(poses, pts, car.rear, car.front, car.half_w).reshape(len(cands), len(s)).min(axis=1)
    if clr[0] >= keep:
        return steer, float(clr[0]), float(clr[0])
    ok = np.where(clr[1:] >= keep)[0] + 1
    i = int(ok[np.argmin(np.abs(cands[ok] - steer))]) if len(ok) else int(np.argmax(clr))
    return float(cands[i]), float(clr[0]), float(clr[i])


# ------------------------------------------------------------------------------------------ plan (from wro_next)
class Planner:
    """wro_next.Planner: seat knowledge -> a closed path (N x 2, mm) every `step` mm in TRAVEL order from S's centre.
    Changes here: the car's own half width (car_half) in the lanes' clearances (wro_next: the stock 80 mm)."""

    def __init__(self, direction: int, p: dict, step: float = 20.0):
        self.dir, self.p, self.step = direction, p, step
        self.s_mid = None
        self.q_along = None                  # the park: Q's along in S (travel direction); signs beyond are not passed
        self.extra_pts = []
        self.seat = {}
        self.path = None
        self.version = 0
        self.rebuild()

    def colour(self, key):
        v = self.seat.get(key)
        if not v:
            return None
        r, g = v.get("red", 0), v.get("green", 0)
        if max(r, g) < self.p["seat_votes"]:
            return None
        return "red" if r > g else "green"

    def lane_d(self, k: int, along: float) -> float:
        p, d = self.p, self.p["lane_mid"]
        if k == 0 and self.s_mid is not None:
            d = self.s_mid
        lanes = self.lanes(k)
        if not lanes:
            return d
        xs, ds = [lanes[0][0] - p["hold_front"] - p["ramp"]], [d]
        for a, dd in lanes:
            xs += [a - p["hold_front"], a + p["hold_back"]]
            ds += [dd, dd]
        xs.append(lanes[-1][0] + p["hold_back"] + p["ramp"])
        ds.append(d)
        return float(np.interp(along, xs, ds))

    def lanes(self, k: int):
        p = self.p
        half = float(p["car_half"])
        out = []
        for ai in range(len(FLD.SEAT_ALONG)):
            col = self.colour((k, ai))
            if col is None:
                continue
            a = FLD.SEAT_ALONG[ai] * self.dir
            if k == 0 and self.q_along is not None and a > self.q_along + p["car_front"] + 60.0:
                continue
            toward_outer = (col == "red") == (self.dir > 0)
            d_p = self.sign_d(k, ai)
            wall_lo = (p["lot_band"] if (k == 0 and p["park"]) else 0.0) + half + p["wall_clear"]
            wall_hi = 1000.0 - half - p["wall_clear"]
            gap = 25.0 + half + p["sign_clear"]
            lo, hi = (wall_lo, d_p - gap) if toward_outer else (d_p + gap, wall_hi)
            if lo > hi:
                lo = hi = (lo + hi) / 2.0
            mid = (lo + hi) / 2.0
            if k == 0 and self.s_mid is not None:
                mid = min(max(self.s_mid, lo), hi)       # the park line where the sign allows it
                if self.q_along is not None and a > self.q_along - p["q_zone"] and not lo <= self.s_mid <= hi:
                    # a sign beside the lot / Q that asks the island side: the park needs Q's line there (the
                    # approach from 434 mm off never settled: SIM seed 5 ccw park_off); the line clears the sign
                    # (inner row, 600 mm) by 150+ mm -- one wrong-side pass instead of a failed park
                    mid = lo = hi = self.s_mid
                    self.violated = getattr(self, "violated", set()) | {(k, ai)}
            out.append([a, mid, lo, hi, d_p, gap, ai])
        out.sort(key=lambda r: r[0])
        for _ in range(6):
            for i in range(len(out) - 1):
                a0, l0, lo0, hi0 = out[i][:4]
                a1, l1, lo1, hi1 = out[i + 1][:4]
                step = p["max_step"] * (a1 - a0) / 500.0
                if abs(l1 - l0) > step:
                    mid = (l0 + l1) / 2.0
                    out[i][1] = min(max(mid - np.sign(l1 - l0) * step / 2.0, lo0), hi0)
                    out[i + 1][1] = min(max(mid + np.sign(l1 - l0) * step / 2.0, lo1), hi1)
        # two signs 500 mm apart demanding opposite sides ask a 330-530 mm shift in ~230 mm of travel.  This car (R 339
        # at the true lock) made some of them (SIM seeds 1 ccw, 2 cw) and not others: shield holds, back-offs, then
        # 'blocked' (SIM seed 3 ccw).  So every such pair is TRIED; a shield hold near it puts it in `relax` (the run
        # loop), and from then on the first sign keeps its side and the second is passed on the first one's lane --
        # the wrong side, 280+ mm clear of it (one wrong-side pass a lap instead of holds and a blocked run)
        self.violated = getattr(self, "violated", set())
        self.relax = getattr(self, "relax", set())
        self.tight = getattr(self, "tight", {})
        for i in range(len(out) - 1):
            a0, l0 = out[i][0], out[i][1]
            a1, l1, lo1, hi1, d1, g1, ai1 = out[i + 1]
            step = p["max_step"] * (a1 - a0) / 500.0 + float(p.get("slalom_tol", 60.0))
            if abs(l1 - l0) > step:
                self.tight[(k, ai1)] = FLD.lane_point(k, a1, d1, self.dir)
            if abs(l1 - l0) > step and (k, ai1) in self.relax:
                side = 1.0 if l0 > d1 else -1.0
                lane = d1 + side * g1
                lane = max(lane, l0) if side > 0 else min(lane, l0)
                out[i + 1][1] = float(min(max(lane, half + p["wall_clear"]), 1000.0 - half - p["wall_clear"]))
                self.violated.add((k, ai1))
        return [(r[0], r[1]) for r in out]

    def rebuild(self):
        order = [0, 1, 2, 3] if self.dir > 0 else [0, 3, 2, 1]
        n = len(order)
        cut_in = [-500.0] * n
        cut_out = [500.0] * n
        arcs = []
        self.t1 = {}
        for i, k in enumerate(order):
            nk = order[(i + 1) % n]
            d0, d1 = self.lane_d(k, 500.0), self.lane_d(nk, -500.0)
            a_I_k = 500.0 + (1000.0 - d1)
            a_I_nk = -500.0 - (1000.0 - d0)
            R = self.corner_radius(k, nk, d0, d1, a_I_k, a_I_nk)
            self.t1[k] = a_I_k - R
            cut_out[i] = min(500.0, a_I_k - R)
            cut_in[(i + 1) % n] = max(-500.0, a_I_nk + R)
            I = np.array(FLD.lane_point(k, a_I_k, d0, self.dir))
            T1 = np.array(FLD.lane_point(k, a_I_k - R, d0, self.dir))
            T2 = np.array(FLD.lane_point(nk, a_I_nk + R, d1, self.dir))
            C = T1 + (T2 - I)
            a1 = math.atan2(T1[1] - C[1], T1[0] - C[0])
            arcs.append((T1, C, a1, a_I_k - R, a_I_nk + R, k, nk, d0, d1, R))
        pts = []
        self.radii = []
        for i, k in enumerate(order):
            a_lo, a_hi = cut_in[i], cut_out[i]
            if a_lo < -500.0 + 1e-6:
                a_lo = -500.0
            for a in np.arange(a_lo, a_hi, self.step):
                pts.append(FLD.lane_point(k, a, self.lane_d(k, a), self.dir))
            T1, C, a1, a_t1, a_t2, _k, nk, d0, d1, R = arcs[i]
            self.radii.append(round(R))
            for a in np.arange(max(a_hi, 500.0), a_t1, self.step):
                pts.append(FLD.lane_point(k, a, d0, self.dir))
            m = max(12, int(R * math.pi / 2 / self.step))
            for j in range(m + 1):
                ang = a1 + self.dir * (j / m) * math.pi / 2
                pts.append((C[0] + R * math.cos(ang), C[1] + R * math.sin(ang)))
            for a in np.arange(a_t2 + self.step, -500.0, self.step):
                pts.append(FLD.lane_point(nk, a, d1, self.dir))
        self.path = np.array(pts)
        seg = np.hypot(*np.diff(np.vstack([self.path, self.path[:1]]), axis=0).T)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])[:-1]
        self.length = float(seg.sum())
        self.version += 1

    def sign_d(self, k, ai):
        v = self.seat.get((k, ai), {})
        if v.get("xy"):
            sx, sy, sn = v["xy"]
            _a, d = FLD.to_section(k, sx / sn, sy / sn)
            return float(min(max(d, 380.0), 620.0))
        return 400.0

    def pillars_xy(self):
        out = []
        for key, v in self.seat.items():
            if self.colour(key) and v.get("xy"):
                sx, sy, sn = v["xy"]
                out.append((sx / sn, sy / sn))
        return out

    def corner_poses(self, k, nk, d0, d1, a_I_k, a_I_nk, R, lead=260.0, step=25.0):
        I = np.array(FLD.lane_point(k, a_I_k, d0, self.dir))
        T1 = np.array(FLD.lane_point(k, a_I_k - R, d0, self.dir))
        T2 = np.array(FLD.lane_point(nk, a_I_nk + R, d1, self.dir))
        C = T1 + (T2 - I)
        h0 = FLD.section_heading(k, self.dir)
        poses = []
        for a in np.arange(a_I_k - R - lead, a_I_k - R, step):
            x, y = FLD.lane_point(k, a, d0, self.dir)
            poses.append((x, y, h0))
        a1 = math.atan2(T1[1] - C[1], T1[0] - C[0])
        m = max(4, int(R * math.pi / 2 / step))
        for j in range(m + 1):
            ang = a1 + self.dir * (j / m) * math.pi / 2
            poses.append((C[0] + R * math.cos(ang), C[1] + R * math.sin(ang), h0 + self.dir * (j / m) * math.pi / 2))
        h1 = FLD.section_heading(nk, self.dir)
        for a in np.arange(a_I_nk + R, a_I_nk + R + lead, step):
            x, y = FLD.lane_point(nk, a, d1, self.dir)
            poses.append((x, y, h1))
        return np.array(poses)

    @staticmethod
    def footprint_clearance(poses, pts, rear, front, half):
        if not len(pts) or not len(poses):
            return 1e9
        P_ = np.asarray(pts, float)
        dx = P_[None, :, 0] - poses[:, None, 0]
        dy = P_[None, :, 1] - poses[:, None, 1]
        c, sn = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
        a = c * dx + sn * dy
        b = -sn * dx + c * dy
        ex = np.maximum(np.maximum(-rear - a, a - front), 0.0)
        ey = np.maximum(np.abs(b) - half, 0.0)
        return float(np.hypot(ex, ey).min())

    def wall_clearance(self, poses):
        rear, front, half = self.p["car_rear"], self.p["car_front"], self.p["car_half"]
        c, s_ = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
        a = np.array([-rear, front, front, -rear])[None, :]
        b = np.array([-half, -half, half, half])[None, :]
        cx = poses[:, 0:1] + c * a - s_ * b
        cy = poses[:, 1:2] + s_ * a + c * b
        return float(np.min(FLD.HALF - np.maximum(np.abs(cx), np.abs(cy))))

    def corner_radius(self, k, nk, d0, d1, a_I_k, a_I_nk):
        ic = FLD.lane_point(k, 500.0, 1000.0, self.dir)
        pil = [q for q in self.pillars_xy() if math.hypot(q[0] - ic[0], q[1] - ic[1]) < 1300.0]
        pts = [ic] + [q for q in self.extra_pts if math.hypot(q[0] - ic[0], q[1] - ic[1]) < 1500.0]
        for px, py in pil:
            pts += [(px + ox, py + oy) for ox in (-25.0, 25.0) for oy in (-25.0, 25.0)]
        best_r, best_c = self.p["r_corner"], -1.0
        for R in [self.p["r_corner"]] + [float(r) for r in self.p["r_cands"]]:
            poses = self.corner_poses(k, nk, d0, d1, a_I_k, a_I_nk, R)
            clr = self.footprint_clearance(poses, pts, self.p["car_rear"], self.p["car_front"], self.p["car_half"])
            clr = min(clr, self.wall_clearance(poses))
            if R == self.p["r_corner"] and clr >= self.p["corner_ok"]:
                return R
            if clr > best_c + 2.0:
                best_r, best_c = R, clr
        return best_r

    def vote(self, key, what: str, xy=None):
        v = self.seat.setdefault(key, {})
        before = self.colour(key)
        v[what] = v.get(what, 0) + 1
        if xy is not None:
            sx, sy, sn = v.get("xy", (0.0, 0.0, 0))
            v["xy"] = (sx + xy[0], sy + xy[1], sn + 1)
        return self.colour(key) != before

    def nearest(self, x, y) -> int:
        return int(np.argmin((self.path[:, 0] - x) ** 2 + (self.path[:, 1] - y) ** 2))

    def target(self, x, y, hint: int, look: float):
        n = len(self.path)
        idx = (hint + np.arange(-30, 90)) % n
        d2 = (self.path[idx, 0] - x) ** 2 + (self.path[idx, 1] - y) ** 2
        i = int(idx[int(np.argmin(d2))])
        s1 = (self.s[i] + look) % self.length
        j = int(np.searchsorted(self.s, s1)) % n
        return i, self.path[j], math.sqrt(float(d2.min()))


# ------------------------------------------------------------------------------------------ the field map (ICP)
def wall_segments(lot_rects=()):
    """Axis-aligned FACES (M x 6: x1, y1, x2, y2, nx, ny -- the outward normal of the face) the ICP matches: the outer
    square's inner faces, the island's outer faces, the lot's limitations (rectangles xa, xb, ya, yb) all round.  A
    return is matched only to a face that looks toward the lidar: a limitation is 20 mm thick, and without the normals
    a pose 20 mm off matched its far face and was never corrected (SIM, inside the lot)."""
    h, i = FLD.HALF, FLD.ISL
    segs = [[-h, -h, h, -h, 0, 1], [h, -h, h, h, -1, 0], [h, h, -h, h, 0, -1], [-h, h, -h, -h, 1, 0],
            [-i, -i, i, -i, 0, -1], [i, -i, i, i, 1, 0], [i, i, -i, i, 0, 1], [-i, i, -i, -i, -1, 0]]
    for xa, xb, ya, yb in lot_rects:
        segs += [[xa, ya, xb, ya, 0, -1], [xb, ya, xb, yb, 1, 0], [xa, yb, xb, yb, 0, 1], [xa, ya, xa, yb, -1, 0]]
    return np.array(segs, float)


def seg_assoc(px, py, segs, sensor=None, margin=8.0):
    """Per point: the nearest face it projects INSIDE of (endpoints excluded) among the faces that look toward
    `sensor` (x, y), its signed residual along the axis and whether that face is vertical.  (dist, residual,
    vertical, index); dist inf = none."""
    x1, y1, x2, y2 = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    vert = np.abs(x1 - x2) < 1e-6
    lo_y, hi_y = np.minimum(y1, y2), np.maximum(y1, y2)
    lo_x, hi_x = np.minimum(x1, x2), np.maximum(x1, x2)
    PX, PY = px[:, None], py[:, None]
    rv = PX - x1[None, :]
    rh = PY - y1[None, :]
    inv = (PY >= lo_y[None, :] - margin) & (PY <= hi_y[None, :] + margin)
    inh = (PX >= lo_x[None, :] - margin) & (PX <= hi_x[None, :] + margin)
    R = np.where(vert[None, :], rv, rh)
    ok = np.where(vert[None, :], inv, inh)
    if sensor is not None:
        faces = ((sensor[0] - x1) * segs[:, 4] + (sensor[1] - y1) * segs[:, 5]) > 0.0
        ok = ok & faces[None, :]
    D = np.where(ok, np.abs(R), np.inf)
    j = np.argmin(D, axis=1)
    ar = np.arange(len(px))
    return D[ar, j], R[ar, j], vert[j], j


def icp(Xc, Yc, x, y, th, segs, gate, lam, iters=3, lidar=(0.0, 0.0)):
    """Damped point-to-line ICP of car-frame points onto the segments from the prior (x, y, th).  Returns
    (x, y, th, n_vertical, n_horizontal, rms) or None."""
    x0, y0, th0 = x, y, th
    nv = nh = 0
    rms = 0.0
    for _ in range(iters):
        c, s = math.cos(th), math.sin(th)
        px, py = x + c * Xc - s * Yc, y + s * Xc + c * Yc
        sen = (x + c * lidar[0] - s * lidar[1], y + s * lidar[0] + c * lidar[1])
        d, r, vert, _j = seg_assoc(px, py, segs, sen)
        ok = d < gate
        if int(ok.sum()) < 10:
            return None
        r, vert, pxo, pyo = r[ok], vert[ok], px[ok], py[ok]
        w = np.minimum(1.0, 20.0 / np.maximum(np.abs(r), 1e-6))
        J = np.stack([vert.astype(float), (~vert).astype(float), np.where(vert, -(pyo - y), pxo - x)], 1)
        Jw = J * w[:, None]
        H = Jw.T @ J + np.diag([lam, lam, lam * 4.0e5])
        g = Jw.T @ r
        dlt = -np.linalg.solve(H, g)
        x, y, th = x + dlt[0], y + dlt[1], th + dlt[2]
        nv, nh = int(vert.sum()), int((~vert).sum())
        rms = float(np.sqrt(np.mean(r * r)))
    return x, y, th, nv, nh, rms


def plan_exit_q(car, lot_len, margin, y_min):
    """park.plan_exit_to_lane with Q at least `y_min` off the wall: the same wiggle (park.plan_exit), then the S back to
    heading 0 with the smallest alpha whose end lies >= y_min out and whose legs clear everything by `margin`.  At the
    plan's own minimum (Q 303 mm out for the WLtoys) the body passed the limitations' free ends 47 mm off on the way in
    and the shield held the car beside the lot (SIM)."""
    obs = K.obstacles(lot_len)
    legs = K.plan_exit(car, lot_len, margin, max_legs=40)[:-1]
    pose = legs[-1]["end"]
    for alpha_deg in np.arange(0.0, 85.0, 1.0):
        a = math.radians(alpha_deg)
        ok, p_, clr_a = True, pose, 1e9
        for _ in range(int(a * car.r_min)):
            p_ = K.step(p_, 1, 1, 1.0, car)
            c = K.clearance(car, p_, obs)
            clr_a = min(clr_a, c)
            if c < margin:
                ok = False
                break
        if not ok:
            continue
        pa = p_
        clr_b = 1e9
        for _ in range(int(pa[2] * car.r_min)):
            p_ = K.step(p_, 1, -1, 1.0, car)
            c = K.clearance(car, p_, obs)
            clr_b = min(clr_b, c)
            if c < margin:
                ok = False
                break
        if not ok or p_[1] < y_min:
            continue
        q = (p_[0], p_[1], 0.0)
        legs.append(dict(dir=1, steer=1, dist_mm=round(a * car.r_min, 1), dth_rad=a, start=pose, end=pa,
                         clearance=round(clr_a, 1)))
        legs.append(dict(dir=1, steer=-1, dist_mm=round(pa[2] * car.r_min, 1), dth_rad=-pa[2], start=pa, end=q,
                         clearance=round(clr_b, 1)))
        return legs
    raise RuntimeError("no S-curve reaches %.0f mm out" % y_min)


_PLAN_CACHE = {}
PLAN_SEED = {
    # wltoys_bw2 (car 43.1 / 181.8 / 55.5, wheelbase 137, lock 22, lot 337.4), margin 28, Q >= 360 mm out:
    # plan_exit_q's output, frozen here so a race start does not compute it on the Pi (~8-10 s)
    "[43.1, 181.8, 55.5, 137.0, 22.0, 337.4, 28.0, 360.0]": [
        {"dir": -1, "steer": 0, "dist_mm": 28.0, "dth_rad": 0.0, "start": [99.35, 100.0, 0.0], "end": [71.35, 100.0, 0.0], "clearance": 28.2},
        {"dir": 1, "steer": 1, "dist_mm": 50.0, "dth_rad": 0.147455, "start": [71.35, 100.0, 0.0], "end": [121.169, 103.6797, 0.1475], "clearance": 28.2},
        {"dir": -1, "steer": -1, "dist_mm": 38.0, "dth_rad": 0.112066, "start": [121.169, 103.6797, 0.1475], "end": [83.9725, 96.0044, 0.2595], "clearance": 28.1},
        {"dir": 1, "steer": 1, "dist_mm": 37.0, "dth_rad": 0.109117, "start": [83.9725, 96.0044, 0.2595], "end": [119.1451, 107.4296, 0.3686], "clearance": 28.7},
        {"dir": -1, "steer": -1, "dist_mm": 27.0, "dth_rad": 0.079626, "start": [119.1451, 107.4296, 0.3686], "end": [94.3727, 96.7083, 0.4483], "clearance": 28.0},
        {"dir": 1, "steer": 1, "dist_mm": 35.0, "dth_rad": 0.103218, "start": [94.3727, 96.7083, 0.4483], "end": [125.0766, 113.4769, 0.5515], "clearance": 28.4},
        {"dir": -1, "steer": -1, "dist_mm": 27.0, "dth_rad": 0.079626, "start": [125.0766, 113.4769, 0.5515], "end": [102.6666, 98.4301, 0.6311], "clearance": 28.2},
        {"dir": 1, "steer": 1, "dist_mm": 136.1, "dth_rad": 0.401426, "start": [102.6666, 98.4301, 0.6311], "end": [193.6716, 198.2673, 1.0322], "clearance": 28.8},
        {"dir": 1, "steer": -1, "dist_mm": 350.0, "dth_rad": -1.032184, "start": [193.6716, 198.2673, 1.0322], "end": [483.751, 363.4196, 0.0], "clearance": 54.3},
    ],
}


def _plan_key(car, lot_len, margin, y_min):
    return json.dumps([round(float(v), 2) for v in (car.rear, car.front, car.half_w, car.wheelbase, car.steer_max_deg,
                                                    lot_len, margin, y_min)])


def plan_cached(car, lot_len, margin, y_min):
    """plan_exit_q remembered: PLAN_SEED (shipped for wltoys_bw2), else computed once per process."""
    key = _plan_key(car, lot_len, margin, y_min)
    if key in _PLAN_CACHE:
        return _PLAN_CACHE[key]
    legs = None
    if key in PLAN_SEED:
        legs = PLAN_SEED[key]
    if legs is None:
        legs = plan_exit_q(car, lot_len, margin, y_min)
    legs = [dict(lg, start=tuple(lg["start"]), end=tuple(lg["end"])) for lg in legs]
    _PLAN_CACHE[key] = legs
    return legs


# ------------------------------------------------------------------------------------------ v12: the asymmetric plan
# (obs_v6's planner, its leg DRIVER not taken: v6 failed on the drive, not the plan)
def step2(pose, d, s, ds, rp, rn):
    """park.step with its own radius per lot-frame steering sign: +1 turns about a centre rp to the LEFT (toward the
    field), -1 about rn to the right."""
    x, y, th = pose
    dd = d * ds
    if s == 0:
        return x + dd * math.cos(th), y + dd * math.sin(th), th
    R = rp if s > 0 else -rn
    th2 = th + dd / R
    return x + R * (math.sin(th2) - math.sin(th)), y - R * (math.cos(th2) - math.cos(th)), th2


def _run_leg2(pose, d, s, car, obs, margin, rp, rn, max_mm=600.0):
    done, clr = 0.0, K.clearance(car, pose, obs)
    while done < max_mm:
        nxt = step2(pose, d, s, 1.0, rp, rn)
        c = K.clearance(car, nxt, obs)
        if c < margin:
            break
        pose, done, clr = nxt, done + 1.0, c
    return pose, done, clr


def plan_exit6(car, lot_len, margin, y_min, rp, rn, free_mm=350.0, max_legs=40):
    """plan_exit_q on the asymmetric lock: back to the rear margin, then forward +1 / reverse -1 until a forward +1
    leg runs free_mm, then the S (forward +1 to alpha, forward -1 back to heading 0) with the smallest alpha whose end
    is >= y_min out and whose legs keep `margin`.  Legs as park.py's (dir, steer, dist_mm, dth_rad, start, end)."""
    obs = K.obstacles(lot_len)
    pose = K.parked_pose(car, lot_len)
    end, dist, clr = _run_leg2(pose, -1, 0, car, obs, margin, rp, rn)
    legs = [dict(dir=-1, steer=0, dist_mm=round(dist, 1), dth_rad=0.0, start=pose, end=end, clearance=round(clr, 1))]
    pose = end
    for _ in range(max_legs):
        end, dist, clr = _run_leg2(pose, 1, 1, car, obs, margin, rp, rn, max_mm=free_mm + 1)
        if dist >= free_mm:
            break
        legs.append(dict(dir=1, steer=1, dist_mm=round(dist, 1), dth_rad=end[2] - pose[2], start=pose, end=end,
                         clearance=round(clr, 1)))
        pose = end
        end, dist, clr = _run_leg2(pose, -1, -1, car, obs, margin, rp, rn)
        legs.append(dict(dir=-1, steer=-1, dist_mm=round(dist, 1), dth_rad=end[2] - pose[2], start=pose, end=end,
                         clearance=round(clr, 1)))
        pose = end
    for alpha_deg in range(0, 85):
        a = math.radians(alpha_deg)
        if a <= pose[2]:
            continue
        ok, p_, clr_a = True, pose, 1e9
        for _ in range(int((a - pose[2]) * rp)):
            p_ = step2(p_, 1, 1, 1.0, rp, rn)
            c = K.clearance(car, p_, obs)
            clr_a = min(clr_a, c)
            if c < margin:
                ok = False
                break
        if not ok:
            continue
        pa, clr_b = p_, 1e9
        for _ in range(int(pa[2] * rn)):
            p_ = step2(p_, 1, -1, 1.0, rp, rn)
            c = K.clearance(car, p_, obs)
            clr_b = min(clr_b, c)
            if c < margin:
                ok = False
                break
        if not ok or p_[1] < y_min:
            continue
        legs.append(dict(dir=1, steer=1, dist_mm=round((pa[2] - pose[2]) * rp, 1), dth_rad=pa[2] - pose[2],
                         start=pose, end=pa, clearance=round(clr_a, 1)))
        legs.append(dict(dir=1, steer=-1, dist_mm=round(pa[2] * rn, 1), dth_rad=-pa[2], start=pa,
                         end=(p_[0], p_[1], 0.0), clearance=round(clr_b, 1)))
        return legs
    raise RuntimeError("no S-curve reaches %.0f mm out" % y_min)


PLAN12_SEED = {
    # wltoys_bw2 (car 43.1 / 181.8 / 55.5, wb 137), lot_len 337.4 (the profile's, as obs_v11's park), margin 28 (obs_v11's),
    # Q >= 360 out, true lock left 30 / right 22: ccw (+1 = left, R 237) 7 legs, cw (+1 = right, R 339) 9 -- plan_exit6
    '[43.1, 181.8, 55.5, 337.4, 28.0, 360.0, 237.3, 339.1]': [
        {"dir": -1, "steer": 0, "dist_mm": 28.0, "dth_rad": 0.0, "start": [99.35, 100.0, 0.0], "end": [71.35, 100.0, 0.0], "clearance": 28.2},
        {"dir": 1, "steer": 1, "dist_mm": 49.0, "dth_rad": 0.206498, "start": [71.35, 100.0, 0.0], "end": [120.0025, 105.0412, 0.2065], "clearance": 28.1},
        {"dir": -1, "steer": -1, "dist_mm": 35.0, "dth_rad": 0.103218, "start": [120.0025, 105.0412, 0.2065], "end": [86.1769, 96.1114, 0.3097], "clearance": 28.2},
        {"dir": 1, "steer": 1, "dist_mm": 39.0, "dth_rad": 0.164355, "start": [86.1769, 96.1114, 0.3097], "end": [122.1797, 110.9903, 0.4741], "clearance": 28.1},
        {"dir": -1, "steer": -1, "dist_mm": 26.0, "dth_rad": 0.076677, "start": [122.1797, 110.9903, 0.4741], "end": [99.5245, 98.2462, 0.5507], "clearance": 28.4},
        {"dir": 1, "steer": 1, "dist_mm": 126.0, "dth_rad": 0.530994, "start": [99.5245, 98.2462, 0.5507], "end": [184.8194, 188.9724, 1.0817], "clearance": 28.9},
        {"dir": 1, "steer": -1, "dist_mm": 366.8, "dth_rad": -1.081741, "start": [184.8194, 188.9724, 1.0817], "end": [483.3533, 368.7581, 0.0], "clearance": 58.8},
    ],
    '[43.1, 181.8, 55.5, 337.4, 28.0, 360.0, 339.1, 237.3]': [
        {"dir": -1, "steer": 0, "dist_mm": 28.0, "dth_rad": 0.0, "start": [99.35, 100.0, 0.0], "end": [71.35, 100.0, 0.0], "clearance": 28.2},
        {"dir": 1, "steer": 1, "dist_mm": 50.0, "dth_rad": 0.147455, "start": [71.35, 100.0, 0.0], "end": [121.169, 103.6797, 0.1475], "clearance": 28.2},
        {"dir": -1, "steer": -1, "dist_mm": 36.0, "dth_rad": 0.151712, "start": [121.169, 103.6797, 0.1475], "end": [86.0966, 95.7148, 0.2992], "clearance": 28.6},
        {"dir": 1, "steer": 1, "dist_mm": 36.0, "dth_rad": 0.106167, "start": [86.0966, 95.7148, 0.2992], "end": [119.8702, 108.1294, 0.4053], "clearance": 28.6},
        {"dir": -1, "steer": -1, "dist_mm": 24.0, "dth_rad": 0.101142, "start": [119.8702, 108.1294, 0.4053], "end": [98.3307, 97.5673, 0.5065], "clearance": 28.1},
        {"dir": 1, "steer": 1, "dist_mm": 35.0, "dth_rad": 0.103218, "start": [98.3307, 97.5673, 0.5065], "end": [128.007, 116.0937, 0.6097], "clearance": 28.6},
        {"dir": -1, "steer": -1, "dist_mm": 28.0, "dth_rad": 0.117999, "start": [128.007, 116.0937, 0.6097], "end": [106.0501, 98.7449, 0.7277], "clearance": 28.6},
        {"dir": 1, "steer": 1, "dist_mm": 149.0, "dth_rad": 0.439415, "start": [106.0501, 98.7449, 0.7277], "end": [192.3374, 218.7472, 1.1671], "clearance": 29.3},
        {"dir": 1, "steer": -1, "dist_mm": 276.9, "dth_rad": -1.167109, "start": [192.3374, 218.7472, 1.1671], "end": [409.6102, 362.8255, 0.0], "clearance": 77.4},
    ],
}


def _plan12_key(car, lot_len, margin, y_min, rp, rn):
    return json.dumps([round(float(v), 1) for v in (car.rear, car.front, car.half_w, lot_len, margin, y_min, rp, rn)])


def plan12_cached(car, lot_len, margin, y_min, rp, rn):
    """v12: plan_exit6 (obs_v6's planner on the REAL asymmetric lock) remembered: PLAN12_SEED, else computed once."""
    key = _plan12_key(car, lot_len, margin, y_min, rp, rn)
    if key in _PLAN_CACHE:
        return _PLAN_CACHE[key]
    legs = PLAN12_SEED.get(key) or plan_exit6(car, lot_len, margin, y_min, rp, rn)
    legs = [dict(lg, start=tuple(lg["start"]), end=tuple(lg["end"])) for lg in legs]
    _PLAN_CACHE[key] = legs
    return legs


def run(robot, params, log, stop, hook=None):
    prog = dict(robot.p.get("prog", {}).get("obs_v1", {}) or {})
    preset = str((params or {}).get("preset", prog.get("preset", DEFAULTS["preset"])))
    p = dict(DEFAULTS)
    p.update(PRESETS.get(preset, {}))
    p.update(prog)
    p.update(params or {})
    p["preset"] = preset
    return _race(robot, p, log, stop)


def _race(robot, p, log, stop):
    rp = robot.p
    mx = float(rp["steer"]["max_deg"])
    cap = min(float(p["steer_cap"]), mx)
    wb_slow = float(rp["chassis"]["wheelbase_m"]) * 1000.0
    pc = LP.config(rp)
    lidar_x, lidar_y = (float(v) for v in pc["pos"])
    lag_scan = float(pc["lag"])
    cp = dict(dict(rear_mm=34.0, front_mm=179.0, half_w_mm=80.5, lot_len_mm=K.LOT_LEN), **rp.get("car", {}))
    rear, front, half = float(cp["rear_mm"]), float(cp["front_mm"]), float(cp["half_w_mm"])
    lot_len = float(cp["lot_len_mm"])
    car = K.Car(rear=rear, front=front, half_w=half, wheelbase=wb_slow, steer_max_deg=float(p["park_lock_deg"]))
    trim = float(p["steer_trim"])
    cap_l = min(float(p["steer_cap_left"]), mx - trim)     # v3: the servo clamp +-mx sits at true mx - trim / mx + trim
    cap_r = min(float(p["steer_cap_right"]), mx + trim)
    cap = min(cap_l, cap_r)
    cam = dict(rp.get("camera") or {})
    vp = dict(rp.get("vision") or {})
    lat_cam = float(cam.get("latency_s", 0.12))
    cam_xy = (float(cam.get("x_mm", 150.0)), float(cam.get("y_mm", 0.0)))
    wb_by = "params"
    if p["wb_v"] is None or p["wb_mm"] is None:
        curve = None
        try:
            with open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "profiles",
                                   "plant_mat1001.json"), encoding="utf-8") as f_:
                curve = json.load(f_)["plant"]["wb_curve"]
            wb_by = "plant_mat1001.json"
        except (OSError, ValueError, KeyError, TypeError):
            curve = None
        if not curve:
            curve, wb_by = [[0.2, wb_slow], [0.8, float(p["wb_eff"])]], "wb_eff"
        p["wb_v"] = [float(a) for a, _b in curve]
        p["wb_mm"] = [wb_slow] + [float(b) for _a, b in curve[1:]]
    wb_v, wb_mm = [float(v) for v in p["wb_v"]], [float(v) for v in p["wb_mm"]]

    def wb_of(v):
        return float(np.interp(abs(v), wb_v, wb_mm))

    class _Trimmed:
        """The robot with the wheels' straight-ahead at `trim`: the shield plans on true angles, the servo gets +trim
        (open_fast_v2); pulse() too (the park's step mode)."""
        def drive(self, v, s, ttl=None):
            return robot.drive(v, max(-mx, min(mx, float(s) + trim)), ttl)

        def pulse(self, sign, s, mm):
            return robot.pulse(sign, max(-mx, min(mx, float(s) + trim)), mm)

        def __getattr__(self, name):
            return getattr(robot, name)

    # park mode: the legs are planned park_margin (22) from everything and end on measured quantities; the shield's
    # own park margin + stop (8 + 10 mm) on top braked every leg ending near a limitation (SIM: 'blocked', the next leg
    # then began off plan and the tail met the rear limitation)
    shield_ov = {"on": int(p["shield"]), "park_margin_mm": float(p["shield_park_margin"]),
                 "park_stop_mm": float(p["shield_park_stop"])}
    if float(p["shield_h"]) > 0.0:
        shield_ov["horizon_mm"] = float(p["shield_h"])
    if float(p["shield_decel"]) > 0.0:
        shield_ov["decel"] = float(p["shield_decel"])
    shield = SH.Shield(rp, shield_ov)
    bot = SH.command(_Trimmed() if trim else robot, shield,
                     robot.prog_ttl_ms() if hasattr(robot, "prog_ttl_ms") else None)
    from bluewave import loc as L
    sh_odo = L.Odo()

    class Halt(Exception):
        pass

    t0 = time.monotonic()
    robot.reset_yaw()
    st = dict(t_prev=t0, yaw=robot.yaw, yaw_f=robot.yaw, yr=0.0, scan_t=0.0, log_t=0.0, cmd=(0.0, 0.0), odo=0.0,
              wrong=None, stuck=None, frame_t=0.0, cam_t=0.0, last_dir=1.0, n_icp=0, n_icp_fail=0, v_est=0.0)
    P = dict(x=0.0, y=-1400.0, th_off=0.0)           # the rear axle in the field; th = yaw + th_off
    hist = deque(maxlen=400)
    end = dict(reason=None)
    seats = {}                                       # seat index -> dict(hits, xy sums)
    lot = dict(x0=None, rects=[])
    segs = [wall_segments()]
    planner = [None]
    stats = dict(icp_rms=[], guard=0, holds=0, cam_frames=0, cam_votes=0, lidar_pill=0, look_n=0, look_votes=0)
    saver = LookSaver(int(p["look_save"]), float(p["look_save_gap_s"]))
    vpm = dict(V.DEFAULT_VISION, **vp)                  # v7: the hues map_look counts (the robot's vision params)
    g_cam = [None]

    def th_now():
        return math.radians(robot.yaw) + P["th_off"]

    def pose_at(t):
        if not hist or t >= hist[-1][0]:
            return P["x"], P["y"], th_now()
        if t <= hist[0][0]:
            return hist[0][1], hist[0][2], hist[0][3]
        for i in range(len(hist) - 1, 0, -1):
            if hist[i - 1][0] <= t:
                (ta, xa, ya, tha), (tb, xb, yb, thb) = hist[i - 1], hist[i]
                f = (t - ta) / max(tb - ta, 1e-6)
                return xa + f * (xb - xa), ya + f * (yb - ya), tha + f * wrap(thb - tha)
        return hist[0][1], hist[0][2], hist[0][3]

    def set_lot(x0):
        lot["x0"] = x0
        rects = []
        for xa_, xb_ in ((-K.LIMIT_T, 0.0), (lot_len, lot_len + K.LIMIT_T)):
            fa = K.to_field((xa_, 0.0, 0.0), x0, direction[0])[0]
            fb = K.to_field((xb_, 0.0, 0.0), x0, direction[0])[0]
            rects.append((min(fa, fb), max(fa, fb), -1500.0, -1500.0 + K.LOT_DEPTH))
        lot["rects"] = rects
        segs[0] = wall_segments(rects)
        pts = []
        for xa_, xb_, ya_, yb_ in rects:
            pts += [(xa_, ya_), (xb_, ya_), (xa_, yb_), (xb_, yb_), ((xa_ + xb_) / 2.0, yb_)]
        lot["pts"] = pts
        if planner[0] is not None:
            planner[0].extra_pts = list(pts)

    direction = [int(p["dir"]) if p["dir"] in (1, -1) else 0]

    def capt():
        """v3: the lock the corners can use: the turn side's (ccw turns left, cw right)."""
        d_ = direction[0]
        return cap_l if d_ > 0 else (cap_r if d_ < 0 else min(cap_l, cap_r))
    dirn_ = direction
    rad = [None, None]                                   # v12: the plan's radii for lot-frame steer +1 / -1 (mm)

    # ------------------------------------------------------------------------------ perception per scan
    def on_scan(sc, perc):
        ts = sc.t - lag_scan
        xs, ys, ths = pose_at(ts)
        if perc is None:
            return
        Xw, Yw = perc.points(exclude=(LP.CLS_PILLAR, LP.CLS_CLUTTER))
        res = icp(Xw, Yw, xs, ys, ths, segs[0], float(p["icp_gate"]), float(p["icp_lam"]),
                  lidar=(lidar_x, lidar_y)) if len(Xw) >= 10 else None
        if res is None:
            st["n_icp_fail"] += 1
        else:
            xn, yn, thn, nv, nh, rms = res
            dx, dy = xn - xs, yn - ys
            dth = max(-math.radians(1.5), min(math.radians(1.5), wrap(thn - ths)))
            if math.hypot(dx, dy) < 250.0:
                P["x"] += dx
                P["y"] += dy
                P["th_off"] += float(p["icp_th_gain"]) * dth
                st["n_icp"] += 1
                stats["icp_rms"].append(rms)
                del stats["icp_rms"][:-400]
                if nv >= 8 and nh >= 8 and hasattr(robot, "feed_pose"):
                    try:
                        robot.feed_pose(ts, xn, yn, thn)
                    except Exception:
                        pass
                xs, ys, ths = xn, yn, ths + float(p["icp_th_gain"]) * dth
        # the pillars: lidar_perc's clusters -> the seats (once the field pose is fixed: the exit runs on a nominal x)
        pl = perc.pillars
        if st.get("pose_ok") and pl is not None and len(pl):
            c, s = math.cos(ths), math.sin(ths)
            for row in pl:
                X, Y = float(row[0]), float(row[1])
                if math.hypot(X - lidar_x, Y - lidar_y) > float(p["pillar_max_mm"]) + 300.0:
                    continue
                fx, fy = xs + c * X - s * Y, ys + s * X + c * Y
                if max(abs(fx), abs(fy)) > FLD.HALF - float(p["wall_band_mm"]):
                    continue                        # v2: the lot limitation / the outer wall, not a sign
                dd = np.hypot(FLD.SEAT_XY[:, 0] - fx, FLD.SEAT_XY[:, 1] - fy)
                j = int(np.argmin(dd))
                if dd[j] > float(p["seat_snap"]):
                    continue
                sj = seats.setdefault(j, dict(hits=0, sx=0.0, sy=0.0))
                sj["hits"] += 1
                sj["sx"] += fx
                sj["sy"] += fy
                stats["lidar_pill"] += 1
                if sj["hits"] == 2:
                    k_, ai_, ci_ = FLD.SEATS[j][:3]
                    log(dict(ev="seat_seen", seat=j, k=k_, ai=ai_, ci=ci_, x=round(fx), y=round(fy)))

    def seat_xy(j):
        sj = seats[j]
        return sj["sx"] / sj["hits"], sj["sy"] / sj["hits"]

    def seat_colour(j):
        pl_ = planner[0]
        if pl_ is None:
            return None
        k_, ai_ = FLD.SEATS[j][:2]
        return pl_.colour((k_, ai_))

    def cam_step(now):
        """The camera's colour votes: vision.pillars -> the lidar seats by bearing (else the nearest seat)."""
        pl_ = planner[0]
        if pl_ is None or now - st["cam_t"] < 1.0 / float(p["cam_hz"]):
            return
        bgr, tf = robot.frame()
        if bgr is None or tf == st["frame_t"]:
            return
        st["frame_t"], st["cam_t"] = tf, now
        te = tf - lat_cam
        pe = pose_at(te)
        # only while something is left to read: a lidar seat without a colour within reach, or lap 1
        need = st.get("lap", 0) < 1
        if not need:
            for j in seats:
                if seats[j]["hits"] >= 2 and seat_colour(j) is None:
                    X_, Y_ = LP.field_to_car([seat_xy(j)[0]], [seat_xy(j)[1]], pe)
                    if 0.0 < float(X_[0]) < 2000.0:
                        need = True
                        break
        if not need:
            return
        if g_cam[0] is None:
            g_cam[0] = V.Ground(cam, bgr.shape[1], bgr.shape[0])
        try:
            dets = [q for q in V.pillars(bgr, g_cam[0], vp) if q["colour"] in ("red", "green")
                    and q["X"] <= float(p["pillar_max_mm"])
                    and not (q["touches_bottom"] and q.get("est") != "top")]
        except Exception as e:
            log(dict(ev="cam_error", error="%s: %s" % (type(e).__name__, e)))
            return
        stats["cam_frames"] += 1
        changed = map_look(bgr, tf, pe, pl_)            # v7: the map's unread signs first (no blob needed)
        if not dets:
            if changed:
                seats_changed(pl_)
            return
        tj = [j for j in seats if seats[j]["hits"] >= 2]
        pairs = []
        if tj:
            fxy = np.array([seat_xy(j) for j in tj])
            Xt, Yt = LP.field_to_car(fxy[:, 0], fxy[:, 1], pe)
            det_xy = np.array([(q["X"], q["Y"]) for q in dets])
            pairs = LP.associate(det_xy, np.stack([Xt, Yt], 1), cam_xy, float(p["bearing_tol"]), float(p["range_gate"]))
        used = set()
        c, s = math.cos(pe[2]), math.sin(pe[2])
        for di, ti, _dphi in pairs:
            used.add(di)
            j = tj[ti]
            k_, ai_ = FLD.SEATS[j][:2]
            changed |= pl_.vote((k_, ai_), dets[di]["colour"], seat_xy(j))
            stats["cam_votes"] += 1
        for di, q in enumerate(dets):
            if di in used:
                continue
            fx, fy = pe[0] + c * q["X"] - s * q["Y"], pe[1] + s * q["X"] + c * q["Y"]
            if max(abs(fx), abs(fy)) > FLD.HALF - float(p["wall_band_mm"]):
                continue                            # v2: the lot limitation seen red, not a sign
            dd = np.hypot(FLD.SEAT_XY[:, 0] - fx, FLD.SEAT_XY[:, 1] - fy)
            j = int(np.argmin(dd))
            if dd[j] <= float(p["cam_snap"]):
                k_, ai_ = FLD.SEATS[j][:2]
                changed |= pl_.vote((k_, ai_), q["colour"], (float(FLD.SEAT_XY[j, 0]), float(FLD.SEAT_XY[j, 1])))
                stats["cam_votes"] += 1
        if changed:
            seats_changed(pl_)

    def seats_changed(pl_):
        pl_.rebuild()
        log(dict(ev="seats", colours={"%d.%d" % k_: pl_.colour(k_) for k_ in sorted(pl_.seat) if pl_.colour(k_)},
                 radii=pl_.radii, x=round(P["x"]), y=round(P["y"]),
                 infeasible=["%d.%d" % k_ for k_ in sorted(getattr(pl_, "violated", set()))]))

    def map_look(bgr, tf, pe, pl_):
        """v7: the MAP's unread signs looked at where they ARE.  A lidar seat (hits >= 2, not the lot's band) without a
        colour, look_min_mm..look_max_mm from the lens, is projected into the frame at the exposure's pose: its face
        (the seat centre 25 mm nearer along the ray) from look_z_mm[0] to [1] above the mat.  That strip, +-slack
        wide, is counted pixel by pixel with look_s / look_v (vision.pillars needs a whole blob at s_col / v_col: a
        shadowed or glare-split sign gives none); the best window look_w x the sign's width decides when one colour
        fills look_frac of it and look_dom x the other.  One vote per frame; seat_votes (2) of them set the colour.
        Only the box is converted (a few hundred pixels): no cost to the loop.  Returns True when a colour changed."""
        if not int(p["look"]):
            return False
        import cv2
        g = g_cam[0]
        H_, W_ = bgr.shape[:2]
        z0, z1 = float(p["look_z_mm"][0]), float(p["look_z_mm"][1])
        gh, rh = vpm["green_h"], vpm["red_h"]
        changed = False
        for j in seats:
            if seats[j]["hits"] < 2 or seat_colour(j) is not None:
                continue
            sx_, sy_ = seat_xy(j)
            if max(abs(sx_), abs(sy_)) > FLD.HALF - float(p["wall_band_mm"]):
                continue                                # the lot's limitation, never a sign (v2)
            X_, Y_ = LP.field_to_car([sx_], [sy_], pe)
            X_, Y_ = float(X_[0]), float(Y_[0])
            r_ = math.hypot(X_ - cam_xy[0], Y_ - cam_xy[1])
            if not float(p["look_min_mm"]) <= r_ <= float(p["look_max_mm"]):
                continue
            fx_ = X_ - 25.0 * (X_ - cam_xy[0]) / r_
            fy_ = Y_ - 25.0 * (Y_ - cam_xy[1]) / r_
            u, v, d = g.project(np.array([[fx_, fx_], [fy_, fy_], [z0, z1]]))
            if not (np.all(d > 0) and np.all(np.isfinite(u)) and np.all(np.isfinite(v))):
                continue
            uc = float(np.mean(u))
            w_px = g.fx * 50.0 / max(float(np.mean(d)), 1.0)
            ww = max(3, int(round(float(p["look_w"]) * w_px)))
            slack = max(float(p["look_slack_px"]), 0.8 * w_px)
            ua, ub = int(math.floor(uc - slack - ww / 2.0)), int(math.ceil(uc + slack + ww / 2.0))
            va, vb = int(math.floor(float(np.min(v)))), int(math.ceil(float(np.max(v))))
            if uc < 0.0 or uc > W_ or va < 0 or vb > H_ or vb - va < 4:
                continue                                # its centre out of the frame, or its band cut: no verdict
            ua, ub = max(0, ua), min(W_, ub)            # a sign at the frame's edge: count the part inside
            if ub - ua < ww + 2:
                continue
            hsv = cv2.cvtColor(np.ascontiguousarray(bgr[va:vb, ua:ub]), cv2.COLOR_BGR2HSV)
            hh, ss, vv = hsv[..., 0], hsv[..., 1], hsv[..., 2]
            ok = (ss >= float(p["look_s"])) & (vv >= float(p["look_v"]))
            gm = ok & (hh >= gh[0]) & (hh <= gh[1])
            rm = ok & ((hh <= rh[0]) | (hh >= rh[1]))
            box = np.ones(ww)
            gs = np.convolve(gm.sum(0).astype(float), box, "valid")
            rs = np.convolve(rm.sum(0).astype(float), box, "valid")
            gb = float(gs.max()) if gs.size else 0.0
            rb = float(rs.max()) if rs.size else 0.0
            area = float(ww * (vb - va))
            col = None
            if gb >= float(p["look_frac"]) * area and gb >= float(p["look_dom"]) * rb:
                col = "green"
            elif rb >= float(p["look_frac"]) * area and rb >= float(p["look_dom"]) * gb:
                col = "red"
            stats["look_n"] += 1
            k_, ai_ = FLD.SEATS[j][:2]
            if col:
                changed |= pl_.vote((k_, ai_), col, (sx_, sy_))
                stats["look_votes"] += 1
            saver.put(bgr, dict(t=round(float(tf), 3), seat=int(j), key="%d.%d" % (k_, ai_), r=round(r_),
                                uc=round(uc, 1), box=[ua, va, ub, vb], ww=ww, green=round(gb), red=round(rb),
                                area=round(area), col=col, colour=pl_.colour((k_, ai_)),
                                pose=[round(pe[0]), round(pe[1]), round(math.degrees(pe[2]), 1)]))
        return changed

    # ------------------------------------------------------------------------------ the loop pass
    def go(v, s):
        v, s = float(v), float(s)
        st["cmd"] = (v, s)
        if st.get("imu_dead"):
            v = 0.0
        s = max(-cap_r, min(cap_l, s))
        if st.get("steer_hold") and time.monotonic() < st["steer_hold"]:
            v = 0.0
        if v == 0.0 and abs(float(robot.v_odo(1.0))) < 0.08:
            s = max(-p["stand_max_deg"], min(p["stand_max_deg"], s))
        bot.drive(v, s)

    # v17 bat_hold: the drive's bat_comp (duty x v_ref / V_bat) over-corrects this car -- the mat plant fit has the
    # motor's speed independent of the pack (profiles/plant_mat1001.json vbat_exp 0): at 6.1 V it added +31 % (the park
    # legs and the Q creep overshot 30-150 mm, mat 20261001-204736; SIM: true speed +38 %), at 8.1 V it took 13 % off
    # (slow; the lap-2 pose loss of 20261001-190131).  Every program setting today was tuned at ~7.0 V.  So for this
    # run v_ref follows the pack -- v_ref = v_ref0 x V_bat / bat_hold_v -- and the compensation stays v_ref0 /
    # bat_hold_v whatever the pack: the car drives at any voltage as it did at 7.0 V.  Restored at the end.
    rp_all = getattr(robot, "p", None)
    duty_cfg = rp_all.get("drive", {}).get("duty") if isinstance(rp_all, dict) else None
    v_ref0 = float(duty_cfg.get("v_ref", 8.0)) if isinstance(duty_cfg, dict) else None

    def bat_hold():
        if not int(p["bat_hold"]) or duty_cfg is None or v_ref0 is None or not hasattr(robot, "battery_v"):
            return
        vb = robot.battery_v()
        if vb and 5.0 < float(vb) < 9.5:
            duty_cfg["v_ref"] = v_ref0 * float(vb) / float(p["bat_hold_v"])

    def tick(mode):
        st["mode"] = mode
        if stop.is_set():
            end["reason"] = "estop" if getattr(robot, "estopped", False) else "stopped"
            raise Halt
        tl = time.monotonic()
        dt, st["t_prev"] = tl - st["t_prev"], tl
        bat_hold()
        if tl - t0 > p["seconds"]:
            end["reason"] = "time"
            raise Halt
        if hasattr(robot, "board_down_s") and robot.board_down_s() > 1.0:
            end["reason"] = "board_lost"
            raise Halt
        v_o = float(robot.v_odo(1.0))
        vc = float(getattr(robot, "v_cmd", 0.0))
        if vc > 0.01:
            st["last_dir"] = 1.0
        elif vc < -0.01:
            st["last_dir"] = -1.0
        if v_o > 0.0 and st["last_dir"] < 0.0:
            v_o = -v_o
        st["v_est"] = v_o
        ds = v_o * 1000.0 * dt
        st["ds"] = ds
        st["odo"] += abs(ds)
        th0_ = th_now() if not hist else hist[-1][3]
        th1_ = th_now()
        thm = th0_ + 0.5 * wrap(th1_ - th0_)
        P["x"] += ds * math.cos(thm)
        P["y"] += ds * math.sin(thm)
        hist.append((tl, P["x"], P["y"], th1_))
        dth = math.radians(wrapd(robot.yaw - st["yaw"]))
        st["yaw"] = robot.yaw
        if shield.enabled:
            shield.odom(ds, dth)
            sh_odo.add(tl, ds, dth)
        sc = robot.scan
        if sc is not None and sc.t != st["scan_t"]:
            st["scan_t"] = sc.t
            perc = LP.of(robot)
            if shield.enabled and perc is not None:
                shield.scan(perc, sh_odo.since)
            on_scan(sc, perc)
            st["new_scan"] = True
        yr = max(-400.0, min(400.0, wrapd(robot.yaw - st["yaw_f"]) / max(dt, 1e-3)))
        st["yaw_f"] = robot.yaw
        st["yr"] += (yr - st["yr"]) * min(1.0, dt / 0.08)
        # ---- the Open core's steering / board watchdogs (open_fast_v2)
        s_c = float(robot.steer_cmd) - trim
        if mode not in ("EXIT", "PARK") and abs(v_o) > 0.12 and abs(s_c) >= 20.0 \
                and st["yr"] * math.copysign(1.0, s_c) * math.copysign(1.0, v_o) < -p["steer_fault_dps"]:
            st["wrong"] = st["wrong"] or tl
            if tl - st["wrong"] >= p["steer_fault_s"]:
                log(dict(ev="steer_fault", steer=round(s_c, 1), yaw_rate=round(st["yr"], 1), mode=mode))
                end["reason"] = "steer_fault"
                raise Halt
        else:
            st["wrong"] = None
        if (mode not in ("EXIT", "PARK") and abs(s_c) >= p["follow_dead_deg"] and v_o > 0.3
                and st["yr"] * math.copysign(1.0, s_c) < -p["wrong_dps"] and st.get("wrong2_s", 0.0) * s_c > 0.0):
            st["wrong2"] = st.get("wrong2") or tl
            if tl - st["wrong2"] >= p["wrong_s"] and tl - st.get("reopen_t", -99.0) > p["reopen_gap_s"]:
                ok = robot.board_reopen() if hasattr(robot, "board_reopen") else False
                st["reopen_t"], st["steer_hold"], st["wrong2"] = tl, tl + 0.6, None
                log(dict(ev="steer_wrong", steer=round(s_c, 1), yaw_rate=round(st["yr"], 1), mode=mode,
                         reopened=bool(ok)))
        else:
            st["wrong2"] = None
        st["wrong2_s"] = s_c if abs(s_c) >= p["follow_dead_deg"] else 0.0
        ia = float(robot.imu_age_s()) if hasattr(robot, "imu_age_s") else 0.0
        st["imu_dead"] = ia >= p["dead_imu_s"]
        if st["imu_dead"] and tl - st.get("reopen_t", -99.0) > p["reopen_gap_s"]:
            ok = robot.board_reopen() if hasattr(robot, "board_reopen") else False
            st["reopen_t"] = tl
            log(dict(ev="board_hung", imu_age_ms=round(min(ia, 99.0) * 1000), mode=mode, reopened=bool(ok)))
        seen = robot.motion_seen() if hasattr(robot, "motion_seen") else None
        if (float(getattr(robot, "v_cmd", 0.0)) > 0.05 and seen is False) or getattr(robot, "stalled", False):
            st["stuck"] = st["stuck"] or tl
        else:
            st["stuck"] = None
        if mode in ("GO", "EXIT_BACK", "LOOK"):
            cam_step(tl)
        if tl - st["log_t"] >= p["log_every"]:
            st["log_t"] = tl
            log(dict(mode=mode, x=round(P["x"]), y=round(P["y"]), th=round(math.degrees(th_now()), 1),
                     v=round(st["cmd"][0], 2), v_est=round(st["v_est"], 2), steer=round(st["cmd"][1], 1),
                     prog=round(st.get("prog", 0.0), 2), shield=shield.act, icp=st["n_icp"]))
        time.sleep(1.0 / p["loop_hz"])

    def wait(mode, secs, v=0.0, s=0.0):
        t = time.monotonic()
        while time.monotonic() - t < secs:
            go(v, s)
            tick(mode)

    def run_legs(mode, legs, yaw_lot0):
        """park.py's legs driven on MEASURED quantities: a turning leg ends on the gyro's absolute heading (yaw_lot0 +
        the leg's end heading), a straight leg on the lidar pose in the lot frame; the car stops a coast short
        (v x leg_coast_s), stands leg_settle_s, re-measures and creeps the rest at v_leg_slow (at most leg_tries).
        The wheels swing only standing, the previous lock held while the car stops (park.Executor's rule).  The
        profile's step pulses are not used: the fitted plant's drive lag (tau 0.56 s) moves a 30-60 ms kick ~1 mm."""
        lock = float(p["park_lock_deg"])
        lot_obs = K.obstacles(lot_len)
        out = []
        prev = 0.0
        bot.mode = "park"

        def rem(lg):
            if lg["steer"]:
                want = yaw_lot0 + dirn_[0] * math.degrees(lg["end"][2])
                togo = wrapd(want - robot.yaw)
                sense = 1.0 if dirn_[0] * lg["dth_rad"] > 0 else -1.0
                r_leg = (rad[0] if lg["steer"] > 0 else rad[1]) if rad[0] else car.r_min    # v12: per side
                return math.radians(togo * sense) * r_leg
            lp_ = K.to_lot((P["x"], P["y"], th_now()), lot["x0"], dirn_[0])
            ex_, ey_, eth_ = lg["end"]
            return ((ex_ - lp_[0]) * math.cos(eth_) + (ey_ - lp_[1]) * math.sin(eth_)) * lg["dir"]

        try:
            for i, lg in enumerate(legs):
                steer = lg["steer"] * dirn_[0] * lock
                if rad[0]:                                  # v12: the field side's own true lock (30 left / 22 right)
                    steer = lg["steer"] * dirn_[0] * (float(p["park_lock_left"]) if lg["steer"] * dirn_[0] > 0
                                                      else float(p["park_lock_right"]))
                t_ = time.monotonic()
                while time.monotonic() - t_ < float(p["leg_stop_s"]):
                    bot.drive(0.0, prev)
                    tick(mode)
                t_ = time.monotonic()
                t_servo = abs(steer - prev) / max(float(p["servo_dps"]), 1.0) + float(p["servo_extra_s"])
                while time.monotonic() - t_ < t_servo:
                    bot.drive(0.0, steer)
                    tick(mode)
                t_leg = time.monotonic()
                r = rem(lg)
                n_try, blocked, clear_stops = 0, False, 0
                while n_try < int(p["leg_tries"]) and r > float(p["leg_tol_mm"]) and not blocked and not clear_stops:
                    # first run at v_leg, stopped leg_slow_mm short (the drive only coasts down from a lower
                    # command: tau 0.56 s); then creeps at v_leg_slow, stopped a coast short (a straight leg's pose is
                    # up to a scan old: leg_coast_line_s)
                    fast = n_try == 0 and r > 2.0 * float(p["leg_slow_mm"])
                    v_fast = float(p["v_leg_exit"]) if mode == "EXIT" else float(p["v_leg"])   # v4: the exit only
                    v = v_fast if fast else float(p["v_leg_slow"])
                    stop_at = float(p["leg_slow_mm"]) if fast else \
                        v * float(p["leg_coast_s"] if lg["steer"] else p["leg_coast_line_s"]) * 1000.0
                    clr_prev = None
                    while True:
                        r = rem(lg)
                        if r <= stop_at or time.monotonic() - t_leg > float(p["leg_timeout"]):
                            break
                        # the lidar pose in the lot: a leg that closes on a limitation or the wall nearer than
                        # leg_clear_stop ends there (the next leg ends on its absolute heading and takes the rest)
                        lp_c = K.to_lot((P["x"], P["y"], th_now()), lot["x0"], dirn_[0])
                        clr = K.clearance(car, lp_c, lot_obs)
                        if clr_prev is not None and clr < clr_prev - 0.5 and clr < float(p["leg_clear_stop"]):
                            clear_stops += 1
                            break
                        clr_prev = clr
                        bot.drive(lg["dir"] * v, steer)
                        tick(mode)
                        if getattr(bot, "last_act", None) == "brake":
                            blocked = True
                            break
                    t_ = time.monotonic()
                    while time.monotonic() - t_ < float(p["leg_settle_s"]):
                        bot.drive(0.0, steer)
                        tick(mode)
                    r = rem(lg)
                    n_try += 1
                    if time.monotonic() - t_leg > float(p["leg_timeout"]):
                        break
                out.append(dict(leg=i, want=lg["dist_mm"], rem=round(r, 1), tries=n_try, blocked=blocked,
                                clear_stop=clear_stops, s=round(time.monotonic() - t_leg, 2)))
                prev = steer
        finally:
            bot.mode = "drive"
        bot.drive(0.0, prev)
        return out

    def lot_faces():
        """The lot's limitation faces the lidar sees now: field x of each group of returns in the lot band (> 25 mm
        off the outer wall, below the limitations' free end), <= 40 mm across."""
        sc = robot.scan
        if sc is None:
            return []
        a = np.radians(sc.angles())
        d = sc.dist.astype(float)
        m = (d > 60.0) & (d < 1600.0)
        X, Y = lidar_x + d[m] * np.cos(a[m]), lidar_y + d[m] * np.sin(a[m])
        xs_, ys_, ths_ = pose_at(sc.t - lag_scan)
        c, s_ = math.cos(ths_), math.sin(ths_)
        fx, fy = xs_ + c * X - s_ * Y, ys_ + s_ * X + c * Y
        q = np.flatnonzero((fy > -1475.0) & (fy < -1310.0) & (np.abs(fx) < 1400.0))
        out = []
        if len(q) < 3:
            return out
        q = q[np.argsort(fx[q])]
        for grp in np.split(q, np.flatnonzero(np.diff(fx[q]) > 40.0) + 1):
            if len(grp) >= 3 and float(fx[grp].max() - fx[grp].min()) <= 40.0 and \
                    float(fy[grp].max() - fy[grp].min()) >= 40.0:
                out.append(float(np.median(fx[grp])))
        return out

    def x_search(span):
        """1-D search of the field x (the lot moves with it) against the walls: the scan now, the pose's y / th."""
        perc = LP.of(robot)
        if perc is None:
            return None
        Xw, Yw = perc.points(exclude=(LP.CLS_PILLAR, LP.CLS_CLUTTER))
        if len(Xw) < 10:
            return None
        base = wall_segments()
        c, s = math.cos(th_now()), math.sin(th_now())
        best = None
        for dx in np.arange(-span, span + 0.1, 10.0):
            px = P["x"] + dx + c * Xw - s * Yw
            py = P["y"] + s * Xw + c * Yw
            d, _r_, vert, _j = seg_assoc(px, py, base, (P["x"] + dx + c * lidar_x, P["y"] + s * lidar_x))
            cost = float(np.minimum(d, 60.0).sum())
            nv = int(((d < 40.0) & vert).sum())
            if best is None or cost < best[0]:
                best = (cost, float(dx), nv)
        return best

    arcs_log = []
    parked = None
    try:
        log(dict(ev="start", params=p, wb_slow=wb_slow, wb_by=wb_by, cap=cap, cap_l=cap_l, cap_r=cap_r, trim=trim, car=[rear, front, half], lot_len=lot_len,
                 shield=shield.enabled, shield_decel=round(shield.decel, 3), lidar_x=lidar_x, cam_lat=lat_cam))
        # ------------------------------------------------------------------ START: still scans
        wait("START", 0.3)
        ws = []
        isl = []
        seen = st["scan_t"]
        t_s = time.monotonic()
        while len(ws) < 4 and time.monotonic() - t_s < 1.5:
            go(0.0, 0.0)
            tick("START")
            if st["scan_t"] != seen:
                seen = st["scan_t"]
                perc = LP.of(robot)
                if perc is None:
                    continue
                X, Y = perc.X.astype(float), perc.Y.astype(float)
                near = (X > lidar_x - 40.0) & (X < lidar_x + 60.0)       # beside the car, short of the front limitation
                nr = int((near & (Y < -35.0) & (Y > -180.0)).sum())
                nl = int((near & (Y > 35.0) & (Y < 180.0)).sum())
                ws.append((nr, nl, float(np.median(-Y[near & (Y < -35.0) & (Y > -180.0)])) if nr else float("nan"),
                           float(np.median(Y[near & (Y > 35.0) & (Y < 180.0)])) if nl else float("nan")))
                isl.append((X, Y))
        nr = sum(w[0] for w in ws)
        nl = sum(w[1] for w in ws)
        if direction[0] == 0:
            direction[0] = 1 if nr >= nl else -1
        dirn = direction[0]
        side = [w[2] if dirn > 0 else w[3] for w in ws]
        side = [v for v in side if v == v]
        dw = float(np.median(side)) if side else K.LOT_DEPTH / 2.0
        lot_like = bool(int(p["lot_start"])) and bool(side) and dw < 180.0
        # the heading from the island face (the long wall on the field side): a line through its points
        psi = 0.0
        pts_i = []
        for X, Y in isl:
            # the island face (lot start: ~900 mm off, corridor start: ~500), short of its end: past it the rays meet
            # the next corridor's wall across the view and tilt the fit (SIM: -7.6 deg for a true +5.3)
            # (from the lot only beyond 700 mm: the start straight's signs stand 500 mm off a car in the lot)
            m = (X > lidar_x + 40.0) & (X < lidar_x + 700.0) & (dirn * Y > (700.0 if lot_like else 300.0)) &                 (dirn * Y < 1200.0)
            if int(m.sum()) >= 8:
                k_, b_ = np.polyfit(X[m], Y[m], 1)
                r_ = np.abs(Y[m] - (k_ * X[m] + b_))
                keep = r_ < max(15.0, 2.0 * float(np.median(r_)))
                if int(keep.sum()) >= 8:
                    k_, b_ = np.polyfit(X[m][keep], Y[m][keep], 1)
                    if float(np.median(np.abs(Y[m][keep] - (k_ * X[m][keep] + b_)))) <= 10.0:
                        pts_i.append(-math.atan(k_))
        if pts_i:
            psi = float(np.median(pts_i))
            if abs(psi) > math.radians(8.0):
                psi = 0.0
        th_nom = 0.0 if dirn > 0 else math.pi
        P["th_off"] = th_nom + psi - math.radians(robot.yaw)
        if lot_like:
            P["y"] = -1500.0 + dw
        else:
            # a start in the corridor (no lot): y from the outer wall on the travel's outer side (dir, default ccw)
            X, Y = isl[-1] if isl else (np.zeros(0), np.zeros(0))
            m = (X > lidar_x - 60.0) & (X < lidar_x + 300.0) & (-dirn * Y > 60.0) & (-dirn * Y < 1300.0)
            if int(m.sum()) >= 4:
                P["y"] = -1500.0 + float(np.median(-dirn * Y[m]))
        log(dict(ev="start_view", dir="ccw" if dirn > 0 else "cw", near_right=nr, near_left=nl, wall_mm=round(dw, 1),
                 psi=round(math.degrees(psi), 2), lot=lot_like))
        planner[0] = Planner(dirn, dict(p, car_rear=rear, car_front=front, car_half=half))
        exit_legs = park_legs = None
        Q_lot = None
        rad[0] = rad[1] = None
        if lot_like or int(p["park"]):
            try:
                t_pl = time.monotonic()
                if int(p["park_asym"]):
                    # v12: lot-frame steer +1 = toward the field: LEFT on a ccw round (the 30-deg side), right on cw
                    rl_ = wb_slow / math.tan(math.radians(float(p["park_lock_left"])))
                    rr_ = wb_slow / math.tan(math.radians(float(p["park_lock_right"])))
                    rad[0], rad[1] = (rl_, rr_) if dirn > 0 else (rr_, rl_)
                    park_legs = plan12_cached(car, lot_len, float(p["park_margin"]), float(p["q_lane_mm"]),
                                              rad[0], rad[1])
                    exit_legs = park_legs
                else:
                    park_legs = plan_cached(car, lot_len, float(p["park_margin"]), float(p["q_lane_mm"]))
                    exit_legs = park_legs if float(p["exit_margin"]) == float(p["park_margin"]) else \
                        plan_cached(car, lot_len, float(p["exit_margin"]), float(p["q_lane_mm"]))
                Q_lot = park_legs[-1]["end"]
                log(dict(ev="exit_plan", legs=len(exit_legs), park_legs=len(park_legs), R=round(car.r_min),
                         lock=p["park_lock_deg"], Q=[round(v, 1) for v in Q_lot],
                         ms=round((time.monotonic() - t_pl) * 1000)))
            except RuntimeError as e:
                log(dict(ev="exit_plan", error=str(e)))
                p["park"] = 0
                lot_like = False
        if lot_like:
            # ---------------------------------------------------------------- EXIT the lot
            pk = K.parked_pose(car, lot_len)
            # the field x is not observable from inside the lot (the front limitation hides the wall ahead): a nominal
            # x, the lot frame tied to it, and the ICP on the lot and the outer wall only until the car is out -- then
            # the 1-D search on the wall ahead moves the car and the lot together (x_fix)
            P["x"] = -120.0 * dirn
            x0 = P["x"] - pk[0] if dirn > 0 else P["x"] + pk[0]
            set_lot(x0)
            segs[0] = wall_segments(lot["rects"])[[0] + list(range(8, 8 + 4 * len(lot["rects"])))]  # outer + lot
            best = None
            wait("START", 0.4)                         # the ICP on the lot's faces settles the heading first
            yaw_lot0 = robot.yaw - math.degrees(wrap(th_now() - th_nom))
            log(dict(ev="lot_start", x=round(P["x"]), y=round(P["y"]), lot_x0=round(x0), x_search=best and
                     [round(best[0]), round(best[1]), best[2]]))
            # v13 exit_smooth: the S's last leg (forward, the other lock, back to heading 0 at Q) is NOT driven: the
            # car leaves the lot already turned toward the first corner (~62 deg) and the laps take it from there.
            # Mat 20261001-202156: that leg carried the car 360 mm east to Q, past the corner's turn-in, and
            # exit_back then reversed it 420 mm (~5 s for nothing).  The park still uses the whole plan (Q).
            ex_legs = exit_legs[:-1] if int(p["exit_smooth"]) and len(exit_legs) > 2 else exit_legs
            legs_log = run_legs("EXIT", ex_legs, yaw_lot0)
            # the legs ended on measured headings / the lot pose: the car is at the plan's end, Q, in the lot frame
            # (the odometry past the front limitation is not: the drive model under-reads a slow car by ~40 %)
            qx, qy, _qt = K.to_field(ex_legs[-1]["end"], lot["x0"], dirn)
            log(dict(ev="exit_done", legs=legs_log, x_odo=round(P["x"]), y_odo=round(P["y"]), x_plan=round(qx),
                     y_plan=round(qy), th=round(math.degrees(th_now()), 1), t=round(time.monotonic() - t0, 2)))
            P["x"], P["y"] = qx, qy
            hist.clear()
            wait("EXITED", 0.15)
            best = x_search(float(p["x_search"]))
            if best is not None and best[2] >= 6 and abs(best[1]) > 5.0:
                P["x"] += best[1]
                set_lot(lot["x0"] + best[1])
                hist.clear()
            segs[0] = wall_segments(lot["rects"])
            st["pose_ok"] = True
            wait("EXITED", 0.25)                       # a few scans on the fixed pose: the next corner's seats
            log(dict(ev="x_fix", x_search=best and [round(best[0]), round(best[1]), best[2]], x=round(P["x"]),
                     lot_x0=round(lot["x0"])))
        else:
            best = x_search(900.0)
            if best is not None:
                P["x"] += best[1]
            st["pose_ok"] = True
            if int(p["park"]):
                p["park"] = 0                          # no lot seen at the start: laps only (the park needs its x)
                log(dict(ev="no_lot", note="not parked in a lot at the start: laps only", x=round(P["x"]),
                         y=round(P["y"])))
        if lot["x0"] is not None:
            planner[0].extra_pts = list(lot["pts"])
            planner[0].rebuild()
        if lot_like and not int(p["exit_smooth"]):     # v13: no exit_back / exit_look (the glance reads the signs)
            a_car = FLD.to_section(0, P["x"], P["y"])[0] * dirn
            t1 = planner[0].t1.get(0, 500.0)
            need = a_car - (t1 - float(p["look_min"]) - 60.0)
            # the next straight's first signs: seen by the lidar but not yet read (the camera looks 45+ deg off them
            # from Q) -> the worst case, the island side, whose turn-in lies furthest back
            k_n, ai_n = (1, 0) if dirn > 0 else (3, 2)
            unread = []
            for j_, sj_ in seats.items():
                if FLD.SEATS[j_][0] == k_n and FLD.SEATS[j_][1] == ai_n and sj_["hits"] >= 2 and seat_colour(j_) is None:
                    need = max(need, float(p["exit_back_max"]))
                    unread.append(j_)
            if need > 20.0:
                back = min(need, float(p["exit_back_max"]))
                log(dict(ev="exit_back", a_car=round(a_car), t1=round(t1), back=round(back), unread=unread))
                a_goal = a_car - back
                for j_, sj_ in seats.items():
                    if FLD.SEATS[j_][0] == 0 and sj_["hits"] >= 2:
                        a_p = FLD.to_section(0, *seat_xy(j_))[0] * dirn
                        if a_goal - 25.0 - rear < a_p < a_car:
                            a_goal = max(a_goal, a_p + 25.0 + rear + 30.0)
                t_b = time.monotonic()
                # backing, the nose swings exit_look_deg toward the island: the camera (+-29 deg) then sees the next
                # straight's first signs (45+ deg off the lane from Q)
                h_s = (0.0 if dirn > 0 else math.pi) + (dirn * math.radians(float(p["exit_look_deg"])) if unread else 0.0)
                while time.monotonic() - t_b < 5.0:
                    a_now = FLD.to_section(0, P["x"], P["y"])[0] * dirn
                    if a_now - a_goal <= float(p["v_creep"]) * 0.15 * 1000.0:
                        break
                    # straight back, the heading held on S's direction (backing: the wheels toward the heading error)
                    e_h = math.degrees(wrap(th_now() - h_s))
                    go(-float(p["v_creep"]), max(-cap_r, min(cap_l, 1.5 * e_h)))
                    tick("EXIT_BACK")
                wait("EXIT_BACK", 0.2)
                t_l = time.monotonic()
                while unread and time.monotonic() - t_l < float(p["exit_look_s"]):
                    if all(seat_colour(j_) is not None for j_ in unread):
                        break
                    go(0.0, 0.0)
                    tick("LOOK")
                log(dict(ev="exit_look", read={str(j_): seat_colour(j_) for j_ in unread},
                         s=round(time.monotonic() - t_l, 2)))
        # ------------------------------------------------------------------ the laps
        pl = planner[0]
        Qf = K.to_field(Q_lot, lot["x0"], dirn) if (Q_lot is not None and lot["x0"] is not None) else None
        start_pr = FLD.progress(P["x"], P["y"], dirn)
        if lot_like:
            x_st = K.to_field(K.parked_pose(car, lot_len), lot["x0"], dirn)
            start_pr = FLD.progress(x_st[0], x_st[1], dirn)
        target = 4.0 * int(p["laps"])
        if int(p["park"]) and Qf is not None:
            target += ((FLD.progress(Qf[0], Qf[1], dirn) - start_pr + 2.0) % 4.0) - 2.0
        prog, last_pr = ((FLD.progress(P["x"], P["y"], dirn) - start_pr + 2.0) % 4.0) - 2.0, None
        st["prog"] = prog
        hint = pl.nearest(P["x"], P["y"])
        ver = -1
        prof = None
        holds, hold_t = 0, None
        relax_votes, relax_pending = {}, set()
        s_mid_set = False
        lap_logged = 0
        v_top = float(p["v1"])
        sx = 1.0 if dirn > 0 else -1.0                  # travel along +x (ccw) / -x (cw) in S

        def speed_profile(vt):
            path = pl.path
            n = len(path)
            k3 = 3
            nxt, prv = np.roll(path, -k3, axis=0), np.roll(path, k3, axis=0)
            hd = np.arctan2(nxt[:, 1] - prv[:, 1], nxt[:, 0] - prv[:, 0])
            dh = np.abs((np.roll(hd, -k3) - np.roll(hd, k3) + np.pi) % (2 * np.pi) - np.pi)
            seg = np.hypot(*np.diff(np.vstack([path, path[:1]]), axis=0).T)
            kap = dh / np.maximum(2 * k3 * np.mean(seg), 1.0)
            kap = np.convolve(np.concatenate([kap[-4:], kap, kap[:4]]), np.ones(9) / 9.0, mode="same")[4:-4]
            kap = np.maximum(kap, 1e-7)
            v_ay = np.sqrt(float(p["ay_plan"]) / (kap * 1000.0))
            tmax = math.tan(math.radians(capt() - float(p["reach_margin"])))
            vg = np.arange(0.15, 2.0, 0.05)
            wbg = np.interp(vg, wb_v, wb_mm)
            # the highest grid speed whose wheelbase still reaches the curvature (wb(v) kappa <= tan(lock))
            okm = (wbg[None, :] * kap[:, None]) <= tmax
            v_rc = np.where(okm.any(axis=1), np.max(np.where(okm, vg[None, :], 0.0), axis=1), float(p["v_min"]))
            v = np.minimum(np.minimum(v_ay, v_rc), vt)
            v = np.maximum(v, float(p["v_min"]))
            a = float(p["a_brake"])
            for _ in range(2):
                for i in range(n - 1, -1, -1):
                    j = (i + 1) % n
                    v[i] = min(v[i], math.sqrt(v[j] * v[j] + 2.0 * a * seg[i] / 1000.0))
            return v

        glance_s, glance_on = {}, [None, 0.0]

        def glance(x, y, th):
            """v7: the GLANCE.  A sign the lidar map holds (hits >= 2, not the lot's band) whose colour is unread,
            glance_min_mm..glance_max_mm from the lens and glance_fov_deg - glance_in .. glance_fov_deg + glance_deg
            off the camera's axis, turns the nose toward it: the pursuit's alpha is at least the bearing beyond
            glance_fov_deg - glance_in (capped glance_deg) toward it, until the sign sits that far inside the frame's
            edge -- the camera (and map_look) then read it in 2 frames.  (A bias ADDED to alpha lost to a path aiming
            the other way: SIM seed 1, the nose never got the sign inside the edge.)  At most glance_max_s per sign; the guard and the shield
            stay on every command.  Mat 2026-10-01 17:46 + SIM seed 1: the green after corner 2 stayed 30-60 deg off
            the axis on the straight before it (the car aimed right of it, for the red 1.2 on its right) and came in
            only ~250 mm away, beside the car -- every lap on the default lane.  Returns the alpha bias (rad) or None."""
            if not int(p["glance"]) or s_mid_set:
                return None                             # never in the park's approach (Q's 8 mm is load-bearing)
            c_, s_ = math.cos(th), math.sin(th)
            lx, ly = x + c_ * cam_xy[0] - s_ * cam_xy[1], y + s_ * cam_xy[0] + c_ * cam_xy[1]
            edge, inn, ext = float(p["glance_fov_deg"]), float(p["glance_in"]), float(p["glance_deg"])
            best = None
            for j, sj in seats.items():
                if sj["hits"] < 2 or seat_colour(j) is not None:
                    continue
                if glance_s.get(j, 0.0) >= float(p["glance_max_s"]):
                    continue
                sx_, sy_ = seat_xy(j)
                if max(abs(sx_), abs(sy_)) > FLD.HALF - float(p["wall_band_mm"]):
                    continue
                r_ = math.hypot(sx_ - lx, sy_ - ly)
                if not float(p["glance_min_mm"]) <= r_ <= float(p["glance_max_mm"]):
                    continue
                b = math.degrees(wrap(math.atan2(sy_ - ly, sx_ - lx) - th))
                if edge - inn < abs(b) <= edge + ext and (best is None or r_ < best[0]):
                    best = (r_, b, j)
            now_ = time.monotonic()
            if best is None:
                if glance_on[0] is not None:
                    log(dict(ev="glance_end", seat=glance_on[0], s=round(glance_s.get(glance_on[0], 0.0), 2),
                             colour=seat_colour(glance_on[0]), x=round(x), y=round(y)))
                glance_on[0] = None
                return None
            r_, b, j = best
            # never across a sign close ahead: turning left moves every sign toward the car's right, so a left glance
            # waits while a red (kept on the car's LEFT) or an unread sign sits within glance_block_x ahead and under
            # glance_block_y left of the axis; mirrored for green on a right glance (SIM seed 4 lot: the glance at
            # seat 13 put the red at (1100, 0), 526 mm ahead and 13 mm left, on the wrong side)
            for j2, sj2 in seats.items():
                if j2 == j or sj2["hits"] < 2:
                    continue
                X2, Y2 = LP.field_to_car([seat_xy(j2)[0]], [seat_xy(j2)[1]], (x, y, th))
                X2, Y2 = float(X2[0]), float(Y2[0])
                if not 0.0 < X2 < float(p["glance_block_x"]):
                    continue
                c2 = seat_colour(j2)
                if (b > 0 and c2 != "green" and Y2 < float(p["glance_block_y"])) or \
                        (b < 0 and c2 != "red" and Y2 > -float(p["glance_block_y"])):
                    best = None
                    break
            if best is None:
                if glance_on[0] is not None:
                    log(dict(ev="glance_end", seat=glance_on[0], s=round(glance_s.get(glance_on[0], 0.0), 2),
                             colour=seat_colour(glance_on[0]), x=round(x), y=round(y), blocked=True))
                glance_on[0] = None
                return None
            if glance_on[0] != j:
                glance_on[0], glance_on[1] = j, now_
                log(dict(ev="glance", seat=j, b=round(b, 1), r=round(r_), x=round(x), y=round(y),
                         th=round(math.degrees(th), 1)))
            glance_s[j] = glance_s.get(j, 0.0) + 1.0 / float(p["loop_hz"])
            return math.copysign(math.radians(min(abs(b) - (edge - inn), ext)), b)

        while True:
            tick("GO")
            pr = FLD.progress(P["x"], P["y"], dirn)
            if last_pr is not None:
                prog += ((pr - last_pr + 2.0) % 4.0) - 2.0
            last_pr = pr
            st["prog"] = prog
            lap = int(max(0.0, prog) // 4.0)
            st["lap"] = lap
            if lap > lap_logged:
                lap_logged = lap
                log(dict(ev="lap", n=lap, t=round(time.monotonic() - t0, 2), seats={"%d.%d" % k_: pl.colour(k_)
                                                                                    for k_ in sorted(pl.seat)
                                                                                    if pl.colour(k_)}))
            vt_now = float(p["v1"]) if lap < 1 else float(p["v"])
            if int(p["park"]) and Qf is not None and not s_mid_set and prog >= target - float(p["park_prep"]):
                s_mid_set = True
                pl.s_mid = float(Q_lot[1])
                pl.q_along = FLD.to_section(0, Qf[0], Qf[1])[0] * dirn
                pl.rebuild()
                log(dict(ev="park_prep", lane=round(pl.s_mid), prog=round(prog, 2)))
            if relax_pending:
                wh_ = FLD.where(P["x"], P["y"])
                if wh_[0] == "straight" and all(wh_[1] != k_[0] for k_ in relax_pending):
                    pl.relax.update(relax_pending)
                    log(dict(ev="slalom_relax", seats=["%d.%d" % k_ for k_ in sorted(relax_pending)],
                             x=round(P["x"]), y=round(P["y"])))
                    relax_pending.clear()
                    pl.rebuild()
            if pl.version != ver or vt_now != v_top:
                ver, v_top = pl.version, vt_now
                prof = speed_profile(v_top)
                hint = pl.nearest(P["x"], P["y"])
            x, y, th = P["x"], P["y"], th_now()
            v_now = max(abs(st["v_est"]), abs(st["cmd"][0]))
            look = max(float(p["look_min"]), v_now * float(p["look_t"]) * 1000.0)
            i, tgt, dpath = pl.target(x, y, hint, look)
            hint = i
            alpha = wrap(math.atan2(tgt[1] - y, tgt[0] - x) - th)
            # v13 q_line: the final approach rides Q's LINE straight (lot y = Q_y, the travel's heading) instead of the
            # lap path, which at Q's x already bends into the next corner (Q lies past its turn-in): every park today
            # reached Q 40-50 deg off and 200-340 mm aside and spent ~10 s in 3 realigns
            if int(p["q_line"]) and int(p["park"]) and Qf is not None and prog >= target - 1.0:
                h_line = 0.0 if sx > 0 else math.pi
                if abs(wrap(th - h_line)) < math.radians(float(p["q_line_deg"])) and y < -700.0:
                    alpha = wrap(math.atan2(Qf[1] - y, sx * float(p["q_look"])) - th)
                    Ld = math.hypot(float(p["q_look"]), Qf[1] - y)
                    if not st.get("q_line_on"):
                        st["q_line_on"] = True
                        log(dict(ev="q_line", x=round(x), y=round(y), th=round(math.degrees(th), 1),
                                 q=[round(Qf[0]), round(Qf[1])]))
            Ld = max(math.hypot(tgt[0] - x, tgt[1] - y), 1.0)
            v_pl = float(np.min(prof[(i + np.arange(0, 1 + int((float(p["lag_s"]) * v_now * 1000.0 + 60.0) / 20.0)))
                                     % len(prof)]))
            gl = glance(x, y, th)                       # v7: an unread map sign just past the camera's edge
            if gl is not None:
                alpha = max(alpha, gl) if gl > 0 else min(alpha, gl)   # at least that far toward it
                v_pl = min(v_pl, float(p["v_unknown"]))
            # an existing sign whose colour is not read yet, ahead: slow for the camera
            for j, sj in seats.items():
                if sj["hits"] >= 2 and seat_colour(j) is None:
                    X_, Y_ = LP.field_to_car([seat_xy(j)[0]], [seat_xy(j)[1]], (x, y, th))
                    if 0.0 < float(X_[0]) < float(p["unknown_ahead"]) and abs(float(Y_[0])) < 900.0:
                        v_pl = min(v_pl, float(p["v_unknown"]))
            # the park: brake to a stop at Q
            done = False
            if int(p["park"]) and Qf is not None and prog >= target - 1.0:
                if int(p["lot_refine"]) and st.get("new_scan"):
                    st["new_scan"] = False
                    for fx_ in lot_faces():
                        # the near face of the rear limitation (lot x -20) or of the front one (lot x lot_len): in
                        # the lot frame x grows with the travel (sx in the field)
                        cands = [fx_ + sx * K.LIMIT_T, fx_ - sx * lot_len]
                        bi = min(cands, key=lambda c_: abs(c_ - lot["x0"]))
                        if abs(bi - lot["x0"]) < 160.0:
                            lot.setdefault("obs", []).append(bi)
                    if len(lot.get("obs", [])) >= 3 and not lot.get("refined"):
                        x0n = float(np.median(lot["obs"][-15:]))
                        if abs(x0n - lot["x0"]) > 3.0:
                            log(dict(ev="lot_refine", was=round(lot["x0"], 1), now=round(x0n, 1), n=len(lot["obs"])))
                            set_lot(x0n)
                            Qf = K.to_field(Q_lot, lot["x0"], dirn)
                        if len(lot["obs"]) >= 12:
                            lot["refined"] = True
                rem = (Qf[0] - x) * sx
                # v13 q_decel: on Q's line the car arrives straight and faster than out of the corner; 0.6 m/s^2 with
                # the drive's lag (tau 0.56 s) overshot Q by 130 mm (SIM mat layout) -> brake on a gentler profile
                v_pl = min(v_pl, max(float(p["v_approach"]),
                                     math.sqrt(2.0 * float(p["q_decel"]) * max(0.0, rem) / 1000.0)))
                # v14 q_brake: the stop at Q from the REAL braking distance.  Mat 20261001-203331: the car came off
                # the last corner at 1.0-1.2 m/s, the falling command only let it coast (positive duty), it was still
                # at 0.76-0.93 m/s when the stop fired 50 mm before Q, and the PWM-low brake (~1.8 m/s^2) took 160:
                # Q overshot by 159 mm -> a full realign (~4.5 s).  Now: v^2 / (2 a) + v x latency, v the lidar speed
                # when fresh (the estimate under-read it ~15 %), a = q_brake_decel; the command 0 is the brake.
                v_b = abs(st["v_est"])
                if int(p["q_brake"]) and hasattr(robot, "speed") and robot.speed.fresh(time.monotonic()):
                    v_b = max(v_b, abs(float(robot.speed.v_meas)))
                lead = (v_b * v_b / (2.0 * float(p["q_brake_decel"])) + v_b * float(p["q_brake_lat"])) * 1000.0 \
                    if int(p["q_brake"]) else abs(st["v_est"]) * 60.0
                if rem <= lead + 5.0:
                    done = True
            elif prog >= target:
                done = True
            if done:
                break
            wbv = wb_of(max(0.2, v_pl))
            steer = math.degrees(math.atan(2.0 * wbv * math.sin(alpha) / Ld))
            if int(p["guard"]):
                pts = list(lot.get("pts", []))
                for j, sj in seats.items():
                    if sj["hits"] >= 2:
                        sx_, sy_ = seat_xy(j)
                        pts += [(sx_ + ox, sy_ + oy) for ox in (-25.0, 25.0) for oy in (-25.0, 25.0)]
                hz = float(p["guard_h0"]) + float(p["guard_ht"]) * v_now * 1000.0
                s2, c0, c1 = guard_steer(x, y, th, steer, pts, car, wbv, capt(), float(p["guard_mm"]), hz)
                if s2 != steer:
                    stats["guard"] += 1
                    steer = s2
            go(v_pl, steer)
            now = time.monotonic()
            hold_t = (hold_t or now) if (shield.enabled and shield.act == "brake" and v_pl > 0.0) else None
            stuck_long = bool(st["stuck"]) and now - st["stuck"] > 1.0
            if (hold_t and now - hold_t > p["hold_s"]) or stuck_long:
                holds += 1
                stats["holds"] = holds
                log(dict(ev="held", n=holds, x=round(x), y=round(y), shield=shield.act, stuck=bool(st["stuck"])))
                # a hold beside a tight sign pair: a vote to relax it; relax_holds votes (in any laps) relax it, and
                # only once the car is in another straight -- a path changed beside the signs turned the car into
                # the green one (SIM seed 1 cw, 5 contacts)
                for k_, xy_ in getattr(pl, "tight", {}).items():
                    if k_ not in pl.relax and math.hypot(xy_[0] - x, xy_[1] - y) < float(p["relax_mm"]):
                        relax_votes[k_] = relax_votes.get(k_, 0) + 1
                        if relax_votes[k_] >= int(p["relax_holds"]):
                            relax_pending.add(k_)
                        log(dict(ev="slalom_vote", seat="%d.%d" % k_, votes=relax_votes[k_], x=round(x), y=round(y)))
                if time.monotonic() - st.get("reopen_t", -99.0) > p["reopen_gap_s"] and hasattr(robot, "board_reopen"):
                    st["reopen_t"] = time.monotonic()
                    log(dict(ev="board_reset_on_hold", ok=bool(robot.board_reopen())))
                if holds > p["holds_max"]:
                    end["reason"] = "blocked"
                    raise Halt
                wait("BACK", float(p["back_mm"]) / 1000.0 / float(p["v_creep"]), -float(p["v_creep"]), -steer * 0.5)
                wait("BACK", 0.2)
                hold_t, st["stuck"] = None, None
                hint = pl.nearest(P["x"], P["y"])
        # ------------------------------------------------------------------ the finish / the park
        t_laps = round(time.monotonic() - t0, 2)
        log(dict(ev="laps_done", t=t_laps, prog=round(prog, 2), x=round(P["x"]), y=round(P["y"])))
        if not (int(p["park"]) and Qf is not None):
            wait("STOP", 0.5)
            end["reason"] = "laps"
        else:
            wait("STOP", 0.55 if int(p["q_brake"]) else 0.3)   # v14: 0.8 m/s braked at ~1.8 m/s^2 needs ~0.45 s

            def line_drive(sgn, x_goal, v, mode, t_max):
                """Along Q's line (lot frame y = Q_y, heading 0) to lot x = x_goal, forward (sgn 1) or backward (-1): pure
                pursuit on the line from the rear axle, look q_look ahead in the motion's direction; the lot frame is
                the field mirrored on a clockwise round, so the car's steering is dirn x the lot's."""
                t_c = time.monotonic()
                Lq = float(p["q_look"])
                bot.mode = "park"
                while time.monotonic() - t_c < t_max:
                    lq = K.to_lot((P["x"], P["y"], th_now()), lot["x0"], dirn)
                    if (x_goal - lq[0]) * sgn <= v * float(p["leg_coast_line_s"]) * 1000.0 + 3.0:
                        break
                    dxl, dyl = sgn * Lq, Q_lot[1] - lq[1]
                    c_, s_ = math.cos(lq[2]), math.sin(lq[2])
                    Xc, Yc = c_ * dxl + s_ * dyl, -s_ * dxl + c_ * dyl
                    if sgn > 0:
                        d_lot = math.atan(2.0 * wb_slow * math.sin(math.atan2(Yc, Xc)) / Lq)
                    else:
                        d_lot = math.atan(2.0 * wb_slow * math.sin(math.atan2(Yc, -Xc)) / Lq)
                    go(sgn * v, max(-cap_r, min(cap_l, math.degrees(d_lot) * dirn)))
                    tick(mode)
                bot.mode = "drive"
                wait(mode, 0.25)

            tries = 0
            while tries < int(p["q_tries"]):
                lp_ = K.to_lot((P["x"], P["y"], th_now()), lot["x0"], dirn)
                e_x, e_y, e_th = Q_lot[0] - lp_[0], lp_[1] - Q_lot[1], math.degrees(wrap(lp_[2]))
                if abs(e_y) <= float(p["q_tol_y"]) and abs(e_th) <= float(p["q_tol_deg"]) and \
                        abs(e_x) <= float(p["q_tol_mm"]):
                    break
                tries += 1
                log(dict(ev="q_align", n=tries, e_x=round(e_x), e_y=round(e_y), e_th=round(e_th, 1)))
                if abs(e_y) > float(p["q_tol_y"]) or abs(e_th) > float(p["q_tol_deg"]):
                    # off Q's line: back along it q_back_mm behind Q, then forward onto Q on the line
                    # v14: a stop only a little off the line backs q_back_short, not the whole q_back_mm
                    near_ = abs(e_y) <= float(p["q_near_y"]) and abs(e_th) <= float(p["q_near_deg"])
                    back_ = float(p["q_back_short"]) if near_ else float(p["q_back_mm"])
                    line_drive(-1, Q_lot[0] - back_, float(p["v_creep"]), "REALIGN", 6.0)
                    line_drive(1, Q_lot[0], float(p["v_creep"]), "REALIGN", 6.0)
                    if int(p["q_creep_after"]):
                        # v11: the forward realign stops leg_coast_line_s x v (53 mm at 0.2) before Q and the braked car
                        # stops there: every realign today ended 35-76 mm SHORT (12 q_aligns, 8 runs), and the CREEP
                        # below only ran when e_y and e_th were already in tolerance -- never -- so the park started
                        # 40-70 mm short (4 of 8 parks full).  Now the same try creeps x onto Q at v_leg_slow.
                        lp_ = K.to_lot((P["x"], P["y"], th_now()), lot["x0"], dirn)
                        e_x = Q_lot[0] - lp_[0]
                        if abs(e_x) > float(p["q_tol_mm"]):
                            line_drive(1 if e_x > 0 else -1, Q_lot[0], float(p["v_leg_slow"]), "CREEP", 3.0)
                else:
                    line_drive(1 if e_x > 0 else -1, Q_lot[0], float(p["v_leg_slow"]), "CREEP", 3.0)
            lp_ = K.to_lot((P["x"], P["y"], th_now()), lot["x0"], dirn)
            log(dict(ev="park_start", lot_pose=[round(lp_[0], 1), round(lp_[1], 1), round(math.degrees(lp_[2]), 1)],
                     Q=[round(v, 1) for v in Q_lot], tries=tries, t=round(time.monotonic() - t0, 2)))
            yaw_lot0 = robot.yaw - math.degrees(wrap(th_now() - th_nom))
            legs_log = run_legs("PARK", K.reverse(park_legs), yaw_lot0)
            wait("PARKED", 0.3)
            lp_ = K.to_lot((P["x"], P["y"], th_now()), lot["x0"], dirn)
            chk = K.lot_check(car, lp_, lot_len)
            parked = chk
            log(dict(ev="parked", legs=legs_log, check=chk, t=round(time.monotonic() - t0, 2)))
            end["reason"] = "parked" if chk["ok"] else "park_off"
    except Halt:
        pass
    finally:
        try:
            bot.drive(0.0, 0.0)
        except Exception:
            pass
        if duty_cfg is not None and v_ref0 is not None:
            duty_cfg["v_ref"] = v_ref0                  # v17: the robot's own v_ref back (the Open is not touched)
    secs = round(time.monotonic() - t0, 1)
    saver.close()
    rms = stats["icp_rms"]
    log(dict(ev="shield_stats", n_slow=shield.n_slow, n_steer=shield.n_steer, n_brake=shield.n_brake))
    log(dict(ev="obs_stats", icp=st["n_icp"], icp_fail=st["n_icp_fail"],
             icp_rms_med=round(float(np.median(rms)), 1) if rms else None, guard=stats["guard"], holds=stats["holds"],
             cam_frames=stats["cam_frames"], cam_votes=stats["cam_votes"], lidar_pillar_hits=stats["lidar_pill"],
             look_n=stats["look_n"], look_votes=stats["look_votes"], look_saved=saver.n,
             look_dir=getattr(saver, "dir", None),
             seats={"%d.%d" % k_: planner[0].colour(k_) for k_ in sorted(planner[0].seat)} if planner[0] else None,
             radii=planner[0].radii if planner[0] else None))
    laps = round(max(0.0, st.get("prog", 0.0)) / 4.0, 2)
    log(dict(ev="end", reason=end["reason"], laps=laps, seconds=secs, direction=direction[0], parked=parked))
    return dict(reason=end["reason"], laps=laps, seconds=secs, direction=direction[0], parked=parked)
