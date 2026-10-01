"""A simulated MentorPi A1 on a WRO FE field, with the SAME interfaces as the real Board / Lidar / Camera.

It exists so the dev agent, the UI, the CLI and every program can be run and tested on the laptop before the robot is
reachable (BW_MOCK=1).  It is NOT the judge of a driving law -- the mat is (team rule: the simulator is not the judge): it models a
kinematic bicycle (wheelbase 145 mm), a first-order speed lag, a 3000 x 3000 mm outer wall with a 1000 mm corridor, and
a 1-degree lidar with noise.  Enough to see a program steer, stop and log, not to tune one.

The WLtoys plant (params drive.backend = "gpio_pwm", scene "plant"): the drive is a DUTY on a sign-magnitude driver,
not a wheel speed -- static friction below the breakaway duty, a kinetic intercept (deadband), m/s per duty, a first-
order lag and a friction deceleration that both scale with the car's mass, braking on PWM low, the steering stopped at
the chassis' lock, and a ROLLOVER contact (the car stops for good) when the lateral acceleration at the CG passes
SSF x g.  The truth constants (WL_PLANT) differ from the params on purpose, as the mat will: the speed loop and the
calibration tests have something to find.

The HP60C depth (params depth.on): RenderDepth ray-casts the same scene from the same pose as the colour frame, then
adds the sensor the datasheet and the team's one field test describe (DEPTH_TRUTH; scene "depth_truth" overrides).
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque

import numpy as np

from . import lidar as L
from .rrc import BoardState

WALL_H, ROOM = 100.0, 2400.0                  # mm: the field's walls and signs are 100 tall; the room around it
OUTER, INNER = 1500.0, 500.0                 # mm: outer walls at +-1500, inner block at +-500 (1000 mm corridor)
WALLS = [
    ((-OUTER, -OUTER), (OUTER, -OUTER)), ((OUTER, -OUTER), (OUTER, OUTER)),
    ((OUTER, OUTER), (-OUTER, OUTER)), ((-OUTER, OUTER), (-OUTER, -OUTER)),
    ((-INNER, -INNER), (INNER, -INNER)), ((INNER, -INNER), (INNER, INNER)),
    ((INNER, INNER), (-INNER, INNER)), ((-INNER, INNER), (-INNER, -INNER)),
]
SEG = np.array([[a[0], a[1], b[0], b[1]] for a, b in WALLS], dtype=float)
G = 9.81


def MOCK_BUMP_G(age_s: float, v_mps: float = 0.25, tail: float = 0.3) -> float:
    """BW_MOCK_IMU_BUMP: the horizontal acceleration (g) `age_s` after a contact made at `v_mps` -- ONE 50 Hz sample
    at the peak (min(1.2, 3.0 x |v|) g: 0.22 m/s -> 0.66 g, 0.10 -> 0.30), a second at `tail` x the peak, then
    nothing.  [MAT 2026-09-23] contacts read 0.54-1.08 g; replayed at 20 Hz the mat's hits were "0.70 then 0.30",
    "0.54 then 0.14", "0.86 then 0.14 / 0.22" (review 2026-09-25): the second sample is at or under acc_bump_g about
    half the time, so the Sim draws `tail` in 0.15-0.45 per contact.  (Until 2026-09-25 every contact shook 1.0 g
    falling to 0.28 g over 80 ms whatever its speed: ~8 samples >= 0.28 g at the mock's then 100 Hz, so wro_next's
    two-sample rule always passed in the sim -- the push numbers it gave were circular.)"""
    if age_s < 0.0 or age_s >= 0.04:
        return 0.0
    peak = min(1.2, 3.0 * abs(float(v_mps)))
    return peak if age_s < 0.02 else peak * float(tail)


# The WLtoys build's truth (the sim's, not the params'): research/dynamics_wl.md 5.3-5.4 scaled from the Orange car to
# this car's mass, geometry from 3d/bluewave_body/wl/layout_wl.json.  A scene's "plant" overrides any of them.
WL_PLANT = dict(
    kind="wltoys",
    mass_g=726.3, cg_h_mm=55.5, cg_x_mm=60.1, track_mm=64.24, lock_deg=29.96,   # layout_wl.json
    v_ref=8.0,                    # the pack voltage the duty numbers hold at
    slope=11.42,                  # m/s per unit duty: 44.8 mm/s per 8-bit count [TEAM Orange, sim2/plant_cal.py]
    m_ref_g=360.0,                # the Orange car the numbers were measured on
    intercept_ref=7.7 / 255.0,    # kinetic intercept at m_ref [EST p0 7.7]
    breakaway_ref=25.0 / 255.0,   # from rest at m_ref [TEAM]
    load_share=0.5,               # share of the friction that grows with mass (dynamics_wl.md: bounded 0 .. 1)
    tau_ref_s=0.12,               # mechanical lag at m_ref [EST, the stock sim's 120 ms]; scales with mass
    a_fric_ref=2.9,               # coast deceleration at m_ref, m/s^2 [EST 5.4: 1.7-2.9 at 611 g]
    ay_cap_share=0.35,            # the design cap (x SSF g) dynamics_wl.md 3 -- counted, not enforced
    servo_dps=0.0,                # the steering servo's slew, deg/s (0 = instant, as every earlier run): dynamics_wl.md
    #                               6.4 [EST] ~0.1-0.15 s to lock = 200-300 deg/s; sim_run --plant '{"servo_dps": 250}'
)
# A car MEASURED on the mat brings its own truth in its profile's mock.plant (tools/profile_wltoys.py MAT_BW2 ->
# profiles/wltoys_bw2.json), never typed here; each key is optional and off when absent (plant_constants):
#   intercept, breakaway, slope   the duty_sweep line at this car's mass (replace the mass-scaled Orange estimates)
#   tau_s          s: the drive's speeding-up lag at this car's mass (replaces tau_ref_s x mass; unmeasured so far)
#   brake_tau_s    s: the braked stop the mat saw, as the shorted motor brakes -- dv/dt = -v / tau (a stop of v x tau);
#                  slowing to a LOWER duty's speed follows the same time constant (sign-magnitude, brake on PWM low)
#   brake_decel    m/s^2: the same stop as a constant deceleration instead (d = v^2 / 2a: longer above the measured
#                  speed) -- the pessimistic extrapolation, for an A/B: --plant '{"brake_tau_s": 0, "brake_decel": 0.24}'
#   lidar_cap_mm   the scan plane passes over the 100 mm walls beyond this range: those rays read the room (room_mm)
#   start_jolt_g   the stick-slip start: the IMU jolts this much for one or two 50 Hz samples as the car breaks loose
#                  from rest (with BW_MOCK_IMU_BUMP, as the contacts' bumps)
#   vbat           the pack's voltage (V) at the start (the mat's 7.0-8.2 V band; --fault battery_sag moves it)
# Beyond the mat's envelope (the fast drive: duty above the measured line, a_y and braking near the tyres' limit):
#   duty_meas_max  the top of the measured duty line (effective duty at v_ref); above it the line is extrapolated at
#   slope_hi_share x slope [EST] -- the drive's losses grow with speed
#   v_emf_cap      m/s: no duty drives faster (the back-EMF of the 130 at the pack's voltage) [EST]
#   mu             tyre-mat friction [EST, unmeasured; dynamics_wl.md 3: 0.5 worst]: the drive's acceleration and
#                  braking are capped at mu g, and the lateral acceleration at what the friction circle leaves --
#                  asked for more, the car SLIDES (it turns less than the steering says: a wider arc, the gyro reads
#                  the real yaw), counted in World.slide_s / slides.  Off (0) when absent.  The tip-over (a_y > SSF g,
#                  'rollover') is checked on the a_y the tyres really give
# The mat's calibration (tools/fit_plant.py -> profiles/plant_mat1001.json, sim_run --mat1001); each off when absent:
#   steer_bias_deg the wheels' true angle = the servo angle sent + this (the mat 2026-10-01: straight at servo -4, so
#                  +4): a program that sends no trim drifts left, and its right turns are 4 deg short
#   wb_curve       [[v m/s, mm], ...]: R x tan(true angle) against |v|, piecewise linear, flat outside -- the
#                  understeer (137 mm at 0.2 m/s, ~220 at 0.8-1.0, ~275 at 1.5) in place of the params' wheelbase
#   vbat_exp       the duty acts as duty x (V_bat / v_ref)^vbat_exp (1 = the old law; the mat fit 0: the drive's
#                  bat_comp over-corrects, the same command ran faster on a low pack)
#   (brake_decel with brake_tau_s 0 is the PWM-low brake the mat measured, >= 1.6 m/s^2; the coast stays a_fric)


# The HP60C depth's truth (scene "depth_truth" overrides any of them): what RenderDepth adds to the exact ray-cast depth.
DEPTH_TRUTH = dict(
    min_mm=200.0, max_mm=4000.0,  # [3RD HP60C DS V1.5] range 0.2-4 m: nothing outside it
    sig_a=0.5, sig_b=2.0e-6,      # range noise sigma = a + b z^2: [3RD] < 2 mm @ 1 m, structured light ~ z^2 (8 @ 2 m)
    scale=0.0,                    # a multiplicative error: 0.14 = the one pillar measured 2026-09-22 (1.25 m read 1.43)
    dark_valid=0.3,               # [TEAM 2026-09-22] the black walls: "most missed" -> a return in 30 % of blob_px blobs
    col_valid=0.95,               # [TEAM] the pillars (and the magenta limitations): seen
    floor_valid=0.0,              # [TEAM] the white mat: no return
    edge_invalid=0.5,             # the structured-light shadow: the far side of a depth step loses this share of pixels
    blob_px=16,                   # the size of a black-wall dropout blob, pixels
    step=4,                       # rendered every 4th pixel both ways and repeated (the consumer samples every 4th column)
)


def plant_constants(pl: dict) -> dict:
    """Mass-scaled constants: the friction share `load_share` grows with mass (intercept, breakaway, friction force),
    the lag grows with mass, the friction DECELERATION = force / mass (a heavier car coasts further)."""
    m = float(pl["mass_g"]) / float(pl["m_ref_g"])
    f = (1.0 - float(pl["load_share"])) + float(pl["load_share"]) * m
    ssf = float(pl["track_mm"]) / (2.0 * float(pl["cg_h_mm"]))
    k = dict(intercept=float(pl["intercept_ref"]) * f, breakaway=float(pl["breakaway_ref"]) * f,
             tau=float(pl["tau_ref_s"]) * m, a_fric=float(pl["a_fric_ref"]) * f / m, slope=float(pl["slope"]),
             v_ref=float(pl["v_ref"]), lock=math.radians(float(pl["lock_deg"])), x_cg=float(pl["cg_x_mm"]) / 1000.0,
             ay_tip=ssf * G, ay_cap=float(pl.get("ay_cap_share", 0.35)) * ssf * G, ssf=ssf,
             servo_rate=math.radians(float(pl.get("servo_dps", 0.0) or 0.0)))
    for key in ("intercept", "breakaway"):                     # measured at this car's mass: no scaling
        if pl.get(key) is not None:
            k[key] = float(pl[key])
    if pl.get("tau_s"):                                        # the drive's lag at this car's mass (no scaling)
        k["tau"] = float(pl["tau_s"])
    k.update(brake_tau_s=float(pl.get("brake_tau_s") or 0.0), brake_decel=float(pl.get("brake_decel") or 0.0),
             lidar_cap_mm=float(pl.get("lidar_cap_mm") or 0.0),
             room_mm=float(pl.get("room_mm") or ROOM), start_jolt_g=float(pl.get("start_jolt_g") or 0.0),
             vbat=float(pl["vbat"]) if pl.get("vbat") else None,
             duty_meas_max=float(pl.get("duty_meas_max") or 0.0),
             slope_hi_share=float(pl.get("slope_hi_share") or 1.0),
             v_emf_cap=float(pl.get("v_emf_cap") or 0.0), mu=float(pl.get("mu") or 0.0),
             steer_bias=math.radians(float(pl.get("steer_bias_deg") or 0.0)),
             wb_curve=sorted((float(a), float(b)) for a, b in (pl.get("wb_curve") or [])) or None,
             vbat_exp=float(pl["vbat_exp"]) if pl.get("vbat_exp") is not None else 1.0)
    return k


def wb_at(v_mps: float, curve) -> float:
    """mm: the effective wheelbase (R x tan(true angle)) at |v| on a plant's wb_curve -- piecewise linear, flat outside."""
    v = abs(float(v_mps))
    if v <= curve[0][0]:
        return curve[0][1]
    for (v0, l0), (v1, l1) in zip(curve, curve[1:]):
        if v <= v1:
            return l0 + (l1 - l0) * (v - v0) / max(v1 - v0, 1e-9)
    return curve[-1][1]


def wl_vss(a: float, k: dict) -> float:
    """|steady speed| (m/s) for an effective duty a (at v_ref) above the intercept: the measured line up to its top
    (duty_meas_max), [EST] slope_hi_share of its slope above, never above the back-EMF cap (v_emf_cap)."""
    v = (a - k["intercept"]) * k["slope"]
    top = k.get("duty_meas_max") or 0.0
    if top > k["intercept"] and a > top:
        v = (top - k["intercept"]) * k["slope"] + (a - top) * k["slope"] * float(k.get("slope_hi_share") or 1.0)
    cap = k.get("v_emf_cap") or 0.0
    return min(v, cap) if cap > 0.0 else v


def wl_speed(v: float, duty: float, brake: bool, dt: float, k: dict, vbat: float) -> float:
    """One step of the WLtoys drive: m/s after dt for a signed duty (the driver's) at battery `vbat` -- never a change
    faster than the tyres' mu g (plant key mu)."""
    nv = _wl_speed(v, duty, brake, dt, k, vbat)
    mu = k.get("mu") or 0.0
    if mu > 0.0 and nv != 0.0:
        lim = mu * G * dt                                        # a wheel that would spin up or lock slides instead
        nv = max(v - lim, min(v + lim, nv))
    return nv


def _wl_speed(v: float, duty: float, brake: bool, dt: float, k: dict, vbat: float) -> float:
    e = k.get("vbat_exp", 1.0)
    d = duty * vbat / k["v_ref"] if e == 1.0 else duty * (vbat / k["v_ref"]) ** e
    a = abs(d)
    bt, bd = k.get("brake_tau_s") or 0.0, k.get("brake_decel") or 0.0
    if abs(v) < 0.003 and a < k["breakaway"]:
        return 0.0                                              # static friction holds it
    if a > k["intercept"]:
        v_ss = math.copysign(wl_vss(a, k), d)
        slowing = (v_ss - v) * v < 0.0
        if slowing and bt > 0.0:                                # a lower duty: the motor's own braking time constant
            return v + (v_ss - v) * min(1.0, dt / bt)           # (a REVERSE duty = plug braking, the same law)
        nv = v + (v_ss - v) * min(1.0, dt / k["tau"])
        if slowing and bd > 0.0:                                # ...or never harder than the measured brake
            nv = max(nv, v - bd * dt) if v > 0.0 else min(nv, v + bd * dt)
        return nv
    if bt > 0.0:                                                # the measured stop: the shorted motor, v / tau
        if abs(v) < 0.01:
            return 0.0                                          # the last crawl: static friction catches it
        dec = abs(v) / bt if brake else min(abs(v) / bt, k["a_fric"])
    elif bd > 0.0:                                              # the measured stop, as a constant deceleration
        dec = bd if brake else min(bd, k["a_fric"])
    else:
        dec = k["a_fric"] + (abs(v) / k["tau"] if brake else 0.0)   # braked: the shorted motor's back-EMF drags too
    return math.copysign(max(0.0, abs(v) - dec * dt), v)


class _MockPwm:
    how = "mock PWM"

    def __init__(self):
        self.duty = 0.0

    def set(self, duty):
        self.duty = min(1.0, max(0.0, float(duty)))

    def off(self):
        self.duty = 0.0

    def close(self):
        self.duty = 0.0


class _MockPins:
    def __init__(self, n):
        self.pins = list(range(n))
        self.levels = [0] * n

    def write(self, levels):
        self.levels = [int(v) for v in levels]

    def close(self):
        pass


class MockDriver:
    """The motor driver the real motor.PwmMotor writes in the simulator: a PWM channel + direction pin(s), decoded with
    the driver's truth table into (signed duty, braking) for the plant.  One pin = sign-magnitude (MD13S: PWM low =
    brake); two pins = IN1/IN2 (both high = brake, both low = coast)."""

    def __init__(self, n_pins: int = 1, forward_high: bool = True):
        self.pwm, self.pins, self.fwd_high = _MockPwm(), _MockPins(n_pins), forward_high

    def state(self):
        d, lv = self.pwm.duty, self.pins.levels
        if len(lv) == 1:
            sign = 1.0 if bool(lv[0]) == self.fwd_high else -1.0
            return sign * d, True
        a, b = bool(lv[0]), bool(lv[1])
        if a and b:
            return 0.0, True
        if not a and not b:
            return 0.0, False
        return (1.0 if (a and not b) == self.fwd_high else -1.0) * d, False


class World:
    """Pose in mm / radians; x right, y up; heading 0 = +x; (x, y) = the REAR AXLE centre (as loc.Localizer).
    The body is the MentorPi A1's 213 x 159 mm footprint [product page], 34 mm behind the rear axle.  Anything the body
    touches -- a wall, a pillar, a parking limitation -- stops the car and counts one contact.

    The scene (start pose, pillars, parking lot) comes from params["mock"] or the JSON file named by BW_MOCK_SCENE:
        {"start": [x, y, deg], "pillars": [[x, y, "red"|"green"], ...], "lot": [x_a, x_b] | null, "true_speed": 0.92}
    """

    BODY = (-34.0, 179.0, 79.5)                  # rear, front (from the rear axle), half width -- mm

    def __init__(self, wheelbase_mm: float = 145.0, wheel_d_m: float = 0.067, motor_signs: dict | None = None,
                 scene: dict | None = None):
        sc = dict(scene or {})
        x, y, deg = sc.get("start", [-300.0, -1000.0, 0.0])
        self.x, self.y, self.th = float(x), float(y), math.radians(float(deg))
        self.pillars = [(float(a), float(b), str(c)) for a, b, c in sc.get("pillars", [])]
        lot = sc.get("lot")
        self.limits = []                         # (cx, cy, half_x, half_y) boxes
        if lot:
            for xa in lot:
                self.limits.append((float(xa), -1500.0 + 100.0, 10.0, 100.0))
        self.true_speed = float(sc.get("true_speed", 1.0))
        if sc.get("body"):                                   # [rear, front, half width] from the rear axle, mm
            self.BODY = tuple(float(v) for v in sc["body"])
        # the island's outer faces (left, right, bottom, top); Open Challenge corridors are 600 or 1000 mm per side
        # [RULE], so the island moves: {"corridors": [S, E, N, W]} or an explicit {"island": [l, r, b, t]}
        if sc.get("corridors"):
            cS, cE, cN, cW = (float(v) for v in sc["corridors"])
            self.island = (-OUTER + cW, OUTER - cE, -OUTER + cS, OUTER - cN)
        else:
            self.island = tuple(float(v) for v in sc.get("island", (-INNER, INNER, -INNER, INNER)))
        l, r, b, t = self.island
        self.walls = [((-OUTER, -OUTER), (OUTER, -OUTER)), ((OUTER, -OUTER), (OUTER, OUTER)),
                      ((OUTER, OUTER), (-OUTER, OUTER)), ((-OUTER, OUTER), (-OUTER, -OUTER)),
                      ((l, b), (r, b)), ((r, b), (r, t)), ((r, t), (l, t)), ((l, t), (l, b))]
        self.seg = np.array([[a[0], a[1], c[0], c[1]] for a, c in self.walls], dtype=float)
        self.v = 0.0
        self.L = wheelbase_mm
        self.wheel_c = math.pi * wheel_d_m * 1000.0      # mm per wheel revolution
        self.signs = {int(k): float(v) for k, v in (motor_signs or {2: 1.0, 4: -1.0}).items()}
        self.cmd_rps = {}
        self.servo_us = 1500
        self.servo_center, self.us_per_deg = 1500, 2000 / 180
        self.yaw_rate_dps = 0.0
        self.contacts = 0
        self.hit_v = 0.0                         # m/s at the newest contact
        self.touching = ""
        self.odo_mm = 0.0
        self.hist = deque(maxlen=400)            # (t, x, y, th) for the camera's latency
        self.lock = threading.Lock()
        self.vbat = 7.6                          # V, what MockBoard reports
        self.driver, self.k = None, None         # the WLtoys plant: set_plant()
        self.rolled, self.ay_max, self.ay_over_cap_s = False, 0.0, 0.0
        self.ay_at, self.tight_fast_s = None, 0.0     # where ay_max happened (x, y, v mm/s, steer); S-1 time
        self.delta = 0.0                              # the WLtoys plant's wheel angle (rad) behind the servo command
        self.starts, self.start_sign, self.still_s = 0, 1, 0.0   # the WLtoys plant: breakaways from rest (start jolt)
        self.slide_s, self.slides, self.sliding, self.slide_at = 0.0, 0, False, None   # plant mu: the grip's limit
        self.ax_peak = 0.0                                        # the largest |acceleration| along the car, m/s^2

    def set_plant(self, driver: MockDriver, plant: dict):
        """Drive by DUTY through `driver` on the WLtoys plant instead of by wheel rps."""
        self.driver, self.k = driver, plant_constants(dict(WL_PLANT, **(plant or {})))
        if self.k.get("vbat"):
            self.vbat = self.k["vbat"]

    def step(self, dt: float):
        with self.lock:
            delta = math.radians((self.servo_us - self.servo_center) / self.us_per_deg)
            ax = 0.0                                                     # m/s^2 along the car (the WLtoys plant)
            if self.driver is not None:
                duty, brake = self.driver.state()
                k = self.k
                v0 = self.v
                self.v = 0.0 if self.rolled else wl_speed(self.v / 1000.0, duty, brake, dt, k, self.vbat) * 1000.0
                if dt > 0.0:
                    ax = (self.v - v0) / 1000.0 / dt
                    self.ax_peak = max(self.ax_peak, abs(ax))
                if abs(self.v) >= 3.0:                                   # breaks loose after standing (not a wall
                    if self.still_s >= 0.15:                             # push's per-step restart): a start jolt
                        self.starts, self.start_sign = self.starts + 1, (1 if self.v > 0 else -1)
                    self.still_s = 0.0
                else:
                    self.still_s += dt
                if k.get("steer_bias"):                                  # the wheels' straight-ahead is not the
                    delta += k["steer_bias"]                             # servo's (plant steer_bias_deg)
                delta = max(-k["lock"], min(k["lock"], delta))          # the chassis' own stop
                if k["servo_rate"] > 0.0:                                # the servo turns at its own rate
                    step = k["servo_rate"] * dt
                    delta = self.delta + max(-step, min(step, delta - self.delta))
                self.delta = delta
            else:
                fwd = [r * self.signs[m] for m, r in self.cmd_rps.items() if m in self.signs] or [0.0]
                target = (sum(fwd) / len(fwd)) * self.wheel_c * self.true_speed
                self.v += (target - self.v) * min(1.0, dt / 0.15)                 # ~150 ms speed lag
            wbc = (self.k or {}).get("wb_curve") if self.driver is not None else None
            L = wb_at(self.v / 1000.0, wbc) if wbc else self.L        # plant wb_curve: the understeer at speed
            w = self.v * math.tan(delta) / L
            mu = (self.k or {}).get("mu") or 0.0
            if self.driver is not None and mu > 0.0 and abs(w) > 1e-9 and not self.rolled:
                # the friction circle: the tyres give mu g in all; what the drive's acceleration or braking takes is
                # not there for the turn.  Asked for more, the yaw rate is what the grip gives (a wider arc)
                a_lat = math.sqrt(max(0.0, (mu * G) ** 2 - ax * ax))
                vm, x = abs(self.v) / 1000.0, self.k["x_cg"]
                ay_kin = w * w * math.hypot(vm / abs(w), x)
                if ay_kin > a_lat:
                    if a_lat <= 0.0:                                     # the drive takes all the grip: no turn
                        w_lim = 0.0
                    elif x > 1e-6:                                       # w^2 hypot(v / w, x) = a_lat, solved for w
                        w2 = (math.sqrt(vm ** 4 + 4.0 * x * x * a_lat * a_lat) - vm * vm) / (2.0 * x * x)
                        w_lim = math.sqrt(max(0.0, w2))
                    else:
                        w_lim = a_lat / max(vm, 1e-6)
                    w = math.copysign(w_lim, w)
                    self.slide_s += dt
                    if not self.sliding:
                        self.slides += 1
                        self.slide_at = (round(self.x), round(self.y), round(self.v), round(math.degrees(delta), 1))
                    self.sliding = True
                else:
                    self.sliding = False
            else:
                self.sliding = False
            self.yaw_rate_dps = math.degrees(w)
            if self.driver is not None and abs(w) > 1e-9 and not self.rolled:
                # lateral acceleration AT THE CG, on its own (wider) circle: w^2 * R_cg
                R = abs(self.v / w) / 1000.0
                ay = w * w * math.hypot(R, self.k["x_cg"])
                if ay > self.ay_max:
                    self.ay_max = ay
                    self.ay_at = (round(self.x), round(self.y), round(self.v), round(math.degrees(delta), 1))
                if ay > self.k["ay_cap"]:
                    self.ay_over_cap_s += dt
                if R < 0.25 and abs(self.v) > 300.0:
                    # dynamics_wl.md S-1: <= 0.30 m/s on a radius under 250 mm (full-lock dodges) -- counted, not enforced
                    self.tight_fast_s += dt
                if ay > self.k["ay_tip"]:
                    self.rolled, self.v, self.contacts, self.touching = True, 0.0, self.contacts + 1, "rollover"
            if self.rolled:
                self.yaw_rate_dps = 0.0
                self.hist.append((time.monotonic(), self.x, self.y, self.th))
                return
            nth = (self.th + w * dt + math.pi) % (2 * math.pi) - math.pi
            nx = self.x + self.v * math.cos(self.th + w * dt / 2) * dt
            ny = self.y + self.v * math.sin(self.th + w * dt / 2) * dt
            hit = self.hit(nx, ny, nth)
            if hit:
                if not self.touching:
                    self.contacts += 1
                    self.hit_v = abs(self.v) / 1000.0     # m/s at the moment of the contact (the IMU bump's size)
                self.touching, self.v = hit, 0.0
            else:
                self.odo_mm += math.hypot(nx - self.x, ny - self.y)
                self.x, self.y, self.th, self.touching = nx, ny, nth, ""
            self.hist.append((time.monotonic(), self.x, self.y, self.th))

    def corners(self, x, y, th):
        r, f, h = self.BODY
        c, s = math.cos(th), math.sin(th)
        return [(x + c * a - s * b, y + s * a + c * b) for a, b in ((r, -h), (f, -h), (f, h), (r, h))]

    def hit(self, x, y, th) -> str:
        cs = self.corners(x, y, th)
        if any(abs(px) > OUTER or abs(py) > OUTER for px, py in cs):
            return "outer wall"
        l, r_, b, t = self.island
        if any(l < px < r_ and b < py < t for px, py in cs):
            return "island"
        boxes = [(ix, iy, 0.0, 0.0, "island corner") for ix in (l, r_) for iy in (b, t)]
        boxes += [(px, py, 25.0, 25.0, "%s pillar" % col) for px, py, col in self.pillars]
        boxes += [(cx, cy, hx, hy, "parking limitation") for cx, cy, hx, hy in self.limits]
        r, f, h = self.BODY
        c, s = math.cos(th), math.sin(th)
        for cx, cy, hx, hy, name in boxes:
            for px, py in ((cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy), (cx, cy)):
                a = c * (px - x) + s * (py - y)
                b = -s * (px - x) + c * (py - y)
                if r <= a <= f and -h <= b <= h:
                    return name
            if hx or hy:
                if any(abs(px - cx) <= hx and abs(py - cy) <= hy for px, py in cs):
                    return name
        return ""

    def pose_at(self, t: float):
        """The pose `t` (monotonic) -- for a camera frame that left the sensor a latency ago."""
        with self.lock:
            h = list(self.hist)
        if not h:
            return self.x, self.y, self.th
        for rec in reversed(h):
            if rec[0] <= t:
                return rec[1], rec[2], rec[3]
        return h[0][1], h[0][2], h[0][3]

    @staticmethod
    def _clear(x, y, r=90.0) -> bool:
        if abs(x) > OUTER - r or abs(y) > OUTER - r:
            return False
        return not (abs(x) < INNER + r and abs(y) < INNER + r)

    def raycast(self, res: float = 1.0, pos_mm=(0.0, 0.0), room: float = 0.0) -> np.ndarray:
        """mm per bin, our lidar frame (bin k = -180 + (k + 0.5) * res, CCW positive), from a lidar mounted at
        pos_mm (car frame, rear-axle origin).  room > 0: a scan plane ABOVE the 100 mm walls (the stock LD19 at
        145.7 mm) sees none of the field -- only the room around it, a square +-room mm."""
        with self.lock:
            x, y, th = self.x, self.y, self.th
        x, y = x + pos_mm[0] * math.cos(th) - pos_mm[1] * math.sin(th), y + pos_mm[0] * math.sin(th) + pos_mm[1] * math.cos(th)
        n = int(360 / res)
        ang = th + np.radians(-180.0 + (np.arange(n) + 0.5) * res)
        dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]
        seg = self.seg
        extra = []                                     # a lidar at 50 mm also sees the 100 mm signs and limitations
        for bx, by, hx, hy in [(px, py, 25.0, 25.0) for px, py, _c in self.pillars] + list(self.limits):
            c4 = [(bx - hx, by - hy), (bx + hx, by - hy), (bx + hx, by + hy), (bx - hx, by + hy)]
            extra += [[*c4[i], *c4[(i + 1) % 4]] for i in range(4)]
        if room > 0.0:
            c4 = [(-room, -room), (room, -room), (room, room), (-room, room)]
            seg, extra = np.array([[*c4[i], *c4[(i + 1) % 4]] for i in range(4)], float), []
        if extra:
            seg = np.vstack([seg, np.array(extra, float)])
        x1, y1, x2, y2 = seg[:, 0], seg[:, 1], seg[:, 2], seg[:, 3]
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((x1 - x) * ey - (y1 - y) * ex) / den
            u = ((x1 - x) * dy - (y1 - y) * dx) / den
        t = np.where((den != 0) & (t > 0) & (u >= 0) & (u <= 1), t, np.inf)
        d = t.min(axis=1)
        d = d + np.random.normal(0, 6, n)
        return np.where(np.isfinite(d) & (d < 12000), d, 0).astype(np.int32)


class MockBoard:
    def __init__(self, world: World):
        self.w = world
        self.state = BoardState()
        self.state.battery_mv.value, self.state.battery_mv.t = int(round(world.vbat * 1000)), time.monotonic()
        self.log = deque(maxlen=50)

    def set_motors_rps(self, speeds: dict):
        with self.w.lock:
            self.w.cmd_rps = {int(k): float(v) for k, v in speeds.items()}

    def set_pwm_servo(self, servo_id: int, pulse_us: int, duration_s: float = 0.02):
        with self.w.lock:
            self.w.servo_us = int(pulse_us)

    def set_pwm_servo_offset(self, servo_id, offset):
        pass

    def buzzer(self, *a, **k):
        self.log.append(("buzzer", time.monotonic()))

    def led(self, *a, **k):
        pass

    def rgb(self, *a, **k):
        pass

    def press(self, key_id: int = 1, event: int = 0x20):
        """Simulate the start button (KEY_CLICK by default)."""
        self.state.keys.append((time.monotonic(), key_id, event))

    def close(self):
        pass


class MockLidar:
    """Raycasts in the car's TRUE frame, turns that into the RAW angles of a physically mounted lidar (the truth:
    BW_MOCK_LIDAR_TRUTH="offset_deg,cw", default = whatever params say), then back into our frame through
    ScanAssembler.to_ours with the PARAMS -- the real driver's path.  Wrong params therefore give a wrong scan here
    too, which is what auto_calib has to catch."""

    def __init__(self, world: World, res: float = 1.0, lidar_params: dict | None = None):
        import os
        self.w, self.res = world, res
        lp = lidar_params or {"cw": True, "offset_deg": 0.0}
        self.asm = L.ScanAssembler(res=res, cw=lp["cw"], offset_deg=lp["offset_deg"])
        self.pos = tuple(lp.get("pos_mm", (0.0, 0.0)))
        # the real stock LD19 (145.7 mm) scans over the 100 mm walls and signs: its simulated twin must too, or a
        # stock program fed lidar points would pass every sim run and fail on the mat
        self.room = ROOM if float(lp.get("h_mm", 0.0) or 0.0) > WALL_H else 0.0
        truth = os.environ.get("BW_MOCK_LIDAR_TRUTH")
        self.true_off, self.true_cw = ((float(truth.split(",")[0]), truth.split(",")[1].strip() in ("1", "true"))
                                       if truth else (lp["offset_deg"], lp["cw"]))
        self.latest = None
        self.scans = 0
        self.mute_until = 0.0                    # tools/sim_run.py --fault lidar_dropout: no new scan before this

    def tick(self):
        if time.monotonic() < self.mute_until:  # a dropout (a USB hiccup): the last scan stays, and ages
            return
        d = self.w.raycast(self.res, self.pos, self.room)                          # true car frame, CCW +
        k = self.w.k
        if k and k.get("lidar_cap_mm") and not self.room:
            # [MAT 2026-09-30] the WLtoys nose lidar's plane passes over the 100 mm walls beyond ~2.2 m: those rays
            # read the room instead (never nearer than the wall they passed over)
            far = d > k["lidar_cap_mm"]
            if far.any():
                d = np.where(far, np.maximum(self.w.raycast(self.res, self.pos, k["room_mm"]), d), d)
        a = -180.0 + (np.arange(len(d)) + 0.5) * self.res
        raw = (self.true_off + (-a if self.true_cw else a)) % 360.0      # what the physical lidar reports
        out = np.zeros(len(d), np.int32)
        for r, dd in zip(raw, d):
            k = int((self.asm.to_ours(r) + 180.0) / self.res) % len(d)
            out[k] = dd
        self.latest = L.Scan(time.monotonic(), self.res, out, np.full(len(d), 200, np.uint8), 600.0)
        self.scans += 1

    def close(self):
        pass


class RenderCamera:
    """Renders the field as the real camera sees it -- through vision.Ground, the SAME calibrated pinhole + pose model
    the robot uses, so a program's perception is exercised end to end: white mat, black 100 mm walls, 50 x 50 x 100
    pillars, magenta limitations, the pale orange / blue corner lines, the dark lab above the walls; sensor noise and a
    JPEG round trip like the robot's MJPEG stream; a frame shows the pose `latency_s` ago, at `fps`.

    Painter's algorithm: every vertical face is cut into <= 100 mm pieces and drawn far to near, after the floor."""

    MAT, WALLC, LAB = (192, 196, 196), (26, 24, 22), (62, 60, 58)
    COL = {"red": (45, 40, 190), "green": (55, 140, 40), "magenta": (185, 40, 190)}

    def __init__(self, world, cam: dict, fps: float = 15.0, latency_s: float = 0.12):
        from .vision import Ground
        self.w = world
        self.cam = dict(cam)
        self.g = Ground(cam)
        self.fps_v, self.latency = fps, latency_s
        self.frames = 0
        self._frame, self._t = None, 0.0
        self.faces = self._scene()
        self.ss = 4                                   # supersampling: cv2.fillPoly fills boundary pixels INCLUSIVELY
        #                                               (+0.5 px on every edge = walls ~0.7 row low at 1x); at 4x and
        #                                               area-averaged the error is 1/8 px
        self.floor_mask = self._floor_mask(self.g)
        self.depth = None                             # a RenderDepth (params depth.on): rendered from the same pose
        self._run = True
        self.errors, self.last_error = 0, ""          # a render that raised: counted and skipped, the thread lives on
        self._thread = threading.Thread(target=self._loop, name="render", daemon=True)
        self._thread.start()

    def _floor_mask(self, g):
        S = self.ss
        R = g.R
        u1 = (np.arange(g.W * S) + 0.5) / S - 0.5                 # 1x coordinate of every 4x column centre
        xp = (u1 - g.cx) / g.fx
        vh = g.cy + g.fy * (-(R[2, 0] * xp + R[2, 2]) / R[2, 1])  # horizon row per column (ray z = 0)
        v1 = (np.arange(g.H * S) + 0.5) / S - 0.5
        return v1[:, None] > vh[None, :]

    def set_camera(self, cam: dict):
        """The TRUE camera pose changes mid-run (tools/sim_run.py --fault pitch_jump: a bumped mount); what the program
        believes does not.  The render loop picks the new pose up at its next frame."""
        from .vision import Ground
        g = Ground(dict(cam))
        mask = self._floor_mask(g)
        self.cam, self.g, self.floor_mask = dict(cam), g, mask

    # ---------------------------------------------------------------- scene
    @staticmethod
    def _box_faces(cx, cy, hx, hy, h, colour):
        c = [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]
        out = [(np.array([[*c[i], 0.0], [*c[(i + 1) % 4], 0.0], [*c[(i + 1) % 4], h], [*c[i], h]]), colour)
               for i in range(4)]
        out.append((np.array([[*p, h] for p in c]), colour))
        return out

    def _scene(self):
        faces = []
        for (x1, y1), (x2, y2) in self.w.walls:
            n = max(1, int(math.hypot(x2 - x1, y2 - y1) / 100.0))
            for k in range(n):
                a, b = k / n, (k + 1) / n
                p = (x1 + (x2 - x1) * a, y1 + (y2 - y1) * a)
                q = (x1 + (x2 - x1) * b, y1 + (y2 - y1) * b)
                faces.append((np.array([[*p, 0.0], [*q, 0.0], [*q, 100.0], [*p, 100.0]]), self.WALLC))
        for px, py, col in self.w.pillars:
            faces += self._box_faces(px, py, 25.0, 25.0, 100.0, self.COL[col])
        for cx, cy, hx, hy in self.w.limits:
            faces += self._box_faces(cx, cy, hx, hy, 100.0, self.COL["magenta"])
        return faces

    def _floor_polys(self):
        """(polygon, colour) on z = 0: the mat, then the corner lines (20 mm, low-saturation like the real ones; BGR).
        ROTATIONALLY symmetric like the WRO mat: at every corner the BLUE line crosses the corridor a counter-clockwise
        car comes from and the ORANGE one the corridor it leaves, so orange is met first clockwise and blue first
        counter-clockwise at every corner (the programs' rule).  The old drawing gave both angles one colour at all
        four corners (mirror symmetry): a clockwise start met blue first and the camera read 'ccw' in 32 of 32 cw SIM
        runs (fix review 2026-09-30).  The steep (70 deg) line crosses the x-running corridors, the shallow (20 deg)
        one the y-running ones; the incoming ccw corridor is x-running where sx * sy < 0 (SE, NW)."""
        m = OUTER + 100.0
        polys = [(np.array([[-m, -m, 0], [m, -m, 0], [m, m, 0], [-m, m, 0]], float), self.MAT)]
        orange, blue = (150, 172, 196), (178, 170, 160)
        for sx in (-1, 1):
            for sy in (-1, 1):
                pair = ((20.0, orange), (70.0, blue)) if sx * sy < 0 else ((20.0, blue), (70.0, orange))
                for ang, colour in pair:
                    a = math.radians(ang)
                    d = np.array([math.cos(a) * sx, math.sin(a) * sy])
                    l, r_, b, t = self.w.island
                    p0 = np.array([r_ if sx > 0 else l, t if sy > 0 else b])
                    L = min((OUTER - abs(p0[0])) / max(abs(d[0]), 1e-6), (OUTER - abs(p0[1])) / max(abs(d[1]), 1e-6))
                    nrm = np.array([-d[1], d[0]]) * 10.0
                    q = [p0 + nrm, p0 + d * L + nrm, p0 + d * L - nrm, p0 - nrm]
                    polys.append((np.array([[*p, 0.0] for p in q]), colour))
        return polys

    # ---------------------------------------------------------------- render
    def _to_cam(self, Pw, x, y, th):
        c, s = math.cos(th), math.sin(th)
        dx, dy = Pw[:, 0] - x, Pw[:, 1] - y
        car = np.stack([c * dx + s * dy, -s * dx + c * dy, Pw[:, 2]])
        return self.g.R.T @ (car - self.g.pos[:, None])

    def _clip_project(self, pc, near=20.0):
        """Sutherland-Hodgman against z_cam >= near, then project.  None when nothing is left."""
        pts = pc.T
        out = []
        n = len(pts)
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            ina, inb = a[2] >= near, b[2] >= near
            if ina:
                out.append(a)
            if ina != inb:
                t = (near - a[2]) / (b[2] - a[2])
                out.append(a + t * (b - a))
        if len(out) < 3:
            return None
        o = np.array(out)
        u = self.g.cx + self.g.fx * o[:, 0] / o[:, 2]
        v = self.g.cy + self.g.fy * o[:, 1] / o[:, 2]
        return np.clip(np.stack([u, v], 1), -2e5, 2e5)     # clamping any tighter bends the polygon

    def render(self, x, y, th):
        import cv2
        g, S = self.g, self.ss
        img = np.empty((g.H * S, g.W * S, 3), np.uint8)
        img[:] = self.LAB
        img[: int(g.H * S * 0.45)] = (175, 165, 150)
        img[self.floor_mask] = self.MAT               # every floor ray ends on the mat (the car is inside the walls)
        sh = 2

        def fill(uv, colour):
            uv4 = S * (uv + 0.5) - 0.5                # 1x continuous coords -> 4x (pixel edges stay pixel edges)
            cv2.fillPoly(img, [np.round(uv4 * (1 << sh)).astype(np.int32)], colour, cv2.LINE_8, sh)

        for P, colour in self._floor_polys()[1:]:
            uv = self._clip_project(self._to_cam(P, x, y, th))
            if uv is not None:
                fill(uv, colour)
        items = []
        for P, colour in self.faces:
            pc = self._to_cam(P, x, y, th)
            if (pc[2] < 20.0).all():
                continue
            items.append((float(np.linalg.norm(pc.mean(1))), pc, colour))
        items.sort(key=lambda it: -it[0])
        for _d, pc, colour in items:
            uv = self._clip_project(pc)
            if uv is not None:
                fill(uv, colour)
        img = cv2.resize(img, (g.W, g.H), interpolation=cv2.INTER_AREA)
        img = cv2.GaussianBlur(img, (3, 3), 0.8)
        noise = np.random.default_rng().normal(0, 3.0, img.shape)
        img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)

    def _loop(self):
        period = 1.0 / self.fps_v
        while self._run:
            t0 = time.monotonic()
            x, y, th = self.w.pose_at(t0 - self.latency)
            dep = self.depth
            try:
                d = dep.render(x, y, th) if dep is not None else None
                f = self.render(x, y, th)
            except Exception as e:                    # a render error used to end the thread: the camera froze for
                self.errors += 1                      # good and the program read it as camera_dead (sim 2026-09-25)
                if self.errors <= 3:
                    import traceback
                    self.last_error = "%s at (%.1f, %.1f, %.4f)" % (e, x, y, th)
                    print("mock camera: render failed %s -- %s" % (self.last_error, traceback.format_exc()),
                          flush=True)
                time.sleep(0.02)
                continue
            if dep is not None:                   # registered and synchronous, like the driver's pair: exposure time
                dep.publish(d, t0 - self.latency)
            self._frame, self._t = f, t0          # stamped like the real grabber: capture = stamp - latency
            self.frames += 1
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def read(self):
        return self._frame, self._t

    def fps(self) -> float:
        """As camera.Camera.fps(): 0 once no frame came for 1 s (sim_run --fault camera_freeze)."""
        return self.fps_v if (self._t and time.monotonic() - self._t <= 1.0) else 0.0

    def age_s(self):
        return (time.monotonic() - self._t) if self._t else None

    def close(self):
        self._run = False


class RenderDepth:
    """The HP60C depth stream in the simulator: uint16 mm along the optical axis (0 = no return), from the SAME camera
    pose as RenderCamera's frame (the vendor driver registers depth to RGB) and published with it, so the program can
    run with depth on or off on the same scene.  Exact first, by ray cast -- every pixel block against the walls, the
    signs and the limitations (vertical faces and their 100 mm tops) and the mat -- then DEPTH_TRUTH's sensor: range
    noise, a scale error, no return outside the range, from the mat, from most of the black wall, at a depth step's far
    side.  read() -> (frame, exposure time) like camera.DepthCamera."""

    DARK, COL, FLOOR = 0, 1, 2

    def __init__(self, world, cam: dict, truth: dict | None = None):
        from .vision import Ground
        self.w, self.k = world, dict(DEPTH_TRUTH, **(truth or {}))
        self.g = g = Ground(cam)
        s = self.s = max(1, int(self.k["step"]))
        self.nu, self.nv = g.W // s, g.H // s
        uu, vv = np.meshgrid(np.arange(self.nu) * s + (s - 1) / 2.0, np.arange(self.nv) * s + (s - 1) / 2.0)
        self.ray = g.rays(uu, vv)                  # (3, n) car frame, unit camera z: the distance along one IS its depth
        segs, nrm, kind = [], [], []
        for (x1, y1), (x2, y2) in world.walls:     # black, seen from both sides (outer walls inside, island outside)
            segs.append((x1, y1, x2, y2))
            nrm.append((0.0, 0.0))
            kind.append(self.DARK)
        self.boxes = [(px, py, 25.0, 25.0) for px, py, _c in world.pillars] + [tuple(b) for b in world.limits]
        for bx, by, hx, hy in self.boxes:
            c4 = [(bx - hx, by - hy), (bx + hx, by - hy), (bx + hx, by + hy), (bx - hx, by + hy)]
            for i in range(4):
                (ax, ay), (cx, cy) = c4[i], c4[(i + 1) % 4]
                ln = math.hypot(cx - ax, cy - ay)
                segs.append((ax, ay, cx, cy))
                nrm.append(((cy - ay) / ln, -(cx - ax) / ln))       # outward (the corners run counter-clockwise)
                kind.append(self.COL)
        self.seg, self.nrm = np.array(segs, float), np.array(nrm, float)
        self.fk = np.array(kind, np.int8)
        self.bx = np.array(self.boxes, float).reshape(-1, 4)
        self.rng = np.random.default_rng()
        self._latest = (None, 0.0)
        self._times = deque(maxlen=30)
        self.frames = 0

    def exact(self, x, y, th):
        """(depth mm, kind) per rendered block, flat: inf / -1 where the ray leaves the field above the walls."""
        g = self.g
        c, s = math.cos(th), math.sin(th)
        ox, oy, oz = x + c * g.pos[0] - s * g.pos[1], y + s * g.pos[0] + c * g.pos[1], float(g.pos[2])
        rx, ry, rz = self.ray
        dx, dy = c * rx - s * ry, s * rx + c * ry
        n = len(rz)
        lam, kind = np.full(n, np.inf), np.full(n, -1, np.int8)
        S = self.seg
        axx, axy = c * g.zc[0] - s * g.zc[1], s * g.zc[0] + c * g.zc[1]            # the optical axis, level
        ahead = ((S[:, 0] - ox) * axx + (S[:, 1] - oy) * axy > 0) | ((S[:, 2] - ox) * axx + (S[:, 3] - oy) * axy > 0)
        mx, my = (S[:, 0] + S[:, 2]) / 2.0, (S[:, 1] + S[:, 3]) / 2.0
        facing = ((ox - mx) * self.nrm[:, 0] + (oy - my) * self.nrm[:, 1] > 0) | ~self.nrm.any(1)
        sel = ahead & facing
        if sel.any():
            F, K = S[sel], self.fk[sel]
            ex, ey = F[:, 2] - F[:, 0], F[:, 3] - F[:, 1]
            qx, qy = F[:, 0] - ox, F[:, 1] - oy
            den = dx[:, None] * ey[None, :] - dy[:, None] * ex[None, :]
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (qx * ey - qy * ex)[None, :] / den
                u = (qx[None, :] * dy[:, None] - qy[None, :] * dx[:, None]) / den
            z = oz + t * rz[:, None]
            t = np.where((den != 0) & (t > 0) & (u >= 0) & (u <= 1) & (z >= 0) & (z <= 100.0), t, np.inf)
            j = t.argmin(1)
            lam = t[np.arange(n), j]
            kind = np.where(np.isfinite(lam), K[j], -1).astype(np.int8)
        if len(self.bx) and oz > 100.0:
            with np.errstate(divide="ignore", invalid="ignore"):
                lt = np.where(rz < 0, (100.0 - oz) / rz, np.inf)
            px, py = ox + lt * dx, oy + lt * dy
            B = self.bx
            on = ((np.abs(px[:, None] - B[None, :, 0]) <= B[None, :, 2])
                  & (np.abs(py[:, None] - B[None, :, 1]) <= B[None, :, 3])).any(1) & (lt < lam)
            lam, kind = np.where(on, lt, lam), np.where(on, self.COL, kind).astype(np.int8)
        with np.errstate(divide="ignore", invalid="ignore"):
            lf = np.where(rz < 0, -oz / rz, np.inf)
        fl = lf < lam
        return np.where(fl, lf, lam), np.where(fl, self.FLOOR, kind).astype(np.int8)

    def render(self, x, y, th):
        k, rng, nv, nu = self.k, self.rng, self.nv, self.nu
        lam, kind = self.exact(x, y, th)
        L2, K2 = lam.reshape(nv, nu), kind.reshape(nv, nu)
        fin = np.isfinite(L2)
        b = max(1, int(k["blob_px"]) // self.s)
        blobs = rng.random((nv // b + 1, nu // b + 1)) < float(k["dark_valid"])
        dark_ok = np.repeat(np.repeat(blobs, b, 0), b, 1)[:nv, :nu]
        r = rng.random((nv, nu))
        ok = fin & np.where(K2 == self.DARK, dark_ok, np.where(K2 == self.COL, r < float(k["col_valid"]),
                                                               (K2 == self.FLOOR) & (r < float(k["floor_valid"]))))
        if float(k["edge_invalid"]) > 0:
            Lf = np.where(fin, L2, 1e5)
            dd = Lf[:, 1:] - Lf[:, :-1]
            far = np.zeros((nv, nu), bool)
            far[:, 1:] |= dd > 60.0                  # the right neighbour is the far side of the step
            far[:, :-1] |= dd < -60.0
            ok &= ~(far & (rng.random((nv, nu)) < float(k["edge_invalid"])))
        z = np.where(fin, L2, 0.0) * (1.0 + float(k["scale"]))
        z = z + rng.normal(0.0, 1.0, (nv, nu)) * (float(k["sig_a"]) + float(k["sig_b"]) * z * z)
        ok &= (z >= float(k["min_mm"])) & (z <= float(k["max_mm"]))
        out = np.where(ok, np.rint(z), 0.0).astype(np.uint16)
        if self.s > 1:
            out = np.repeat(np.repeat(out, self.s, 0), self.s, 1)
        return out

    def publish(self, d, t_exposure: float):
        self._latest = (d, t_exposure)
        self._times.append(time.monotonic())
        self.frames += 1

    def read(self):
        return self._latest

    def fps(self) -> float:
        ts = self._times
        return (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else 0.0

    def close(self):
        pass


class Sim:
    """Owns the world and steps it; feeds the mock IMU and lidar at their real rates (the RRC's 50 Hz / 10 Hz)."""

    def __init__(self, params: dict):
        import json
        import os
        s, d = params["steer"], params["drive"]
        scene = dict(params.get("mock") or {})
        if os.environ.get("BW_MOCK_SCENE"):
            with open(os.environ["BW_MOCK_SCENE"], encoding="utf-8") as f:
                scene.update(json.load(f))
        self.world = World(params["chassis"]["wheelbase_m"] * 1000.0, d["wheel_d_m"], d["motors"], scene)
        self.world.servo_center, self.world.us_per_deg = s["center_us"], s["us_per_deg"]
        self.driver = None
        if d.get("backend", "rrc") == "gpio_pwm":          # the WLtoys build: a duty-driven plant
            pw = d.get("pwm", {})
            self.driver = MockDriver(len(pw.get("dir_pins", [16])), bool(pw.get("dir_forward_high", True)))
            self.world.set_plant(self.driver, scene.get("plant"))
        self.board = MockBoard(self.world)
        self.lidar = MockLidar(self.world, params["lidar"]["res_deg"], params["lidar"])
        # the camera the renderer uses can differ from what the program believes (scene "camera_truth"): a bumped
        # mount on the real car is exactly that
        cam_truth = dict(params["camera"], **(scene.get("camera_truth") or {}))
        self.camera = RenderCamera(self.world, cam_truth, latency_s=float(scene.get("latency_s", 0.12)))
        self.depth = None                                   # the HP60C depth: only with params depth.on (scene
        if int((params.get("depth") or {}).get("on", 0)):  # "depth_truth" = its noise, mock.DEPTH_TRUTH)
            self.depth = RenderDepth(self.world, cam_truth, scene.get("depth_truth"))
            self.camera.depth = self.depth
        self._run = True
        threading.Thread(target=self._loop, name="sim", daemon=True).start()

    def _loop(self):
        # stepped by REAL elapsed time: Windows' sleep granularity is ~15 ms, and a fixed 5 ms step would run the world
        # slower than the wall clock the gyro is integrated against (measured: 95 deg integrated for 86 deg turned)
        import os
        # BW_MOCK_IMU_BUMP=1 (sim_run --runfile, the analyzer's tests): every new contact shakes the accelerometer as the
        # mat's contacts did -- one or two 50 Hz samples sized by the speed of the contact (MOCK_BUMP_G), plus a 0.02 g
        # noise floor.  The 20 Hz tel sees a hit only when a tel sample lands on it, as on the robot (the run file's
        # imu records carry every sample).  Off: the IMU reads exactly (0, 0, 1) as before
        bump_on = os.environ.get("BW_MOCK_IMU_BUMP", "0") == "1"
        bump_seen, bump_t0, bump_dir, bump_v, bump_tail = self.world.contacts, None, 0.0, 0.0, 0.3
        # the WLtoys plant's stick-slip start (mock.plant start_jolt_g, [MAT 2026-09-30] ~0.5 g): the same one-or-two
        # 50 Hz sample shape as a contact's bump, along the direction the car lurches
        k = self.world.k or {}
        jolt_g = float(k.get("start_jolt_g") or 0.0) if self.world.driver is not None else 0.0
        jolt_seen, jolt_t0, jolt_dir, jolt_tail = self.world.starts, None, 0.0, 0.3
        rng = np.random.default_rng()
        last, t_imu, t_lidar = time.monotonic(), 0.0, 0.0
        while self._run:
            time.sleep(0.004)
            now = time.monotonic()
            dt, last = min(now - last, 0.05), now
            self.world.step(dt)
            if now - t_imu >= 0.02:            # 50 Hz IMU as the RRC sends it (deg/s on gz), on a fixed cadence
                t_imu = max(t_imu + 0.02, now - 0.02)
                gz = self.world.yaw_rate_dps + np.random.normal(0, 0.15)
                st = self.board.state
                acc = (0.0, 0.0, 1.0)
                if bump_on:
                    if self.world.contacts > bump_seen:
                        bump_seen, bump_t0 = self.world.contacts, now
                        bump_dir = self.world.th + (math.pi if self.world.v >= 0.0 else 0.0)
                        bump_v, bump_tail = float(getattr(self.world, "hit_v", 0.25)), float(rng.uniform(0.15, 0.45))
                    g = MOCK_BUMP_G(now - bump_t0, bump_v, bump_tail) if bump_t0 is not None else 0.0
                    gx, gy = g * math.cos(bump_dir), g * math.sin(bump_dir)
                    if jolt_g > 0.0:
                        if self.world.starts > jolt_seen:
                            jolt_seen, jolt_t0 = self.world.starts, now
                            jolt_dir = self.world.th + (0.0 if self.world.start_sign > 0 else math.pi)
                            jolt_tail = float(rng.uniform(0.15, 0.45))
                        age = now - jolt_t0 if jolt_t0 is not None else 1.0
                        gj = jolt_g if 0.0 <= age < 0.02 else jolt_g * jolt_tail if age < 0.04 else 0.0
                        gx, gy = gx + gj * math.cos(jolt_dir), gy + gj * math.sin(jolt_dir)
                    nz = np.random.normal(0.0, 0.02, 3)
                    acc = (gx + nz[0], gy + nz[1], 1.0 + nz[2])
                per = now - st.imu.t if st.imu.t and 0.0 < now - st.imu.t < 0.2 else 0.02
                st.add_imu(now, (acc[0], acc[1], acc[2], 0.0, 0.0, gz), per)
            if now - t_lidar >= 0.1:           # 10 Hz lidar
                t_lidar = now
                self.lidar.tick()

    def close(self):
        self._run = False
        self.camera.close()
