"""Every tunable number in one JSON file on the robot (~/bluewave/params.json), with the defaults below.

Each default says where it comes from: [VENDOR] = Hiwonder's MentorPi-A1 code, [DAY1] = must be measured on the car
before it is trusted (tests.py measures it), [RULE] = WRO FE 2026.  The dev UI and `bw params` edit the file; a race
program reads it once at start.
"""
from __future__ import annotations

import copy
import json
import os

DEFAULTS = {
    "steer": {
        "servo_id": 3,              # [VENDOR] ackermann.py: PWM servo ID 3
        "center_us": 1500,          # [VENDOR] pulse at straight ahead; [DAY1] trim with tests.servo_center
        "us_per_deg": 2000 / 180,   # [VENDOR] pulse = 1500 + 2000*deg/180
        "max_deg": 29.0,            # [VENDOR] odom_publisher_node limit; docs say 0-36 usable; [DAY1] measure the stop
        "invert": False,            # [DAY1] True if +deg turns the wheels RIGHT
        "min_us": 0, "max_us": 0,   # pulse limits (0 = none): a micro servo stalls against its end stop [DAY1 E18]
    },
    "drive": {
        "backend": "rrc",           # "rrc": the RRC Lite closes the loop on its encoder motors (stock A1).  "gpio_pwm":
                                    #   one encoderless motor on a PWM + DIR driver on Pi 5 GPIO (the WLtoys build,
                                    #   bluewave/motor.py, profiles/wltoys.json); the RRC keeps servo, IMU, keys
        "rrc_motor": 1,             # 0 = never send a non-zero speed to the RRC motor ports: an encoderless motor on
                                    #   one runs away at full duty and does not stop (research/electronics_wl.md 2.1)
        "motors": {"2": 1.0, "4": -1.0},   # [VENDOR] stock A1: motors 2 and 4, mirrored.  After the rule-11.13 rework
                                           #          (one driving axle) this becomes the single motor's id and sign.
        "wheel_d_m": 0.067,         # [VENDOR] ackermann.py wheel diameter; [DAY1] roll 10 turns on the mat
        "max_mps": 1.0,             # safety cap for every command
        "rps_scale": 1.0,           # [DAY1] measured speed / commanded (tests.speed_run)
        "ay_max": 0.0,              # m/s^2 lateral-acceleration cap on every command (0 = off): |v| <= sqrt(ay R)
        "cg_x_m": 0.0,              # CG ahead of the rear axle, for ay_max (the CG runs on a wider circle)
        "v_tight": 0.0,             # m/s on an arc tighter than r_tight_mm, rising linearly to max_mps at r_free_mm
        "r_tight_mm": 250.0,        #   (0 = off): dynamics_wl.md S-1 on every command (hw.Robot.tight_cap); WLtoys 0.30
        "r_free_mm": 300.0,
        "stall_s": 0.0,             # gpio_pwm: duty above breakaway while the lidar speed stays < 20 mm/s this long ->
                                    #   motor off until the command stops or reverses (0 = off; a stalled 130 = 20-40 W)
        "stall_retry_s": 2.0,       #   ...or until this long has passed: then one more kicked try (0 = never)
        "stop_decel": 2.2,          # m/s^2 braked stop, d = v^2 / 2a -- the park's step mode stops early by it
                                    #   [EST dynamics_wl.md 5.4] [DAY1] tests.coast
        "pwm": {                    # gpio_pwm: the MD13S on the Pi (electronics_wl.md 2.3); bluewave/motor.py
            "gpio": 12, "channel": 0,           # pin 32 = RP1 PWM0 channel 0 (dtoverlay=pwm-2chan,pin=12,func=4,...)
            "chip_match": "1f00098000.pwm",     # [3RD] the RP1 PWM0 device path; `python -m bluewave.motor --probe`
            "gpio_chip_label": "rp1", "hw_pwm": 1, "lgpio_fallback": 1, "sysfs_root": "/sys",
            "freq_hz": 490.0,                   # [TEAM] the Uno's 490 Hz: the Orange's calibration still applies
            "dir_pins": [16],                   # pin 36; two pins = an IN1/IN2 driver (brake = both high)
            "dir_forward_high": True,           # [TEAM Orange] DIR high = forward; [DAY1] re-check on the chassis
            "dir_dwell_ms": 50,                 # duty 0 this long before a direction flip
            "stop_mode": "brake",               # "brake" | "coast" (the MD13S can only brake: PWM low = brake)
            "watchdog_ms": 200,                 # duty 0 when the control loop misses this long
            "guardian": 1, "guardian_silence_ms": 800,   # a child process zeroes the PWM when this one dies
            "max_duty": 0.5,                    # hard cap at the driver (the stock car does 22 km/h at 100 %)
        },
        "duty": {                   # gpio_pwm: commanded m/s -> duty (bluewave/speed.py DutyModel); duties at v_ref
            "breakaway": 0.137,     # from rest; [TEAM] Orange 25/255 at 360 g, [EST] 25-42/255 at ~611 g -> 35/255;
                                    #   [DAY1] tests.duty_sweep
            "deadband": 0.041,      # kinetic intercept (a rolling car stops below it): [EST] 7.7-13.1/255 -> 10.5/255
            "mps_per_duty": 11.4,   # [TEAM] 44.8 mm/s per 8-bit count (Orange v(30/255) = 998 mm/s)
            "kick": 0.157,          # from rest, kick_ms long: breakaway + 5 counts (dynamics_wl.md S-4)
            "kick_ms": 60, "rest_ms": 150,   # a command after rest_ms at zero starts with the kick
            "max": 0.35, "v_zero": 0.01,
            "v_ref": 8.0, "bat_comp": 1,     # duty x v_ref / V_bat (the RRC reports V_bat)
        },
        "speed_loop": {"on": 0, "kp": 0.03, "ki": 0.09, "i_max": 0.04},   # PI on the lidar-measured speed, duty
        #                                                                    units; on the gpio_pwm drive only
    },
    "chassis": {"wheelbase_m": 0.145,   # the EFFECTIVE (kinematic) wheelbase: the car turns on R = wheelbase / tan(steer).
                #                         Every arc reads it (pure pursuit, guard, shield, park legs, ay cap).  [VENDOR]
                #                         0.145 geometric; [MAT 2026-09-23] the stock A1 turned R ~510 mm at 29 deg =
                #                         0.28 effective; [DAY1] tests.turn_radius measures it with the lidar
                "wheelbase_geom_m": 0.145,   # the axle distance: drawing and TF only
                "track_m": 0.133},   # [VENDOR] ackermann.py:12; [DAY1] tape-measure
    "imu": {
        "gyro_axis": 5,             # index into the board's (ax, ay, az, gx, gy, gz); deg/s [VENDOR node: radians(gz)]
        "gyro_sign": 1.0,           # [DAY1] +1 if turning LEFT (CCW) reads positive
        "gyro_bias": 0.0,           # deg/s, [DAY1] tests.imu_bias at rest
    },
    "lidar": {
        "device": "/dev/ldlidar",   # [VENDOR] ms200_scan.launch.py; [DAY1] use the /dev/serial/by-id path
        "baud": 230400,             # [VENDOR]
        "res_deg": 1.0,
        "cw": True,                 # [DAY1] raw angles grow clockwise (tests.lidar_direction)
        "offset_deg": 0.0,          # [DAY1] raw angle of the car's nose (URDF hints at 180 on some mounts)
        "min_conf": 0,
        "lot_map": 0,               # 1 = the lot's two limitations join the lidar's map once the lot is found (wro_next):
        #                             inside the lot they hide the far walls from a scan plane below 100 mm
        "h_mm": 145.7,              # [MEAS 2026-09-22] scan plane above the floor: the stock LD19 scans ABOVE the 100 mm
        #                             walls (the WLtoys build: 50); TF, the 3D view, tests_cal.lidar_tilt
        "min_mm": 60.0,             # lidar_perc drops nearer returns (the car's own parts)
    },
    "perc": {                       # bluewave/lidar_perc.py (BRAIN_SPEC 4): a scan -> walls, signs, limitations, clearance
        "max_mm": 3000.0,           # perception range
        "break_deg": 10.0,          # adaptive breakpoint angle lambda (Borges-Aldon)
        "break_mm": 30.0,           # + 3 sigma noise term of the breakpoint distance
        "line_tol_mm": 20.0, "line_tol_pct": 1.0,   # split threshold = tol + pct % of the range
        "pillar_max_mm": 80.0,      # a sign cluster's width (+ one beam spacing)
        "pillar_gap_mm": 100.0,     # a sign stands this much in front of both neighbours
        "wall_band_mm": 45.0,       # with a pose: a return this close to a map wall is a wall
        "seat_snap_mm": 120.0,      # lidar sign -> seat association radius
    },
    "shield": {                     # bluewave/shield.py (BRAIN_SPEC 5.3): the lidar safety layer under every drive command
        "on": -1,                   # -1 = on iff lidar.loc = 1 (the scan plane below the walls); 0 off; 1 on
        "margin_mm": 40.0,          # footprint growth ahead / behind while driving (stopping margin)
        "side_mm": 15.0,            # ...to the sides: below every planned lateral clearance (signs 85, walls 45, the
        #                             final-lap lane beside the lot 30, the park 22), so a planned pass never trips it
        "park_margin_mm": 8.0,      # all round in APPROACH / EXIT / PARK and for every step pulse
        "stop_mm": 25.0,            # added to the braking distance
        "latency_s": 0.15,          # scan age + loop period, added to the braking distance
        "decel": -1.0,              # m/s^2; -1 = drive.stop_decel
        "horizon_mm": 900.0,        # the swept arc is checked this far
        "memory_s": 1.5,            # older scans fill the sector the body hides this long
        "v_floor": 0.12,            # the slowest "slow" before it steers away or brakes
        "brake_hold_s": 0.6,        # braked this long = blocked (the program backs off / relocalises)
        "virtual": 1,               # map walls, known signs and the lot fill the blind sector while the pose is confident
        "manual": -1,               # the console's joystick passes it too, in "manual" mode (slows / brakes, never
        #                             steers: hub.Hub._manual_filter, BRAIN_SPEC 8.7): -1 = on iff lidar.loc = 1, 0, 1
        "pulse_stop_mm": 0.0,       # a step pulse needs its travel + this free (shield.py: why not stop_mm)
        "pulse_margin_mm": 0.0,     # ...at this footprint growth (not park_margin_mm: shield.py, sim lot seed 2 cw)
        "park_stop_mm": 10.0,       # stop_mm in "park" mode: the plan's park_margin_mm is the park's own margin (a 25
        #                             mm stop braked a correct PARK leg in the sim: shield.py)
    },
    "map": {"res_mm": 20.0, "size": 512, "max_mm": 4000.0, "live_hz": 5.0},   # the console's occupancy grid (mapper)
    "cloud": {"voxel_mm": 25.0, "max_points": 200000, "stride": 8, "max_mm": 3000.0, "rgb": 0},   # the depth cloud
    "camera": {
        "device": 0, "width": 640, "height": 480, "fps": 30,        # [MEAS] the RGB arrives as MJPEG from the container
        "intrinsics": [570.169, 569.464, 325.870, 238.096],          # [MEAS] fx fy cx cy, ascamera camera_info
        "calib_size": [640, 480],
        "h_mm": 123.3, "pitch_deg": -5.58, "roll_deg": -1.34,        # [MEAS] floor fit 2026-09-22 (pitch - = up)
        "x_mm": 150.0, "y_mm": 0.0,                                  # [EST] lens ahead of the rear axle
        "latency_s": 0.12,                                           # [EST] exposure -> frame in the program
        "verified_t": 0.0,          # Unix time of the last camera verification (bw preflight pitch / --field, a calib
        #                             fit applied); older than the agent's start = the camera may have moved [MAT
        #                             2026-09-23: the mount moved ~10 deg in one day] -- preflight says NO for wro_next
    },
    "depth": {                      # the HP60C depth stream (bluewave/depth.py): ranges that do not depend on the camera
        #                             pitch.  Measure before use (tests.depth): on the team's field (2026-09-22) the white
        #                             mat gave no return, the black walls mostly none, one red pillar at ~1.25 m read 1.43 m
        "on": 0,                    # 1 = open the stream (bluewave/depth_bridge.py in the camera container, TCP) at agent
        #                             start; 0 = no socket, no thread, no cost.  Each use below needs it and is its own switch
        "walls": 0,                 # the localiser takes a wall column's depth range instead of its floor-model range
        "pillars": 0,               # a sign's position from its depth columns instead of its base row
        "start": 0,                 # the start search on depth wall points, then the camera pitch fitted at that pose
        "host": "127.0.0.1", "port": 8091,   # the bridge (the container is net=host)
        "registered": 1,            # [MEAS 2026-09-22] depth0 is registered to rgb0 (same K): depth pixel = RGB pixel
        "intrinsics": [570.169, 569.464, 325.870, 238.096], "calib_size": [640, 480],   # registered = 0 only
        "offset_mm": [0.0, 0.0, 0.0],   # registered = 0: the depth lens in the RGB camera frame (x right, y down, z fwd)
        "scale": 1.0,               # [DAY1] tests.depth: true range / depth range
        "latency_s": 0.12,          # [EST research 6.4] exposure -> host when the frame's stamp is unusable
        "stamp_lag_s": 0.067,       # [EST] exposure -> the driver's message stamp (one frame at 15 fps) [DAY1]
        "pair_s": 0.08,             # a depth frame goes with an RGB frame at most this far apart in exposure time
        "min_mm": 200.0, "max_mm": 2000.0,   # [3RD HP60C DS] 0.2-4 m, accurate within 2 m
        "band_mm": [30.0, 90.0],    # the heights a wall / sign return may have (car frame): the mat is 0, the tops 100
        "col_step": 4, "row_step": 2, "k_near": 3, "tol_mm": 30.0,   # per column the k-th nearest return in the band,
        #                                                              then the mean of the returns within tol of it
        "gate": 0.35,               # a depth range must match its column's RGB floor range within this share (0 = off)
        "gate_start": 0.7,          #   ...and at the start search, before the pitch is fitted
        "pillar_cols": 1,           # depth columns a sign needs before its position comes from depth
        "sig_mm": [12.0, 8e-6],     # the localiser's sigma = a + b r^2: [3RD] 2 mm @ 1 m growing ~ z^2, + mounting, sync
    },
    "vision": {"v_dark": 70, "s_col": 90, "v_col": 40, "K": 6, "col_step": 4, "red_h": [10, 165],
               "green_h": [38, 92], "magenta_h": [135, 165], "max_range_mm": 3000.0, "min_blob": 60},
    "odo": {"speed_scale": 0.92,
            "source": "cmd",        # dead reckoning: "cmd" = commanded speed x speed_scale (the stock A1's board holds
            #                         the speed on its encoders); "lidar" = the speed estimator (the command through a
            #                         first-order lag, corrected by the speed the lidar pose measured)
            "est": {"n_scans": 4, "tau_s": 0.24, "gain": 0.6, "max_age_s": 0.6}},   # the estimator (bluewave/speed.py)
    "step": {                       # the park's STEP mode (park.Executor): stop - lidar measure - pulse - repeat
        "on": 0,
        "table": [[7.0, 0.165, 30], [12.0, 0.165, 40], [18.0, 0.165, 50], [24.0, 0.165, 60]],   # [mm, duty, ms]:
        #                             [EST] the sim plant at 726 g (breakaway + 4 counts); [DAY1] tests.step_table
        "near_mm": 40.0,            # the continuous part of a leg stops this far (+ the braked stop) from its end
        "done_mm": 5.0,             # a leg is done within this of its end (~ the smallest step's scatter)
        "settle_s": 0.25, "scans": 2,        # still, then this many lidar scans after it, before measuring
        "max_pulses": 10, "timeout_s": 8.0,
        "v_rrc": 0.08,              # the closed-loop board's "pulse": a timed drive at this speed
    },
    "buttons": {"start_gpio": -1, "start_key": 1},   # a start button on Pi GPIO (the WLtoys build: 17, pin 11 + GND
    #                                                  pin 9, pull-up): its release reads as KEY `start_key` CLICK
    # the car's footprint from the REAR AXLE, and the parking lot it implies (1.5 x the car's length [RULE p.8]).
    # Stock A1: 213 x 159 mm -> lot 320.  BLUE WAVE body (3d/bluewave_body/design/LAYOUT.py): nose to x +192.3 in the
    # floor frame = 262.6 mm ahead of the rear axle, 296.8 mm long -> lot 445.2 -> set front_mm=262.6 lot_len_mm=445.2.
    "car": {"rear_mm": 34.0, "front_mm": 179.0, "half_w_mm": 80.5, "lot_len_mm": 320.0,   # [MEAS] reverse 0.12 m/s x 1 s moved 108 mm -> true / commanded
            "park_margin_mm": 13.0},   # the park legs' clearance to the wall and limitations (wro_next park_margin
    #                                    = -1 reads it): what this car's pose and leg-end accuracy need
    "board": {"device": "/dev/rrc", "baud": 1000000},                   # [VENDOR]
    "hsv": {                        # OpenCV HSV (H 0-179).  [DAY1] tune on the mat under the hall's light
        "red1": [0, 120, 70, 8, 255, 255],
        "red2": [170, 120, 70, 179, 255, 255],
        "green": [45, 90, 50, 85, 255, 255],
        "magenta": [140, 90, 60, 169, 255, 255],
        "orange": [9, 120, 110, 22, 255, 255],
        "blue": [95, 110, 60, 125, 255, 255],
    },
    "safety": {"deadman_ms": 400,   # manual drive stops when commands stop arriving
               "prog_ttl_ms": 250},  # wro_next / lapcam: every motion command carries this dead-man, so a program
    #                                 thread that stalls (CPU, a blocking call) stops the car instead of leaving its last
    #                                 command running unguarded (sim: a 1.0 s stall at 0.50 m/s = ~0.5 m); 0 = none.
    #                                 2.5 x the Pi's projected p99 command gap (~100 ms, BRAIN_SPEC 13)
    "prog": {},                     # per-program defaults of THIS car, {"wro_next": {"v": 0.4, ...}, "lapcam": {...}}: over
    #                                 the program's DEFAULTS, under a run's own params (bw run k=v, race.params).  {} =
    #                                 the programs' DEFAULTS (the stock A1); a profile sets it (profiles/wltoys.json)
    "race": {                       # what race mode runs after the start button
        "challenge": "obstacle",    # "obstacle" -> programs/obs_v17.py (signs + parking), "open" -> BEST_OPEN_14s.py
        "programs": {"obstacle": "obs_v17", "open": "BEST_OPEN_14s"},
        "program": "",              # set to force one program by name (overrides the challenge)
        "params": {},
    },
    # ---- BRAIN4_SPEC 2.1: start anywhere (bluewave/globloc.py), the camera shield, the crash report, the body check
    #      and the mat runner.  Every consumer reads .get(key, default) with these same defaults.
    "loc": {                        # globloc: the whole-field search from still views (4.4-4.6)
        "min_p": 0.95,              # LOCATE moves only when the best hypothesis class has at least this probability
        "good": 0.30,               # per-column ray-cast cost of a fitting view (field_check's `good`)
        "bad": 0.80,                # best per-column cost above this with enough view: not this layout
        "min_cols": 24,             # wall columns / lidar points (all views) before any decision
        "min_cov_deg": 40.0,        # bearing span of the wall points in the LOCATE frame
        "temper_k": 8.0,            # T = max(1, cols / temper_k): the correlated-columns temper (Localizer.update)
        "step_mm": 40.0,            # coarse position grid
        "heading_win_deg": 3.0,     # +- window around the dominant-wall heading
        "full_step_deg": 2.0,       # heading step when no dominant wall direction exists
        "nms_mm": 150.0, "nms_deg": 6.0,   # two hypotheses closer than this are one
        "max_hyps": 8,
        "lot_min_pts": 8,           # magenta bases / limitation returns needed to resolve the frame by the lot
        "expect_mm": 150.0, "expect_deg": 15.0,   # start mark tolerance (start_check); beyond: start_mismatch
        "locate_s": 8.0,            # LOCATE gives up (or LOOKs) after this long without a confident fix
    },
    "look": {                       # LOOK: short moves to see more when a still view is not enough (4.7, S14)
        "v": 0.15,                  # m/s, every LOOK move
        "step_mm": 150.0,           # one LOOK move
        "max_mm": 900.0,            # all LOOK moves together
        "views": 6,                 # still views at most
        "still_s": 0.6,             # the car stands this long before a view
    },
    "camshield": {                  # the stock A1's camera as its range sensor under shield.Shield (4.8)
        "memory_s": 3.0,            # camera points kept for the unseen zone
        "margin_mm": 30.0, "side_mm": 10.0,   # footprint growth ahead / behind, to the sides
        "latency_s": 0.25,          # [EST] camera latency + frame period + loop period (0.09 + 0.067 + 0.025, rounded up)
        "clip_rays": 1,             # a clipped wall column is an obstacle from the image bottom down to the nose
        "v_floor": 0.10,
    },
    "an": {                         # the crash / near-miss analyzer (bluewave/analyze.py, section 6)
        "auto": 1,                  # the agent analyses every run when it ends
        "acc_contact_g": 0.45,      # [MAT 2026-09-23, tel 20 Hz, M4-M6] IMU deviation of a contact; [DAY1] re-check at
        #                             50 Hz on the first clean laps: max(0.45, 1.5 x the clean p99.9)
        "acc_bump_g": 0.28,         # [MAT 2026-09-23] a "bump?" candidate (weak)
        "gyro_jerk_dps": 60.0,      # yaw-rate deviation from the steering's expected rate within 0.1 s
        "push_s": 0.6, "push_ratio": 0.2,   # commanded >= 0.08 m/s this long while the measured motion < ratio x command
        "touch_mm": 8.0,            # lidar body clearance of a contact
        "near_mm": 30.0,            # footprint clearance of a near miss
        "merge_s": 0.6,             # candidates this close in time are one incident
        "loop_gap_ms": 150.0,       # a control step gap this long is an overrun
        "pose_jump_mm": 80.0,       # a relocalisation that moves the pose this far marks the pose before it as wrong
        "sxy_bad_mm": 60.0,
        "bat_rest_v": 7.5, "bat_min_v": 7.0, "bat_sag_v": 0.5,   # battery warnings [MAT 2026-09-23: 8.2 -> 7.05 V in 1.5 h]
        "thumbs": 3,                # frames per incident (at t, t - 0.5 s, t - 1.0 s)
        "max_incidents": 40,
        "cam_frames": 1,            # analyse the recorded frames (the A1's camera signals, D6)
    },
    "body": {                       # which chassis these params are for (bluewave/bodyid.py, section 8)
        "profile": "",              # the applied profile (stamped by paramstore.profile_apply)
        "expect": "",               # the chassis a profile is for: "stock_a1" | "wltoys" ("" = what drive.backend implies)
        "shell": "",                # "a1" | "wl" | "bw2"
        "id_gpio": -1,              # the body strap's GPIO (profiles: 5 = header pin 29; the WLtoys harness bridges pin
        #                             29 to pin 30 GND); -1 = no strap read
        "enforce": 1,               # a DRIVES start is refused when the strap proves the wrong chassis (S15)
    },
    "mat": {                        # bw mat: scripted mat series (tools/bw_mat.py, section 7)
        "cooldown_s": 20.0,         # between mat runs (the WLtoys profiles: 60 -- motor heat, WLTOYS_BUILD 8.7)
        "wait_s": 120.0,            # how long a wait=button run waits for the start button
    },
}


def path() -> str:
    return os.environ.get("BW_PARAMS", os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                                    "params.json"))


def _merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        out[k] = _merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def load() -> dict:
    try:
        with open(path(), encoding="utf-8") as f:
            return _merge(DEFAULTS, json.load(f))
    except FileNotFoundError:
        return copy.deepcopy(DEFAULTS)


def save(p: dict) -> None:
    tmp = path() + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(p, f, indent=2)
    os.replace(tmp, path())       # atomic: a power cut mid-write never leaves a half file
