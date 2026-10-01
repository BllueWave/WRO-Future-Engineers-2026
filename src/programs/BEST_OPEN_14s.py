"""BEST_OPEN_14s -- THE MAIN OPEN PROGRAM of BLUE WAVE (Fawaz 2026-10-01 16:15: "the best and fastest Open").
Mat 2026-10-01 16:13, run 20261001-161301.412-open_v7: 14.7 s, 3 laps, 12/12 corners rolling, finish inside the start
section, no contact (max 0.56 g), pack 8.05 V, CCW.  = open_v7 (frozen) with DEFAULTS preset "l2" (ay_plan 2.6, v 1.8);
nothing else changed.  FROZEN: never edit -- a faster version is a new file.
"""
"""open_v7 -- the FASTER Open Challenge program: BEST_OPEN (open_v6, frozen: CW 18.8 s / CCW 18.7 s on the mat
2026-10-01) + the precision fix of its corners + a faster racing line (preset fast7).  Both ways, 3 laps, rolling corners,
finish inside the start section.  Each change names its mechanism (docs/OPEN_V7.md has the SIM table and the mat
ladder; SIM numbers are SIM -- the mat is the judge):

  V7-1 OBSERVER  the car's TRUE speed.  The drive's estimate is its duty model (1.24-1.53 x under the car at 7.1 V, fact
                 5) corrected by lidar poses that go stale in every arc and most short straights (SIM laps 2-3: the
                 lidar speed not fresh ~70 % of the time) -- so BEST_OPEN's pre-corner brake barely fired (entries 1.3-1.5
                 m/s into 1.15 m/s arcs, SIM +0.2..+0.36 m/s).  Here: the mat-fitted duty line (plant_mat1001: v_ss =
                 5.276 x (duty - 0.1012), lag 0.56 s, the PWM-low brake 2.08 m/s^2) run on the duty the motor really
                 gets (robot.duty_out), corrected at every scan by the front wall's slope (the lidar, delay-matched).
  V7-2 BRAKE     the brake plan on that speed with a CONSTANT deceleration (brake_dec, the mat measured >= 1.6 m/s^2
                 from 0.48 m/s; the fit 2.08) and a short lag: command 0 (PWM-low) until the observer is on the plan.
  V7-3 HOLD      the arc's speed is held: the command is the one whose duty makes v_arc on the fitted line (the drive's
                 PI and its model's under-read inverted) -- BEST_OPEN's arcs asked 1.0-1.15 and ran 1.3-1.5 (SIM truth).
  V7-4 STEER     the arc's steering from the speed-dependent effective wheelbase the mat measured (fit bins: 137 mm at
                 0.2 m/s, ~220 at 0.6-1.1, 262 at 1.25, 270 at 1.45, 280 at 1.65) -- one wb_eff 210 made every arc at
                 1.3-1.5 m/s ~30 % too wide (exits toward the outer wall both ways, SIM -350..+311 mm).
  V7-5 UNKNOWN   a corner whose next width was not measured in time (lap 1, a transient after the last arc): rolls on the
                 rules' SAFE width 600 (a late turn in a 1000 corridor, never an early one into a 600) instead of the stop
                 and measure (~2.5 s; SIM BEST_OPEN 6 of 12 runs).
  V7-6 SPEED     fast7: bigger arcs (R_max, s_min), ay_plan, the envelope's ay ceiling for the drive's own cap (it reads
                 the trimmed servo angle on the 137 mm wheelbase: right arcs were capped at ~1.04 m/s command).
  V7-8 SHIELD    the shield plans on the TRUE speed and the curvature-equivalent steering, its remembered points move by
                 the true travel, and it sees the last 40 deg of an arc unwinding (it projected a circle twice too tight
                 with points lagging forward: brake / 'steer 0' 36 deg before the end of every fast arc, 1.47 -> 0.76).
  V7-9 BOOST     below the asked true speed the drive is asked more (the motor's 0.56 s lag closes 2.5 x faster).
  V7-10 TRIGGER  the arc fires on its PREDICTED end (the steering the car already has, the servo's slew) -- the fixed
                 lead coupled the old lane's offset into the new one 1:1, alternating (+149 / -190 mm).
  V7-11 NARROW   the clearance of the radius the car WILL turn at speed (its lock radius grows with the understeer) and,
                 when none clears, the fastest slower arc that does; a 45 mm outer lane in <= 800 corridors only.

SIM (plant_mat1001, the mat's 1000 x 4 layout, seeds 1-6 both ways): fast7 12/12 clean, 14.0-14.4 s, every corner rolling,
island >= 258 mm; BEST_OPEN 17.9-23.1 s.  Presets: l1 / l2 (the mat ladder's first rungs), fast7 (default), max7 (ay
3.4: past the tip margin, SIM 13.4-13.9 s), fast6 (BEST_OPEN's own, the A/B reference).  The mat is the judge.
"""
"""Open Challenge, FAST: open_final's perception and safety with a racing line, a braking speed plan and a corridor map.

Built from programs/open_final.py (the shipped, SIM-clean Open program; FROZEN, never edited): the same lidar
corridor frame (the scan rotated by the gyro yaw at the scan's time minus the lane), the same F / side walls / island
corner / w_next = F - corner measurement, the same direction logic (start votes, corner gap, island-end jump, camera
cross-check, dir_doubt -> stop and re-decide standing), the same stop-and-measure fallback at full lock, the same
safety (lidar_config halt, lidar_mirrored, steer_fault, stuck / shield holds -> back off, the time limit, the finish in
the start section).  What changed, each with its mechanism:

  RACING LINE  open_final turns every corner at FULL LOCK (R 237 / 278) from the lane centre, so every corner runs at
               the tight-arc speed (~0.35-0.5 m/s).  Here each corner is a 90 deg constant-radius arc from this
               corridor's lane to the next one's, R as big as it fits: the island corner stays >= clear_mm from the
               body (half width + clear_mm from the rear-axle arc, sampled exactly), the straight after it keeps s_min
               for the next arc (half of it: the next corner gets the other half), the trigger is still ahead of the
               car, and the LAST arc leaves the finish its stop (brake + constant run + coast) in the start corridor.
               The lane runs lane_off outside the corridor's centre (toward the outer wall, keeping lane_side_min):
               the island corner is then as far as in a corridor 2 x lane_off wider -- a bigger R in 600 corridors,
               and the shield's projection of the arc into the next island face starts farther.  Steering
               atan(wheelbase / R) (the effective wheelbase, the profile), trigger at F = (w_next / 2 - lane_off) + R +
               the servo-slew lead (as open_final).  v_arc = sqrt(ay_plan x R_cg) (ay at the CG, the drive's formula),
               capped by the drive's own ay / tight caps and its duty ceiling.
  SPEED PLAN   straights run at v (up to the drive's duty ceiling at the pack's voltage, logged) and brake to v_arc at
               the trigger.  With the drive's plug brake on (drive.brake.active, the envelope's brake=1): a command
               dv_min under the speed estimate is the drive's 'brake hard to v' (reverse duty <= 0.3, <= 250 ms, the
               law's ax <= 4.47) and the plan brakes at brake_a (2.5 / 3.2 m/s^2, [EST]: the plant law with the mat's
               tau, unmeasured).  Without it: the shorted-motor model -- PWM low brakes with a force growing with
               speed, v(t) = v0 exp(-t / tau), distance tau x (v0 - v1); tau_plan from the profile's one measured stop
               (drive.stop_decel 0.174 = 180 mm from v_coast_ref 0.25 m/s -> 0.72 s) unless the preset sets it; the
               command drops to 0 while the speed is above the plan (a lower duty slows on the same tau towards a
               NONZERO speed: three times weaker at 1.5 -> 1.0 m/s).
  SHIELD       unchanged filter.  Its braking model is the profile's constant 0.174 m/s^2 (mat1): that caps a straight
               at ~0.8 m/s (v^2 / 2a over a 2 m horizon).  shield_tau > 0 (fast / max) gives it the brake a ZERO
               command really gets (the PWM-low exponential; the plug brake never arms on a stop): decel = v / (2 tau)
               at the commanded speed, need = v (tau + latency) + stop_mm, never below the profile's decel.  It is the
               binding limit in 600-corridor arcs (the arc's circle projected into the next island face).  [NEXT] bw
               test coast at 0.5 / 1.0 / 1.5 m/s BEFORE fast / max on the mat -- the only measured stop is 0.25 m/s.
  MAP/REPLAY   lap 1 measures every corridor's width (side walls) and every next width (island corner); laps 2-3 plan
               every corner from that map before it is in sight (R, v_arc, the braking point), keep measuring, and a
               measurement 2 x alike that disagrees with the map by > map_tol wins (logged map_disagree).
  ENVELOPE     `with robot.speed_envelope(max_mps, duty_max, ay_max, v_tight, brake)` (hw.Robot) around the race: each
               cap loosens only as far as the profile's drive.env ceilings allow (wltoys_bw2: max_mps 1.71, duty 0.5,
               ay 3.731 = min(tip, grip) / 1.5, v_tight 0, brake 1) and is put back however the run ends.  A drive
               without it, or refusing it: the race runs on the present caps (duty 0.35 = ~0.96 m/s), logged
               `envelope` ok=false.  The speed plan always reads the caps the drive really has.

  FIX REVIEW 2026-09-30  (1) the island clearance the EXECUTED arc keeps: planned 130-150 mm, executed 63-98 mm in
               SIM at 1.1-1.52 m/s (the car sits 25-30 mm inside its lane at the trigger, F reads short at speed, the
               servo's delay).  plan_arc now needs clear_mm (180 in every preset) + the car's measured offset toward
               the island (<= clear_y_max) + clear_kv per m/s above clear_v0, and re-plans once near the trigger (R
               only shrinks).  (2) brake_v's lag term braked before EVERY arc even at the arc's speed (105 of 131
               top-speed arcs): nothing to brake -> v_allow >= v_arc.  (3) max's ay 3.731 -> 3.0: the ceiling is the
               rigid tip / 1.5, the stock springs put the tip at 3.83-4.54 (/1.5 = 2.55-3.03) until R0.  (4) logs:
               loop_stats (R6: loop p99, scan age), `unmeasured` when the envelope runs above the profile's measured
               duty, v_capped says why, an E-STOP ends as 'estop'.

Presets (params preset=mat1|fast|max): mat1 = the first mat run -- NO envelope (the mat's caps), the shield on the
profile's measured stop, ay 1.2, centre lanes, lap-1 straights >= 700 mm; fast = envelope ay 2.6 / duty 0.5 / plug
brake, ay_plan 2.4, lane_off 80; max = the SIM limit, the envelope's duty / speed ceilings, ay 3.0, lane_off 100.
DEFAULTS < PRESET < the robot's prog.open_fast < the run's own params.  racing=0 = open_final's full-lock corners.
The mat is the judge; SIM numbers are SIM (see the step report for the table).  Returns {reason, corners, laps,
direction}.
"""
import json
import math
import os
import time
from collections import deque

import numpy as np

from bluewave import field as FLD
from bluewave import lidar_perc as LP
from bluewave import loc as L
from bluewave import shield as SH

PRESETS = {
    # fast7 (open_v7): fast6's safety and perception + V7-1..5 (the observer, the constant-decel brake on it, the held
    # arc speed, the speed-dependent wheelbase, the safe-width roll) + V7-6's line.  Every speed here is the TRUE speed
    # (the observer's), not the drive's command.  Tuned in the calibrated SIM (plant_mat1001, docs/OPEN_V7.md).
    "fast7": dict(v=2.0, v_turn=0.8, ay_plan=3.0, clear_mm=180.0, tau_plan=0.0, shield_tau=0.0,
                  s_min1=400.0, s_min=250.0, lag_s=0.30, corner_reach=1300.0, exit_mm=60.0, cooldown_mm=150.0,
                  brake_a=0.0, racing=1, see_mm=-200.0, turn_trim=15.0, v_start=0.6, v_finish=0.6, start_s=0.2,
                  brake=1, arc_brake=1, shield_decel=1.8, v_meas_max=1, k_y=0.06, exit_add_deg=5.0, steer_trim=-4.0,
                  wb_eff=210.0, steer_cap=22.0, env=1, env_v_tight=0.0, env_max_mps=1.6, env_duty_max=0.5,
                  env_ay_max=3.731, env_brake=0,
                  obs=1, hold=1, wb_v=1, brake_dec=1.8, brake_lag=0.10, brake_hyst=0.04, brake_end=0.03,
                  assume_F=1300.0, assume_w=600.0, cool_map=0.0, psi_trig=0, fin_lo=1.0, finish_room=0.0,
                  finish_back_mm=150.0, start_wait_s=0.05, stop_adaptive=1, stop_extra_s=0.1, stop_scans=1,
                  sh_true=1, sh_unw_deg=40.0, boost_k=1.5, boost_max=0.5, R_max=900.0, pred_trig=1, pred_e0=0.0,
                  slow_fit=1, lane_off=45.0, lane_off_wmax=800.0),
    "fast6": dict(v=1.5, v_turn=1.2, ay_plan=1.9, R_max=700.0, clear_mm=180.0, tau_plan=0.0, shield_tau=0.0,
                  s_min1=700.0, s_min=500.0, lag_s=0.30, corner_reach=1300.0, exit_mm=250.0, cooldown_mm=200.0,
                  brake_a=0.0, racing=1, see_mm=-200.0, turn_trim=20.0, v_start=0.5, v_finish=0.45, start_s=0.4,
                  brake=1, arc_brake=0, shield_decel=1.0, v_meas_max=1, k_y=0.06, exit_add_deg=5.0, steer_trim=-4.0,
                  wb_eff=210.0, steer_cap=22.0, env=1, env_v_tight=0.0, env_max_mps=1.6, env_duty_max=0.5, env_ay_max=3.0,
                  env_brake=0),
    # mat1: the first mat run of this program: NO envelope (the mat's caps: duty 0.35 = ~0.96 m/s, ay 1.959,
    # v_tight 0.30, no plug brake), the shield on the profile's measured stop (0.174 m/s^2: straights ~0.8 m/s at most),
    # the racing line at ay 1.2 (R 500 -> 0.77 m/s), lap-1 straights >= 700 mm
    # fast6 = BEST_OPEN's own preset (open_v5_18s CW 18.8 s + the brake / lead on the lidar speed, v_meas_max): the
    # entries were 1.3-1.6 m/s into 1.15 m/s arcs.  Here only as the A/B reference: preset=fast6 obs=0 hold=0 wb_v=0
    "mat1": dict(v=0.80, v_turn=0.35, ay_plan=1.2, R_max=700.0, clear_mm=180.0, tau_plan=0.0, shield_tau=0.0,
                 s_min1=700.0, s_min=500.0, lag_s=0.30, corner_reach=1300.0, exit_mm=250.0, cooldown_mm=200.0,
                 env=0, brake_a=0.0),
    # fast: the envelope within half the rollover design's margin (ay 2.6 of the ceiling 3.731), duty 0.5, the plug
    # brake planned at 2.5 m/s^2 (its law asks <= 4.47); the shield on the PWM-low brake's exponential model at the
    # measured stop's long end (shield_tau 0.72: need = v (0.72 + latency) + stop_mm) -- after bw test coast.
    # clear_mm 180 = mat1's (fix review 2026-09-30: planned 150 -> executed 76-90 mm at 1.06-1.2 m/s) + clear_kv
    "fast": dict(v=1.50, v_turn=0.45, ay_plan=2.4, R_max=900.0, clear_mm=180.0, tau_plan=0.62, shield_tau=0.72,
                 s_min1=600.0, s_min=320.0, lag_s=0.18, corner_reach=1700.0, exit_mm=120.0, cooldown_mm=120.0,
                 env=1, env_max_mps=1.60, env_duty_max=0.5, env_ay_max=2.6, env_v_tight=0.0, env_brake=1,
                 brake_a=2.5, finish_back_mm=220.0, finish_const_s=0.25, finish_room=60.0, lane_off=80.0,
                 lane_side_min=200.0),
    # max: the SIM limit -- the envelope's duty / speed ceilings (drive.env: max_mps 1.71, duty 0.5), the plug brake
    # planned at 3.2 m/s^2, the shield's exponential model at the mat's middle tau.  ay 3.0, not the ceiling 3.731
    # (fix review 2026-09-30): the ceiling is the RIGID tip 5.60 / 1.5, and the car rides on its stock springs --
    # 13-27 deg/g of roll scaled to 726 g puts the compliant tip at 3.83-4.54 m/s^2, /1.5 = 2.55-3.03.  The drive's
    # own cap follows drive.env ay_max once R0 (the tilt test) sets it.  clear_mm 180 as mat1 (planned 130 -> executed
    # 63-98 mm at 1.3-1.52 m/s): SIM-only until the mat measures the trigger scatter (R6) and the brake (R3)
    "max": dict(v=1.60, v_turn=0.50, ay_plan=3.0, R_max=1000.0, clear_mm=180.0, tau_plan=0.55, shield_tau=0.60,
                s_min1=500.0, s_min=280.0, lag_s=0.15, corner_reach=1800.0, exit_mm=100.0, cooldown_mm=100.0,
                env=1, env_max_mps=1.71, env_duty_max=0.5, env_ay_max=3.0, env_v_tight=0.0, env_brake=1,
                brake_a=3.2, finish_back_mm=220.0, finish_const_s=0.2, finish_room=60.0, lane_off=100.0,
                lane_side_min=200.0),
}

# the mat ladder (docs/OPEN_V7.md): one rung per run, up after 2 clean runs in a row
PRESETS["l1"] = dict(PRESETS["fast7"], ay_plan=2.2, v=1.6)      # ~ BEST_OPEN's real arc speeds, now held and precise
PRESETS["l2"] = dict(PRESETS["fast7"], ay_plan=2.6, v=1.8)
# max7: past the tip design margin (ay 3.4 of the compliant tip 3.83-4.54 m/s^2 = / 1.1-1.3, not / 1.5): only after
# fast7 is clean on the mat AND the tilt test (R0) -- SIM 13.3-13.9 s
PRESETS["max7"] = dict(PRESETS["fast7"], ay_plan=3.4)

DEFAULTS = dict(
    preset="l2",                              # fast7 | fast6 | mat1 | fast | max (PRESETS)
    # ---- open_v7 (V7-1..5; 0 = BEST_OPEN's behaviour for that mechanism)
    obs=0,                                    # V7-1: the true-speed observer drives the plan (v_coast / v_x / odo)
    obs_slope=0.0, obs_icpt=0.0,              # the mat-fitted duty line: v_ss = slope (duty - icpt); 0 = from the fit
    obs_tau=0.0, obs_brake=0.0,               # its speeding-up lag (s) and the PWM-low brake (m/s^2); 0 = from the fit
    fit_file="profiles/plant_mat1001.json",   # tools/fit_plant.py's fit of the mat 2026-09-30/10-01 logs (its `plant`)
    obs_gain=0.5,                             # each front-wall slope pulls the observer this share of its error
    obs_kgain=0.08,                           # ... and the line's scale this much (the pack, the tyres), 0.75-1.35
    obs_win=0.35,                             # s of front-wall ranges one slope is fitted over
    hold=0,                                   # V7-3: the command whose duty makes the asked TRUE speed (>= hold_min)
    hold_min=0.45, hold_floor=0.25,
    wb_v=0,                                   # V7-4: the arc's steering from wb_curve at the observer's speed
    wb_curve=None,                            # [[v m/s, mm], ...]: R x tan(true angle) at speed; None = the fit's
    brake_dec=0.0,                            # V7-2: m/s^2 > 0 = the pre-corner brake plans this constant deceleration
    brake_lag=0.10,                           # s from the command 0 to the brake (the loop + the motor tick)
    assume_F=0.0,                             # V7-5: mm > 0 = the next width still unmeasured this near the wall ahead:
    assume_w=600.0,                           # roll on this width (600 = the rules' safe side: a late turn)
    boost_k=0.0, boost_max=0.5,               # V7-9: the command's lead while below the asked true speed
    slow_fit=0, slow_fit_min=0.5,             # V7-11: no radius clears at the ay speed -> the fastest slower arc
    lane_off_wmax=0.0,                        # V7-11: mm > 0 = lane_off only in corridors this narrow or narrower
    stop_adaptive=0, stop_extra_s=0.1,        # 1 = the finish waits for the observer's standstill (+ extra)
    start_wait_s=0.25,                        # s standing before the start's scans are read
    cool_map=-1.0,                            # V7-6: mm >= 0 = the trigger's cooldown on a map-planned corner
    sh_unw_deg=0.0,                           # V7-8: deg > 0 = the arc's last this-many deg unwind for the shield
    sh_true=0,                                # V7-8: 1 = the shield plans on the true speed and curvature
    pred_e0=1.0,                              # V7-10: the weight of the heading at the trigger in that prediction
    pred_trig=0,                              # V7-10: 1 = fire the arc on its predicted end (steering, heading, slew)
    psi_trig=0,                               # V7-7: 1 = the trigger moves by R sin(the heading toward the turn)
    fin_lo=0.0,                               # mm > 0: the last arc leaves its brake to 0 ending this far past the
    #                                           section's near line (the stop right after the last arc)
    stop_wait_s=0.6, stop_scans=2,            # the finish's standing wait and its measured scans (the log's 'inside')
    laps=3, seconds=175.0, dir=0,             # dir: 0 = decide from the walls, 1 = ccw (left turns), -1 = cw (right)
    v=0.80, v_turn=0.35, v_creep=0.16, v_start=0.35, v_finish=0.25,   # m/s (the values below the preset line: mat1's)
    turn_deg=0.0,                             # the FALLBACK turn's steering (stop-and-measure), 0 = full lock
    R_left=0.0, R_right=0.0,                  # mm > 0: a MEASURED full-lock radius (tests.turn_radius)
    racing=1,                                 # 1 = the racing arc per corner; 0 = open_final's full-lock corners
    ay_plan=1.2,                              # m/s^2 at the CG on the arc (the drive's ay_max caps it anyway)
    R_max=700.0,                              # mm: the biggest arc asked
    clear_mm=180.0,                           # mm: the body's inner side to the island corner, at least
    clear_kv=60.0, clear_v0=0.70,             # + clear_kv mm per m/s of arc speed above clear_v0: the executed arc
    #                                           passes nearer at speed (SIM: 3 mm short at 0.6-0.8 m/s, 43-52 mm median /
    #                                           71-85 p90 at 1.1-1.5: an early trigger, the servo's delay)
    clear_y_max=80.0,                         # mm: the car's measured offset toward the island (from its lane) at the
    #                                           plan is added to the clearance, at most this much
    s_min1=700.0, s_min=500.0,                # mm of straight kept between two arcs: lap 1 (widths measured on it) / 2-3
    map_on=1, map_tol=150.0,                  # laps 2-3: widths from lap 1's map; a measurement off by > map_tol wins
    tau_plan=0.0,                             # s: the braking plan's exponential time constant; 0 = from the profile's
    v_coast_ref=0.25,                         # measured stop: v_coast_ref / (2 drive.stop_decel) (the coast test's speed)
    brake=1, brake_hyst=0.06, brake_end=0.10,                 # 1 = brake while the speed is above the plan by hyst: the plug brake's
    #                                           'brake hard to v' when the drive has it on (drive.brake.active), else 0
    brake_a=0.0,                              # m/s^2: the plug brake's planned deceleration (0 / plug off: tau_plan)
    arc_brake=1,                              # 1 = brake inside a rolling arc entered above v_arc (+ brake_hyst)
    shield_tau=0.0,                           # s > 0: the shield brakes on the exponential model (bw test coast first)
    env=0,                                    # 1 = ask the drive for the envelope below (robot.speed_envelope)
    env_max_mps=1.60, env_duty_max=0.5, env_ay_max=2.6, env_v_tight=0.0, env_brake=1,
    v_ceil_share=0.95,                        # of the drive's duty ceiling at the pack's voltage: headroom for the PI
    a_plan=0.25,                              # m/s^2: the FALLBACK stop's planned slow-down (constant, as open_final)
    lag_s=0.30,                               # s: the command-to-brake lag the plan adds (loop + scan age)
    servo_s=0.35,                             # s from straight to full lock (mat)
    lat_s=0.03,                               # the loop's half period + the servo command's delay
    replan_mm=400.0,                          # re-plan the arc once when F is this (+ 0.3 s of travel) from its trigger
    exit_min_deg=6.0,                         # the turn hands over at least this far short of the new heading
    exit_add_deg=0.0,                         # v2: + this on every hand-over (mat 2026-09-30: exits 4-7 deg over-rotated)
    yr_max=150.0,                             # v2: deg/s the exit's anticipation believes at most (the car's top: v/R ~ 145):
    #                                           IMU samples arriving in one burst after a stall read 250+ and ended a turn 64 deg short
    arc_exit_deg=0.0,                         # > 0: a racing arc hands over this far short (the lane's steering, at
    #                                           most the ay limit at speed, finishes it: a wider exit, less curvature
    #                                           for the shield to project into the next corridor's island face)
    turn_trim=0.0,                            # mm added to the trigger distance (+ turns earlier)
    w_assume=1000.0, w_min=450.0, w_max=1300.0,
    band_mm=150.0,
    corner_band=80.0, front_excl=150.0,
    corner_nb=60.0,
    corner_reach=1300.0,                      # w_next estimates with the island corner this near the lidar
    gap_min=380.0, gap_outer=260.0,
    k_y=0.03, k_psi=0.9, follow_max_deg=18.0, k_wall=0.08,
    side_max=1300.0, r_lo=120.0, r_hi=2200.0,
    stop_early=160.0, reverse_mm=250.0, see_mm=120.0,   # v2: reverse only for a real overshoot (Fawaz: no backing up)
    snap_mm=150.0,                            # v2: a width within this of 600 / 1000 (the Open rule) snaps to it
    seek_mm=600.0,                            # v2: F unknown at the stop -> creep this far looking for the wall
    settle_s=0.3, tol_mm=20.0, creep_s=3.0,
    turn_s=5.0, turn_stall_s=1.2, back_mm=120.0, hold_s=1.2, holds_max=4, stuck_s=1.0,
    finish_room=100.0,                        # mm of margin the last arc leaves for the finish's stop
    lane_off=0.0,                             # mm: the racing lane runs this far outside the corridor's centre (toward
    lane_side_min=150.0,                      # its outer wall), keeping lane_side_min between the body and that wall
    finish_max_mm=1600.0,                     # v2: mm from the last corner the finish may run at most (mat 01:02:
    #                                           F unread after a late last corner -> it drove into lap 4)
    finish_back_mm=0.0,                       # > 0: stop this far inside the section's back line (0 = its middle)
    exit_mm=250.0,                            # mm after a turn at the arc's speed before FOLLOW (open_final: 350)
    cooldown_mm=200.0,                        # mm into FOLLOW before corner estimates count (open_final: 250)
    coast_s=0.5,
    finish_margin=60.0,
    finish_const_s=0.5,
    finish_retry=1,
    shield=1, shield_h=2000.0,
    shield_decel=0.0,
    cfg_check=1,
    feed_pose=1,
    v_extrap=1,                               # 1 = the drive's speed estimate carries scans / times triggers (v_x)
    steer_fault_s=0.8, steer_fault_dps=25.0,
    dead_imu_s=0.35, reopen_gap_s=2.5,        # v2: no IMU sample from the RRC this long = the board hung (mat 2026-09-30,
    #                                           full lock at 7.0 V): hold the car and reopen the port (it resets the board)
    steer_cap=0.0,                            # v2: deg > 0 = no command past this (the linkage's measured lock: past
    #                                           it the servo only pushes on the stop)
    follow_dead_mm=150.0, follow_dead_deg=8.0,  # v2: >= 0.4 s and this far toward what is ahead with >= this steering
    #                                           and the gyro turned < 30 % of it = the steering is not following
    #                                           (mat 2026-09-30 corner 9: wheels held ~+2 deg, 26 asked, into the wall)
    wrong_dps=25.0, wrong_s=0.35,             # v2: >= follow_dead_deg one way and the gyro turning the OTHER way this fast
    #                                           this long = the servo froze on its last angle (mat 23:45: +12.9 held,
    #                                           -12.9 asked, 86 deg the wrong way into the island) -> reopen
    wb_eff=0.0,                               # v2: mm > 0 = the wheelbase the arcs are planned with (mat 2026-09-30 at
    #                                           0.8-1.0 m/s: R x tan(true angle) = 210-248 mm both ways, not 137)
    steer_trim=0.0,                           # v2: deg added to every steering the servo gets: the wheels' straight-ahead
    #                                           (mat 2026-09-30: straight running needed -4..-7 deg all day -> ~-5; ccw
    #                                           hid it, cw lost half of every right turn).  Programs see true angles.
    back_align=1, back_steer=18.0, back_align_mm=350.0,   # v2: a held car backs off turning its nose onto the lane
    v_meas_max=0,                             # v2: 1 = brake / lead on max(model, lidar-measured speed)
    stand_max_deg=12.0,                       # v2: the steering while the command is 0 -- full lock on a standing car
    #                                           is the servo's highest load; the lock comes as the car rolls
    mirror_check=1, cam_lines=1,
    cam_doubt=1,
    cam_halt=0,
    rolling=1,
    dir_at_start=1,
    start_s=0.8,                              # v2: s the start measures at most (was 2.0: the mat start stood 2.3 s)
    cam_dir=0,                                # v2 (off: the camera read orange on a ccw mat 2026-09-30): corner 1 rolls too: the camera's first line colour sets the direction
    #                                           before the corner (the lidar's corner votes overrule it, and a rolling
    #                                           corner still needs the island corner MEASURED on that side)
    loop_hz=40.0, log_every=0.5,
)


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _r(x):
    return None if x is None or x != x else round(x)


def reach_deg(rp):
    """The steering the servo can reach on each side (deg, +left, -right): steer.max_deg and the pulse limits."""
    s = rp["steer"]
    mx = float(s["max_deg"])
    upd = float(s.get("us_per_deg", 0) or 0)
    lo, hi, c = float(s.get("min_us", 0) or 0), float(s.get("max_us", 0) or 0), float(s.get("center_us", 1500))
    left = right = mx
    if upd > 0 and 0 < lo < hi:
        up, down = (hi - c) / upd, (c - lo) / upd
        if s.get("invert"):
            up, down = down, up
        left, right = min(mx, up), min(mx, down)
    return left, right


def _dotted(d, key):
    for part in key.split("."):
        if not isinstance(d, dict) or part not in d:
            return None
        d = d[part]
    return d


def _same(key, have, want):
    if have is None:
        return False
    if key.endswith("offset_deg"):
        try:
            return abs(wrap(float(have) - float(want))) <= 0.5
        except (TypeError, ValueError):
            return False
    if isinstance(want, bool) or isinstance(have, bool):
        return bool(have) == bool(want)
    return have == want


def config_check(robot):
    """open_final.config_check: the robot's lidar convention (params AND the running driver) and drive.stop_decel
    against the stamped profile file."""
    rp = robot.p
    name = str((rp.get("body") or {}).get("profile") or "")
    out = dict(profile=name or None, diffs=[], decel_have=None, decel_want=None, note=None)
    if not name:
        out["note"] = "no body.profile stamp: the lidar convention is not checked"
        return out
    try:
        from bluewave import paramstore as PS
        patch = (PS.file_profiles().get(name) or {}).get("patch")
    except Exception as e:
        out["note"] = "profiles unreadable: %s: %s" % (type(e).__name__, e)
        return out
    if not patch:
        out["note"] = "no profiles/%s.json on this robot: the lidar convention is not checked" % name
        return out
    asm = getattr(getattr(robot, "lidar", None), "asm", None)
    for key, attr in (("lidar.cw", "cw"), ("lidar.offset_deg", "offset")):
        if key not in patch:
            continue
        want = patch[key]
        have = _dotted(rp, key)
        if not _same(key, have, want):
            out["diffs"].append((key, "params", have, want))
        if asm is not None and hasattr(asm, attr):
            run_v = getattr(asm, attr)
            if not _same(key, run_v, want):
                out["diffs"].append((key, "driver", run_v, want))
    if "drive.stop_decel" in patch:
        try:
            out["decel_want"] = float(patch["drive.stop_decel"])
            out["decel_have"] = float((rp.get("drive") or {}).get("stop_decel", 2.2))
        except (TypeError, ValueError):
            out["decel_want"] = out["decel_have"] = None
    return out


def arc_clear(R, wA, wB, n=46):
    """mm from the island corner to the rear-axle arc (90 deg, radius R, lane centre of a wA corridor to the lane
    centre of the next wB one).  Frame: u along this corridor from the arc's start, w toward the island side; the
    arc's centre (0, R), its end (R, R); the wall ahead at u = R + wB / 2, the island corner (R - wB / 2, wA / 2).
    The straights before / after run at wA / 2 and wB / 2 from the island's faces."""
    uc, wc = R - wB / 2.0, wA / 2.0
    th = np.linspace(-math.pi / 2.0, 0.0, n)
    d = float(np.min(np.hypot(R * np.cos(th) - uc, R + R * np.sin(th) - wc)))
    return min(d, wA / 2.0, wB / 2.0)


def _caps_now(robot):
    d = robot.p["drive"]
    return dict(duty_max=float(d["duty"]["max"]), max_mps=float(d["max_mps"]),
                ay_max=float(d.get("ay_max", 0.0) or 0.0), v_tight=float(d.get("v_tight", 0.0) or 0.0),
                brake=int((d.get("brake") or {}).get("active", 0) or 0))


# the fit's numbers when its file cannot be read (the robot's tree without profiles/): plant_mat1001.json 2026-10-01 as
# written by tools/fit_plant.py -- the file is the source, this is its copy for a deploy that lacks it
_FIT_FALLBACK = dict(slope=5.276, intercept=0.1012, tau_s=0.56, brake_decel=2.08,
                     wb_curve=[[0.2, 137.0], [0.6, 227.8], [1.0, 217.7], [1.4, 270.2], [1.7, 280.7]])


def _fit_fill(p):
    """V7-1 / V7-4: the observer's duty line, lag, brake and the speed-dependent wheelbase from the mat fit's file
    (profiles/plant_mat1001.json `plant`) wherever the run / preset left them 0 / None.  Returns where they came from."""
    pl, src = None, "fallback"
    try:
        path = str(p.get("fit_file") or "")
        if path and not os.path.isabs(path):
            path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), path)
        with open(path, encoding="utf-8") as f:
            pl = json.load(f)["plant"]
        src = os.path.basename(path)
    except Exception:
        pl = None
    pl = pl or _FIT_FALLBACK
    for key, fk in (("obs_slope", "slope"), ("obs_icpt", "intercept"), ("obs_tau", "tau_s"), ("obs_brake", "brake_decel")):
        if not float(p.get(key) or 0.0) > 0.0:
            p[key] = float(pl.get(fk) or _FIT_FALLBACK[fk])
    if not p.get("wb_curve"):
        p["wb_curve"] = [[float(a), float(b)] for a, b in (pl.get("wb_curve") or _FIT_FALLBACK["wb_curve"])]
    return src


def run(robot, params, log, stop, hook=None):
    """The preset's envelope (hw.Robot.speed_envelope: the caps loosen only as far as the profile's drive.env allows,
    and are put back however the run ends), then the race.  A drive without the knob, or one refusing it: the race runs
    on the present caps, logged."""
    prog = dict(robot.p.get("prog", {}).get("open_fast", {}) or {})
    preset = str((params or {}).get("preset", prog.get("preset", DEFAULTS["preset"])))
    p = dict(DEFAULTS)
    p.update(PRESETS.get(preset, {}))
    p.update(prog)
    p.update(params or {})
    p["preset"] = preset
    fit_src = _fit_fill(p)
    log(dict(ev="fit", src=fit_src, obs_slope=p["obs_slope"], obs_icpt=p["obs_icpt"], obs_tau=p["obs_tau"],
             obs_brake=p["obs_brake"], wb_curve=p["wb_curve"]))
    caps0 = _caps_now(robot)                    # the profile's own caps (the mat's measured envelope) before any loosening
    ctx = None
    if int(p["env"]):
        want = dict(max_mps=float(p["env_max_mps"]), duty_max=float(p["env_duty_max"]), ay_max=float(p["env_ay_max"]),
                    v_tight=float(p["env_v_tight"]), brake=int(p["env_brake"]))
        fn = getattr(robot, "speed_envelope", None)
        if not callable(fn):
            log(dict(ev="envelope", offered=False, want=want, caps=_caps_now(robot),
                     note="this drive has no speed_envelope: running on its present caps"))
        else:
            try:
                ctx = fn(**want)
                ctx.__enter__()
            except Exception as e:                              # one active already, an unknown cap: say so, run on
                log(dict(ev="envelope", offered=True, ok=False, want=want, caps=_caps_now(robot),
                         error="%s: %s" % (type(e).__name__, e)))
                ctx = None
            else:
                log(dict(ev="envelope", offered=True, ok=True, want=want, applied=getattr(ctx, "applied", None),
                         caps=_caps_now(robot)))
    else:
        log(dict(ev="envelope", offered=None, asked=False, caps=_caps_now(robot)))
    try:
        return _race(robot, p, log, stop, ctx is not None, caps0)
    finally:
        if ctx is not None:
            ctx.__exit__(None, None, None)


def _race(robot, p, log, stop, env_ok, caps0=None):
    rp = robot.p
    mx = float(rp["steer"]["max_deg"])
    wb = float(rp["chassis"]["wheelbase_m"]) * 1000.0            # the EFFECTIVE wheelbase (mat: R 237 at full lock)
    if float(p["wb_eff"]) > 0.0:
        wb = float(p["wb_eff"])
    cg_x = float(rp["drive"].get("cg_x_m", 0.0) or 0.0)
    pc = LP.config(rp)
    lidar_x, lidar_y = (float(v) for v in pc["pos"])
    lag_scan = float(pc["lag"])
    blocks = [(float(a) % 360.0, float(b) % 360.0) for a, b in pc["blind"]]
    r_hi = min(float(p["r_hi"]), float(pc["max_mm"]))
    front_mm = float(rp["car"]["front_mm"])
    rear_mm = float(rp["car"]["rear_mm"])
    half_w = float(rp["car"]["half_w_mm"])
    field = 2.0 * FLD.HALF
    reach_l, reach_r = reach_deg(rp)
    if float(p["steer_cap"]) > 0.0:
        reach_l, reach_r = min(reach_l, float(p["steer_cap"])), min(reach_r, float(p["steer_cap"]))
    reach = {1: reach_l, -1: reach_r}
    turn_deg = dict(reach)
    if float(p["turn_deg"]) > 0.0:
        turn_deg = {1: min(reach_l, float(p["turn_deg"])), -1: min(reach_r, float(p["turn_deg"]))}
    R_of = {k: wb / math.tan(math.radians(d)) for k, d in turn_deg.items()}
    R_lock = {k: wb / math.tan(math.radians(d)) for k, d in reach.items()}
    # v2: a STANDING turn runs slow, where the car turns on the profile's wheelbase (no understeer): the stop point
    # and the standing turn use it (mat 00:06: wb_eff's R 520 put corner 1's stop 180 mm early -> 'passed')
    wb_slow = float(rp["chassis"]["wheelbase_m"]) * 1000.0
    R_slow = {k: wb_slow / math.tan(math.radians(d)) for k, d in turn_deg.items()}
    for k_, key_ in ((1, "R_left"), (-1, "R_right")):
        if float(p[key_]) > 0.0:
            R_of[k_] = float(p[key_])
            R_lock[k_] = float(p[key_])
    slew_dps = mx / max(float(p["servo_s"]), 1e-3)
    # the drive's real ceiling: the duty model's speed at duty.max at the pack's voltage (and max_mps)
    dcfg = rp["drive"]
    try:
        v_duty = float(robot.duty_model.speed(float(dcfg["duty"]["max"]), robot.battery_v()))
    except Exception:
        v_duty = float(dcfg["max_mps"])
    v_ceil = min(float(dcfg["max_mps"]), float(p["v_ceil_share"]) * v_duty) if v_duty > 0 else float(dcfg["max_mps"])
    v_top_all = min(float(p["v"]), v_ceil)
    stop_decel = float(dcfg.get("stop_decel", 0.174) or 0.174)
    tau_plan = float(p["tau_plan"]) if float(p["tau_plan"]) > 0 else float(p["v_coast_ref"]) / (2.0 * stop_decel)
    brk = dcfg.get("brake") or {}
    plug = bool(int(brk.get("active", 0) or 0)) and float(p["brake_a"]) > 0.0 and hasattr(robot, "_brake_arm")
    plug_dv = float(brk.get("dv_min", 0.30) or 0.30)
    plug_floor = float(brk.get("v_floor", 0.25) or 0.25)

    def drive_cap(v, steer):
        """What the drive itself lets through on this steering (hw.Robot._ay_cap, tight_cap)."""
        try:
            return abs(float(robot.tight_cap(robot._ay_cap(v, steer), steer)))
        except AttributeError:
            return v

    v_turn_eff = {k_: drive_cap(float(p["v_turn"]), k_ * d_) for k_, d_ in turn_deg.items()}

    # ------------------------------------------------------------------------------ open_v7: the TRUE speed (V7-1..4)
    obs_on = bool(int(p["obs"]))
    hold_on = bool(int(p["hold"])) and obs_on
    o_sl, o_ic = float(p["obs_slope"]), float(p["obs_icpt"])
    wbc = sorted((float(a_), float(b_)) for a_, b_ in (p.get("wb_curve") or []))
    wbv_on = bool(int(p["wb_v"])) and len(wbc) >= 2
    trim0 = float(p["steer_trim"])
    ok_ = dict(k=1.0)                           # the fitted line's scale, learnt from the front wall (V7-1)

    def wb_at(v):
        """mm: the effective wheelbase R x tan(true angle) at |v| m/s -- the mat fit's bins, linear between, flat
        outside (V7-4); wb (wb_eff) when wb_v is 0."""
        if not wbv_on:
            return wb
        v = abs(float(v))
        if v <= wbc[0][0]:
            return wbc[0][1]
        for (va, la), (vb, lb) in zip(wbc, wbc[1:]):
            if v <= vb:
                return la + (lb - la) * (v - va) / max(vb - va, 1e-6)
        return wbc[-1][1]

    def duty_line():
        """The drive's command -> duty map (hw.Robot: the model's feed-forward x the pack's compensation, capped)."""
        c_ = robot.duty_model.c
        return (float(c_["deadband"]), float(c_["mps_per_duty"]), float(robot.duty_model.comp(robot.battery_v())),
                min(float(c_["max"]), float((dcfg.get("pwm") or {}).get("max_duty", 1.0))))

    def loop_i():
        lp = getattr(robot, "loop", None)
        try:
            return float(lp.i) if lp is not None and int(lp.c.get("on", 0)) else 0.0
        except Exception:
            return 0.0

    def v_true_of_cmd(vc):
        """m/s TRUE the fitted line makes of a steady drive command vc (the loop's integral as it stands)."""
        try:
            db, mpd, comp, dmax = duty_line()
            du = min(dmax, (db + abs(vc) / mpd) * comp) + loop_i()
            return max(0.0, o_sl * ok_["k"] * (du - o_ic))
        except Exception:
            return abs(vc)

    def cmd_for(vt):
        """V7-3: the drive command whose duty makes the TRUE speed vt on the fitted line -- the feed-forward and the
        speed loop (kp on the drive's own estimate when that is fresh, the integral as it stands) inverted.  Below
        hold_min, or on a drive without these internals: vt itself (BEST_OPEN)."""
        if not hold_on or vt < float(p["hold_min"]):
            return vt
        try:
            db, mpd, comp, dmax = duty_line()
            du = o_ic + vt / (o_sl * ok_["k"])
            lp = robot.loop
            on = int(lp.c.get("on", 0))
            kp = float(lp.c.get("kp", 0.03)) if on and robot.speed.fresh(time.monotonic()) else 0.0
            i_ = float(lp.i) if on else 0.0
            vh = float(robot.v_odo(1.0))
            vc = (du - i_ - db * comp + kp * vh) / (comp / mpd + kp)
            return max(float(p["hold_floor"]), vc)
        except Exception:
            return vt

    def boost(v):
        """V7-9: below the asked TRUE speed the drive is asked more (boost_k x the gap, at most boost_max, never above
        the straight's top): the drive's lag (tau 0.56 s) closes (1 + boost_k) x faster -- lap 1 entered its 1.55 m/s
        arcs at 1.29-1.33 (SIM).  At the asked speed the exact command again: a first-order drive does not overshoot."""
        if not obs_on or float(p["boost_k"]) <= 0.0 or v < float(p["hold_min"]):
            return v
        gap = v - ob["v"] - 0.02
        if gap <= 0.0:
            return v
        return min(max(v, v_top_all), v + min(float(p["boost_max"]), float(p["boost_k"]) * gap))

    def v_cap_true(steer_true):
        """m/s TRUE that the drive's own caps let through on this steering: its ay / tight caps read the SERVO angle
        (steer + trim) on the profile's slow wheelbase, on the command -- mapped to the true speed."""
        vc = drive_cap(float(dcfg["max_mps"]), steer_true + trim0)
        return v_true_of_cmd(min(vc, float(dcfg["max_mps"])))

    if hold_on:
        try:
            v_ceil_t = o_sl * (duty_line()[3] - o_ic)
        except Exception:
            v_ceil_t = v_ceil
        v_top_all = min(float(p["v"]), float(p["v_ceil_share"]) * v_ceil_t, v_true_of_cmd(float(dcfg["max_mps"])))
        v_turn_eff = {k_: min(float(p["v_turn"]), v_cap_true(k_ * d_)) for k_, d_ in turn_deg.items()}
    sec_lo = FLD.HALF - FLD.ISL + front_mm
    sec_hi = FLD.HALF + FLD.ISL - rear_mm
    F_mid = 0.5 * (sec_lo + sec_hi)
    # the finish's aim: the section's middle, or finish_back_mm inside its BACK line (the tail's side: less travel
    # after the last corner, more room to stop in; the whole car stays inside with the coast's error either way)
    F_aim = F_mid if float(p["finish_back_mm"]) <= 0.0 else \
        min(F_mid + 300.0, max(sec_lo + 150.0, sec_hi - float(p["finish_back_mm"])))
    cfg = config_check(robot) if int(p["cfg_check"]) else dict(profile=None, diffs=[], decel_have=None,
                                                               decel_want=None, note="cfg_check 0")
    shield_ov = {"on": int(p["shield"])} if int(p["shield"]) >= 0 else {}
    if float(p["shield_h"]) > 0.0:
        shield_ov["horizon_mm"] = float(p["shield_h"])
    decel_by = "drive.stop_decel"
    if float(p["shield_decel"]) > 0.0:
        shield_ov["decel"] = float(p["shield_decel"])
        decel_by = "shield_decel"
    elif cfg["decel_want"] and cfg["decel_have"] and cfg["decel_have"] > cfg["decel_want"] + 1e-6:
        shield_ov["decel"] = cfg["decel_want"]
        decel_by = "profile"
    shield = SH.Shield(rp, shield_ov or None)
    dec_base = float(shield.decel)
    if float(p["shield_tau"]) > 0.0:
        decel_by += "+tau%.2f" % float(p["shield_tau"])
    trim = float(p["steer_trim"])

    sh_true = bool(int(p["sh_true"])) and obs_on
    sh_wb = float(getattr(shield, "wb", wb_slow) or wb_slow)

    def s_eq_of(s_real):
        """V7-8: the steering the shield's wheelbase (the profile's slow 137 mm) needs for the curvature the car REALLY
        turns on at its speed now (wb_at): the shield projects the true circle, not one twice as tight."""
        k_ = math.tan(math.radians(float(s_real))) / max(wb_at(max(ob["v"], 0.2)), 1.0)
        return math.degrees(math.atan(k_ * sh_wb))

    def s_real_of(s_eq):
        k_ = math.tan(math.radians(float(s_eq))) / max(sh_wb, 1.0)
        return math.degrees(math.atan(k_ * wb_at(max(ob["v"], 0.2))))

    class _Trimmed:
        """The robot with the wheels' straight-ahead at `trim`: the shield plans on true angles, the servo gets +trim.
        V7-8 (sh_true): the shield plans on the TRUE speed and the curvature-equivalent steering; below it the real
        angle and the true speed's drive command (V7-3) go out."""
        def drive(self, v, s, ttl=None):
            v, s = float(v), float(s)
            if sh_true:
                sp_ = st.get("sh_pass")
                # the shield let the steering through: the real angle asked; else its own choice, mapped back
                s = sp_[1] if (sp_ is not None and abs(s - sp_[0]) < 1e-6) else s_real_of(s)
                v = cmd_for(boost(v)) if v > 0.0 else v
            return robot.drive(v, max(-mx, min(mx, s + trim)), ttl)

        def __getattr__(self, name):
            return getattr(robot, name)

    bot = SH.command(_Trimmed() if (trim or sh_true) else robot, shield,
                     robot.prog_ttl_ms() if hasattr(robot, "prog_ttl_ms") else None)
    sh_odo = L.Odo()
    st = dict(yaw=robot.yaw, scan_t=0.0, t_prev=time.monotonic(), yaw_f=robot.yaw, wrong=None, log_t=0.0,
              cmd=(0.0, 0.0), odo=0.0, yr=0.0, stuck=None, feat_t=None, cam_t=0.0, braking=False)
    yh = deque(maxlen=240)
    t0 = time.monotonic()
    robot.reset_yaw()
    st["yaw"] = st["yaw_f"] = robot.yaw
    S = dict(lane=0.0, corners=0, direction=int(p["dir"]) if p["dir"] in (1, -1) else 0, dir_by="param",
             w_cur=float(p["w_assume"]), mode="START")
    feat = dict(F=float("nan"), t=0.0, side={1: float("nan"), -1: float("nan")}, corner={1: None, -1: None},
                pw=None, t_proc=0.0, yaw_s=robot.yaw, carry=0.0)
    Fh = []
    end = dict(reason=None)
    cam = dict(first=None, dir=None, n=0, err=None)
    wmap = {}                                   # corridor index (0 = the start corridor) -> width mm (lap 1's)
    arcs = []                                   # per corner: the plan that ran (the log's summary)
    loop_dt = deque(maxlen=20000)               # s per loop pass (loop_stats at the end: R6's p99 <= 50 ms)
    scan_age = deque(maxlen=5000)               # s from a scan's time to the pass that used it
    fvalid = {}                                 # phase -> [F valid, scans where a wall 0.4-1.8 m was due, F short]

    # ------------------------------------------------------------------------------------------ V7-1 the observer
    ob = dict(v=0.0, h=deque(maxlen=240), Fr=deque(maxlen=12), n=0, err=deque(maxlen=600))
    try:
        brk_away = float(robot.duty_model.c.get("breakaway", 0.1362))
    except Exception:
        brk_away = 0.1362

    def obs_step(dt):
        """The fitted drive on the duty the motor really has: the lag towards its line, the PWM-low brake below it."""
        vc = float(getattr(robot, "v_cmd", 0.0) or 0.0)
        du = abs(float(getattr(robot, "duty_out", 0.0) or 0.0))
        vo = ob["v"]
        if vc < 0.0:
            nv = 0.0                                # reversing (a back-off): the forward plan restarts from 0
        elif vo < 0.003 and du < brk_away:
            nv = 0.0                                # static friction holds it
        elif du > o_ic:
            vss = o_sl * ok_["k"] * (du - o_ic)
            nv = vo + (vss - vo) * min(1.0, dt / float(p["obs_tau"]))
            if vss < vo:
                nv = max(nv, vo - float(p["obs_brake"]) * dt)
        else:
            nv = max(0.0, vo - float(p["obs_brake"]) * dt)
        ob["v"] = nv
        ob["h"].append((time.monotonic(), nv))

    def vo_at(t):
        h = ob["h"]
        if not h or t >= h[-1][0]:
            return ob["v"]
        for i_ in range(len(h) - 1, -1, -1):
            if h[i_][0] <= t:
                return h[i_][1]
        return h[0][1]

    def obs_scan(t_s, F_raw):
        """A raw front-wall range at its scan's time: the slope over obs_win corrects the observer at the window's
        mean time (the observer's own value THEN: no lag in the correction), and the correction is carried to now."""
        Fr = ob["Fr"]
        if Fr and (F_raw - Fr[-1][1] > 150.0 or Fr[-1][1] - F_raw > 400.0):
            Fr.clear()                              # a new wall ahead (a corner, a gap): not a speed
        Fr.append((t_s, F_raw))
        hh = [q for q in Fr if q[0] >= t_s - float(p["obs_win"])]
        if len(hh) < 3 or hh[-1][0] - hh[0][0] < 0.15:
            return
        ts = np.array([q[0] for q in hh])
        fs = np.array([q[1] for q in hh])
        k_, b_ = np.polyfit(ts - ts[0], fs, 1)
        if float(np.median(np.abs(fs - (k_ * (ts - ts[0]) + b_)))) > 25.0:
            return
        vF = max(0.0, -float(k_) / 1000.0)
        tm = float(np.mean(ts))
        v_then = vo_at(tm)
        err = vF - v_then
        if abs(err) > 0.8:
            return
        dv = float(p["obs_gain"]) * err
        ob["v"] = max(0.0, ob["v"] + dv)
        h = ob["h"]
        for i_ in range(len(h) - 1, -1, -1):        # the history after tm moves with it (no double count)
            if h[i_][0] < tm:
                break
            h[i_] = (h[i_][0], max(0.0, h[i_][1] + dv))
        ob["n"] += 1
        ob["err"].append(err)
        du = abs(float(getattr(robot, "duty_out", 0.0) or 0.0))
        if du > o_ic + 0.05 and not st["braking"] and v_then > 0.4:
            ok_["k"] = min(1.35, max(0.75, ok_["k"] * (1.0 + float(p["obs_kgain"]) * err / v_then)))

    class Halt(Exception):
        pass

    # ------------------------------------------------------------------------------------------------ scan geometry
    blk_cache = {}

    def blocked(a):
        k = len(a)
        if k not in blk_cache:
            m = np.zeros(k, bool)
            aa = a % 360.0
            for lo, hi in blocks:
                m |= (aa >= lo) & (aa <= hi) if lo <= hi else ((aa >= lo) | (aa <= hi))
            blk_cache[k] = m
        return blk_cache[k]

    def lid(ang, half):
        sc = robot.scan
        if sc is None:
            return float("nan")
        a = sc.angles()
        d = ((a - ang + 180.0) % 360.0) - 180.0
        m = (np.abs(d) <= half) & (sc.dist > p["r_lo"]) & (sc.dist < r_hi)
        return float(np.median(sc.dist[m])) if m.any() else float("nan")

    def yaw_at(t):
        if not yh or t >= yh[-1][0]:
            return robot.yaw
        if t <= yh[0][0]:
            return yh[0][1]
        lo, hi = 0, len(yh) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if yh[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        (ta, ya), (tb, yb) = yh[lo], yh[hi]
        return ya + wrap(yb - ya) * (t - ta) / max(tb - ta, 1e-6)

    def v_meas():
        h = [q for q in Fh if q[0] > time.monotonic() - 0.45]
        if len(h) >= 3 and h[-1][0] - h[0][0] >= 0.18:
            ts = np.array([q[0] for q in h]) - h[0][0]
            fs = np.array([q[1] for q in h])
            k, b = np.polyfit(ts, fs, 1)
            if float(np.median(np.abs(fs - (k * ts + b)))) < 40.0:
                return max(0.0, -float(k) / 1000.0)
        return max(0.0, float(robot.v_odo(1.0)))

    def island_end(um, vi, res_rad):
        for i in range(len(um) - 1, 0, -1):
            th_ = math.atan2(vi, max(um[i] - lidar_x, 1.0))
            step = min(150.0, vi / max(math.sin(th_) ** 2, 1e-3) * res_rad)
            if um[i] - um[i - 1] <= max(p["corner_nb"], 1.6 * step):
                return float(um[i])
        return None

    def process_scan(sc):
        a = sc.angles()
        d = sc.dist.astype(float)
        m = (d > p["r_lo"]) & (d < r_hi) & ~blocked(a)
        ar = np.radians(a[m])
        X = lidar_x + d[m] * np.cos(ar)
        Y = lidar_y + d[m] * np.sin(ar)
        v_now = v_x()
        carry = v_now * max(0.0, time.monotonic() - sc.t + lag_scan) * 1000.0
        X = X - carry
        yaw_s = yaw_at(sc.t - lag_scan)
        psi = math.radians(wrap(yaw_s - S["lane"]))
        c, s = math.cos(psi), math.sin(psi)
        u, w = X * c - Y * s, X * s + Y * c
        m = (np.abs(w) < p["band_mm"]) & (u > lidar_x + 80.0)
        # v2: the NEAREST quarter of the band, not the median -- a gap in the wall ahead let the far side through,
        # the median jumped or vanished and the car turned onto the island (mat 2026-09-30, corner 3)
        F = float(np.percentile(u[m], 25.0)) if int(m.sum()) >= 3 else float("nan")
        side = {}
        for sg in (1, -1):
            m = (sg * w > 60.0) & (sg * w < p["side_max"]) & (u > lidar_x - 40.0) & (u < lidar_x + 350.0)
            side[sg] = float(np.median(sg * w[m])) if int(m.sum()) >= 4 else float("nan")
            if side[sg] == side[sg]:
                S.setdefault("v_side", {})[sg] = side[sg]
        corner = {1: None, -1: None}
        res_rad = math.radians(sc.res)
        if F == F:
            for sg in (1, -1):
                vi = side[sg] if side[sg] == side[sg] else S.get("v_side", {}).get(sg)
                if vi is None:
                    continue
                m = (np.abs(sg * w - vi) < p["corner_band"]) & (u > lidar_x - 40.0) & (u < F - p["front_excl"])
                if int(m.sum()) < 3:
                    continue
                ul = island_end(np.sort(u[m]), vi, res_rad)
                if ul is None:
                    continue
                th = math.atan2(vi, max(ul - lidar_x, 1.0))
                dres = min(150.0, vi / max(math.sin(th) ** 2, 1e-3) * res_rad)
                uc = ul + 0.5 * dres
                corner[sg] = dict(u=uc, w=F - uc, gap=F - ul, vi=vi)
        feat.update(F=F, side=side, corner=corner, t=sc.t, t_proc=time.monotonic(), yaw_s=yaw_s, carry=carry * c,
                    pw=wall_psi(X, Y, u, w, side, corner, F))
        if obs_on and F == F and S["mode"] in ("FOLLOW", "FINISH", "START"):
            obs_scan(sc.t - lag_scan, F + carry * c)        # V7-1: the raw range (the carry is the estimate's)
        # R1 / R3 (the body pitches under acceleration and braking: the 50 mm plane loses the wall or reads the mat):
        # F's validity where the wall ahead SHOULD be 0.4-1.8 m (the last F dead-reckoned by the odometer), by phase
        fl = st.get("F_last")
        if fl is not None and S["mode"] in ("FOLLOW", "FINISH"):
            exp = fl[0] - (st["odo"] - fl[1])
            if 400.0 <= exp <= 1800.0:
                ph = "brake" if st["braking"] else ("accel" if st["cmd"][0] > v_now + 0.15 else "cruise")
                q_ = fvalid.setdefault(ph, [0, 0, 0])
                q_[1] += 1
                if F == F:
                    q_[0] += 1
                    if exp - F > 150.0:
                        q_[2] += 1                      # read >150 mm SHORT of the dead-reckoned wall: the mat?
        st["F_last"] = (F, st["odo"]) if F == F else (None if fl is None else fl)
        if F == F:
            Fh.append((sc.t, F))
            del Fh[:-8]
            if int(p["feed_pose"]) and hasattr(robot, "feed_pose"):
                try:
                    robot.feed_pose(sc.t - lag_scan, -(F + carry * c), 0.0, 0.0)
                except Exception:
                    pass
        return feat

    def wall_psi(X, Y, u, w, side, corner, F):
        out = []
        for sg in (1, -1):
            vi = side[sg]
            if vi != vi:
                continue
            u_hi = lidar_x + 1300.0
            if F == F:
                u_hi = min(u_hi, F - 250.0)
            c = corner[sg]
            if c and c["gap"] >= p["gap_min"]:
                u_hi = min(u_hi, c["u"] - 60.0)
            m = (np.abs(sg * w - vi) < 60.0) & (u > lidar_x - 40.0) & (u < u_hi)
            if int(m.sum()) < 8 or float(np.ptp(u[m])) < 250.0:
                continue
            k, b = np.polyfit(X[m], Y[m], 1)
            if float(np.median(np.abs(Y[m] - (k * X[m] + b)))) > 15.0:
                continue
            out.append(math.degrees(math.atan(k)))
        if not out or (len(out) == 2 and abs(out[0] - out[1]) > 3.0):
            return None
        return -float(np.mean(out))

    def v_coast():
        if obs_on:
            return ob["v"]                          # V7-1: the observer's true speed
        sp = getattr(robot, "speed", None)
        if int(p["feed_pose"]) and sp is not None and str(rp.get("odo", {}).get("source", "cmd")) == "lidar":
            try:
                if sp.fresh(time.monotonic()):
                    v_ = max(0.0, float(robot.v_odo(1.0)))
                    if int(p["v_meas_max"]):
                        # v2: the lidar's own last measurement when it is higher: the duty model under-read the
                        # car by 24-53 % at 7.1 V (mat 01:05) and the pre-corner brake never fired at 1.5 m/s
                        v_ = max(v_, float(getattr(sp, "v_meas", 0.0) or 0.0))
                    return v_
            except Exception:
                pass
        return st.get("v_est", 0.0)

    def F_now():
        F = feat["F"]
        if F != F:
            return F
        return F - v_x() * (time.monotonic() - feat["t_proc"]) * 1000.0

    def v_x():
        """m/s to carry a scan to now and to time a trigger: the drive's estimate (its duty model corrected by the
        lidar speed our poses feed) with v_extrap=1; the F slope (open_final) lags ~0.2 s -- accelerating out of the
        start it read 0.6 at a true 1.2 m/s and the first arc began 97 mm late: its end 56 mm toward the outer wall
        (SIM max seed 1)."""
        return v_coast() if int(p["v_extrap"]) else st.get("v_est", 0.0)

    # ------------------------------------------------------------------------------------------------ the loop pass
    def go(v, s):
        v = float(v)
        if float(p["shield_tau"]) > 0.0 and v != 0.0:
            # the shield's stop on the exponential brake at THIS command's speed: need = v tau + v latency + stop_mm
            shield.decel = max(dec_base, abs(v) / (2.0 * float(p["shield_tau"])))
        st["cmd"] = (v, float(s))
        if st.get("imu_dead"):
            v = 0.0                             # no heading: nobody drives on a frozen yaw
        s = float(s)
        if p["steer_cap"] > 0.0:
            s = max(-p["steer_cap"], min(p["steer_cap"], s))
        if st.get("steer_hold") and time.monotonic() < st["steer_hold"]:
            v = 0.0                             # a steering that stopped following: stand while the board resets
        if v == 0.0 and abs(float(robot.v_odo(1.0))) < 0.08:
            s = max(-p["stand_max_deg"], min(p["stand_max_deg"], s))
        s = max(-mx, min(mx, s))
        if sh_true:
            # V7-8: the shield sees the truth (_Trimmed maps it back); the last sh_unw_deg of an arc it sees the arc
            # unwinding (the hand-over is ~20 deg before the lane: its constant circle ran into the next island face)
            s_p = s_eq_of(s) * float(st.get("sh_unw", 1.0))
            st["sh_pass"] = (s_p, s)
            bot.drive(v, s_p)
        else:
            bot.drive(cmd_for(boost(v)) if v > 0.0 else v, s)      # V7-3: the true speed -> its command

    def tick(mode):
        S["mode"] = mode
        if stop.is_set():
            # an E-STOP also sets stop: say which, so the library's last run tells them apart
            end["reason"] = "estop" if getattr(robot, "estopped", False) else "stopped"
            raise Halt
        tl = time.monotonic()
        dt, st["t_prev"] = tl - st["t_prev"], tl
        if obs_on:
            obs_step(min(dt, 0.1))
        if mode not in ("START",):
            loop_dt.append(dt)                  # R6: the loop's real period on the Pi (the SIM has no stalls)
        if tl - t0 > p["seconds"]:
            end["reason"] = "time"
            raise Halt
        if hasattr(robot, "board_down_s") and robot.board_down_s() > 1.0:
            end["reason"] = "board_lost"
            raise Halt
        v_o = robot.v_odo(1.0)
        st["odo"] += (ob["v"] if obs_on else abs(v_o)) * 1000.0 * dt
        dth = math.radians(wrap(robot.yaw - st["yaw"]))
        st["yaw"] = robot.yaw
        yh.append((tl, robot.yaw))
        sc = robot.scan
        if shield.enabled:
            # V7-8: the shield's remembered points (the island corner beside the car, out of the lidar's front 180 deg)
            # move by the TRUE travel -- on the drive's under-read they lagged forward into the swept path (SIM: brake /
            # 'steer 0' at 36 deg to go in every lap-2/3 arc, 1.47 -> 0.76 m/s)
            v_sh = ob["v"] if (sh_true and float(getattr(robot, "v_cmd", 0.0) or 0.0) >= 0.0) else v_o
            shield.odom(v_sh * 1000.0 * dt, dth)
            sh_odo.add(tl, v_sh * 1000.0 * dt, dth)
        if sc is not None and sc.t != st["scan_t"]:
            st["scan_t"] = sc.t
            scan_age.append(tl - sc.t)
            if shield.enabled:
                shield.scan(LP.of(robot), sh_odo.since)
            if mode not in ("TURN",):
                process_scan(sc)
                st["v_est"] = v_meas()
                st["new_scan"] = True
        yr = max(-400.0, min(400.0, wrap(robot.yaw - st["yaw_f"]) / max(dt, 1e-3)))
        st["yaw_f"] = robot.yaw
        st["yr"] += (yr - st["yr"]) * min(1.0, dt / 0.08)
        s_c = float(robot.steer_cmd) - trim     # the wheels' true angle
        if v_o > 0.12 and abs(s_c) >= 20.0 and st["yr"] * math.copysign(1.0, s_c) < -p["steer_fault_dps"]:
            st["wrong"] = st["wrong"] or tl
            if tl - st["wrong"] >= p["steer_fault_s"]:
                log(dict(ev="steer_fault", steer=round(s_c, 1), yaw_rate=round(st["yr"], 1), mode=mode))
                end["reason"] = "steer_fault"
                raise Halt
        else:
            st["wrong"] = None
        fr_ = lid(0.0, 3.0)
        fh = st.setdefault("fh", [])
        if fh and (s_c * fh[-1][2] <= 0.0 or abs(s_c) < p["follow_dead_deg"]):
            fh.clear()                          # the window runs while the steering holds one side, >= follow_dead_deg
        if abs(s_c) >= p["follow_dead_deg"]:
            fh.append((tl, robot.yaw, s_c, fr_))
        fv = [q for q in fh if q[3] == q[3]]
        if len(fv) >= 2 and fv[-1][0] - fv[0][0] >= 0.4 and fv[0][3] - fv[-1][3] >= p["follow_dead_mm"]:
            sm = sum(abs(q[2]) for q in fh) / len(fh)
            ran = fv[0][3] - fv[-1][3]
            exp_ = math.degrees(ran * math.tan(math.radians(sm)) / wb)
            got = wrap(robot.yaw - fv[0][1]) * math.copysign(1.0, s_c)
            if got < 0.3 * exp_ and tl - st.get("reopen_t", -99.0) > p["reopen_gap_s"]:
                ok = robot.board_reopen() if hasattr(robot, "board_reopen") else False
                st["reopen_t"], st["steer_hold"] = tl, tl + 0.6
                log(dict(ev="steer_dead", ran_mm=round(ran), steer=round(sm, 1), yaw_exp=round(exp_, 1),
                         yaw_got=round(got, 1), mode=mode, reopened=bool(ok)))
                fh.clear()
        if (abs(s_c) >= p["follow_dead_deg"] and v_o > 0.3 and st["yr"] * math.copysign(1.0, s_c) < -p["wrong_dps"]
                and st.get("wrong2_s", 0.0) * s_c > 0.0):
            st["wrong2"] = st.get("wrong2") or tl
            if tl - st["wrong2"] >= p["wrong_s"] and tl - st.get("reopen_t", -99.0) > p["reopen_gap_s"]:
                ok = robot.board_reopen() if hasattr(robot, "board_reopen") else False
                st["reopen_t"], st["steer_hold"], st["wrong2"] = tl, tl + 0.6, None
                log(dict(ev="steer_wrong", steer=round(s_c, 1), yaw_rate=round(st["yr"], 1), mode=mode, reopened=bool(ok)))
        else:
            st["wrong2"] = None
        st["wrong2_s"] = s_c if abs(s_c) >= p["follow_dead_deg"] else 0.0
        ia = float(robot.imu_age_s()) if hasattr(robot, "imu_age_s") else 0.0
        st["imu_dead"] = ia >= p["dead_imu_s"]
        if st["imu_dead"] and tl - st.get("reopen_t", -99.0) > p["reopen_gap_s"]:
            ok = robot.board_reopen() if hasattr(robot, "board_reopen") else False
            st["reopen_t"] = tl
            st["n_reopen"] = st.get("n_reopen", 0) + 1
            log(dict(ev="board_hung", imu_age_ms=round(min(ia, 99.0) * 1000), steer=round(s_c, 1), mode=mode,
                     reopened=bool(ok), n=st["n_reopen"]))
        seen = robot.motion_seen() if hasattr(robot, "motion_seen") else None
        if (float(getattr(robot, "v_cmd", 0.0)) > 0.05 and seen is False) or getattr(robot, "stalled", False):
            st["stuck"] = st["stuck"] or tl
        else:
            st["stuck"] = None
        if int(p["cam_lines"]) and S["corners"] == 0 and tl - st["cam_t"] >= 0.2:
            st["cam_t"] = tl
            cam_look()
        if tl - st["log_t"] >= p["log_every"]:
            st["log_t"] = tl
            log(dict(mode=mode, corners=S["corners"], yaw=round(robot.yaw, 1), lane=round(S["lane"], 1),
                     F=_r(F_now()), left=_r(feat["side"][1]), right=_r(feat["side"][-1]), w=_r(S["w_cur"]),
                     w_next=_r(S.get("w_next")), v=round(st["cmd"][0], 2), v_est=round(st.get("v_est", 0.0), 2),
                     steer=round(st["cmd"][1], 1), shield=shield.act, brk=st["braking"],
                     pw=None if feat["pw"] is None else round(feat["pw"], 1)))
        time.sleep(1.0 / p["loop_hz"])

    def wait(mode, secs, v=0.0, s=None):
        t = time.monotonic()
        while time.monotonic() - t < secs:
            go(v, st["cmd"][1] if s is None else s)
            tick(mode)

    def still_feat(mode, n=3, max_s=1.5, votes=None):
        Fs, t, seen = [], time.monotonic(), st["scan_t"]
        while len(Fs) < n and time.monotonic() - t < max_s:
            go(0.0, st["cmd"][1])
            tick(mode)
            if st["scan_t"] != seen:
                seen = st["scan_t"]
                if feat["F"] == feat["F"]:
                    Fs.append(feat["F"])
                if votes is not None:
                    votes.append(decide_dir(feat["corner"][1], feat["corner"][-1]))
        return (float(np.median(Fs)) if Fs else float("nan")), feat

    def back_off(mode, mm, align=False):
        s_b = 0.0
        psi_b = wrap(robot.yaw - S["lane"])
        if align and int(p["back_align"]) and abs(psi_b) >= 6.0:
            # v2: reverse with the wheels turned so the nose swings back onto the lane (a 3-point turn): straight
            # back left the car 18 deg off and the shield held it 5 times (mat 00:12).  Reversing, steering s turns
            # the heading by -s: psi < 0 (nose right of the lane) -> s < 0.
            s_b = math.copysign(min(float(p["back_steer"]), 1.2 * abs(psi_b)), psi_b)
            need = math.radians(abs(psi_b)) * wb_slow / max(math.tan(math.radians(abs(s_b))), 0.05)
            mm = min(float(p["back_align_mm"]), max(mm, need))
            log(dict(ev="back_align", psi=round(psi_b, 1), steer=round(s_b, 1), mm=round(mm)))
        wait(mode, mm / 1000.0 / p["v_creep"], -p["v_creep"], s_b)
        wait(mode, 0.15, 0.0, 0.0)

    # ------------------------------------------------------------------------------------------------ the speed plan
    def brake_v(d_mm, v_goal):
        """The highest speed now that still brakes to v_goal within d_mm: lag_s at this speed, then the plug brake at
        brake_a (constant) when the drive has it on, else the PWM-low brake's exponential (tau_plan x (v - v_goal))."""
        d = max(0.0, float(d_mm)) / 1000.0
        if obs_on and float(p["brake_dec"]) > 0.0:
            # V7-2: the PWM-low brake as the mat measured it -- a constant deceleration (>= 1.6 m/s^2 from 0.48 m/s;
            # the fit 2.08), after brake_lag -- on the observer's true speed
            a, lag = float(p["brake_dec"]), float(p["brake_lag"])
            return -a * lag + math.sqrt(a * a * lag * lag + v_goal * v_goal + 2.0 * a * d)
        lag = float(p["lag_s"])
        if plug and v_goal >= plug_floor:
            a = float(p["brake_a"])
            return -a * lag + math.sqrt(a * a * lag * lag + v_goal * v_goal + 2.0 * a * d)
        return (d + tau_plan * v_goal) / (lag + tau_plan)

    def brake_dist(v0, v1):
        """mm the plan needs to brake from v0 to v1 (the inverse of brake_v)."""
        if v0 <= v1:
            return 0.0
        if obs_on and float(p["brake_dec"]) > 0.0:
            a, lag = float(p["brake_dec"]), float(p["brake_lag"])
            return 1000.0 * (v0 * lag + (v0 * v0 - v1 * v1) / (2.0 * a))
        lag = float(p["lag_s"])
        if plug and v1 >= plug_floor:
            return 1000.0 * (v0 * lag + (v0 * v0 - v1 * v1) / (2.0 * float(p["brake_a"])))
        return 1000.0 * (v0 * lag + tau_plan * (v0 - v1))

    def fin_pre(v_f):
        """mm the finish needs after its speed v_f: V7-2 the brake from v_f to 0; else the coast + constant run."""
        if obs_on and float(p["brake_dec"]) > 0.0:
            return brake_dist(v_f, 0.0)
        return v_f * (float(p["coast_s"]) + float(p["finish_const_s"])) * 1000.0

    def speed_cmd(v_allow, v_cap, v_goal=0.0):
        """The command for a planned speed: v_allow capped.  brake=1 while the measured speed is above the plan by
        brake_hyst (until it is back on it): the plug brake on -> a command dv_min under the speed (the drive's 'brake
        hard to v', never under v_goal); off -> 0 (the motor's PWM-low brake)."""
        v_want = max(0.0, min(v_cap, v_allow))
        if int(p["brake"]):
            vb = v_coast()
            if vb > v_want + float(p["brake_hyst"]) and vb > 0.15:
                st["braking"] = True
            elif st["braking"] and vb <= v_want + (0.02 if st.get("plugging") else float(p["brake_end"])):
                # the PWM-low brake ends brake_end early: the drive's estimate decays on its stop_decel model (0.174)
                # between the lidar's corrections, so it trails the braked car (SIM: ~1 s over 12 corners, fast)
                st["braking"] = False
            if st["braking"]:
                # the plug arms only dv_min under the estimate: nearer v_goal than that, a v_goal command is only a
                # lower duty (the same tau towards a nonzero speed) -- the PWM-low brake (0) is stronger there
                # (SIM max seed 3 cw: 1.36 m/s into a 1.03 arc, ay 4.9)
                if plug and vb > plug_floor and vb - max(v_goal, 0.0) >= plug_dv + 0.02:
                    st["plugging"] = True
                    return max(v_goal, min(v_want, vb - plug_dv - 0.02), 0.0)
                st["plugging"] = False
                return 0.0
        return v_want

    def lane_off(w):
        """mm the racing lane runs outside the corridor's centre (toward its OUTER wall): lane_off, at most what keeps
        lane_side_min between the body and that wall."""
        if float(p["lane_off_wmax"]) > 0.0 and w > float(p["lane_off_wmax"]):
            return 0.0                          # V7-11: only in the narrow (600) corridors
        return max(0.0, min(float(p["lane_off"]), w / 2.0 - half_w - float(p["lane_side_min"])))

    def follow_steer(v=None):
        psi = wrap(robot.yaw - S["lane"])
        y = lateral()
        sg_ = S["direction"]
        y_t = -sg_ * lane_off(S["w_cur"]) if (sg_ and int(p["racing"])) else 0.0
        steer = -p["k_psi"] * psi - (p["k_y"] * (y - y_t) if y is not None else 0.0)
        lim = float(p["follow_max_deg"])
        v_ = abs(st["cmd"][0]) if v is None else abs(v)
        if v_ > 0.5:
            # the lane's corrections within ay_plan at this speed (a straight-speed 18 deg is ay > 5 m/s^2)
            lim = min(lim, max(4.0, math.degrees(math.atan(wb / 1000.0 * float(p["ay_plan"]) / (v_ * v_)))))
        return max(-lim, min(lim, steer))

    # ------------------------------------------------------------------------------------------------ the camera
    def cam_look():
        if cam["first"] is not None or cam["err"]:
            return
        try:
            fr, _t = robot.frame()
            if fr is None:
                return
            import cv2
            h = fr.shape[0]
            roi = fr[int(h * 0.62):h:4, ::4]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            H, Sa, V = hsv[..., 0], hsv[..., 1], hsv[..., 2]
            ok = (Sa >= 20) & (V >= 90)
            n_or = int(((H >= 5) & (H <= 25) & ok).sum())
            n_bl = int(((H >= 95) & (H <= 130) & ok).sum())
            cam["n"] += 1
            need = max(40, int(0.006 * H.size))
            if st["odo"] > 150.0 and max(n_or, n_bl) >= need and abs(n_or - n_bl) >= need // 2:
                cam["first"] = "orange" if n_or > n_bl else "blue"
                cam["dir"] = -1 if cam["first"] == "orange" else 1
                log(dict(ev="cam_lines", first=cam["first"], dir="cw" if cam["dir"] < 0 else "ccw", orange=n_or,
                         blue=n_bl, lidar_dir=S["direction"] or None,
                         agree=None if not S["direction"] else cam["dir"] == S["direction"]))
        except Exception as e:
            cam["err"] = "%s: %s" % (type(e).__name__, e)
            log(dict(ev="cam_lines", error=cam["err"]))

    def doubt(by, **kw):
        if S.get("dir_doubt") or S["corners"] != 0:
            return
        S["dir_doubt"] = by
        log(dict(ev="dir_doubt", by=by, dir=S["direction"] or None, dir_by=S["dir_by"], **kw))

    # ------------------------------------------------------------------------------------------------ the lane
    def lateral():
        vl, vr = feat["side"][1], feat["side"][-1]
        if vl == vl and vr == vr:
            w = vl + vr
            if p["w_min"] <= w <= p["w_max"]:
                S["w_cur"] = w
                wm = S.setdefault("w_meas", [])
                wm.append(w)
                del wm[:-15]
            return (vr - vl) / 2.0
        if vl == vl:
            return S["w_cur"] / 2.0 - vl
        if vr == vr:
            return vr - S["w_cur"] / 2.0
        return None

    mir = []

    def lane_correct(F):
        pw = feat["pw"]
        if pw is None or abs(pw) >= 20.0 or not (F != F or F > 1200.0):
            return
        ys_ = feat["yaw_s"]
        step = p["k_wall"] * wrap((ys_ - pw) - S["lane"])
        S["lane"] = wrap(S["lane"] + max(-0.5, min(0.5, step)))
        if int(p["mirror_check"]):
            mir.append((ys_, pw))
            del mir[:-40]
            if len(mir) >= 12:
                ys = np.unwrap(np.radians([q[0] for q in mir]))
                if float(np.ptp(ys)) >= math.radians(4.0):
                    k = float(np.polyfit(np.degrees(ys), [q[1] for q in mir], 1)[0])
                    if k < -0.3:
                        log(dict(ev="lidar_mirrored", slope=round(k, 2), n=len(mir)))
                        end["reason"] = "lidar_mirrored"
                        raise Halt

    # ------------------------------------------------------------------------------------------------ the corner
    def slew_of(deg):
        return abs(deg) / slew_dps

    def plan_arc(sg, wA, wB, F, v_e, R_cap=None):
        """The racing arc for the next corner: dict(R, deg, v, trig) -- R the biggest radius that keeps the island
        corner clear (arc_clear, the clearance c_need: clear_mm + the car's offset toward the island + clear_kv per
        m/s above clear_v0), leaves the straight after it half of (its length - s_min) and starts ahead of the car;
        the speed the ay plan and the drive's caps allow on it.  R_cap: a re-plan near the trigger never grows R (a
        bigger arc's trigger could already be behind the car)."""
        n = S["corners"]
        ci = n % 4                                          # this corridor
        s_min = float(p["s_min1"] if n < 4 else p["s_min"])
        # each lane runs lane_off(w) outside its corridor's centre: hX = the lane's distance from X's OUTER wall
        oA, oB = lane_off(wA), lane_off(wB)
        hA, hB = wA / 2.0 - oA, wB / 2.0 - oB
        w_after = wmap.get((ci + 2) % 4, p["w_assume"])     # the corridor after the next (unknown: 1000, the worst)
        L_next = field - hA - (w_after / 2.0 - lane_off(w_after))    # the next corridor's lane line, lane to lane
        R_hi = min(float(p["R_max"]), 0.5 * (L_next - s_min))
        if n >= 1 and S.get("R_last"):
            w_prev = wmap.get((ci - 1) % 4, p["w_assume"])
            L_here = field - (w_prev / 2.0 - lane_off(w_prev)) - hB
            R_hi = min(R_hi, L_here - s_min - float(S["R_last"]))
        if F == F:
            lead0 = v_e * (0.5 * slew_of(math.degrees(math.atan(wb_at(v_e) / max(R_hi, 1.0)))) + p["lat_s"]) * 1000.0
            R_hi = min(R_hi, F - hB - lead0 - 20.0)
        # the island corner seen from lanes oA / oB outside the centres = the centre-lane geometry of corridors
        # 2 oA / 2 oB wider (arc_clear)
        wA_e, wB_e = wA + 2.0 * oA, wB + 2.0 * oB
        R_lo = R_lock[sg]
        # the clearance the EXECUTED arc keeps (fix review 2026-09-30: planned 130-150, executed 63-98 mm in SIM at
        # 1.3-1.52 m/s): the arc starts where the car IS -- its measured offset toward the island from this lane (SIM
        # median 25-30 mm inside at the trigger, p90 56-61) -- and the trigger / servo errors grow with the arc's speed
        y_now = lateral()
        dy = 0.0 if y_now is None else max(0.0, min(float(p["clear_y_max"]), sg * (y_now - (-sg * oA))))

        def c_need(v_):
            return half_w + float(p["clear_mm"]) + dy + float(p["clear_kv"]) * max(0.0, v_ - float(p["clear_v0"]))
        last = n + 1 >= 4 * int(p["laps"])
        # the last corner ends in the start corridor: the straight after it must hold the finish's stop -- the plan's
        # brake from v_arc to v_finish, its constant-speed run and the coast, before the section's middle (a big last
        # arc left 90 mm at 1.4 m/s: SIM 43 mm past the section, seed 5)
        w_front = (S.get("w_hist") or [p["w_assume"]])[0]
        F_goal = max(F_aim, w_front + front_mm + p["finish_margin"])
        v_f = float(p["v_finish"])
        pre = fin_pre(v_f)

        def arc_v(r_, v_cap=None):
            if not (wbv_on or hold_on):
                dg = min(reach[sg], math.degrees(math.atan(wb / r_)))
                rr = wb / math.tan(math.radians(dg))
                Rcg = math.hypot(rr / 1000.0, cg_x)
                v_ay = math.sqrt(float(p["ay_plan"]) * Rcg) * (rr / 1000.0) / Rcg
                v_ = min(v_ay, v_top_all, drive_cap(v_top_all, sg * dg))
                return dg, rr, max(v_, min(float(p["v_turn"]), v_turn_eff[sg]))
            # V7-3 / V7-4: the steering for R at the arc's own (true) speed, the drive's caps on the servo's angle
            vc_ = 99.0 if v_cap is None else float(v_cap)
            v_ = min(math.sqrt(float(p["ay_plan"]) * r_ / 1000.0), v_top_all, vc_)
            dg, rr = reach[sg], r_
            for _i in range(3):
                wv = wb_at(v_)
                dg = min(reach[sg], math.degrees(math.atan(wv / r_)))
                rr = wv / math.tan(math.radians(dg))
                Rcg = math.hypot(rr / 1000.0, cg_x)
                v_ay = math.sqrt(float(p["ay_plan"]) * Rcg) * (rr / 1000.0) / Rcg
                v_ = min(v_ay, v_top_all, vc_, v_cap_true(sg * dg) if hold_on else drive_cap(v_top_all, sg * dg))
            if v_cap is not None:
                return dg, rr, v_
            return dg, rr, max(v_, min(float(p["v_turn"]), v_turn_eff[sg]))

        def fin_room(rr, v_):
            F_end = field - hA - rr                         # the rear axle's F in the start corridor at the arc's end
            if obs_on and float(p["brake_dec"]) > 0.0 and float(p["fin_lo"]) > 0.0:
                # V7-2 finish: the whole brake from the arc to 0 inside the section, fin_lo past its near line
                F_lo = max(sec_lo + float(p["fin_lo"]), w_front + front_mm + p["finish_margin"])
                return F_end - F_lo - brake_dist(v_, 0.0) - float(p["finish_room"]) >= 0.0
            d_brk = brake_dist(v_, v_f)
            return F_end - F_goal - pre - d_brk - float(p["finish_room"]) >= 0.0

        R = R_lo
        r = min(R_hi, float(R_cap)) if R_cap else R_hi
        v7 = wbv_on or hold_on
        found = None
        while r > R_lo:
            dg_, rr_, v_ = arc_v(r)
            # V7-11: the clearance of the radius the car WILL turn (rr: at speed it cannot turn tighter than its lock
            # radius there, 668 mm at 1.41 m/s), not the one asked
            if arc_clear(rr_ if v7 else r, wA_e, wB_e) >= c_need(v_):
                if not last or fin_room(rr_, v_):
                    R = r
                    found = (dg_, rr_, v_)
                    break
            r -= 10.0
        if found is None and v7 and int(p["slow_fit"]):
            # V7-11: no radius clears the island at the ay speed (a 600 -> 600 corner: SIM random corridors, the lock
            # at 1.41 m/s ran R 668 past the island corner, contact) -> the fastest SLOWER arc that does (the car
            # turns tighter slower: 137 mm effective wheelbase at 0.2 m/s, 218-228 at 0.6-1.0)
            vv = arc_v(R_lo)[2]
            while vv > float(p["slow_fit_min"]) + 1e-6:
                vv = max(float(p["slow_fit_min"]), vv - 0.05)
                dg_, rr_, v_ = arc_v(R_lo, vv)
                if arc_clear(rr_, wA_e, wB_e) >= c_need(v_) and (not last or fin_room(rr_, v_)):
                    found = (dg_, rr_, v_)
                    break
            if found is None:
                found = arc_v(R_lo, float(p["slow_fit_min"]))
            log(dict(ev="slow_fit", n=n + 1, v=round(found[2], 2), R=round(found[1]), wA=round(wA), wB=round(wB),
                     clear=round(arc_clear(found[1], wA_e, wB_e) - half_w), need=round(c_need(found[2]) - half_w)))
        deg, R, v = found if found is not None else arc_v(R)
        return dict(R=R, deg=deg, v=v, trig=hB + R + p["turn_trim"], wA=wA, wB=wB, oA=oA, oB=oB,
                    clear=arc_clear(R, wA_e, wB_e) - half_w, need=round(c_need(v) - half_w), dy=round(dy),
                    last=last)

    def turn(rolling, s_turn, v_arc, ex_min=None, v_in=None, R_arc=0.0):
        """The 90 deg turn at s_turn, rolling or from standing.  Ends exit short of the new lane.  An arc entered above
        its speed (v_in, measured at the trigger) and too near it for the plug brake: the PWM-low brake for the model's
        time tau_plan ln(v_in / v_arc) first -- the drive's ay cap alone only lowers the duty, on the same tau towards
        v_arc (SIM max seed 3 cw: 1.32 m/s into a 1.03 m/s arc, ay 4.5-4.9 of the tip 5.6).  No speed is measured in
        the arc (no scans processed): the time is the model's."""
        sg = S["direction"]
        lane_new = wrap(S["lane"] + 90.0 * sg)
        sl = slew_of(s_turn)
        t_brk = 0.0
        plug_ok = plug and v_in is not None and v_in - v_arc >= plug_dv + 0.02     # the drive's plug takes it
        if not obs_on and rolling and int(p["arc_brake"]) and v_in is not None and \
                v_in > v_arc + float(p["brake_hyst"]) \
                and not plug_ok:
            t_brk = time.monotonic() + tau_plan * math.log(v_in / max(v_arc, 0.1))
        if not rolling:
            wait("TURN", sl + 0.05, 0.0, s_turn)
        t_t, best, best_t, tries = time.monotonic(), 999.0, time.monotonic(), 0
        while True:
            err = abs(wrap(lane_new - robot.yaw))
            ex = max(p["exit_min_deg"] if ex_min is None else ex_min,
                     min(abs(st["yr"]), p["yr_max"]) * (0.5 * sl + p["lat_s"])) + p["exit_add_deg"]
            if err <= ex:
                break
            if err < best - 1.5:
                best, best_t = err, time.monotonic()
            if st.get("imu_dead"):
                best_t = t_t = time.monotonic()  # a hung board's outage is not a blocked turn
            if time.monotonic() - best_t > p["turn_stall_s"] or time.monotonic() - t_t > p["turn_s"] or \
                    (st["stuck"] and time.monotonic() - st["stuck"] > p["stuck_s"]):
                tries += 1
                log(dict(ev="turn_blocked", err=round(err, 1), n=tries, shield=shield.act))
                if tries > 2:
                    end["reason"] = "blocked"
                    raise Halt
                back_off("BACK", p["back_mm"])
                wait("TURN", sl + 0.05, 0.0, s_turn)
                best, best_t, t_t = 999.0, time.monotonic(), time.monotonic()
                st["stuck"] = None
            s_go = s_turn
            if rolling and R_arc > 0.0 and float(p["sh_unw_deg"]) > 0.0:
                st["sh_unw"] = min(1.0, err / float(p["sh_unw_deg"]))
            if rolling and R_arc > 0.0 and wbv_on:
                # V7-4: the steering that makes R_arc at the speed the car has NOW (it understeers more as it speeds up)
                s_go = math.copysign(min(reach[sg], math.degrees(math.atan(wb_at(max(v_coast(), 0.2)) / R_arc))),
                                     s_turn)
            if obs_on and rolling and int(p["arc_brake"]):
                # V7-2 in the arc: above v_arc (+ hyst) -> the PWM-low brake until the observer is back on it
                if ob["v"] > v_arc + float(p["brake_hyst"]):
                    st["arc_brk"] = True
                elif ob["v"] <= v_arc + float(p["brake_end"]):
                    st["arc_brk"] = False
                v_go = 0.0 if st.get("arc_brk") else v_arc
            else:
                v_go = 0.0 if time.monotonic() < t_brk else v_arc
            go(v_go, s_go)
            tick("TURN")
        st["braking"] = False
        st["sh_unw"] = 1.0
        S["lane"] = lane_new
        # the map: the corridor just left (its side-wall width, the median of this straight) and the one entered
        ci = S["corners"] % 4
        wm = S.get("w_meas") or []
        if len(wm) >= 3 and S["corners"] < 4:
            wmap[ci] = float(np.median(wm))
        S["w_meas"] = []
        S["corners"] += 1
        S["w_cur"] = S.get("w_turn") or S["w_cur"]
        if S["corners"] <= 4 and S.get("w_turn"):
            wmap.setdefault(S["corners"] % 4, float(S["w_turn"]))
        S.setdefault("w_hist", []).append(round(S["w_cur"]))
        S.pop("v_side", None)
        mir.clear()
        Fh.clear()
        st["F_last"] = None                             # a new wall ahead: F_valid's dead reckoning starts again
        feat.update(F=float("nan"), side={1: float("nan"), -1: float("nan")}, corner={1: None, -1: None}, pw=None)
        log(dict(ev="corner_done", n=S["corners"], yaw=round(robot.yaw, 1), exit=round(ex, 1),
                 map={k: round(v) for k, v in sorted(wmap.items())}))

    def decide_dir(c1, cm1):
        for sg, c, o in ((1, c1, cm1), (-1, cm1, c1)):
            if not c or c["gap"] < p["gap_min"] or c["u"] - lidar_x > p["corner_reach"]:
                continue
            og = o["gap"] if o else None
            if og is not None and og <= max(p["gap_outer"], 0.6 * c["gap"]):
                return sg
            if og is None:
                # v2: no corner at all on the other side = its wall runs on (the outer wall in the start section):
                # the open side decides -- when that wall is really there (every start vote read 0 on the mat)
                ov = lid(-sg * 90.0, 2.0)
                if ov == ov and ov < p["side_max"]:
                    return sg
        return 0

    def open_side():
        l_o, r_o = lid(75.0, 12.0), lid(-75.0, 12.0)
        lo = l_o if l_o == l_o else 5000.0
        ro = r_o if r_o == r_o else 5000.0
        if max(lo, ro) > 900.0 and abs(lo - ro) > 300.0:
            return (1 if lo > ro else -1), l_o, r_o
        return 0, l_o, r_o

    try:
        log(dict(ev="start", params=p, R_left=round(R_of[1]), R_right=round(R_of[-1]),
                 R_lock=[round(R_lock[1]), round(R_lock[-1])],
                 v_turn_eff=[round(v_turn_eff[1], 2), round(v_turn_eff[-1], 2)],
                 turn_deg=[round(turn_deg[1], 1), round(turn_deg[-1], 1)], lidar_x=lidar_x, shield=shield.enabled,
                 shield_h=shield.c["horizon_mm"], shield_decel=round(shield.decel, 3), decel_by=decel_by,
                 shield_tau=float(p["shield_tau"]), tau_plan=round(tau_plan, 3), v_ceil=round(v_ceil, 3),
                 v_duty=round(v_duty, 3), v_top=round(v_top_all, 3), envelope=env_ok, plug=plug,
                 brake_a=float(p["brake_a"]) if plug else None,
                 duty_max=float(dcfg["duty"]["max"]), max_mps=float(dcfg["max_mps"]),
                 ay_max=float(dcfg.get("ay_max", 0.0) or 0.0), v_tight=float(dcfg.get("v_tight", 0.0) or 0.0),
                 section=[round(sec_lo), round(sec_hi)], F_mid=round(F_mid), F_aim=round(F_aim)))
        if v_top_all < float(p["v"]) - 1e-6:
            if env_ok:
                by_ = "the drive's duty ceiling %.2f at %.2f V (envelope applied)" % (float(dcfg["duty"]["max"]),
                                                                                      robot.battery_v() or 0.0)
            elif int(p["env"]):
                by_ = "the drive's present caps (envelope refused or not offered: see the envelope event)"
            else:
                by_ = "the drive's present caps (no envelope asked: preset %s)" % p["preset"]
            log(dict(ev="v_capped", asked=float(p["v"]), v_top=round(v_top_all, 3), v_duty=round(v_duty, 3),
                     max_mps=float(dcfg["max_mps"]), by=by_))
        if caps0 and (float(dcfg["duty"]["max"]) > caps0["duty_max"] + 1e-6 or plug):
            # above the profile's own duty cap (the mat's measured line) and with the plug brake, the plant / brake are
            # [EST]: the
            # SIM's times are not evidence of the mat's (fix review 2026-09-30, blocker 'brake at speed')
            log(dict(ev="unmeasured", what="duty > 0.35, the plug brake, the stop from > 0.25 m/s, the tip angle",
                     duty_measured=caps0["duty_max"], duty_now=float(dcfg["duty"]["max"]),
                     first="bw test coast 0.5/0.8/1.2/1.5 m/s x3, a plug leg 1.2->0.4 and 1.5->0.8, duty_sweep to 0.5, "
                           "the R0 tilt test -- then mat1 before fast"))
        log(dict(ev="cfg_check", profile=cfg["profile"], diffs=[list(d) for d in cfg["diffs"]],
                 decel_robot=cfg["decel_have"], decel_profile=cfg["decel_want"], decel_by=decel_by, note=cfg["note"]))
        if cfg["diffs"]:
            want_ = {k: w for k, _wh, _h, w in cfg["diffs"]}
            log(dict(ev="lidar_config", diffs=[list(d) for d in cfg["diffs"]],
                     fix="bw apply " + " ".join("'%s=%s'" % (k, str(w).lower() if isinstance(w, bool) else w)
                                                for k, w in want_.items()) + " then bw sys restart_agent --yes"))
            end["reason"] = "lidar_config"
            raise Halt
        # ------------------------------------------------------------------ START: still scans, the lane, direction
        wait("START", float(p["start_wait_s"]), 0.0, 0.0)
        pws = []
        F0s = []
        votes = []
        seen = st["scan_t"]
        t_s = time.monotonic()
        while (len(F0s) < 4 or len(pws) < 3) and time.monotonic() - t_s < p["start_s"]:
            go(0.0, 0.0)
            tick("START")
            if st["scan_t"] != seen:
                seen = st["scan_t"]
                if feat["pw"] is not None and abs(feat["pw"]) < 20.0:
                    pws.append(feat["yaw_s"] - feat["pw"])
                    S["lane"] = wrap(float(np.median(pws)))
                if feat["F"] == feat["F"]:
                    F0s.append(feat["F"])
                lateral()
                votes.append(decide_dir(feat["corner"][1], feat["corner"][-1]))
                for sg_ in (1, -1):
                    c_ = feat["corner"][sg_]
                    if c_ and c_["gap"] >= p["gap_min"] and c_["u"] - lidar_x <= p["corner_reach"] \
                            and p["w_min"] <= c_["w"] <= p["w_max"]:
                        S.setdefault("start_w", {}).setdefault(sg_, []).append(c_["w"])
        F0 = float(np.median(F0s)) if F0s else float("nan")
        if S["direction"] == 0 and int(p["dir_at_start"]):
            vs = [q for q in votes if q]
            if len(vs) >= 2 and all(q == vs[0] for q in vs):
                S["direction"], S["dir_by"] = vs[0], "start_corner"
        log(dict(ev="front0", F0=_r(F0), lane=round(S["lane"], 1), dir=S["direction"] or None, by=S["dir_by"],
                 votes=votes, corner_left=feat["corner"][1], corner_right=feat["corner"][-1], w0=_r(S["w_cur"]),
                 in_section=None if F0 != F0 else bool(sec_lo <= F0 <= sec_hi)))
        dir_votes = []
        while S["corners"] < 4 * p["laps"]:
            # ------------------------------------------------------------------ FOLLOW, the corner measured on the way
            odo0, holds, hold_t = st["odo"], 0, None
            ests, w_jump = [], None
            S["w_next"] = None
            S.pop("w_turn", None)
            S.pop("plan", None)
            cool = p["cooldown_mm"] if S["corners"] else 0.0
            n_next = S["corners"] + 1
            if int(p["map_on"]) and S["corners"] >= 4 and (n_next % 4) in wmap:
                S["w_next"], S["w_by"] = wmap[n_next % 4], "map"           # laps 2-3: known before it is in sight
            if S["corners"] == 0 and S["direction"]:
                ests = [w_ for w_ in S.get("start_w", {}).get(S["direction"], [])][-3:]
                if len(ests) >= 2 and max(ests) - min(ests) <= 100.0:
                    S["w_next"], S["w_by"] = float(np.median(ests)), "corner_start"
            side_prev, other_prev = float("nan"), float("nan")
            first = S["corners"] == 0
            while True:
                tick("FOLLOW")
                new = st.pop("new_scan", False)
                F = F_now()
                sg = S["direction"]
                if new:
                    lane_correct(feat["F"])
                    lateral()
                    if sg == 0:
                        q = decide_dir(feat["corner"][1], feat["corner"][-1])
                        if q:
                            dir_votes.append(q)
                            if len(dir_votes) >= 2 and dir_votes[-1] == dir_votes[-2]:
                                S["direction"], S["dir_by"], sg = q, "corner_gap", q
                                log(dict(ev="direction", dir="ccw (left)" if q > 0 else "cw (right)", by="corner_gap",
                                         F=_r(F), gap_l=_r((feat["corner"][1] or {}).get("gap")),
                                         gap_r=_r((feat["corner"][-1] or {}).get("gap"))))
                    if sg == 0 and first and int(p["cam_dir"]) and cam["dir"]:
                        S["direction"], S["dir_by"], sg = cam["dir"], "cam", cam["dir"]
                        log(dict(ev="direction", dir="ccw (left)" if sg > 0 else "cw (right)", by="cam",
                                 F=_r(F), cam=cam["first"]))
                    elif sg and first and S["dir_by"] == "cam":
                        q = decide_dir(feat["corner"][1], feat["corner"][-1])
                        if q:
                            dir_votes.append(q)
                            if len(dir_votes) >= 2 and dir_votes[-1] == dir_votes[-2] and q != sg:
                                log(dict(ev="direction", dir="ccw (left)" if q > 0 else "cw (right)",
                                         by="corner_gap_over_cam", F=_r(F)))
                                S["direction"], S["dir_by"], sg = q, "corner_gap", q
                                S["w_next"] = None
                                S.pop("plan", None)
                                ests.clear()
                    if sg:
                        c = feat["corner"][sg]
                        if c and c["gap"] >= p["gap_min"] and -100.0 <= c["u"] - lidar_x <= p["corner_reach"] \
                                and p["w_min"] <= c["w"] <= p["w_max"] and st["odo"] - odo0 >= cool:
                            ests.append(c["w"])
                            del ests[:-3]
                            if len(ests) >= 2 and max(ests) - min(ests) <= 100.0:
                                w_m = float(np.median(ests))
                                if S.get("w_by") == "map" and S.get("w_next") is not None:
                                    if abs(w_m - S["w_next"]) > p["map_tol"] and not S.get("plan_fixed"):
                                        log(dict(ev="map_disagree", n=n_next, map=round(S["w_next"]),
                                                 meas=round(w_m)))
                                        S["w_next"], S["w_by"] = w_m, "corner"
                                        S.pop("plan", None)
                                else:
                                    if S.get("plan") is not None and S.get("w_next") is not None and \
                                            abs(w_m - S["w_next"]) > 50.0:
                                        S.pop("plan", None)                      # re-plan on a new width
                                    S["w_next"], S["w_by"] = w_m, "corner"
                        val = lid(sg * 90.0, 2.0)
                        far = not (val == val) or val >= p["side_max"]
                        if (w_jump is None and far and side_prev == side_prev and side_prev < p["side_max"]
                                and F == F and st["odo"] - odo0 >= cool):
                            w_jump = F - lidar_x - 0.035 * side_prev
                            if p["w_min"] <= w_jump <= p["w_max"]:
                                log(dict(ev="island_end", w_jump=round(w_jump), w_est=_r(S.get("w_next")),
                                         n_est=len(ests)))
                                if first:
                                    log(dict(ev="direction_check", by="island_end", dir=sg, from_=S["dir_by"],
                                             agree=True))
                                if S.get("w_next") is None:
                                    S["w_next"], S["w_by"] = w_jump, "jump"
                            else:
                                w_jump = None
                        if not far:
                            side_prev = val
                        if first:
                            ov = lid(-sg * 90.0, 2.0)
                            ofar = not (ov == ov) or ov >= p["side_max"]
                            if ofar and other_prev == other_prev and w_jump is None and F == F and F > 700.0:
                                log(dict(ev="direction_check", by="island_end", dir=-sg, from_=S["dir_by"],
                                         agree=False, F=round(F)))
                                doubt("other_side_end", F=round(F))
                            other_prev = float("nan") if ofar else ov
                if float(p["assume_F"]) > 0.0 and sg and not first and S.get("w_next") is None and F == F \
                        and F <= float(p["assume_F"]) and st["odo"] - odo0 >= cool:
                    # V7-5: the next width still unmeasured this near the wall: the rules' safe width (a late turn)
                    S["w_next"], S["w_by"] = float(p["assume_w"]), "assume"
                    log(dict(ev="assume_w", n=n_next, w=round(S["w_next"]), F=round(F), v=round(v_coast(), 2)))
                geom = S.get("w_next") is not None and sg != 0
                known = geom and not (first and S.get("dir_doubt"))
                w_plan = S["w_next"] if geom else p["w_assume"]
                for legal in (600.0, 1000.0):
                    if abs(w_plan - legal) <= p["snap_mm"]:
                        w_plan = legal     # v2: the plan too (mat 2026-09-30: estimates 938-995 of 1016-1053)
                        break
                v_e = v_x()
                racing = int(p["racing"]) and int(p["rolling"]) and known
                if racing:
                    if S.get("plan") is None:
                        S["plan"] = plan_arc(sg, S["w_cur"], w_plan, F, max(v_e, v_coast()))
                        S["plan_fixed"] = False
                        S["replanned"] = False
                        pl_ = S["plan"]
                        log(dict(ev="plan", n=n_next, R=round(pl_["R"]), deg=round(pl_["deg"], 1),
                                 v_arc=round(pl_["v"], 2), trig=round(pl_["trig"]), wA=round(pl_["wA"]),
                                 wB=round(pl_["wB"]), oA=round(pl_["oA"]), oB=round(pl_["oB"]), clear=round(pl_["clear"]),
                                 need=pl_["need"], dy=pl_["dy"], by=S.get("w_by"), F=_r(F),
                                 v=round(v_e, 2)))
                    pl = S["plan"]
                    # once, near the trigger: the car's offset toward the island THERE (a plan made from the map 2 m
                    # earlier saw the last arc's exit transient) -- R only shrinks, the trigger stays ahead
                    if new and not S.get("replanned") and not S.get("plan_fixed") and F == F \
                            and F <= pl["trig"] + float(p["replan_mm"]) + v_e * 300.0:
                        S["replanned"] = True
                        q_ = plan_arc(sg, S["w_cur"], w_plan, F, max(v_e, v_coast()), R_cap=pl["R"])
                        if q_["R"] < pl["R"] - 5.0:
                            log(dict(ev="replan", n=n_next, R_was=round(pl["R"]), R=round(q_["R"]),
                                     v_arc=round(q_["v"], 2), trig=round(q_["trig"]), clear=round(q_["clear"]),
                                     need=q_["need"], dy=q_["dy"], F=round(F), v=round(v_e, 2)))
                            S["plan"] = pl = q_
                    R, s_deg, v_arc, F_trig = pl["R"], pl["deg"], pl["v"], pl["trig"]
                else:
                    R = R_of[sg] if sg else max(R_of.values())
                    s_deg = turn_deg[sg] if sg else max(turn_deg.values())
                    v_arc = min(p["v_turn"], v_turn_eff[sg]) if sg else min(v_turn_eff.values())
                    F_trig = w_plan / 2.0 + R + p["turn_trim"]
                v_sl = 0.5 * (v_e + min(v_e, v_arc))
                lead = (v_sl * 0.5 * slew_of(s_deg) + v_e * p["lat_s"]) * 1000.0
                F_stop_pt = w_plan / 2.0 + (R_slow[sg] if sg else max(R_slow.values())) + p["turn_trim"] + p["stop_early"]
                if not sg:
                    # v2: only while the direction is unknown -- once it is, w_next is measured on the way in
                    # (mat: the direction came at F 1253, just past this 1270, and corner 1 stopped every run)
                    F_stop_pt = max(F_stop_pt, p["w_assume"] + lidar_x + p["see_mm"])
                if first and sg and int(p["cam_doubt"]) and cam["dir"] and cam["dir"] != sg and not S.get("cam_late") \
                        and not S.get("dir_doubt"):
                    ds = v_e * (p["coast_s"] + p["lag_s"]) * 1000.0
                    if F == F and F - ds < F_stop_pt:
                        S["cam_late"] = True
                        log(dict(ev="dir_doubt_late", by="cam_lines", cam=cam["first"], dir=sg, F=round(F),
                                 stop_pt=round(F_stop_pt), brake_mm=round(ds)))
                        if int(p["cam_halt"]):
                            log(dict(ev="dir_conflict", lidar=sg, cam=cam["dir"], late=True))
                            end["reason"] = "dir_conflict"
                            raise Halt
                    else:
                        doubt("cam_lines", cam=cam["first"])
                        known = False
                        racing = False
                cool_t = cool
                if float(p["cool_map"]) >= 0.0 and S.get("w_by") == "map":
                    cool_t = min(cool, float(p["cool_map"]))      # V7-6: laps 2-3 -- only the trigger waits
                psi_t = wrap(robot.yaw - S["lane"])
                d_psi = 0.0
                if int(p["psi_trig"]) and racing and sg:
                    # V7-7: begun eps already turned toward its side, the arc ends R sin(eps) nearer the island
                    # (the new corridor's lateral = the old wall ahead): the trigger moves by that much
                    d_psi = R * math.sin(math.radians(max(-12.0, min(12.0, sg * psi_t))))
                fire_pred = None
                if int(p["pred_trig"]) and racing and sg and F == F:
                    # V7-10: where THIS arc would end if it began now -- from the steering the car already has (the
                    # lane controller's, toward or away from the turn: the servo's slew to the arc is shorter or
                    # longer), the heading it already has, and the slew's own advance; fire when that end is the new
                    # lane.  The fixed lead (slew from 0, heading 0) coupled the old lane's offset into the new one 1:1
                    # (SIM: 131 mm outside at the trigger -> 149 mm toward the island at the end, alternating)
                    wv_ = max(wb_at(max(v_e, 0.2)), 1.0)
                    s_now = sg * float(st["cmd"][1])
                    k0 = math.tan(math.radians(s_now)) / wv_
                    k1 = 1.0 / max(R, 1.0)
                    T_ = max(0.0, s_deg - s_now) / slew_dps + float(p["lat_s"])
                    d_ = v_e * 1000.0 * T_
                    e0 = math.radians(max(-15.0, min(15.0, sg * psi_t))) * float(p["pred_e0"])
                    dps = d_ * 0.5 * (k0 + k1)
                    e1 = e0 + dps
                    F_end = F - d_ * math.cos(e0 + 0.5 * dps) - R * (1.0 - math.sin(e1))
                    fire_pred = F_end <= F_trig - R
                    st["pred"] = dict(F_end=round(F_end), s_now=round(s_now, 1), e0=round(math.degrees(e0), 1),
                                      T=round(T_, 3), vo=round(v_e, 2), k=round(ok_["k"], 3))
                if F == F and st["odo"] - odo0 >= cool_t and abs(psi_t) < 20.0:
                    fire = (F <= F_trig + lead - d_psi) if fire_pred is None else fire_pred
                    if known and int(p["rolling"]) and fire:
                        S["w_turn"] = w_plan
                        S["plan_fixed"] = True
                        log(dict(ev="corner", n=S["corners"] + 1, rolling=True, racing=bool(racing), F=round(F),
                                 trig=round(F_trig), lead=round(lead), v=round(v_e, 2), v_arc=round(v_arc, 2),
                                 w_next=round(w_plan), by=S.get("w_by"), psi=round(wrap(robot.yaw - S["lane"]), 1),
                                 y=_r(lateral()), n_est=len(ests), R=round(R), deg=round(s_deg, 1), dir=sg,
                                 pred=st.get("pred") if fire_pred is not None else None))
                        S["turn_cmd"] = (sg * s_deg, v_arc, R if racing else 0.0, v_e)
                        break
                    if (not known or not int(p["rolling"])) and F <= F_stop_pt:
                        S["w_turn"] = None
                        break
                # the speed: the straight's top, then the brake to the arc's speed at the trigger (the exponential
                # brake), or to v_tt at the stop point when the geometry is not known yet
                v_top = v_top_all if sg else min(p["v_start"], v_top_all)
                F_pl = F if F == F else r_hi
                if known and int(p["rolling"]):
                    v_goal = v_arc
                    v_allow = brake_v(F_pl - (F_trig + lead), v_arc)
                    if v_coast() <= v_arc + 0.02:
                        # nothing to brake: brake_v's lag term is for a brake still to come.  Without this the plan
                        # commanded 0 in the last ~190 mm before 105 of 131 top-speed arcs (true speed -0.25 m/s
                        # median, ~0.05-0.1 s a corner, the nose pitched down while F is read: fix review 2026-09-30)
                        v_allow = max(v_allow, v_arc)
                else:
                    v_tt = min(p["v_turn"], v_turn_eff[sg]) if sg else min(v_turn_eff.values())
                    v_goal = v_tt = min(v_tt, 0.9 * p["stop_early"] / 1000.0 / max(p["coast_s"], 0.05))
                    v_allow = brake_v(F_pl - F_stop_pt, v_tt)
                v_c = speed_cmd(v_allow, v_top, v_goal)
                go(v_c, follow_steer(v_c))
                now = time.monotonic()
                hold_t = (hold_t or now) if (shield.enabled and shield.act == "brake" and v_c > 0.0) else None
                stuck_long = bool(st["stuck"]) and now - st["stuck"] > p["stuck_s"]
                if (hold_t and now - hold_t > p["hold_s"]) or stuck_long:
                    holds += 1
                    log(dict(ev="held", front=_r(F), n=holds, shield=shield.act, stuck=bool(st["stuck"])))
                    if time.monotonic() - st.get("reopen_t", -99.0) > p["reopen_gap_s"] and hasattr(robot, "board_reopen"):
                        st["reopen_t"] = time.monotonic()  # a held car may sit on a frozen servo: reset before backing off
                        log(dict(ev="board_reset_on_hold", ok=bool(robot.board_reopen())))
                    if holds > p["holds_max"]:
                        end["reason"] = "blocked"
                        raise Halt
                    back_off("BACK", p["back_mm"], align=True)
                    hold_t, st["stuck"] = None, None
            if S.get("w_turn") is None:
                # ---------------------------------------------------------------- STOP + MEASURE (geometry unknown)
                st["braking"] = False
                wait("STOP", p["settle_s"], 0.0, 0.0)
                vts = []
                F, _f = still_feat("MEASURE", 3, votes=vts)
                sg = S["direction"]
                if sg == 0:
                    q = decide_dir(feat["corner"][1], feat["corner"][-1])
                    q2, l_o, r_o = open_side()
                    if q:
                        sg, S["dir_by"] = q, "stop_corner"
                    elif q2:
                        sg, S["dir_by"] = q2, "open_side"
                    else:
                        sg, S["dir_by"] = 1, "default"
                    S["direction"] = sg
                    log(dict(ev="direction", dir="ccw (left)" if sg > 0 else "cw (right)", by=S["dir_by"],
                             left=_r(l_o), right=_r(r_o)))
                elif S["corners"] == 0 and S.get("dir_doubt"):
                    vs = [q for q in vts if q]
                    q = vs[0] if len(vs) >= 2 and all(x == vs[0] for x in vs) else 0
                    q2, l_o, r_o = open_side()
                    new_dir, how = (q, "stop_corner") if q else ((q2, "open_side") if q2 else (sg, "kept"))
                    before = sg
                    if new_dir != sg:
                        sg = S["direction"] = new_dir
                        S["dir_by"] = "recheck_" + how
                        S["w_next"] = None
                    winner = "lidar_changed" if before != sg else "lidar_kept"
                    log(dict(ev="direction_recheck", doubt=S["dir_doubt"], before=before, after=sg, how=how,
                             votes=vts, left=_r(l_o), right=_r(r_o), cam=cam["dir"], winner=winner,
                             cam_agree=None if not cam["dir"] else cam["dir"] == sg))
                    if int(p["cam_halt"]) and cam["dir"] and cam["dir"] != sg:
                        log(dict(ev="dir_conflict", lidar=sg, cam=cam["dir"]))
                        end["reason"] = "dir_conflict"
                        raise Halt
                c = feat["corner"][sg]
                side_open = lid(sg * 90.0, 2.0)
                if S.get("w_next") is not None:
                    w_turn, how = S["w_next"], S.get("w_by")
                elif c and c["gap"] >= p["gap_min"] and p["w_min"] <= c["w"] <= p["w_max"]:
                    w_turn, how = c["w"], "corner_still"
                elif side_open != side_open and F == F:
                    w_turn, how = float(np.clip(F - lidar_x + 60.0, p["w_min"], p["w_max"])), "passed"
                else:
                    # v2: never turn blind -- creep on the lane until the wall ahead is seen again
                    odo_s = st["odo"]
                    while F != F and st["odo"] - odo_s < p["seek_mm"]:
                        go(p["v_creep"], follow_steer())
                        tick("SEEK")
                        F = F_now()
                    wait("SEEK", 0.3, 0.0, 0.0)
                    F, _f = still_feat("SEEK", 2, 0.6)
                    if F != F:
                        log(dict(ev="no_front", seek=round(st["odo"] - odo_s)))
                        end["reason"] = "no_front"
                        raise Halt
                    c = feat["corner"][sg]
                    if c and c["gap"] >= p["gap_min"] and p["w_min"] <= c["w"] <= p["w_max"]:
                        w_turn, how = c["w"], "corner_seek"
                    else:
                        w_turn, how = p["w_assume"], "assumed_seen"
                for legal in (600.0, 1000.0):                    # v2: the Open rule draws 600 or 1000 only
                    if abs(w_turn - legal) <= p["snap_mm"]:
                        w_turn = legal
                ft = w_turn / 2.0 + R_slow[sg] + p["turn_trim"]      # v2: a standing turn runs slow
                t_c = time.monotonic()
                back_ok = F == F and F < ft - p["reverse_mm"]
                t_max = 2.0 + (abs(F - ft) / 1000.0 / p["v_creep"] if F == F else 0.0)
                while F == F and time.monotonic() - t_c < t_max:
                    v_e = v_coast()
                    if F > ft:
                        if F - v_e * p["coast_s"] * 1000.0 - ft <= p["tol_mm"]:
                            break
                        go(min(p["v_start"], max(p["v_creep"], 1.2 * (F - ft) / 1000.0)), follow_steer())
                    elif back_ok and ft - F > p["tol_mm"]:
                        go(-p["v_creep"], 0.0)
                    else:
                        break
                    tick("CORRECT")
                    F = F_now()
                wait("CORRECT", 0.3, 0.0, 0.0)
                F, _f = still_feat("CORRECT", 2, 0.6)
                S["w_turn"] = w_turn
                log(dict(ev="corner", n=S["corners"] + 1, rolling=False, F=_r(F), trig=round(ft), w_next=round(w_turn),
                         by=how, dir=sg, R=round(R_slow[sg])))
                S["R_last"] = R_slow[sg]
                arcs.append(dict(n=S["corners"] + 1, R=round(R_slow[sg]), v=round(p["v_turn"], 2), rolling=False))
                turn(False, sg * turn_deg[sg], p["v_turn"])
            else:
                s_turn, v_arc_, R_, v_in_ = S.pop("turn_cmd")
                S["R_last"] = R_ if R_ > 0 else R_of[S["direction"]]
                arcs.append(dict(n=S["corners"] + 1, R=round(S["R_last"]), v=round(v_arc_, 2), rolling=True))
                turn(True, s_turn, v_arc_, float(p["arc_exit_deg"]) if R_ > 0 and float(p["arc_exit_deg"]) > 0 else None,
                     v_in_, R_)
            # ---------------------------------------------------------------- the new lane: how the turn ended
            odo_t = st["odo"]
            v_exit = max(float(arcs[-1]["v"]), min(p["v_start"], v_top_all)) if S["direction"] else p["v_start"]
            while st["odo"] - odo_t < float(p["exit_mm"]) and S["corners"] < 4 * p["laps"]:
                tick("FOLLOW")
                if st.pop("new_scan", False):
                    lane_correct(feat["F"])
                go(v_exit, follow_steer(v_exit))
                if shield.enabled and shield.act == "brake":
                    hold_t = hold_t or time.monotonic()
                    if time.monotonic() - hold_t > p["hold_s"]:
                        log(dict(ev="held", front=_r(F_now()), n=0, shield=shield.act, where="exit"))
                        back_off("BACK", p["back_mm"])
                        hold_t = None
                else:
                    hold_t = None
            y = lateral()
            log(dict(ev="lane_entry", n=S["corners"], y=_r(y), psi=round(wrap(robot.yaw - S["lane"]), 1),
                     w=_r(S["w_cur"]), v=round(st.get("v_est", 0.0), 2)))
        # -------------------------------------------------------------------- FINISH in the start section
        w_front = (S.get("w_hist") or [p["w_assume"]])[0]
        F_stop = max(F_aim, w_front + front_mm + p["finish_margin"])
        hold_t, retries, where = None, 0, "planned"
        v_f = float(p["v_finish"])
        odo_f = st["odo"]
        pre = fin_pre(v_f)
        while True:
            tick("FINISH")
            if st.pop("new_scan", False):
                lane_correct(feat["F"])
                c = feat["corner"][S["direction"]]
                if c and c["gap"] >= p["gap_min"] and p["w_min"] <= c["w"] <= p["w_max"]:
                    w_front = c["w"]
                    F_stop = max(F_aim, w_front + front_mm + p["finish_margin"])
            F = F_now()
            d_stop = brake_dist(v_coast(), 0.0) if (obs_on and float(p["brake_dec"]) > 0.0) else \
                v_coast() * p["coast_s"] * 1000.0
            if F == F and (F - d_stop <= F_stop or F <= sec_lo + p["tol_mm"]):
                break
            if st["odo"] - odo_f > float(p["finish_max_mm"]):
                where = "odo_cap"                  # v2: the wall ahead never read: stop by distance, never a 4th lap
                break
            if F == F:
                v_c = speed_cmd(brake_v(F - F_stop - pre, v_f), v_top_all, v_f)
            else:
                v_c = v_f                             # v2: no wall ahead read -> the finish speed, not the top one
            go(v_c, follow_steer(v_c))
            if shield.enabled and shield.act == "brake" and v_c > 0.0:
                hold_t = hold_t or time.monotonic()
                if time.monotonic() - hold_t > p["hold_s"]:
                    inside = F == F and sec_lo <= F <= sec_hi
                    log(dict(ev="held", front=_r(F), n=retries, shield=shield.act, where="finish", inside=inside))
                    if inside or retries >= int(p["finish_retry"]):
                        where = "held"
                        break
                    retries += 1
                    back_off("BACK", p["back_mm"])
                    hold_t = None
            else:
                hold_t = None
        st["braking"] = False
        if obs_on and int(p["stop_adaptive"]):
            # the car at rest, not a fixed wait: the observer's brake to < 0.03 m/s (+ stop_extra_s: the mat's brake
            # may be the weaker 1.6 m/s^2), at most 1.5 s -- the clock ends with the car standing
            t_st = time.monotonic()
            while time.monotonic() - t_st < 1.5 and (ob["v"] > 0.03 or time.monotonic() - t_st < 0.1):
                go(0.0, 0.0)
                tick("STOP")
            wait("STOP", float(p["stop_extra_s"]), 0.0, 0.0)
        else:
            wait("STOP", float(p["stop_wait_s"]), 0.0, 0.0)
        F, _f = still_feat("STOP", int(p["stop_scans"]), 0.6)
        inside = F == F and sec_lo <= F <= sec_hi
        log(dict(ev="finish", F=_r(F), F_stop=round(F_stop), F_mid=round(F_mid), F0=_r(F0), w_front=round(w_front),
                 section=[round(sec_lo), round(sec_hi)], inside=inside, how=where))
        end["reason"] = "finish" if inside or F != F else "finish_outside"
    except Halt:
        pass
    finally:
        try:
            bot.drive(0.0, 0.0)
        except Exception:
            pass
    secs = round(time.monotonic() - t0, 1)
    log(dict(ev="shield_stats", n_slow=shield.n_slow, n_steer=shield.n_steer, n_brake=shield.n_brake))
    if obs_on:
        # V7-1 on the mat: err = the front wall's slope minus the observer (m/s): |median| > 0.1 or k far from 1 = the
        # fitted line (obs_slope / obs_icpt) is off for this car / pack -> docs/OPEN_V7.md, the ladder's knobs
        er_ = np.array(ob["err"]) if ob["err"] else np.zeros(1)
        log(dict(ev="observer", n=int(ob["n"]), err_med=round(float(np.median(er_)), 3),
                 err_p90=round(float(np.percentile(np.abs(er_), 90)), 3), k=round(float(ok_["k"]), 3),
                 slope=o_sl, icpt=o_ic, brake_dec=float(p["brake_dec"])))
    if loop_dt:
        # R6 (loop latency on the Pi): the arc's exit threshold assumes lat_s; a p99 above ~50 ms at 1.5 m/s eats the
        # island margin -- then lat_s = the measured p99 and clear_mm up (fix review 2026-09-30)
        ld = np.array(loop_dt) * 1000.0
        sa = np.array(scan_age) * 1000.0 if scan_age else np.zeros(1)
        log(dict(ev="loop_stats", n=int(ld.size), loop_ms_p50=round(float(np.percentile(ld, 50)), 1),
                 loop_ms_p99=round(float(np.percentile(ld, 99)), 1), loop_ms_max=round(float(ld.max()), 1),
                 scan_age_ms_p50=round(float(np.percentile(sa, 50)), 1),
                 scan_age_ms_p99=round(float(np.percentile(sa, 99)), 1), lat_s=float(p["lat_s"]),
                 hz=float(p["loop_hz"])))
    if fvalid:
        # pass (R1 / R3): >= 90 % valid while accelerating and braking; else lock the suspension or gate F by a_x
        log(dict(ev="F_valid", **{k: dict(valid=v[0], due=v[1], short=v[2],
                                           pct=round(100.0 * v[0] / max(v[1], 1), 1)) for k, v in fvalid.items()}))
    log(dict(ev="arcs", arcs=arcs, map={k: round(v) for k, v in sorted(wmap.items())}))
    wsim = getattr(getattr(robot, "sim", None), "world", None)
    if getattr(robot, "mock", False) and wsim is not None:
        # the simulator's truth for the report only (never read by the race): the grip's slides, the rollover
        log(dict(ev="sim_dyn", slides=getattr(wsim, "slides", None), slide_s=round(float(getattr(wsim, "slide_s", 0.0)
                                                                                           or 0.0), 2),
                 rolled=bool(getattr(wsim, "rolled", False)), ay_max=round(float(getattr(wsim, "ay_max", 0.0)), 2)))
    log(dict(ev="end", reason=end["reason"], corners=S["corners"], laps=round(S["corners"] / 4.0, 2), seconds=secs,
             direction=S["direction"], dir_by=S["dir_by"], cam_first=cam["first"], dir_doubt=S.get("dir_doubt"),
             cam_agree=None if cam["dir"] is None or not S["direction"] else cam["dir"] == S["direction"]))
    return dict(reason=end["reason"], corners=S["corners"], laps=round(S["corners"] / 4.0, 2),
                direction=S["direction"])
