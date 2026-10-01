# Systems thinking and engineering decisions

Appendix C criterion 4. The constraints we worked under, the decisions we made and the data behind them, what we
learned on the mat, how the programs evolved, and the risks that remain.

Tags: **MAT** measured on the car; **FIT** fitted from mat run logs; **SIM** simulator. A run ID names the log file of
that run. Pack voltages are the first reading in each log.

## Summary

- We changed platform on 2026-09-23 and had three weeks to the event. That, and the rule that the car may only use
  parts the team already owns, shaped most decisions: no encoder, no new electronics.
- The car's position comes from the lidar, so the lidar had to see the walls; its speed comes from a fitted motor
  model, so the model had to match the mat.
- Four measured lessons changed the programs most: the steering trim and the unequal lock, the battery compensation,
  the stop at Q, and the camera's field of view. Each is below with its numbers and run IDs.

## Constraints

| Constraint | Effect on the design |
|---|---|
| Rules 11.3, 11.5, 11.13: one driving axle, no independently connected drive motors | the kit's two-motor rear axle could not be used; WLtoys chassis with one motor and differentials |
| Rule 11.10: no radios during a round | race mode blocks every radio before the program starts |
| Rules 9.10, 9.11: one power switch, one start button | one latching switch, one push button on GPIO17 |
| Only parts the team owns | no wheel encoder; the motor runs on the MD13S we already had |
| Walls 100 mm high | lidar scan plane at 50 mm |
| Corridors 600 or 1000 mm, drawn at random | the Open program measures each next width and plans unknown ones on 600 mm |
| Parking lot 1.5 × the car's length | the park is planned on the car's true footprint and the real lock of each side |
| Three weeks from platform change to event | one change per mat run, each version a new file, a simulator calibrated to the mat |

## How the subsystems depend on each other

| From | To | What can go wrong |
|---|---|---|
| Lidar plane height | pose, sign positions, shield | a plane above the walls sees the room, not the field |
| Steering trim | every arc, every park leg | an untrimmed car turns less on one side |
| Motor model | speed observer, braking point, park legs | a model that ignores the pack's effect moves the park by tens of millimetres |
| Camera pitch and field of view | sign colour, map look | a sign outside the frame is never read; a pitch error moves the map projection |
| Gyro integration | pose between scans | a dropped sample turns into a heading error in a corner |
| Board link (RRC Lite) | servo and IMU | a stalled stream leaves the wheels at their last angle |

## Decision log

| We chose | Instead of | Because |
|---|---|---|
| WLtoys chassis, one motor, differentials | the kit's chassis | the kit drives each rear wheel with its own motor, which rules 11.5 and 11.13 do not allow |
| MD13S driven from the Pi's GPIO | the RRC Lite's motor ports | those ports run a speed loop on an encoder; without one the board drives the motor to 100 % and a stop does not stop it |
| Software PWM at 490 Hz | the Pi 5's hardware PWM | the hardware PWM channel returned an error on GPIO12 |
| Lidar scan plane at 50 mm | the kit's mount | on the kit's mount the lidar read 2.5–4.3 m past the field and missed a pillar 25 cm ahead (MAT, 2026-09-22) |
| Sign colour by bearing to the lidar pillar | range from the camera | bearing pairing does not depend on the pitch; the depth stream over-read a pillar at 1.25 m as 1.43 m |
| Fitted speed line 5.276 × (duty − 0.1012) | the profile's first line (4.49, 0.1187) | rms error 0.075 m/s against 0.271 m/s on the same 1599 lidar speeds (FIT) |
| A speed observer for the Open braking plan | the drive's own estimate | the drive's estimate read 1.24–1.53 times under the car at 7.1 V and the lidar speed is stale in arcs; the pre-corner brake barely fired |
| One constant-radius arc per corner | full lock from the lane centre | full-lock corners run at 0.35–0.5 m/s |
| Effective wheelbase taken at the present speed | one value, 210 mm | with one value every arc at 1.3–1.5 m/s came out about 30 % too wide (SIM) |
| Plan an unmeasured corridor on 600 mm | stop and measure | the stop cost about 2.5 s; a late turn into 1000 mm is safe, an early one into 600 mm is not |
| Steering lock per side, 30° left and 22° right | 22° both ways | corners asked 34–44° while the program capped 22°, and the car blocked (see below) |
| Battery hold at 7.0 V in the Obstacle program | the drive's battery compensation | the compensation over-corrected this car (see below) |
| Stop at Q on the braking distance | a fixed 50 mm lead | the car came off the last corner at 0.8–0.9 m/s and stopped 159 mm past Q (see below) |
| A new file for each program version, proven ones frozen by SHA-256 | editing the program in place | every mat result stays reproducible; a regression can be compared with the exact file that worked |

## Lessons measured on the mat

### Steering trim and the unequal lock

**Symptom.** `open_v2_25s` drove 25.2 s counter-clockwise (run `20260930-235056.569-open_fast_v2`, 7.01 V) but
blocked after one corner when run clockwise (11.6 s). Clockwise runs lost about half of each right turn;
counter-clockwise runs hid the problem because left turns still reached their radius.

**Measurement.** On 2026-10-01 the wheels pointed straight ahead at a servo command of −4°, all day (MAT). The fit over
the mat logs found the same bias: +4.05° in corners at 0.6–1.2 m/s (FIT, 2694 samples).

**Change.** `open_v4` adds −4° to every steering command. Result: 12 of 12 corners driven without stopping in both
directions: clockwise 23.3 s (run `20261001-001646.802-open_v4`, 6.40 V) and 26.3 s
(`20261001-005127.677-open_v4`, 7.28 V), counter-clockwise 28.6 s (`20261001-005755.103-open_v4`, 7.18 V).

**Second effect: the lock.** The servo is limited to ±26°. On top of the −4° trim that gives 30° at the wheels to the
left and 22° to the right (MAT). The first Obstacle programs capped the steering at 22° both ways. At the corners every
stop asked for 34–44°, and `obs_v2` blocked twice (runs `20261001-164201.458-obs_v2` at 0.33 laps and
`20261001-164524.976-obs_v2` at 0.60 laps). `obs_v3` gave each side its own cap and smaller corner radii (300 and
340 mm) and made the first full Obstacle run with a park: 69.4 s, all 4 corners in, nearest lot edge 12.7 mm (run
`20261001-165354.604-obs_v3`, 7.45 V). `obs_v12` then planned the lot exit and the park on the per-side lock too: 7 legs
instead of 9 each way counter-clockwise, 73.3 s to 68.2 s (run `20261001-202156.652-obs_v12`, 7.00 V).

**Checked again.** `BEST_OPEN_14s` with the trim at −4.6° drove 15.0 s against 14.7 s at −4° (runs
`20261001-161351.882-open_v7` and `20261001-161301.412-open_v7`), so −4° stayed.

### Battery compensation that over-corrected

**Mechanism.** The drive code multiplies the duty by v_ref / V_pack with v_ref = 8.0 V, assuming the motor slows on a
low pack. The fit over 20 mat runs found that this car's speed per duty does not fall with the pack (exponent 0.0,
FIT); with the compensation a 6.4 V run went faster than a 7.3 V run for the same command. At 6.1 V the compensation
multiplies the duty by 1.31; in SIM that made the true speed 38 % higher.

**Symptom on the mat.** `obs_v16` (run `20261001-204736.982-obs_v16`, 6.33 V at the start) drove fast laps (lap 3 done at 40.0 s) and stopped 34 mm from Q, but the park legs and the creep onto Q overshot by 30–150 mm
and the park ended with 1 of 4 corners in the lot.

**At the other end.** On a full pack the compensation lowers the duty. `obs_v7` at 8.09 V (run
`20261001-190131.030-obs_v7`) lost its along-track position in lap 2 and blocked at 1.22 laps; the same code at
7.25 V drove all three laps.

**Change.** `obs_v17` holds the compensation at its 7.0 V value for the whole run, whatever the pack, and restores it at
the end. Result at 8.25 V: 55.8 s, all signs on the correct side, full park (run `20261001-222840.106-obs_v17`). The
Open program is not affected: it plans on the observer's true speed.

### The stop at Q: short, then long

The park starts from Q, the point where the lot exit ended. The car must stop there within 8 mm along the line, 12 mm
across and 2.5° in heading.

**Short.** In the 8 runs from `obs_v3` to `obs_v7` that tried to park, every realign after the first stop ended 35–76 mm
short of Q (15 realigns, MAT), and 4 of the 8 parks were full. The forward drive along the line stops 0.25 s × v before
the target (about 50 mm at 0.2 m/s) to leave room for coasting, but the PWM-low brake stops the car dead, so it stayed
there. The creep that should close the gap only ran when the lateral and heading errors were already in tolerance, and
they never were. `obs_v11` creeps onto Q after every realign: the realign errors fell to 15 and 18 mm and the park was
full (run `20261001-201055.680-obs_v11`, 73.3 s, 7.10 V).

**Off the line.** Until `obs_v12` the first stop at Q came 39–51° off the line's heading (63° in `obs_v10`, whose lot model was wrong), because the lap path already
bends into the next corner there; the realigns took about 10 s. `obs_v13` rides Q's line straight for the final
approach: 57.6 s with a full park (run `20261001-203331.245-obs_v13`, 6.80 V).

**Long.** In that run the car came off the last corner at 0.8–0.9 m/s and stopped 159 mm past Q with the fixed 50 mm
lead, which cost a 4.5 s realign. Fitting the stop gave 2.4 m/s². `obs_v16` fires the stop at
v² / (2 × 2.4) + v × 0.08 s: the first stop was 34 mm off (run `20261001-204736.982-obs_v16`). In the 55.8 s run the
first stop was 59 mm off and two realigns brought it in; the park started 5.1 s after lap 3.

**Rejected.** A longer look-ahead on Q's line (`obs_v18`, team note; that run's log was not kept) reached Q 25 mm
aside; each realign left 16 mm and then 14 mm across, over the 12 mm tolerance, and the run took 62.4 s. The lateral
tolerance is the one that binds.

### The camera could not see the sign

**Symptom.** In run `20261001-174628.799-obs_v5` the green sign after corner 2 (seat 2.0) was passed on the wrong side in
all three laps. The lidar had that seat in every lap; the camera never gave it one colour vote (the lap maps list five
signs). The other five signs were read at the usual 1.2–1.7 m, so the pitch was not the cause, and the simulator's
rendered camera failed the same way (seed 1), so the light was not the cause either.

**Mechanism.** On the straight before corner 2 the car aims to the right to pass the red sign 1.2 on its right. The
green sign then stays 30–60° off the camera axis, outside the ±29.3° field of view, and enters the frame only about
250 mm away, beside the car. With no colour the car took the default lane.

**Change.** `obs_v7` reads an unread sign on purpose: the nose turns toward it when it sits just past the frame edge,
and the program counts pixels where the map projects the sign. In run `20261001-195904.068-obs_v7` the car glanced at
that seat at a bearing of 33.2° and 1560 mm, all six signs had a colour by the end of lap 1, and all 18 passes were on
the correct side.

**What we kept from it.** When a sign is passed on the wrong side, first check whether its seat got any colour vote. If
not, it is a field-of-view problem, not a path or colour-threshold problem.

### Camera pitch

The model's wedge is 16°; on the car we measured 19.61° (MAT, 2026-10-01). In SIM, 1° of pitch error moved the pose of
our camera-based localisation by a median 116 mm, and the kit's camera mount moved about 10° in one day before we had
the printed tower. The tower and the keyed wedge hold the camera; the pitch check before every session measures it.
The Obstacle program pairs colours by bearing, so the pitch only affects the map look's projection.

## Version history

The Open Challenge, on the mat:

| Program | Change | Result | Run |
|---|---|---|---|
| `open_v2_57s`, `open_v2_46s`, `open_v2_27s` | lidar laps; then the racing line | 57.5 s, 45.9 s, 27.3 s | frozen file notes |
| `open_v2_25s` | faster arcs (planned 1.5 m/s²) | 25.2 s CCW, 7.01 V; blocked clockwise | `20260930-235056.569-open_fast_v2` |
| `open_v4` | steering trim −4°, arcs on a 210 mm effective wheelbase | 23.3 s CW, 6.40 V; 28.6 s CCW, 7.18 V | `20261001-001646.802-open_v4`, `20261001-005755.103-open_v4` |
| `open_v5_18s` | larger arcs, duty up to 0.5, shield on the measured brake | 18.8 s CW, 7.10 V | `20261001-010507.423-open_v5` |
| `open_v6` (`BEST_OPEN`) | brake and lead on the lidar speed | 18.7 s CCW, 7.04 V | `20261001-011031.386-open_v6` |
| `open_v7`, preset `l1` | speed observer, constant-deceleration brake, held arc speed, wheelbase curve, unknown corridor on 600 mm | 16.0 s CCW, 8.06 V | `20261001-161218.085-open_v7` |
| **`BEST_OPEN_14s`** (`open_v7`, preset `l2`) | planned lateral acceleration 2.6 m/s², straights to 1.8 m/s | **14.7 s CCW, 8.05 V** | `20261001-161301.412-open_v7` |
| `open_v7`, preset `fast7` | 3.0 m/s², straights to 2.0 m/s | 16.3 s CCW, 8.06 V: the shield braked 18 times near the walls | `20261001-161418.641-open_v7` |

The Obstacle Challenge, on the mat, all counter-clockwise, start in the lot:

| Program | Change | Result | Run |
|---|---|---|---|
| `obs_v1` | first program: lidar pose, sign seats, lane per sign, lot exit and park | blocked at 0.1 laps: a 0.14 m/s creep never moved the car, and the lot read as red signs | `20261001-163348.547-obs_v1` |
| `obs_v2` | outer-wall band, creep at 0.20 m/s | blocked at 0.33 and 0.60 laps: 22° cap | `20261001-164201.458-obs_v2`, `20261001-164524.976-obs_v2` |
| `obs_v3` | lock per side, corner radii 300 and 340 mm | **first full park: 69.4 s**, 7.45 V; repeat 71.2 s, 7.36 V | `20261001-165354.604-obs_v3`, `20261001-165946.260-obs_v3` |
| `obs_v4` | faster exit legs (0.16 m/s) | 67.6 s parked, 7.17 V; two more runs ended with 3 and 1 corners in the lot | `20261001-171309.381-obs_v4` |
| `obs_v5` | 6° of lock kept in reserve | 72.6 s parked, 6.99 V; repeat: green 2.0 wrong side, 3 corners in | `20261001-173009.016-obs_v5`, `20261001-174628.799-obs_v5` |
| `obs_v7` | glance and map look | all signs read; Q stop short, 1 corner in | `20261001-195904.068-obs_v7` |
| `obs_v10` | lot refined on every lap | 0 corners in: one limitation face is ambiguous between two lot positions 340 mm apart | `20261001-200413.723-obs_v10` |
| `obs_v11` | creep onto Q after each realign | 73.3 s parked, 7.10 V | `20261001-201055.680-obs_v11` |
| `obs_v12` | exit and park on the lock of each side (7 legs) | 68.2 s parked, 7.00 V | `20261001-202156.652-obs_v12` |
| `obs_v13` | leave the lot turned; ride Q's line | 57.6 s parked, 6.80 V | `20261001-203331.245-obs_v13` |
| `obs_v16` | stop at Q on the braking distance | Q within 34 mm; 1 corner in at 6.33 V | `20261001-204736.982-obs_v16` |
| **`obs_v17`** | battery hold at 7.0 V | **55.8 s parked, 8.25 V** | `20261001-222840.106-obs_v17` |

Of the 18 Obstacle runs on 2026-10-01, 8 ended with a full park. The others are the development failures in the table.

## What did not work

| Attempt | Result | Why |
|---|---|---|
| Faster Open preset `fast7` on the mat | 16.3 s, slower than `l2` | the shield braked 18 times near the walls |
| Trim −4.6° on `BEST_OPEN_14s` | 15.0 s | −4° was already right |
| Lot refined on every lap (`obs_v10`) | 0 corners in | a single limitation face fits two lot positions 340 mm apart |
| Quicker park legs and a shorter back-up (SIM) | lost the park in SIM | about half of the exit time is servo swings (0.66 s per full reversal at 85.6 °/s); a faster servo would gain more than faster legs |
| Longer look-ahead to Q (`obs_v18`) | 62.4 s | the lateral error converged too slowly |
| A three-move park | not possible in this lot | the planner's minimum is 5 legs at 30° both sides |

## Risks and mitigations

| Risk | Mitigation |
|---|---|
| Both main programs have been run on the mat counter-clockwise only | SIM both ways; on the mat the Open fallbacks cover both directions, `BEST_OPEN` 18.7 s CCW and `open_v5_18s` 18.8 s CW (the same settings without `v_meas_max`); clockwise mat runs of the main programs before the event |
| Our mat had only 1000 mm corridors | SIM on random 600 / 1000 corridors; unknown widths are planned on 600 mm |
| Camera pitch changes after a knock | keyed wedge; pitch check before each session |
| Venue light changes the colours | hue check on arrival; colour by bearing; map look with looser limits |
| Lot length set by the judges differs from our model | the lot's position is refined from the limitations seen on the last lap |
| Servo or board link stalls | watchdogs reopen the board; standing steering limited to 12° |
| Pack voltage | battery hold in the Obstacle program; runs started at 7.4 V or more |
| Race-mode start path | rehearse the full race-mode start on the practice mat before the event |
| Mass and height only from CAD | weigh and measure the car before the event; both have a large margin to rules 11.1 and 11.2 |

## Next steps

1. Clockwise mat runs of `BEST_OPEN_14s` and `obs_v17`.
2. Runs with 600 mm corridors on the practice mat.
3. Full race-mode rehearsal: radios off, start button only.
4. Weigh the car and measure its height.
5. Re-centre the servo horn for a more equal lock, then re-measure trim and lock before any program uses it.

<sub>[Back to the README](../README.md)</sub>
