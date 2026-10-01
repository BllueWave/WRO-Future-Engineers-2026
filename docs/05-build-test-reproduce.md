# Build, test and reproduce

Appendix C criterion 5. What to buy and print, how to set up the software, how we test, every mat result with its
run ID, and how to repeat each number on these pages.

Tags: **MAT** measured on the car; **FIT** fitted from mat run logs; **SIM** simulator; **CAD** body model; **EST**
estimate or listing value.

## Bill of materials

| Part | Model | Role | Mass |
|---|---|---|---|
| Chassis | WLtoys 1:28, model 284131, four-wheel drive, with its stock tyres, 130-size motor, micro servo, propshaft and differentials | drive and steering | 125 g with motor and servo (EST) |
| Computer | Raspberry Pi 5 with the Active Cooler | perception, planning, control | 66 g (EST) |
| Controller board | Hiwonder RRC Lite (STM32F407), stock firmware | servo, IMU, buzzer, keys; powers the Pi | 32 g (vendor) |
| Motor driver | Cytron MD13S (6–30 V, 13 A) | PWM and direction drive of the motor | 20 g |
| Lidar | LDROBOT LD19 with its USB serial adapter | 360° range scan | 47 g without cable (vendor) |
| Camera | Angstrong HP60C RGB-D, USB-C | colour of signs and limitations | 56.2 g (EST) |
| Battery | Hiwonder 7.4 V 2200 mAh 10C (2 × 18650, protection board), DC 5.5 × 2.5 mm charge lead, 8.4 V 2 A charger | power | 105 g (EST) |
| Power switch | 16 mm latching push switch | the one power switch | 12 g (EST) |
| Start button | 12 mm momentary push button | the one start button | 5 g (EST) |
| Cables | USB-C to USB-C rated 5 A (board to Pi); USB cables for lidar and camera; Grove cable to the MD13S; jumper for the body strap | | |
| Printed body | 5 PETG parts, see [Models](../Models/README.md) | body, lidar bay, camera mount | 157 g of PETG (EST) |

## Printed parts

Five parts, PETG, 0.4 mm nozzle; settings and files in [Models](../Models/README.md). Print the tower with its brim
and the support enforcer under its foot; parts 02, 05 and 07 need no supports. Print time for parts 02, 04, 05 and 07 is
about 353 min (EST, slicer).

## Wiring

Pin table and schematic: [Schemes](../Schemes/README.md). The short version:

1. Pack positive → power switch → one splice to the RRC Lite power input and the MD13S V+. Pack negative → one splice
   to both.
2. RRC Lite 5 V USB-C output → Pi 5 USB-C, with a 5 A cable.
3. Pi GPIO12 (pin 32) → MD13S PWM, GPIO16 (pin 36) → MD13S DIR, pin 34 → MD13S signal ground.
4. Start button between GPIO17 (pin 11) and GND (pin 9). Body strap between GPIO5 (pin 29) and GND (pin 30).
5. Steering servo on RRC Lite PWM port 3, with the port's supply jumper at 5 V (measure 4.8–5.2 V before plugging the
   servo).
6. RRC Lite, LD19 and HP60C to the Pi's USB ports.

## Software set-up

Commands run on the laptop from `src/`, in Git Bash or cmd; `py -3` is the Windows Python launcher (`python3`
elsewhere). Every command is described in [src/README.md](../src/README.md#running-on-the-robot).

### Raspberry Pi 5, once

1. On a fresh Raspberry Pi OS Lite (64-bit) card, run `os/setup_bluewave_os.sh`: it installs `python3-numpy`,
   `python3-opencv`, `python3-serial` and `python3-lgpio` from apt, the udev rule, and the development and race
   services. On Hiwonder's stock card, run `os/install_on_stock.sh` instead; it keeps Hiwonder's container, which holds
   the HP60C driver our code reads the colour stream from.
2. The RRC Lite keeps Hiwonder's firmware. Nothing is compiled or flashed: the programs are Python and load at the start
   of a run.

### Each session

1. `py -3 tools/bw.py deploy` copies `bluewave/`, `programs/`, `race/` and `profiles/` to the car.
2. `py -3 tools/bw.py body wltoys_bw2` applies the car's calibration file and checks the body strap.
3. With the car still: `py -3 tools/bw.py preflight pitch` with one sign 500 mm ahead of the bumper, then
   `py -3 tools/bw.py preflight` until every row says GO.
4. Development runs: `py -3 tools/bw.py run BEST_OPEN_14s --wait-button` or `py -3 tools/bw.py run obs_v17 --wait-button`,
   then `py -3 tools/bw.py pull RUN_ID` and `py -3 tools/bw.py analyze latest`.
5. After any change to the car, repeat the calibrations in [Power and sensors](02-power-and-sensors.md#calibration)
   and write the new numbers into `profiles/wltoys_bw2.json`.

### Race mode

1. Choose the programs and the round:
   `py -3 tools/bw.py apply race.program= race.programs.open=BEST_OPEN_14s race.programs.obstacle=obs_v17 race.challenge=open`
   (`race.challenge=obstacle` for the Obstacle round).
2. `py -3 tools/bw.py mode race`, then switch the car off.
3. Place the car and switch on. At boot `os/bluewave-mode.sh` runs `rfkill block all` and starts `race/race_main.py`,
   which loads the program, computes what it can before the car moves, and beeps twice (READY). Nothing moves until the
   start button is pressed and released.
4. The run is written to `runs/<time>-race-<program>.jsonl` on the car. A long press of the board's second key returns
   to development mode (radios on).

### Laptop tools

| Command (from `src/`) | What it does |
|---|---|
| `py -3 -m pip install -r requirements.txt` | the simulator's packages (numpy, OpenCV, SciPy for the plant fit) |
| `py -3 tools/bw.py status` | pack voltage and board state |
| `py -3 tools/bw.py stop` | stops the car |
| `py -3 tools/sim_run.py <program> --profile wltoys_bw2 --mat1001 ...` | the simulator with the mat-fitted plant |
| `py -3 tools/fit_plant.py --bias-from corners` | refits the plant from run logs |

## Bench check before a round

| Step | Pass when |
|---|---|
| Pack voltage, car still | 7.4 V or more |
| Gyro bias, car still | measured and stored for the session |
| Camera pitch: one sign centred, its near face 500 mm from the front bumper, car still | GO |
| Lidar: scan direction and offset match the calibration | no halt at program start |
| Colours under the venue light | red, green and magenta limitations each found on a test frame |
| Start position | in the start section (Open) or in the lot parallel to the wall (Obstacle) |
| Race mode | two beeps, then still until the start button |

The [test sheet](test-sheet.md) has one row per round.

## How we test

| Level | Tool | What it gives |
|---|---|---|
| Component tests on the car | duty sweep, coast, turn radius, servo sweep, camera pitch, lidar tilt, field check, still check | the calibration numbers in `profiles/wltoys_bw2.json` |
| Unit tests | development test suite, run on 2026-10-01: test_units 23 of 23, test_body 19 of 19, test_fast_drive 11 of 11, test_wltoys 29 of 29 | the code paths without the car |
| Calibrated simulator | `tools/sim_run.py --mat1001`: the plant fitted to 20 mat run logs of 2026-09-30 and 2026-10-01, a rendered camera, the field with chosen corridors and signs | program changes compared before a mat run |
| Simulation queue | at most 3 real-time simulations at once, started only below 80 % CPU; each result stamped TRUSTED or SUSPECT | results that are not distorted by a loaded machine |
| Mat runs | one change per run, same start pose, known pack; stop after two similar failures or a pack under 7.4 V; about 60 s between runs | the results below |
| After each run | the run log; a crash map (where on the field and in which program state an incident happened); the IMU black box (contact or jolt) | what to change next |

The simulator had to pass a gate before we used it: it must reproduce mat results it was not fitted to choose. It
reproduced `open_v2_25s` blocking clockwise after one corner and `open_v4` and `BEST_OPEN` finishing clean. The old,
unfitted plant had passed `open_v2_25s` clockwise, which the mat did not.

## Results on the mat

Practice mat, all corridors 1000 mm, 2026-09-30 and 2026-10-01. Times are the program's own clock. Pack = first
voltage in the log.

### Open Challenge (3 laps, 12 of 12 corners, stop inside the start section in every row)

| Program | Run ID | Direction | Time | Pack |
|---|---|---|---|---|
| **`BEST_OPEN_14s`** (`open_v7`, preset `l2`) | `20261001-161301.412-open_v7` | CCW | **14.7 s** | 8.05 V |
| `open_v7` `l2`, trim −4.6° | `20261001-161351.882-open_v7` | CCW | 15.0 s | 8.05 V |
| `open_v7` `l1` | `20261001-161218.085-open_v7` | CCW | 16.0 s | 8.06 V |
| `open_v7` `fast7` | `20261001-161418.641-open_v7` | CCW | 16.3 s | 8.06 V |
| `open_v6` (`BEST_OPEN`) | `20261001-011031.386-open_v6` | CCW | 18.7 s | 7.04 V |
| `open_v5_18s` | `20261001-010507.423-open_v5` | CW | 18.8 s | 7.10 V |
| `open_v4` | `20261001-001646.802-open_v4` | CW | 23.3 s | 6.40 V |
| `open_v4` | `20261001-005127.677-open_v4` | CW | 26.3 s | 7.28 V |
| `open_v4` | `20261001-005755.103-open_v4` | CCW | 28.6 s | 7.18 V |
| `open_v2_25s` | `20260930-235056.569-open_fast_v2` | CCW | 25.2 s | 7.01 V |

The 14.7 s run in numbers: top speed 1.39 m/s, peak horizontal acceleration 0.56 g, control loop 99th percentile
32.0 ms, speed observer median error 0.01 m/s, stop at 1541 mm from the wall ahead inside the 1182–1957 mm start
window.

### Obstacle Challenge (start in the lot, 3 laps, 6 signs, parallel park; counter-clockwise; all 4 corners in the lot in every row)

| Program | Run ID | Time | Pack |
|---|---|---|---|
| **`obs_v17`** | `20261001-222840.106-obs_v17` | **55.8 s** | 8.25 V |
| `obs_v13` | `20261001-203331.245-obs_v13` | 57.6 s | 6.80 V |
| `obs_v4` | `20261001-171309.381-obs_v4` | 67.6 s | 7.17 V |
| `obs_v12` | `20261001-202156.652-obs_v12` | 68.2 s | 7.00 V |
| `obs_v3` | `20261001-165354.604-obs_v3` | 69.4 s | 7.45 V |
| `obs_v3` | `20261001-165946.260-obs_v3` | 71.2 s | 7.36 V |
| `obs_v5` | `20261001-173009.016-obs_v5` | 72.6 s | 6.99 V |
| `obs_v11` | `20261001-201055.680-obs_v11` | 73.3 s | 7.10 V |

Sign layout on the mat in these runs: 1.0 green, 1.2 red, 2.0 green, 2.2 red, 3.0 green, 3.2 green. On 2026-10-01,
8 of 18 Obstacle runs ended with a full park; the other 10 were development runs listed in
[Engineering decisions](04-engineering-decisions.md#version-history).

## Simulation results

SIM, calibrated plant `plant_mat1001`, one run each, all TRUSTED by the simulation queue.

| Program, direction | MAT | SIM on the mat's layout (4 × 1000 mm) | SIM on random corridors |
|---|---|---|---|
| `open_v2_25s` CW | blocked after 1 corner, 11.6 s | blocked after 1 corner, 9.2 s | blocked after 1 corner, 9.3 s (seed 1) |
| `open_v4` CCW | 28.6 s | 30.8 s, 0 contacts | 30.8 s (seed 5) |
| `open_v4` CW | 23.3 s / 26.3 s | 30.6 s, 0 contacts | 38.8 s (seed 1) |
| `BEST_OPEN` CCW | 18.7 s | 20.1 s, 0 contacts | 20.2 s (seed 5) |
| `BEST_OPEN` CW | not run on the mat (the CW 18.8 s run was `open_v5_18s`) | 18.2 s, 0 contacts | blocked at 31.1 s on 1000/600/600/600 (seed 1) |

`open_v7` with its `fast7` preset in the same simulator, seeds 1 to 6 in both directions on the mat's layout: 12 of 12 clean runs, 14.0–14.4 s (SIM); with the `l2` preset we race, 14.7 s counter-clockwise and 14.9 s clockwise (SIM). On random corridors it stayed clean, with 600 mm to 600 mm corners slower (up to 25 s, SIM). The last row matters for the event: in SIM `BEST_OPEN` blocked on narrow corridors, while `open_v7` stayed clean.

## Versioning

- Each program version is a separate file with one named change in its header. Proven versions are frozen: their
  SHA-256 is recorded and the file is never edited again; a faster version is a new file.
- Printed parts follow the same rule: a part that has been printed is never changed; a new version is a new file.
- The calibration file keeps the date and the test behind each measured number in its notes.
- Releases of this repository are tagged; the release notes name the programs and the body version of each tag.

## How to reproduce each number

| Number | Where it comes from | How to repeat it |
|---|---|---|
| 14.7 s Open | run `20261001-161301.412-open_v7` | `py -3 tools/bw.py run BEST_OPEN_14s` in the start section; the run log's `end` event |
| 55.8 s Obstacle with park | run `20261001-222840.106-obs_v17` | `py -3 tools/bw.py run obs_v17` with the car in the lot; the `end` event (reason `parked`, corners in 4) |
| Speed line 5.276 × (duty − 0.1012), rms 0.075 m/s | `profiles/plant_mat1001.json` | `py -3 tools/fit_plant.py --bias-from corners --no-save` over the run logs |
| Effective wheelbase 137–281 mm | same file, `wb_curve` | same fit; the 137 mm point from the turn-radius test at 0.2 m/s |
| Brake 2.08 m/s² | same file, `brake_decel` | same fit; stop test from 0.48 m/s for the ≥ 1.6 m/s² lower bound |
| Steering 30° left, 22° right, trim −4° | profile notes, program headers | servo sweep and turn-radius test; a straight run at each trim |
| Camera pitch 19.61° | run parameters of 2026-10-01 | `py -3 tools/bw.py preflight pitch` with a sign 500 mm ahead |
| Lidar 598–599 rpm, loop 99th percentile 32.0 ms | telemetry and `loop_stats` of the runs above | any run log |
| Size 224.8 × 111.0 × 157.2 mm | the BW-2 CAD model | measure the car (to be done before the event) |
| SIM tables | the simulator with `--mat1001` | `py -3 tools/sim_run.py open_v7 --profile wltoys_bw2 --mat1001 --open --corridors 1000,1000,1000,1000 --seed 5` |

<sub>[Back to the README](../README.md)</sub>
