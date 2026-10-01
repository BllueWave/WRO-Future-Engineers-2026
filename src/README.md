# Source code

This folder is the software of our car: a WLtoys 1:28 chassis (284131) carrying a Raspberry Pi 5, a Hiwonder RRC Lite
board (steering servo, IMU, buzzer), an LDROBOT LD19 lidar, an Angstrong HP60C RGB-D camera and a Cytron MD13S motor
driver. Everything is Python 3. It runs on the Pi 5; the laptop tools deploy it, start runs, pull the run logs back and
run the same program files in a simulator.

The tree is the code as deployed for our practice-mat session of 1 October 2026, the session in which both competition
programs below set their times. One default differs from that deployment: race mode's default programs
(`race.programs` in `bluewave/params.py`) now name the two programs below. The Arduino code of our previous car stays
in the git history of this repository.

The design reasoning is in [docs/03-software-and-strategy.md](../docs/03-software-and-strategy.md); the test method and
results are in [docs/05-build-test-reproduce.md](../docs/05-build-test-reproduce.md).

## Layout

```
src/
  programs/            the competition programs, one file each, loaded by name
    BEST_OPEN_14s.py     Open Challenge
    obs_v17.py           Obstacle Challenge
    _library.json        the console's program list: rank, status and the team's mat notes (Arabic)
  bluewave/            the robot package: drivers, perception, localisation, safety, run logs, simulator, console
  race/race_main.py    race mode: radios off, one START button, then the program
  profiles/            wltoys_bw2.json = this car's calibration; plant_mat1001.json = the simulator's mat-fitted plant
  tools/               laptop side: bw (deploy, run, test, pull), sim_run, fit_plant, lib_push
  os/                  Raspberry Pi setup: systemd services, udev rule, sudo rules
  requirements.txt     laptop packages
  requirements-robot.txt  Pi packages
```

| folder | files | lines |
|---|---|---|
| `bluewave/` | 43 Python files + `ui/index.html` (the web console) | 20,772 Python |
| `programs/` | 2 programs + the library seed | 1,998 + 1,945 |
| `race/` | `race_main.py` | 128 |
| `tools/` | 11 Python files | 4,296 |
| `os/` | 10 files | 306 |

`src/` has the same layout as the `mentorpi/` folder of our robot repository, so the commands below and the relative
paths the tools use resolve from `src/`. Some code comments name design notes of the robot repository (`docs/*_SPEC.md`,
`plans/*.json`, `3d/...`) that are not part of this tree.

### The `bluewave` package

| module | what it does |
|---|---|
| `hw.py` | `Robot`: the one object a program drives. The simulator builds the same object, so a program file runs unchanged on the car and on the laptop. |
| `rrc.py` | RRC Lite serial driver at 1,000,000 baud: servo pulses, IMU stream, buzzer, keys. |
| `motor.py` | MD13S drive: PWM on GPIO12, DIR on GPIO16, software PWM through `lgpio`. A separate guardian process zeroes the PWM if the program process dies. |
| `speed.py` | Speed without a wheel encoder: the lidar pose plus a duty-to-speed line fitted on the mat. |
| `lidar.py`, `lidar_perc.py` | LD19 driver; walls, pillars and parking limitations from one revolution. |
| `loc.py`, `globloc.py` | Particle filter; whole-field position search from still views. |
| `field.py`, `seats.py` | The field map with the 24 legal sign positions; each sign's position from the lidar and its colour from the camera. |
| `camera.py`, `vision.py` | Frame grabber (the HP60C colour stream) and the colour masks. |
| `park.py` | Parallel-parking legs planned on the car's true footprint. |
| `shield.py` | Lidar safety layer: every drive command is checked against the free space before it reaches the motor. |
| `blackbox.py`, `runlog.py` | 50 Hz IMU record and the run file (one JSON line per event or telemetry sample). |
| `analyze.py`, `report_html.py` | After a run: where and why the car touched something, as text and as an HTML report. |
| `params.py`, `paramstore.py`, `progmeta.py` | Robot parameters with defaults, history and undo; profiles; program metadata and presets. |
| `mock.py` | The simulator: lidar, camera image, IMU and drive with the same interfaces as the hardware. |
| `tests.py`, `tests_cal.py` | Component and calibration tests run from the console or `bw test` (duty sweep, coast, turn radius, camera pitch, lidar tilt, field check). |
| `agent.py`, `hub.py`, `*_api.py`, `ui/` | Development server (FastAPI, port 8000) and the web console. Used in dev mode only. |
| `depth_bridge.py`, `ros_bridge.py` | Run inside Hiwonder's ROS 2 container: the HP60C depth stream, and read-only ROS topics for Foxglove or RViz. |

## The competition programs

Best runs on our practice mat, 1 October 2026 (all four corridors 1000 mm). Times are the program's own clock.

| challenge | file | result | direction | pack at start | run |
|---|---|---|---|---|---|
| Open | `programs/BEST_OPEN_14s.py` | 3 laps in 14.7 s, 12/12 corners, stopped inside the start section, no contact | counter-clockwise | 8.05 V | `20261001-161301.412-open_v7` |
| Obstacle | `programs/obs_v17.py` | 3 laps and a parallel park in 55.8 s, all four car corners inside the lot | counter-clockwise | 8.25 V | `20261001-222840.106-obs_v17` |

`BEST_OPEN_14s.py` is `open_v7` with its preset `l2` (planned lateral acceleration 2.6 m/s^2, speed cap 1.8 m/s) made
the defaults; the mat run was filed under `open_v7`. Neither program has been run clockwise on the mat yet.

**Open, `BEST_OPEN_14s.py`.** The lidar scan is rotated into the corridor frame by the gyro yaw. Each corner is one
constant-radius 90-degree arc from lane to lane, as wide as the inner wall allows, driven at
sqrt(planned lateral acceleration x radius). Lap 1 measures every corridor's width; laps 2 and 3 plan each corner from
that map before it is in sight. The speed comes from the mat-fitted duty line, corrected by the change of the front-wall
distance; braking to the arc speed follows a constant-deceleration plan (1.8 m/s^2). The car stops inside the start
section on the planned front-wall distance.

**Obstacle, `obs_v17.py`.** Pose: every scan's wall points are fitted to the known field walls (point-to-line ICP,
seeded by the gyro and the speed). Signs: lidar pillar clusters are snapped to the 24 legal positions and coloured by the
camera, matched by bearing, so the camera pitch does not enter the colour decision; red is passed on its right, green
on its left. Path: one lane per sign, checked against signs, inner wall and lot, followed by pure pursuit from the rear
axle. The lot exit and the park are planned on the measured steering lock of each side (30 degrees left, 22 degrees
right). The park starts with a braked stop at a fixed point beside the lot and is counted as parked only when all four
corners are inside. The program does not implement a direction change after the second lap.

### The program contract

Each program is one file with a `DEFAULTS` dict, optional `PRESETS`, and

```python
def run(robot, params, log, stop, hook=None): ...
```

- `robot` is `bluewave.hw.Robot` (on the car or in the simulator).
- `params` is layered: `DEFAULTS` < the chosen preset < the robot's per-program parameters < the run's own `k=v`.
- `log(dict)` writes one JSON event to the run file.
- `stop` is a `threading.Event`; the program returns when it is set.

The run file lands in `runs/<time>-<program>.jsonl` on the robot, with the parameters and robot parameters in its first
line, so a run can be traced to the exact numbers it ran on.

## Car interface

Values from `profiles/wltoys_bw2.json`; the full harness is in [Schemes/](../Schemes/).

| signal | Pi 5 header pin | GPIO / port | note |
|---|---|---|---|
| motor PWM | 32 | GPIO12 | 490 Hz software PWM (`drive.pwm.hw_pwm` 0) |
| motor DIR | 36 | GPIO16 | DIR low = forward (`drive.pwm.dir_forward_high` false) |
| START button | 11, GND 9 | GPIO17 | internal pull-up; read as the RRC's KEY1 |
| body strap | 29, GND 30 | GPIO5 | jumper present = this chassis (`bodyid.py`) |
| steering servo | - | RRC Lite PWM port 3 | centre 1441 us, 22.4 us per degree |
| RRC Lite | USB | `/dev/rrc` | udev rule in `os/99-bluewave.rules` |

Drive safety in the profile: PWM-low brake on every stop, a 200 ms command watchdog (`drive.pwm.watchdog_ms`), motor cut
after 0.8 s of stall (`drive.stall_s`).

## Running on the robot

The commands run on the laptop from `src/`, in Git Bash or cmd (PowerShell rejects the `<` redirections used in
`os/`). The robot address and the optional console token come from the environment, never from a file in this tree:

| variable | default | meaning |
|---|---|---|
| `BW_HOST` | `bluewave.local` | robot address |
| `BW_PORT` | `8000` | dev server port |
| `BW_USER` | `bluewave` | SSH user (key login; `bw.py` uses `~/.ssh/bluewave_ed25519` when it exists) |
| `BW_TOKEN` | empty | console token, if the robot sets one |

1. **Set up the Pi once.** On a fresh Raspberry Pi OS Lite (64-bit) card: `os/setup_bluewave_os.sh` (installs the
   packages, the udev rule, the dev and race services). On Hiwonder's stock card: `os/install_on_stock.sh` (keeps
   Hiwonder's container for the HP60C driver and adds the dev server).
2. **Deploy:** `py -3 tools/bw.py deploy` copies `bluewave/`, `programs/`, `race/` and `profiles/` to the robot and
   restarts the dev server. It refuses while a program drives the car.
3. **Select the car's profile:** `py -3 tools/bw.py body wltoys_bw2` (applies the profile, checks the body strap and the
   lidar fingerprint, restarts the server).
4. **Calibrate with the car still:** `py -3 tools/bw.py preflight pitch` with one pillar 500 mm ahead of the bumper,
   then `py -3 tools/bw.py preflight` until every row says GO (battery, board, gyro bias, camera, lidar, body).
   The camera pitch is measured on the car, not taken from the CAD: the profile carries the 16-degree wedge angle, the
   car ran on 19.61 degrees measured on 1 October 2026.
5. **Run:** `py -3 tools/bw.py run BEST_OPEN_14s --wait-button` or `py -3 tools/bw.py run obs_v17 --wait-button`; the car
   waits for its START button. Ctrl-C stops it.
6. **Read the run back:** `py -3 tools/bw.py runs`, `py -3 tools/bw.py pull RUN_ID`,
   `py -3 tools/bw.py analyze latest`.

`py -3 tools/bw.py help` lists the other commands (live telemetry, component tests, parameters, presets, logs).

### Race mode

Rule 11.10 wants every radio off during a run, and rules 9.10 and 9.11 allow one power switch and one start button.
Race mode is that path, on the card set up by `os/setup_bluewave_os.sh`:

1. Choose the programs and the challenge:
   `py -3 tools/bw.py apply race.program= race.programs.open=BEST_OPEN_14s race.programs.obstacle=obs_v17 race.challenge=open`
   (`race.challenge=obstacle` for the Obstacle round).
2. `py -3 tools/bw.py mode race`, then power-cycle the car.
3. At boot `os/bluewave-mode.sh` blocks every radio (`rfkill block all`) and starts `race/race_main.py`. It loads the
   program and computes what can be computed before the car moves, beeps twice (READY), and waits for START. Then it
   runs the program and writes `runs/<time>-race-<program>.jsonl`.
4. Holding the second RRC key long returns to dev mode (radios on, dev server started).

## Running in the simulator

```sh
py -3 -m pip install -r requirements.txt
py -3 tools/sim_run.py BEST_OPEN_14s --profile wltoys_bw2 --mat1001 --open --corridors 1000,1000,1000,1000 --seed 5
py -3 tools/sim_run.py obs_v17 --profile wltoys_bw2 --mat1001 --lot --lot-start --seed 2
```

- `--profile wltoys_bw2` builds the simulated car from this car's profile (body, sensors, steering limits).
- `--mat1001` replaces the simulator's drive and steering with `profiles/plant_mat1001.json`, fitted by
  `tools/fit_plant.py` to 20 mat run logs of 30 September and 1 October 2026: speed = 5.276 m/s per unit duty x
  (duty - 0.1012), rms error 0.075 m/s over 1,599 lidar speed samples; PWM-low brake 2.08 m/s^2; speed lag 0.56 s;
  steering bias +4.05 degrees; effective wheelbase 137 mm at 0.2 m/s rising to 280.7 mm at 1.7 m/s (2,694 corner
  samples). The car understeers more as it goes faster, and the fitted curve is what the simulated car steers with.
- `--open` draws 600 / 1000 mm corridors from the seed (`--corridors S,E,N,W` fixes them); `--lot --lot-start` places
  the parking lot and starts the car in it; `--cw` runs clockwise; `--seeds 1-8` runs a batch, one line each;
  `--runfile DIR` writes a run file the analyzer reads; `--fault '{...}'` injects a fault mid-run (camera pitch jump, lidar dropout, battery sag, frozen camera, stuck wall
  reading).
- Each run prints the program's events, one JSON line each, then a `RESULT` line: reason, laps, seconds, contacts,
  wrong-side passes, park corners inside, maximum lateral acceleration.

The simulator finds bugs and regressions before the mat. It does not prove a program: a result counts when the car
repeats it on the mat.

## Configuration and secrets

No network name, password, key or token is stored in this tree. The robot address, SSH user and console token come
from the environment variables above or from an untracked `.robot.json` beside `tools/` (listed in `.gitignore`). SSH
uses key login only.
