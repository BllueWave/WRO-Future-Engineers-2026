# Systems thinking and engineering decisions

Our decision log has 17 entries: 8 settled on the mat, 5 in simulation, 2 by both, 1 by our measurements and the rulebook, 1 by a compile.
Each entry names the alternative we tried and the number that decided it.

<sub>[Back to the README](../README.md) · Criterion 4 of 5 · Previous: [Software and strategy](03-software-and-strategy.md) · Next: [Build, test and reproduce](05-build-test-reproduce.md)</sub>

## Evidence

| Claim | Where to check |
|---|---|
| Ten reverted versions (8-17) led to our named-mechanism rule | [Version history](#version-history) |
| The corner vote: 37/40 against 21/40 (simulation) | [Decision log](#decision-log); corner code at [Open_Challenge.ino lines 228-232](../src/Open_Challenge/Open_Challenge.ino#L228-L232) |
| The 6 September build never parked because its lap total was zeroed 50 degrees rotated | Fix at [Obstacle_Challenge.ino line 409](../src/Obstacle_Challenge/Obstacle_Challenge.ino#L409) |
| Obstacle v20 completes three laps in 12 of 120 simulated rounds | [Simulation results](05-build-test-reproduce.md#simulation-results) |
| Flash is the tightest constraint: 29,604 of 32,256 B | Compile output, [expected sizes](05-build-test-reproduce.md#expected-sizes) |
| On 14 Sep 2026 the car did not move, for four reasons we found and fixed | [Version history](#version-history) |

## System map

<p align="center">
  <img src="diagrams/system_overview.svg" width="760" alt="System overview: three HC-SR04 sonars, the Pixy2 camera, the BNO055 IMU and the start switch feed the Arduino Uno R3, which drives the Cytron MD13S and drive motor and the steering servo, all powered by the 2S LiPo">
</p>

### One Obstacle round across the subsystems

1. Power on. `setup()` centres the servo, holds the motor at 0, tries the BNO055 three times and starts the Pixy2. One servo wiggle means every part answered.
2. WAIT. The three sonars and the IMU print every 400 ms. A change on A2 starts the round.
3. Side pick. The side sonars average for 500 ms. The shorter side is the outer wall, which seeds the software's turn direction.
4. EXIT. The servo alternates full locks, the MD13S drives 200 ms legs at PWM 18, and the BNO055 ends the exit at 50 degrees. The exit rotation goes into the lap total.
5. LAPS. Side sonars give the lane error for the servo. The front sonar triggers corners and checks pillar range. The Pixy2 picks a ladder step. The BNO055 counts corners, which also tells the lap-1 map which straight the car is on.
6. Handover. The BNO055 total is within 8 degrees of 1080 and the Pixy2 has been clear for 5 frames.
7. APPROACH. The front sonar tracks the far wall to the stop mark, the wall-side sonar holds the lane, and the BNO055 holds the heading.
8. PARK. The front sonar measures the real step length and the along-lot position. The BNO055 closes every arc. A geometry model decides whether each move fits in the lot.
9. DONE. Motor 0, servo 90.

### Where the subsystems interact

| Chain | What happens | What we did |
|---|---|---|
| Sonar angle → lost-echo copy → camera view | The inner 40-degree unit goes silent in a 1000 mm corridor, the copy makes the lane error zero, the car rides 250-300 mm off the outer wall, and the next pillar sits outside the 60-degree view (simulation) | Obstacle: not solved, see [What failed](#what-failed) |
| Servo library → PWM pins → motor driver | The Servo library's Timer1 removes PWM from D9 and D10; sonars and SPI use D5, D6, D11 | D3 is the one PWM pin left, so a PWM plus direction driver (MD13S) |
| Motor break-away → slow moves → battery | PWM 15 does not start the car; park and exit moves are slow | Kicks at PWM 55; park steps measured before use; arcs closed on the IMU |
| IMU heading → lap logic → lot exit | The exit leaves the car about 50 degrees rotated | Turn total never reset; exit rotation seeded (fix 26) |
| Flash → features → debug text | 91 % of flash used by the Obstacle sketch | A new feature must fit, or replace debug print text |

## Constraints

| Constraint | Number | Effect on the design |
|---|---|---|
| Vehicle size and mass | ≤ 300 × 200 × 300 mm, ≤ 1.5 kg (rules 11.1, 11.2, p.23) | 200 × 125 mm fits with room; mass not measured |
| Drivetrain | 4 wheels, one driving axle, one steering actuator (11.3, p.23) | RC chassis with Ackermann steering |
| Parking lot | 1.5 × car length × 200 mm (p.8) | 300 mm lot: 50 mm per end, 75 mm across |
| Flash | 32,256 B on the Uno | Obstacle sketch 29,604 B (91 %), or 31,278 B (96 %) if the Pixy2 library keeps its two Zumo files |
| RAM | 2,048 B | Obstacle globals 818 B; a 316 × 208 camera frame (65,728 px) could never fit, so the Pixy2 sends block lists |
| PWM pins | 6 on the Uno, 5 taken by sonars, servo timer and SPI | One motor channel on D3 |
| Camera view | 60 degrees horizontal (Pixy2 2.0 figure) | The first pillar after a corner can be out of view |
| Side sonar geometry | About 40 degrees from the nose | Only a bearing-free law (left minus right) works; corner ties need a vote |
| Start procedure | One power switch, one start button (9.10, 9.11, p.17) | A2 start input; `PRACTICE 0` in `src/` |
| Calibration | No sensor calibration in preparation time (9.9, p.17) | All calibration in practice time |
| Toolchain | We upload from the Arduino IDE | One `.ino` per challenge; sizes checked with the IDE's own compiler |
| Time | The v20 sketches were finished on 15 Sep 2026, days before travel | A hardware change such as a rear sonar would need its own test time on the mat |

## Decision log

| Decision | Alternative we tried | Number that decided it | Evidence |
|---|---|---|---|
| Lane law on left minus right with the 40-degree side sonars | Laws tuned for 90-degree flank sonars | The flank-sonar laws drove into the walls on the real car; the left-minus-right law keeps the lane with the same sonars | Mat |
| Corner direction from a vote of counted corners | Per-frame left minus right with a right-turn default | 37/40 against 21/40 successful Open rounds | Simulation |
| Keep the corner trigger on the front sonar | An early corner cue from raw side readings | 22/30 fell to 5/30 | Simulation |
| Turn total never reset, corner counted at 70 of 90 degrees | Lap closed at every 360 degrees, total zeroed after the exit | The 6 September build never parked; 360-degree closes sometimes closed lap 3 at corner 13 in simulation | Mat and simulation |
| PWM 30, with 38 only on calm straights | PWM 37 for the whole Open round | About 20 s but 28/40, against 34/40 at PWM 30 | Simulation |
| Avoid at PWM 25 plus a 70 ms kick at PWM 55 | Avoid at PWM 15 | PWM 15 does not start the car from rest, 18 creeps, 25 always moves it | Mat |
| Pillar steering scale with a 0.35 floor (fix 15) | No floor (fix 25) | With no floor the car stopped avoiding pillars | Mat |
| Each measurement in its own variable | Limiter distance written into `frontDist` (fix 13) | A limiter 20 cm away with a green pillar in view steered the car to full lock into the wall | Mat |
| Range every block, keep the nearest; area as `long` | `blocks[0]` and a 16-bit area | `blocks[0]` is the largest blob; a serial log printed `area=-9646` for a pillar at about 30 cm | Mat serial log |
| Park legs closed on the IMU, straight moves in measured steps | Legs that end on a sensor reading (versions 14-16) | Those parks stalled or drove into the limiter | Mat |
| Park stop mark from the front-wall range | Side sonar or camera inside the bay | HC-SR04 floor about 34 mm; our measurements of 1.0 m and 1.7 m, matching the rulebook positions | Team measurement, datasheet, rule |
| Solve the entry angle from the real stop pose | Fixed timed legs from a nominal pose | Earlier model: 37 mm success window against 24 mm positioning scatter | Simulation |
| Place the car about 40 mm from the outer wall in the lot | Flush with the open edge of the lot (75 mm gap) | Exit 30/30 at 40 mm, 3/30 at 75 mm, both with an unmeasured reverse-speed factor | Simulation |
| Start on any A2 change held 30 ms | Wait for press then release | A toggle switch never started the car on 14 Sep 2026 | Mat |
| Servo wiggle when the BNO055 fails | Silent `while(1)` | The silent hang was one of the four causes we found when the car did not move on 14 Sep 2026 | Mat |
| One `.ino` per challenge, sized with the IDE compiler | PlatformIO build with extra flags; `.ino` stub plus `.cpp` tabs | A park build fitted only with `-mcall-prologues -mrelax` (31,724 of 32,256 B); the tab split failed in the IDE. The v20 Obstacle sketch fits at 29,604 B with stock flags | Build |
| Simulator speed anchored to a real run | 520 mm/s at PWM 30 (a guess) | The guess predicted 40-48 s for three laps; the car did it in about 23 s, so we refitted to 998 mm/s | Mat and simulation |

We have not yet written down why we chose this chassis, an Arduino-only design, HC-SR04 sonars and a Pixy2.

## Version history

We did not commit this work to git while we did it ([versioning](05-build-test-reproduce.md#versioning-and-releases)). This table was written in September 2026 from our firmware folders and dated notes, and it gives dates only where a note records one.

| When | Version | Problem found | Change | Result |
|---|---|---|---|---|
| June 2026 | National-round code | - | - | 1st place, Kuwait national round |
| Before Sep 2026 | Fixes in `obstacle_kuwait` | Pillar area overflowed a 16-bit `int` (`area=-9646`) | Area as `long` | Kept in v20 |
| Before Sep 2026 | Same | Wrong-side passes; our fix notes give the likely cause, signature 1 trained on red while the code treated 1 as green | Code reads red as 1, green as 2 | Kept in v20 |
| Before Sep 2026 | Same | `blocks[0]` is the largest blob, not the nearest pillar | Every block ranged; nearest within 900 mm steers | Kept in v20 |
| Before Sep 2026 | Same | PWM 15 could not restart a stopped car | Avoid PWM 25 plus the PWM 55 kick (fix 4) | PWM 25 always moves the car |
| Before Sep 2026 | Versions 8-17 | All ten were worse on the mat and were reverted | Rule: no change without a named mechanism | Applied to every later change |
| Before Sep 2026 | Parking versions 14-16 | Legs ending on sensor readings stalled or hit the limiter | Timed legs closed on the IMU heading | Became the park architecture |
| 2 Sep 2026 | Car on `obstacle_kuwait` (per our notes) | - | - | Three full Obstacle laps with one light touch on one pillar |
| 6 Sep 2026 | 6 September parking build | Exit worked, no park: lap total zeroed while the car was 50 degrees rotated | Fix 26: never-reset total, exit rotation seeded | Lap 3 can close; the park can arm |
| 11 Sep 2026 | Simulator | Model car 1.9 times too slow | Speed refitted to the real 23 s run | All later simulator results use the fitted speed |
| 14 Sep 2026 | Team drawing of the nose | Laws tuned for 90-degree side sonars hit the walls | Side units modelled at 39 and 41 degrees; only the left-minus-right law kept | Simulator uses the drawn geometry |
| 14 Sep 2026 | Finals build of that day | Car did not move: start waited for press then release, exit PWM below 25, a park that fitted only with PlatformIO flags, silent `while(1)` on IMU failure | Start on any A2 change; IDE build; servo wiggle fault code | v20 keeps the start logic and the fault code and fits the IDE build; its exit keeps PWM 18 from the 6 September build |
| 15 Sep 2026 | `open_v20` | Corner direction a coin toss; lap 3 sometimes closed at corner 13 (simulation) | Corner vote; 12 counted corners, then a front-range stop | 37/40 against 21/40 in simulation |
| 15 Sep 2026 | `obstacle_v20` | 6 September build never reached its park, had no start input, turned right on every tie, ended its round on a nose-on contact | Front-wall park with a solved entry angle, lap-1 map, corner vote, stuck recovery, A2 start | Exit 119/120, three laps 12/120 in simulation; not yet on the mat |
| 15 Sep 2026 | `src/` | Rule 9.11 | `open_v20` and `obstacle_v20` committed as `Open_Challenge.ino` and `Obstacle_Challenge.ino` with `PRACTICE 0`, nothing else changed | Compiles with the Arduino IDE compiler |

## What failed

### Changes that did not work

- Early corner cue (Open, simulation). Starting the turn from raw side readings as the inner side opened took one batch from 22 to 5 successful rounds out of 30. We removed it.
- Steering scale with no floor (fix 25, mat). A pillar clipped by the frame edge reads far, so the turn scaled to nothing and the car stopped avoiding pillars. We went back to fix 15 and its 0.35 floor.
- One variable for two subsystems (fix 13, mat). The limiter distance written into `frontDist` fed the 25 cm full-lock rule and steered the car into the wall.
- Unreachable code (version 15). A start-in-the-lot mode printed on the serial monitor, but its arming line was never written. We now search the file for the call site after every edit.
- The finals build of 14 September (mat). The car did not move; the four causes are in the [version history](#version-history).

### Obstacle inner-pillar failure (simulation, not solved)

In 23 of 24 wrong-side passes we traced, the pillar needed an inner pass: green when driving counter-clockwise, red when clockwise. Most were in the inner row, first in the straight. The mechanism is the outer-wall drift from the 40-degree sonars ([placement](02-power-and-sensors.md#placement-and-the-40-degree-side-sonars)). None of these moved the success rate, 60 seeds each:

- an inward sweep after corners;
- earlier corner triggers at 60, 70 and 80 cm;
- engaging pillars from 1200 mm;
- faster pillar-mode entry;
- a 0.7 floor instead of 0.35;
- a stronger steering ladder;
- a bearing-offset tracking law;
- treating a silent side as long;
- disabling the fix-16 wall clamp.

Our next attempt plans the lane after each corner instead of changing another constant.

### The park (simulation)

Among 12 three-lap runs out of 120 there were 3 partial parks and 0 full parks. From the lap-3 handover pose alone, 4 of 16 parks were full, and the limiter was touched in about half.

### The simulator

It fails two of its three fidelity gates: `open_kuwait` completes 55 % of simulated rounds where the real car was reliable, and `obstacle_kuwait`, listed in our notes as the national-round code, completes 2 % ([fidelity gates](05-build-test-reproduce.md#fidelity-gates)).

## Risks and mitigations

| Risk | What we would see | Mitigation | Status |
|---|---|---|---|
| `PRACTICE 1` in an official round | Car moves 3 s after power-up, breaking rule 9.11 | `src/` has `PRACTICE 0`; bench check before every round | Done in `src/` |
| Start switch not wired to A2 | Car never starts with `PRACTICE 0` | Test in practice time | Wiring on the car not confirmed |
| BNO055 not answering | Servo wiggles twice, repeating | Visible fault code instead of a silent hang | In v20 |
| IMU mounted the other way up | Corners counted the wrong way | Clockwise hand-turn check; Obstacle detects the sign during the exit | In v20 |
| NDOF magnetometer moves the heading | Phantom or missed corner | 20 degrees of margin in the corner count; IMUPLUS mode to test | Not measured on the car |
| Exit creeps at PWM 18, below the 25 break-away value | Car stalls in the lot | Same values as the 6 September exit that worked; watch the first practice exits | Exit worked on 6 Sep 2026; no attempt count |
| Wrong outer-wall pick | Car exits the wrong way | Place the car about 40 mm from the outer wall; check the printout | Procedure |
| Inner sonar silent in a 1000 mm corridor | Car rides off the outer wall and misses the next pillar | None yet | Open problem |
| Park constants tuned only in simulation | Park too shallow, too deep or touching a limiter | Measure the park constants on the mat ([procedures](02-power-and-sensors.md#calibration-procedures)) | Not measured |
| Park window missed | Car would start lap 4 | Stops at the next corner in the start section | In v20 |
| Obstacle v20 worse than the backup on the mat | Wrong direction or pillar hits | Test-day rule: if it drives the wrong way or hits pillars twice in practice, switch to the backup sketch | Backup not yet chosen or in this repository |
| Pixy2 2.1 lens with 2.0 range constants | Pillars engage at the wrong distance | One-minute width check in PixyMon | Version not confirmed |
| Power path and servo rail unconfirmed | Brown-out at servo lock | Measure the 5 V rail and the servo current | Not measured |
| Battery charge changes speed | Timed moves behave differently on a flat battery | Arcs closed on the IMU, steps measured before use | Speed effect not measured |
| Flash near the limit | A new feature does not fit | Size checked with the IDE compiler before upload | 91 % used |
| Battery negative through the Uno header | Heat, burnt parts | Motor current kept off the Uno; 5 V to GND checked with power off after a fault | Wiring rule |

## v20 status

| Part | State on 15 Sep 2026 | Evidence |
|---|---|---|
| `src/Open_Challenge/Open_Challenge.ino` (`open_v20`) | Frozen, `PRACTICE 0`, compiles at 15,136 B. Not yet run on the mat | Simulation: 36/40 on fresh seeds, 287/320 across all draw cells |
| `src/Obstacle_Challenge/Obstacle_Challenge.ino` (`obstacle_v20`) | Frozen, `PRACTICE 0`, compiles at 29,604 B. Not yet run on the mat | Simulation: exit 119/120, three laps 12/120 |
| Front-wall park | Simulation only; four inputs marked MEASURE in the source | 4 full parks in 16 park-only runs |
| Backups: `open_kuwait`, `obstacle_kuwait`, the 6 September build | In our workspace, not in this repository | Rule 7 requires every sketch that may run at the event |
| `open_v21` | In development: fixed path at each corner, stronger direction choice, mid-straight stop on the front sensor | Not finished; nothing on these pages depends on it |

## Next steps

### On the mat, before we trust the park

| Step | Why | How |
|---|---|---|
| Measure `LOT_RIGHT_MM`, `R_PARK_MM` at both locks, the four side-sonar fit values, `PARK_LANE_MM`, `CAR_NOSE_MM` | Every park move scales with them | [Calibration procedures](02-power-and-sensors.md#calibration-procedures) |
| 20 Obstacle runs from legal random starts, recording exit, laps and park result | Turns simulation rates into mat rates, reported as a rate with failure modes | [How we test](05-build-test-reproduce.md#how-we-test) |
| Serial log of one counter-clockwise Open round, and single-sonar wall readings at 150-800 mm | The two checks that would show whether the simulator's sonar model can be trusted | [How to reproduce every number](05-build-test-reproduce.md#how-to-reproduce-every-number) |

### Measurements for these pages

| Step | Why |
|---|---|
| Height, mass, wheelbase, track, wheel diameter, centre of mass | Rule compliance, torque reasoning, R = L / tan δ |
| Battery label and current in four states: standing, PWM 30, break-away, servo at lock | Power budget and run time |
| Time 1 m at PWM 18, 25, 30 and 55, both directions, full and flat battery | A speed curve instead of one anchor, and the battery factor |
| Loop period from `millis()` on the serial monitor | Servo slew per second and control timing |
| Pixy2 version (27 px or 19 px at 500 mm) | Pillar range constants |
| Mounting heights of the sonars and the Pixy2 | Sonars must sit below the 100 mm wall top |

### Facts to confirm

- Chassis model number (284010 in our notes, 284131 on the plate), driven axles, motor part number and gear ratio, servo model.
- Power path, main switch type, start switch on A2, supply rail of the servo, sonars and BNO055.
- Which sketch ran at the June national round, and the mechanical changes made since June with their dates.
- Why we chose this chassis, an Arduino-only design, HC-SR04 sonars and a Pixy2.

### Changes to test, each with its mechanism named first

| Candidate | Mechanism it addresses | Test before it ships |
|---|---|---|
| BNO055 in IMUPLUS mode, `bno.begin(OPERATION_MODE_IMUPLUS)` | A magnetic field moving the NDOF heading and counting a phantom corner | Heading drift over 3 minutes standing, and 12-corner counts, in both modes |
| Corner-exit lane plan for Obstacle, from the lap-1 map or the camera during the turn | Outer-wall drift puts the first pillar outside the camera view | 60 lot-start seeds against v20, then practice runs |
| Rear-facing HC-SR04 on two free pins (TRIG and ECHO from A0, A1, A3) | The park's reverse leg is dead-reckoned into a 37 mm window | Park-only runs in simulation, then on the mat |

### Repository

- Pick the backup sketch for India, give it the A2 start input, and commit it (rule 7).
- Publish the simulator and the Webots replay files in a separate folder, or say that they stay private.
- STEP or STL exports of the body parts and a dimensioned top and side drawing from measured values.
- Prices and suppliers for the bill of materials.
- Tags and release notes for the June 2026 and Asia-final versions ([versioning](05-build-test-reproduce.md#versioning-and-releases)).

<sub>[Back to the README](../README.md) · Previous: [Software and strategy](03-software-and-strategy.md) · Next: [Build, test and reproduce](05-build-test-reproduce.md)</sub>
