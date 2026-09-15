# Wiring

The wiring reference for the competition car is in the documentation:

- signal pins: [`docs/diagrams/wiring_pinmap.png`](../docs/diagrams/wiring_pinmap.png) and the pin table in [Build, test and reproduce](../docs/05-build-test-reproduce.md#wiring);
- supply: [`docs/diagrams/power_tree.svg`](../docs/diagrams/power_tree.svg) and [Power](../docs/02-power-and-sensors.md#power).

## The June 2026 drawing

[`Schematic_Wiring_Diagram.pdf`](Schematic_Wiring_Diagram.pdf) is a one-page Fritzing drawing from June 2026. We keep it as a record from that time. It draws fewer parts than the finals sketches use:

| In the PDF | In the finals sketches |
|---|---|
| Arduino Uno, Cytron MD13S, DC motor, servo, breadboard | Same parts, plus the ones below |
| 2 HC-SR04 | 3 HC-SR04, including the front unit on D6/D7 |
| No Pixy2 | Pixy2 on the ICSP header (SPI) |
| No BNO055 | BNO055 on A4 (SDA) and A5 (SCL) |
| No start switch | Start switch from A2 to GND |
| Servo signal drawn to the analog header (A0) | Servo signal on D10 |
| Two 3.7 V 110 mAh cells | 2S LiPo, 7.4 V nominal |

## Pin map

The pins are identical in `Open_Challenge.ino` and `Obstacle_Challenge.ino`.

| Component | Pins | Notes |
|---|---|---|
| HC-SR04 left | TRIG D4, ECHO D5 | Slanted front-left corner, about 40 degrees from the nose axis |
| HC-SR04 right | TRIG D2, ECHO D9 | Slanted front-right corner, about 40 degrees from the nose axis |
| HC-SR04 front | TRIG D6, ECHO D7 | Centred on the nose, straight ahead |
| Steering servo | Signal D10 | Servo library; 90 straight, above 90 steers left |
| Cytron MD13S | PWM D3, DIR D8 | DIR HIGH = forward |
| Start switch | A2 to GND | Internal pull-up; any change held 30 ms starts the round |
| BNO055 | SDA A4, SCL A5 | I2C with a 25 ms bus timeout |
| Pixy2 | ICSP header: MOSI D11, MISO D12, SCK D13 | SPI; powered from the Uno 5 V through the ICSP header |
| Serial | D0, D1 | 115200 baud, debug output |
| Free | A0, A1, A3 | - |

One wiring rule applies to every job: the battery negative must not return through the Arduino header. A black lead once connected near the Uno power header was followed by heat and a burnt component. Only signal ground goes to the Uno.
