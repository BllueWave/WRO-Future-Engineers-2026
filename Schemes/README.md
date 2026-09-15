# Wiring

<p align="center">
  <a href="wiring_schematic.png"><img src="wiring_schematic.png" width="100%" alt="Wiring schematic of the competition car: the Arduino Uno R3 with the front HC-SR04 on A0 and D7, the left one on D4 and D5, the right one on D2 and D9, the BNO055 on A4 and A5, the Pixy2 on the ICSP header, the steering servo on D10, the Cytron MD13S on D3 and D8 and the start switch on A2; the Uno 5 V rail feeds the sonars, the BNO055 and the servo; battery 2 reaches the Uno VIN through the main power switch; battery 1 feeds the MD13S power input, and the MD13S drives the motor"></a>
</p>

One sheet with every wire of the competition car: signal, supply and ground, between the Arduino Uno R3, the three HC-SR04 sonars, the BNO055, the Pixy2, the steering servo, the Cytron MD13S, the drive motor, the start switch, the main power switch and both batteries. The pins are the ones in `Open_Challenge.ino` and `Obstacle_Challenge.ino`, which are identical. A hop marks two wires that cross without connecting; a dot marks a junction.

| File | What it is |
|---|---|
| [`wiring_schematic.png`](wiring_schematic.png) | The schematic above, 3840 × 2214 px |
| [`wiring_schematic_source.svg`](wiring_schematic_source.svg) | Editable source of the same sheet: every wire, junction and crossing in plain SVG |
| [`archive_june_2026/`](archive_june_2026/) | The June 2026 Fritzing drawing, kept as a record (see below) |

Other views of the same wiring: [pin map](../docs/diagrams/wiring_pinmap.png), [power tree](../docs/diagrams/power_tree.png), [system overview](../docs/diagrams/system_overview.png).

## Pin map

| Component | Pins | Notes |
|---|---|---|
| HC-SR04 left | TRIG D4, ECHO D5 | Slanted front-left corner, about 40 degrees from the nose axis |
| HC-SR04 right | TRIG D2, ECHO D9 | Slanted front-right corner, about 40 degrees from the nose axis |
| HC-SR04 front | TRIG A0, ECHO D7 | Centred on the nose, straight ahead |
| Steering servo | Signal D10 | Servo library; 90 straight, above 90 steers left |
| Cytron MD13S | PWM D3, DIR D8 | DIR HIGH = forward |
| Start switch | A2 to GND | Internal pull-up; any change held 30 ms starts the round |
| BNO055 | SDA A4, SCL A5 | I2C with a 25 ms bus timeout |
| Pixy2 | ICSP header: MOSI D11, MISO D12, SCK D13 | SPI; powered from the Uno 5 V through the ICSP header |
| Serial | D0, D1 | 115200 baud, debug output |
| Free | D6, A1, A3 | - |

D11, D12 and D13 belong to the Pixy2 link. The front trigger moved from D6 to A0 on 15 September 2026, and it cannot go on D13: while SPI is on, the ATmega328P drives D13 as the SPI clock, so a trigger written there never reaches the sensor. Our development sketches refuse to compile with a sonar pin on D11-D13.

## Power

| Supply | Connection | Feeds |
|---|---|---|
| Battery 1, 2S LiPo, 7.4 V nominal | Straight to the Cytron MD13S power input | The drive motor only |
| Battery 2 | Main power switch (rule 9.10), then the Uno's VIN | The Uno; its on-board regulator makes 5 V |
| Uno 5 V | 5 V rail from the Uno; the Pixy2 through the ICSP header | The three HC-SR04, the BNO055, the steering servo and the Pixy2 |

## Grounds

| Ground | Connects to |
|---|---|
| Battery 2 negative | Uno GND |
| Sonars, BNO055, servo, start switch | Uno GND |
| MD13S logic GND | Uno GND, so PWM and DIR share a reference with the Uno |
| Battery 1 negative | MD13S power input only |

Motor current stays on battery 1 and never returns through the Arduino header. A black lead once connected near the Uno power header was followed by heat and a burnt component, so only the signal ground runs between the MD13S and the Uno.

## The June 2026 drawing

[`archive_june_2026/Schematic_Wiring_Diagram.pdf`](archive_june_2026/Schematic_Wiring_Diagram.pdf) is a one-page Fritzing drawing from June 2026. It draws fewer parts than the finals sketches use:

| In the PDF | In the finals sketches |
|---|---|
| Arduino Uno, Cytron MD13S, DC motor, servo, breadboard | Same parts, plus the ones below |
| 2 HC-SR04 | 3 HC-SR04, including the front unit on A0/D7 |
| No Pixy2 | Pixy2 on the ICSP header (SPI) |
| No BNO055 | BNO055 on A4 (SDA) and A5 (SCL) |
| No start switch | Start switch from A2 to GND |
| Servo signal drawn to the analog header (A0) | Servo signal on D10 |
| Two 3.7 V 110 mAh cells | Two batteries: battery 1 (2S LiPo, 7.4 V nominal) straight to the MD13S power input; battery 2 through the main power switch to the Uno's VIN |

<sub>[Back to the README](../README.md)</sub>
