# Wiring

## The June 2026 diagram

[`Schematic_Wiring_Diagram.pdf`](Schematic_Wiring_Diagram.pdf) is a one-page Fritzing drawing made in June 2026. **It is incomplete for the orange car** and should not be used to wire it. Compared with the pins in the finals sketches:

| In the PDF | On the orange car (finals sketches) |
|---|---|
| Arduino Uno, Cytron MD13S, DC motor, servo, breadboard | Same parts, plus the ones below |
| 2 HC-SR04 | 3 HC-SR04: the front unit on D6/D7 is missing from the PDF |
| No Pixy2 | Pixy2 on the ICSP header (SPI) |
| No BNO055 | BNO055 on A4 (SDA) and A5 (SCL) |
| No start switch | Start switch from A2 to GND |
| Servo signal drawn to the analog header (A0) | Servo signal on D10 |
| Two 3.7 V 110 mAh cells | 2S LiPo, 7.4 V nominal; capacity TODO |

TODO(team): redraw the diagram with every part above, the main power switch and the real power path, and export it as PDF and PNG.

## Pin map (from `open_v20` and `obstacle_v20`; identical in both)

| Component | Pins | Notes |
|---|---|---|
| HC-SR04 left | TRIG D4, ECHO D5 | Slanted front-left corner, about 40 degrees from the nose axis |
| HC-SR04 right | TRIG D2, ECHO D9 | Slanted front-right corner, about 40 degrees from the nose axis |
| HC-SR04 front | TRIG D6, ECHO D7 | Centred on the nose, straight ahead |
| Steering servo | Signal D10 | Servo library; 90 straight, above 90 steers left |
| Cytron MD13S | PWM D3, DIR D8 | DIR HIGH = forward |
| Start switch | A2 to GND | Internal pull-up; any change held 30 ms starts the round |
| BNO055 | SDA A4, SCL A5 | I2C with a 25 ms bus timeout; supply pin not recorded (TODO) |
| Pixy2 | ICSP header: MOSI D11, MISO D12, SCK D13 | SPI; powered from the Uno 5 V through the ICSP header |
| Serial | D0, D1 | 115200 baud, debug output |
| Free | A0, A1, A3 | - |

## Power distribution

**TODO(team): measure and confirm.** Our June documents give two different paths:

```text
Version in the June README          Version in the June Schemes page
LiPo 7.4 V ─┬─> Cytron MD13S        LiPo 7.4 V ─┬─> Cytron MD13S
            └─> Uno Vin                         └─> 5 V buck converter ─> Uno Vin
```

Feeding 5 V into Vin would go through the Uno's own 5 V regulator, so the rail voltage on the car decides which drawing is right. Record the part number and rating of the converter if one is fitted.

One wiring rule is not in doubt: the battery negative must not return through the Arduino header. A black lead once connected near the Uno power header was followed by heat and a burnt component. Only signal ground goes to the Uno.
