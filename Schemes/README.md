# ⚡ Electrical Schematic — Blue Wave Team

📄 **[Schematic_Wiring_Diagram.pdf](Schematic_Wiring_Diagram.pdf)** — Full Fritzing wiring diagram


## Wiring Summary

| Component | Pin(s) | Notes |
|-----------|--------|-------|
| HC-SR04 Left | TRIG→D4, ECHO→D5 | 5V powered |
| HC-SR04 Right | TRIG→D2, ECHO→D9 | 5V powered |
| HC-SR04 Front | TRIG→D6, ECHO→D7 | 5V powered |
| Servo | SIG→D10 | 5V powered |
| Cytron MD13S | PWM→D3, DIR→D8 | Motor driver |
| Pixy2 | SPI via ICSP (MOSI/MISO/SCK) | 5V powered |
| BNO055 IMU | SDA→A4, SCL→A5 | I2C, 3.3V, yaw lap counting |

## Power Distribution

```
LiPo 7.4V ──→ Cytron MD13S  (motor power)
          └──→ 5V Buck Converter ──→ Arduino Vin
                                        └──→ Servo / 3× Ultrasonic / Pixy2 / BNO055
```
