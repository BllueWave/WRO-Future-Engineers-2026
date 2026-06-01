<div align="center">

<img src="https://raw.githubusercontent.com/BllueWave/WRO-Future-Engineers-2026/main/docs/logo.png" width="220" alt="Blue Wave Team Logo"/>

# 🌊 Blue Wave Team — WRO Future Engineers 2026

**Kuwait National Qualifier · May 31, 2026**

![Arduino](https://img.shields.io/badge/Arduino-Uno-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-C%2FC%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![Status](https://img.shields.io/badge/Status-Competition%20Ready-brightgreen?style=for-the-badge)
![WRO](https://img.shields.io/badge/WRO-Future%20Engineers-0057A8?style=for-the-badge)

</div>

---

## 📋 Table of Contents

- [🧩 Project Overview](#-project-overview)
- [📁 Repository Structure](#-repository-structure)
- [🔩 Hardware & Components](#-hardware--components)
- [⚡ Wiring & Electrical](#-wiring--electrical)
- [💻 Software Architecture](#-software-architecture)
- [🚗 Open Challenge](#-open-challenge)
- [🚧 Obstacle Challenge](#-obstacle-challenge)
- [📸 Robot Photos](#-robot-photos)
- [🎬 Video Demonstrations](#-video-demonstrations)
- [🧪 Testing & Calibration](#-testing--calibration)
- [👨‍💻 Team Members](#-team-members)

---

## 🧩 Project Overview

Blue Wave Team presents an autonomous self-driving vehicle engineered for the **WRO Future Engineers 2026** competition. Our robot navigates a closed-loop track, avoids colored obstacles, and completes three full laps without human intervention.

The system is built around three core pillars:

| Pillar | Description |
|--------|-------------|
| 🏎️ **Mobility** | Rear-wheel drive via DC motor + Ackermann front steering servo |
| 👁️ **Perception** | Dual HC-SR04 ultrasonics for wall-following + Pixy2 camera for pillar detection |
| 🧠 **Control** | Tuned PD controller with anti-zigzag smoothing + vision-based mode switching |

---

## 📁 Repository Structure

```
WRO-Future-Engineers-2026/
├── README.md
├── src/
│   ├── Open_Challenge/
│   │   └── Open_Challenge.ino       ← PD wall-following controller
│   └── Obstacle_Challenge/
│       └── Obstacle_Challenge.ino   ← PD + Pixy2 obstacle avoidance
├── Vehicle_Photos/                   ← 6-angle robot photography
├── Models/                           ← 3D design files
├── Schemes/                          ← Electrical schematic
├── videos/                           ← Demo video links
└── docs/                             ← Team logo & assets
```

---

## 🔩 Hardware & Components

| Component | Model | Purpose |
|-----------|-------|---------|
| Microcontroller | Arduino Uno (ATmega328P) | Main logic & control |
| Motor Driver | Cytron MD13S | DC motor speed & direction |
| Drive Motor | Brushed DC (7.4V) | Rear-wheel propulsion |
| Steering | Servo Motor | Front Ackermann steering |
| Distance Sensors | 2× HC-SR04 | Left/right wall distance |
| Vision Sensor | Pixy2 Camera (SPI) | Red/green pillar detection |
| Battery | 7.4V LiPo 2S | Main power supply |
| Chassis | WLtoys 284010 (1:28 RC) | Compact, robust platform |

**Dimensions:** ~17 × 9 × 7 cm — within WRO 30×20×30 cm limit ✅  
**Mass:** ~0.5 kg — within 1.5 kg limit ✅

---

## ⚡ Wiring & Electrical

```
HC-SR04 LEFT    TRIG → D4    ECHO → D5    VCC → 5V   GND
HC-SR04 RIGHT   TRIG → D2    ECHO → D9    VCC → 5V   GND
Servo           SIG  → A0                 VCC → 5V   GND
Cytron MD13S    PWM  → D3    DIR  → D8
Pixy2 (SPI)     CS   → D10   MOSI → D11   MISO → D12   SCK → D13
```

> **Power:** LiPo 7.4V → Cytron (motor power) + 5V buck converter → Arduino & sensors

---

## 💻 Software Architecture

### Control Flow

```
[HC-SR04 L/R] ──→ [LPF + Clamp] ──→ ┐
                                      ├──→ [Mode Manager] ──→ [PD / Pixy] ──→ [Servo]
[Pixy2 Camera] ──→ [Blob Filter] ──→ ┘                                          │
                                                                                  ↓
                                                                         [Cytron MD13S]
                                                                                  │
                                                                             [DC Motor]
```

### Key Tuned Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `KP` | 0.10 | Proportional gain |
| `KD` | 0.09 | Derivative gain |
| `MOTOR_SPEED` | 55 | Normal cruising speed (0–255) |
| `TURN_SPEED` | 45 | Reduced speed on sharp turns |
| `CENTER_ANGLE` | 90° | Servo straight-ahead position |
| `DERIV_ALPHA` | 0.45 | Derivative low-pass smoother |
| `SERVO_ALPHA` | 0.55 | Servo output smoother |
| `ERROR_DEADBAND` | 0.8 cm | Ignore sensor noise below threshold |

---

## 🚗 Open Challenge

**File:** [`src/Open_Challenge/Open_Challenge.ino`](src/Open_Challenge/Open_Challenge.ino)

Pure wall-following using a PD controller on dual ultrasonic sensor error:

```
error  = leftDist − rightDist
output = KP × error + KD × smoothed_derivative
servo  = CENTER_ANGLE + output
```

**Anti-zigzag techniques:**
- ±0.8 cm error deadband — ignores sensor noise
- Derivative low-pass filter — prevents jitter from spike readings
- Servo output smoothing — eliminates mechanical oscillation
- Corner spike clamping (max 75 cm) — ignores false corner readings
- Adaptive speed — slows on sharp turns (>28° from center)

---

## 🚧 Obstacle Challenge

**File:** [`src/Obstacle_Challenge/Obstacle_Challenge.ino`](src/Obstacle_Challenge/Obstacle_Challenge.ino)

Extends wall-following with Pixy2 color detection and hysteresis-based mode switching:

| Pillar Color | Signature | Robot Action |
|-------------|-----------|-------------|
| 🟢 Green | Sig 1 | Steer **LEFT** — pass pillar on the right |
| 🔴 Red | Sig 2 | Steer **RIGHT** — pass pillar on the left |

**Mode switching logic:**
- Enters `PIXY MODE` after **2 consecutive** valid detections
- Returns to `PID MODE` after **3 consecutive** misses
- Minimum hold time **280 ms** prevents flickering
- Slew-rate limiting **6°/step** for smooth servo transitions

---

## 📸 Robot Photos

<div align="center">
<table>
  <tr>
    <td align="center"><img width="220" src="https://github.com/user-attachments/assets/bcfedd58-0cac-417d-aeb3-2eb8b212f7ef"/><br/><sub><b>Front</b></sub></td>
    <td align="center"><img width="220" src="https://github.com/user-attachments/assets/a8079dca-615e-4b55-9f72-6bb9f630937b"/><br/><sub><b>Back</b></sub></td>
    <td align="center"><img width="220" src="https://github.com/user-attachments/assets/e3ce21d4-4cd9-439d-9274-82af3605b75f"/><br/><sub><b>Left</b></sub></td>
  </tr>
  <tr>
    <td align="center"><img width="220" src="https://github.com/user-attachments/assets/581d9ba8-7fb7-40f2-ab8f-64d2de9f3165"/><br/><sub><b>Right</b></sub></td>
    <td align="center"><img width="220" src="https://github.com/user-attachments/assets/2a9d8bb3-ee46-4803-8cc3-46ba6ddcda77"/><br/><sub><b>Top</b></sub></td>
    <td align="center"><img width="220" src="https://github.com/user-attachments/assets/afcc0fe8-2d0e-4aef-aea4-6d611f3f30ef"/><br/><sub><b>Bottom</b></sub></td>
  </tr>
</table>
</div>

---

## 🎬 Video Demonstrations

| Challenge | Description |
|-----------|-------------|
| Open Challenge | 3-lap autonomous wall-following run |
| Obstacle Challenge | Full obstacle avoidance with pillar detection |

> Video files in [`videos/`](videos/)

---

## 🧪 Testing & Calibration

| Test | Goal | Status |
|------|------|--------|
| Ultrasonic calibration | Verify left/right readings accuracy | ✅ |
| Servo center calibration | Find true CENTER_ANGLE | ✅ |
| Motor dead zone test | Find minimum effective PWM | ✅ |
| PD on-track tuning | KP/KD field calibration | ✅ |
| Pixy2 color training | Red & green under arena lighting | ✅ |
| Full 3-lap run | Open Challenge completion | ✅ |
| Obstacle avoidance | All pillar configurations | ✅ |

---

## 👨‍💻 Team Members

<div align="center">

| Name | Role |
|------|------|
| **Fawaz Al-Assousi** — فواز العسعوسي | Hardware design · PD control · System integration |
| **Bassam** — بسام | Software development · Pixy2 vision · Testing & calibration |

</div>

---

<div align="center">

**🌊 Blue Wave Team — Kuwait 2026**

*Built with precision. Driven by code.*

</div>
