# Arduino UNO Q (4GB) Geo-Aware YOLO Vision Rover

An autonomous vision-guided robotics system built specifically for the **Arduino UNO Q (4GB)** dual-core platform. It combines real-time edge AI object detection with GPS tracking, 6-DOF IMU orientation sensing, and reactive motor control.

---

## Architecture

The **Arduino UNO Q** features a hybrid dual-brain architecture:

```
┌─────────────────────────────────────────────────────────────────┐
│              Qualcomm Dragonwing QRB2210 (MPU)                  │
│                   Debian 13 Linux (4GB RAM)                     │
│                                                                 │
│  - USB / CSI Camera Feed                                        │
│  - YOLOv8 Nano Edge AI Inference                                │
│  - Vision-based Steering & Obstacle Avoidance Decisions         │
│  - Geo-referenced CSV Event Logging                             │
└────────────────────────────────┬────────────────────────────────┘
                                 │ Internal RPC / Bridge Serial
┌────────────────────────────────┴────────────────────────────────┐
│                   STM32U585 Microcontroller (MCU)               │
│                        Real-Time Control                        │
│                                                                 │
│  - NEO-6M / M8N GPS Ingestion (UART - Pins D4, D3)             │
│  - MPU-6050 6-DOF IMU Telemetry (I2C / Qwiic - SDA/SCL)        │
│  - Dual DC Motor Drive Actuation (PWM Pins D5, D6, D7, D8, D9, D10)│
└─────────────────────────────────────────────────────────────────┘
```

---

## Hardware Pinout & Wiring

### 1. GPS Module (NEO-6M / NEO-M8N) $\to$ Arduino UNO Q
| GPS Pin | Arduino UNO Q Pin | Description |
|---|---|---|
| `VCC` | `5V` | Power supply |
| `GND` | `GND` | Common Ground |
| `TX` | `Pin D4` | GPS serial transmit $\to$ MCU software RX |
| `RX` | `Pin D3` | GPS serial receive $\to$ MCU software TX |

### 2. IMU Module (MPU-6050) $\to$ Arduino UNO Q
| MPU-6050 Pin | Arduino UNO Q Pin | Description |
|---|---|---|
| `VCC` | `5V` (or `3.3V`) | Power supply |
| `GND` | `GND` | Common Ground |
| `SDA` | `SDA` (or `A4` / Qwiic) | I2C Data |
| `SCL` | `SCL` (or `A5` / Qwiic) | I2C Clock |

### 3. Motor Driver (L298N / TB6612FNG) $\to$ Arduino UNO Q
| Motor Driver Pin | Arduino UNO Q Pin | Function |
|---|---|---|
| `ENA` | `Pin D5` | Left Motor Speed (PWM) |
| `IN1` | `Pin D6` | Left Motor Direction |
| `IN2` | `Pin D7` | Left Motor Direction |
| `IN3` | `Pin D8` | Right Motor Direction |
| `IN4` | `Pin D9` | Right Motor Direction |
| `ENB` | `Pin D10` | Right Motor Speed (PWM) |

---

## Quickstart & Deployment

### 1. Clone onto the Arduino UNO Q
SSH into your Arduino UNO Q over Wi-Fi or USB-C:

```bash
ssh arduino@unoq.local
```

Clone this repository:

```bash
git clone https://github.com/devanshvpurohit/uno-q-geo-yolo-rover.git
cd uno-q-geo-yolo-rover
```

### 2. Run the Environment Setup
Run the automated Debian setup script to configure Python, OpenCV, and pre-cache YOLOv8n weights:

```bash
chmod +x setup.sh
./setup.sh
```

### 3. Flash the Real-Time MCU Sketch
You can upload `sketch/sketch.ino` using **Arduino App Lab** or via the CLI:

```bash
arduino-app-cli app start /home/arduino/uno-q-geo-yolo-rover
```

*Or via Arduino CLI directly:*
```bash
arduino-cli compile --fqbn arduino:zephyr:unoq_mcu sketch
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:zephyr:unoq_mcu sketch
```

### 4. Launch the Vision Core
Start the autonomous mission loop on Debian Linux:

```bash
python3 python/main.py
```

---

## Telemetry & Logging

When an object is detected in the rover's trajectory, it calculates reactive steering vectors and records a geo-tagged detection to `geo_detections_log.csv`:

```csv
Timestamp,Class,Confidence,Lat,Lng,Speed_kmh,Pitch_Ax,Roll_Ay,Action
2026-11-17T20:30:15.123456,person,0.88,28.613939,77.209021,4.2,0.12,-0.04,R
```

---

## License
MIT License. Created by [Devansh Purohit](https://github.com/devanshvpurohit).
