# Arduino UNO Q (4GB) Geo-Aware YOLO Vision Rover

An autonomous vision-guided robotics system built for the **Arduino UNO Q (4GB)**. 

**Operates 100% standalone — NO Arduino App Lab or mobile app required.**
Connect to the rover from any browser (on your phone, tablet, or PC) over Wi-Fi, or let it run autonomously on boot.

---

## Key Features

* **No External App Needed:** The UNO Q Debian MPU runs an embedded responsive Web Dashboard on port `5000`.
* **Real-Time Edge AI:** Direct YOLOv8 Nano inference on the 4GB Qualcomm QRB2210 core.
* **Dual-Brain Architecture:** Qualcomm MPU handles high-level vision and web streaming; STM32 MCU handles low-latency GPS (NEO-6M), IMU (MPU-6050), and motor driving.
* **Live Web HUD (`http://unoq.local:5000`):**
  * Live camera stream with YOLO bounding boxes and confidence scores.
  * Real-time GPS Latitude, Longitude, and Speed gauges.
  * 6-DOF IMU Pitch and Roll indicators.
  * Switch between **Autonomous YOLO Navigation** and **Manual D-Pad Controls**.
  * One-click download for geo-tagged detection CSV logs.
* **Auto-Start on Boot:** Includes a `systemd` service so the rover runs immediately when powered by battery.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│              Qualcomm Dragonwing QRB2210 (MPU)                  │
│                   Debian 13 Linux (4GB RAM)                     │
│                                                                 │
│  - USB / CSI Camera Feed                                        │
│  - YOLOv8 Nano Edge AI Inference                                │
│  - Embedded Web Server (Flask + MJPEG Stream on Port 5000)      │
│  - Geo-referenced CSV Event Logging                             │
└────────────────────────────────┬────────────────────────────────┘
                                 │ Internal Bridge Serial / RPC
┌────────────────────────────────┴────────────────────────────────┐
│                   STM32U585 Microcontroller (MCU)               │
│                        Real-Time Control                        │
│                                                                 │
│  - NEO-6M / M8N GPS (Pins D4, D3 via SoftwareSerial)           │
│  - MPU-6050 6-DOF IMU (I2C / Qwiic Pins SDA, SCL)              │
│  - Dual DC Motor Driver (PWM Pins D5, D6, D7, D8, D9, D10)      │
└─────────────────────────────────────────────────────────────────┘
```

---

## Hardware Pinout & Wiring

### 1. GPS Module (NEO-6M / M8N) $\to$ Arduino UNO Q
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

## Quickstart (No App Required)

### 1. SSH into the UNO Q & Clone
```bash
ssh arduino@unoq.local
git clone https://github.com/devanshvpurohit/uno-q-geo-yolo-rover.git
cd uno-q-geo-yolo-rover
```

### 2. Run Setup
```bash
chmod +x setup.sh run.sh flash_mcu.sh
./setup.sh
```

*(Optional: Enable auto-start on boot)*
```bash
./setup.sh --enable-autostart
```

### 3. Flash the STM32 MCU (No IDE needed)
Flash the microcontroller core directly via CLI:
```bash
./flash_mcu.sh
```

### 4. Launch Rover
```bash
./run.sh
```

---

## Access the Web Dashboard

Open any web browser on your phone, tablet, or laptop connected to the same Wi-Fi:

```
http://unoq.local:5000
```
*(or `http://<UNO_Q_IP_ADDRESS>:5000`)*

From this page you can:
1. Watch the live camera feed with YOLO detection bounding boxes.
2. Monitor real-time GPS coordinates and IMU orientation.
3. Toggle between **Auto Mode** and **Manual D-Pad** driving.
4. Download the `geo_detections_log.csv` file.

---

## License
MIT License. Created by [Devansh Purohit](https://github.com/devanshvpurohit).
