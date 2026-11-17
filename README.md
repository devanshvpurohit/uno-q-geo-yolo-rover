# SURDAS: AI-Powered Assistive Vision & Navigation Platform for the Blind

An intelligent assistive wearable system designed for visually impaired individuals, powered by the **Arduino UNO Q (4GB)** dual-core architecture (Qualcomm QRB2210 Linux MPU + STM32U585 Real-Time MCU).

---

## Key Assistive Features

1. **Ultra-Fast Edge Vision (YOLOv8):**
   * Optimized 320x320 inference with frame-skipping running locally on the 4GB Qualcomm MPU.
   * Recognizes people, obstacles, doors, stairs, chairs, and vehicles with high accuracy.
2. **Directional Haptic Feedback (STM32 MCU):**
   * Three haptic vibration channels alert the user tactically without relying solely on hearing:
     * **Left Motor (Pin D5):** Obstacle detected on the left.
     * **Center Motor / Buzzer (Pin D6):** Hazard directly ahead (Stop command).
     * **Right Motor (Pin D7):** Obstacle detected on the right.
3. **Real-Time Voice Assistant (Text-to-Speech):**
   * The web interface runs speech synthesis directly through your phone/earphones (e.g., *"Caution: Person on your left"*, *"Stop! Obstacle directly in front of you"*).
4. **IMU Fall Detection & Orientation (MPU-6050):**
   * Continuously monitors G-force dynamics ($>2.8g$ impact or free-fall); triggers an emergency Fall Alert with GPS location if a fall occurs.
5. **Geo-Location & SOS Emergency (NEO-6M GPS):**
   * Satellite tracking with live coordinates and satellite count diagnostics.
   * Emergency pushbutton on Pin D2 sends instant SOS coordinates.

---

## Hardware Pinout & Wiring

### 1. GPS Module (NEO-6M / M8N) $\to$ Arduino UNO Q
| GPS Pin | Arduino UNO Q Pin | Description |
|---|---|---|
| `VCC` | `5V` (or `3.3V`) | Power |
| `GND` | `GND` | Ground |
| `TX` | `Pin 0 (RX)` | GPS Serial Transmit $\to$ MCU Hardware `Serial1` RX |
| `RX` | `Pin 1 (TX)` | GPS Serial Receive $\to$ MCU Hardware `Serial1` TX |

### 2. IMU Module (MPU-6050) $\to$ Arduino UNO Q
| MPU-6050 Pin | Arduino UNO Q Pin | Description |
|---|---|---|
| `VCC` | `5V` (or `3.3V`) | Power |
| `GND` | `GND` | Ground |
| `SDA` | `SDA` (or `A4` / Qwiic) | I2C Data |
| `SCL` | `SCL` (or `A5` / Qwiic) | I2C Clock |

### 3. Haptics, Buzzer & SOS $\to$ Arduino UNO Q
| Peripheral | Arduino UNO Q Pin | Description |
|---|---|---|
| **Left Haptic Motor** | `Pin D5` | Vibrates on left hazard |
| **Center Haptic / Buzzer** | `Pin D6` | Vibrates on frontal hazard |
| **Right Haptic Motor** | `Pin D7` | Vibrates on right hazard |
| **SOS Pushbutton** | `Pin D2` | Connected to GND (Internal PULLUP) |

---

## Quickstart & Deployment

### 1. Pull the Updates on the UNO Q
```bash
cd ~/uno-q-geo-yolo-rover
git fetch origin main
git reset --hard origin/main
```

### 2. Flash the STM32 Assistive Firmware
```bash
./flash_mcu.sh
```

### 3. Launch SURDAS Vision
```bash
./run.sh
```

---

## Using the Assistive Dashboard

Open any web browser on your phone connected to the same Wi-Fi:

```
http://unoq.local:5000
```
*(or `http://<UNO_Q_IP_ADDRESS>:5000`)*

* Tap **"ENABLE VOICE ASSISTANT"**: The phone will start announcing detected obstacles and navigation cues through your headphones or speaker.
* Monitor live **Fall Detection**, **GPS coordinates**, and **Satellite count**.
* Haptic feedback automatically drives vibration motors on pins D5, D6, and D7.

---

## License
MIT License. Created by [Devansh Purohit](https://github.com/devanshvpurohit).
