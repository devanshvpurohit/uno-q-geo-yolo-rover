# SURDAS: Assistive GPS & IMU Platform for Arduino UNO Q (4GB)

A lightweight, real-time telemetry and safety system built for the **Arduino UNO Q (4GB)** dual-core platform (Qualcomm QRB2210 Debian Linux + STM32U585 Zephyr MCU).

Dedicated exclusively to **GPS location tracking**, **6-DOF IMU orientation & fall detection**, and **audio status readout** for visually impaired users. Zero heavy neural network dependencies — starts instantly and runs with < 1% CPU.

---

## Hardware Pinout & Wiring

### 1. GPS Module (NEO-6M / M8N) $\to$ Arduino UNO Q
| GPS Pin | Arduino UNO Q Pin | Description |
|---|---|---|
| `VCC` | `5V` (or `3.3V`) | Power supply |
| `GND` | `GND` | Common Ground |
| `TX` | `Pin 0 (RX)` | GPS serial transmit $\to$ Hardware `Serial1` RX |
| `RX` | `Pin 1 (TX)` | GPS serial receive $\to$ Hardware `Serial1` TX |

### 2. IMU Module (MPU-6050) $\to$ Arduino UNO Q
| MPU-6050 Pin | Arduino UNO Q Pin | Description |
|---|---|---|
| `VCC` | `5V` (or `3.3V`) | Power supply |
| `GND` | `GND` | Common Ground |
| `SDA` | `SDA` (or `A4` / Qwiic) | I2C Data |
| `SCL` | `SCL` (or `A5` / Qwiic) | I2C Clock |

---

## Quickstart (Deploy & Run)

Run these 3 commands on your Arduino UNO Q:

```bash
cd ~/uno-q-geo-yolo-rover

# 1. Pull the streamlined GPS + IMU code
git fetch origin main
git reset --hard origin/main

# 2. Flash the STM32 MCU
./flash_mcu.sh

# 3. Launch the platform
./run.sh
```

---

## Live Dashboard

Open any web browser on your phone or PC connected to the same Wi-Fi:

```
http://unoq.local:5000
```
*(or `http://<UNO_Q_IP_ADDRESS>:5000`)*

* **GPS Coordinates & Google Maps:** Live Latitude, Longitude, Altitude, Ground Speed, and Satellite count with a direct Google Maps pin link.
* **IMU Diagnostics:** Real-time Pitch & Roll angles, 3-axis acceleration ($a_x, a_y, a_z$), and Fall Detection indicator.
* **Audio Voice Output:** Tap *"READ TELEMETRY ALOUD"* to hear live audio updates of your current position and alerts through your phone or earphones.
* **CSV Logging:** Telemetry is automatically saved to `telemetry_log.csv` and can be downloaded with one click.

---

## License
MIT License. Created by [Devansh Purohit](https://github.com/devanshvpurohit).
