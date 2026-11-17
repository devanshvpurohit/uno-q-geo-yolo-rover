#!/usr/bin/env python3
"""
Autonomous Geo-Aware YOLO Vision Core for Arduino UNO Q (4GB)
Runs on Qualcomm Dragonwing QRB2210 (Debian Linux).
Integrates:
 - Real-time YOLOv8 object detection
 - Telemetry stream (GPS + IMU) from STM32 MCU
 - Reactive navigation decisions sent back to MCU
 - Geo-tagged CSV event logging
"""

import cv2
import csv
import glob
import os
import sys
import time
import serial
import threading
from datetime import datetime
from ultralytics import YOLO

# Detect serial bridge between Linux MPU and STM32 MCU
possible_ports = (
    glob.glob('/dev/ttyRPMSG*') +
    glob.glob('/dev/ttyACM*') +
    glob.glob('/dev/ttyUSB*') +
    ['/dev/ttyS0']
)
serial_port = possible_ports[0] if possible_ports else '/dev/ttyS0'

print(f"[UNO Q] Connecting to MCU via port: {serial_port}...")
try:
    mcu_serial = serial.Serial(serial_port, 115200, timeout=0.1)
    time.sleep(1.5)
    print(f"[UNO Q] Connected to STM32 MCU successfully.")
except Exception as err:
    print(f"[UNO Q] Notice: Serial connection failed ({err}). Operating in standalone/simulation mode.")
    mcu_serial = None

# Shared telemetry cache
telemetry = {
    "lat": 0.0,
    "lng": 0.0,
    "speed": 0.0,
    "ax": 0.0,
    "ay": 0.0,
    "gz": 0.0,
    "timestamp": 0.0
}
telemetry_lock = threading.Lock()

# CSV Logging
LOG_FILE = "geo_detections_log.csv"
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Class", "Confidence", "Lat", "Lng", "Speed_kmh", "Pitch_Ax", "Roll_Ay", "Action"])

def log_event(cls_name, conf, action):
    with telemetry_lock:
        t_data = telemetry.copy()
    try:
        with open(LOG_FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.utcnow().isoformat(),
                cls_name,
                f"{conf:.2f}",
                t_data["lat"],
                t_data["lng"],
                t_data["speed"],
                t_data["ax"],
                t_data["ay"],
                action
            ])
    except Exception as e:
        print(f"[LOG ERROR] {e}")

def telemetry_listener():
    global telemetry
    while mcu_serial and mcu_serial.is_open:
        try:
            line = mcu_serial.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("$TEL"):
                parts = line.split(',')
                if len(parts) >= 7:
                    with telemetry_lock:
                        telemetry["lat"] = float(parts[1])
                        telemetry["lng"] = float(parts[2])
                        telemetry["speed"] = float(parts[3])
                        telemetry["ax"] = float(parts[4])
                        telemetry["ay"] = float(parts[5])
                        telemetry["gz"] = float(parts[6])
                        telemetry["timestamp"] = time.time()
        except Exception:
            pass

# Launch background telemetry thread
t = threading.Thread(target=telemetry_listener, daemon=True)
t.start()

def send_command(cmd):
    if mcu_serial and mcu_serial.is_open:
        try:
            mcu_serial.write(cmd.encode())
        except Exception as e:
            print(f"[SERIAL SEND ERROR] {e}")

def main():
    print("[UNO Q] Loading YOLOv8n (optimized for 4GB RAM + Adreno GPU)...")
    model = YOLO("yolov8n.pt")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[UNO Q] Error: Could not open camera (index 0). Retrying index 1...")
        cap = cv2.VideoCapture(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("[UNO Q] Mission loop active. Press Ctrl+C to terminate.")

    last_logged_time = 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.02)
                continue

            results = model(frame, verbose=False, conf=0.45)
            boxes = results[0].boxes

            frame_w = frame.shape[1]
            mid_left = frame_w * 0.35
            mid_right = frame_w * 0.65

            action = 'F' # Default cruising
            obstacle_present = False

            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                cx = (x1 + x2) / 2
                area = (x2 - x1) * (y2 - y1)
                total_area = frame.shape[0] * frame.shape[1]

                # Check if obstacle is close (occupying >12% of field of view)
                if area > (total_area * 0.12):
                    obstacle_present = True
                    if cx < mid_left:
                        action = 'R'  # Steer Right
                    elif cx > mid_right:
                        action = 'L'  # Steer Left
                    else:
                        action = 'S'  # Stop / Emergency brake

                    if time.time() - last_logged_time > 1.0:
                        last_logged_time = time.time()
                        log_event(cls_name, conf, action)
                        with telemetry_lock:
                            print(f"[TARGET] {cls_name} ({conf:.2f}) | Lat: {telemetry['lat']}, Lng: {telemetry['lng']} | Action: {action}")

            if not obstacle_present:
                action = 'F'

            send_command(action)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[UNO Q] Shutting down rover cleanly...")
    finally:
        send_command('S')
        cap.release()
        if mcu_serial and mcu_serial.is_open:
            mcu_serial.close()
        print("[UNO Q] System stopped.")

if __name__ == "__main__":
    main()
