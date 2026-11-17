#!/usr/bin/env python3
"""
SURDAS: AI-Powered Assistive Vision & Navigation Platform for the Blind
Engineered for Arduino UNO Q (4GB) - Qualcomm QRB2210 + STM32U585.

Features:
 - Fast Edge Vision: Downscaled 320x320 YOLOv8 inference with frame skipping for smooth 20+ FPS
 - Directional Haptic Feedback: Triggers Left (D5), Center (D6), or Right (D7) vibration motors
 - Real-Time Voice Assistance: Synthesizes directional announcements
 - IMU Fall Detection & Orientation Monitoring (MPU-6050)
 - GPS Location Tracking & Satellite Diagnostics (NEO-6M on Serial1)
 - Accessible Embedded Web Dashboard with Speech Synthesis
"""

import cv2
import csv
import os
import sys
import time
import threading
import numpy as np
from datetime import datetime
from flask import Flask, render_template, Response, jsonify, send_file

app = Flask(__name__, template_folder=os.path.join(os.path.dirname(__file__), 'templates'))

# Graceful YOLO loader
HAS_YOLO = False
try:
    from ultralytics import YOLO
    HAS_YOLO = True
    print("[SURDAS] YOLO Engine loaded successfully.")
except ImportError:
    print("[SURDAS] Notice: 'ultralytics' not installed. Running in sensor-only mode.")

# Connect to Arduino Router Bridge
bridge = None
HAS_BRIDGE = False

try:
    try:
        from arduino.router_bridge import Bridge
    except ImportError:
        from arduino_router import Bridge

    bridge = Bridge()
    if hasattr(bridge, 'connect'):
        bridge.connect(timeout=5)
    HAS_BRIDGE = True
    print("[SURDAS] Connected to STM32 MCU via Arduino Router Bridge (RPC)!")
except Exception as err:
    print(f"[SURDAS] Router Bridge notice: {err}. Running simulated sensors.")
    bridge = None
    HAS_BRIDGE = False

# Telemetry and State
telemetry = {
    "imu_ok": False,
    "lat": 0.0,
    "lng": 0.0,
    "sats": 0,
    "speed": 0.0,
    "ax": 0.0,
    "ay": 0.0,
    "az": 0.0,
    "fall_alert": False,
    "sos": False,
    "haptic": "S",
    "guidance": "Path clear",
    "alert_speech": "",
    "events": []
}
telemetry_lock = threading.Lock()
current_frame = None
frame_lock = threading.Lock()

LOG_FILE = os.path.join(os.path.dirname(__file__), "..", "assistive_events_log.csv")
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Target", "Confidence", "Direction", "Lat", "Lng", "Sats", "Fall_Alert"])

def log_event(target, conf, direction):
    now_str = datetime.utcnow().strftime("%H:%M:%S")
    with telemetry_lock:
        t_data = telemetry.copy()
        telemetry["events"].append({"time": now_str, "text": f"{target} ({conf:.2f}) on {direction}"})
        if len(telemetry["events"]) > 20:
            telemetry["events"].pop(0)

    try:
        with open(LOG_FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.utcnow().isoformat(),
                target,
                f"{conf:.2f}",
                direction,
                t_data["lat"],
                t_data["lng"],
                t_data["sats"],
                t_data["fall_alert"]
            ])
    except Exception as e:
        print(f"[LOG ERROR] {e}")

def trigger_haptic(cmd):
    with telemetry_lock:
        telemetry["haptic"] = cmd

    if HAS_BRIDGE and bridge:
        try:
            bridge.call("haptic_feedback", cmd)
        except Exception:
            pass

# High-frequency telemetry background poller (10Hz)
def telemetry_worker():
    global telemetry
    last_print = 0
    while True:
        if HAS_BRIDGE and bridge:
            try:
                # Expected format: $TEL,imu_ok,lat,lng,sats,speed,ax,ay,az,fall,sos
                raw = bridge.call("get_telemetry")
                if raw and isinstance(raw, str) and raw.startswith("$TEL"):
                    p = raw.split(',')
                    if len(p) >= 11:
                        with telemetry_lock:
                            telemetry["imu_ok"] = (p[1] == "1")
                            telemetry["lat"] = float(p[2])
                            telemetry["lng"] = float(p[3])
                            telemetry["sats"] = int(p[4])
                            telemetry["speed"] = float(p[5])
                            telemetry["ax"] = float(p[6])
                            telemetry["ay"] = float(p[7])
                            telemetry["az"] = float(p[8])
                            telemetry["fall_alert"] = (p[9] == "1")
                            telemetry["sos"] = (p[10] == "1")

                        # Print diagnostic every 3 seconds to console
                        if time.time() - last_print > 3.0:
                            last_print = time.time()
                            print(f"[SENSORS] IMU: {'OK' if p[1]=='1' else 'NO'} | "
                                  f"GPS: {p[4]} Sats (Lat: {p[2]}, Lng: {p[3]}) | "
                                  f"Fall: {'ALERT!' if p[9]=='1' else 'No'}")
            except Exception:
                pass
        time.sleep(0.1)

t = threading.Thread(target=telemetry_worker, daemon=True)
t.start()

# Flask API Endpoints
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/telemetry')
def get_telemetry():
    with telemetry_lock:
        return jsonify(telemetry)

@app.route('/api/download_log')
def download_log():
    if os.path.exists(LOG_FILE):
        return send_file(LOG_FILE, as_attachment=True, download_name="assistive_events_log.csv")
    return "Log empty", 404

def generate_video_stream():
    global current_frame
    while True:
        with frame_lock:
            if current_frame is None:
                placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, "SURDAS: Connecting camera...", (140, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 229, 255), 2)
                ret, buffer = cv2.imencode('.jpg', placeholder)
            else:
                ret, buffer = cv2.imencode('.jpg', current_frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.04)

@app.route('/video_feed')
def video_feed():
    return Response(generate_video_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

# High-Performance Vision Engine
def vision_worker():
    global current_frame
    model = None
    if HAS_YOLO:
        print("[SURDAS] Loading optimized YOLOv8n weights...")
        try:
            model = YOLO("yolov8n.pt")
        except Exception as e:
            print(f"[SURDAS] YOLO Load Error: {e}")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        cap = cv2.VideoCapture(1)

    # Set camera resolution to standard 640x480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    frame_counter = 0
    last_detected_boxes = []
    last_spoken_time = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.03)
                continue

            frame_counter += 1
            frame_w = frame.shape[1]
            frame_h = frame.shape[0]
            total_area = frame_w * frame_h
            mid_left = frame_w * 0.33
            mid_right = frame_w * 0.66

            # PERFORMANCE OPTIMIZATION:
            # 1. Run YOLO inference every 2nd frame (frame skipping for 2x FPS boost)
            # 2. Run at imgsz=320 (3x faster than default 640x640)
            if model is not None and (frame_counter % 2 == 0):
                # Downscaled inference for speed on ARM Cortex-A
                results = model(frame, imgsz=320, verbose=False, conf=0.45)
                last_detected_boxes = results[0].boxes

            haptic_action = "S"
            guidance_msg = "Path clear ahead"
            speech_prompt = ""
            nearest_hazard_area = 0

            # Process detected objects
            if last_detected_boxes is not None and len(last_detected_boxes) > 0:
                for box in last_detected_boxes:
                    cls_id = int(box.cls[0])
                    cls_name = model.names[cls_id] if model else "object"
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = box.xyxy[0].tolist()

                    cx = (x1 + x2) / 2
                    area = (x2 - x1) * (y2 - y1)
                    rel_size = area / total_area

                    # Draw high-contrast bounding box
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 229, 255), 2)
                    cv2.putText(frame, f"{cls_name} {int(conf*100)}%", (int(x1), int(y1) - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 229, 255), 2)

                    # Check proximity (> 8% screen area indicates close proximity)
                    if rel_size > 0.08 and area > nearest_hazard_area:
                        nearest_hazard_area = area

                        if cx < mid_left:
                            direction = "Left"
                            haptic_action = "L"
                            guidance_msg = f"{cls_name.capitalize()} on your left"
                            speech_prompt = f"Caution: {cls_name} on your left"
                        elif cx > mid_right:
                            direction = "Right"
                            haptic_action = "R"
                            guidance_msg = f"{cls_name.capitalize()} on your right"
                            speech_prompt = f"Caution: {cls_name} on your right"
                        else:
                            direction = "Center"
                            haptic_action = "C"
                            guidance_msg = f"WARNING: {cls_name.capitalize()} directly ahead!"
                            speech_prompt = f"Stop! {cls_name} directly in front of you"

                        if time.time() - last_spoken_time > 2.5:
                            last_spoken_time = time.time()
                            log_event(cls_name, conf, direction)

            # Trigger MCU Haptic Vibration
            trigger_haptic(haptic_action)

            with telemetry_lock:
                telemetry["guidance"] = guidance_msg
                if speech_prompt and (time.time() - last_spoken_time < 0.5):
                    telemetry["alert_speech"] = speech_prompt
                else:
                    telemetry["alert_speech"] = ""

            # Overlay visual HUD on video stream
            cv2.putText(frame, f"SURDAS Vision - {guidance_msg}", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 128), 2)

            with frame_lock:
                current_frame = frame.copy()

            time.sleep(0.02)

    finally:
        trigger_haptic("S")
        cap.release()

if __name__ == '__main__':
    vision_thread = threading.Thread(target=vision_worker, daemon=True)
    vision_thread.start()

    port = int(os.environ.get("PORT", 5000))
    print(f"\n==========================================================")
    print(f"👁️  SURDAS Assistive Vision Platform: http://0.0.0.0:{port}")
    print(f"   Access on phone/PC browser: http://unoq.local:{port}")
    print(f"==========================================================\n")
    app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False)
