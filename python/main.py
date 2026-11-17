#!/usr/bin/env python3
"""
Standalone Autonomous Geo-Aware YOLO Vision Rover for Arduino UNO Q (4GB)
Communicates with the STM32 MCU via the native Arduino Router Bridge (RPC),
completely removing dependency on legacy hardware serial ports:
 - Native Bridge RPC calls: drive_motor() and get_telemetry()
 - Embedded Flask web server (Camera stream + Telemetry HUD + Manual/Auto Controls)
 - Live YOLOv8 inference directly on Qualcomm MPU (Debian 13)
 - Geo-tagged CSV event logging
"""

import cv2
import csv
import os
import sys
import time
import threading
from datetime import datetime
from flask import Flask, render_template, Response, jsonify, request, send_file
from ultralytics import YOLO

# Initialize Flask app
app = Flask(__name__, template_folder=os.path.join(os.path.dirname(__file__), 'templates'))

# Connect to the Arduino Router Bridge
bridge = None
HAS_BRIDGE = False

try:
    try:
        from arduino.router_bridge import Bridge
    except ImportError:
        from arduino_router import Bridge

    print("[UNO Q] Connecting to Arduino Router daemon via RPC...")
    bridge = Bridge()
    if hasattr(bridge, 'connect'):
        bridge.connect(timeout=5)
    HAS_BRIDGE = True
    print("[UNO Q] Successfully connected to Arduino Router Bridge!")
except Exception as err:
    print(f"[UNO Q] Bridge connection notice: {err}. Operating in standalone/simulation mode.")
    bridge = None
    HAS_BRIDGE = False

# Global telemetry cache
telemetry = {
    "lat": 0.0,
    "lng": 0.0,
    "speed": 0.0,
    "ax": 0.0,
    "ay": 0.0,
    "gz": 0.0,
    "last_target": "None",
    "current_action": "S",
    "mode": "auto",  # 'auto' or 'manual'
    "events": []
}
telemetry_lock = threading.Lock()
current_frame = None
frame_lock = threading.Lock()

LOG_FILE = os.path.join(os.path.dirname(__file__), "..", "geo_detections_log.csv")
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Class", "Confidence", "Lat", "Lng", "Speed_kmh", "Pitch_Ax", "Roll_Ay", "Action"])

def log_event(cls_name, conf, action):
    now_str = datetime.utcnow().strftime("%H:%M:%S")
    with telemetry_lock:
        t_data = telemetry.copy()
        telemetry["events"].append({"time": now_str, "text": f"{cls_name} ({conf:.2f}) -> {action}"})
        if len(telemetry["events"]) > 20:
            telemetry["events"].pop(0)

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

def send_command(cmd):
    with telemetry_lock:
        telemetry["current_action"] = cmd

    if HAS_BRIDGE and bridge:
        try:
            bridge.call("drive_motor", cmd)
        except Exception as e:
            # Non-blocking RPC error logging
            pass

def telemetry_poller():
    global telemetry
    while True:
        if HAS_BRIDGE and bridge:
            try:
                raw_tel = bridge.call("get_telemetry")
                if raw_tel and isinstance(raw_tel, str) and raw_tel.startswith("$TEL"):
                    parts = raw_tel.split(',')
                    if len(parts) >= 7:
                        with telemetry_lock:
                            telemetry["lat"] = float(parts[1])
                            telemetry["lng"] = float(parts[2])
                            telemetry["speed"] = float(parts[3])
                            telemetry["ax"] = float(parts[4])
                            telemetry["ay"] = float(parts[5])
                            telemetry["gz"] = float(parts[6])
            except Exception:
                pass
        time.sleep(0.2)  # 5Hz polling

t = threading.Thread(target=telemetry_poller, daemon=True)
t.start()

# Web Endpoints
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/telemetry')
def get_telemetry():
    with telemetry_lock:
        return jsonify(telemetry)

@app.route('/api/control/<cmd>', methods=['POST'])
def control_rover(cmd):
    with telemetry_lock:
        if telemetry["mode"] == "manual":
            send_command(cmd)
            return jsonify({"status": "ok", "command": cmd})
        return jsonify({"status": "ignored", "reason": "in auto mode"})

@app.route('/api/mode/<new_mode>', methods=['POST'])
def set_mode(new_mode):
    with telemetry_lock:
        if new_mode in ["auto", "manual"]:
            telemetry["mode"] = new_mode
            if new_mode == "manual":
                send_command('S')
            return jsonify({"status": "ok", "mode": new_mode})
        return jsonify({"status": "error", "message": "invalid mode"}), 400

@app.route('/api/download_log')
def download_log():
    if os.path.exists(LOG_FILE):
        return send_file(LOG_FILE, as_attachment=True, download_name="geo_detections_log.csv")
    return "Log file empty", 404

def generate_video_stream():
    global current_frame
    while True:
        with frame_lock:
            if current_frame is None:
                time.sleep(0.03)
                continue
            ret, buffer = cv2.imencode('.jpg', current_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.04)

@app.route('/video_feed')
def video_feed():
    return Response(generate_video_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Vision Inference Engine
def vision_worker():
    global current_frame
    print("[UNO Q] Initializing YOLOv8n model...")
    model = YOLO("yolov8n.pt")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[UNO Q] Camera index 0 not found. Attempting index 1...")
        cap = cv2.VideoCapture(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    last_logged_time = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.03)
                continue

            results = model(frame, verbose=False, conf=0.45)
            boxes = results[0].boxes

            frame_w = frame.shape[1]
            mid_left = frame_w * 0.35
            mid_right = frame_w * 0.65

            action = 'F'
            obstacle_present = False
            detected_label = "None"

            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                # Draw UI overlays on stream
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 240, 255), 2)
                cv2.putText(frame, f"{cls_name} {conf:.2f}", (int(x1), int(y1) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 240, 255), 1)

                cx = (x1 + x2) / 2
                area = (x2 - x1) * (y2 - y1)
                total_area = frame.shape[0] * frame.shape[1]
                detected_label = f"{cls_name} ({conf:.2f})"

                if area > (total_area * 0.12):
                    obstacle_present = True
                    if cx < mid_left:
                        action = 'R'
                    elif cx > mid_right:
                        action = 'L'
                    else:
                        action = 'S'

                    if time.time() - last_logged_time > 1.2:
                        last_logged_time = time.time()
                        log_event(cls_name, conf, action)

            with telemetry_lock:
                telemetry["last_target"] = detected_label
                mode = telemetry["mode"]

            if mode == "auto":
                if not obstacle_present:
                    action = 'F'
                send_command(action)

            with frame_lock:
                current_frame = frame.copy()

            time.sleep(0.03)

    finally:
        send_command('S')
        cap.release()

if __name__ == '__main__':
    vision_thread = threading.Thread(target=vision_worker, daemon=True)
    vision_thread.start()

    port = int(os.environ.get("PORT", 5000))
    print(f"\n==========================================================")
    print(f"🚀 UNO Q Rover Web Dashboard: http://0.0.0.0:{port}")
    print(f"   Open on your phone or PC: http://unoq.local:{port}")
    print(f"==========================================================\n")
    app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False)
