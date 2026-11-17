#!/usr/bin/env python3
"""
SURDAS: Real-Time Assistive Vision, Spatial AI & Telemetry Engine for Arduino UNO Q (4GB)
- Hardware Telemetry: GPS (NEO-6M), IMU (MPU-6050, fall detection) via native monitor stream
- Vision & Spatial AI: YOLOv8n object detection + MiDaS depth estimation via ONNX Runtime
- Threaded WSGI server ensures zero blocking between video stream and telemetry API
"""

import csv
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from datetime import datetime
from flask import Flask, render_template, jsonify, send_file, Response

from ai_vision import AIVisionEngine

app = Flask(__name__, template_folder=os.path.join(os.path.dirname(__file__), 'templates'))

# Initialize AI Vision Engine
ai_engine = AIVisionEngine(models_dir=os.path.join(os.path.dirname(__file__), "..", "models"))
ai_engine.start()

# Shared Telemetry State
telemetry = {
    "imu_ok": False,
    "lat": 0.0,
    "lng": 0.0,
    "sats": 0,
    "speed": 0.0,
    "altitude": 0.0,
    "ax": 0.0,
    "ay": 0.0,
    "az": 0.0,
    "gx": 0.0,
    "gy": 0.0,
    "gz": 0.0,
    "pitch": 0.0,
    "roll": 0.0,
    "temp": 0.0,
    "fall_alert": False,
    "gps_bytes": 0,
    "bridge_connected": True,
    "last_update": "None",
    "ai_guidance": "SURDAS vision scanning environment...",
    "detected_objects": [],
    "spatial_sectors": {"left": 0.0, "center": 0.0, "right": 0.0},
    "events": []
}
telemetry_lock = threading.Lock()

LOG_FILE = os.path.join(os.path.dirname(__file__), "..", "telemetry_log.csv")
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Lat", "Lng", "Sats", "Speed_kmh", "Alt_m", "Pitch", "Roll", "Ax", "Ay", "Az", "Fall_Alert"])

def update_fall_detection(ax, ay, az):
    total_g = math.sqrt(ax * ax + ay * ay + az * az)
    return (total_g > 2.8 or (0.02 < total_g < 0.25))

def parse_line(line, state):
    """Parses live formatted serial lines from MCU"""
    line = line.strip()
    if not line:
        return

    if 'ACCELEROMETER' in line:
        state['section'] = 'accel'
    elif 'GYROSCOPE' in line:
        state['section'] = 'gyro'
    elif 'ORIENTATION' in line:
        state['section'] = 'orient'
    elif 'NEO-6M GPS' in line:
        state['section'] = 'gps'
    elif 'IMU DATA' in line:
        state['section'] = 'imu'
    elif line.startswith('IMU STATUS:'):
        with telemetry_lock:
            telemetry['imu_ok'] = ('WORKING' in line)
            if not telemetry['imu_ok']:
                telemetry['imu_status_msg'] = line.split(':', 1)[1].strip()
    elif line.startswith('X:'):
        try:
            val = float(line.split(':', 1)[1].strip())
            with telemetry_lock:
                if state.get('section') == 'accel': telemetry['ax'] = val
                elif state.get('section') == 'gyro': telemetry['gx'] = val
        except ValueError:
            pass
    elif line.startswith('Y:'):
        try:
            val = float(line.split(':', 1)[1].strip())
            with telemetry_lock:
                if state.get('section') == 'accel': telemetry['ay'] = val
                elif state.get('section') == 'gyro': telemetry['gy'] = val
        except ValueError:
            pass
    elif line.startswith('Z:'):
        try:
            val = float(line.split(':', 1)[1].strip())
            with telemetry_lock:
                if state.get('section') == 'accel': telemetry['az'] = val
                elif state.get('section') == 'gyro': telemetry['gz'] = val
        except ValueError:
            pass
    elif line.startswith('ROLL:'):
        try:
            r = float(line.split(':', 1)[1].replace('degrees', '').replace('deg', '').strip())
            with telemetry_lock:
                if abs(r) > 0.01:
                    telemetry['roll'] = r
        except ValueError:
            pass
    elif line.startswith('PITCH:'):
        try:
            p = float(line.split(':', 1)[1].replace('degrees', '').replace('deg', '').strip())
            with telemetry_lock:
                if abs(p) > 0.01:
                    telemetry['pitch'] = p
        except ValueError:
            pass
    elif line.startswith('TEMPERATURE:'):
        try:
            val = float(line.split(':', 1)[1].replace('C', '').strip())
            with telemetry_lock:
                telemetry['temp'] = val
        except ValueError:
            pass
    elif 'GPS BYTES:' in line or 'GPS RAW BYTES:' in line:
        try:
            val = int(line.split(':', 1)[1].strip())
            with telemetry_lock:
                telemetry['gps_bytes'] = val
        except ValueError:
            pass
    elif line.startswith('SATELLITES:'):
        val_str = line.split(':', 1)[1].strip()
        try:
            sats = int(val_str)
            with telemetry_lock:
                telemetry['sats'] = sats
        except ValueError:
            pass
    elif line.startswith('LATITUDE:') or line.startswith('LAT:'):
        try:
            val = float(line.split(':', 1)[1].strip())
            with telemetry_lock:
                telemetry['lat'] = val
        except ValueError:
            pass
    elif line.startswith('LONGITUDE:') or line.startswith('LNG:'):
        try:
            val = float(line.split(':', 1)[1].strip())
            with telemetry_lock:
                telemetry['lng'] = val
        except ValueError:
            pass
    elif line.startswith('ALTITUDE:'):
        try:
            val = float(line.split(':', 1)[1].replace('m', '').strip())
            with telemetry_lock:
                telemetry['altitude'] = val
        except ValueError:
            pass
    elif line.startswith('SPEED:'):
        try:
            val = float(line.split(':', 1)[1].replace('km/h', '').strip())
            with telemetry_lock:
                telemetry['speed'] = val
        except ValueError:
            pass

    # Post-process calculated fields
    with telemetry_lock:
        telemetry['last_update'] = datetime.now().strftime("%H:%M:%S")
        telemetry['bridge_connected'] = True

        # Compute dynamic pitch/roll from accelerometer if angles are default
        if telemetry['imu_ok']:
            ax, ay, az = telemetry['ax'], telemetry['ay'], telemetry['az']
            if abs(ax) > 0.001 or abs(ay) > 0.001 or abs(az) > 0.001:
                telemetry['roll'] = round(math.atan2(ay, az) * 180.0 / math.pi, 1)
                telemetry['pitch'] = round(math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi, 1)

        # Fall detection
        is_fall = update_fall_detection(telemetry['ax'], telemetry['ay'], telemetry['az'])
        if is_fall and not telemetry['fall_alert']:
            telemetry['fall_alert'] = True
            telemetry['events'].append({
                "time": telemetry['last_update'],
                "text": f"⚠️ FALL DETECTED at Lat: {telemetry['lat']:.5f}, Lng: {telemetry['lng']:.5f}"
            })
            if len(telemetry['events']) > 20:
                telemetry['events'].pop(0)
        elif not is_fall:
            telemetry['fall_alert'] = False

def monitor_stream_worker():
    """Background thread that reads telemetry from 'arduino-app-cli monitor'"""
    state = {'section': ''}
    last_print = 0

    while True:
        try:
            print("[SURDAS] Connecting to MCU serial monitor via arduino-app-cli...", flush=True)
            proc = subprocess.Popen(
                ['stdbuf', '-oL', '-eL', 'arduino-app-cli', 'monitor'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )

            while proc.poll() is None:
                line = proc.stdout.readline()
                if not line:
                    break
                parse_line(line, state)

                if time.time() - last_print >= 1.0:
                    last_print = time.time()
                    with telemetry_lock:
                        telemetry['ai_guidance'] = ai_engine.latest_guidance
                        telemetry['detected_objects'] = [d["class"] for d in ai_engine.latest_detections]
                        telemetry['spatial_sectors'] = ai_engine.spatial_sectors
                        t = telemetry.copy()

                    gps_str = f"{t['lat']:.5f}, {t['lng']:.5f} ({t['sats']} Sats, {t['speed']:.1f} km/h)" if t['lat'] != 0.0 else f"Searching ({t['sats']} Sats, {t['gps_bytes']} bytes rx)"
                    imu_str = f"OK (P:{t['pitch']:.1f}°, R:{t['roll']:.1f}°)" if t['imu_ok'] else "OFF"
                    print(f"[SURDAS] IMU: {imu_str} | GPS: {gps_str} | AI: {t['ai_guidance'][:40]}", flush=True)

            proc.wait()
        except FileNotFoundError:
            print("[SURDAS] 'arduino-app-cli' not found in PATH, retrying in 3s...", flush=True)
            time.sleep(3)
        except Exception as e:
            print(f"[SURDAS] Monitor stream error: {e}, reconnecting in 2s...", flush=True)
            time.sleep(2)

# Start serial stream worker
worker_thread = threading.Thread(target=monitor_stream_worker, daemon=True)
worker_thread.start()

# Flask Endpoints
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Live video stream with YOLO detections & MiDaS depth overlay"""
    return Response(ai_engine.get_jpeg_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/telemetry')
def get_telemetry_api():
    """Instant JSON response: never blocked by video feed"""
    with telemetry_lock:
        telemetry['ai_guidance'] = ai_engine.latest_guidance
        telemetry['detected_objects'] = [d["class"] for d in ai_engine.latest_detections]
        telemetry['spatial_sectors'] = ai_engine.spatial_sectors
        return jsonify(telemetry)

@app.route('/api/download_log')
def download_log():
    if os.path.exists(LOG_FILE):
        return send_file(LOG_FILE, as_attachment=True, download_name="telemetry_log.csv")
    return "Log file empty", 404

if __name__ == '__main__':
    port = int(os.environ.get("PORT", 5000))
    print(f"\n==========================================================", flush=True)
    print(f"📍 SURDAS Assistive AI Prototype Live: http://0.0.0.0:{port}", flush=True)
    print(f"   Open in browser: http://unoq.local:{port} or http://192.168.29.84:{port}", flush=True)
    print(f"==========================================================\n", flush=True)
    # threaded=True is CRITICAL: ensures video stream doesn't block telemetry API
    app.run(host='0.0.0.0', port=port, threaded=True, debug=False, use_reloader=False)
