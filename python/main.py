#!/usr/bin/env python3
"""
SURDAS: Real-Time GPS & IMU Telemetry Engine for Arduino UNO Q (4GB)
Dedicated to GPS location tracking, 6-DOF IMU orientation, and Fall Alert.
Lightweight, instant startup, zero compute lag.
"""

import csv
import os
import sys
import time
import threading
from datetime import datetime
from flask import Flask, render_template, jsonify, send_file

app = Flask(__name__, template_folder=os.path.join(os.path.dirname(__file__), 'templates'))

# Connect to the Arduino Router Bridge
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
    print(f"[SURDAS] Router Bridge notice: {err}. Operating in simulated mode.")
    bridge = None
    HAS_BRIDGE = False

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
    "fall_alert": False,
    "last_update": "None",
    "events": []
}
telemetry_lock = threading.Lock()

LOG_FILE = os.path.join(os.path.dirname(__file__), "..", "telemetry_log.csv")
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Lat", "Lng", "Sats", "Speed_kmh", "Alt_m", "Pitch", "Roll", "Ax", "Ay", "Az", "Fall_Alert"])

def log_telemetry():
    with telemetry_lock:
        t = telemetry.copy()
    try:
        with open(LOG_FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.utcnow().isoformat(),
                t["lat"], t["lng"], t["sats"], t["speed"], t["altitude"],
                t["pitch"], t["roll"], t["ax"], t["ay"], t["az"], t["fall_alert"]
            ])
    except Exception:
        pass

# Background Telemetry Reader (10Hz)
def telemetry_worker():
    global telemetry
    last_print = 0
    last_log = 0

    while True:
        if HAS_BRIDGE and bridge:
            try:
                # Format: $TEL,imu_ok,lat,lng,sats,speed_kmh,alt_m,ax,ay,az,gx,gy,gz,pitch,roll,fall
                raw = bridge.call("get_telemetry")
                if raw and isinstance(raw, str) and raw.startswith("$TEL"):
                    p = raw.split(',')
                    if len(p) >= 16:
                        with telemetry_lock:
                            telemetry["imu_ok"] = (p[1] == "1")
                            telemetry["lat"] = float(p[2])
                            telemetry["lng"] = float(p[3])
                            telemetry["sats"] = int(p[4])
                            telemetry["speed"] = float(p[5])
                            telemetry["altitude"] = float(p[6])
                            telemetry["ax"] = float(p[7])
                            telemetry["ay"] = float(p[8])
                            telemetry["az"] = float(p[9])
                            telemetry["gx"] = float(p[10])
                            telemetry["gy"] = float(p[11])
                            telemetry["gz"] = float(p[12])
                            telemetry["pitch"] = float(p[13])
                            telemetry["roll"] = float(p[14])
                            telemetry["fall_alert"] = (p[15] == "1")
                            telemetry["last_update"] = datetime.now().strftime("%H:%M:%S")

                            if telemetry["fall_alert"]:
                                telemetry["events"].append({
                                    "time": telemetry["last_update"],
                                    "text": f"⚠️ FALL ALERT triggered at {telemetry['lat']}, {telemetry['lng']}"
                                })
                                if len(telemetry["events"]) > 20:
                                    telemetry["events"].pop(0)

                        # Print clean summary to terminal every second
                        if time.time() - last_print >= 1.0:
                            last_print = time.time()
                            gps_str = f"Lat: {p[2]}, Lng: {p[3]} ({p[4]} Sats)" if (float(p[2]) != 0.0) else f"Searching Sats ({p[4]} tracked)"
                            print(f"[SURDAS] IMU: {'OK' if p[1]=='1' else 'OFF'} | "
                                  f"Pitch: {p[13]}°, Roll: {p[14]}° | "
                                  f"GPS: {gps_str} | "
                                  f"Fall: {'⚠️ ALERT!' if p[15]=='1' else 'Normal'}")

                        # Log to CSV every 2 seconds
                        if time.time() - last_log >= 2.0:
                            last_log = time.time()
                            log_telemetry()
            except Exception:
                pass

        time.sleep(0.1)

t = threading.Thread(target=telemetry_worker, daemon=True)
t.start()

# Web Endpoints
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
        return send_file(LOG_FILE, as_attachment=True, download_name="telemetry_log.csv")
    return "Log file empty", 404

if __name__ == '__main__':
    port = int(os.environ.get("PORT", 5000))
    print(f"\n==========================================================")
    print(f"📍 SURDAS GPS + IMU Live Dashboard: http://0.0.0.0:{port}")
    print(f"   Open on your phone or PC: http://unoq.local:{port}")
    print(f"==========================================================\n")
    app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False)
