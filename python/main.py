#!/usr/bin/env python3
"""
SURDAS: Real-Time GPS & IMU Telemetry Engine for Arduino UNO Q (4GB)
Optimized RPC Bridge communication with STM32 MCU
"""

import csv
import os
import sys
import time
import threading
from datetime import datetime
from flask import Flask, render_template, jsonify, send_file

app = Flask(__name__, template_folder=os.path.join(os.path.dirname(__file__), 'templates'))

BridgeObj = None
HAS_BRIDGE = False

def connect_bridge(timeout=3):
    global BridgeObj, HAS_BRIDGE
    try:
        from arduino.router_bridge import Bridge
        b = Bridge()
        b.connect(timeout=timeout)
        BridgeObj = b
        HAS_BRIDGE = True
        print("[SURDAS] Connected to Arduino Router Bridge!")
        return b
    except Exception as e:
        try:
            from arduino.app_utils import Bridge
            BridgeObj = Bridge
            HAS_BRIDGE = True
            print("[SURDAS] Connected using arduino.app_utils.Bridge!")
            return Bridge
        except Exception as e2:
            HAS_BRIDGE = False
            return None

# Initial connection attempt
connect_bridge(timeout=2)

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
    "gps_bytes": 0,
    "bridge_connected": HAS_BRIDGE,
    "last_update": "None",
    "events": []
}
telemetry_lock = threading.Lock()

LOG_FILE = os.path.join(os.path.dirname(__file__), "..", "telemetry_log.csv")
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Lat", "Lng", "Sats", "Speed_kmh", "Alt_m", "Pitch", "Roll", "Ax", "Ay", "Az", "Fall_Alert"])

def parse_telemetry_packet(raw):
    global telemetry
    if not raw or not isinstance(raw, str) or not raw.startswith("$TEL"):
        return False

    p = raw.strip().split(',')
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
            if len(p) >= 17:
                telemetry["gps_bytes"] = int(p[16])
            if len(p) >= 18:
                telemetry["imu_status_msg"] = p[17]
            telemetry["bridge_connected"] = True
            telemetry["last_update"] = datetime.now().strftime("%H:%M:%S")

            if telemetry["fall_alert"]:
                telemetry["events"].append({
                    "time": telemetry["last_update"],
                    "text": f"⚠️ FALL ALERT at {telemetry['lat']}, {telemetry['lng']}"
                })
                if len(telemetry["events"]) > 20:
                    telemetry["events"].pop(0)
        return True
    return False

# Register Bridge callback for pushed updates
if HAS_BRIDGE and BridgeObj:
    try:
        def on_telemetry(data):
            parse_telemetry_packet(str(data))

        if hasattr(BridgeObj, 'provide'):
            BridgeObj.provide("on_telemetry", on_telemetry)
    except Exception as e:
        pass

# Active Telemetry Polling Thread with Auto-Reconnect
def telemetry_worker():
    last_print = 0
    consecutive_fails = 0

    while True:
        b = BridgeObj
        if b is None or consecutive_fails > 5:
            b = connect_bridge(timeout=1)
            consecutive_fails = 0

        if b is not None:
            try:
                # Query MCU for telemetry
                raw = b.call("get_telemetry")
                if raw is not None:
                    parse_telemetry_packet(str(raw))
                    consecutive_fails = 0
                else:
                    consecutive_fails += 1
            except Exception as e:
                consecutive_fails += 1
                # Try reconnecting if disconnected
                if "Not connected" in str(e) or "send failed" in str(e):
                    try:
                        if hasattr(b, 'connect'):
                            b.connect(timeout=2)
                    except Exception:
                        pass

        # Print console status every second
        if time.time() - last_print >= 1.0:
            last_print = time.time()
            with telemetry_lock:
                t = telemetry.copy()

            gps_desc = f"{t['lat']:.5f}, {t['lng']:.5f} ({t['sats']} Sats)" if (t['lat'] != 0.0) else f"Searching ({t['sats']} Sats, {t['gps_bytes']} bytes rx)"
            imu_desc = "OK" if t['imu_ok'] else f"OFF [{t.get('imu_status_msg', 'NO I2C')}]"
            print(f"[SURDAS] Bridge: {'CONNECTED' if t['bridge_connected'] else 'CONNECTING...'} | "
                  f"IMU: {imu_desc} (P:{t['pitch']}°, R:{t['roll']}°) | "
                  f"GPS: {gps_desc}")

        time.sleep(0.15)

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
        return send_file(LOG_FILE, as_attachment=True, download_name="telemetry_log.csv")
    return "Log file empty", 404

if __name__ == '__main__':
    port = int(os.environ.get("PORT", 5000))
    print(f"\n==========================================================")
    print(f"📍 SURDAS GPS + IMU Live Dashboard: http://0.0.0.0:{port}")
    print(f"   Open in browser: http://unoq.local:{port}")
    print(f"==========================================================\n")
    app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False)
