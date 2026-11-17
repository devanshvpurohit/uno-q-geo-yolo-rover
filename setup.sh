#!/usr/bin/env bash
# Automated Setup for Arduino UNO Q (4GB) Debian Linux environment
# Configures standalone execution without any external app or App Lab

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================================="
echo "  Setting up Standalone Geo-YOLO on Arduino UNO Q (4GB)   "
echo "=========================================================="

echo "[1/4] Updating Debian repositories..."
sudo apt-get update -y

echo "[2/4] Installing system dependencies (Python, OpenCV, V4L, Curl)..."
sudo apt-get install -y python3-pip python3-opencv v4l-utils git curl

echo "[3/4] Installing Python requirements (YOLO, Flask, PySerial)..."
pip3 install --upgrade pip
pip3 install -r python/requirements.txt

echo "[4/4] Pre-caching YOLOv8 Nano weights..."
if [ ! -f "yolov8n.pt" ]; then
    python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
fi

# Ensure scripts are executable
chmod +x setup.sh run.sh flash_mcu.sh

# Setup auto-start systemd service (optional)
if [ "$1" == "--enable-autostart" ]; then
    echo "Configuring systemd service for boot auto-start..."
    sudo cp uno-q-rover.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable uno-q-rover.service
    echo "Auto-start enabled! The rover will launch automatically on boot."
fi

echo "=========================================================="
echo "  Setup Completed Successfully!                           "
echo "                                                          "
echo "  To launch:                                              "
echo "    ./run.sh                                              "
echo "                                                          "
echo "  Web Dashboard will be live at:                          "
echo "    http://unoq.local:5000 (or http://<IP>:5000)         "
echo "=========================================================="
