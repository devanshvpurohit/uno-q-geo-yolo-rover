#!/usr/bin/env bash
# Setup script for Arduino UNO Q (4GB) Debian Linux environment

set -e

echo "=========================================================="
echo "  Setting up Autonomous Geo-YOLO on Arduino UNO Q (4GB)  "
echo "=========================================================="

echo "[1/4] Updating Debian repositories..."
sudo apt-get update -y

echo "[2/4] Installing system dependencies (Python, OpenCV, V4L)..."
sudo apt-get install -y python3-pip python3-opencv v4l-utils git

echo "[3/4] Installing Python requirements..."
pip3 install --upgrade pip
pip3 install -r python/requirements.txt

echo "[4/4] Pre-caching YOLOv8 Nano weights..."
if [ ! -f "yolov8n.pt" ]; then
    python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
fi

echo "=========================================================="
echo "  Setup Completed Successfully!                           "
echo "  To launch: python3 python/main.py                       "
echo "=========================================================="
