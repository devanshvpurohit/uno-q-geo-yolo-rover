#!/usr/bin/env bash
# Automated Setup for Arduino UNO Q (4GB) Debian Linux environment
# Handles Debian 13 PEP 668 externally-managed-environment automatically

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================================="
echo "  Setting up Standalone Geo-YOLO on Arduino UNO Q (4GB)   "
echo "=========================================================="

echo "[1/4] Updating Debian repositories..."
sudo apt-get update -y

echo "[2/4] Installing system dependencies (Python, OpenCV, Serial, Flask, V4L)..."
sudo apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-serial \
    python3-flask \
    python3-numpy \
    python3-venv \
    v4l-utils \
    git \
    curl

echo "[3/4] Configuring Python environment for Debian 13..."
# Create a virtual environment that shares system site-packages (OpenCV/Serial/Flask)
if [ ! -d ".venv" ]; then
    echo "Creating virtual environment (.venv) with system-site-packages..."
    python3 -m venv --system-site-packages .venv
fi

# Upgrade pip inside venv and install YOLO requirements
echo "Installing pip requirements in .venv..."
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r python/requirements.txt

# Also attempt system-wide pip install with --break-system-packages for direct terminal usage
pip3 install --break-system-packages -r python/requirements.txt 2>/dev/null || true

PYTHON_BIN="$SCRIPT_DIR/.venv/bin/python3"
if [ ! -f "$PYTHON_BIN" ]; then
    PYTHON_BIN="python3"
fi

echo "[4/4] Pre-caching YOLOv8 Nano weights..."
if [ ! -f "yolov8n.pt" ]; then
    $PYTHON_BIN -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
fi

# Ensure all scripts are executable
chmod +x setup.sh run.sh flash_mcu.sh

# Optional: setup systemd auto-start
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
echo "  To launch the rover now, run:                           "
echo "    ./run.sh                                              "
echo "                                                          "
echo "  Web Dashboard will be live at:                          "
echo "    http://unoq.local:5000 (or http://<IP>:5000)         "
echo "=========================================================="
