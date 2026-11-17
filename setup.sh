#!/usr/bin/env bash
# Fast Setup for Arduino UNO Q (4GB) - GPS + IMU Platform
# Clean, lightweight, and completes in seconds

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================================="
echo "  Setting up SURDAS GPS + IMU Platform on Arduino UNO Q   "
echo "=========================================================="

echo "[1/3] Updating apt and installing core tools..."
sudo apt-get update -y
sudo apt-get install -y \
    python3-pip \
    python3-flask \
    python3-venv \
    curl \
    git

echo "[2/3] Configuring Python environment..."
if [ ! -d ".venv" ]; then
    echo "Creating virtual environment (.venv)..."
    python3 -m venv --system-site-packages .venv
fi

echo "Installing requirements (Flask, Arduino Router Bridge)..."
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r python/requirements.txt

# Ensure executable scripts
chmod +x setup.sh run.sh flash_mcu.sh

# Optional: setup systemd auto-start
if [ "$1" == "--enable-autostart" ]; then
    echo "Configuring systemd service for boot auto-start..."
    sudo cp uno-q-rover.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable uno-q-rover.service
    echo "Auto-start enabled! Will launch automatically on boot."
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
