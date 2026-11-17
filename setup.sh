#!/usr/bin/env bash
# Clean, reliable setup for Arduino UNO Q (4GB) - GPS + IMU Platform

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

echo "[2/3] Configuring Python virtual environment (.venv)..."
if [ ! -d ".venv" ]; then
    python3 -m venv --system-site-packages .venv
fi

echo "[3/3] Installing Python dependencies (Flask, Arduino Router Bridge)..."
.venv/bin/pip install --upgrade pip
.venv/bin/pip install arduino-router-bridge flask

# Ensure scripts have execute permissions
chmod +x setup.sh run.sh flash_mcu.sh

# Restart arduino-router service to ensure clean socket
if command -v systemctl &>/dev/null; then
    sudo systemctl restart arduino-router 2>/dev/null || true
fi

echo "=========================================================="
echo "  Setup Completed Successfully!                           "
echo "  1. Flash MCU:  ./flash_mcu.sh                           "
echo "  2. Start:      ./run.sh                                 "
echo "=========================================================="
