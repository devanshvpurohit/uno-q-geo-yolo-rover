#!/usr/bin/env bash
# Launch script for SURDAS GPS & IMU Platform on Arduino UNO Q
# Automatically validates and installs dependencies before running

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Ensure arduino-router daemon is running
if command -v systemctl &>/dev/null; then
    if ! systemctl is-active --quiet arduino-router 2>/dev/null; then
        echo "Starting arduino-router system service..."
        sudo systemctl restart arduino-router 2>/dev/null || true
    fi
fi

# Ensure .venv exists
if [ ! -d "$SCRIPT_DIR/.venv" ]; then
    echo "Creating virtual environment (.venv)..."
    python3 -m venv --system-site-packages "$SCRIPT_DIR/.venv"
fi

PYTHON_CMD="$SCRIPT_DIR/.venv/bin/python3"
PIP_CMD="$SCRIPT_DIR/.venv/bin/pip"

# Check if arduino-router-bridge and flask are installed
if ! "$PYTHON_CMD" -c "import flask; from arduino.router_bridge import Bridge" &>/dev/null; then
    echo "Installing arduino-router-bridge & flask in .venv..."
    "$PIP_CMD" install --upgrade pip
    "$PIP_CMD" install arduino-router-bridge flask
fi

echo "Starting SURDAS GPS + IMU Platform with $PYTHON_CMD..."
exec "$PYTHON_CMD" python/main.py
