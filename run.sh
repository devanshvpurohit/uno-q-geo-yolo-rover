#!/usr/bin/env bash
# Launch script for UNO Q Geo-YOLO Rover
# Automatically detects virtualenv or system python

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ -f "$SCRIPT_DIR/.venv/bin/python3" ]; then
    PYTHON_CMD="$SCRIPT_DIR/.venv/bin/python3"
else
    PYTHON_CMD="python3"
fi

echo "Starting UNO Q Geo-YOLO Vision Rover with $PYTHON_CMD..."
exec "$PYTHON_CMD" python/main.py
