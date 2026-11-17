#!/usr/bin/env bash
# Launch script for UNO Q Geo-YOLO Rover (Completely standalone, no app needed)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Activate python virtualenv if exists
if [ -d "$SCRIPT_DIR/.venv" ]; then
    source "$SCRIPT_DIR/.venv/bin/activate"
fi

echo "Starting UNO Q Geo-YOLO Vision Rover..."
python3 python/main.py
