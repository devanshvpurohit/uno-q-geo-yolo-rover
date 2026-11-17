#!/usr/bin/env bash
# Flash the STM32 MCU directly on the Arduino UNO Q without any IDE or App Lab

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_PATH="$SCRIPT_DIR/sketch"

echo "=== Flashing STM32 MCU on Arduino UNO Q ==="

# Check if arduino-cli is installed
if ! command -v arduino-cli &> /dev/null; then
    echo "Installing arduino-cli..."
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/local/bin sh
fi

# Update core index
arduino-cli core update-index

# Compile and upload
echo "Compiling sketch..."
arduino-cli compile --fqbn arduino:zephyr:unoq_mcu "$SKETCH_PATH"

echo "Uploading to STM32 core..."
# Detect serial port for MCU flashing
PORT=$(arduino-cli board list | grep -i "unoq" | awk '{print $1}' || true)
if [ -z "$PORT" ]; then
    PORT="/dev/ttyACM0"
fi

arduino-cli upload -p "$PORT" --fqbn arduino:zephyr:unoq_mcu "$SKETCH_PATH"

echo "=== MCU Flashed Successfully! ==="
