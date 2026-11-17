#!/usr/bin/env bash
# Flash the STM32 MCU directly on the Arduino UNO Q without any IDE or App Lab

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_PATH="$SCRIPT_DIR/sketch"

echo "=== Flashing STM32 MCU on Arduino UNO Q ==="

# Check if arduino-cli is installed
if ! command -v arduino-cli &> /dev/null; then
    echo "Installing arduino-cli..."
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/local/bin sudo sh
fi

# Update core index
arduino-cli core update-index

# Install arduino:zephyr core if not installed
echo "Verifying arduino:zephyr core..."
if ! arduino-cli core list | grep -q "arduino:zephyr"; then
    echo "Installing arduino:zephyr core..."
    arduino-cli core install arduino:zephyr
fi

# Compile using the official FQBN for UNO Q: arduino:zephyr:unoq
echo "Compiling sketch for Arduino UNO Q (arduino:zephyr:unoq)..."
arduino-cli compile -b arduino:zephyr:unoq "$SKETCH_PATH"

# Upload to STM32 core via internal remoteocd bus
echo "Uploading to STM32 core..."
arduino-cli upload -b arduino:zephyr:unoq "$SKETCH_PATH"

echo "=== MCU Flashed Successfully! ==="
