#!/usr/bin/env bash
# Flash the STM32 MCU directly on the Arduino UNO Q using arduino:zephyr:unoq

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_PATH="$SCRIPT_DIR/sketch"

echo "=== Flashing STM32 MCU on Arduino UNO Q (arduino:zephyr:unoq) ==="

# 1. Ensure arduino-cli is installed
if ! command -v arduino-cli &> /dev/null; then
    echo "Installing arduino-cli..."
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/local/bin sudo sh
fi

# 2. Update core index
arduino-cli core update-index

# 3. Ensure arduino:zephyr core is installed
echo "Verifying arduino:zephyr core..."
if ! arduino-cli core list | grep -q "arduino:zephyr"; then
    echo "Installing arduino:zephyr core..."
    arduino-cli core install arduino:zephyr
fi

# 4. Install TinyGPSPlus library
echo "Ensuring TinyGPSPlus library is installed..."
arduino-cli lib install "TinyGPSPlus" || true

# 5. Compile using official FQBN: arduino:zephyr:unoq
echo "Compiling sketch for Arduino UNO Q (arduino:zephyr:unoq)..."
arduino-cli compile -b arduino:zephyr:unoq "$SKETCH_PATH"

# 6. Upload to STM32 core
echo "Uploading to STM32 MCU..."
arduino-cli upload -b arduino:zephyr:unoq "$SKETCH_PATH"

echo "=== MCU Flashed Successfully with arduino:zephyr:unoq! ==="
