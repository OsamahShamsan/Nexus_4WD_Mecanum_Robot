#!/bin/bash

# Settings
BOARD="arduino:avr:diecimila"
PORT="/dev/ttyUSB0"
LIB_PATH="/home/mecroka/Downloads/arduino-1.8.19/libraries"
SKETCH_DIR="/home/mecroka/bumperbot_ws/nexus_arduino/sketch_nexus"

cd "$SKETCH_DIR" || exit 1

echo "🔧 Compiling sketch..."
arduino-cli compile --fqbn $BOARD --libraries $LIB_PATH .

echo "🚀 Uploading to $PORT..."
arduino-cli upload -p $PORT --fqbn $BOARD .

echo "✅ Done!"
