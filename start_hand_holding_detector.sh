#!/bin/bash
# Startup script for hand holding detector
# This script sources the TROS environment and starts the detector

set -e  # Exit on error

# Source TROS environment
if [ -f /opt/tros/humble/setup.bash ]; then
    source /opt/tros/humble/setup.bash
else
    echo "ERROR: TROS environment not found at /opt/tros/humble/setup.bash" >&2
    exit 1
fi

# Change to the script directory
cd /home/sunrise/Desktop/bloom || {
    echo "ERROR: Cannot change to /home/sunrise/Desktop/bloom" >&2
    exit 1
}

# Wait for serial port to be available (optional, uncomment if needed)
# SERIAL_PORT="/dev/ttyUSB0"
# MAX_WAIT=30
# WAITED=0
# while [ ! -e "$SERIAL_PORT" ] && [ $WAITED -lt $MAX_WAIT ]; do
#     echo "Waiting for $SERIAL_PORT to be available..."
#     sleep 1
#     WAITED=$((WAITED + 1))
# done
# if [ ! -e "$SERIAL_PORT" ]; then
#     echo "WARNING: $SERIAL_PORT not found after $MAX_WAIT seconds" >&2
# fi

# Run the hand holding detector
# Adjust parameters as needed
exec ./hand_holding_detector.py --ros-args \
  -p serial_port:='/dev/ttyUSB0' \
  -p holding_threshold_pixels:=30.0 \
  -p strict_pair_threshold:=30.0

