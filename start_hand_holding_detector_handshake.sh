#!/bin/bash
# Startup script for the handshake-based hand holding detector
# Sources the TROS environment and launches the handshake variant with recommended parameters

set -e  # Exit on error

# Source TROS environment
if [ -f /opt/tros/humble/setup.bash ]; then
    source /opt/tros/humble/setup.bash
else
    echo "ERROR: TROS environment not found at /opt/tros/humble/setup.bash" >&2
    exit 1
fi

# Change to the project directory
cd /home/sunrise/Desktop/bloom || {
    echo "ERROR: Cannot change to /home/sunrise/Desktop/bloom" >&2
    exit 1
}

# Run the handshake detector with recommended defaults
exec ./hand_holding_detector_handshake.py --ros-args \
  -p serial_port:='/dev/ttyUSB0' \
  -p palm_distance_threshold:=60.0 \
  -p fingertip_palm_threshold:=50.0 \
  -p min_fingertip_contacts:=3 \
  -p holding_frames_threshold:=3 \
  -p release_frames_threshold:=5 \
  -p min_open_duration_seconds:=10.0 \
  -p cooldown_seconds:=5.0 \
  -p strict_pair_threshold:=120.0

