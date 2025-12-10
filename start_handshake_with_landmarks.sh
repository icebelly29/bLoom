#!/bin/bash
# Launch hand landmark detection, then the handshake detector once landmarks are available.
# This lets you start everything from one terminal or systemd unit.

set -euo pipefail

# --- Config (override via env vars) ---
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
POLL_SECONDS="${POLL_SECONDS:-2}"
MAX_WAIT_SECONDS="${MAX_WAIT_SECONDS:-30}"

cleanup() {
    if [[ -n "${HAND_LMK_PID:-}" ]]; then
        kill "${HAND_LMK_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# Source TROS environment
if [[ -f /opt/tros/humble/setup.bash ]]; then
    source /opt/tros/humble/setup.bash
else
    echo "ERROR: TROS environment not found at /opt/tros/humble/setup.bash" >&2
    exit 1
fi

cd /home/sunrise/Desktop/bloom || {
    echo "ERROR: Cannot change to /home/sunrise/Desktop/bloom" >&2
    exit 1
}

echo "[INFO] Starting hand_lmk_detection.launch.py in background..."
ros2 launch hand_lmk_detection hand_lmk_detection.launch.py >/tmp/hand_lmk_detection.log 2>&1 &
HAND_LMK_PID=$!

echo "[INFO] Waiting for /hobot_hand_lmk_detection topic (timeout: ${MAX_WAIT_SECONDS}s)..."
elapsed=0
while ! ros2 topic list 2>/dev/null | grep -q "/hobot_hand_lmk_detection"; do
    sleep "${POLL_SECONDS}"
    elapsed=$((elapsed + POLL_SECONDS))
    if (( elapsed >= MAX_WAIT_SECONDS )); then
        echo "ERROR: Topic /hobot_hand_lmk_detection not found after ${MAX_WAIT_SECONDS}s. Check /tmp/hand_lmk_detection.log" >&2
        exit 1
    fi
done
echo "[INFO] Topic detected. Starting handshake detector..."

exec ./hand_holding_detector_handshake.py --ros-args \
  -p serial_port:="${SERIAL_PORT}" \
  -p palm_distance_threshold:=60.0 \
  -p fingertip_palm_threshold:=50.0 \
  -p min_fingertip_contacts:=3 \
  -p holding_frames_threshold:=3 \
  -p release_frames_threshold:=5 \
  -p min_open_duration_seconds:=10.0 \
  -p cooldown_seconds:=5.0 \
  -p strict_pair_threshold:=120.0

