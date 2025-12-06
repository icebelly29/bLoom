#!/bin/bash
# Quick installation script for hand holding detector systemd service

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="hand-holding-detector.service"
SERVICE_FILE="$SCRIPT_DIR/$SERVICE_NAME"
SYSTEMD_PATH="/etc/systemd/system/$SERVICE_NAME"

echo "Installing hand holding detector systemd service..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "ERROR: This script must be run as root (use sudo)"
    exit 1
fi

# Verify files exist
if [ ! -f "$SERVICE_FILE" ]; then
    echo "ERROR: Service file not found: $SERVICE_FILE"
    exit 1
fi

if [ ! -f "$SCRIPT_DIR/start_hand_holding_detector.sh" ]; then
    echo "ERROR: Startup script not found: $SCRIPT_DIR/start_hand_holding_detector.sh"
    exit 1
fi

# Make scripts executable
chmod +x "$SCRIPT_DIR/hand_holding_detector.py"
chmod +x "$SCRIPT_DIR/start_hand_holding_detector.sh"

# Copy service file
echo "Copying service file to $SYSTEMD_PATH..."
cp "$SERVICE_FILE" "$SYSTEMD_PATH"

# Reload systemd
echo "Reloading systemd daemon..."
systemctl daemon-reload

# Enable service
echo "Enabling service to start on boot..."
systemctl enable "$SERVICE_NAME"

echo ""
echo "Installation complete!"
echo ""
echo "The service is now enabled and will start on boot."
echo ""
echo "To start the service now (without rebooting):"
echo "  sudo systemctl start $SERVICE_NAME"
echo ""
echo "To check service status:"
echo "  sudo systemctl status $SERVICE_NAME"
echo ""
echo "To view logs:"
echo "  sudo journalctl -u $SERVICE_NAME -f"
echo ""
echo "To stop the service:"
echo "  sudo systemctl stop $SERVICE_NAME"
echo ""
echo "To disable auto-start:"
echo "  sudo systemctl disable $SERVICE_NAME"


