# Setting Up Auto-Start on Boot

This guide explains how to set up the hand holding detector to start automatically on system boot.

## Prerequisites

- The hand holding detector script must be executable
- The startup script must be executable
- You must have sudo/root access to install systemd services

## Installation Steps

### Step 1: Verify Scripts are Executable

```bash
cd /home/sunrise/Desktop/bloom
chmod +x hand_holding_detector.py
chmod +x start_hand_holding_detector.sh
```

### Step 2: Test the Startup Script Manually

Before setting up auto-start, test that the script works:

```bash
./start_hand_holding_detector.sh
```

Press Ctrl+C to stop it. If it works correctly, proceed to the next step.

### Step 3: Install the Systemd Service

Copy the service file to the systemd directory:

```bash
sudo cp hand-holding-detector.service /etc/systemd/system/
```

### Step 4: Reload Systemd

Reload systemd to recognize the new service:

```bash
sudo systemctl daemon-reload
```

### Step 5: Enable the Service

Enable the service to start on boot:

```bash
sudo systemctl enable hand-holding-detector.service
```

### Step 6: Start the Service (Optional)

You can start the service immediately without rebooting:

```bash
sudo systemctl start hand-holding-detector.service
```

### Step 7: Check Service Status

Verify the service is running:

```bash
sudo systemctl status hand-holding-detector.service
```

## Managing the Service

### View Logs

```bash
# View recent logs
sudo journalctl -u hand-holding-detector.service -n 50

# Follow logs in real-time
sudo journalctl -u hand-holding-detector.service -f

# View logs since boot
sudo journalctl -u hand-holding-detector.service -b
```

### Stop the Service

```bash
sudo systemctl stop hand-holding-detector.service
```

### Restart the Service

```bash
sudo systemctl restart hand-holding-detector.service
```

### Disable Auto-Start

If you want to disable auto-start but keep the service installed:

```bash
sudo systemctl disable hand-holding-detector.service
```

### Remove the Service

To completely remove the service:

```bash
sudo systemctl stop hand-holding-detector.service
sudo systemctl disable hand-holding-detector.service
sudo rm /etc/systemd/system/hand-holding-detector.service
sudo systemctl daemon-reload
```

## Customization

### Adjust Parameters

Edit `/home/sunrise/Desktop/bloom/start_hand_holding_detector.sh` to change:
- Serial port path
- Threshold values
- Other ROS parameters

After making changes, restart the service:

```bash
sudo systemctl restart hand-holding-detector.service
```

### Change Serial Port

If your serial port is different (e.g., `/dev/ttyACM0`), edit the startup script:

```bash
nano /home/sunrise/Desktop/bloom/start_hand_holding_detector.sh
```

Change the `serial_port` parameter, then restart:

```bash
sudo systemctl restart hand-holding-detector.service
```

### Delay Startup

If you need to delay startup (e.g., wait for USB devices), edit the service file:

```bash
sudo nano /etc/systemd/system/hand-holding-detector.service
```

Add a delay in the `[Service]` section:

```ini
ExecStartPre=/bin/sleep 10
```

Then reload and restart:

```bash
sudo systemctl daemon-reload
sudo systemctl restart hand-holding-detector.service
```

## Troubleshooting

### Service Fails to Start

1. Check the service status:
   ```bash
   sudo systemctl status hand-holding-detector.service
   ```

2. Check the logs:
   ```bash
   sudo journalctl -u hand-holding-detector.service -n 100
   ```

3. Verify the script works manually:
   ```bash
   ./start_hand_holding_detector.sh
   ```

### Serial Port Not Found

If the service starts before the USB device is ready:

1. Add a delay (see "Delay Startup" above)
2. Or create a udev rule to ensure the device is available
3. Or modify the startup script to wait for the device:

```bash
# Add to start_hand_holding_detector.sh before running the detector
while [ ! -e /dev/ttyUSB0 ]; do
    echo "Waiting for /dev/ttyUSB0..."
    sleep 1
done
```

### ROS2/TROS Not Found

If you get errors about ROS2/TROS not being found:

1. Verify TROS is installed at `/opt/tros/humble/setup.bash`
2. Check that the startup script sources it correctly
3. Verify the user has permission to access TROS

### Permission Issues

If you get permission errors:

1. Ensure the user is in the `dialout` group (for serial port access):
   ```bash
   sudo adduser sunrise dialout
   ```
   (Log out and back in for this to take effect)

2. Check file permissions:
   ```bash
   ls -l /home/sunrise/Desktop/bloom/hand_holding_detector.py
   ls -l /home/sunrise/Desktop/bloom/start_hand_holding_detector.sh
   ```

## Notes

- The service will automatically restart if it crashes (RestartSec=10)
- Logs are available via `journalctl`
- The service runs as user `sunrise` (change in the service file if needed)
- Make sure `hand_lmk_detection` is also set to auto-start if needed


