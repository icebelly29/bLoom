# Hand Holding Detector for ROS2 on RDK X5

This ROS2 package provides a node that detects when two hands are being held together and sends a signal over a serial port. It is designed to work with the D-Robotics RDK platform and its `hand_lmk_detection` package.

## How It Works

The `hand_holding_detector.py` script initializes a ROS2 node that:
1.  **Subscribes** to the `/hobot_hand_lmk_detection` topic to receive real-time hand keypoint data.
2.  **Analyzes** the incoming data to check if at least two hands are present in the camera's view.
3.  **Calculates** the Euclidean distance between the wrists (keypoint 0) of the first two detected hands.
4.  **Detects Holding**: If the distance between the wrists falls below a configurable threshold, it registers a "holding" event.
5.  **Sends a Serial Signal**: Upon the initial detection of a "holding" state, it sends a single `1` character over the specified serial port. It is stateful and will not send another signal until the hands are released and the holding gesture is initiated again.

## Requirements

- D-Robotics RDK X5 (or similar)
- Ubuntu 22.04 with ROS2 Humble and TogetherROS.Bot (tros) installed.
- A MIPI or USB camera configured and working.
- A device connected via a serial port (e.g., Arduino, Raspberry Pi Pico, FTDI adapter) to receive the signal.
- The `pyserial` library (`pip install pyserial`).

## Setup and Usage

Follow these steps to set up and run the hand holding detector.

### Step 1: Configure Serial Port Permissions

You need to grant your user account permission to access the serial hardware.

1.  **Find your serial port**: Identify the name of your serial port by running `ls /dev/tty*`. Common names include `/dev/ttyUSB0`, `/dev/ttyACM0`, or `/dev/ttyS0`.

2.  **Add user to `dialout` group**: This is the most common way to grant access.
    ```bash
    sudo adduser $USER dialout
    ```

3.  **Apply permissions**: For the group change to take effect, you must **log out and log back in**.

### Step 2: Run the Hand Landmark Detection

This node depends on the keypoint data from the `hand_lmk_detection` package. You must run it in a separate terminal.

1.  Open a new terminal and source the TROS environment:
    ```bash
    source /opt/tros/humble/setup.bash
    ```

2.  Copy the necessary configuration files to your working directory (if you haven't already):
    ```bash
    cp -r /opt/tros/humble/lib/hand_lmk_detection/config/ .
    ```

3.  Set your camera type and launch the node:
    ```bash
    # For a MIPI camera
    export CAM_TYPE=mipi
    
    # For a USB camera
    # export CAM_TYPE=usb
    
    ros2 launch hand_lmk_detection hand_lmk_detection.launch.py
    ```
    Leave this terminal running.

### Step 3: Run the Hand Holding Detector

1.  Open a **second terminal** and source the TROS environment:
    ```bash
    source /opt/tros/humble/setup.bash
    ```

2.  Make the detector script executable:
    ```bash
    chmod +x hand_holding_detector.py
    ```

3.  Run the script. You can configure the serial port and detection sensitivity using ROS arguments:
    ```bash
    ./hand_holding_detector.py --ros-args -p serial_port:='/dev/ttyUSB0' -p holding_threshold_pixels:=50.0
    ```
    - **`serial_port`**: Replace `/dev/ttyUSB0` with the actual path to your serial device.
    - **`holding_threshold_pixels`**: This is the maximum distance (in pixels) between wrists to be considered "holding". Decrease this value for stricter detection or increase it for more lenient detection.

## Expected Output

When the system is running correctly:
- The first terminal will show the output from the camera and hand landmark detection.
- The second terminal will log messages like:
  ```
  [INFO] [hand_holding_detector]: Hand Holding Detector is running. Waiting for hand landmarks...
  [INFO] [hand_holding_detector]: Hand holding DETECTED! (Distance: 42.17px). Sending '1' to serial.
  [INFO] [hand_holding_detector]: Hands released. (Distance: 85.91px)
  ```
- Your serial device will receive the character `1` each time the holding gesture begins.

### Monitoring Serial Communication

To view the Arduino's serial output and verify commands are being received, you can use:

```bash
./bin/arduino-cli monitor -p /dev/ttyUSB0 -b arduino:avr:uno --config baudrate=9600
```

Alternatively, you can use:
- Arduino IDE Serial Monitor
- `screen /dev/ttyUSB0 9600`

## Ideas for Improvement

Here are some ways to enhance the hand-holding detection for better accuracy and robustness:

### 1. The "Handshake" Method: Using More Keypoints

Instead of relying solely on the distance between wrists, leverage more hand keypoints to detect actual hand interaction.

*   **Concept:**
    1.  Calculate the approximate center of the palm for each detected hand.
    2.  Check for proximity between the palm centers.
    3.  Crucially, check if fingertip keypoints from one hand are close to the palm or finger keypoints of the other hand, indicating interlaced fingers or a firm grip.
*   **Benefits:** Reduces false positives where hands are merely close but not holding.

### 2. The "Context is King" Method: Associating Hands with People

Distinguish between a single person holding their own hands and two different people holding hands.

*   **Concept:**
    1.  Utilize a multi-task model (like the one hinted at by `multitask_body_head_face_hand_kps_960x544.hbm` in your `config` folder) that detects both people and their associated hand landmarks.
    2.  Group hands by the person they belong to.
    3.  Only trigger a "holding" event if the detected hands belong to *different* individuals.
*   **Benefits:** Eliminates false positives from a single person's self-interaction (e.g., clapping, wringing hands).

### 3. The "Patience" Method: Temporal Filtering (Debouncing)

Improve the stability of detection by preventing rapid flickering due to momentary sensor noise or model glitches.

*   **Concept:**
    1.  **Require a "Hold" Streak:** Instead of immediately declaring "holding" when the condition is met, require the holding criteria to be true for a certain number of consecutive frames (e.g., 5 frames).
    2.  **Require a "Release" Streak:** Similarly, only declare "hands released" after the non-holding condition has been true for several consecutive frames.
*   **Benefits:** Provides a more stable and natural user experience by smoothing out transient detection errors.

## Documentation

For detailed documentation, see the `docs/` folder:

- **[DEVELOPMENT_GUIDE.md](docs/DEVELOPMENT_GUIDE.md)** - Comprehensive development guide for beginners
- **[CURRENT_STATUS.md](docs/CURRENT_STATUS.md)** - Current project status and known issues
- **[STABILITY_IMPROVEMENTS.md](docs/STABILITY_IMPROVEMENTS.md)** - Recent stability improvements (pair tracking, cooldown, hysteresis)
- **[SETUP_AUTOSTART.md](docs/SETUP_AUTOSTART.md)** - Auto-start setup instructions for edge devices
- **[HAND_DETECTION_COMPARISON.md](docs/HAND_DETECTION_COMPARISON.md)** - Comparison of hand detection methods
- **[ISSUES_FOUND.md](docs/ISSUES_FOUND.md)** - Log of issues found and fixed during development
- **[PUSH_TO_GITHUB.md](docs/PUSH_TO_GITHUB.md)** - GitHub setup and push instructions