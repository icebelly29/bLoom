# Hand Holding Detector for ROS2 on RDK X5

This ROS2 package provides a node that detects when two hands are being held together and sends a signal over a serial port. It is designed to work with the D-Robotics RDK platform and its `hand_lmk_detection` package.

## How It Works

The detector scripts initialize ROS2 nodes that:
1.  **Subscribe** to the `/hobot_hand_lmk_detection` topic to receive real-time hand keypoint data.
2.  **Analyze** the incoming data to check if at least two hands are present in the camera's view.
3.  **Calculate** distances and interaction scores between detected hands using different methods:
    - **Standard detector**: Uses wrist-to-wrist distance
    - **Handshake detector**: Uses palm centers and fingertip-to-palm contacts
4.  **Detect Holding**: When the detection criteria are met for a configurable number of consecutive frames, it registers a "holding" event.
5.  **Send Serial Signals**: 
    - Sends `"open"` (newline-terminated) when holding is detected
    - Waits a minimum duration (default 10 seconds)
    - Sends `"close"` only after the minimum duration AND hands are released
    - Includes cooldown period and pair tracking to prevent rapid state changes

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

There are two detector variants available:

#### Option A: Standard Detector (Wrist Distance Method)

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

#### Option B: Handshake Detector (Advanced Method)

The handshake detector uses a more sophisticated approach that checks for actual hand interaction by analyzing palm proximity and fingertip-to-palm contacts. This reduces false positives where hands are merely close but not actually holding.

1.  Open a **second terminal** and source the TROS environment:
    ```bash
    source /opt/tros/humble/setup.bash
    ```

2.  Make the detector script executable:
    ```bash
    chmod +x hand_holding_detector_handshake.py
    ```

3.  Run the handshake detector with recommended parameters:
    ```bash
    ./hand_holding_detector_handshake.py --ros-args \
      -p serial_port:='/dev/ttyUSB0' \
      -p palm_distance_threshold:=60.0 \
      -p fingertip_palm_threshold:=50.0 \
      -p min_fingertip_contacts:=3 \
      -p holding_frames_threshold:=3 \
      -p release_frames_threshold:=5 \
      -p min_open_duration_seconds:=10.0 \
      -p cooldown_seconds:=5.0 \
      -p strict_pair_threshold:=120.0
    ```

    **Parameters:**
    - **`serial_port`**: Serial port path (e.g., `/dev/ttyUSB0`)
    - **`palm_distance_threshold`**: Maximum distance between palm centers (pixels) to consider holding
    - **`fingertip_palm_threshold`**: Maximum distance for fingertip-to-opposite-palm contact (pixels)
    - **`min_fingertip_contacts`**: Minimum number of fingertip-to-palm contacts required (default: 3)
    - **`holding_frames_threshold`**: Consecutive frames required to confirm holding (default: 3)
    - **`release_frames_threshold`**: Consecutive frames required to confirm release (default: 5)
    - **`min_open_duration_seconds`**: Minimum time flower stays open after detection (default: 10.0)
    - **`cooldown_seconds`**: Delay after closing before new detection allowed (default: 5.0)
    - **`strict_pair_threshold`**: Maximum wrist distance to consider a valid pair (pixels, default: 120.0)

## Expected Output

When the system is running correctly:
- The first terminal will show the output from the camera and hand landmark detection.
- The second terminal will log messages like:
  ```
  [INFO] [hand_holding_detector]: Hand Holding Detector is running. Waiting for hand landmarks...
  [INFO] [hand_holding_detector]: Hand holding DETECTED! (Distance: 42.17px, Threshold: 30.0px). Sending 'open' to serial.
  [INFO] [hand_holding_detector]: Minimum open duration (10.0s) passed and hands released. Sending 'close'.
  ```
- Your serial device will receive the string `"open"` or `"close"` (newline-terminated) each time the state changes.
- The Arduino Master will print confirmation messages like:
  ```
  Master Ready. Type 'open', 'close', or 'speed <value>'
  Sent 'open 30' to all slaves.
  Sent 'close 200' to all slaves.
  ```

### Monitoring Serial Communication

To view the Arduino's serial output and verify commands are being received, you can use:

**For Arduino UNO (AVR):**
```bash
./bin/arduino-cli monitor -p /dev/ttyUSB0 -b arduino:avr:uno --config baudrate=9600
```

**For Arduino UNO R4 (Renesas):**
```bash
./bin/arduino-cli monitor -p /dev/ttyUSB0 -b arduino:renesas_uno:renesas_uno --config baudrate=9600
```

Alternatively, you can use:
- Arduino IDE Serial Monitor
- `screen /dev/ttyUSB0 9600`

**Note:** Make sure to close the serial monitor before running the hand holding detector, as the port can only be accessed by one process at a time.

## Detector Variants

### Standard Detector (`hand_holding_detector.py`)

Uses wrist-to-wrist distance for detection. Simple and fast, suitable for most use cases.

**Features:**
- Wrist distance-based detection
- Pair tracking to prevent switching between hand pairs
- Hysteresis (different thresholds for holding vs releasing)
- Minimum open duration (10 seconds default)
- Cooldown period after closing

### Handshake Detector (`hand_holding_detector_handshake.py`) ✅ **IMPLEMENTED**

Uses a more sophisticated approach that analyzes actual hand interaction by checking:
1. Palm center proximity (using wrist + finger base keypoints)
2. Fingertip-to-opposite-palm contacts
3. Interaction scoring based on multiple keypoint proximities

**Benefits:**
- Reduces false positives where hands are merely close but not actually holding
- More robust to hand overlap/occlusion
- Better detection of actual hand-holding gestures

**When to use:**
- Crowded environments with multiple hands
- Need for higher accuracy (fewer false positives)
- When hands may overlap significantly

## Ideas for Improvement

Here are additional ways to enhance the hand-holding detection:

### 1. The "Handshake" Method: Using More Keypoints ✅ **IMPLEMENTED**

See `hand_holding_detector_handshake.py` for the implementation. This method:
- Calculates palm centers using wrist and finger base keypoints (less occluded)
- Checks fingertip-to-opposite-palm proximity
- Uses interaction scoring for more robust detection

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