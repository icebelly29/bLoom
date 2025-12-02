# Project Setup and Operation Guide

This guide provides a complete workflow for setting up and running the hand-holding detection project to control the robotic flower.

## 1. Prerequisites

- D-Robotics RDK X5 with ROS2 Humble and TogetherROS.Bot installed.
- A configured MIPI or USB camera.
- An Arduino Uno board.
- `arduino-cli` installed and functional.
- The `pyserial` library (`pip install pyserial`).

## 2. Arduino Setup

You have two sketches: `MCbloomMaster5.ino` and `MCbloomSlave5.ino`. You need to decide which functionality you want on the Arduino connected to your RDK. Typically, the **Master** sketch is what you'll upload to the Arduino that directly receives commands from the computer.

### Step 2.1: Upload the Sketch with `arduino-cli`

1.  **Identify your Arduino Port**: Connect your Arduino Uno. It should appear as `/dev/ttyUSB0`. You can confirm this by unplugging, running `ls /dev/tty*`, plugging it back in, and running the command again to see the new device.

2.  **Compile the Sketch**: Open a terminal in the project directory and run this command to compile the master sketch for your Arduino Uno.

    ```bash
    arduino-cli compile --fqbn arduino:avr:uno MCbloomMaster5.ino
    ```

3.  **Upload the Sketch**: Once compilation is successful, upload the sketch to your board.

    ```bash
    arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno MCbloomMaster5.ino
    ```

    *Note: If you intended to use the slave sketch instead, replace `MCbloomMaster5.ino` with `MCbloomSlave5.ino` in the commands above.*

## 3. Run the Hand Detection System

The system requires two separate processes running in two different terminals.

### Step 3.1: (Terminal 1) Launch Hand Landmark Detection

This terminal runs the underlying AI model that finds hands in the camera feed.

1.  Open a new terminal and source the TROS environment:
    ```bash
    source /opt/tros/humble/setup.bash
    ```

2.  Copy the necessary configuration files (if you haven't already):
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

### Step 3.2: (Terminal 2) Launch the Hand Holding Detector

This terminal runs the script that analyzes the hand data and sends the "open" command to your Arduino.

1.  Open a **second terminal** and source the TROS environment:
    ```bash
    source /opt/tros/humble/setup.bash
    ```

2.  Make the detector script executable:
    ```bash
    chmod +x hand_holding_detector.py
    ```

3.  Run the script, telling it to use your Arduino's serial port:
    ```bash
    ./hand_holding_detector.py --ros-args -p serial_port:='/dev/ttyUSB0'
    ```

## 4. Operation

With both terminals running and the Arduino connected and programmed:
1.  The first terminal will show output from the camera and the AI model.
2.  The second terminal will log `Hand Holding Detector is running...`.
3.  When you bring two hands close together in the camera's view, the second terminal will log `Hand holding DETECTED!` and send the `open` command to the Arduino.
4.  Your Arduino, connected to the flower mechanism, will receive the command and trigger the opening action.
