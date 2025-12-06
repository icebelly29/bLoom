# Hand Holding Detector - Comprehensive Development Guide

## Table of Contents
1. [Project Overview](#project-overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Software Setup](#software-setup)
4. [Project Architecture](#project-architecture)
5. [Code Walkthrough](#code-walkthrough)
6. [Running the Project](#running-the-project)
7. [Configuration](#configuration)
8. [Troubleshooting](#troubleshooting)
9. [Development Notes](#development-notes)
10. [Future Improvements](#future-improvements)

---

## Project Overview

This project detects when two hands are being held together using computer vision and sends commands to control a robotic flower mechanism via serial communication. It's designed for edge devices (D-Robotics RDK X5) and works in crowded environments.

### How It Works

1. **Camera captures video** → Hand landmark detection model processes frames
2. **Hand keypoints extracted** → Wrists, palms, and finger positions identified
3. **Distance calculation** → Finds closest pair of hands among all detected hands
4. **Holding detection** → Checks if hands are close enough (within threshold)
5. **Serial command** → Sends "open" to Arduino, waits 10 seconds, then sends "close"

---

## Hardware Requirements

### Essential Components

1. **D-Robotics RDK X5** (or compatible edge device)
   - Ubuntu 22.04
   - ROS2 Humble
   - TogetherROS.Bot (TROS) installed

2. **Camera**
   - MIPI camera (built-in) OR
   - USB camera

3. **Arduino Uno** (or compatible)
   - Connected via USB to RDK
   - Running Master sketch (`MCbloomMaster5.ino`)

4. **Serial Connection**
   - USB cable connecting Arduino to RDK
   - Typically appears as `/dev/ttyUSB0` or `/dev/ttyACM0`

### Optional Components

- **Arduino Slaves** (up to 5) - For controlling multiple flower mechanisms
- **Stepper Motors** - For flower opening/closing mechanism
- **NeoPixel LEDs** - For visual feedback

---

## Software Setup

### Step 1: Install Prerequisites

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Python dependencies
pip3 install pyserial

# Verify TROS installation
ls /opt/tros/humble/setup.bash
```

### Step 2: Set Up Serial Port Permissions

```bash
# Add user to dialout group (for serial port access)
sudo adduser $USER dialout

# Log out and log back in for changes to take effect
# Or run: newgrp dialout
```

### Step 3: Verify Serial Port

```bash
# List available serial ports
ls -la /dev/ttyUSB* /dev/ttyACM*

# Your Arduino should appear here when connected
# Common names: /dev/ttyUSB0, /dev/ttyACM0
```

### Step 4: Clone/Download Project

```bash
# Navigate to your workspace
cd ~/Desktop

# If using git:
git clone <repository-url> bloom
cd bloom

# Or extract from archive
# Make sure all files are in the bloom directory
```

### Step 5: Make Scripts Executable

```bash
cd ~/Desktop/bloom
chmod +x hand_holding_detector.py
chmod +x start_hand_holding_detector.sh
```

### Step 6: Copy Configuration Files

```bash
# Copy hand detection model config files
cp -r /opt/tros/humble/lib/hand_lmk_detection/config/ .
```

---

## Project Architecture

### System Components

```
┌─────────────────┐
│   Camera        │
│  (MIPI/USB)     │
└────────┬────────┘
         │
         ▼
┌─────────────────────────┐
│  hand_lmk_detection     │
│  (ROS2 Node)            │
│  - Detects hands         │
│  - Extracts keypoints    │
└────────┬────────────────┘
         │
         │ ROS2 Topic: /hobot_hand_lmk_detection
         │ Message Type: ai_msgs/msg/PerceptionTargets
         ▼
┌─────────────────────────┐
│  hand_holding_detector  │
│  (ROS2 Node)            │
│  - Analyzes keypoints    │
│  - Detects holding       │
│  - Sends serial commands │
└────────┬────────────────┘
         │
         │ Serial: /dev/ttyUSB0 (9600 baud)
         ▼
┌─────────────────────────┐
│  Arduino Master         │
│  (MCbloomMaster5.ino)   │
│  - Receives commands     │
│  - Distributes via I2C   │
└────────┬────────────────┘
         │
         │ I2C Bus
         ▼
┌─────────────────────────┐
│  Arduino Slaves         │
│  (MCbloomSlave5.ino)    │
│  - Control stepper motors│
│  - Manage LEDs           │
└─────────────────────────┘
```

### Message Flow

1. **PerceptionTargets Message Structure:**
   ```
   PerceptionTargets
   ├── header (timestamp, frame_id)
   ├── fps (frame rate)
   ├── perfs (performance metrics)
   └── targets[] (array of detected objects)
       └── Target
           ├── type: "person"
           ├── track_id: unique ID
           ├── rois[]: bounding boxes (body, head, face, hand)
           └── points[]: keypoints
               └── Point
                   ├── type: "hand_kps" (or "body_kps", "face_kps")
                   └── point[]: array of Point32 (x, y, z coordinates)
   ```

2. **Hand Keypoint Structure:**
   - Each `hand_kps` contains 21 keypoints:
     - Index 0: Wrist
     - Indices 1-4: Thumb
     - Indices 5-8: Index finger
     - Indices 9-12: Middle finger
     - Indices 13-16: Ring finger
     - Indices 17-20: Pinky finger

---

## Code Walkthrough

### Main Components

#### 1. `HandHoldingDetector` Class

**Initialization (`__init__`):**
- Declares ROS2 parameters (thresholds, serial port, etc.)
- Opens serial port connection
- Creates subscription to `/hobot_hand_lmk_detection` topic
- Initializes state variables

**Key Methods:**

##### `get_all_hand_keypoints_from_person(target)`
- Extracts all hand keypoints from a person target
- Looks for `point_type == 'hand_kps'` in the points array
- Returns list of hand keypoint arrays (one per hand)

##### `get_hand_palm_center(hand_keypoints)`
- Calculates palm center from less-occluded keypoints
- Uses wrist (index 0) and finger bases (indices 1, 5, 9, 13, 17)
- More robust when hands overlap

##### `calculate_hand_interaction_score(hand1, hand2)`
- Calculates how likely two hands are holding
- Uses multiple keypoints for robustness
- Returns score 0-1 (1 = definitely holding)

##### `hand_lmk_callback(msg)`
- Main callback function called for each frame
- Extracts hands from all person targets
- Finds closest pair of hands
- Detects holding and sends commands

##### `send_serial_signal(signal)`
- Sends "open" or "close" to Arduino
- Prevents duplicate commands
- Handles errors gracefully

### Detection Algorithm

1. **Extract Hands:**
   ```python
   for each person in targets:
       for each hand_kps in person.points:
           extract keypoints
           calculate palm center
           add to valid_hands list
   ```

2. **Find Closest Pair:**
   ```python
   for each pair of hands:
       calculate wrist distance
       calculate palm distance
       calculate interaction score
       if within threshold:
           track as potential pair
   select best pair (highest score, closest distance)
   ```

3. **Detect Holding:**
   ```python
   if distance < threshold:
       increment holding_frames_count
       if count >= threshold and not already holding:
           send "open"
           start 10-second timer
   else:
       reset counters
   ```

4. **Auto-Close:**
   ```python
   if 10 seconds have passed since "open":
       send "close"
       reset all state
   ```

### State Machine

```
IDLE
  │
  │ (hands detected holding)
  ▼
DETECTING (counting frames)
  │
  │ (enough frames)
  ▼
OPENING (sending "open")
  │
  │ (start 10-second timer)
  ▼
OPEN_CYCLE (waiting 10 seconds)
  │
  │ (10 seconds elapsed)
  ▼
CLOSING (sending "close")
  │
  │ (reset state)
  ▼
IDLE
```

---

## Running the Project

### Manual Run (Testing)

#### Terminal 1: Start Hand Landmark Detection

```bash
# Source TROS environment
source /opt/tros/humble/setup.bash

# Set camera type
export CAM_TYPE=mipi  # or 'usb' for USB camera

# Copy config files (first time only)
cp -r /opt/tros/humble/lib/hand_lmk_detection/config/ .

# Launch detection node
ros2 launch hand_lmk_detection hand_lmk_detection.launch.py
```

#### Terminal 2: Start Hand Holding Detector

```bash
# Source TROS environment
source /opt/tros/humble/setup.bash

# Navigate to project
cd ~/Desktop/bloom

# Run detector
./hand_holding_detector.py --ros-args \
  -p serial_port:='/dev/ttyUSB0' \
  -p holding_threshold_pixels:=30.0 \
  -p strict_pair_threshold:=30.0 \
  -p min_open_duration_seconds:=10.0
```

### Auto-Start on Boot

See `SETUP_AUTOSTART.md` for detailed instructions.

Quick setup:
```bash
cd ~/Desktop/bloom
sudo ./install_autostart.sh
```

---

## Configuration

### ROS2 Parameters

All parameters can be set via command line:

```bash
./hand_holding_detector.py --ros-args \
  -p serial_port:='/dev/ttyUSB0' \
  -p baud_rate:=9600 \
  -p holding_threshold_pixels:=30.0 \
  -p strict_pair_threshold:=30.0 \
  -p holding_frames_threshold:=3 \
  -p release_frames_threshold:=5 \
  -p ema_alpha:=0.2 \
  -p min_open_duration_seconds:=10.0
```

### Parameter Descriptions

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Serial port path for Arduino |
| `baud_rate` | `9600` | Serial communication speed |
| `holding_threshold_pixels` | `30.0` | Max distance (px) to consider "holding" |
| `strict_pair_threshold` | `30.0` | Max distance to consider a hand pair |
| `holding_frames_threshold` | `3` | Frames required to trigger "holding" |
| `release_frames_threshold` | `5` | Frames required to trigger "release" |
| `ema_alpha` | `0.2` | Smoothing factor (0-1, lower = more smoothing) |
| `min_open_duration_seconds` | `10.0` | Minimum time to stay open before auto-close |

### Tuning Tips

**Too many false positives (detects holding when not):**
- Decrease `holding_threshold_pixels` (e.g., 25.0)
- Decrease `strict_pair_threshold` (e.g., 25.0)
- Increase `holding_frames_threshold` (e.g., 5)

**Too many false negatives (misses actual holding):**
- Increase `holding_threshold_pixels` (e.g., 35.0)
- Increase `strict_pair_threshold` (e.g., 35.0)
- Decrease `holding_frames_threshold` (e.g., 2)

**Too jittery (rapid state changes):**
- Increase `ema_alpha` (e.g., 0.3) for less smoothing
- Increase `holding_frames_threshold` and `release_frames_threshold`

**Hands not detected in crowded environment:**
- Increase `strict_pair_threshold` to find pairs among more hands
- Check camera view and lighting

---

## Troubleshooting

### Issue: Serial Port Not Found

**Symptoms:**
- Error: "Could not open serial port"
- "Serial port not available" warnings

**Solutions:**
```bash
# Check if port exists
ls -la /dev/ttyUSB* /dev/ttyACM*

# Check permissions
groups | grep dialout

# Add user to dialout group
sudo adduser $USER dialout
# Log out and back in

# Check if device is busy
sudo lsof /dev/ttyUSB0
```

### Issue: No Hand Detections

**Symptoms:**
- "Only X valid hand(s) with keypoints" messages
- No detections in logs

**Solutions:**
1. **Check hand_lmk_detection is running:**
   ```bash
   ros2 topic list | grep hand
   ros2 topic echo /hobot_hand_lmk_detection --once
   ```

2. **Verify camera is working:**
   ```bash
   # Check camera topic
   ros2 topic list | grep image
   ```

3. **Check lighting and camera angle:**
   - Ensure hands are visible in frame
   - Good lighting helps detection
   - Avoid backlighting

4. **Verify config files:**
   ```bash
   ls -la config/
   # Should see: handLMKs.hbm, multitask_body_head_face_hand_kps_960x544.hbm
   ```

### Issue: Rapid Opening/Closing

**Symptoms:**
- Flower opens and closes rapidly
- Commands spamming serial monitor

**Solutions:**
1. **Increase frame thresholds:**
   ```bash
   -p holding_frames_threshold:=5 \
   -p release_frames_threshold:=7
   ```

2. **Increase smoothing:**
   ```bash
   -p ema_alpha:=0.1  # Lower = more smoothing
   ```

3. **Check if min_open_duration is working:**
   - Look for "Auto-closing after 10 seconds" in logs
   - Verify timer is counting correctly

### Issue: False Positives in Crowded Environment

**Symptoms:**
- Detects holding when hands are just close
- Multiple people trigger false detections

**Solutions:**
1. **Decrease thresholds:**
   ```bash
   -p holding_threshold_pixels:=25.0 \
   -p strict_pair_threshold:=25.0
   ```

2. **Increase interaction requirements:**
   - The code already uses interaction scoring
   - May need to adjust `calculate_hand_interaction_score()` method

### Issue: Hands Overlap - Keypoints Lost

**Symptoms:**
- "Only X valid hand(s)" when hands overlap
- Missing keypoints in detection

**Solutions:**
1. **The code already handles this:**
   - Uses wrist and finger bases (less occluded)
   - `get_hand_palm_center()` handles missing keypoints
   - `calculate_hand_interaction_score()` is robust to occlusion

2. **If still issues:**
   - Consider using MediaPipe (see `HAND_DETECTION_COMPARISON.md`)
   - Adjust keypoint indices if using different model

### Issue: Performance Issues

**Symptoms:**
- High CPU usage
- Laggy detection
- Frame drops

**Solutions:**
1. **Remove debug prints (already done in production code)**
2. **Reduce frame rate:**
   - Adjust hand_lmk_detection launch parameters
3. **Use MediaPipe:**
   - Better optimized for edge devices
   - See comparison document

### Issue: Arduino Not Responding

**Symptoms:**
- Commands sent but Arduino doesn't react
- Serial monitor shows no commands

**Solutions:**
1. **Check Arduino is programmed:**
   ```bash
   # Verify sketch is uploaded
   arduino-cli board list
   ```

2. **Check serial connection:**
   ```bash
   # Test serial communication
   echo "open" > /dev/ttyUSB0
   # Check Arduino serial monitor
   ```

3. **Verify baud rate matches:**
   - Code uses 9600
   - Arduino should use `Serial.begin(9600)`

4. **Check Arduino code:**
   - Master should use `Serial.readStringUntil('\n')`
   - Should handle "open" and "close" commands

---

## Development Notes

### Key Design Decisions

1. **Why find closest pair instead of requiring exactly 2 hands?**
   - In crowded environments, there are often 4+ hands (2 people)
   - We need to find which 2 are actually holding
   - Closest pair algorithm handles this

2. **Why use wrist + palm center instead of just wrist?**
   - More robust to occlusion
   - Palm center uses multiple keypoints
   - Better accuracy when hands overlap

3. **Why 10-second auto-close?**
   - Prevents rapid cycling
   - Gives flower time to fully open
   - User requirement for minimum display time

4. **Why interaction scoring?**
   - Simple distance can give false positives
   - Multiple keypoints provide better confidence
   - Handles edge cases better

### Code Structure Best Practices

1. **Error Handling:**
   - All keypoint extraction wrapped in try-except
   - Serial communication has error handling
   - Graceful degradation (continues without serial if unavailable)

2. **State Management:**
   - Clear state variables
   - Proper reset on transitions
   - Prevents state inconsistencies

3. **Performance:**
   - Minimal debug prints in production
   - Efficient algorithms (O(n²) for pair finding, acceptable for <10 hands)
   - EMA smoothing reduces computation

4. **Maintainability:**
   - Clear function names
   - Comments explain complex logic
   - Parameters are configurable

### Testing Checklist

- [ ] Serial port opens successfully
- [ ] Hand detection receives messages
- [ ] Keypoints extracted correctly
- [ ] Closest pair found correctly
- [ ] Holding detection works
- [ ] "open" command sent once
- [ ] 10-second timer works
- [ ] "close" command sent after 10 seconds
- [ ] No command spamming
- [ ] Works with 2 hands (one person)
- [ ] Works with 4 hands (two people)
- [ ] Works in crowded environment
- [ ] Handles hand overlap
- [ ] Auto-start on boot works

---

## Future Improvements

### Short-term

1. **Better Occlusion Handling:**
   - Use temporal tracking to predict hand positions
   - Interpolate missing keypoints

2. **Gesture Recognition:**
   - Distinguish handshake vs. hand-holding
   - Detect specific gestures

3. **Multi-person Tracking:**
   - Track which person's hands are holding
   - Prevent same-person false positives

### Medium-term

1. **Switch to MediaPipe:**
   - Better performance on edge devices
   - Better occlusion handling
   - See `HAND_DETECTION_COMPARISON.md`

2. **Machine Learning:**
   - Train custom model for hand-holding
   - Better accuracy in crowded scenes

3. **Configuration UI:**
   - Web interface for parameter tuning
   - Real-time visualization

### Long-term

1. **Multi-camera Support:**
   - Use multiple cameras for better coverage
   - 3D hand position estimation

2. **Advanced Analytics:**
   - Track holding duration
   - Count interactions
   - Generate reports

3. **Integration:**
   - Connect to other systems
   - API for external control
   - Database logging

---

## Additional Resources

### Documentation
- [D-Robotics RDK Documentation](https://developer.d-robotics.cc/rdk_doc/en/RDK)
- [Hand Landmark Detection](https://developer.d-robotics.cc/rdk_doc/en/Robot_development/boxs/body/hand_lmk_detection)
- [MediaPipe Hand Gesture](https://developer.d-robotics.cc/rdk_doc/en/Robot_development/boxs/body/hand_lmk_gesture_mediapipe)

### Related Files
- `README.md` - Basic usage guide
- `SETUP_AUTOSTART.md` - Auto-start setup
- `HAND_DETECTION_COMPARISON.md` - MediaPipe vs hand_lmk_detection
- `ISSUES_FOUND.md` - Known issues and fixes

### Support
- Check logs: `journalctl -u hand-holding-detector.service -f`
- ROS2 topics: `ros2 topic list`
- Serial monitor: Use Arduino IDE or `screen /dev/ttyUSB0 9600`

---

## Quick Reference

### Common Commands

```bash
# Start hand detection
source /opt/tros/humble/setup.bash
export CAM_TYPE=mipi
ros2 launch hand_lmk_detection hand_lmk_detection.launch.py

# Start holding detector
source /opt/tros/humble/setup.bash
cd ~/Desktop/bloom
./hand_holding_detector.py --ros-args -p serial_port:='/dev/ttyUSB0'

# Check topics
ros2 topic list
ros2 topic echo /hobot_hand_lmk_detection --once

# View service logs
sudo journalctl -u hand-holding-detector.service -f

# Restart service
sudo systemctl restart hand-holding-detector.service
```

### File Locations

- Main script: `~/Desktop/bloom/hand_holding_detector.py`
- Startup script: `~/Desktop/bloom/start_hand_holding_detector.sh`
- Service file: `/etc/systemd/system/hand-holding-detector.service`
- Config files: `~/Desktop/bloom/config/`
- Arduino sketches: `~/Desktop/bloom/MCbloomMaster5/` and `MCbloomSlave5.ino`

---

**Last Updated:** December 2024  
**Version:** 1.0  
**Author:** Development Team

