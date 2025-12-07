# Current Project Status - Hand Holding Detector

**Last Updated:** December 2024  
**Status:** Functional but needs stability improvements

---

## ✅ What's Working

### Core Functionality
- ✅ Hand detection from `hand_lmk_detection` ROS2 topic
- ✅ Extracts hand keypoints from person targets (handles multiple people)
- ✅ Finds closest pair of hands among all detected hands
- ✅ Calculates distance between wrists and palm centers
- ✅ Detects hand holding when distance < threshold
- ✅ Sends "open" command to Arduino via serial
- ✅ Auto-closes after 10 seconds (prevents rapid cycling)
- ✅ Command deduplication (prevents spamming)
- ✅ Serial port communication working

### Features Implemented
- ✅ Multi-hand support (works with 2, 4, or more hands)
- ✅ Hand overlap handling (uses wrist + finger bases, not fingertips)
- ✅ Exponential Moving Average (EMA) for distance smoothing
- ✅ Temporal filtering (frame thresholds prevent false triggers)
- ✅ Auto-start on boot (systemd service configured)
- ✅ Comprehensive documentation (see `docs/` folder)

### Code Quality
- ✅ Error handling for serial port failures
- ✅ Graceful shutdown handling
- ✅ ROS2 parameter configuration
- ✅ Clean code structure with helper methods

---

## ⚠️ Current Issues

### 1. **Random/Unpredictable Behavior** (MAIN ISSUE)

**Problem:** System behaves randomly - detections are inconsistent

**Root Causes Identified:**
1. **Best pair selection changes every frame**
   - In crowded environments, algorithm picks "best pair" each frame
   - Different hands can be selected between frames (left vs right)
   - Keypoints can be lost/regained due to occlusion
   - Result: Distance jumps around, causing instability

2. **EMA smoothing with changing pairs**
   - When best pair changes, distance changes suddenly
   - EMA smooths these jumps, causing delayed/false triggers
   - Example: Frame 1 = Pair A (25px), Frame 2 = Pair B (28px), Frame 3 = Pair A (32px)
   - EMA mixes these, causing random behavior

3. **No pair tracking/persistence**
   - System doesn't remember which specific pair it detected
   - Just picks "best" each frame, can switch between different pairs
   - No way to track the same two hands over time

4. **Threshold edge case**
   - If distance is right at threshold (29-31px), can flicker
   - No hysteresis (different thresholds for on/off)

5. **Immediate re-detection after cycle**
   - After 10-second cycle ends, if hands still detected, immediately triggers again
   - No cooldown period

**Impact:**
- Unpredictable detection timing
- Can trigger when hands aren't actually holding
- Can miss actual holding events
- Inconsistent user experience

---

## 🔧 Proposed Solutions (Not Yet Implemented)

### Solution 1: Pair Tracking (HIGH PRIORITY)
**What:** Track specific hand pair once detected, stick with it
**Why:** Prevents switching between different pairs
**Status:** Design ready, not implemented

### Solution 2: Cooldown Period (HIGH PRIORITY)
**What:** Add 5-second cooldown after closing before allowing new detection
**Why:** Prevents immediate re-triggering
**Status:** Design ready, not implemented

### Solution 3: Hysteresis (MEDIUM PRIORITY)
**What:** Different thresholds for detecting holding (30px) vs releasing (40px)
**Why:** Prevents flickering at threshold edge
**Status:** Design ready, not implemented

### Solution 4: Increase Frame Thresholds (LOW PRIORITY)
**What:** Require more consecutive frames (5-7 instead of 3-5)
**Why:** More stability, less noise sensitivity
**Status:** Easy to implement via parameters

### Solution 5: Better Pair Stability (MEDIUM PRIORITY)
**What:** Only switch pairs if current tracked pair is lost
**Why:** Maintains consistency
**Status:** Design ready, not implemented

---

## 📊 Metrics System (Requested, Not Implemented)

**Requested Features:**
- Track detection counts
- Distance statistics (min, max, average)
- Holding durations
- Performance metrics (FPS, callback times)
- Export to JSON file
- ROS2 service to query metrics

**Status:** Design provided, not yet implemented

---

## 📁 Project Structure

```
bloom/
├── hand_holding_detector.py          # Main detection script (WORKING)
├── MCbloomMaster5/                   # Arduino master sketch
│   └── MCbloomMaster5.ino
├── MCbloomSlave5.ino                 # Arduino slave sketch
├── start_hand_holding_detector.sh     # Startup script
├── hand-holding-detector.service     # Systemd service file
├── install_autostart.sh              # Auto-start installer
├── test/                             # Unit tests
│   └── test_hand_holding_detector.py
├── config/                           # AI model config files
├── README.md                         # Basic usage
├── docs/
│   ├── DEVELOPMENT_GUIDE.md              # Comprehensive dev guide
│   ├── SETUP_AUTOSTART.md                # Auto-start instructions
│   ├── HAND_DETECTION_COMPARISON.md      # MediaPipe vs hand_lmk_detection
│   ├── ISSUES_FOUND.md                   # Known issues log
│   ├── PUSH_TO_GITHUB.md                 # GitHub setup guide
│   ├── STABILITY_IMPROVEMENTS.md         # Stability fixes documentation
│   └── CURRENT_STATUS.md                 # This file
```

---

## 🎯 Next Steps (Priority Order)

### Immediate (Fix Randomness)
1. **Implement pair tracking** - Track specific hand pair once detected
2. **Add cooldown period** - 5 seconds after closing before new detection
3. **Add hysteresis** - Different thresholds for on/off (30px/40px)
4. **Test and tune** - Adjust thresholds based on real-world testing

### Short-term
5. **Implement metrics system** - Track detections, distances, durations
6. **Add logging/visualization** - Better debugging and analysis
7. **Performance optimization** - If needed after metrics show bottlenecks

### Medium-term
8. **Consider MediaPipe switch** - If hand_lmk_detection still has issues
9. **Add web dashboard** - Real-time metrics visualization
10. **Database logging** - Long-term storage of metrics

---

## 🔑 Key Configuration

**Current Parameters:**
- `holding_threshold_pixels`: 30.0 (distance to consider holding)
- `strict_pair_threshold`: 30.0 (max distance to consider a pair)
- `holding_frames_threshold`: 3 (frames needed to trigger)
- `release_frames_threshold`: 5 (frames needed to release)
- `ema_alpha`: 0.2 (smoothing factor)
- `min_open_duration_seconds`: 10.0 (auto-close after 10s)

**Serial Port:** `/dev/ttyUSB0` at 9600 baud

**ROS2 Topic:** `/hobot_hand_lmk_detection` (ai_msgs/msg/PerceptionTargets)

---

## 🐛 Known Issues Summary

1. **Random behavior** - Main issue, needs pair tracking + cooldown + hysteresis
2. **No metrics** - Requested but not implemented
3. **Performance** - Could be better, but acceptable for now
4. **Hand overlap** - Partially handled, but could be improved with MediaPipe

---

## 💡 Key Design Decisions Made

1. **Closest pair algorithm** - Instead of requiring exactly 2 hands, finds closest pair
   - Works in crowded environments
   - Handles 2 people (4 hands) scenario

2. **Wrist + palm center** - Uses less-occluded keypoints
   - Better when hands overlap
   - More robust to missing keypoints

3. **10-second auto-close** - Prevents rapid cycling
   - User requirement
   - Gives flower time to display

4. **Command deduplication** - Prevents serial spamming
   - Only sends when state changes
   - Tracks last command sent

---

## 📝 For Next Conversation

When starting a new conversation, you can say:

> "I have a hand holding detector project. See CURRENT_STATUS.md for context. The main issue is random/unpredictable behavior. I need to implement pair tracking, cooldown period, and hysteresis to fix it. The code is in `hand_holding_detector.py`."

**Key Files to Reference:**
- `hand_holding_detector.py` - Main code
- `docs/DEVELOPMENT_GUIDE.md` - Full documentation
- `docs/CURRENT_STATUS.md` - This file (current state)

**Current Focus:**
- Fix random behavior (pair tracking + cooldown + hysteresis)
- Add metrics system (optional, but requested)

---

## 🔄 Recent Changes Made

1. ✅ Fixed shutdown errors (ExternalShutdownException handling)
2. ✅ Removed excessive debug prints (performance improvement)
3. ✅ Implemented closest pair finding (crowded environment support)
4. ✅ Added palm center calculation (occlusion handling)
5. ✅ Added interaction scoring (better detection)
6. ✅ Implemented 10-second auto-close (prevents rapid cycling)
7. ✅ Added command deduplication (prevents spamming)
8. ✅ Created comprehensive documentation

---

## 📞 Quick Reference

**Start detection:**
```bash
source /opt/tros/humble/setup.bash
export CAM_TYPE=mipi
ros2 launch hand_lmk_detection hand_lmk_detection.launch.py
```

**Start detector:**
```bash
source /opt/tros/humble/setup.bash
cd ~/Desktop/bloom
./hand_holding_detector.py --ros-args -p serial_port:='/dev/ttyUSB0'
```

**View logs:**
```bash
sudo journalctl -u hand-holding-detector.service -f
```

**Check metrics (when implemented):**
```bash
ros2 service call /get_metrics std_srvs/srv/Trigger
cat /tmp/hand_holding_metrics.json
```

---

**Status:** Ready for stability improvements and metrics implementation.

