#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ai_msgs.msg import PerceptionTargets
import serial
import math
import time

class HandHoldingDetector(Node):
    def __init__(self):
        super().__init__('hand_holding_detector')

        # --- Parameters ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('holding_threshold_pixels', 30.0)  # Threshold to detect holding
        self.declare_parameter('release_threshold_pixels', 40.0)  # Threshold to detect release (hysteresis)
        self.declare_parameter('strict_pair_threshold', 30.0)  # Maximum distance to consider a pair
        self.declare_parameter('holding_frames_threshold', 3)
        self.declare_parameter('release_frames_threshold', 5)
        self.declare_parameter('ema_alpha', 0.2)  # Smoothing factor for distance
        self.declare_parameter('min_open_duration_seconds', 10.0)  # Minimum time to stay open before allowing close
        self.declare_parameter('cooldown_seconds', 5.0)  # Cooldown period after closing before new detection

        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.distance_threshold = self.get_parameter('holding_threshold_pixels').get_parameter_value().double_value
        self.release_threshold = self.get_parameter('release_threshold_pixels').get_parameter_value().double_value
        self.strict_pair_threshold = self.get_parameter('strict_pair_threshold').get_parameter_value().double_value
        self.holding_frames_threshold = self.get_parameter('holding_frames_threshold').get_parameter_value().integer_value
        self.release_frames_threshold = self.get_parameter('release_frames_threshold').get_parameter_value().integer_value
        self.ema_alpha = self.get_parameter('ema_alpha').get_parameter_value().double_value
        self.min_open_duration = self.get_parameter('min_open_duration_seconds').get_parameter_value().double_value
        self.cooldown_seconds = self.get_parameter('cooldown_seconds').get_parameter_value().double_value


        # --- State ---
        self.is_holding = False
        self.holding_frames_count = 0
        self.release_frames_count = 0
        self.smoothed_distance = None
        self.last_open_time = None  # Timestamp when "open" was last sent
        self.last_close_time = None  # Timestamp when "close" was last sent (for cooldown)
        self.last_command_sent = None  # Track last command to prevent spamming
        self.is_in_open_cycle = False  # True when we've sent "open" and are waiting for auto-close
        self.tracked_pair_ids = None  # Track which specific hand pair we're monitoring (hand1_id, hand2_id)
        self.tracked_pair_lost_frames = 0  # Count frames where tracked pair wasn't found (grace period)
        self.tracked_pair_lost_threshold = 10  # Clear tracking after this many consecutive frames without finding pair

        # --- Serial Port Setup ---
        self.serial_port = None
        try:
            self.get_logger().info(f"Attempting to open serial port: {self.serial_port_name} at {self.baud_rate} baud.")
            # Check if port exists
            import os
            if not os.path.exists(self.serial_port_name):
                self.get_logger().error(f"Serial port {self.serial_port_name} does not exist!")
                self.get_logger().warn("Node will run without serial output. Make sure Arduino is connected and port is correct.")
            else:
                self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
                self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            self.get_logger().warn("Node will run without serial output.")
            self.get_logger().warn("Possible causes:")
            self.get_logger().warn("  1. Port is busy (close Arduino monitor/IDE)")
            self.get_logger().warn("  2. Wrong port name (check with: ls /dev/ttyUSB* /dev/ttyACM*)")
            self.get_logger().warn("  3. Permissions issue (run: sudo adduser $USER dialout, then log out/in)")

        # --- ROS2 Subscription ---
        self.subscription = self.create_subscription(
            PerceptionTargets,
            '/hobot_hand_lmk_detection',
            self.hand_lmk_callback,
            10
        )
        self.get_logger().info(f"Hand Holding Detector is running. Waiting for hand landmarks...")
        self.get_logger().info(f"Holding threshold: {self.distance_threshold}px, Release threshold: {self.release_threshold}px (hysteresis)")
        self.get_logger().info(f"Strict pair threshold set to: {self.strict_pair_threshold} pixels.")
        self.get_logger().info(f"EMA alpha set to: {self.ema_alpha}.")
        self.get_logger().info(f"Minimum open duration: {self.min_open_duration} seconds.")
        self.get_logger().info(f"Cooldown period: {self.cooldown_seconds} seconds after closing.")


    def get_all_hand_keypoints_from_person(self, person_target):
        """
        Extract all hand keypoints from a person target.
        Each person can have multiple hand_kps entries (left hand, right hand).
        Returns list of hand keypoint arrays, each with 21 keypoints.
        """
        hands = []
        try:
            if not hasattr(person_target, 'points') or not person_target.points:
                return hands
            
            # Look for hand keypoints in the points array
            # Each hand_kps entry represents one hand (left or right)
            for point_group in person_target.points:
                point_type = getattr(point_group, 'type', '')
                
                # Look for hand keypoint types - should be 'hand_kps'
                if point_type == 'hand_kps':
                    if hasattr(point_group, 'point') and point_group.point and len(point_group.point) > 0:
                        # Extract all keypoints as (x, y) tuples
                        # Hand landmarks typically have 21 keypoints (wrist + 4 fingers * 4 joints + fingertips)
                        keypoints = []
                        for kp in point_group.point:
                            if hasattr(kp, 'x') and hasattr(kp, 'y'):
                                keypoints.append((kp.x, kp.y))
                        # Only add if we have a reasonable number of keypoints (at least 5)
                        if len(keypoints) >= 5:
                            hands.append(keypoints)
        except (AttributeError, IndexError, TypeError) as e:
            self.get_logger().warn(f"Exception extracting hand keypoints: {type(e).__name__}: {e}", throttle_duration_sec=5)
        return hands

    def get_hand_palm_center(self, hand_keypoints):
        """
        Calculate palm center from wrist and base of fingers (less occluded keypoints).
        Handles missing keypoints gracefully for better overlap robustness.
        Returns (x, y) or None if insufficient keypoints.
        """
        if not hand_keypoints or len(hand_keypoints) < 3:
            return None
        
        # Use wrist (0) and base of fingers - less likely to be occluded
        # Hand landmark indices: 0=wrist, 1=thumb_cmc, 5=index_mcp, 9=middle_mcp, 13=ring_mcp, 17=pinky_mcp
        # Try to get as many as available, but need at least wrist
        palm_keypoint_indices = [0, 1, 5, 9, 13, 17]
        valid_points = []
        
        for idx in palm_keypoint_indices:
            if idx < len(hand_keypoints):
                # Check if point is valid (not zero or invalid)
                pt = hand_keypoints[idx]
                if pt[0] > 0 and pt[1] > 0:  # Basic validity check
                    valid_points.append(pt)
        
        # Need at least wrist (index 0) and one other point
        if len(valid_points) < 2:
            return None
        
        # Calculate centroid
        avg_x = sum(p[0] for p in valid_points) / len(valid_points)
        avg_y = sum(p[1] for p in valid_points) / len(valid_points)
        return (avg_x, avg_y)
    
    def calculate_hand_interaction_score(self, hand1_keypoints, hand2_keypoints):
        """
        Calculate interaction score between two hands using multiple keypoints.
        Returns a score 0-1 indicating how likely they are holding hands.
        More robust to occlusion/overlap.
        """
        if not hand1_keypoints or not hand2_keypoints:
            return 0.0
        
        if len(hand1_keypoints) < 5 or len(hand2_keypoints) < 5:
            return 0.0
        
        # Get wrist positions (most reliable)
        wrist1 = hand1_keypoints[0] if len(hand1_keypoints) > 0 else None
        wrist2 = hand2_keypoints[0] if len(hand2_keypoints) > 0 else None
        
        if not wrist1 or not wrist2:
            return 0.0
        
        # Calculate wrist-to-wrist distance
        wrist_dist = math.sqrt((wrist1[0] - wrist2[0])**2 + (wrist1[1] - wrist2[1])**2)
        
        # Get palm centers (more robust to occlusion)
        palm1 = self.get_hand_palm_center(hand1_keypoints)
        palm2 = self.get_hand_palm_center(hand2_keypoints)
        
        if not palm1 or not palm2:
            # Fall back to wrist distance only
            return 1.0 if wrist_dist < self.distance_threshold else 0.0
        
        palm_dist = math.sqrt((palm1[0] - palm2[0])**2 + (palm1[1] - palm2[1])**2)
        
        # Check multiple keypoints for proximity
        # Use base of fingers (less occluded than fingertips)
        finger_base_indices = [1, 5, 9, 13, 17]  # Thumb, index, middle, ring, pinky bases
        close_keypoints = 0
        total_checks = 0
        
        for idx in finger_base_indices:
            if idx < len(hand1_keypoints) and idx < len(hand2_keypoints):
                pt1 = hand1_keypoints[idx]
                pt2 = hand2_keypoints[idx]
                
                # Check if points are valid
                if pt1[0] > 0 and pt1[1] > 0 and pt2[0] > 0 and pt2[1] > 0:
                    dist = math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
                    if dist < self.distance_threshold * 1.5:  # Slightly more lenient for finger bases
                        close_keypoints += 1
                    total_checks += 1
        
        # Score based on wrist distance (primary) and palm distance (secondary)
        # If wrist is very close, it's likely holding
        if wrist_dist < self.distance_threshold:
            score = 1.0
        elif wrist_dist < self.distance_threshold * 1.5:
            # Check if multiple keypoints are close
            if total_checks > 0 and close_keypoints >= 2:
                score = 0.8
            else:
                score = 0.5
        else:
            score = 0.0
        
        return score

    def hand_lmk_callback(self, msg):
        try:
            # Extract all hand keypoints from all person targets
            # Each person can have 0, 1, or 2 hands (left/right)
            valid_hands_with_keypoints = []
            
            if not hasattr(msg, 'targets') or msg.targets is None:
                return
            
            for i, target in enumerate(msg.targets):
                target_type = getattr(target, 'type', 'unknown')
                
                # Extract hand keypoints from person targets only
                if target_type == 'person':
                    # Get all hand keypoints for each person (can be 0, 1, or 2 hands)
                    hand_keypoints_list = self.get_all_hand_keypoints_from_person(target)
                    for hand_idx, hand_kps in enumerate(hand_keypoints_list):
                        if len(hand_kps) >= 5:  # Need at least 5 keypoints
                            palm_center = self.get_hand_palm_center(hand_kps)
                            if palm_center:
                                valid_hands_with_keypoints.append({
                                    'id': f"person_{i}_hand_{hand_idx}",
                                    'wrist': hand_kps[0],  # Wrist position (index 0)
                                    'palm_center': palm_center,
                                    'keypoints': hand_kps
                                })
            
            # Need at least 2 hands to detect holding
            if len(valid_hands_with_keypoints) < 2:
                # If we're in an open cycle, check if 10 seconds have passed before allowing close
                if self.is_in_open_cycle:
                    if self.last_open_time is not None:
                        time_since_open = time.time() - self.last_open_time
                        if time_since_open >= self.min_open_duration:
                            # 10 seconds have passed - now we can close if hands are gone
                            self.release_frames_count += 1
                            if self.release_frames_count >= self.release_frames_threshold:
                                self.get_logger().info(f"Hands released (only {len(valid_hands_with_keypoints)} hand(s) detected) after {self.min_open_duration} seconds. Sending 'close'.")
                                self.send_serial_signal('close')
                                self.is_in_open_cycle = False
                                self.last_open_time = None
                                self.last_close_time = time.time()  # Record close time for cooldown
                                self.is_holding = False
                                self.holding_frames_count = 0
                                self.release_frames_count = 0
                                self.smoothed_distance = None
                                self.tracked_pair_ids = None  # Clear tracking after closing
                        else:
                            # Still within 10-second minimum - don't allow closing yet
                            self.release_frames_count = 0
                            self.get_logger().debug(f"Only {len(valid_hands_with_keypoints)} hand(s) detected, but still within {self.min_open_duration}s minimum. Waiting...")
                return


            # Check cooldown period - don't detect if we just closed
            if self.last_close_time is not None:
                time_since_close = time.time() - self.last_close_time
                if time_since_close < self.cooldown_seconds:
                    # Still in cooldown, skip detection
                    return
            
            # Find the best pair using interaction score and distance
            # If we have a tracked pair, try to use it first
            tracked_pair = None
            if self.tracked_pair_ids is not None:
                # Look for the tracked pair in current hands
                tracked_hand1 = None
                tracked_hand2 = None
                for hand in valid_hands_with_keypoints:
                    if hand['id'] == self.tracked_pair_ids[0]:
                        tracked_hand1 = hand
                    elif hand['id'] == self.tracked_pair_ids[1]:
                        tracked_hand2 = hand
                
                if tracked_hand1 and tracked_hand2:
                    # Tracked pair still available - use it
                    wrist_dist = math.sqrt(
                        (tracked_hand1['wrist'][0] - tracked_hand2['wrist'][0])**2 + 
                        (tracked_hand1['wrist'][1] - tracked_hand2['wrist'][1])**2
                    )
                    if wrist_dist < self.strict_pair_threshold * 2:  # More lenient for tracked pair
                        tracked_pair = (tracked_hand1, tracked_hand2, wrist_dist)
                else:
                    # Tracked pair not found this frame - increment lost counter
                    self.tracked_pair_lost_frames += 1
                    if self.tracked_pair_lost_frames >= self.tracked_pair_lost_threshold:
                        # Lost for too many frames - clear tracking
                        self.tracked_pair_ids = None
                        self.tracked_pair_lost_frames = 0
                        self.get_logger().info("Tracked pair lost (not found for {} frames), will search for new pair".format(self.tracked_pair_lost_threshold))
            
            # If we have a tracked pair, use it; otherwise find best pair
            best_pair = None
            best_score = 0.0
            best_wrist_dist = float('inf')
            
            if tracked_pair:
                # Use tracked pair - reset lost counter since we found it
                best_pair = tracked_pair
                self.tracked_pair_lost_frames = 0  # Reset counter when pair is found
            else:
                # Find best pair from all available hands
                for i in range(len(valid_hands_with_keypoints)):
                    for j in range(i + 1, len(valid_hands_with_keypoints)):
                        hand1 = valid_hands_with_keypoints[i]
                        hand2 = valid_hands_with_keypoints[j]
                        
                        # Calculate interaction score (handles overlap better)
                        interaction_score = self.calculate_hand_interaction_score(
                            hand1['keypoints'], 
                            hand2['keypoints']
                        )
                        
                        # Calculate wrist-to-wrist distance
                        wrist_dist = math.sqrt(
                            (hand1['wrist'][0] - hand2['wrist'][0])**2 + 
                            (hand1['wrist'][1] - hand2['wrist'][1])**2
                        )
                        
                        # Only consider pairs within strict threshold
                        if wrist_dist < self.strict_pair_threshold:
                            # Prefer pairs with high interaction score and close distance
                            if interaction_score > best_score or (interaction_score == best_score and wrist_dist < best_wrist_dist):
                                best_score = interaction_score
                                best_wrist_dist = wrist_dist
                                best_pair = (hand1, hand2, wrist_dist)
            
            # If we found a new best pair and don't have a tracked pair, start tracking it
            if best_pair and self.tracked_pair_ids is None:
                hand1, hand2, _ = best_pair
                self.tracked_pair_ids = (hand1['id'], hand2['id'])
                self.tracked_pair_lost_frames = 0  # Reset lost counter
                self.get_logger().info(f"Started tracking pair: {hand1['id']} and {hand2['id']}")
            
            if best_pair is None:
                # No valid pair found
                # Don't clear tracking immediately - let the grace period handle it
                # This prevents clearing on temporary occlusions
                
                # If we're in an open cycle, check if we should close
                if self.is_in_open_cycle:
                    if self.last_open_time is not None:
                        time_since_open = time.time() - self.last_open_time
                        if time_since_open >= self.min_open_duration:
                            # 10 seconds have passed - no pair found means hands released
                            # Send close since hands are no longer holding
                            self.release_frames_count += 1
                            if self.release_frames_count >= self.release_frames_threshold:
                                self.get_logger().info(f"Hands released (no pair found) after {self.min_open_duration} seconds. Sending 'close'.")
                                self.send_serial_signal('close')
                                self.is_in_open_cycle = False
                                self.last_open_time = None
                                self.last_close_time = time.time()  # Record close time for cooldown
                                self.is_holding = False
                                self.holding_frames_count = 0
                                self.release_frames_count = 0
                                self.smoothed_distance = None
                                self.tracked_pair_ids = None  # Clear tracking after closing
                    else:
                        # Still within 10-second minimum - don't allow closing yet
                        self.release_frames_count = 0
                return
            
            hand1, hand2, wrist_dist = best_pair
            
            # Use wrist distance for detection (most reliable, least occluded)
            dist = wrist_dist

            # --- Calculate and Smooth Distance ---
            # Apply Exponential Moving Average (EMA) to smooth the distance
            if self.smoothed_distance is None:
                self.smoothed_distance = dist
            else:
                self.smoothed_distance = self.ema_alpha * dist + (1 - self.ema_alpha) * self.smoothed_distance

            # --- Detection Logic with Hysteresis ---
            # Use different thresholds for detecting holding vs releasing (hysteresis)
            # This prevents flickering at the threshold edge
            
            if self.is_holding:
                # Currently holding - use release threshold (higher, more lenient)
                # Only release if distance exceeds release threshold
                currently_holding = self.smoothed_distance < self.release_threshold
            else:
                # Not holding - use holding threshold (lower, stricter)
                # Only detect holding if distance is below holding threshold
                currently_holding = self.smoothed_distance < self.distance_threshold

            # Check if we're in an open cycle (waiting for minimum 10 seconds before allowing close)
            if self.is_in_open_cycle:
                if self.last_open_time is not None:
                    time_since_open = time.time() - self.last_open_time
                    if time_since_open >= self.min_open_duration:
                        # 10 seconds have passed - now we can check if hands are still holding
                        # If hands are NOT holding anymore, send close
                        if not currently_holding:
                            self.release_frames_count += 1
                            if self.release_frames_count >= self.release_frames_threshold:
                                # Hands released after minimum duration - send close
                                self.get_logger().info(f"Hands released after {self.min_open_duration} seconds. Sending 'close'.")
                                self.send_serial_signal('close')
                                self.is_in_open_cycle = False
                                self.last_open_time = None
                                self.last_close_time = time.time()  # Record close time for cooldown
                                self.is_holding = False
                                self.holding_frames_count = 0
                                self.release_frames_count = 0
                                self.smoothed_distance = None  # Reset EMA
                                self.tracked_pair_ids = None  # Clear tracking after closing
                        else:
                            # Still holding - reset release counter and keep flower open
                            self.release_frames_count = 0
                    else:
                        # Still within 10-second minimum - don't allow closing yet
                        # Reset release counter to prevent premature closing
                        self.release_frames_count = 0
                return

            # Normal detection logic (only when not in open cycle)
            if currently_holding:
                self.holding_frames_count += 1
                self.release_frames_count = 0
                if self.holding_frames_count >= self.holding_frames_threshold and not self.is_holding:
                    # Detected holding - send open and start 10-second minimum timer
                    self.is_holding = True
                    self.is_in_open_cycle = True
                    self.get_logger().info(f"Hand holding DETECTED! (Distance: {self.smoothed_distance:.2f}px, Threshold: {self.distance_threshold}px). Sending 'open' to serial.")
                    self.send_serial_signal('open')
                    self.last_open_time = time.time()  # Record when we sent "open"
                    # Ensure we're tracking this pair
                    if self.tracked_pair_ids is None:
                        hand1, hand2, _ = best_pair
                        self.tracked_pair_ids = (hand1['id'], hand2['id'])
            else:
                # Hands not holding - reset counters
                self.holding_frames_count = 0
                self.release_frames_count = 0

        except AttributeError as e:
            self.get_logger().warn(f"Could not get x, y from point structure: {e}", throttle_duration_sec=5)
            return
        except Exception as e:
            # Catch any other exceptions to prevent node crash
            self.get_logger().error(f"Exception in hand_lmk_callback: {type(e).__name__}: {e}", throttle_duration_sec=2)
            return

    def send_serial_signal(self, signal):
        # Prevent spamming - only send if command changed
        if signal == self.last_command_sent:
            self.get_logger().debug(f"Skipping duplicate '{signal}' command")
            return
        
        if self.serial_port and self.serial_port.is_open:
            try:
                # Ensure the command is newline-terminated for readStringUntil('\n') on Arduino
                data = f"{signal}\n".encode('utf-8')
                bytes_written = self.serial_port.write(data)
                self.serial_port.flush()  # Ensure data is sent immediately
                self.get_logger().info(f"Sent '{signal}' to serial port ({bytes_written} bytes)")
                self.last_command_sent = signal  # Remember last command sent
            except serial.SerialException as e:
                self.get_logger().error(f"Error writing to serial port: {e}")
        else:
            self.get_logger().warn(f"Serial port not available. Cannot send signal: '{signal}'.")

def main(args=None):
    rclpy.init(args=args)
    node = HandHoldingDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # ROS2 context was shut down externally (e.g., by timeout or signal)
        pass
    except Exception as e:
        try:
            node.get_logger().error(f"Exception occurred: {e}")
        except Exception:
            pass
    finally:
        if node.serial_port and node.serial_port.is_open:
            node.serial_port.close()
            # Only log if ROS2 context is still valid
            try:
                if rclpy.ok():
                    node.get_logger().info("Serial port closed.")
            except Exception:
                pass
        try:
            if rclpy.ok():
                node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
