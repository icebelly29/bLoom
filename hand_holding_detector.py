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
        self.declare_parameter('holding_threshold_pixels', 30.0)  # Stricter default threshold
        self.declare_parameter('strict_pair_threshold', 30.0)  # Maximum distance to consider a pair
        self.declare_parameter('holding_frames_threshold', 3)
        self.declare_parameter('release_frames_threshold', 5)
        self.declare_parameter('ema_alpha', 0.2)  # Smoothing factor for distance
        self.declare_parameter('min_open_duration_seconds', 10.0)  # Minimum time to stay open before allowing close

        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.distance_threshold = self.get_parameter('holding_threshold_pixels').get_parameter_value().double_value
        self.strict_pair_threshold = self.get_parameter('strict_pair_threshold').get_parameter_value().double_value
        self.holding_frames_threshold = self.get_parameter('holding_frames_threshold').get_parameter_value().integer_value
        self.release_frames_threshold = self.get_parameter('release_frames_threshold').get_parameter_value().integer_value
        self.ema_alpha = self.get_parameter('ema_alpha').get_parameter_value().double_value
        self.min_open_duration = self.get_parameter('min_open_duration_seconds').get_parameter_value().double_value


        # --- State ---
        self.is_holding = False
        self.holding_frames_count = 0
        self.release_frames_count = 0
        self.smoothed_distance = None
        self.last_open_time = None  # Timestamp when "open" was last sent
        self.last_command_sent = None  # Track last command to prevent spamming
        self.is_in_open_cycle = False  # True when we've sent "open" and are waiting for auto-close

        # --- Serial Port Setup ---
        self.serial_port = None
        try:
            self.get_logger().info(f"Attempting to open serial port: {self.serial_port_name} at {self.baud_rate} baud.")
            self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            self.get_logger().warn("Node will run without serial output. Please check port and permissions (e.g., 'sudo adduser $USER dialout').")

        # --- ROS2 Subscription ---
        self.subscription = self.create_subscription(
            PerceptionTargets,
            '/hobot_hand_lmk_detection',
            self.hand_lmk_callback,
            10
        )
        self.get_logger().info(f"Hand Holding Detector is running. Waiting for hand landmarks...")
        self.get_logger().info(f"Holding distance threshold set to: {self.distance_threshold} pixels.")
        self.get_logger().info(f"Strict pair threshold set to: {self.strict_pair_threshold} pixels.")
        self.get_logger().info(f"EMA alpha set to: {self.ema_alpha}.")
        self.get_logger().info(f"Minimum open duration: {self.min_open_duration} seconds.")


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
        # Extract all hand keypoints from all person targets
        # Each person can have 0, 1, or 2 hands (left/right)
        valid_hands_with_keypoints = []
        
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
            if self.is_holding:
                self.release_frames_count += 1
                if self.release_frames_count >= self.release_frames_threshold:
                    self.is_holding = False
                    self.release_frames_count = 0
                    self.smoothed_distance = None
                    self.get_logger().info(f"Hands released (only {len(valid_hands_with_keypoints)} hand(s) detected). Sending 'close'.")
                    self.send_serial_signal('close')
            return


        # Find the best pair using interaction score and distance
        best_pair = None
        best_score = 0.0
        best_wrist_dist = float('inf')
        
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
        
        if best_pair is None:
            # If we're in an open cycle, check if 10 seconds have passed
            if self.is_in_open_cycle:
                if self.last_open_time is not None:
                    time_since_open = time.time() - self.last_open_time
                    if time_since_open >= self.min_open_duration:
                        # 10 seconds have passed, send close automatically
                        self.get_logger().info(f"Auto-closing after {self.min_open_duration} seconds. Sending 'close'.")
                        self.send_serial_signal('close')
                        self.is_in_open_cycle = False
                        self.last_open_time = None
                        self.is_holding = False
                        self.holding_frames_count = 0
                        self.release_frames_count = 0
                        self.smoothed_distance = None
            return
        
        hand1, hand2, wrist_dist = best_pair
        
        # Use wrist distance for detection (most reliable, least occluded)
        dist = wrist_dist

        # --- Calculate and Smooth Distance ---
        try:
            # Apply Exponential Moving Average (EMA) to smooth the distance
            if self.smoothed_distance is None:
                self.smoothed_distance = dist
            else:
                self.smoothed_distance = self.ema_alpha * dist + (1 - self.ema_alpha) * self.smoothed_distance

            # --- Detection Logic based on Smoothed Distance ---
            # Use strict threshold - hands holding are very close
            currently_holding = self.smoothed_distance < self.distance_threshold

            # Check if we're in an open cycle (waiting for auto-close after 10 seconds)
            if self.is_in_open_cycle:
                if self.last_open_time is not None:
                    time_since_open = time.time() - self.last_open_time
                    if time_since_open >= self.min_open_duration:
                        # 10 seconds have passed, send close automatically
                        self.get_logger().info(f"Auto-closing after {self.min_open_duration} seconds. Sending 'close'.")
                        self.send_serial_signal('close')
                        self.is_in_open_cycle = False
                        self.last_open_time = None
                        self.is_holding = False
                        self.holding_frames_count = 0
                        self.release_frames_count = 0
                        self.smoothed_distance = None  # Reset EMA
                # Don't process new detections while in open cycle
                return

            # Normal detection logic (only when not in open cycle)
            if currently_holding:
                self.holding_frames_count += 1
                self.release_frames_count = 0
                if self.holding_frames_count >= self.holding_frames_threshold and not self.is_holding:
                    # Detected holding - send open and start 10-second timer
                    self.is_holding = True
                    self.is_in_open_cycle = True
                    self.get_logger().info(f"Hand holding DETECTED! (Distance: {self.smoothed_distance:.2f}px). Sending 'open' to serial.")
                    self.send_serial_signal('open')
                    self.last_open_time = time.time()  # Record when we sent "open"
            else:
                # Hands not holding - reset counters but don't close (wait for auto-close)
                self.holding_frames_count = 0
                self.release_frames_count = 0

        except AttributeError:
            self.get_logger().warn("Could not get x, y from point structure.", throttle_duration_sec=5)
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
