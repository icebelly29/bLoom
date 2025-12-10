#!/usr/bin/env python3
"""
Hand Holding Detector (Handshake Method)
---------------------------------------
This variant uses a "handshake" style check:
- Uses palm centers (wrist + finger bases) for proximity
- Uses fingertip-to-opposite-palm proximity
- Uses interaction scoring (palm + fingertips)
- Sends "open" when holding detected, waits minimum duration, then
  sends "close" only after hands are released
"""

import math
import time
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ai_msgs.msg import PerceptionTargets
import serial


class HandHoldingDetectorHandshake(Node):
    def __init__(self):
        super().__init__("hand_holding_detector_handshake")

        # --- Parameters ---
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 9600)

        # Detection thresholds
        self.declare_parameter("palm_distance_threshold", 60.0)  # px
        self.declare_parameter("fingertip_palm_threshold", 50.0)  # px
        self.declare_parameter("min_fingertip_contacts", 3)  # fingertip-to-palm contacts

        # Temporal smoothing and hysteresis
        self.declare_parameter("holding_frames_threshold", 3)
        self.declare_parameter("release_frames_threshold", 5)
        self.declare_parameter("ema_alpha", 0.2)

        # Timing
        self.declare_parameter("min_open_duration_seconds", 10.0)
        self.declare_parameter("cooldown_seconds", 5.0)

        # Pair selection
        self.declare_parameter("strict_pair_threshold", 120.0)  # max wrist distance to consider pair

        # Load parameters
        self.serial_port_name = self.get_parameter("serial_port").get_parameter_value().string_value
        self.baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.palm_distance_threshold = self.get_parameter("palm_distance_threshold").get_parameter_value().double_value
        self.fingertip_palm_threshold = self.get_parameter("fingertip_palm_threshold").get_parameter_value().double_value
        self.min_fingertip_contacts = self.get_parameter("min_fingertip_contacts").get_parameter_value().integer_value
        self.holding_frames_threshold = self.get_parameter("holding_frames_threshold").get_parameter_value().integer_value
        self.release_frames_threshold = self.get_parameter("release_frames_threshold").get_parameter_value().integer_value
        self.ema_alpha = self.get_parameter("ema_alpha").get_parameter_value().double_value
        self.min_open_duration = self.get_parameter("min_open_duration_seconds").get_parameter_value().double_value
        self.cooldown_seconds = self.get_parameter("cooldown_seconds").get_parameter_value().double_value
        self.strict_pair_threshold = self.get_parameter("strict_pair_threshold").get_parameter_value().double_value

        # --- State ---
        self.is_holding = False
        self.holding_frames_count = 0
        self.release_frames_count = 0
        self.smoothed_score = None
        self.last_open_time = None
        self.last_close_time = None
        self.last_command_sent = None
        self.is_in_open_cycle = False

        # --- Serial Port Setup ---
        self.serial_port = None
        try:
            self.get_logger().info(
                f"Attempting to open serial port: {self.serial_port_name} at {self.baud_rate} baud."
            )
            if not os.path.exists(self.serial_port_name):
                self.get_logger().error(f"Serial port {self.serial_port_name} does not exist!")
                self.get_logger().warn("Node will run without serial output. Check port and permissions.")
            else:
                self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
                self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            self.get_logger().warn("Node will run without serial output.")
            self.get_logger().warn(
                "Possible causes: 1) Port busy (close other monitors), 2) Wrong port, 3) Permissions (dialout)."
            )

        # --- ROS2 Subscription ---
        self.subscription = self.create_subscription(
            PerceptionTargets, "/hobot_hand_lmk_detection", self.hand_lmk_callback, 10
        )

        self.get_logger().info("Handshake detector running. Waiting for hand landmarks...")
        self.get_logger().info(
            f"Thresholds: palm={self.palm_distance_threshold}px, fingertip-palm={self.fingertip_palm_threshold}px, "
            f"contacts>={self.min_fingertip_contacts}, strict_pair={self.strict_pair_threshold}px"
        )
        self.get_logger().info(
            f"Timing: min_open={self.min_open_duration}s, cooldown={self.cooldown_seconds}s, "
            f"frames_hold={self.holding_frames_threshold}, frames_release={self.release_frames_threshold}"
        )

    # --- Helpers ---
    def get_hand_keypoints(self, person_target):
        """Extract all hand keypoints arrays from a person target."""
        hands = []
        try:
            if not hasattr(person_target, "points") or not person_target.points:
                return hands
            for point_group in person_target.points:
                ptype = getattr(point_group, "type", "")
                if ptype == "hand_kps" and hasattr(point_group, "point") and point_group.point:
                    kps = []
                    for kp in point_group.point:
                        if hasattr(kp, "x") and hasattr(kp, "y"):
                            kps.append((kp.x, kp.y))
                    if len(kps) >= 5:
                        hands.append(kps)
        except Exception as e:
            self.get_logger().warn(f"Error extracting hand keypoints: {e}", throttle_duration_sec=5)
        return hands

    def palm_center(self, kps):
        """Palm center from wrist + finger bases (indices 0,1,5,9,13,17)."""
        indices = [0, 1, 5, 9, 13, 17]
        pts = []
        for idx in indices:
            if idx < len(kps):
                x, y = kps[idx]
                if x > 0 and y > 0:
                    pts.append((x, y))
        if len(pts) < 3:
            return None
        avgx = sum(p[0] for p in pts) / len(pts)
        avgy = sum(p[1] for p in pts) / len(pts)
        return (avgx, avgy)

    def interaction_score(self, hand1, hand2):
        """
        Compute interaction score using:
        - Palm center distance
        - Fingertip to opposite palm proximity
        Score range ~[0,1], higher = more likely holding/shaking.
        """
        palm1 = self.palm_center(hand1)
        palm2 = self.palm_center(hand2)
        if not palm1 or not palm2:
            return 0.0

        # Palm proximity
        palm_dist = math.dist(palm1, palm2)
        palm_ok = palm_dist < self.palm_distance_threshold

        # Fingertip proximity to opposite palm
        fingertip_indices = [4, 8, 12, 16, 20]
        fingertip_contacts = 0
        for idx in fingertip_indices:
            if idx < len(hand1):
                if math.dist(hand1[idx], palm2) < self.fingertip_palm_threshold:
                    fingertip_contacts += 1
            if idx < len(hand2):
                if math.dist(hand2[idx], palm1) < self.fingertip_palm_threshold:
                    fingertip_contacts += 1

        # Score composition
        score = 0.0
        if palm_ok:
            score += 0.6  # palm proximity weight
        # Normalize fingertip contacts (0-10 possible contacts)
        score += min(fingertip_contacts / 10.0, 0.4)  # fingertip weight

        return min(score, 1.0), palm_dist, fingertip_contacts

    def send_serial_signal(self, signal):
        if signal == self.last_command_sent:
            return
        if self.serial_port and self.serial_port.is_open:
            try:
                data = f"{signal}\n".encode("utf-8")
                self.serial_port.write(data)
                self.serial_port.flush()
                self.get_logger().info(f"Sent '{signal}' to serial.")
                self.last_command_sent = signal
            except serial.SerialException as e:
                self.get_logger().error(f"Error writing to serial port: {e}")
        else:
            self.get_logger().warn(f"Serial port not available. Cannot send '{signal}'.")

    # --- Main Callback ---
    def hand_lmk_callback(self, msg):
        try:
            if not hasattr(msg, "targets") or msg.targets is None:
                return

            # Collect all hands
            hands = []
            for i, target in enumerate(msg.targets):
                if getattr(target, "type", "") == "person":
                    for hi, kps in enumerate(self.get_hand_keypoints(target)):
                        hands.append({"id": f"person_{i}_hand_{hi}", "kps": kps})

            # Need at least 2 hands
            if len(hands) < 2:
                # If in open cycle, wait for min duration before closing
                if self.is_in_open_cycle and self.last_open_time:
                    if time.time() - self.last_open_time >= self.min_open_duration:
                        self.release_frames_count += 1
                        if self.release_frames_count >= self.release_frames_threshold:
                            self.get_logger().info("Hands released (fewer than 2 detected). Sending 'close'.")
                            self.send_serial_signal("close")
                            self.is_in_open_cycle = False
                            self.last_open_time = None
                            self.last_close_time = time.time()
                            self.is_holding = False
                            self.holding_frames_count = 0
                            self.release_frames_count = 0
                            self.smoothed_score = None
                    else:
                        self.release_frames_count = 0
                return

            # Cooldown check
            if self.last_close_time:
                if time.time() - self.last_close_time < self.cooldown_seconds:
                    return

            # Find best pair by interaction score within distance limit
            best = None
            best_score = 0.0
            for i in range(len(hands)):
                for j in range(i + 1, len(hands)):
                    h1, h2 = hands[i], hands[j]
                    # Quick wrist distance check
                    if len(h1["kps"]) == 0 or len(h2["kps"]) == 0:
                        continue
                    wrist_dist = math.dist(h1["kps"][0], h2["kps"][0])
                    if wrist_dist > self.strict_pair_threshold:
                        continue
                    score, palm_dist, tip_contacts = self.interaction_score(h1["kps"], h2["kps"])
                    if score > best_score:
                        best_score = score
                        best = {
                            "pair": (h1, h2),
                            "score": score,
                            "palm_dist": palm_dist,
                            "wrist_dist": wrist_dist,
                            "tip_contacts": tip_contacts,
                        }

            if not best:
                return

            # Smooth the score
            if self.smoothed_score is None:
                self.smoothed_score = best["score"]
            else:
                self.smoothed_score = (
                    self.ema_alpha * best["score"] + (1 - self.ema_alpha) * self.smoothed_score
                )

            # Holding decision: require min fingertip contacts AND palm distance AND smoothed score
            holding_now = (
                best["palm_dist"] < self.palm_distance_threshold
                and best["tip_contacts"] >= self.min_fingertip_contacts
                and self.smoothed_score > 0.6
            )

            # If in open cycle, only close after min duration and release frames
            if self.is_in_open_cycle:
                if time.time() - self.last_open_time >= self.min_open_duration:
                    if not holding_now:
                        self.release_frames_count += 1
                        if self.release_frames_count >= self.release_frames_threshold:
                            self.get_logger().info(
                                f"Hands released after min duration. Score={self.smoothed_score:.2f}, "
                                f"palm_dist={best['palm_dist']:.2f}, tip_contacts={best['tip_contacts']}"
                            )
                            self.send_serial_signal("close")
                            self.is_in_open_cycle = False
                            self.last_open_time = None
                            self.last_close_time = time.time()
                            self.is_holding = False
                            self.holding_frames_count = 0
                            self.release_frames_count = 0
                            self.smoothed_score = None
                    else:
                        self.release_frames_count = 0
                return

            # Not in open cycle: detect holding
            if holding_now:
                self.holding_frames_count += 1
                self.release_frames_count = 0
                if self.holding_frames_count >= self.holding_frames_threshold and not self.is_holding:
                    self.is_holding = True
                    self.is_in_open_cycle = True
                    self.last_open_time = time.time()
                    self.get_logger().info(
                        f"Hand holding DETECTED (score={self.smoothed_score:.2f}, "
                        f"palm_dist={best['palm_dist']:.2f}, tip_contacts={best['tip_contacts']}). Sending 'open'."
                    )
                    self.send_serial_signal("open")
            else:
                self.holding_frames_count = 0
                self.release_frames_count = 0

        except AttributeError as e:
            self.get_logger().warn(f"Could not parse hand data: {e}", throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Exception in hand_lmk_callback: {type(e).__name__}: {e}", throttle_duration_sec=2)


def main(args=None):
    rclpy.init(args=args)
    node = HandHoldingDetectorHandshake()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        if node.serial_port and node.serial_port.is_open:
            node.serial_port.close()
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


if __name__ == "__main__":
    main()


