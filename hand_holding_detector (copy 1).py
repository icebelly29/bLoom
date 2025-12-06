#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
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
        self.declare_parameter('holding_threshold_pixels', 50.0)
        
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.distance_threshold = self.get_parameter('holding_threshold_pixels').get_parameter_value().double_value

        # --- State ---
        self.is_holding = False
        self.last_detection_time = 0

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


    def hand_lmk_callback(self, msg):
        # We need at least two hands to detect holding
        if len(msg.targets) < 2:
            if self.is_holding:
                self.is_holding = False
                self.get_logger().info("Hands released (less than 2 hands detected).")
            return

        # For simplicity, we'll just use the first two hands detected
        hand1 = msg.targets[0]
        hand2 = msg.targets[1]

        # Ensure both hands have keypoints, and each hand's main point list has at least one point, and that point's sub-point list has at least one actual coordinate.
        if not hand1.points or not hand1.points[0].point or len(hand1.points[0].point) < 1 or \
           not hand2.points or not hand2.points[0].point or len(hand2.points[0].point) < 1:
            return

        # The wrist is typically the first keypoint (index 0) within the nested point list
        # Access x and y from geometry_msgs.msg.Point32, which is inside ai_msgs.msg.Point.point list
        wrist1_x = hand1.points[0].point[0].x
        wrist1_y = hand1.points[0].point[0].y

        wrist2_x = hand2.points[0].point[0].x
        wrist2_y = hand2.points[0].point[0].y
        
        try:
            # Calculate Euclidean distance between the two wrists
            dist = math.sqrt((wrist1_x - wrist2_x)**2 + (wrist1_y - wrist2_y)**2)

            currently_holding = dist < self.distance_threshold

            # State change detection: from NOT holding to HOLDING
            if currently_holding and not self.is_holding:
                self.is_holding = True
                self.get_logger().info(f"Hand holding DETECTED! (Distance: {dist:.2f}px). Sending 'open' to serial.")
                self.send_serial_signal('open\n')

            # State change detection: from HOLDING to NOT holding
            elif not currently_holding and self.is_holding:
                self.is_holding = False
                self.get_logger().info(f"Hands released. (Distance: {dist:.2f}px)")

        except AttributeError:
            self.get_logger().warn(
                f"Could not get x, y from point structure. Wrist1: {wrist1}, Wrist2: {wrist2}",
                throttle_duration_sec=5 # Avoid spamming logs
            )
            self.get_logger().info(f"Hand1 points length: {len(hand1.points) if hand1.points else 0}, Hand2 points length: {len(hand2.points) if hand2.points else 0}")
            return

    def send_serial_signal(self, signal):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(signal.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"Error writing to serial port: {e}")
        else:
            self.get_logger().warn("Serial port not available. Cannot send signal.")

def main(args=None):
    rclpy.init(args=args)
    node = HandHoldingDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port and node.serial_port.is_open:
            node.serial_port.close()
            node.get_logger().info("Serial port closed.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
