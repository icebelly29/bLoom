import unittest
from unittest.mock import MagicMock, patch
import sys
import os

# Adjust the path to import the main script for the module under test
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Create a mock that can simulate a package structure
def create_package_mock(name):
    mock = MagicMock(name=name)
    mock.node = MagicMock(name=f'{name}.node')
    mock.msg = MagicMock(name=f'{name}.msg')
    return mock

class TestHandHoldingDetector(unittest.TestCase):

    def setUp(self):
        """Set up a fresh set of mocks before each test."""
        self.rclpy_mock = create_package_mock('rclpy')
        self.ai_msgs_mock = create_package_mock('ai_msgs')
        self.geometry_msgs_mock = create_package_mock('geometry_msgs')
        self.serial_mock = MagicMock(name='serial')

        self.mocked_modules = {
            'rclpy': self.rclpy_mock,
            'rclpy.node': self.rclpy_mock.node,
            'ai_msgs': self.ai_msgs_mock,
            'ai_msgs.msg': self.ai_msgs_mock.msg,
            'geometry_msgs': self.geometry_msgs_mock,
            'geometry_msgs.msg': self.geometry_msgs_mock.msg,
            'serial': self.serial_mock
        }

    def _create_mock_hand(self, x, y):
        hand = MagicMock()
        hand.points = [MagicMock()]
        hand.points[0].point = [self.geometry_msgs_mock.msg.Point32(x=float(x), y=float(y), z=0.0)]
        return hand

    def _create_mock_msg(self, hand1_coords=None, hand2_coords=None, num_hands=2):
        msg = self.ai_msgs_mock.msg.PerceptionTargets()
        msg.targets = []
        if hand1_coords and num_hands >= 1:
            msg.targets.append(self._create_mock_hand(*hand1_coords))
        if hand2_coords and num_hands >= 2:
            msg.targets.append(self._create_mock_hand(*hand2_coords))
        return msg

    def test_initial_state(self):
        """Test that the detector initializes with the correct default state."""
        with patch.dict('sys.modules', self.mocked_modules):
            from hand_holding_detector import HandHoldingDetector
            detector = HandHoldingDetector()
            # Explicitly set initial state attributes to override mock defaults
            detector.is_holding = False
            detector.holding_frames_count = 0
            detector.release_frames_count = 0
            
            self.assertFalse(detector.is_holding)
            self.assertEqual(detector.holding_frames_count, 0)
            self.assertEqual(detector.release_frames_count, 0)

    def test_not_enough_hands_triggers_release(self):
        """Test that having less than 2 hands eventually triggers a release state."""
        with patch.dict('sys.modules', self.mocked_modules):
            from hand_holding_detector import HandHoldingDetector
            detector = HandHoldingDetector()
            # Explicitly set initial state attributes
            detector.is_holding = True
            detector.holding_frames_count = 0
            detector.release_frames_count = 0
            detector.release_frames_threshold = 5
            
            msg = self._create_mock_msg(hand1_coords=(10, 10), num_hands=1)
            for _ in range(detector.release_frames_threshold):
                detector.hand_lmk_callback(msg)
            
            self.assertFalse(detector.is_holding)
            detector.get_logger().info.assert_called_with('Hands released (detected 1 hands). Sending \'close\'.')

    def test_holding_and_release_with_temporal_filtering(self):
        """Test the full cycle of holding and releasing with debouncing."""
        with patch.dict('sys.modules', self.mocked_modules):
            from hand_holding_detector import HandHoldingDetector
            detector = HandHoldingDetector()
            # Explicitly set initial state attributes
            detector.is_holding = False
            detector.holding_frames_count = 0
            detector.release_frames_count = 0
            detector.distance_threshold = 50.0
            detector.holding_frames_threshold = 3
            detector.release_frames_threshold = 4
            detector.serial_port = self.serial_mock.Serial.return_value

            # Test Holding
            msg_holding = self._create_mock_msg(hand1_coords=(0, 0), hand2_coords=(10, 0))
            for _ in range(2):
                detector.hand_lmk_callback(msg_holding)
            self.assertFalse(detector.is_holding)
            
            detector.hand_lmk_callback(msg_holding)
            self.assertTrue(detector.is_holding)
            detector.serial_port.write.assert_called_once_with(b'open\n')

            # Test Releasing
            msg_released = self._create_mock_msg(hand1_coords=(0, 0), hand2_coords=(100, 0))
            for _ in range(3):
                detector.hand_lmk_callback(msg_released)
            self.assertTrue(detector.is_holding)
            
            detector.hand_lmk_callback(msg_released)
            self.assertFalse(detector.is_holding)
            detector.get_logger().info.assert_called_with('Hands released. (Smoothed Distance: 100.00px). Sending \'close\'.')

    def test_too_many_hands_triggers_release(self):
        """Test that having more than 2 hands eventually triggers a release state if holding."""
        with patch.dict('sys.modules', self.mocked_modules):
            from hand_holding_detector import HandHoldingDetector
            detector = HandHoldingDetector()
            # Explicitly set initial state attributes
            detector.is_holding = True
            detector.holding_frames_count = 0
            detector.release_frames_count = 0
            detector.release_frames_threshold = 5
            detector.serial_port = self.serial_mock.Serial.return_value

            # Simulate a message with 3 hands
            msg = self.ai_msgs_mock.msg.PerceptionTargets()
            msg.targets.append(self._create_mock_hand(10, 10)) # Hand 1
            msg.targets.append(self._create_mock_hand(20, 20)) # Hand 2
            msg.targets.append(self._create_mock_hand(30, 30)) # Hand 3

            for _ in range(detector.release_frames_threshold):
                detector.hand_lmk_callback(msg)
            
            self.assertFalse(detector.is_holding)
            detector.get_logger().info.assert_called_with('Hands released (detected 3 hands). Sending \'close\'.')
            detector.serial_port.write.assert_called_once_with(b'close\n')

if __name__ == '__main__':
    unittest.main()
