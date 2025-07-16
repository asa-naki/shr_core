import sys
import os
import unittest
from unittest.mock import MagicMock
from servo_control_ros2.servo_control_tester import ServoControlTester

class TestSetPos(unittest.TestCase):

    def setUp(self):
        self.tester = ServoControlTester()
        self.tester.connect = MagicMock(return_value=True)
        self.tester.read_register = MagicMock(side_effect=self.mock_read_register)
        self.tester.setPos = MagicMock(side_effect=self.mock_set_pos)

    def mock_read_register(self, register):
        if register == 256:  # Present Position
            return self.current_position
        elif register == 129:  # Torque Enable
            return 1  # Assume torque is enabled
        return 0

    def mock_set_pos(self, position, enable_torque=True, timeout=3.0):
        self.current_position = position
        return True

    def test_initial_state(self):
        self.current_position = 2048  # Set an initial position
        initial_pos = self.tester.read_register(256)
        torque_status = self.tester.read_register(129)
        self.assertEqual(initial_pos, 2048)
        self.assertEqual(torque_status, 1)

    def test_set_position(self):
        test_positions = [2048, 1024, 3072, 2048]
        for pos in test_positions:
            success = self.tester.setPos(pos, enable_torque=True, timeout=3.0)
            self.assertTrue(success)
            current_pos = self.tester.read_register(256)
            self.assertEqual(current_pos, pos)

if __name__ == '__main__':
    unittest.main()