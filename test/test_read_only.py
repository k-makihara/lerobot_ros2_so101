"""Mock bus tests: no serial port is opened by this test."""
import importlib
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch


class ReadOnlyTests(unittest.TestCase):
    def setUp(self):
        try:
            self.module = importlib.import_module('lerobot_ros2_so101.arm.FollowerArm')
        except ImportError:
            self.skipTest('LeRobot SDK is available in the ROS container')
        self.arm = self.module.FollowerArm.__new__(self.module.FollowerArm)
        self.arm.port = '/not-a-real-device'
        self.arm.calibration = {'shoulder_pan': SimpleNamespace(
            id=1, homing_offset=10, range_min=200, range_max=3800)}
        self.bus = MagicMock()
        self.bus.read_calibration.return_value = self.arm.calibration

    def test_read_only_makes_no_writes(self):
        with patch.object(self.module, 'FeetechMotorsBus', return_value=self.bus):
            self.arm.connect(read_only=True)
        self.assertEqual([c[0] for c in self.bus.mock_calls], ['connect', 'read_calibration'])
        with self.assertRaises(RuntimeError):
            self.arm.write_goal_position({'shoulder_pan': 2048})
        self.bus.write.assert_not_called()

    def test_mismatch_closes_without_changing_torque(self):
        self.bus.read_calibration.return_value = {'shoulder_pan': SimpleNamespace(
            id=1, homing_offset=999, range_min=200, range_max=3800)}
        with patch.object(self.module, 'FeetechMotorsBus', return_value=self.bus):
            with self.assertRaises(ValueError):
                self.arm.connect(read_only=True)
        self.bus.disconnect.assert_called_once_with(disable_torque=False)
        self.bus.write.assert_not_called()
        self.bus.disable_torque.assert_not_called()


if __name__ == '__main__':
    unittest.main()
