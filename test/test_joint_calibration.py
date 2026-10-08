import json
import math
import tempfile
import unittest
from pathlib import Path

from lerobot_ros2_so101.joint_calibration import (
    LIMITS, TICKS_PER_RADIAN, JointCalibration, fingerprint, fit_joint,
)


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'motor.json'
        self.path.write_text(json.dumps({name: {'range_min': 0, 'range_max': 4095} for name in LIMITS}))
        self.data = {'version': 1, 'lerobot_sha256': fingerprint(self.path),
                     'joints': {name: {'zero_ticks': 2048, 'sign': 1} for name in LIMITS}}

    def test_roundtrip_both_directions(self):
        for sign in (-1, 1):
            for joint in self.data['joints'].values():
                joint['sign'] = sign
            calibration = JointCalibration(self.data, self.path)
            for name, (low, high) in LIMITS.items():
                for angle in (low, 0, high):
                    self.assertAlmostEqual(calibration.to_ros(name, calibration.to_encoder(name, angle)), angle, delta=0.001)

    def test_fingerprint_prevents_stale_calibration(self):
        self.path.write_text('{}')
        with self.assertRaises(ValueError):
            JointCalibration(self.data, self.path)

    def test_single_pose_does_not_claim_verified_directions(self):
        self.data['method'] = 'single_reference_pose'
        calibration = JointCalibration(self.data, self.path)
        self.assertFalse(calibration.directions_verified)
        for name in LIMITS:
            self.assertEqual(calibration.to_ros(name, 2048), 0)

    def test_reject_invalid_commands(self):
        calibration = JointCalibration(self.data, self.path)
        for angle in (float('nan'), float('inf'), -100, 100):
            with self.assertRaises(ValueError):
                calibration.to_encoder('elbow_flex', angle)

    def test_reject_encoder_wrap_and_nan(self):
        calibration = JointCalibration(self.data, self.path)
        for ticks in (-1, 4096, float('nan')):
            with self.assertRaises(ValueError):
                calibration.to_ros('wrist_roll', ticks)
        self.data['joints']['wrist_roll']['zero_ticks'] = 100
        with self.assertRaises(ValueError):
            calibration.to_encoder('wrist_roll', -1)

    def test_motor_endstop_is_not_rescaled(self):
        calibration = JointCalibration(self.data, self.path)
        calibration.motor['elbow_flex']['range_max'] = 2100
        with self.assertRaises(ValueError):
            calibration.to_encoder('elbow_flex', 0.5)
        self.assertAlmostEqual(calibration.to_ros('elbow_flex', 2100), 52 / TICKS_PER_RADIAN)

    def test_fit_and_independent_verification(self):
        for sign in (-1, 1):
            fit = fit_joint(2048, 2048 + sign * 4096 / 18,
                            2048 - sign * 4096 / 36, math.radians(-10))
            self.assertEqual(fit['sign'], sign)
        with self.assertRaises(ValueError):
            fit_joint(2048, 2048, 2048, math.radians(-10))
        with self.assertRaises(ValueError):
            fit_joint(2048, 2048 + 4096 / 18, 2048 + 4096 / 36, math.radians(-10))


if __name__ == '__main__':
    unittest.main()
