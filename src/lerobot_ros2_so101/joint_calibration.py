"""ROS joint coordinates, independent of LeRobot's normalized joint ranges."""
import hashlib
import json
import math

LIMITS = {
    'shoulder_pan': (-1.91986, 1.91986),
    'shoulder_lift': (-1.74533, 1.74533),
    'elbow_flex': (-1.69, 1.69),
    'wrist_flex': (-1.65806, 1.65806),
    'wrist_roll': (-2.74385, 2.84121),
    'gripper': (-0.174533, 1.74533),
}
TICKS_PER_RADIAN = 4096 / math.tau


def fingerprint(path):
    with open(path) as stream:
        data = json.load(stream)
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


class JointCalibration:
    def __init__(self, data, lerobot_path):
        if data.get('version') != 1 or data.get('lerobot_sha256') != fingerprint(lerobot_path):
            raise ValueError('ROS calibration does not match this LeRobot calibration. Recalibrate.')
        self.joints = data['joints']
        self.directions_verified = data.get('directions_verified', data.get('method') != 'single_reference_pose') is True
        if set(self.joints) != set(LIMITS):
            raise ValueError('All six joints must be calibrated')
        with open(lerobot_path) as stream:
            self.motor = json.load(stream)
        for name, joint in self.joints.items():
            if joint['sign'] not in (-1, 1) or not math.isfinite(joint['zero_ticks']):
                raise ValueError(f'Invalid calibration: {name}')
            if not 0 <= joint['zero_ticks'] <= 4095:
                raise ValueError(f'Zero crosses encoder boundary: {name}')

    @classmethod
    def load(cls, path, lerobot_path):
        if not path:
            raise ValueError('joint_calibration_file is required. Run manual_calibrate.py first.')
        with open(path) as stream:
            return cls(json.load(stream), lerobot_path)

    def to_ros(self, name, ticks):
        if not math.isfinite(ticks) or not 0 <= ticks <= 4095:
            raise ValueError(f'Invalid encoder position: {name}={ticks}')
        joint = self.joints[name]
        return joint['sign'] * (ticks - joint['zero_ticks']) / TICKS_PER_RADIAN

    def to_encoder(self, name, angle):
        lower, upper = LIMITS[name]
        if not math.isfinite(angle) or not lower <= angle <= upper:
            raise ValueError(f'Outside URDF limits: {name}={angle}')
        joint = self.joints[name]
        ticks = round(joint['zero_ticks'] + joint['sign'] * angle * TICKS_PER_RADIAN)
        motor = self.motor[name]
        if not max(0, motor['range_min']) <= ticks <= min(4095, motor['range_max']):
            raise ValueError(f'Outside calibrated encoder range: {name}={ticks}; command rejected')
        return ticks


def fit_joint(zero, positive, verification, verification_angle):
    """Use physical resolution, not a fit of the encoder range to URDF limits."""
    if any(not math.isfinite(v) or not 0 <= v <= 4095 for v in (zero, positive, verification)):
        raise ValueError('Invalid calibration sample')
    if not math.isfinite(verification_angle):
        raise ValueError('Invalid verification angle')
    delta = positive - zero
    if abs(abs(delta) / TICKS_PER_RADIAN - math.radians(20)) > math.radians(5):
        raise ValueError('The +20 degree sample differs by more than 5 degrees (or wraps). Retry.')
    sign = 1 if delta > 0 else -1
    error = sign * (verification - zero) / TICKS_PER_RADIAN - verification_angle
    if abs(error) > math.radians(5):
        raise ValueError('Independent verification differs by more than 5 degrees. Retry.')
    return {'zero_ticks': zero, 'sign': sign, 'verification_error_deg': math.degrees(error)}
