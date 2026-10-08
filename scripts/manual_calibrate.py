#!/usr/bin/env python3
"""Interactive, torque-OFF only calibration. Never writes a motor goal."""
import argparse
import json
import math
import os
import signal
import stat
from pathlib import Path
import statistics
import subprocess
import tempfile
import threading
import time

from lerobot_ros2_so101.joint_calibration import (
    LIMITS, JointCalibration, fingerprint,
)


def ensure_port_free(port):
    port_stat = os.stat(port)
    if not stat.S_ISCHR(port_stat.st_mode):
        raise ValueError('Port must be a serial character device')
    target = port_stat.st_rdev
    for process in Path('/proc').glob('[0-9]*'):
        if process.name == str(os.getpid()):
            continue
        for fd in process.glob('fd/*'):
            try:
                if fd.stat().st_rdev == target:
                    raise RuntimeError(f'Serial device is in use by PID {process.name}. Stop real launch first.')
            except (FileNotFoundError, PermissionError):
                pass


def stable_sample(bus):
    readings = []
    for _ in range(20):
        readings.append(bus.sync_read('Present_Position', normalize=False))
        time.sleep(0.05)
    result = {}
    for name in LIMITS:
        values = [sample[name] for sample in readings]
        if any(not math.isfinite(v) or not 0 <= v <= 4095 for v in values):
            raise ValueError(f'{name}: Invalid encoder reading')
        if max(values) - min(values) > 12:
            raise ValueError(f'{name}: Joint moved during measurement. Support it and retry.')
        result[name] = statistics.median(values)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--calibration-file', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--domain-id', type=int, default=79)
    args = parser.parse_args()
    output = Path(args.output).expanduser()
    if output.exists():
        parser.error('Output already exists; choose a new file (existing calibration is never overwritten).')
    ensure_port_free(args.port)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    import rclpy
    from sensor_msgs.msg import JointState
    from ament_index_python.packages import get_package_share_directory
    from lerobot_ros2_so101.arm.FollowerArm import FollowerArm
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    arm = FollowerArm(args.port, args.calibration_file)  # load only; NOT arm.connect()
    bus = FeetechMotorsBus(port=args.port, calibration=arm.calibration,
                          motors={name: Motor(c.id, 'sts3215', MotorNormMode.RANGE_0_100)
                                  for name, c in arm.calibration.items()})
    rclpy.init()
    node = rclpy.create_node('manual_joint_reference')
    publisher = node.create_publisher(JointState, '/joint_states', 10)
    time.sleep(1)
    if node.count_publishers('/joint_states') > 1:
        raise RuntimeError('Reference domain already has a joint publisher. Use another --domain-id.')
    reference = dict.fromkeys(LIMITS, 0.0)
    stop = threading.Event()

    def publish():
        while not stop.wait(0.1):
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = list(reference)
            msg.position = list(reference.values())
            publisher.publish(msg)

    children = []
    worker = threading.Thread(target=publish, daemon=True)
    torque_disabled = False
    try:
        share = Path(get_package_share_directory('so101_description'))
        description = subprocess.check_output([
            'xacro', str(share / 'urdf/so101.urdf.xacro'),
            'use_sim:=false', 'use_fake_hardware:=true'], text=True)
        # Temporary parameter file avoids shell quoting and command-line limits.
        import yaml
        with tempfile.TemporaryDirectory(prefix='so101-reference-') as directory:
            parameter_file = Path(directory) / 'robot.yaml'
            parameter_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': description}}}))
            children.append(subprocess.Popen(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                                              '--ros-args', '--params-file', str(parameter_file)],
                                             stdout=subprocess.DEVNULL, start_new_session=True))
            children.append(subprocess.Popen(['rviz2', '-d', str(share / 'rviz/display.rviz')],
                                             stdout=subprocess.DEVNULL, start_new_session=True))
            worker.start()
            print('\nRViz is a REFERENCE ONLY: it sends no motor commands. Base is the fixed reference.')
            print('Stop ALL real launches, support the arm, remove objects and keep fingers clear.')
            print('All six motors will be torque-OFF. Do not force end stops. Ctrl-C aborts without saving.')
            if input('Type SUPPORTED to confirm the arm is supported and controllers are stopped: ').strip() != 'SUPPORTED':
                return
            if any(child.poll() is not None for child in children):
                raise RuntimeError('Reference display failed to start; no torque changes made.')
            ensure_port_free(args.port)
            bus.connect()  # SDK handshake only reads identity/firmware
            actual = bus.read_calibration()
            for name, expected in arm.calibration.items():
                for field in ('id', 'homing_offset', 'range_min', 'range_max'):
                    if getattr(actual[name], field) != getattr(expected, field):
                        raise ValueError(f'{name}: motor {field} differs from LeRobot file; aborting without changes.')
            torque_disabled = True  # cleanup also covers a partial disable failure
            bus.disable_torque(num_retry=3)
            result = {'version': 1, 'lerobot_sha256': fingerprint(args.calibration_file),
                      'method': 'single_reference_pose', 'directions_verified': False, 'joints': {}}
            print('\n見本はこの最初の姿勢から変わりません。全関節を見本に合わせてください。')
            print('全関節0は全部水平ではなく、グリッパーも全閉ではありません。')
            print('一姿勢では回転方向を測れないため、従来の符号を仮使用します。保存結果は読み取り専用です。')
            while True:
                input('アーム全体を支え、見本と同じ姿勢で静止させてEnter: ')
                if any(child.poll() is not None for child in children):
                    raise RuntimeError('Reference display closed; aborting calibration.')
                try:
                    zeros = stable_sample(bus)
                    break
                except ValueError as error:
                    print(error)
            result['joints'] = {name: {'zero_ticks': zeros[name],
                                      'sign': -1 if arm.calibration[name].drive_mode else 1}
                                for name in LIMITS}
            JointCalibration(result, args.calibration_file)
            output.parent.mkdir(parents=True, exist_ok=True)
            with output.open('x') as stream:
                json.dump(result, stream, indent=2)
                stream.write('\n')
            print(f'\nSaved: {output}. Torque stays OFF. Secure the arm before releasing it.')
            print('Next: launch with joint_calibration_file:=... read_only:=true to verify RViz against several poses.')
    finally:
        try:
            if bus.is_connected:
                bus.disconnect(disable_torque=torque_disabled)
        except Exception as error:
            print(f'WARNING: disconnect failed: {error}. Keep supporting the arm; check power safely.')
        stop.set()
        if worker.is_alive():
            worker.join(timeout=2)
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\nAborted. Support the arm: torque is not re-enabled.')
