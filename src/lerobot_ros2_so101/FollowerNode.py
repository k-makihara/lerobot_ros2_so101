from .arm.FollowerArm import FollowerArm

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
# URDF limits for the physical SO-101.  LeRobot calibration supplies the
# encoder endpoints; mapping those endpoints onto these ROS joint limits keeps
# MoveIt's joint convention and the follower's physical convention identical.
JOINT_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}


class FollowerNode(Node):
    def __init__(self):
        super().__init__("follower_node")

        self.declare_parameter("port", "")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("joint_states", "/joint_states")
        self.declare_parameter("joint_commands", "/joint_commands")
        self.declare_parameter("state_publish_rate", 50.0)

        port = self.get_parameter("port").get_parameter_value().string_value
        calibration_file = self.get_parameter("calibration_file").get_parameter_value().string_value
        joint_states = (
            self.get_parameter("joint_states").get_parameter_value().string_value
        )
        joint_commands = (
            self.get_parameter("joint_commands").get_parameter_value().string_value
        )
        state_publish_rate = self.get_parameter("state_publish_rate").value
        if state_publish_rate <= 0.0:
            raise ValueError("'state_publish_rate' must be greater than zero")

        self.follower_arm = FollowerArm(port=port, calibration_file=calibration_file)
        self.get_logger().info("Connect the follower arm.")
        self.follower_arm.connect()

        self.get_logger().info(f"Create publisher topic. {joint_states}")
        self.pub_joint_states = self.create_publisher(JointState, joint_states, 10)
        self.get_logger().info(f"Create subscription topic. {joint_commands}")
        self.sub_joint_commands = self.create_subscription(
            JointState, joint_commands, self.joint_commands_callback, 10
        )

        self.timer = self.create_timer(1.0 / state_publish_rate, self.timer_callback)

    def timer_callback(self):
        positions = self.follower_arm.read_present_position()
        positions = {
            name: self.encoder_to_ros(name, value)
            for name, value in positions.items()
            if name in JOINT_LIMITS
        }

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(positions.keys())
        msg.position = list(positions.values())
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = []
        self.pub_joint_states.publish(msg)

    def joint_commands_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        positions = {
            name: self.ros_to_encoder(name, value)
            for name, value in positions.items()
            if name in JOINT_LIMITS
        }

        self.follower_arm.write_goal_position(positions)

    def encoder_to_ros(self, name, encoder_position):
        calibration = self.follower_arm.calibration[name]
        lower, upper = JOINT_LIMITS[name]
        ratio = (encoder_position - calibration.range_min) / (
            calibration.range_max - calibration.range_min
        )
        if calibration.drive_mode:
            ratio = 1.0 - ratio
        ratio = min(1.0, max(0.0, ratio))
        return lower + ratio * (upper - lower)

    def ros_to_encoder(self, name, ros_position):
        calibration = self.follower_arm.calibration[name]
        lower, upper = JOINT_LIMITS[name]
        ratio = (ros_position - lower) / (upper - lower)
        if calibration.drive_mode:
            ratio = 1.0 - ratio
        ratio = min(1.0, max(0.0, ratio))
        return round(
            calibration.range_min + ratio * (calibration.range_max - calibration.range_min)
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
