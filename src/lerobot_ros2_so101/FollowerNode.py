from .arm.FollowerArm import FollowerArm
from .joint_calibration import JointCalibration, LIMITS

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
JOINT_LIMITS = LIMITS


class FollowerNode(Node):
    def __init__(self):
        super().__init__("follower_node")

        self.declare_parameter("port", "")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("joint_calibration_file", "")
        self.declare_parameter("read_only", True)
        self.declare_parameter("joint_states", "/joint_states")
        self.declare_parameter("joint_commands", "/joint_commands")
        self.declare_parameter("state_publish_rate", 50.0)

        port = self.get_parameter("port").get_parameter_value().string_value
        calibration_file = self.get_parameter("calibration_file").get_parameter_value().string_value
        self.joint_calibration = JointCalibration.load(
            self.get_parameter("joint_calibration_file").value, calibration_file)
        self.read_only = self.get_parameter("read_only").value
        if not self.read_only and not self.joint_calibration.directions_verified:
            raise ValueError('Single-pose calibration has unverified directions. Use read_only:=true; motor connection refused.')
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
        self.follower_arm.connect(read_only=self.read_only)

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
        if self.read_only:
            return
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            self.get_logger().error('Malformed joint command rejected')
            return
        positions = dict(zip(msg.name, msg.position))
        try:
            positions = {
                name: self.ros_to_encoder(name, value)
                for name, value in positions.items()
                if name in JOINT_LIMITS
            }
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        self.follower_arm.write_goal_position(positions)

    def encoder_to_ros(self, name, encoder_position):
        return self.joint_calibration.to_ros(name, encoder_position)

    def ros_to_encoder(self, name, ros_position):
        return self.joint_calibration.to_encoder(name, ros_position)


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
