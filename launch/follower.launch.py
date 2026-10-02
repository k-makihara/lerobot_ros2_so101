from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', description='SO-101 follower serial device, e.g. /dev/ttyACM0'),
        DeclareLaunchArgument('calibration_file', description='Follower calibration JSON created by LeRobot'),
        DeclareLaunchArgument('joint_states', default_value='/joint_states'),
        DeclareLaunchArgument('joint_commands', default_value='/joint_commands'),
        Node(
            package='lerobot_ros2_so101',
            executable='follower_node.py',
            name='so101_follower',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'calibration_file': LaunchConfiguration('calibration_file'),
                'joint_states': LaunchConfiguration('joint_states'),
                'joint_commands': LaunchConfiguration('joint_commands'),
            }],
        ),
    ])
