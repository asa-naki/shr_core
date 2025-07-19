import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('esc_motor_control')

    # Config file path
    config_file = os.path.join(package_dir, 'config', 'esc_motor_control.yaml')

    # Launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the config file'
    )

    # ESC Motor Control Node
    esc_motor_control_node = Node(
        package='esc_motor_control',
        executable='esc_motor_control_node',
        name='esc_motor_control',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        config_arg,
        esc_motor_control_node
    ])
