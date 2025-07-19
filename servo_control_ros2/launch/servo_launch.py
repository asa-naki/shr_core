from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo_control_ros2',
            executable='servo_node',
            name='servo_controller',
            output='screen'
        )
    ])