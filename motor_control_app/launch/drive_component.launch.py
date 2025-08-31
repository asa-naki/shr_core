import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # パッケージのパスを取得
    package_dir = get_package_share_directory('motor_control_app')
    
    # 設定ファイルのパス
    config_file = os.path.join(package_dir, 'config', 'drive_component.yaml')
    
    # Launch引数を定義
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the configuration file'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for motor communication'
    )
    
    # DriveComponentノードを起動
    drive_component_node = Node(
        package='motor_control_app',
        executable='drive_component_node',
        name='drive_component',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'serial_port': LaunchConfiguration('serial_port')
            }
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        config_file_arg,
        serial_port_arg,
        drive_component_node,
    ])
