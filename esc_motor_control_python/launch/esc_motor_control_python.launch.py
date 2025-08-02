#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # パッケージディレクトリ取得
    pkg_dir = get_package_share_directory('esc_motor_control_python')
    
    # 設定ファイルパス
    config_file = os.path.join(pkg_dir, 'config', 'esc_motor_control_python.yaml')
    
    # Launch引数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',
        description='Run in test mode (no actual GPIO)'
    )
    
    # ESC Motor Control Node
    esc_motor_control_node = Node(
        package='esc_motor_control_python',
        executable='esc_motor_control_node',
        name='esc_motor_control',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'test_mode': LaunchConfiguration('test_mode')}
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        test_mode_arg,
        esc_motor_control_node
    ])
