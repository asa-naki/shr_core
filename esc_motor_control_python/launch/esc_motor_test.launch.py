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
    
    # ESC Motor Control Node
    esc_motor_control_node = Node(
        package='esc_motor_control_python',
        executable='esc_motor_control_node',
        name='esc_motor_control',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'test_mode': True}  # テスト用は常にテストモード
        ],
        output='screen',
        emulate_tty=True
    )
    
    # ESC Motor Test Node
    esc_motor_test_node = Node(
        package='esc_motor_control_python',
        executable='esc_motor_test_node',
        name='esc_motor_test',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        esc_motor_control_node,
        esc_motor_test_node
    ])
