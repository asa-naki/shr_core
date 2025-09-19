#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch引数の宣言
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='DDTモータのシリアルポート'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='シリアル通信のボーレート'
    )
    
    motor1_id_arg = DeclareLaunchArgument(
        'motor1_id',
        default_value='1',
        description='モーター1のID'
    )
    
    motor2_id_arg = DeclareLaunchArgument(
        'motor2_id',
        default_value='2',
        description='モーター2のID'
    )
    
    max_motor_rpm_arg = DeclareLaunchArgument(
        'max_motor_rpm',
        default_value='1000',
        description='最大モーターRPM'
    )

    # 個別モーター制御ノード
    individual_motor_node = Node(
        package='motor_control_app',
        executable='individual_motor_app',
        name='individual_motor_controller',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'motor1_id': LaunchConfiguration('motor1_id'),
            'motor2_id': LaunchConfiguration('motor2_id'),
            'max_motor_rpm': LaunchConfiguration('max_motor_rpm'),
            'wheel_radius': 0.1,
            'wheel_separation': 0.5,
        }],
        output='screen'
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        motor1_id_arg,
        motor2_id_arg,
        max_motor_rpm_arg,
        individual_motor_node,
    ])
