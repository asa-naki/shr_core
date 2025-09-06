from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パッケージディレクトリを取得
    pkg_dir = get_package_share_directory('motor_control_app')
    
    # 設定ファイルのパス
    config_file = os.path.join(pkg_dir, 'config', 'shot_config.yaml')
    
    # Launch引数
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for servo communication'
    )
    
    # device_arg = DeclareLaunchArgument(
    #     'device',
    #     default_value='/dev/input/js0',
    #     description='Joystick device'
    # )
    
    fire_button_arg = DeclareLaunchArgument(
        'fire_button',
        default_value='0',
        description='Fire button number'
    )
    
    # joyノード
    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node',
    #     parameters=[{
    #         'device': LaunchConfiguration('device'),
    #         'deadzone': 0.05,
    #         'autorepeat_rate': 20.0
    #     }],
    #     output='screen'
    # )
    
    # shot componentノード
    shot_component_node = Node(
        package='motor_control_app',
        executable='shot_component_node',
        name='shot_component',
        parameters=[
            config_file,
            {
                'port': LaunchConfiguration('port'),
                'fire_button': LaunchConfiguration('fire_button')
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        port_arg,
        # device_arg,
        fire_button_arg,
        # # joy_node,
        shot_component_node
    ])
