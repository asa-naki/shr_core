from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パッケージディレクトリを取得
    pkg_dir = get_package_share_directory('motor_control_lib')
    
    # 設定ファイルのパス
    config_file = os.path.join(pkg_dir, 'config', 'servo_config.yaml')
    
    # Launch引数
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for servo communication'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for servo communication'
    )
    
    # サーボ制御ノード
    servo_control_node = Node(
        package='motor_control_lib',
        executable='servo_control_node',
        name='servo_control_node',
        parameters=[
            config_file,
            {
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate')
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        port_arg,
        baudrate_arg,
        servo_control_node
    ])
