#!/bin/bash
set -e

# ROS2環境のソース
source /opt/ros/${ROS_DISTRO}/setup.bash

# ワークスペースが存在し、buildディレクトリがある場合はソース
if [ -f "/home/ros/robot_ws/install/setup.bash" ]; then
    echo "Sourcing workspace setup..."
    source /home/ros/robot_ws/install/setup.bash
fi

# GPU設定（Raspberry Pi 5のGPU使用時）
if [ -d "/opt/vc/lib" ]; then
    export LD_LIBRARY_PATH=/opt/vc/lib:$LD_LIBRARY_PATH
fi

# I2C/SPI/GPIO デバイスの権限設定（ホストマウント時）
if [ -e "/dev/i2c-1" ]; then
    echo "I2C device detected"
fi

if [ -e "/dev/spidev0.0" ]; then
    echo "SPI device detected"
fi

if [ -d "/sys/class/gpio" ]; then
    echo "GPIO access available"
fi

# ディスプレイ設定（X11フォワーディング用）
if [ -n "$DISPLAY" ]; then
    echo "Display forwarding enabled: $DISPLAY"
fi

# 初回起動時の自動セットアップ
if [ ! -f "/home/ros/.docker_initialized" ]; then
    echo "🚀 Initializing ROS2 Raspberry Pi development environment..."
    echo "📝 Creating sample packages..."
    
    cd /home/ros/robot_ws/src
    
    # サンプルパッケージの作成
    if [ ! -d "robot_teleop" ]; then
        ros2 pkg create robot_teleop --build-type ament_python --dependencies rclpy geometry_msgs sensor_msgs
        echo "✅ robot_teleop package created"
    fi
    
    if [ ! -d "robot_bringup" ]; then
        ros2 pkg create robot_bringup --build-type ament_cmake --dependencies rclcpp std_msgs sensor_msgs
        echo "✅ robot_bringup package created"
    fi
    
    # 初期化完了マーカー
    touch /home/ros/.docker_initialized
    
    # 初回ビルドの実行
    echo "🔨 Building workspace for the first time..."
    cd /home/ros/robot_ws
    colcon build --symlink-install
    echo "✅ Initial build completed!"
    
    echo "🎉 ROS2 Raspberry Pi environment initialized!"
fi

echo "🤖 ROS2 Jazzy development environment ready!"
echo "📁 Workspace: /home/ros/robot_ws"
echo "🔧 Available aliases:"
echo "   rw   - cd to workspace"
echo "   rs   - cd to src"
echo "   rb   - build workspace (with symlink-install)"
echo "   rbd  - build workspace (Debug mode)"
echo "   rbr  - build workspace (Release mode)"
echo "   rbc  - clean build workspace"
echo "   rt   - test workspace"
echo "   rtr  - test result verbose"
echo "   rr   - source workspace"
echo "   cbp  - colcon build --packages-select <package>"
echo "   cbu  - colcon build --packages-up-to <package>"
echo ""
echo "💡 Quick start:"
echo "   rs              # Go to src directory"
echo "   ros2 pkg create my_package --build-type ament_python"
echo "   rb              # Build all packages"
echo "   cbp my_package  # Build specific package"
echo ""

# 引数が渡された場合は実行、そうでなければbashを起動
exec "$@"
