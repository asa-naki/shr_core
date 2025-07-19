#!/bin/bash

# ESC Motor Control テストスクリプト
# このスクリプトはラズパイでESCモータ制御をテストするためのものです

echo "=== ESC Motor Control Test ==="

# ワークスペースのセットアップ
cd /home/asahi/shr_core
source install/setup.bash

echo "1. パッケージの確認"
ros2 pkg list | grep esc_motor_control

echo "2. ノードの利用可能な確認"
ros2 pkg executables esc_motor_control

echo "3. インターフェースの確認"
echo "Subscribed topics:"
echo "  - /cmd_vel (geometry_msgs/msg/Twist)"
echo "  - /joy (sensor_msgs/msg/Joy)"
echo "  - /motor_speed (std_msgs/msg/Float32)"
echo ""
echo "Published topics:"
echo "  - /motor_feedback (std_msgs/msg/Float32)"

echo ""
echo "4. 使用例:"
echo "# 基本的な起動"
echo "ros2 launch esc_motor_control esc_motor_control.launch.py"
echo ""
echo "# 直接ノード起動"
echo "ros2 run esc_motor_control esc_motor_control_node --ros-args --params-file src/esc_motor_control/config/esc_motor_control.yaml"
echo ""
echo "# モータ速度テスト (0.5の速度で前進)"
echo "ros2 topic pub /motor_speed std_msgs/msg/Float32 'data: 0.5' --once"
echo ""
echo "# フル回転テスト (最大速度で前進)"
echo "ros2 topic pub /motor_speed std_msgs/msg/Float32 'data: 1.0' --once"
echo ""
echo "# 緊急停止"
echo "ros2 topic pub /motor_speed std_msgs/msg/Float32 'data: 0.0' --once"
echo ""
echo "# Twistメッセージでの制御"
echo "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --once"

echo ""
echo "=== ジョイスティック制御 ==="
echo "- Aボタン (Button 0): 緊急停止"
echo "- Bボタン (Button 1): フル回転 (ボタンを押している間だけ)"
echo "- 右トリガー (Axis 5): 前進制御"
echo "- 左トリガー (Axis 2): 後進制御"
echo "注意: フル回転ボタンを押している間は通常のトリガー制御は無効です"

echo ""
echo "=== 注意事項 ==="
echo "1. 実際のハードウェアで使用する前に設定を確認してください"
echo "2. PWMピンの設定を確認してください (デフォルト: GPIO 18)"
echo "3. 適切な電源とESCの接続を確認してください"
echo "4. 初回使用時は安全のため低速でテストしてください"
echo "5. フル回転ボタンは慎重に使用してください（ボタンを離すまで最大速度で動作）"
