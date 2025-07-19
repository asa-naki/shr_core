#!/bin/bash

# Manual Joy Message Test Script
# このスクリプトはjoyメッセージを手動で送信してESCモータ制御をテストします

echo "=== Manual Joy Message Test ==="

# ワークスペースのセットアップ
cd /home/asahi/shr_core
source install/setup.bash

echo "ESCモータ制御ノードをバックグラウンドで起動中..."

# ESCモータ制御ノードを起動（バックグラウンド）
ros2 run esc_motor_control esc_motor_control_node --ros-args --params-file src/esc_motor_control/config/esc_motor_control.yaml &
ESC_PID=$!

# ノードの起動を待つ
sleep 3

echo "ノードが起動しました (PID: $ESC_PID)"
echo ""

# フィードバックをモニタ
echo "モータフィードバックをモニタリング開始..."
ros2 topic echo /motor_feedback &
ECHO_PID=$!

sleep 2

echo ""
echo "=== Joy Message Tests ==="

echo "1. 緊急停止ボタンテスト (Button 0)"
ros2 topic pub /joy sensor_msgs/msg/Joy "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
axes: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
buttons: [1, 0, 0, 0, 0, 0, 0, 0]" --once

sleep 2

echo ""
echo "2. フル回転ボタンテスト (Button 1 - 押下)"
ros2 topic pub /joy sensor_msgs/msg/Joy "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
axes: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
buttons: [0, 1, 0, 0, 0, 0, 0, 0]" --once

sleep 3

echo ""
echo "3. フル回転ボタンリリーステスト (Button 1 - 離す)"
ros2 topic pub /joy sensor_msgs/msg/Joy "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
axes: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0]" --once

sleep 2

echo ""
echo "4. 右トリガーテスト (Axis 5 - 半分押し)"
ros2 topic pub /joy sensor_msgs/msg/Joy "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
axes: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0]" --once

sleep 2

echo ""
echo "5. 右トリガーフル押しテスト (Axis 5 - フル押し)"
ros2 topic pub /joy sensor_msgs/msg/Joy "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
axes: [0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0]" --once

sleep 2

echo ""
echo "6. 左トリガーテスト (Axis 2 - 後進)"
ros2 topic pub /joy sensor_msgs/msg/Joy "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
axes: [0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0]" --once

sleep 2

echo ""
echo "7. ニュートラル状態テスト"
ros2 topic pub /joy sensor_msgs/msg/Joy "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
axes: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0]" --once

sleep 2

echo ""
echo "8. フル回転 + トリガー排他テスト (Button 1 + Axis 5)"
ros2 topic pub /joy sensor_msgs/msg/Joy "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
axes: [0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0]
buttons: [0, 1, 0, 0, 0, 0, 0, 0]" --once

sleep 3

echo ""
echo "テスト完了！最終的に停止..."
ros2 topic pub /joy sensor_msgs/msg/Joy "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
axes: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
buttons: [1, 0, 0, 0, 0, 0, 0, 0]" --once

sleep 2

# プロセスを終了
echo ""
echo "テスト終了 - プロセスを終了します"
kill $ECHO_PID 2>/dev/null
kill $ESC_PID 2>/dev/null

echo "=== Manual Joy Test Complete ==="
