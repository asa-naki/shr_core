#!/bin/bash

# ESC Motor Control Joy Test Script
# ジョイスティック入力をシミュレートしてESCモータ制御をテストするスクリプト

echo "=== ESC Motor Control Joy Test ==="

# ワークスペースのセットアップ
cd /home/asahi/shr_core
source install/setup.bash

echo "このスクリプトはジョイスティック入力をシミュレートしてESCモータ制御をテストします"
echo "実際のハードウェアに接続している場合は注意してください！"
echo ""

# テスト関数
test_joy_message() {
    local test_name="$1"
    local joy_msg="$2"
    echo "テスト: $test_name"
    echo "送信するJoyメッセージ: $joy_msg"
    ros2 topic pub /joy sensor_msgs/msg/Joy "$joy_msg" --once
    echo "送信完了 - 2秒待機..."
    sleep 2
    echo ""
}

echo "1. 基本的なジョイスティック制御テスト"
echo "----------------------------------------"

# 通常のトリガー制御テスト（右トリガー50%）
test_joy_message "右トリガー50%前進" \
    "header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, axes: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

# 左トリガー制御テスト（左トリガー50%）
test_joy_message "左トリガー50%後進" \
    "header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, axes: [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

# 停止
test_joy_message "ニュートラル（停止）" \
    "header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, axes: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

echo "2. フル回転ボタンテスト"
echo "--------------------"

# フル回転ボタン（Bボタン）テスト
test_joy_message "Bボタン押下（フル回転）" \
    "header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, axes: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0], buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

# フル回転ボタンリリース
test_joy_message "Bボタンリリース（停止）" \
    "header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, axes: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

echo "3. 緊急停止テスト"
echo "----------------"

# 緊急停止ボタン（Aボタン）テスト
test_joy_message "Aボタン押下（緊急停止）" \
    "header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, axes: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0], buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

echo "4. 複合テスト"
echo "------------"

# 右トリガー100%前進
test_joy_message "右トリガー100%前進" \
    "header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, axes: [0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

# 左トリガー100%後進  
test_joy_message "左トリガー100%後進" \
    "header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, axes: [0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

# 最終停止
test_joy_message "最終停止" \
    "header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, axes: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

echo "=== テスト完了 ==="
echo ""
echo "注意:"
echo "- 上記のテストは標準的なPS4コントローラーの軸配置を想定しています"
echo "- 実際のコントローラーでは軸の番号や配置が異なる場合があります"
echo "- リアルタイムでフィードバックを確認するには別ターミナルで以下を実行:"
echo "  ros2 topic echo /motor_feedback"
echo ""
echo "デバッグ用コマンド:"
echo "ros2 topic echo /joy                  # Joyメッセージの確認"
echo "ros2 topic hz /joy                    # Joy入力の頻度確認"
echo "ros2 node info /esc_motor_control     # ノード情報の確認"
