#!/bin/bash

# ESC Motor Control Joy Test Runner
# このスクリプトはジョイスティック制御のテストを実行します

echo "=== ESC Motor Control Joy Test Runner ==="

# ワークスペースのセットアップ
cd /home/asahi/shr_core
source install/setup.bash

echo "1. パッケージのビルド（テスト付き）"
colcon build --packages-select esc_motor_control

if [ $? -ne 0 ]; then
    echo "ビルドに失敗しました"
    exit 1
fi

source install/setup.bash

echo ""
echo "2. 全テストの実行"
colcon test --packages-select esc_motor_control

echo ""
echo "3. テスト結果の表示"
colcon test-result --verbose --all

echo ""
echo "4. 個別テストの実行例:"
echo ""
echo "# C++単体テスト"
echo "colcon test --packages-select esc_motor_control --ctest-args tests"
echo ""
echo "# Python統合テスト"
echo "colcon test --packages-select esc_motor_control --pytest-args tests"
echo ""
echo "# 特定のテストのみ実行"
echo "cd build/esc_motor_control && ctest --verbose"

echo ""
echo "5. テストのカバレッジ確認:"
echo "find build/esc_motor_control/Testing -name '*.xml' | xargs cat"

echo ""
echo "=== テスト完了 ==="
