#!/bin/bash
# SHR Core ランチャーユーティリティスクリプト

set -e

# 色付きメッセージ用関数
print_info() {
    echo -e "\033[1;34m[INFO]\033[0m $1"
}

print_success() {
    echo -e "\033[1;32m[SUCCESS]\033[0m $1"
}

print_error() {
    echo -e "\033[1;31m[ERROR]\033[0m $1"
}

print_warning() {
    echo -e "\033[1;33m[WARNING]\033[0m $1"
}

# ヘルプメッセージ
show_help() {
    echo "SHR Core System Launcher"
    echo "使用方法: $0 [オプション] <コマンド>"
    echo ""
    echo "コマンド:"
    echo "  full               全システムを起動 (デフォルト構成)"
    echo "  navigation         ナビゲーション用システム (LiDAR + モータ + Joy)"
    echo "  servo              サーボ制御システム"
    echo "  test               テスト用最小構成"
    echo "  ddt-motor          DDTモータコントローラのみ"
    echo "  lidar              LiDARのみ"
    echo "  joy                ジョイスティックコントローラのみ"
    echo ""
    echo "オプション:"
    echo "  --help, -h         このヘルプを表示"
    echo "  --sim-time         シミュレーション時間を使用"
    echo "  --rviz             RVizを起動"
    echo "  --no-rviz          RVizを起動しない"
    echo "  --joy-device DEV   ジョイスティックデバイス (デフォルト: /dev/input/js0)"
    echo "  --motor-type TYPE  モータタイプ (ddt|esc, デフォルト: ddt)"
    echo ""
    echo "例:"
    echo "  $0 full --rviz"
    echo "  $0 navigation --motor-type esc"
    echo "  $0 test --joy-device /dev/input/js1"
}

# デフォルト設定
USE_SIM_TIME="false"
ENABLE_RVIZ="false"
JOY_DEVICE="/dev/input/js0"
MOTOR_TYPE="ddt"
COMMAND=""

# 引数の解析
while [[ $# -gt 0 ]]; do
    case $1 in
        --help|-h)
            show_help
            exit 0
            ;;
        --sim-time)
            USE_SIM_TIME="true"
            shift
            ;;
        --rviz)
            ENABLE_RVIZ="true"
            shift
            ;;
        --no-rviz)
            ENABLE_RVIZ="false"
            shift
            ;;
        --joy-device)
            JOY_DEVICE="$2"
            shift 2
            ;;
        --motor-type)
            MOTOR_TYPE="$2"
            shift 2
            ;;
        full|navigation|servo|test|ddt-motor|lidar|joy)
            if [ -n "$COMMAND" ]; then
                print_error "複数のコマンドが指定されています"
                exit 1
            fi
            COMMAND="$1"
            shift
            ;;
        *)
            print_error "不明なオプション: $1"
            show_help
            exit 1
            ;;
    esac
done

# デフォルトコマンド
if [ -z "$COMMAND" ]; then
    COMMAND="full"
fi

# ワークスペースのチェック
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

if [ ! -f "$WS_ROOT/install/setup.bash" ]; then
    print_error "ワークスペースがビルドされていません"
    print_info "以下のコマンドでビルドしてください:"
    print_info "cd $WS_ROOT && colcon build"
    exit 1
fi

# 環境のセットアップ
print_info "ワークスペース環境をセットアップ中..."
source "$WS_ROOT/install/setup.bash"

# コマンドの実行
case $COMMAND in
    full)
        print_info "全システムを起動しています..."
        ros2 launch "$SCRIPT_DIR/launch/shr_core_launch.xml" \
            use_sim_time:=$USE_SIM_TIME \
            enable_rviz:=$ENABLE_RVIZ \
            joy_device:=$JOY_DEVICE
        ;;
    navigation)
        print_info "ナビゲーションシステムを起動しています..."
        ros2 launch "$SCRIPT_DIR/launch/navigation_launch.xml" \
            use_sim_time:=$USE_SIM_TIME \
            enable_rviz:=$ENABLE_RVIZ \
            joy_device:=$JOY_DEVICE \
            motor_type:=$MOTOR_TYPE
        ;;
    servo)
        print_info "サーボ制御システムを起動しています..."
        ros2 launch "$SCRIPT_DIR/launch/servo_system_launch.xml" \
            use_sim_time:=$USE_SIM_TIME \
            joy_device:=$JOY_DEVICE
        ;;
    test)
        print_info "テスト構成を起動しています..."
        ros2 launch "$SCRIPT_DIR/launch/test_launch.xml" \
            use_sim_time:=$USE_SIM_TIME \
            joy_device:=$JOY_DEVICE
        ;;
    ddt-motor)
        print_info "DDTモータコントローラを起動しています..."
        ros2 launch "$SCRIPT_DIR/launch/ddt_motor_launch.xml" \
            use_sim_time:=$USE_SIM_TIME
        ;;
    lidar)
        print_info "LiDARを起動しています..."
        ros2 launch ydlidar_ros2_driver ydlidar_launch.py
        ;;
    joy)
        print_info "ジョイスティックコントローラを起動しています..."
        ros2 launch joy_controller joy_controller.launch.py \
            use_sim_time:=$USE_SIM_TIME \
            joy_device:=$JOY_DEVICE
        ;;
    *)
        print_error "不明なコマンド: $COMMAND"
        show_help
        exit 1
        ;;
esac
