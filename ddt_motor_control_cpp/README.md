# DDT Motor Control C++

C++のComponent指向で実装されたROS2ノードです。東莞直驱驱动科技有限公司のM15シリーズモーター用に最適化されています。

## 特徴

- **M15シリーズ対応** - M15モーターのデータシートに準拠した実装
- **ROS2 Component架構** - 高性能なComponent指向設計
- **リアルタイムフィードバック** - 温度、電流、速度、角度、故障状態の監視
- **RS-485通信** - M15の通信仕様（115200 bps、CRC8 Maxim）に準拠
- **安全機能** - 過熱保護、故障検出、ウォッチドッグタイマー
- **パラメーター設定** - YAMLファイルによる柔軟な設定管理

## M15モーターの主要仕様

- **速度範囲**: -330 ～ 330 rpm
- **通信**: RS-485 (115200 bps, 8N1)
- **フィードバック**: 電流、速度、角度、温度、故障コード
- **保護機能**: 過電流、過熱、センサー故障、スタリング検出
- **CRC**: CRC8 (Maximポリノミアル)

## ビルド

```bash
cd /home/asahi/workspace/shr_ws/shr_core
colcon build --packages-select ddt_motor_control_cpp
```

## 使用方法

### スタンドアロンノードとして実行

```bash
ros2 run ddt_motor_control_cpp ddt_motor_controller_node
```

### Launchファイルで実行

```bash
# デフォルト設定で実行
ros2 launch ddt_motor_control_cpp ddt_motor_controller.launch.xml

# Componentとして実行
ros2 launch ddt_motor_control_cpp ddt_motor_controller_component.launch.xml

# カスタム設定ファイルで実行
ros2 launch ddt_motor_control_cpp ddt_motor_controller.launch.xml \
  config_file:=ddt_motor_controller_demo.yaml
```

## 設定ファイル

パラメーターは`config/`ディレクトリ内のYAMLファイルで管理されています：

- `ddt_motor_controller.yaml`: デフォルト設定
- `ddt_motor_controller_demo.yaml`: デモ用設定（小型ロボット向け）

### 設定ファイルの例

```yaml
ddt_motor_controller:
  ros__parameters:
    # シリアル通信設定
    serial_port: "/dev/ttyACM0"
    baud_rate: 115200
    
    # 車輪パラメータ
    wheel_radius: 0.1        # 車輪の半径 [m]
    wheel_separation: 0.5    # 左右車輪間の距離 [m]
    
    # モーター設定
    left_motor_id: 1         # 左モーターのID
    right_motor_id: 2        # 右モーターのID
    max_motor_rpm: 100       # モーターの最大RPM
```

## パラメータ

- `serial_port`: シリアルポート (デフォルト: "/dev/ttyACM0")
- `baud_rate`: ボーレート (デフォルト: 115200)
- `wheel_radius`: 車輪半径 [m] (デフォルト: 0.1)
- `wheel_separation`: 車輪間距離 [m] (デフォルト: 0.5)
- `left_motor_id`: 左モーターID (デフォルト: 1)
- `right_motor_id`: 右モーターID (デフォルト: 2)

## トピック

### Subscribe

- `cmd_vel` (geometry_msgs/Twist): 速度指令

### Publish

- `motor_status` (ddt_motor_control/MotorStatus): モーター状態

## 依存関係

- rclcpp
- rclcpp_components
- geometry_msgs
- std_msgs
- ddt_motor_control (メッセージ定義用)
- serial
