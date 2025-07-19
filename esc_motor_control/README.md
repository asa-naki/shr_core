# ESC Motor Control

ROS2パッケージでラズパイを使ってESC（Electronic Speed Controller）経由でブラシ付きモータをPWM制御するためのパッケージです。

## 機能

- **PWM制御**: ラズパイのハードウェアPWMを使用してESCを制御
- **複数の入力ソース**:
  - `cmd_vel` (geometry_msgs/Twist) - ナビゲーションスタックとの互換性
  - `joy` (sensor_msgs/Joy) - ジョイスティック制御
  - `motor_speed` (std_msgs/Float32) - 直接的な速度制御
- **安全機能**:
  - 緊急停止機能
  - 通信タイムアウト時の自動停止
  - 速度制限
- **フィードバック**: 現在のモータ速度をパブリッシュ

## ハードウェア要件

- Raspberry Pi（PWM対応のGPIOピン）
- ESC（Electronic Speed Controller）
- ブラシ付きDCモータ
- 適切な電源

## インストール

1. ワークスペースでパッケージをビルド:

```bash
cd /home/asahi/shr_core
colcon build --packages-select esc_motor_control
source install/setup.bash
```

## 使用方法

### 基本的な起動

```bash
ros2 launch esc_motor_control esc_motor_control.launch.py
```

### パラメータ付きで起動

```bash
ros2 launch esc_motor_control esc_motor_control.launch.py config_file:=/path/to/your/config.yaml
```

### 直接ノードを起動

```bash
ros2 run esc_motor_control esc_motor_control_node --ros-args --params-file src/esc_motor_control/config/esc_motor_control.yaml
```

## トピック

### Subscribed Topics

- `cmd_vel` (geometry_msgs/Twist): 速度指令（linear.xを使用）
- `joy` (sensor_msgs/Joy): ジョイスティック入力
- `motor_speed` (std_msgs/Float32): 直接的な速度指令 (-1.0 ～ 1.0)

### Published Topics

- `motor_feedback` (std_msgs/Float32): 現在のモータ速度

## パラメータ

- `pwm_pin`: PWM出力に使用するGPIOピン番号（デフォルト: 18）
- `pwm_chip`: PWMチップ番号（デフォルト: 0）
- `pwm_channel`: PWMチャンネル番号（デフォルト: 0）
- `pwm_frequency`: PWM周波数 [Hz]（デフォルト: 50）
- `max_speed`: 最大前進速度（デフォルト: 1.0）
- `min_speed`: 最大後進速度（デフォルト: -1.0）
- `full_speed_button`: フル回転ボタン番号（デフォルト: 1, Bボタン）
- `full_speed_value`: フル回転時の速度値（デフォルト: 1.0）
- `enable_safety_stop`: 安全停止機能の有効/無効（デフォルト: true）
- `safety_timeout`: 安全停止タイムアウト [秒]（デフォルト: 1.0）

## ジョイスティック制御

- **Aボタン (Button 0)**: 緊急停止
- **Bボタン (Button 1)**: フル回転（ボタンを押している間だけフル回転、設定可能）
- **右トリガー (Axis 5)**: 前進制御
- **左トリガー (Axis 2)**: 後進制御

**注意**: フル回転ボタンを押している間は通常のトリガー制御は無効になります。ボタンを離すとモータは自動的に停止します。

## PWM設定

このパッケージはLinuxのsysfsインターフェースを使用してPWMを制御します:

- `/sys/class/pwm/pwmchip0/` を使用
- ESC標準のパルス幅: 1ms（後進） ～ 1.5ms（停止） ～ 2ms（前進）

## 配線例

```
Raspberry Pi GPIO 18 (PWM) -> ESC Signal Wire (通常は白/黄色)
Raspberry Pi GND          -> ESC Ground Wire (通常は黒/茶色)
ESC Red Wire              -> モータ用電源 (ESP別電源推奨)
ESC Motor Wires           -> ブラシ付きDCモータ
```

## 注意事項

1. **電源**: ESCとモータには適切な外部電源を使用してください
2. **権限**: PWMアクセスには適切な権限が必要です
3. **安全**: 初回使用時は低速で動作確認を行ってください
4. **ESCキャリブレーション**: 初回使用前にESCのキャリブレーションを行ってください

## トラブルシューティング

### PWMアクセスエラー

```bash
# PWMアクセス権限の確認/設定
sudo chown $USER:$USER /sys/class/pwm/pwmchip0/export
sudo chown $USER:$USER /sys/class/pwm/pwmchip0/unexport
```

### デバイス確認

```bash
# 利用可能なPWMチップの確認
ls /sys/class/pwm/
```

## 開発者向け情報

### コンポーネントとして使用

```cpp
#include "esc_motor_control/esc_motor_control_component.hpp"

auto node = std::make_shared<esc_motor_control::ESCMotorControlComponent>(options);
```

### テスト

```bash
# モータ速度コマンドのテスト
ros2 topic pub /motor_speed std_msgs/msg/Float32 "data: 0.5" --once

# 緊急停止のテスト
ros2 topic pub /motor_speed std_msgs/msg/Float32 "data: 0.0" --once
```
