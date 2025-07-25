# Joy Controller

ROS2の統合型ジョイスティックコントローラーパッケージです。joy_to_twistの機能を統合し、拡張された機能を提供します。

## 機能

### 基本機能

- ジョイスティック入力をTwistメッセージに変換
- 設定可能な入力スケーリング
- デッドゾーン処理
- パラメータの動的変更サポート

### 拡張機能

- **ターボモード**: 高速移動モード (デフォルトで2倍速)
- **精密モード**: 低速精密制御モード (デフォルトで0.3倍速)
- **緊急停止**: ワンボタンで全ての動作を停止
- **安全機能**: 最大速度制限、ジョイスティックタイムアウト検知
- **ボタン状態管理**: プレス、ホールド、リリース状態の詳細な追跡

### 出力トピック

- `/cmd_vel` (geometry_msgs/Twist): 速度コマンド
- `/emergency_stop` (std_msgs/Bool): 緊急停止状態
- `/turbo_mode` (std_msgs/Bool): ターボモード状態
- `/precision_mode` (std_msgs/Bool): 精密モード状態
- `/speed_multiplier` (std_msgs/Float64): 現在の速度倍率

## 使用方法

### ビルド

```bash
cd /home/asahi/shr_core
colcon build --packages-select joy_controller
source install/setup.bash
```

### 起動

```bash
# 基本起動
ros2 launch joy_controller joy_controller.launch.py

# カスタム設定ファイルを使用
ros2 launch joy_controller joy_controller.launch.py config_file:=/path/to/your/config.yaml

# デバッグモードで起動
ros2 run joy_controller joy_controller_node --ros-args -p debug_mode:=true
```

## コントローラーマッピング (デフォルト)

### 軸 (Axes)

- **軸1** (左スティック Y): 前後移動 (linear.x)
- **軸0** (左スティック X): 左右移動 (linear.y) - ホロノミックロボット用
- **軸3** (右スティック X): 回転 (angular.z)

### ボタン (Buttons)

- **ボタン6**: 緊急停止トグル
- **ボタン4** (L1/LB): ターボモードトグル
- **ボタン5** (R1/RB): 精密モードトグル

## パラメータ

### 移動制御パラメータ

- `longitudinal_input_ratio` (default: 1.0): 前後移動のスケーリング
- `lateral_input_ratio` (default: 0.3): 左右移動のスケーリング
- `angular_input_ratio` (default: 1.0): 回転のスケーリング
- `turbo_multiplier` (default: 2.0): ターボモード倍率
- `precision_multiplier` (default: 0.3): 精密モード倍率
- `deadzone_threshold` (default: 0.1): デッドゾーン閾値

### 安全パラメータ

- `enable_safety_checks` (default: true): 安全チェック有効化
- `max_linear_velocity` (default: 2.0): 最大線形速度 [m/s]
- `max_angular_velocity` (default: 2.0): 最大角速度 [rad/s]
- `joy_timeout_duration` (default: 1.0): ジョイスティックタイムアウト [秒]

### デバッグパラメータ

- `debug_mode` (default: false): デバッグログ有効化

## 設定例

```yaml
joy_controller:
  ros__parameters:
    # より遅い動作に設定
    longitudinal_input_ratio: 0.5
    angular_input_ratio: 0.5
    
    # より強力なターボモード
    turbo_multiplier: 3.0
    
    # 安全性を重視した設定
    max_linear_velocity: 1.0
    max_angular_velocity: 1.0
    
    # デバッグモード有効
    debug_mode: true
```

## パラメータの動的変更

実行時にパラメータを変更することができます：

```bash
# 速度比率を変更
ros2 param set /joy_controller longitudinal_input_ratio 0.5

# ターボモード倍率を変更
ros2 param set /joy_controller turbo_multiplier 3.0

# デバッグモードを有効化
ros2 param set /joy_controller debug_mode true
```

## トラブルシューティング

### ジョイスティックが認識されない

```bash
# ジョイスティックデバイスを確認
ls /dev/input/js*

# joyノードのテスト
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0

# joy_controllerのデバッグ
ros2 run joy_controller joy_controller_node --ros-args -p debug_mode:=true
```

### ボタンマッピングの確認

```bash
# ジョイスティック入力の監視
ros2 topic echo /joy
```

## 依存関係

- rclcpp
- sensor_msgs
- geometry_msgs
- std_msgs
- joy

## ライセンス

Apache License 2.0
