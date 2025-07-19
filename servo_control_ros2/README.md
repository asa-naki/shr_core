# servo_control_ros2/README.md

# Servo Control ROS2 Package

このパッケージは、FEETECH磁気エンコーダ版サーボモーターを制御するためのROS2パッケージです。サーボモーターの位置設定やトルク制御を行うための機能を提供します。

## インストール

このパッケージをインストールするには、以下の手順に従ってください。

1. リポジトリをクローンします。

   ```bash
   git clone <repository-url>
   cd servo_control_ros2
   ```

2. 依存関係をインストールします。

   ```bash
   rosdep install -i --from-path src --rosdistro <ros-distro> -y
   ```

3. パッケージをビルドします。

   ```bash
   colcon build
   ```

4. 環境をセットアップします。

   ```bash
   source install/setup.bash
   ```

## 使用方法

### ノードの起動

サーボモーターを制御するためのノードを起動するには、以下のコマンドを実行します。

```bash
ros2 launch servo_control_ros2 servo_launch.py
```

### サービスの使用

サーボモーターの位置を設定するためのサービスを呼び出すことができます。サービスの定義は`srv/SetPosition.srv`にあります。

### インタラクティブモード

インタラクティブな位置設定テストを行うには、以下のコマンドを実行します。

```bash
ros2 run servo_control_ros2 test_setpos_simple.py interactive
```

## テスト

ユニットテストを実行するには、以下のコマンドを使用します。

```bash
colcon test
```

## 注意事項

- サーボモーターの接続が正しいことを確認してください。
- 使用するROS2のバージョンに応じて、依存関係を適切に管理してください。

## ライセンス

このプロジェクトはMITライセンスの下で提供されています。詳細はLICENSEファイルを参照してください。