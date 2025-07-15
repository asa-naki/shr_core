#!/usr/bin/env python3
# filepath: /home/scramble/ros2_ws/src/ddt_motor_control/ddt_motor_control/differential_drive_controller_node.py

import rclpy
from rclpy.node import Node
import serial
import struct
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from ddt_motor_control.msg import MotorStatus

# モーター設定
LEFT_MOTOR_ID = 1
RIGHT_MOTOR_ID = 2
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# 速度制限
MAX_MOTOR_RPM = 100  # モーターの最大RPM

# 車輪パラメータ (メートル単位)
WHEEL_RADIUS = 0.065  # 車輪の半径 (m)
WHEEL_SEPARATION = 0.3  # 左右車輪間の距離 (m)

def crc8_maxim(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if (crc & 0x01):
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc

def send_command(ser, command, retry_count=3):
    """コマンドを送信し、必要に応じてリトライする"""
    for attempt in range(retry_count):
        try:
            ser.write(command)
            ser.flush()  # 送信バッファを確実にフラッシュ
            time.sleep(0.05)
            return True
        except serial.SerialException as e:
            print(f"シリアル通信エラー (試行 {attempt + 1}): {e}")
            time.sleep(0.1)
    return False

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('differential_drive_controller')
        
        # シリアル通信の初期化
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            # シリアルポートのバッファをクリア
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"シリアルポートが開きました: {SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"シリアルポートの初期化に失敗しました: {e}")
            raise
        
        # 両方のモーターを速度モードに設定
        self.set_mode_velocity(LEFT_MOTOR_ID)
        self.set_mode_velocity(RIGHT_MOTOR_ID)
        time.sleep(0.2)
        
        # Twistメッセージのサブスクライバーを作成
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
        self.get_logger().info("Twistメッセージのサブスクライバーを作成しました")
        
        # MotorStatusメッセージのパブリッシャーを作成
        self.motor_status_pub = self.create_publisher(
            MotorStatus,
            'motor_status',
            10)
        self.get_logger().info("MotorStatusパブリッシャーを作成しました")
        
        # 安全のためのウォッチドッグタイマー
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)
        self.last_twist_time = time.time()
        
        # 現在のモーター速度を保存
        self.left_motor_velocity = 0
        self.right_motor_velocity = 0
        
        # モーター回転量の累積値
        self.left_motor_rotations = 0.0
        self.right_motor_rotations = 0.0
        self.last_status_time = time.time()
        
        # 速度ステータスのログタイマー
        self.status_timer = self.create_timer(1.0, self.log_status)
        
        # モーターステータスの定期パブリッシュ
        self.status_publish_timer = self.create_timer(0.1, self.publish_motor_status)

    def set_mode_velocity(self, motor_id):
        """モーターを速度制御モードに設定"""
        data_fields = bytearray([
            motor_id, 0xA0, 0, 0, 0, 0, 0, 0, 0
        ])
        crc = crc8_maxim(data_fields)
        command = data_fields + bytearray([crc])
        
        success = send_command(self.ser, command)
        if success:
            self.get_logger().info(f"モーター {motor_id} を速度制御モードに設定しました")
        else:
            self.get_logger().error(f"モーター {motor_id} の速度制御モード設定に失敗しました")

    def set_motor_velocity(self, motor_id, velocity_rpm):
        """指定したモーターに速度指令を送信"""
        # 速度を整数に変換して制限
        velocity_int = int(velocity_rpm)
        if velocity_int > MAX_MOTOR_RPM:
            velocity_int = MAX_MOTOR_RPM
        elif velocity_int < -MAX_MOTOR_RPM:
            velocity_int = -MAX_MOTOR_RPM
            
        # 速度バイトを作成
        vel_bytes = struct.pack("<h", velocity_int)
        
        # コマンドを組み立てて送信
        data_fields = bytearray([
            motor_id, 0x64, vel_bytes[0], vel_bytes[1],
            0, 0, 0, 0, 0
        ])
        crc = crc8_maxim(data_fields)
        command = data_fields + bytearray([crc])
        
        success = send_command(self.ser, command)
        if success:
            # モーターIDに応じて現在速度を更新（送信成功時のみ）
            if motor_id == LEFT_MOTOR_ID:
                self.left_motor_velocity = velocity_int
            elif motor_id == RIGHT_MOTOR_ID:
                self.right_motor_velocity = velocity_int
        else:
            self.get_logger().error(f"モーター {motor_id} の速度設定に失敗しました (目標: {velocity_int} RPM)")
        
    def twist_to_motor_velocities(self, linear_x, angular_z):

        # 差動駆動の運動学方程式を使用
        # 左右ホイールの並進速度 (m/s)
        v_left = linear_x - (angular_z * WHEEL_SEPARATION / 2.0)
        v_right = linear_x + (angular_z * WHEEL_SEPARATION / 2.0)
        
        # 並進速度からRPMに変換 (RPM = v / (2π*r) * 60)
        rpm_left = (v_left / (2.0 * 3.14159 * WHEEL_RADIUS)) * 60.0
        rpm_right = -1 * (v_right / (2.0 * 3.14159 * WHEEL_RADIUS)) * 60.0  # 右モーターは逆方向
        
        return rpm_left, rpm_right
    
    def twist_callback(self, msg):
        """Twistメッセージを処理して左右のモーターに適切な速度を設定"""
        # Twistメッセージから線形速度と角速度を取得
        linear_x = msg.linear.x   # m/s
        angular_z = msg.angular.z  # rad/s
        
        # モーター速度に変換
        left_rpm, right_rpm = self.twist_to_motor_velocities(linear_x, angular_z)
        
        # デバッグ情報をログ出力
        self.get_logger().debug(
            f"Twist受信 - 線形: {linear_x:.3f} m/s, 角速度: {angular_z:.3f} rad/s -> "
            f"左: {left_rpm:.1f} RPM, 右: {right_rpm:.1f} RPM"
        )
        
        # 左右のモーターを同期して制御するため、少し間隔を空ける
        self.set_motor_velocity(LEFT_MOTOR_ID, left_rpm)
        time.sleep(0.01)  # 短い間隔でモーター間の同期を改善
        self.set_motor_velocity(RIGHT_MOTOR_ID, right_rpm)
        
        # ウォッチドッグタイマーのリセット
        self.last_twist_time = time.time()
    
    def update_motor_rotations(self):
        """モーターの回転量を更新"""
        current_time = time.time()
        dt = current_time - self.last_status_time
        
        # RPMから1秒間の回転数を計算し、積算
        left_rotations_per_sec = self.left_motor_velocity / 60.0
        right_rotations_per_sec = self.right_motor_velocity / 60.0
        
        self.left_motor_rotations += left_rotations_per_sec * dt
        self.right_motor_rotations += right_rotations_per_sec * dt
        
        self.last_status_time = current_time
    
    def publish_motor_status(self):
        """モーターステータスをパブリッシュ"""
        self.update_motor_rotations()
        
        msg = MotorStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.left_motor_rpm = self.left_motor_velocity
        msg.right_motor_rpm = self.right_motor_velocity
        msg.left_motor_rotations = self.left_motor_rotations
        msg.right_motor_rotations = self.right_motor_rotations
        
        self.motor_status_pub.publish(msg)

    def stop_motors(self):
        """両方のモーターを停止"""
        self.get_logger().info("モーターを停止しています...")
        self.set_motor_velocity(LEFT_MOTOR_ID, 0)
        time.sleep(0.01)  # 左右モーター間の同期のため少し待機
        self.set_motor_velocity(RIGHT_MOTOR_ID, 0)
        
    def watchdog_callback(self):
        """一定時間Twistメッセージがないとモーターを停止"""
        if time.time() - self.last_twist_time > 0.5:
            self.stop_motors()
            self.get_logger().warn("Twistメッセージのタイムアウト: モーターを停止しました")
    
    def log_status(self):
        """現在のモーター速度をログ出力"""
        self.get_logger().info(
            f"モーター状態 - 左: {self.left_motor_velocity} RPM ({self.left_motor_rotations:.2f} 回転), "
            f"右: {self.right_motor_velocity} RPM ({self.right_motor_rotations:.2f} 回転)"
        )

    def destroy_node(self):
        """ノード終了時の処理"""
        self.stop_motors()
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
