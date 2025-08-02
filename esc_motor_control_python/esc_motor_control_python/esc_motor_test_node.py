#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time
import threading
from typing import Optional


class ESCMotorTestNode(Node):
    """
    test_esc.pyをベースにしたROS2 ESCモーターテストノード
    手動制御とテストパターン実行用
    """
    
    def __init__(self):
        super().__init__('esc_motor_test_node')
        
        # パラメータの宣言
        self.declare_parameter('test_pattern_speeds', [0.1, 0.2, 0.3, 0.4, 0.5])
        self.declare_parameter('test_pattern_duration', 2.0)
        self.declare_parameter('test_pattern_pause', 1.0)
        
        # パラメータの取得
        self.test_speeds = self.get_parameter('test_pattern_speeds').get_parameter_value().double_array_value
        self.test_duration = self.get_parameter('test_pattern_duration').get_parameter_value().double_value
        self.test_pause = self.get_parameter('test_pattern_pause').get_parameter_value().double_value
        
        # 内部状態
        self.is_running = False
        self.test_running = False
        self.lock = threading.Lock()
        
        # パブリッシャー
        self.speed_publisher = self.create_publisher(Float32, 'motor_speed', 10)
        self.emergency_publisher = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # サブスクライバー
        self.status_subscription = self.create_subscription(
            Float32,
            'motor_status',
            self.status_callback,
            10
        )
        
        # ユーザーインターフェース開始
        self.user_interface_thread = threading.Thread(target=self.user_interface, daemon=True)
        self.user_interface_thread.start()
        
        self.get_logger().info('ESC Motor Test Node initialized')
        self.print_safety_warning()
        self.print_commands()
    
    def print_safety_warning(self):
        """安全警告表示"""
        self.get_logger().warn('=== ⚠️  安全確認 ⚠️  ===')
        self.get_logger().warn('🚨 重要: 初期化中にモーターが回転する可能性があります！')
        self.get_logger().warn('以下の安全対策を必ず実施してください：')
        self.get_logger().warn('1. プロペラを取り外してください')
        self.get_logger().warn('2. または、モーターを安全に固定してください')
        self.get_logger().warn('3. 人や物がモーター/プロペラから離れていることを確認してください')
        self.get_logger().warn('4. バッテリーを外してください')
        self.get_logger().warn('5. ESCの電源も切ってください')
    
    def print_commands(self):
        """コマンド一覧表示"""
        self.get_logger().info('=== 制御コマンド ===')
        self.get_logger().info('"max" または "m" - 最大回転')
        self.get_logger().info('"stop" または "s" - 停止')
        self.get_logger().info('"test" または "t" - テストパターン実行')
        self.get_logger().info('"emergency" または "e" - 緊急停止')
        self.get_logger().info('"reset" または "r" - 緊急停止解除')
        self.get_logger().info('"quit" または "q" - 終了')
        self.get_logger().info('数値 (例: 0.5) - 指定速度設定')
    
    def status_callback(self, msg: Float32):
        """ステータスコールバック"""
        pass  # 必要に応じてステータス表示
    
    def set_motor_speed(self, speed: float):
        """モーター速度設定"""
        msg = Float32()
        msg.data = speed
        self.speed_publisher.publish(msg)
        
        if speed == 0.0:
            self.is_running = False
            self.get_logger().info('🔴 停止指令送信')
        else:
            self.is_running = True
            self.get_logger().info(f'🟢 速度指令送信: {speed:.3f}')
    
    def emergency_stop(self, activate: bool):
        """緊急停止"""
        msg = Bool()
        msg.data = activate
        self.emergency_publisher.publish(msg)
        
        if activate:
            self.get_logger().warn('🚨 緊急停止指令送信')
        else:
            self.get_logger().info('✅ 緊急停止解除指令送信')
    
    def run_test_pattern(self):
        """テストパターン実行"""
        if self.test_running:
            self.get_logger().warn('テストパターンは既に実行中です')
            return
        
        self.test_running = True
        self.get_logger().info('=== テストパターン実行 ===')
        
        try:
            for i, speed_val in enumerate(self.test_speeds):
                if not self.test_running:  # 中断チェック
                    break
                    
                self.get_logger().info(f'テスト速度 {i+1}/{len(self.test_speeds)}: {speed_val:.1f}')
                self.set_motor_speed(speed_val)
                time.sleep(self.test_duration)
                
                if not self.test_running:  # 中断チェック
                    break
                    
                self.get_logger().info('停止')
                self.set_motor_speed(0.0)
                time.sleep(self.test_pause)
        
        except Exception as e:
            self.get_logger().error(f'テストパターンエラー: {str(e)}')
        
        finally:
            self.get_logger().info('=== テストパターン終了 ===')
            self.set_motor_speed(0.0)
            self.test_running = False
    
    def user_interface(self):
        """ユーザーインターフェース（別スレッド）"""
        while rclpy.ok():
            try:
                status = "🟢 最大回転中" if self.is_running else "🔴 停止中"
                print(f"\n現在の状態: {status}")
                
                if self.test_running:
                    print("テストパターン実行中...")
                    time.sleep(1)
                    continue
                
                inp = input("コマンド入力: ").strip().lower()
                
                if inp in ["max", "m"]:
                    if not self.is_running:
                        print("⚠️  最大回転を開始します！")
                        confirm = input("本当に実行しますか？ (yes/y で実行): ").strip().lower()
                        if confirm in ['yes', 'y']:
                            self.set_motor_speed(1.0)
                        else:
                            print("キャンセルしました")
                    else:
                        print("既に回転中です")
                
                elif inp in ["stop", "s"]:
                    self.set_motor_speed(0.0)
                
                elif inp in ["test", "t"]:
                    if not self.test_running:
                        print("⚠️  テストパターンを実行します")
                        confirm = input("本当に実行しますか？ (yes/y で実行): ").strip().lower()
                        if confirm in ['yes', 'y']:
                            test_thread = threading.Thread(target=self.run_test_pattern)
                            test_thread.start()
                        else:
                            print("キャンセルしました")
                    else:
                        print("テストパターンを中断します")
                        self.test_running = False
                
                elif inp in ["emergency", "e"]:
                    self.emergency_stop(True)
                
                elif inp in ["reset", "r"]:
                    self.emergency_stop(False)
                
                elif inp in ["quit", "q"]:
                    print("終了します...")
                    self.set_motor_speed(0.0)
                    break
                
                else:
                    # 数値入力チェック
                    try:
                        speed = float(inp)
                        if -1.0 <= speed <= 1.0:
                            self.set_motor_speed(speed)
                        else:
                            print("速度は-1.0から1.0の範囲で入力してください")
                    except ValueError:
                        print("使用可能コマンド: max, stop, test, emergency, reset, quit, または数値(-1.0~1.0)")
            
            except (EOFError, KeyboardInterrupt):
                print("\n終了します...")
                self.set_motor_speed(0.0)
                break
            except Exception as e:
                self.get_logger().error(f'ユーザーインターフェースエラー: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = ESCMotorTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+Cが押されました。終了します。')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
