#!/usr/bin/env python3

import unittest
from unittest.mock import Mock, patch, MagicMock
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# テスト対象のインポート（GPIOエラーを避けるためmockを先に設定）
with patch('esc_motor_control_python.esc_motor_control_node.GPIO_AVAILABLE', False):
    from esc_motor_control_python.esc_motor_control_node import ESCMotorControlNode


class TestESCMotorControlNode(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        """各テスト前の準備"""
        with patch('esc_motor_control_python.esc_motor_control_node.GPIO_AVAILABLE', False):
            self.node = ESCMotorControlNode()
    
    def tearDown(self):
        """各テスト後のクリーンアップ"""
        self.node.destroy_node()
    
    def test_node_initialization(self):
        """ノード初期化テスト"""
        self.assertEqual(self.node.get_name(), 'esc_motor_control_python')
        self.assertEqual(self.node.current_speed, 0.0)
        self.assertFalse(self.node.emergency_stop_active)
        self.assertFalse(self.node.full_speed_active)
    
    def test_parameter_loading(self):
        """パラメータ読み込みテスト"""
        # デフォルトパラメータの確認
        self.assertEqual(self.node.pwm_pin, 13)
        self.assertEqual(self.node.max_speed, 1.0)
        self.assertEqual(self.node.min_speed, -1.0)
        self.assertTrue(self.node.enable_safety_stop)
    
    def test_speed_limits(self):
        """速度制限テスト"""
        # 正常範囲
        self.node.set_motor_speed(0.5)
        self.assertEqual(self.node.current_speed, 0.5)
        
        # 上限超過
        self.node.set_motor_speed(1.5)
        self.assertEqual(self.node.current_speed, 1.0)
        
        # 下限超過
        self.node.set_motor_speed(-1.5)
        self.assertEqual(self.node.current_speed, -1.0)
    
    def test_emergency_stop(self):
        """緊急停止テスト"""
        # 通常動作
        self.node.set_motor_speed(0.8)
        self.assertEqual(self.node.current_speed, 0.8)
        
        # 緊急停止作動
        emergency_msg = Bool()
        emergency_msg.data = True
        self.node.emergency_stop_callback(emergency_msg)
        
        self.assertTrue(self.node.emergency_stop_active)
        
        # 緊急停止中は速度指令を無視
        self.node.set_motor_speed(1.0)
        self.assertEqual(self.node.current_speed, 0.0)
        
        # 緊急停止解除
        emergency_msg.data = False
        self.node.emergency_stop_callback(emergency_msg)
        self.assertFalse(self.node.emergency_stop_active)
    
    def test_joy_callback(self):
        """ジョイスティックコールバックテスト"""
        joy_msg = Joy()
        joy_msg.axes = [0.0, 0.7, 0.0, 0.0]  # 左スティック縦軸 = 0.7
        joy_msg.buttons = [0, 0, 0, 0]  # ボタンは押されていない
        
        self.node.joy_callback(joy_msg)
        self.assertEqual(self.node.current_speed, 0.7)
        
        # フルスピードボタンテスト
        joy_msg.buttons = [0, 1, 0, 0]  # ボタン1が押された
        self.node.joy_callback(joy_msg)
        self.assertTrue(self.node.full_speed_active)
        self.assertEqual(self.node.current_speed, self.node.full_speed_value)
    
    def test_cmd_vel_callback(self):
        """cmd_velコールバックテスト"""
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.6
        
        self.node.cmd_vel_callback(cmd_vel_msg)
        self.assertEqual(self.node.current_speed, 0.6)
    
    def test_speed_callback(self):
        """直接速度指令コールバックテスト"""
        speed_msg = Float32()
        speed_msg.data = -0.3
        
        self.node.speed_callback(speed_msg)
        self.assertEqual(self.node.current_speed, -0.3)


if __name__ == '__main__':
    unittest.main()
