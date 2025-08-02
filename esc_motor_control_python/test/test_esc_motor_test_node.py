#!/usr/bin/env python3

import unittest
from unittest.mock import Mock, patch, MagicMock
import rclpy
import threading
import time
from std_msgs.msg import Float32

from esc_motor_control_python.esc_motor_test_node import ESCMotorTestNode


class TestESCMotorTestNode(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        """各テスト前の準備"""
        with patch('builtins.input', return_value='quit'):
            self.node = ESCMotorTestNode()
    
    def tearDown(self):
        """各テスト後のクリーンアップ"""
        self.node.destroy_node()
    
    def test_node_initialization(self):
        """ノード初期化テスト"""
        self.assertEqual(self.node.get_name(), 'esc_motor_test_node')
        self.assertFalse(self.node.is_running)
        self.assertFalse(self.node.test_running)
    
    def test_parameter_loading(self):
        """パラメータ読み込みテスト"""
        # デフォルトパラメータの確認
        expected_speeds = [0.1, 0.2, 0.3, 0.4, 0.5]
        self.assertEqual(self.node.test_speeds, expected_speeds)
        self.assertEqual(self.node.test_duration, 2.0)
        self.assertEqual(self.node.test_pause, 1.0)
    
    @patch.object(ESCMotorTestNode, 'speed_publisher')
    def test_set_motor_speed(self, mock_publisher):
        """モーター速度設定テスト"""
        mock_publisher.publish = Mock()
        
        # 速度設定
        self.node.set_motor_speed(0.5)
        self.assertTrue(self.node.is_running)
        
        # 停止
        self.node.set_motor_speed(0.0)
        self.assertFalse(self.node.is_running)
        
        # publishが呼ばれたことを確認
        self.assertEqual(mock_publisher.publish.call_count, 2)
    
    @patch.object(ESCMotorTestNode, 'emergency_publisher')
    def test_emergency_stop(self, mock_publisher):
        """緊急停止テスト"""
        mock_publisher.publish = Mock()
        
        # 緊急停止作動
        self.node.emergency_stop(True)
        
        # 緊急停止解除
        self.node.emergency_stop(False)
        
        # publishが呼ばれたことを確認
        self.assertEqual(mock_publisher.publish.call_count, 2)
    
    @patch.object(ESCMotorTestNode, 'set_motor_speed')
    @patch('time.sleep', return_value=None)  # sleepをmock
    def test_run_test_pattern(self, mock_sleep, mock_set_speed):
        """テストパターン実行テスト"""
        # テストパターン実行
        self.node.run_test_pattern()
        
        # 各テスト速度と停止が呼ばれたことを確認
        expected_calls = len(self.node.test_speeds) * 2  # 各速度 + 停止
        self.assertGreaterEqual(mock_set_speed.call_count, expected_calls)
        
        # テスト完了後はtest_runningがFalseになる
        self.assertFalse(self.node.test_running)


if __name__ == '__main__':
    unittest.main()
