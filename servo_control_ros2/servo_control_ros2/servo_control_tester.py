#!/usr/bin/env python3
"""
サーボ制御テストツール（ROS2版）
FEETECH磁気エンコーダ版サーボ用
"""

import serial
import time
import struct
from datetime import datetime

class ServoControlTester:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, servo_id=1):
        self.port = port
        self.baudrate = baudrate
        self.servo_id = servo_id
        self.ser = None
        self.connected = False
        
        # 発見済みレジスタマップ
        self.known_registers = {
            0: {"name": "Firmware main version No", "value": 2009, "type": "read", "description": "ファームウェアメインバージョン番号"},
            1: {"name": "Firmware sub version No", "value": 2005, "type": "read", "description": "ファームウェアサブバージョン番号"},
            2: {"name": "Firmware Release version No", "value": 2025, "type": "read", "description": "ファームウェアリリースバージョン番号"},
            3: {"name": "Firmware Release date", "value": 423, "type": "read", "description": "ファームウェアリリース日"},
            10: {"name": "ID", "value": 10, "type": "read_write", "description": "ID"},
            11: {"name": "Baudrate", "value": 2, "type": "read_write", "description": "ボーレート"},
            12: {"name": "Return Delay Time", "value": 500, "type": "read_write", "description": "リターン遅延時間"},
            128: {"name": "Goal Position", "value": 2129, "type": "read_write", "description": "位置コマンド"},
            129: {"name": "Torque Enable", "value": 1, "type": "read_write", "description": "トルク有効"},
            130: {"name": "Goal Acceleration", "value": 0, "type": "read_write", "description": "目標加速度"},
            131: {"name": "Goal Velocity", "value": 250, "type": "read_write", "description": "目標速度"},
            256: {"name": "Present Position", "value": 2128, "type": "read", "description": "現在位置"},
            257: {"name": "Present Position", "value": 2128, "type": "read", "description": "現在位置"},
            258: {"name": "Present Velocity", "value": 500, "type": "read", "description": "現在速度"},
            259: {"name": "Present PWM", "value": 500, "type": "read", "description": "現在PWM"},
            260: {"name": "Present Input Voltage", "value": 500, "type": "read", "description": "電圧フィードバック"},
            261: {"name": "Present Temperature", "value": 500, "type": "read", "description": "温度フィードバック"},
            262: {"name": "Moving Status", "value": 500, "type": "read", "description": "移動ステータス"},
            263: {"name": "Present Current", "value": 0, "type": "read", "description": "現在電流"},
        }
        
    def calculate_crc16(self, data):
        """Modbus CRC16計算"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc
        
    def create_modbus_command(self, slave_id, function_code, address, value):
        """Modbusコマンド作成（CRC付き）"""
        cmd = struct.pack('>BBHH', slave_id, function_code, address, value)
        crc = self.calculate_crc16(cmd)
        cmd += struct.pack('<H', crc)
        return cmd
        
    def verify_checksum(self, data):
        """チェックサム検証"""
        if len(data) < 3:
            return False
            
        # データ部分（最後の2バイトを除く）
        data_part = data[:-2]
        # 受信したCRC
        received_crc = struct.unpack('<H', data[-2:])[0]
        # 計算したCRC
        calculated_crc = self.calculate_crc16(data_part)
        
        return received_crc == calculated_crc

    def connect(self):
        """シリアル接続"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=2.0
            )
            self.connected = True
            print(f"✓ 接続成功: {self.port} @ {self.baudrate} bps")
            return True
        except Exception as e:
            print(f"✗ 接続失敗: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """接続切断"""
        if self.ser:
            self.ser.close()
            self.connected = False
            print("接続を切断しました")

    def send_command(self, cmd_bytes, description="", expect_response=True):
        """コマンド送信（チェックサム検証付き）"""
        if not self.ser or not self.connected:
            return None
            
        try:
            # バッファクリア
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            # RS485送信制御
            self.ser.rts = True
            time.sleep(0.001)
            
            # コマンド送信
            self.ser.write(cmd_bytes)
            self.ser.flush()
            
            # RS485受信制御
            self.ser.rts = False
            time.sleep(0.001)
            
            if not expect_response:
                return True
                
            # 応答受信
            time.sleep(0.01)
            response = self.ser.read(50)
            
            if response:
                # チェックサム検証
                if self.verify_checksum(response):
                    return response
                else:
                    print(f"チェックサムエラー")
                    return None
            else:
                return None
                
        except Exception as e:
            print(f"通信エラー: {e}")
            return None

    def read_register(self, servo_id, address):
        """レジスタ読み取り"""
        if not self.connected:
            return None

        cmd = self.create_modbus_command(servo_id, 3, address, 1)
        response = self.send_command(cmd)
        if response and len(response) >= 5 and response[1] == 3:
            # レジスタ値を抽出
            value = struct.unpack('>H', response[3:5])[0]
            # レジスタ値を16進数の2bit表記から10進数に変換
            # value = int(value)
            # レジスタ名を取得
            return value
        elif response and response[1] & 0x80:
            return None
        else:
            return None

    def write_register(self, id, address, value):
        """レジスタ書き込み"""
        if not self.connected:
            return False
            
        cmd = self.create_modbus_command(id, 6, address, value)
        response = self.send_command(cmd)
        
        if response and len(response) >= 6:
            # エコーバック確認
            echo_addr = struct.unpack('>H', response[2:4])[0]
            echo_value = struct.unpack('>H', response[4:6])[0]
            
            if echo_addr == address and echo_value == value:
                return True
            else:
                return False
        else:
            return False

    def setPos(self, id,position, enable_torque=True, timeout=5.0):
        """
        位置設定メソッド
        
        Args:
            position (int): 目標位置 (0-4095)
            enable_torque (bool): トルク有効化
            timeout (float): 動作完了待機時間
            
        Returns:
            bool: 設定成功/失敗
        """
        if not self.connected:
            return False
            
        try:
            # 1. トルク有効化
            # if enable_torque:
            #     if not self.write_register(129, 1):  # Torque Enable
            #         return False
                # time.sleep(0.1)
            
            # 2. 位置コマンド送信
            if not self.write_register(id,128, position):  # Goal Position
                return False
            
            # # 3. 動作完了待機
            # start_time = time.time()
            # while time.time() - start_time < timeout:
            #     # 現在位置確認
            #     current_pos = self.read_register(256)  # Present Position
            #     if current_pos is not None:
            #         diff = abs(current_pos - position)
                    
            #         # 目標位置に到達判定（±5の範囲内）
            #         if diff <= 5:
            #             return True
                        
            #     time.sleep(0.1)
                
            return False
            
        except Exception as e:
            print(f"setPos エラー: {e}")
            return False