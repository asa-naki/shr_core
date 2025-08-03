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
            10: {"name": "ID", "value": 1, "type": "read_write", "description": "ID"},
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
        
    def calculate_checksum(self, data):
        """サーボプロトコル用チェックサム計算"""
        # ID + 長さ + コマンド + パラメータの合計の最下位1バイトを反転
        checksum = sum(data) & 0xFF
        return (~checksum) & 0xFF
        
    def create_servo_command(self, servo_id, command, parameters=None):
        """サーボコマンド作成（データシート準拠）"""
        if parameters is None:
            parameters = []
        
        # パケット構成: ヘッダ（0xFF 0xFF） | ID | 長さ | コマンド | パラメータ | チェックサム
        length = len(parameters) + 2  # パラメータ数 + 2
        
        # データ部分（チェックサム計算用）
        data = [servo_id, length, command] + parameters
        checksum = self.calculate_checksum(data)
        
        # 完全なパケット
        packet = [0xFF, 0xFF] + data + [checksum]
        return bytes(packet)
        
    def verify_checksum(self, data):
        """チェックサム検証"""
        if len(data) < 6:  # 最小パケットサイズ
            return False
            
        # ヘッダー確認
        if data[0] != 0xFF or data[1] != 0xFF:
            return False
            
        # データ部分（ID、長さ、エラー/コマンド、パラメータ）
        data_part = data[2:-1]
        # 受信したチェックサム
        received_checksum = data[-1]
        # 計算したチェックサム
        calculated_checksum = self.calculate_checksum(data_part)
        
        return received_checksum == calculated_checksum

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
            
            # RS485送信制御（必要に応じて）
            if hasattr(self.ser, 'rts'):
                self.ser.rts = True
                time.sleep(0.001)
            
            # コマンド送信
            self.ser.write(cmd_bytes)
            self.ser.flush()
            
            # RS485受信制御（必要に応じて）
            if hasattr(self.ser, 'rts'):
                self.ser.rts = False
            
            if not expect_response:
                return True
                
            # 応答受信（リターン遅延時間を考慮）
            time.sleep(0.02)  # サーボの応答待機時間を少し長めに
            response = self.ser.read(50)
            
            if response:
                # チェックサム検証
                if self.verify_checksum(response):
                    return response
                else:
                    print(f"チェックサムエラー: {[hex(b) for b in response]}")
                    return None
            else:
                return None
                
        except Exception as e:
            print(f"通信エラー: {e}")
            return None

    def read_register(self, address, servo_id=None):
        """レジスタ読み取り（データシート準拠）"""
        if not self.connected:
            return None
            
        if servo_id is None:
            servo_id = self.servo_id
            
        # READ DATA コマンド (0x02): 開始アドレス(2バイト)、読み出し長(2バイト)
        address_bytes = [(address >> 8) & 0xFF, address & 0xFF]
        length_bytes = [0x00, 0x02]  # 2バイト読み出し
        parameters = address_bytes + length_bytes
        
        cmd = self.create_servo_command(servo_id, 0x02, parameters)
        response = self.send_command(cmd)
        
        if response and len(response) >= 8:
            # 応答パケット: FF FF ID 長さ エラーステータス パラメータ チェックサム
            if response[4] == 0:  # エラーステータス = 0（正常）
                # パラメータ部分から値を取得（2バイト）
                if len(response) >= 8:
                    value = (response[5] << 8) | response[6]
                    return value
                    
        return None

    def write_register(self, address, value, servo_id=None):
        """レジスタ書き込み（データシート準拠）"""
        if not self.connected:
            return False
            
        if servo_id is None:
            servo_id = self.servo_id
            
        # WRITE DATA コマンド (0x03): 書き込み開始アドレス(2バイト)、データ(2バイト)
        address_bytes = [(address >> 8) & 0xFF, address & 0xFF]
        value_bytes = [(value >> 8) & 0xFF, value & 0xFF]
        parameters = address_bytes + value_bytes
        
        cmd = self.create_servo_command(servo_id, 0x03, parameters)
        response = self.send_command(cmd)
        
        if response and len(response) >= 6:
            # 応答パケット: FF FF ID 長さ エラーステータス チェックサム
            if response[4] == 0:  # エラーステータス = 0（正常）
                return True
                
        return False

    def setPos(self, position, enable_torque=True, timeout=5.0):
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
            if enable_torque:
                if not self.write_register(129, 1):  # Torque Enable
                    return False
                time.sleep(0.1)
            
            # 2. 位置コマンド送信
            if not self.write_register(128, position):  # Goal Position
                return False
            
            # 3. 動作完了待機
            start_time = time.time()
            while time.time() - start_time < timeout:
                # 現在位置確認
                current_pos = self.read_register(256)  # Present Position
                if current_pos is not None:
                    diff = abs(current_pos - position)
                    
                    # 目標位置に到達判定（±5の範囲内）
                    if diff <= 5:
                        return True
                        
                time.sleep(0.1)
                
            return False
            
        except Exception as e:
            print(f"setPos エラー: {e}")
            return False
            
    def ping(self, servo_id=None):
        """PING - サーボの状態照会"""
        if not self.connected:
            return False
            
        if servo_id is None:
            servo_id = self.servo_id
            
        cmd = self.create_servo_command(servo_id, 0x01, [])  # PING コマンド
        response = self.send_command(cmd)
        
        if response and len(response) >= 6:
            if response[4] == 0:  # エラーステータス = 0（正常）
                return True
                
        return False
    
    def sync_write(self, address, servo_data_list):
        """
        SYNC WRITE - 複数サーボへの同時書き込み
        
        Args:
            address (int): 書き込み開始アドレス
            servo_data_list (list): [(servo_id, value1, value2, ...), ...] のリスト
            
        Returns:
            bool: 送信成功/失敗
        """
        if not self.connected or not servo_data_list:
            return False
            
        # データ長を最初のサーボのデータから計算
        data_length = len(servo_data_list[0]) - 1  # servo_idを除く
        
        # パラメータ構築: 開始アドレス(2バイト) + データ長(1バイト) + 各サーボのデータ
        parameters = []
        parameters.extend([(address >> 8) & 0xFF, address & 0xFF])  # 開始アドレス
        parameters.append(data_length * 2)  # データ長（バイト数）
        
        # 各サーボのデータ
        for servo_data in servo_data_list:
            servo_id = servo_data[0]
            values = servo_data[1:]
            
            parameters.append(servo_id)
            for value in values:
                parameters.extend([(value >> 8) & 0xFF, value & 0xFF])
        
        cmd = self.create_servo_command(0xFE, 0x83, parameters)  # SYNC WRITE, ブロードキャストID
        response = self.send_command(cmd, expect_response=False)  # 応答なし
        
        return response is not None
    
    def action(self):
        """ACTION - REG WRITEの実行トリガー"""
        if not self.connected:
            return False
            
        cmd = self.create_servo_command(0xFE, 0x05, [])  # ACTION コマンド、ブロードキャストID
        response = self.send_command(cmd, expect_response=False)  # 応答なし
        
        return response is not None
    
    def reg_write(self, address, value, servo_id=None):
        """REG WRITE - 非同期書き込み（後で実行）"""
        if not self.connected:
            return False
            
        if servo_id is None:
            servo_id = self.servo_id
            
        # REG WRITE コマンド (0x04): 書き込み開始アドレス(2バイト)、データ(2バイト)
        address_bytes = [(address >> 8) & 0xFF, address & 0xFF]
        value_bytes = [(value >> 8) & 0xFF, value & 0xFF]
        parameters = address_bytes + value_bytes
        
        cmd = self.create_servo_command(servo_id, 0x04, parameters)
        response = self.send_command(cmd)
        
        if response and len(response) >= 6:
            if response[4] == 0:  # エラーステータス = 0（正常）
                return True
                
        return False
    
    def setPos_multi(self, servo_positions, enable_torque=True, timeout=5.0):
        """
        複数サーボの位置設定（SYNC WRITE使用）
        
        Args:
            servo_positions (dict): {servo_id: position, ...}
            enable_torque (bool): トルク有効化
            timeout (float): 動作完了待機時間
            
        Returns:
            bool: 設定成功/失敗
        """
        if not self.connected or not servo_positions:
            return False
            
        try:
            # 1. トルク有効化（各サーボ個別）
            if enable_torque:
                for servo_id in servo_positions.keys():
                    if not self.write_register(129, 1, servo_id):  # Torque Enable
                        print(f"サーボID {servo_id} のトルク有効化に失敗")
                        return False
                time.sleep(0.1)
            
            # 2. 位置コマンドをSYNC WRITEで送信
            servo_data_list = []
            for servo_id, position in servo_positions.items():
                servo_data_list.append((servo_id, position))
            
            if not self.sync_write(128, servo_data_list):  # Goal Position
                print("SYNC WRITE失敗")
                return False
            
            # 3. 動作完了待機（全サーボ）
            start_time = time.time()
            while time.time() - start_time < timeout:
                all_reached = True
                
                for servo_id, target_pos in servo_positions.items():
                    current_pos = self.read_register(256, servo_id)  # Present Position
                    if current_pos is None:
                        continue
                        
                    diff = abs(current_pos - target_pos)
                    if diff > 5:  # 目標位置に到達していない
                        all_reached = False
                        break
                
                if all_reached:
                    return True
                    
                time.sleep(0.1)
                
            print("タイムアウト: 一部のサーボが目標位置に到達していません")
            return False
            
        except Exception as e:
            print(f"setPos_multi エラー: {e}")
            return False
    
    def scan_servos(self, id_range=(1, 10)):
        """
        指定範囲のサーボIDをスキャンして接続されているサーボを検出
        
        Args:
            id_range (tuple): (開始ID, 終了ID)
            
        Returns:
            list: 検出されたサーボIDのリスト
        """
        found_servos = []
        
        print(f"サーボスキャン中 (ID {id_range[0]} - {id_range[1]})...")
        
        for servo_id in range(id_range[0], id_range[1] + 1):
            if self.ping(servo_id):
                found_servos.append(servo_id)
                print(f"✓ サーボID {servo_id} 検出")
            else:
                print(f"  サーボID {servo_id} 未検出")
            time.sleep(0.1)
        
        return found_servos
    
    def demo_multi_servo(self):
        """複数サーボ制御のデモ"""
        if not self.connected:
            print("接続されていません")
            return False
            
        # サーボスキャン
        servos = self.scan_servos()
        if not servos:
            print("サーボが見つかりません")
            return False
            
        print(f"検出されたサーボ: {servos}")
        
        # 複数サーボでの動作デモ
        positions = [2048, 1024, 3072, 2048]  # 中央、左、右、中央
        
        for i, pos in enumerate(positions):
            print(f"\n位置 {i+1}: {pos}")
            
            # 全サーボを同じ位置に移動
            servo_positions = {servo_id: pos for servo_id in servos}
            
            if self.setPos_multi(servo_positions, timeout=3.0):
                print("✓ 移動完了")
            else:
                print("✗ 移動失敗")
                
            time.sleep(2.0)
        
        return True


if __name__ == "__main__":
    # テスト実行例
    tester = ServoControlTester(port='/dev/ttyUSB0', baudrate=115200, servo_id=1)
    
    try:
        # 接続
        if tester.connect():
            print("=== サーボ制御テスト ===")
            
            # 単体サーボテスト
            print("\n1. 単体サーボテスト")
            if tester.ping():
                print("✓ PING成功")
                
                # 位置読み取り
                pos = tester.read_register(256)  # Present Position
                if pos is not None:
                    print(f"現在位置: {pos}")
                
                # 位置設定
                if tester.setPos(2048):  # 中央位置
                    print("✓ 位置設定成功")
                else:
                    print("✗ 位置設定失敗")
            else:
                print("✗ PING失敗")
            
            # 複数サーボテスト
            print("\n2. 複数サーボテスト")
            tester.demo_multi_servo()
            
    except KeyboardInterrupt:
        print("\n中断されました")
    finally:
        tester.disconnect()