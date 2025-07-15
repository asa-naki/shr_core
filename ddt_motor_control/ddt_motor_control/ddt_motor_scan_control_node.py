#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import time

MOTOR_ID = 1
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

MAX_SPEED = 300  # 最大速度
STOP_DISTANCE = 0.2  # 10cm
MAX_DISTANCE = 0.6   # 1m

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

def send_velocity(ser, velocity):
    vel_bytes = struct.pack("<h", int(velocity))
    data_fields = bytearray([
        MOTOR_ID, 0x64, vel_bytes[1], vel_bytes[0],
        0, 0, 0, 0, 0
    ])
    crc = crc8_maxim(data_fields)
    command = data_fields + bytearray([crc])
    ser.write(command)

def set_velocity_mode(ser):
    command = bytearray([
        MOTOR_ID, 0xA0, 0, 0, 0, 0, 0, 0, 0, 0x02
    ])
    ser.write(command)

class MotorScanControllerNode(Node):
    def __init__(self):
        super().__init__('motor_scan_control_node')

        # シリアルポート初期化
        self.ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )
        self.get_logger().info(f"シリアルポートオープン: {SERIAL_PORT}")

        # 速度制御モードに切替
        set_velocity_mode(self.ser)
        time.sleep(0.2)
        self.get_logger().info("速度制御モードに設定しました")

        # /scan 購読
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        # 最小距離を算出
        valid_ranges = [r for r in msg.ranges if r > 0.01 and r < msg.range_max]
        if not valid_ranges:
            distance = MAX_DISTANCE
        else:
            distance = min(valid_ranges)

        # 距離に応じて速度決定
        if distance <= STOP_DISTANCE:
            velocity = 0
        elif distance >= MAX_DISTANCE:
            velocity = MAX_SPEED
        else:
            ratio = (distance - STOP_DISTANCE) / (MAX_DISTANCE - STOP_DISTANCE)
            velocity = int(ratio * MAX_SPEED)

        # モータへ送信
        send_velocity(self.ser, velocity)
        self.get_logger().info(f"距離: {distance:.2f} m, 速度: {velocity} rpm")

    def destroy_node(self):
        # ノード終了時に停止
        send_velocity(self.ser, 0)
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorScanControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

