#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct
import time

MOTOR_ID = 1
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

VELOCITY_FORWARD = 100
VELOCITY_REVERSE = -100
VELOCITY_STOP = 0

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

def send_command(ser, command):
    ser.write(command)
    time.sleep(0.05)

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )
        self.get_logger().info("ポートオープン成功: " + SERIAL_PORT)
        self.set_mode_velocity()
        time.sleep(0.2)
        self.state = 0
        self.timer = self.create_timer(1.0, self.control_loop)

    def set_mode_velocity(self):
        data_fields = bytearray([
            MOTOR_ID, 0xA0, 0, 0, 0, 0, 0, 0, 0
        ])
        crc = crc8_maxim(data_fields)
        command = data_fields + bytearray([crc])
        send_command(self.ser, command)
        self.get_logger().info("Velocity modeに設定！")

    def set_velocity(self, velocity):
        vel_bytes = struct.pack("<h", int(velocity))
        data_fields = bytearray([
            MOTOR_ID, 0x64, vel_bytes[0], vel_bytes[1],
            0, 0, 0, 0, 0
        ])
        crc = crc8_maxim(data_fields)
        command = data_fields + bytearray([crc])
        send_command(self.ser, command)
        self.get_logger().info(f"速度指令: {velocity} rpm")

    def control_loop(self):
        if self.state == 0:
            self.set_velocity(VELOCITY_FORWARD)
            self.state = 1
        elif self.state == 1:
            self.set_velocity(VELOCITY_STOP)
            self.state = 2
        elif self.state == 2:
            self.set_velocity(VELOCITY_REVERSE)
            self.state = 3
        elif self.state == 3:
            self.set_velocity(VELOCITY_STOP)
            self.state = 0

    def destroy_node(self):
        self.set_velocity(VELOCITY_STOP)
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

