import rclpy
from rclpy.node import Node
from servo_control_ros2.servo_control_tester import ServoControlTester
from servo_control_ros2.srv import SetPosition

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.tester = ServoControlTester()
        
        if not self.tester.connect():
            self.get_logger().error("接続に失敗しました")
            return
        
        self.srv = self.create_service(SetPosition, 'set_position', self.set_position_callback)

    def set_position_callback(self, request, response):
        self.get_logger().info(f"位置 {request.position} に移動中...")
        success = self.tester.setPos(request.position, enable_torque=True, timeout=3.0)
        
        if success:
            current_pos = self.tester.read_register(256)
            self.get_logger().info(f"✅ 移動完了 - 現在位置: {current_pos}")
            response.success = True
            response.current_position = current_pos
        else:
            self.get_logger().error("❌ 移動失敗")
            response.success = False
            response.current_position = self.tester.read_register(256)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.tester.disconnect()
        node.get_logger().info("接続を切断しました")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()