import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from .servo_control_tester import ServoControlTester

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.tester = ServoControlTester()
        
        # 初期化フラグ
        self.initialized = False
        
        if not self.tester.connect():
            self.get_logger().error("接続に失敗しました")
            # 接続に失敗してもノードは作成するが、機能を無効化
            self.initialized = False
        else:
            self.initialized = True
        
        # サブスクライバーとパブリッシャーを作成
        self.position_subscription = self.create_subscription(
            Int32,
            'servo_position',
            self.position_callback,
            10
        )
        
        self.status_publisher = self.create_publisher(Bool, 'servo_status', 10)
        self.current_position_publisher = self.create_publisher(Int32, 'current_position', 10)
        
        # 定期的に現在位置を公開
        self.timer = self.create_timer(0.1, self.publish_current_position)
        
        if self.initialized:
            self.get_logger().info("サーボノードが起動しました")
        else:
            self.get_logger().warn("サーボノードが起動しましたが、サーボとの接続に失敗しています")

    def position_callback(self, msg):
        if not self.initialized:
            self.get_logger().warn("サーボが初期化されていません。コマンドを無視します。")
            # ステータスを公開（失敗）
            status_msg = Bool()
            status_msg.data = False
            self.status_publisher.publish(status_msg)
            return
            
        self.get_logger().info(f"位置 {msg.data} に移動中...")
        success = self.tester.setPos(msg.data, enable_torque=True, timeout=3.0)
        
        # ステータスを公開
        status_msg = Bool()
        status_msg.data = success
        self.status_publisher.publish(status_msg)
        
        if success:
            self.get_logger().info(f"移動成功")
        else:
            self.get_logger().error("移動失敗")
    
    def publish_current_position(self):
        """現在位置を定期的に公開"""
        if not self.initialized:
            return
            
        try:
            current_pos = self.tester.read_register(256)  # Present Position
            if current_pos is not None:
                pos_msg = Int32()
                pos_msg.data = current_pos
                self.current_position_publisher.publish(pos_msg)
        except Exception as e:
            self.get_logger().warn(f"位置読み取りエラー: {e}")

    def destroy_node(self):
        if hasattr(self, 'tester'):
            self.tester.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    servo_node = None
    try:
        servo_node = ServoNode()
        rclpy.spin(servo_node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down...")
    except Exception as e:
        print(f"予期しないエラーが発生しました: {e}")
    finally:
        if servo_node is not None:
            servo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()