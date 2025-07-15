#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import math

class LaserToPointCloudNode(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud_node')

        # LaserScan購読
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # PointCloud2配信用Publisher
        self.publisher_ = self.create_publisher(PointCloud2, '/cloud', 10)
        self.get_logger().info("LaserScan→PointCloud2変換ノード起動！")

    def laser_callback(self, msg):
        points = []

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0
                points.append((x, y, z))
            angle += msg.angle_increment

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id  # 通常 "laser" フレーム

        cloud = pc2.create_cloud_xyz32(header, points)
        self.publisher_.publish(cloud)
        self.get_logger().debug("PointCloud2をPublishしました")

def main(args=None):
    rclpy.init(args=args)
    node = LaserToPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

