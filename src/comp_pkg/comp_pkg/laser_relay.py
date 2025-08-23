#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserRelay(Node):
    def __init__(self):
        super().__init__('laser_relay')

        # Subscriber to original lidar topic
        self.sub = self.create_subscription(
            LaserScan,
            '/rect_obj/laser_controller/out',
            self.laser_callback,
            10
        )

        # Publisher to /scan
        self.pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        self.get_logger().info("Laser relay node started: /rect_obj/laser_controller/out → /scan")

    def laser_callback(self, msg: LaserScan):
        # Just republish the message
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
