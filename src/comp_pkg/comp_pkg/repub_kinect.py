#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

class KinectRepublisher(Node):
    def __init__(self):
        super().__init__('kinect_republisher')

        # Publishers
        self.pub_info = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        self.pub_image = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)

        # Subscribers
        self.sub_info = self.create_subscription(
            CameraInfo,
            '/kinect/depth/camera_info',
            self.info_callback,
            10
        )
        self.sub_image = self.create_subscription(
            Image,
            '/kinect/depth/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info("Kinect republisher started.")

    def info_callback(self, msg: CameraInfo):
        self.pub_info.publish(msg)

    def image_callback(self, msg: Image):
        self.pub_image.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinectRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
