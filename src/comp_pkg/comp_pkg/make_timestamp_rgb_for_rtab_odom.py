import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from rclpy.qos import qos_profile_sensor_data



class SyncRGB(Node):
    def __init__(self):
        super().__init__('sync_rgb')
        self.latest_depth_stamp = None
        self.latest_camera_info = None
        self.latest_depth_camera_info = None

        # Publishers (all remapped under /kinect_fixed)
        self.pub_rgb = self.create_publisher(Image, '/kinect_fixed/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/kinect_fixed/camera_info', 10)
        self.pub_depth = self.create_publisher(Image, '/kinect_fixed/depth/image_raw', 10)
        self.pub_depth_info = self.create_publisher(CameraInfo, '/kinect_fixed/depth/camera_info', 10)
        self.pub_points = self.create_publisher(PointCloud2, '/kinect_fixed/points', 10)

        # Subscribers (original /kinect topics)
        self.sub_rgb = self.create_subscription(Image, '/kinect/image_raw', self.cb_rgb, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/kinect/camera_info', self.cb_info, 10)
        self.sub_depth = self.create_subscription(Image, '/kinect/depth/image_raw', self.cb_depth, 10)
        self.sub_depth_info = self.create_subscription(CameraInfo, '/kinect/depth/camera_info', self.cb_depth_info, 10)
        #self.sub_points = self.create_subscription(PointCloud2, '/kinect/points', self.cb_points, 10)
        self.sub_points = self.create_subscription(PointCloud2, '/kinect/points', self.cb_points, qos_profile_sensor_data)

    def cb_depth(self, msg: Image):
        """Republish depth image with corrected frame_id and save timestamp."""
        self.latest_depth_stamp = msg.header.stamp
        msg.header.frame_id = 'depth_camera_link_optical'
        self.pub_depth.publish(msg)

    def cb_rgb(self, msg: Image):
        """Republish RGB image with depth timestamp and corrected frame_id."""
        if self.latest_depth_stamp:
            msg.header.stamp = self.latest_depth_stamp
        msg.header.frame_id = 'depth_camera_link_optical'
        self.pub_rgb.publish(msg)

        # Also republish RGB camera info if available
        if self.latest_camera_info:
            info = self.latest_camera_info
            info.header.stamp = msg.header.stamp
            info.header.frame_id = 'depth_camera_link_optical'
            self.pub_info.publish(info)

    def cb_info(self, msg: CameraInfo):
        """Store latest RGB camera info."""
        self.latest_camera_info = msg

    def cb_depth_info(self, msg: CameraInfo):
        """Republish depth camera info with corrected frame_id."""
        msg.header.frame_id = 'depth_camera_link_optical'
        self.pub_depth_info.publish(msg)
        self.latest_depth_camera_info = msg

    def cb_points(self, msg: PointCloud2):
        """Republish point cloud with corrected frame_id."""
        msg.header.frame_id = 'depth_camera_link_optical'
        self.pub_points.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyncRGB()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#ros2 run rtabmap_odom rgbd_odometry   --ros-args     -r /rgb/image:=/kinect_fixed/image_raw     -r /depth/image:=/kinect_fixed/depth/image_raw     -r /rgb/camera_info:=/kinect_fixed/camera_info


