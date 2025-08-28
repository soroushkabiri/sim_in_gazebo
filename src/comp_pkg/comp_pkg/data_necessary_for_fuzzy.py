#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped

class DataforFuzzy(Node):
    def __init__(self):
        super().__init__('data_for_fuzzy')
        
        # Subscriber to Gazebo /link_states
        self.subscription = self.create_subscription(
            LinkStates,
            '/link_states',
            self.link_states_callback,
            10
        )

        # Publisher for rect_obj pose
        self.publisher = self.create_publisher(
            PoseStamped,
            '/rect_obj_pose',
            10
        )

        self.get_logger().info("rect_obj_pose_publisher node started")

    def link_states_callback(self, msg: LinkStates):
        try:
            # Find index of rect_obj::base_link
            idx = msg.name.index('rect_obj::base_link')
        except ValueError:
            self.get_logger().warn("rect_obj::base_link not found in /link_states")
            return

        # Get the pose
        pose = msg.pose[idx]

        # Publish as PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'  # Gazebo world frame
        pose_msg.pose = pose

        self.publisher.publish(pose_msg)
        # Optional: log x, y, yaw
        import math
        q = pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.get_logger().info(f"rect_obj_pose: x={pose.position.x:.2f}, y={pose.position.y:.2f}, yaw={math.degrees(yaw):.1f}°")

def main(args=None):
    rclpy.init(args=args)
    node = DataforFuzzy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
