#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import math
import numpy as np



# this node is for the my_bot package and it make odom to base link for rect_obj


class ImuToOdom(Node):
    def __init__(self):
        super().__init__('imu_to_odom')

        # Subscribers & Publishers
        self.imu_sub = self.create_subscription(Imu, '/rect_obj/imu', self.imu_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'rect_obj/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # State
        self.last_time = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        # Get yaw from quaternion
        q = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        _, _, yaw = tf_transformations.euler_from_quaternion(q)
        self.yaw = yaw

        # Integrate linear acceleration in the IMU frame
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y

        # Rotate to world frame using yaw
        ax_world = ax * math.cos(self.yaw) - ay * math.sin(self.yaw)
        ay_world = ax * math.sin(self.yaw) + ay * math.cos(self.yaw)

        # Subtract gravity (only z, we assume flat ground so ignore z)
        # ax_world, ay_world remain the same

        # Integrate acceleration → velocity
        self.vx += ax_world * dt
        self.vy += ay_world * dt

        # Integrate velocity → position
        self.x += self.vx * dt
        self.y += self.vy * dt

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "rect_obj_odom"
        odom.child_frame_id = "rect_obj_base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = msg.orientation

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = msg.angular_velocity.z

        self.odom_pub.publish(odom)

        print(f"vx: {self.vx:.4f}, vy: {self.vy:.4f}, angular_z: {msg.angular_velocity.z:.4f}")


        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "rect_obj_odom"
        t.child_frame_id = "rect_obj_base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImuToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
