#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

class XHatToCmdNode(Node):
    def __init__(self):
        super().__init__('x_hat_to_cmd_node')

        # List of robot names
        self.robot_names = ['robot0_1', 'robot1_0', 'robot1_1']

        # Dictionary to store latest v_hat values and yaw_hat values
        self.v_hat_values = {name: 0.0 for name in self.robot_names}
        self.yaw_hat_values = {name: 0.0 for name in self.robot_names}
        self.current_orientation = {name: 0.0 for name in self.robot_names}

        # Create subscriptions and publishers
        self.cmd_publishers = {}
        for name in self.robot_names:
            self.create_subscription(Float32, f'/{name}/v_hat', self.make_v_hat_callback(name), 10)
            self.create_subscription(Float32, f'/{name}/yaw_hat', self.make_yaw_hat_callback(name), 10)
            self.create_subscription(Float32, f'/{name}/yaw_deg', self.make_current_orientation_callback(name), 10)

            self.cmd_publishers[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)

        # Timer to publish at fixed rate
        self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def make_v_hat_callback(self, robot_name):
        def callback(msg):
            self.v_hat_values[robot_name] = msg.data
        return callback
    
    def make_yaw_hat_callback(self, robot_name):
        def callback(msg):
            self.yaw_hat_values[robot_name] = msg.data
        return callback
    
    def make_current_orientation_callback(self, robot_name):
        def callback(msg):
            self.current_orientation[robot_name] = msg.data # its on degree (-180 to 180) degree
        return callback

    def timer_callback(self):
        for name in self.robot_names:
            cmd_msg = Twist()
            cmd_msg.linear.x = self.v_hat_values[name]
            cmd_msg.angular.z = self.calculate_angular_z(self.yaw_hat_values[name], self.current_orientation[name])


            # All other fields remain zero
            self.cmd_publishers[name].publish(cmd_msg)

    def calculate_angular_z(self, desired_orientation, current_orientation):

        current_orientation=(current_orientation/180)*math.pi
        kp=0.5 #proportional gain
        diff = ((current_orientation - desired_orientation + math.pi) % (2*math.pi)) - math.pi
        angular_z = kp * diff
        # Optional: Limit maximum speed to avoid excessive rotation
        max_speed = 0.5
        angular_z = max(-max_speed, min(max_speed, angular_z))

        return -angular_z  # negative because positive diff means need to rotate negative direction

def main(args=None):
    rclpy.init(args=args)
    node = XHatToCmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
