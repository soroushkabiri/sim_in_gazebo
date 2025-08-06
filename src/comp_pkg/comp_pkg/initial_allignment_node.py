#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class InitialAlign(Node):
    def __init__(self):
        super().__init__('initial_align')

        self.robot_topics = ['robot0_0', 'robot0_1', 'robot1_0', 'robot1_1']
        self.subs_yaw = []
        self.cmd_vel_pubs = {}
        self.robot_aligned = {name: False for name in self.robot_topics}  # Track alignment status


        for name in self.robot_topics:
            # Subscribe to each robot's yaw_deg topic
            sub_yaw = self.create_subscription(
                Float32,
                f'/{name}/yaw_deg',
                lambda msg, source=name: self.yaw_callback(msg, source),
                10
            )
            self.subs_yaw.append(sub_yaw)

            # Create a publisher for each robot's cmd_vel
            pub = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.cmd_vel_pubs[name] = pub

        self.get_logger().info("InitialAlign node running, waiting for yaw_deg messages...")

    def yaw_callback(self, msg, robot_name):
        if self.robot_aligned[robot_name]:
            # Already aligned; do nothing
            return
        yaw_deg = msg.data
        yaw_radian=(yaw_deg/180)*math.pi
        target_angle = -math.pi  # Target heading in degrees
        tolerance = (5/180)*math.pi  # Acceptable error in degrees
        Kp = 0.5  # Proportional gain (you can tune this)

        # Log current angle
        self.get_logger().info(f"[{robot_name}] yaw: {yaw_radian}")

        pub = self.cmd_vel_pubs.get(robot_name)
        if pub is None:
            self.get_logger().error(f"No cmd_vel publisher found for {robot_name}")
            return

        # Compute shortest signed angle difference
        diff = ((yaw_radian - target_angle + math.pi) % (2*math.pi)) - math.pi
        twist_msg = Twist()

        if abs(diff) > tolerance:
            # Apply proportional control
            angular_z = Kp * diff

            # Optional: Limit maximum speed to avoid excessive rotation
            max_speed = 0.5
            angular_z = max(-max_speed, min(max_speed, angular_z))

            twist_msg.angular.z = -angular_z  # negative because positive diff means need to rotate negative direction
            self.get_logger().info(f"[{robot_name}] Rotating: error={diff}, angular.z={twist_msg.angular.z}")
        else:
            twist_msg.angular.z = 0.0
            self.robot_aligned[robot_name] = True  # Mark this robot as aligned
            self.get_logger().info(f"[{robot_name}] Aligned to -pi rad, stopping!")
        
        pub.publish(twist_msg)
        # Check if all robots are aligned
        if all(self.robot_aligned.values()):
            self.get_logger().info("All robots aligned. Shutting down node.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitialAlign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
