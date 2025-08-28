#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np

class FuzzyPathPlanner(Node):
    def __init__(self):
        super().__init__('fuzzy_path_planner_node')
        self.rect_obj_position=[0,0]
        self.rect_obj_veocity=[0,0]
        self.current_way_point=[0,0]
        self.position_obstacles=[0,0]
        self.velocity_obstacles=[0,0]


    def attractive_part(self):
        diff = self.current_way_point - self.rect_obj_position
        distance = np.linalg.norm(diff)

        if distance == 0:
            return np.zeros_like(diff)

        # Unit direction vector from leader to goal
        direction = diff / distance
        # Apply a constant magnitude force (capped by max_v_leader)
        F_att = k_att * direction
        F_att = 1*np.clip(F_att,10* -max_v_leader, 10*max_v_leader)     

        return F_att

    
    def repulsive_part(self):

        num_obs=self.position_obs.shape[0]


        # Initialize repulsive forces
        F_rep_p = np.array([np.zeros(2, dtype=np.float64)])
        F_rep_v = np.array([np.zeros(2, dtype=np.float64)])

        # Constants for r_safe calculation (these are find experimentally and should be improved)
        kv = 2
        ka = 1


        for i in range(num_obs): # in this loop we will be calculating cos_delta and threat level to use in repulsive potential
            
            # Calculate relative position and velocity
            p_relative = self.position_obs[i] - self.rect_obj_position
            velocity_relative = self.rect_obj_veocity - self.velocity_obstacles[i]
            # Calculate distance to obstacle
            dist_relative = np.linalg.norm(p_relative)
            # Calculate angle between relative position and relative velocity. this angle shows us that the obstacle and our object will be colliding or not. if cos_delta>0
            # then they will be colliding 
            if dist_relative == 0 or np.linalg.norm(velocity_relative) == 0:
                cos_delta = 0.0
            else:
                cos_delta = np.dot(p_relative, velocity_relative.T) / (dist_relative * np.linalg.norm(velocity_relative))
            delta = np.arccos(np.clip(cos_delta, -1, 1))
            # Calculate variable safety radius. the safety radius is the measure that act as on off switch in repulsive force calculation. r_safe=r_min_safe+r_var_safe
            if cos_delta > 0: # if cos_delta>0 -> the obstacle and our object are in dangerous situation -> r=r_safe+r_var_safe
                r_var_safe = (kv / (ka + max_omega_leader)) * np.linalg.norm(velocity_relative) * np.cos(delta) 
                # kv and ka are constants. max omega leader is in the denominator which is true. relative velocity and cos_delta have direct impact on r_var safe
                r_safe = r_safe_min + r_var_safe
            else:
                r_safe = r_safe_min
            # Calculate threat level
            if cos_delta > 0 and dist_relative > 0: # threat level is another factor that improve our dynamic collision avoidance
                TH_level = (1/dist_relative - 1/r_safe) * np.linalg.norm(velocity_relative) * np.cos(delta) # this factor has inverse relation with dis_relative and
                                                                                            #direct relation with norm(velocity) and cos_delta
            else:
                TH_level = 0
            # repulsive force in potential field method has 2 parts. the first part happen because of the dist relative of obstacle and rectangular object.
            # Update forces if in danger zone
            if (dist_relative < r_safe) and (cos_delta > 0):
                # Position-based repulsive force
                F_rep_p += k_rep_p * TH_level * (-p_relative / dist_relative)
                # Velocity-based repulsive force. this is the second part of potential field repulsive force.
                if np.linalg.norm(velocity_relative) > 0:
                    F_rep_v += k_rep_v * TH_level * ((-p_relative/dist_relative) * (1/cos_delta) +velocity_relative/np.linalg.norm(velocity_relative))
        # Total repulsive force
        F_t = F_rep_p + F_rep_v
        return F_t



    def fuzzy_planner(self):

        # Calculate attractive force
        F_att = self.attractive_part()
        F_rep=self.repulsive_part()



        # Total force

        F_total = F_att + F_rep

        # Convert resultant force to velocity commands
        theta_d = np.arctan2(F_total[0][1], F_total[0][0])  # Desired heading angle. this is the angle of potential field force
        v_d = 1*(F_total / 20) # 20 is the number that have reached experimentally (need to be work on)

        # Limit linear velocity
        if np.linalg.norm(v_d) > max_v_leader:
            v_d = (v_d / np.linalg.norm(v_d)) * max_v_leader

        # Calculate angular velocity
        current_theta = np.arctan2(leader_position[0][1], leader_position[0][0])  # Current heading
        omega_d = (theta_d - current_theta) / dt

        # Limit angular velocity
        omega_d = np.sign(omega_d) * min(abs(omega_d), max_omega_leader)

        return v_d, omega_d 









class ImuYawPublisher(Node):
    def __init__(self):
        super().__init__('imu_yaw_publisher')
        self.declare_parameter('imu_topic', 'imu')
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.sub = self.create_subscription(Imu, imu_topic+'/imu', self.imu_callback, 10)
        self.pub = self.create_publisher(Float32, imu_topic+ '/yaw_deg', 10)
        self.get_logger().info(f"Subscribed to IMU topic: {imu_topic+'/imu'}, Publishing {imu_topic} '/yaw_deg' in same namespace.")

    def imu_callback(self, msg):
        q = msg.orientation
        yaw_deg = quaternion_to_yaw_deg(q.x, q.y, q.z, q.w)
        yaw_msg = Float32()
        yaw_msg.data = yaw_deg
        self.pub.publish(yaw_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuYawPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
