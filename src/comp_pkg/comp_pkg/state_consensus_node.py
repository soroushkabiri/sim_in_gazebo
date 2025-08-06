import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
import math
import matplotlib.pyplot as plt
import time

class ConsensusObserver(Node):
    def __init__(self):
        super().__init__('consensus_observer_followers')

        # Follower Names
        self.follower_names = ['robot0_1', 'robot1_0', 'robot1_1']
        self.num_followers = len(self.follower_names)
        self.step_consensus=10
        
        # Parameters 
        self.alpha_v = 0.5
        self.beta_v = 0.5
        self.gamma_v = 2.0
        self.alpha_yaw = 0.5
        self.beta_yaw = 0.5
        self.gamma_yaw = 2.0

        # Adjacency matrix (size NxN) and leader connectivity vector (size N)
        self.a = np.array([[0, 1, 0],
                           [1, 0, 1],
                           [0, 1, 0]])
        self.b = np.array([1, 0, 0])  # Only first follower connected to leader directly

        # Leader States 
        self.v_leader = 0.0
        self.yaw_leader = -math.pi

        self.create_subscription(Twist, '/robot0_0/cmd_vel', self.leader_vel_callback, 10)
        self.create_subscription(Float32, '/robot0_0/yaw_deg', self.leader_yaw_callback, 10)

        # Initialize observation States 
        # Each follower has: velocity_hat and yaw_hat
        self.v_hat = np.zeros(self.num_followers)
        self.yaw_hat = np.zeros(self.num_followers)-math.pi/2

        # Neighbor Estimates 
        self.neighbor_estimates_v=np.zeros(self.num_followers)
        self.neighbor_estimates_yaw=np.zeros(self.num_followers)

        # Subscribers to Neighbors
        for j, neighbor in enumerate(self.follower_names):
            # Subscribe to neighbor's v_hat
            self.create_subscription(Float32, f'/{neighbor}/v_hat', self.make_neighbor_callback(j, neighbor, 'v'), 10)
            # Subscribe to neighbor's yaw_hat
            self.create_subscription(Float32, f'/{neighbor}/yaw_hat', self.make_neighbor_callback(j, neighbor, 'yaw'), 10)

        # Publishers 
        self.publishers_v = []
        self.publishers_yaw = []

        for name in self.follower_names:
            self.publishers_v.append(self.create_publisher(Float32, f'/{name}/v_hat', 10))
            self.publishers_yaw.append(self.create_publisher(Float32, f'/{name}/yaw_hat', 10))

        # Plotting setup
        self.time_history = []
        self.yaw_history_leader = []
        self.yaw_history_followers = [[] for _ in range(self.num_followers)]
        self.start_time = time.time()

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10,6))

        # Timer 
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz


    def leader_vel_callback(self, msg):
        self.v_leader = msg.linear.x

    def leader_yaw_callback(self, msg):
        self.yaw_leader = msg.data

    def make_neighbor_callback(self, j,neighbor, var_type):
        def callback(msg):
            if var_type == 'v':
                self.neighbor_estimates_v[j] = msg.data
            elif var_type == 'yaw':
                self.neighbor_estimates_yaw[j] = msg.data
        return callback

    def timer_callback(self):
        current_time = time.time() - self.start_time

        estimate_v=self.neighbor_estimates_v
        estimate_yaw=self.neighbor_estimates_yaw

        for k in range(self.step_consensus):
            for i, follower in enumerate(self.follower_names):
                # Velocity consensus update
                self.v_hat[i] = self.observer_update(i, estimate_v, self.v_leader, self.v_hat[i],'v')
                # Yaw consensus update
                self.yaw_hat[i] = self.observer_update(i, estimate_yaw, (self.yaw_leader/180)*math.pi, self.yaw_hat[i],'yaw')

            estimate_v=self.v_hat
            estimate_yaw=self.yaw_hat

        for i, follower in enumerate(self.follower_names):

            # Sanity check for v_hat
            v_value = float(self.v_hat[i])
            v_msg = Float32()
            v_msg.data = v_value
            
#            self.get_logger().info(f"[velocity observer for robot {i}] is {v_value}")

            self.publishers_v[i].publish(v_msg)

            # Sanity check for yaw_hat
            yaw_value = float(self.yaw_hat[i])
            yaw_msg = Float32()
            yaw_msg.data = yaw_value
            
#            self.get_logger().info(f"[yaw observer for robot {i}] is {yaw_value}")

            self.publishers_yaw[i].publish(yaw_msg)

        # Update plots
#        self.time_history.append(current_time)
#        self.yaw_history_leader.append((self.yaw_leader*math.pi)/180)
#        for i in range(self.num_followers):
#            self.yaw_history_followers[i].append(self.yaw_hat[i])
#
#        self.update_plot()

    def observer_update(self, i, estimates, leader_value, x_hat_i,var_type):
        temp_sum = 0.0
        for j, neighbor in enumerate(self.follower_names):
            if var_type=='yaw':
                diff = math.atan2(math.sin(estimates[j] - x_hat_i), math.cos(estimates[j] - x_hat_i))
            else:
                diff = estimates[j] - x_hat_i
            temp_sum += self.a[i][j] * diff
            #temp_sum+=self.a[i][j]*(estimates[j]-x_hat_i)
        if var_type=='yaw':
            leader_diff = math.atan2(math.sin(leader_value - x_hat_i), math.cos(leader_value - x_hat_i))
        else:
            leader_diff = leader_value - x_hat_i
        temp_sum+=self.b[i]*leader_diff
        if var_type=='v':
            alpha=self.alpha_v
            beta=self.beta_v
            gamma=self.gamma_v

        elif var_type=='yaw':
            alpha=self.alpha_yaw
            beta=self.beta_yaw
            gamma=self.gamma_yaw

        dx = alpha * np.sign(temp_sum) + beta * (abs(temp_sum) ** gamma) * np.sign(temp_sum)
        updated_x_hat = x_hat_i + dx * 0.1  # Assuming dt = 0.1

        if var_type=='yaw':
            # Normalize to [-pi, pi]
            updated_x_hat = ((updated_x_hat + math.pi) % (2*math.pi)) - math.pi
        return updated_x_hat

#    def update_plot(self):
#        self.ax.clear()
#        self.ax.plot(self.time_history, self.yaw_history_leader, 'k--', label='Leader Yaw')
#        for i in range(self.num_followers):
#            self.ax.plot(self.time_history, self.yaw_history_followers[i], label=f'Follower {i} Yaw Est')
#        self.ax.set_title('Leader vs Followers Yaw Estimation')
 #       self.ax.set_xlabel('Time (s)')
 #       self.ax.set_ylabel('Yaw (rad)')
#        self.ax.legend()
#        self.ax.grid(True)
#        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = ConsensusObserver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()