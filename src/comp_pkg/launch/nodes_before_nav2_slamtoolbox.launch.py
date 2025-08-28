# this launch file 1) catch yaw data from imu 
# 2) initialize consensus observer 
# 3) initialize desired cmd_vel node

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction


def generate_launch_description():

    ld = LaunchDescription()


    # republish lidar data on /scan topic
    laser_relay_node = TimerAction(
        period=0.1,  # delay in seconds 
        actions=[
            Node(
                package='comp_pkg',
                executable='laser_relay',
                name='laser_relay',
                output='screen',
            )
        ]
    )
    ld.add_action(laser_relay_node)


    # creating odom for rect obj by gazebo
    gazebo_to_odom_node = TimerAction(
        period=0.4,  # delay in seconds 
        actions=[
            Node(
                package='comp_pkg',
                executable='gazebo_to_odom',
                name='gazebo_to_odom',
                output='screen',
            )
        ]
    )
    ld.add_action(gazebo_to_odom_node)




    return ld

