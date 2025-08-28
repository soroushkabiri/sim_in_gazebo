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


    # Desired Velocity Publisher Node
    des_publisher_node = TimerAction(
        period=0.1,  # delay in seconds 
        actions=[
            Node(
                package='comp_pkg',
                executable='pub_des_vel_node',
                name='initialize_des_publisher',
                output='screen',
            )
        ]
    )
    ld.add_action(des_publisher_node)




    # publish desired data for fuzzy planner
    data_for_fuzzy = TimerAction(
        period=2.0,  # delay in seconds 
        actions=[
            Node(
                package='comp_pkg',
                executable='data_necessary_for_fuzzy',
                name='necessary_data_for_fuzzy',
                output='screen',
            )
        ]
    )
    ld.add_action(data_for_fuzzy)


    return ld

