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


    for i in range(2):
        for j in range(2):
            name='robot'+str(i)+'_'+str(j)
            yaw_from_imu=Node(package='comp_pkg',
            executable='imu_yaw_node',
            name='imu_yaw_node',
            parameters=[{'imu_topic': name}],
            output='screen',)

            ld.add_action(yaw_from_imu)

    # Consensus Node (delayed by 5 seconds)
    consensus_node = TimerAction(
        period=4.0,  # delay in seconds
        actions=[
            Node(
                package='comp_pkg',
                executable='state_consensus_node',
                name='initialize_consensus_observer',
                output='screen',
            )
        ]
    )
    ld.add_action(consensus_node)


    # Desired Velocity Publisher Node (delayed by 10 seconds total)
    #des_publisher_node = TimerAction(
    #   period=5.0,  # delay in seconds (IMU + consensus time)
    #    actions=[
    #        Node(
    #            package='comp_pkg',
    #            executable='pub_des_vel_node',
    #            name='initialize_des_publisher',
    #            output='screen',
    #        )
    #    ]
    #)
    #ld.add_action(des_publisher_node)


    return ld

