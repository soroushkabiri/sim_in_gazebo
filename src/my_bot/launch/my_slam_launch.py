from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():



    leader_robot='robot0_0'

    # Declare configurable launch arguments
    prefix_arg = DeclareLaunchArgument('prefix', default_value='robot0_0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('my_bot'),
            'config',
            'mapper_params_online_async.yaml'
        )
    )

    # Use the launch configuration
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params = LaunchConfiguration('slam_params_file')

    # Hardcoded or derived topic/frame names for the single robot
    scan_topic = '/'+leader_robot+'/laser_controller/out'
    odom_frame = leader_robot + '_odom'
    map_frame = leader_robot + '_map'
    base_frame = leader_robot + '_base_footprint'

    # Define the SLAM toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {
                'scan_topic': scan_topic,
                'odom_frame': odom_frame,
                'map_frame': map_frame,
                'base_frame': base_frame,
                'use_sim_time': use_sim_time,
            }
        ]
    )

    return LaunchDescription([
        prefix_arg,
        use_sim_time_arg,
        slam_params_arg,
        slam_toolbox_node
    ])
