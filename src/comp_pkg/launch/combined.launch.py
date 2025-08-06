from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    my_bot_dir = get_package_share_directory('my_bot')

    # First launch file: Simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_bot_dir, 'launch', 'launch_sim_multi_corr.launch.py')),
        launch_arguments={}
    )
    print(type(sim_launch))


    # Second launch file: Connect Robots
    connect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_bot_dir, 'launch', 'connect_robots.launch.py')),
        launch_arguments={}
    )

    # Register event: when sim_launch exits → start connect_launch
    event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=sim_launch,
            on_exit=[connect_launch]
        )
    )

    return LaunchDescription([
        sim_launch,
        event_handler,
    ])
