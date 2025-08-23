from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        # First service (no delay)
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/ATTACHLINK', 'linkattacher_msgs/srv/AttachLink',
                "{model1_name: 'rect_obj', link1_name: 'base_link', model2_name: 'robot0_0', link2_name: 'robot0_0_fixed_link'}"
            ],
            output='screen'
        ),

        # Second service after 2 seconds
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/ATTACHLINK', 'linkattacher_msgs/srv/AttachLink',
                        "{model1_name: 'rect_obj', link1_name: 'base_link', model2_name: 'robot1_1', link2_name: 'robot1_1_fixed_link'}"
                    ],
                    output='screen'
                )
            ]
        ),

        # Third service after 4 seconds
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/ATTACHLINK', 'linkattacher_msgs/srv/AttachLink',
                        "{model1_name: 'rect_obj', link1_name: 'base_link', model2_name: 'robot1_0', link2_name: 'robot1_0_fixed_link'}"
                    ],
                    output='screen'
                )
            ]
        ),

        # Fourth service after 6 seconds
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/ATTACHLINK', 'linkattacher_msgs/srv/AttachLink',
                        "{model1_name: 'rect_obj', link1_name: 'base_link', model2_name: 'robot0_1', link2_name: 'robot0_1_fixed_link'}"
                    ],
                    output='screen'
                )
            ]
        ),
    ])
