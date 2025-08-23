import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import xacro


from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')


    #robot_description_config = xacro.process_file(xacro_file,
     #   mappings={"frame_prefix": namespace + "/"}).toxml()

    world = os.path.join(
        pkg_path, "worlds", "empty.world"
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        ),
    )

    #ld.add_action(declare_enable_drive)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    ROWS = 1
    COLS = 2

    x = -ROWS
    y = -COLS
    last_action = None

    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    

    # Spawn turtlebot3 instances in gazebo
    for i in range(COLS):
        x = -ROWS
        for j in range(ROWS):
            # Construct a unique name and namespace
            name = "robot" + str(i) + "_" + str(j)
            namespace = "/tb" + str(i) + "_" + str(j)
            namespace_c = "tb" + str(i) + "_" + str(j)


            robot_description_config = xacro.process_file(xacro_file,
                    mappings={"frame_prefix": namespace + "/"}).toxml()

            turtlebot_state_publisher = Node(
                package="robot_state_publisher",
                namespace=namespace,
                executable="robot_state_publisher",
                output="screen",
                parameters=[{#'frame_prefix': namespace + "/",
                            'robot_description': robot_description_config,
                             "use_sim_time": True,
                             "publish_frequency": 10.0}],
                remappings=remappings,
                #arguments=[xacro_file],
            )

            # Create spawn call
            spawn_entity = Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-topic", namespace + "/robot_description",
                    #"-file",#xacro_file,
                    "-entity",
                    name,
                    "-robot_namespace",
                    namespace,
                    "-x", str(x), "-y", str(y), "-z", "0.0", "-Y", "3.14159", "-unpause",
                ],
                output="screen",
            )

            joint_broad_spawner=Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace_c,
                arguments=["joint_broad",
                      #      "--controller-manager", f"{namespace}/controller_manager",
                            "--ros-args", "--log-level", "info"]
                            )
            
            diff_drive_spawner=Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace_c,
                arguments=["diff_cont",
                   # "--controller-manager", f"{namespace}/controller_manager",
                    "--ros-args", "--log-level", "info"],
                        )
            
            # Advance by 2 meter in x direction for next robot instantiation
            x += 2.0

            if last_action is None:
                # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
                ld.add_action(turtlebot_state_publisher)
                ld.add_action(spawn_entity)



            else:
                # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
                # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
                spawn_turtlebot3_event = RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=last_action,
                        on_exit=[
                                 turtlebot_state_publisher,
                                 spawn_entity,
                        ]
                    )
                )
                ld.add_action(spawn_turtlebot3_event)

            # Save last instance for next RegisterEventHandler
            last_action = spawn_entity

        # Advance by 2 meter in y direction for next robot instantiation
        y += 2.0

    return ld