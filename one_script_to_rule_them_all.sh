#!/bin/bash
set -e



# Terminal 1: make gazebo model
gnome-terminal -- bash -c "
  source install/setup.bash;
  ros2 launch my_bot launch_sim_multi_corr.launch.py;
  exec bash
"
sleep 30


# Terminal 2: connect robots to rect_obj
gnome-terminal -- bash -c "
  source install/setup.bash;
  ros2 launch my_bot connect_robots.launch.py;
  exec bash
"
sleep 15

# Terminal 3: make yaw from imu and doing odom for lidar
gnome-terminal -- bash -c "
  source install/setup.bash;
  ros2 launch comp_pkg imu_yaw.launch.py;
  exec bash
"
sleep 12


# Terminal 4: map server from nav2
gnome-terminal -- bash -c "
  source install/setup.bash;
  ros2 launch my_bot navigation_launch.py use_sim_time:=True;
  exec bash
"
sleep 18



# Terminal 5: Run localization from nav2
gnome-terminal -- bash -c "
  source install/setup.bash;
  ros2 launch my_bot localization_launch.py map:=./my_map_save.yaml use_sim_time:=True;
  exec bash
"

# Give AMCL some time to start
sleep 18

# Terminal 6: Publish initial pose
gnome-terminal -- bash -c "
  source install/setup.bash;
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
    header: {frame_id: \"map\"},
    pose: {
      pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
      },
      covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0685, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0685, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
    }
  }';
  exec bash
"
sleep 5



# Terminal 7: start consensus for leader followers
gnome-terminal -- bash -c "
  source install/setup.bash;
  ros2 launch comp_pkg velocity_publishing.launch.py;
  exec bash
"

