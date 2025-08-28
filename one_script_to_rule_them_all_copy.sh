#!/bin/bash
set -e




tmux new-session -d -s ros_sim "source install/setup.bash; ros2 launch my_bot launch_sim_multi_corr.launch.py" \; \
split-window -v "sleep 30; source install/setup.bash; ros2 launch my_bot connect_robots.launch.py" \; \
split-window -h "sleep 50; source install/setup.bash; ros2 launch comp_pkg imu_yaw.launch.py" \; \
split-window -v "sleep 62; source install/setup.bash; ros2 launch my_bot navigation_launch.py use_sim_time:=True" \; \
split-window -h "sleep 80; source install/setup.bash; ros2 launch my_bot localization_launch.py map:=./my_map_save.yaml use_sim_time:=True" \; \
split-window -v "sleep 98; ./publish_initial_pose.sh" \; \
split-window -h "sleep 103; source install/setup.bash; ros2 launch comp_pkg velocity_publishing.launch.py" \; \
select-layout tiled \; attach

#ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "{start: {},goal: {header: {frame_id: 'map'}, pose: {position: {x: -14.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}},use_start: false}"
