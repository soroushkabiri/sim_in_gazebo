#!/bin/bash
set -e


source install/setup.bash



ros2 launch my_bot connect_robots.launch.py


sleep 5

ros2 launch comp_pkg imu_yaw.launch.py



