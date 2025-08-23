#!/bin/bash
set -e


source install/setup.bash



ros2 launch my_bot localization_launch.py map:=./my_map_save.yaml use_sim_time:=True



