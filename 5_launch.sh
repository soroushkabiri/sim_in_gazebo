#!/bin/bash
set -e


source install/setup.bash

ros2 launch my_bot navigation_launch.py use_sim_time:=True





