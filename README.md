# ROS2 workspace for multi-robot transporting of a rectangular object


---

## Tested on
- **ROS 2 Humble** (Ubuntu 22.04)
---


# Build the workspace
colcon build
source install/setup.bash


# mapping the environment

for mapping the environment we use slam toolbox package. assume that the lidar sensor is attached on top of the transporting object.
<img width="1279" height="726" alt="lidar_on_rect_obj" src="https://github.com/user-attachments/assets/639a0c3d-6de0-42cb-99b0-39fc8a01e249" />


./mapping.sh

# transporting object
./main_script_visual.sh
