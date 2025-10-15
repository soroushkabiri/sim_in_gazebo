# ROS2 workspace for multi-robot transporting of a rectangular object


---

## Tested on
- **ROS 2 Humble** (Ubuntu 22.04)
---


# Build the workspace
colcon build
source install/setup.bash


# mapping the environment
./mapping.sh

# transporting object
./main_script_visual.sh
