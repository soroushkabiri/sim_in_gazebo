# Multi-Robot Transport of a Rectangular Object in ROS 2
---
## Tested on
- **ROS 2 Humble** (Ubuntu 22.04)
---
# Build the workspace

colcon build

source install/setup.bash

# mapping the environment

To map the environment, we use the slam_toolbox package. In this setup, the LiDAR sensor is mounted on top of the rectangular object being transported.

<img width="1279" height="726" alt="lidar_on_rect_obj" src="https://github.com/user-attachments/assets/639a0c3d-6de0-42cb-99b0-39fc8a01e249" />

There are three pre-defined maps located in the world directory of the my_bot package. To use a specific map, update the world directory path in launch_sim_multi_corr.launch.py.

For example, world3:
<img width="1003" height="574" alt="map3_gazebo" src="https://github.com/user-attachments/assets/31a86830-4e9b-454e-ad26-58736927cb12" />

The corresponding map is:

<img width="574" height="288" alt="my_map_3_save (copy)" src="https://github.com/user-attachments/assets/cd051f33-7b09-40ef-86ed-d2f5cda0ec37" />

To perform mapping, run the following script:

./mapping.sh

# Transporting the Object
After mapping, the rectangular object can be transported using multiple robots. Run the following script:

./main_script_visual.sh

To switch between worlds, update the world directory path in the script 6_launch.sh.

An example of four robots transporting the rectangular object is shown below:






