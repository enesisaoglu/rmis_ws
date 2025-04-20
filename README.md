# RMIS 

## Overview
RMIS is a humanoid robot project designed to process skeletal data from a Kinect camera and move its motors based on human movements. This is an open-source project developed for a capstone project, built using ROS2 Humble, Blender, and a Kinect camera.

## Current Status
- Robot model designed in Blender and exported as URDF using Phobos, with DAE format models referenced within the URDF.
- Kinect camera integrated into the ROS2 workspace following the instructions from [kinect2_ros2](https://gitioc.upc.edu/labs/kinect2_ros2/-/blob/main/README.md?ref_type=heads). A working launch file is included in the `launch/` directory.
- A custom launch file (`display.launch.py`) has been created to visualize the robot’s URDF in RViz and enable joint movement. This launch file:
  - Loads the URDF model (`rmis_robot.urdf`) from the `rmis_description` package.
  - Runs `robot_state_publisher` to publish the robot’s state.
  - Includes `joint_state_publisher` (headless) or `joint_state_publisher_gui` (with GUI) to control joint states, configurable via a `gui` launch argument.
  - Launches RViz and Gazebo for visualization.

## Prerequisites
- **ROS2 Humble** (tested on Ubuntu 22.04)
- **Blender** (with Phobos plugin for URDF export, used to design the robot model and export it as URDF with DAE format references)
- **rosbridge_server** (for web interface, install with `sudo apt install ros-humble-rosbridge-server`)
  
## Installation
1. **Set up ROS2 Humble:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   source /opt/ros/humble/setup.bash

2. **Clone the repository:**
   ```bash
    mkdir -p ~/rmis_ws/src
    cd ~/rmis_ws/src
    git clone [git@github.com:enesisaoglu/rmis_ws.git](https://github.com/enesisaoglu/rmis_ws.git)

3. **Install dependencies and build:**
   ```bash
    cd ~/rmis_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    source install/setup.bash

## Usage

1. **Visualize the robot in RViz and Gazebo:**
   ```bash
   ros2 launch rmis_description display.launch.py
 ![rmis_rviz](https://github.com/user-attachments/assets/7640da43-6544-4a3b-98e2-9ecd17414d78)

2. **Connect to the RMIS web interface (optional):**
To use the graphical user interface for interacting with the RMIS robot, follow the instructions in the rmis_interface_ws repository to set up and run the web server. Then, in this workspace, run the following command to start the rosbridge_server and publish messages to /my_topic for the web interface:
   ```bash
   ros2 launch rmis_data_publisher data_publisher_launch.py

   
## Contributing
Feel free to fork this project, submit issues, or send pull requests. All contributions are welcome!

## License
This project is licensed under the MIT License.




