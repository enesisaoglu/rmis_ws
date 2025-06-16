# RMIS

## Overview
RMIS (Robot Motion Imitation System) is an open-source humanoid robot project designed to mimic human movements by processing skeletal data from a Kinect camera using MediaPipe for pose estimation. The system captures RGB and depth data from the Kinect, estimates 3D joint positions, computes inverse kinematics (IK) for a humanoid robot, and visualizes the results in RViz and Gazebo. A Flask-based web interface allows real-time monitoring and control of the robot’s joint states. Developed as a capstone project, RMIS leverages ROS2 Humble, Blender, MediaPipe, and IKPy.

## Features
- **Robot Model**: Designed in Blender and exported as a URDF with STL models using the Phobos plugin.
- **Pose Estimation**: Uses MediaPipe to track human skeletal joints (nose, wrists, feet) from Kinect RGB images, combined with depth data for 3D positioning.
- **Inverse Kinematics**: Computes joint angles for the robot’s arms, legs, and head using IKPy, enabling human motion imitation.
- **Visualization**: Displays the robot model and joint movements in RViz and Gazebo via a custom launch file.
- **Web Interface**: A Flask-based UI (`robot_control_ui`) provides real-time visualization of MediaPipe-annotated images, robot camera feed, and manual joint control via sliders.
- **ROS2 Integration**: Built on ROS2 Humble, with packages for Kinect integration, pose estimation, and UI control.

## Current Status
- **Robot Model**: URDF (`rmisurdf.urdf`) defined in the `rmis` package, with STL models and Gazebo-compatible configurations.
- **Kinect Integration**: Uses the `kinect2_bridge` package to process RGB and depth data at QHD resolution (30 FPS, CPU-based depth method).
- **Pose Estimation**: The `pose_estimation` package processes Kinect data with MediaPipe to detect key joints and publishes 3D coordinates to `/pose/joint_poses`. Inverse kinematics handles robot joint movements.
- **Visualization**: The `gazebo.launch.py` file launches Gazebo, RViz, and `robot_state_publisher` to visualize the robot and its environment.
- **Web Interface**: The `robot_control_ui` package runs a Flask server with a web interface for real-time monitoring and manual joint control.

## Prerequisites
- **Operating System**: Ubuntu 22.04 (tested).
- **ROS2**: Humble Hawksbill (`ros-humble-desktop`).
- **Blender**: Version with Phobos plugin for URDF export.
- **Kinect Camera**: Compatible with `kinect2_bridge` (Kinect v2).
- **Python Dependencies**:
  - `mediapipe`, `opencv-python`, `ikpy`, `numpy`.
  - `rosbridge_server` for web interface (`sudo apt install ros-humble-rosbridge-server`).
- **Other Tools**: Git, `colcon` build tool, `rosdep`.

## Installation
1. **Set up ROS2 Humble**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   source /opt/ros/humble/setup.bash
2. **Clone the Repository**:
   ```bash
   mkdir -p ~/rmis_ws/src
   cd ~/rmis_ws/src
   git clone https://github.com/enesisaoglu/rmis_ws.git
3. **Install Dependencies**:
   ```bash
   cd ~/rmis_ws
   rosdep install --from-paths src --ignore-src -r -y
   pip install mediapipe opencv-python ikpy numpy
4. **Build the Workspace**:
   ```bash
   colcon build
   source install/setup.bash

## Usage
1. **Launch Gazebo and RViz**:
   - Run the Gazebo simulation and RViz visualization:
   ```bash
   ros2 launch rmis gazebo.launch.py
   ```   
   - This will:
     - Loads the URDF (`rmisurdf.urdf`) and world file (`stable.world`).
     - Starts robot_state_publisher and spawns the robot in Gazebo.
     - Launches RViz for visualization.
2. **Start the Kinect Camera**:
   - Run the Kinect bridge to receive RGB and depth images:
   ```bash
   ros2 run kinect2_bridge kinect2_bridge --ros-args -p resolution:=qhd -p fps_limit:=30.0 -p depth_method:=cpu
   ```
3. **Run Pose Estimation**:
   - Launch the pose estimation nodes to process Kinect data with MediaPipe:
   ```bash
   ros2 launch pose_estimation pose_estimation_launch.py
   ```
   - This will run:
     - `image_processing_node`: Detects skeletal joints (nose, wrists, feet) using MediaPipe, computes 3D coordinates with depth data, and publishes to `/pose/joint_poses` and `/pose/annotated_image`.
     - `rmis_mimic_node`: Computes inverse kinematics for the robot’s joints (arms, legs, head) using IKPy and publishes to `/joint_states`.
    
4. **Launch the Web Interface**:
   - Start the Flask-based web interface for monitoring and control:
      ```bash
      ros2 launch rmis_data_publisher data_publisher_launch.py

## Project Structure 
