# Project-Bob-ROS

## Overview

Project-Bob-ROS is a robotic system designed to control a UR5e robotic arm to draw images on a whiteboard. This project integrates computer vision, robotic motion planning, and control to achieve precise and automated image drawing.

## Features

- **Image Processing**: Detects and processes images to extract drawing waypoints using edge detection and other computer vision techniques.
- **Waypoint Transformation**: Converts 2D waypoints from the image plane to 3D coordinates in the robot's reference frame.
- **Motion Planning**: Plans and executes the robotic arm's movements to draw the image on the whiteboard.
- **Collision Avoidance**: Ensures safe operation by adding collision objects to the robot's environment.
- **Real-Time Visualization**: Visualizes the robot's movements and the drawing process in RViz.

## Workspace Structure

The workspace is divided into the following main components:

### 1. **Master_WS**
   - **bob_ros**: Contains the core ROS nodes for controlling the UR5e arm and processing waypoints.
     - `arm_controller.py`: Controls the UR5e arm's movements.
     - `find_waypoints_image.py`: Processes images to extract drawing waypoints.
     - `transform_points.py`: Transforms waypoints from the image plane to the robot's reference frame.
     - `set_collision_scene.py`: Adds collision objects to the robot's environment.
   - **perception**: Handles image detection and processing for edge detection and canvas isolation.

### 2. **Universal_Robots_ws**
   - **Universal_Robots_ROS_Driver**: Provides the ROS driver for controlling Universal Robots' robotic arms.
   - **ur_description**: Contains URDF files for the UR5e robot model.
   - **ur_moveit_config**: Configures MoveIt! for motion planning with the UR5e.

### 3. **Visualization Files**
   - `plane_visualisation.rviz`: RViz configuration for visualizing the drawing plane.
   - `robot_visualisation.rviz`: RViz configuration for visualizing the UR5e robot.

## Installation

1. **Install ROS Noetic**:
   Follow the [ROS Noetic installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

2. **Clone the Repository**:
   ```bash
   git clone <repository-url> ~/catkin_ws/src
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. **Install Dependencies**:
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Usage

1. **Launch the System**:
   ```bash
   roslaunch bob_ros main.launch
   ```

2. **Provide an Image**:
   Place the image to be drawn in the `Master_WS/src/bob_ros/images/` directory.

3. **Start Drawing**:
   The robot will process the image, compute waypoints, and begin drawing on the whiteboard.

## Contributors

- **Connor Latham**: [clat0391@uni.sydney.edu.au](mailto:clat0391@uni.sydney.edu.au)
- **Terry Wang**: Image processing and detection.

## License

This project is licensed under the BSD-3-Clause License. See the `LICENSE` file for details.

