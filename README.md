# AUV Max: Submarine AUV Project

## Introduction
This project focuses on the development of a submersible UAV, using advanced technologies in robotics, automatic control and computer vision. Developed with ROS2 Humble, Gazebo Garden 7 and Rviz2, this project represents a pioneering effort in the simulation and operation of underwater autonomous vehicles.

## Project Structure
The project is divided into several packages, each focused on a specific functionality:


| Package | Description |
| --- | --- |
| `auv_max` | Packages necessary for the correct functioning of the project |
| `auv_max_bringup` | Launch files for the launch of Gazebo, Rviz and future physical implementations |
| `auv_max_control_pos` | Position control using a PID to control linear X-Z and angular position Yaw-Pitch |
| `auv_max_description` | URDF Files and Display Settings for Rviz |
| `auv_max_gazebo` | AUV model and world for simulation in Gazebo |
| `auv_max_graphics` | Graphic modules and visualizations for the AUV |
| `auv_max_node` | Main node in charge of communication between Gazebo-Rviz-ROS |
| `auv_max_sonar` | Algorithm commissioned to recognize obstacles underwater |
| `auv_max_teleoperation` | Algorithm for AUV teleoperation through the use of the keyboard |
| `auv_max_vision_opencv` | Computer vision implementation using OpenCV |

## Requirements
- ROS2 Humble
- Gazebo Garden 7
- Rviz2
- OpenCV

## Installation and configuration
Follow these steps to install and configure the project:

1. Clone the repository:
   ```
   git clone https://github.com/iesusdavila/auv_max
   ```
2. Navigate to the project directory and run:
   ```
   colcon build
   ```
3. Set up the ROS environment:
   ```
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

## Use
- To launch the simulation in Gazebo: `ros2 launch auv_max_bringup max_sim.launch.py`
- To activate teleoperated control: `ros2 run auv_max_teleoperation auv_teleop_keyboard`

## Development and Contributions
If you are interested in contributing to this project, consider the following:
- **Code Style:** Follow the ROS2 and PEP8 conventions for Python.
- **Tests:** Ensure that all new code is accompanied by unit tests.
- **Documentation:** All new developments must be properly documented.

---
© 2023 AUV Max Project
