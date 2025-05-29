# Release Notes

## AI Worker
https://github.com/ROBOTIS-GIT/ai_worker

### 1.0.5 (2025-05-09)
* Fixed Dockerfile
* Updated Camera URDF
* Contributors: Woojin Wie

### 1.0.4 (2025-05-08)
* Fixed Dockerfile
* Updated ros2 control xacro file to support async
* Contributors: Woojin Wie

### 1.0.3 (2025-04-28)

* Added support for Joystick controller
* Added ffw_spring_actuator_controller
* Contributors: Woojin Wie

### 1.0.2 (2025-04-16)
* Added support for ROBOTIS RH Gripper
* Added differentiation between slow and fast versions
* Updated codebase to comply with flake8 linting standards
* Contributors: Wonho Yun

### 1.0.1 (2025-04-07)
* Modified the profile velocity parameters for enhanced arm and hand teleoperation performance
* Modified the README file to reflect usage instructions for this package
* Removed unused files and redundant comments to streamline the codebase
* Contributors: Wonho Yun, Pyo

### 1.0.0 (2025-04-06)
* Added the initial version of the FFW ROS package
* Added arm and hand teleoperation support for FFW
* Added integrated controller compatibility for Inspire Robot Hand
* Contributors: Sungho Woo, Woojin Wie, Wonho Yun, Pyo

### 0.1.0 (2025-03-27)
* Added bringup scripts for system initialization
* Added robot description files for visualization and planning
* Added base controller functionalities
* Added MoveIt for motion planning support
* Contributors: Sungho Woo, Woojin Wie

## Physical AI Tools
- https://github.com/ROBOTIS-GIT/physical_ai_tools

### 0.5.0 (2025-05-20)
* Added a web UI for physical AI data collection
* Removed unnecessary dependencies and cleaned up the codebase
* Updated the LeRobot submodule to the latest version
* Refactored to a scalable structure that supports N cameras and various joint configurations
* Contributors: Dongyun Kim, Kiwoong Park, Woojin Wie, Seongwoo Kim

### 0.4.0 (2025-05-15)
* Added a pipeline for data collection and inference based on ROS2
* Refactored to a scalable structure that supports N cameras and various joint configurations
* Contributors: Dongyun Kim

### 0.3.1 (2025-05-08)
* Updated the LeRobot submodule to the latest version
* Contributors: Woojin Wie

### 0.3.0 (2025-04-25)
* Unified multiple launch files into a single configurable launch file for better usability
* Contributors: Seongwoo Kim

### 0.2.0 (2025-04-08)
* Added a time stamper node for data synchronization purposes
* Removed unused joints and motors bus config
* Contributors: Seongwoo Kim, Hyungyu Kim

### 0.1.0 (2025-04-07)
* Added a full workflow for recording and visualizing datasets using the LeRobot interface
* Added bringup scripts for system initialization
* Contributors: Seongwoo Kim, Pyo
