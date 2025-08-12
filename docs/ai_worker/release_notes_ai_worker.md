# Release Notes

## AI Worker
https://github.com/ROBOTIS-GIT/ai_worker

### 1.1.9 (2025-07-18)
* Added robot manager ros2 controller
* Added Swerve Steer initialization feature
* Fixed Gazebo simulation issue
* Added error code reading feature for Dynamixel
* Contributors: Woojin Wie

### 1.1.8 (2025-07-14)
* Modified joystick controller to enable lift control while in swerve mode
* Refactored joystick controller
* Contributors: Wonho Yun

### 1.1.7 (2025-07-11)
* Added slow start feature for joint_trajectory_command_broadcaster
* Contributors: Woojin Wie

### 1.1.6 (2025-07-07)
* Updated ros2_control xacro file for ffw_bg2_rev4_follower
* Fix header file name for tf2
* Contributors: Woojin Wie

### 1.1.5 (2025-06-30)
* Updated ros2_control xacro file for ffw_bg2_rev2_follower
* Contributors: Woojin Wie

### 1.1.4 (2025-06-27)
* Added ROS_DOMAIN_ID to the Dockerfile
* Contributors: Woojin Wie

### 1.1.3 (2025-06-26)
* Modified jog scale for ffw_lg2_leader
* Added dependencies to the package.xml file for image_transport_plugins
* Contributors: Woojin Wie

### 1.1.2 (2025-06-26)
* Added dependencies to the package.xml file
* Contributors: Woojin Wie

### 1.1.1 (2025-06-26)
* Reordered pip install order in Dockerfile to fix the numpy version issue
* Added Current Limit parameter to the ros2_control xacro file for ffw_sg2_rev1
* Contributors: Woojin Wie

### 1.1.0 (2025-06-16)
* Add installation of some ROS 2 packages for physical AI tools in Dockerfile
* Add an alias command in Dockerfile for running the physical AI server
* Support ffw_sg2_rev1 Model
* Add swerve drive controller package for ffw_sg2_rev1
* Modify joystick controller to support swerve mode
* Contributors: Kiwoong Park, Woojin Wie, Geonhee Lee, Wonho Yun

### 1.0.9 (2025-06-09)
* Updated urdf files for ffw_bg2_rev4
* Modified Gazebo launch file
* Contributors: Woojin Wie, Wonho Yun

### 1.0.8 (2025-06-02)
* Updated Model files for ffw_bg2_rev4
* Contributors: Woojin Wie

### 1.0.7 (2025-05-30)
* Updated Model files for ffw_bg2_rev4
* Contributors: Woojin Wie

### 1.0.6 (2025-05-28)
* Modified Docker volume mapping
* Created RealSense and ZED launch files
* Adjusted joint names
* Improved file structure
* Removed deprecated files
* Contributors: Woojin Wie

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

### 0.6.3 (2025-07-25)
* Fixed a bug to allow setting the output folder path to a specified location.
* Fixed a bug that did not guarantee the order of messages.
* Contributors: Dongyun Kim, Seongwoo Kim, Woojin Wie

### 0.6.2 (2025-07-24)
* Updated Lerobot to the latest version and modified related functionalities.
* Contributors: Dongyun Kim, Seongwoo Kim, Woojin Wie

### 0.6.1 (2025-07-23)
* Implemented robust error handling during data collection to prevent server crashes due to incorrect robot type configuration
* Contributors: Seongwoo Kim

### 0.6.0 (2025-07-23)
* Added training-related service types and message types
* Added a new training page for training imitation learning models
* Implemented a Training Manager to support model training through the Web UI
* Contributors: Kiwoong Park, Seongwoo Kim

### 0.5.13 (2025-07-21)
* Updated Docker volume mount paths from /root to ${HOME} for improved portability and compatibility
* Contributors: Seongwoo Kim

### 0.5.12 (2025-07-18)
* Enabled appending video encodings without overwriting existing data in multi-task mode
* Contributors: Seongwoo Kim

### 0.5.11 (2025-07-16)
* Added functionality for evaluating trained models
* Contributors: Dongyun Kim

### 0.5.10 (2025-07-15)
* Added multi-tasking data recording support to the Physical AI Server
* Contributors: Kiwoong Park, Seongwoo Kim

### 0.5.9 (2025-07-07)
* Use global ROS connection instead of multiple instances
* Add proper cleanup for image streams to prevent accumulation
* Remove unnecessary scrollbars in Chrome browser
* Contributors: Kiwoong Park

### 0.5.8 (2025-07-07)
* Applied Redux Toolkit for better state management
* Added heartbeat status to the UI
* Added heartbeat topic publishing to monitor alive status of Physical AI Server
* Contributors: Kiwoong Park, Dongyun Kim

### 0.5.7 (2025-06-26)
* Added Image Transport Plugin and fixed missing Gstreamer components
* Contributors: Dongyun Kim

### 0.5.6 (2025-06-26)
* Reordered pip install order in Dockerfile to fix the numpy version issue
* Contributors: Woojin Wie

### 0.5.5 (2025-06-26)
* Fixed control panel button states not reflecting correct taskType when switching between Record and Inference pages
* Contributors: Kiwoong Park

### 0.5.4 (2025-06-25)
* Added support for inference mode in the physical AI Server, including a new InferencePage and related UI components.
* Changed the robot naming format.
* Added Robot Config to support FFW-SG2 robot.
* Added Msg Topic and data acquisition functionality to support Mobile Robot.
* Fixed minor errors in the data acquisition process to improve software stability.
* Added a new inference page for running and monitoring inference tasks.
* Added inference-related msgs and srv types.
* Contributors: Dongyun Kim, Kiwoong Park

### 0.5.3 (2025-06-16)
* Refactored Physical AI Server for improved data collection capabilities
* Implemented data acquisition functionality using ROS2 topics
* Modified configuration system to allow flexible robot type selection
* Updated data collection method to utilize image buffers for efficiency
* Overall UI improvements for physical_ai_manager
* Added status information display from physical_ai_server
* Added functionality to receive task information from users and send commands to physical_ai_server
* Added bringup launch file that runs physical_ai_server with rosbridge_server and webvideo_server
* Contributors: Dongyun Kim, Kiwoong Park

### 0.5.2 (2025-05-29)
* Adjusted the waiting timeout for joint states.
* Contributors: Dongyun Kim

### 0.5.1 (2025-05-29)
* Added quality and transport parameters to image streaming URL
* Added a Docker setup for physical AI server
* Contributors: Kiwoong Park

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
