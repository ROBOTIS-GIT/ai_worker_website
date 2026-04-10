# Release Notes

Release notes are posted in the **Discord** release notes channel. Use the link below to open the channel (a Discord account may be required).

[Open the release notes channel on Discord](https://discordapp.com/channels/1377230275393884170/1491682343553863750)

<!--
<details>
<summary>2026.04.01</summary>

📦 **AI Worker**
> **Version:** `1.2.0`
> **Released:** 2026-04-01
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.2.0)

- Added support for SH5, BH5 model
</details>

<details>
<summary>2026.03.26</summary>

📦 **AI Worker**
> **Version:** `1.1.21`
> **Released:** 2026-03-26
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.21)

- Added docker-compose.novnc.yml
</details>

<details>
<summary>2026.03.12</summary>

📦 **AI Worker**
> **Version:** `1.1.19`
> **Released:** 2026-03-12
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.19)

- Added MPPI navigation mode
- Merged AMCL params into navigation.yaml and added navigation_mppi.yaml
- Added BT tree with IsPathLengthUnder
- Added a console entry point for the mobile teleop node

> **Version:** `1.1.20`
> **Released:** 2026-03-12
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.20)

- Fixed version of docker image in docker-compose.yml
- Added a notice in container.sh when update is available

📦 **Physical AI Tools**
> **Version:** `0.8.2`
> **Released:** 2026-03-12
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.8.2)

- Changed talos repository name and url in Dockerfile
- Removed server pipeline service in s6 user bundle
</details>

<details>
<summary>2026.02.12</summary>

📦 **AI Worker**
> **Version:** `1.1.18`
> **Released:** 2026-02-09
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.18)

- Use sigint signal to shutdown s6-overlay services
- Supported System Manager
- Added s6-overlay services
</details>

<details>
<summary>2026.02.06</summary>

📦 **Physical AI Tools**
> **Version:** `0.8.1`
> **Released:** 2026-02-06
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.8.1)

- Add s6-agent and s6-services for supporting talos system manager
</details>

<details>
<summary>2026.01.29</summary>

📦 **AI Worker**
> **Version:** `1.1.16`
> **Released:** 2026-01-20
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.16)

- Added Navigation and mapping
<img src="/release_note/ai_worker/1.1.16/ffw_aiw_rviz2.gif" alt="AI Worker Rviz2">
<img src="/release_note/ai_worker/1.1.16/ffw_mapping.gif" alt="AI Worker Mapping">
<img src="/release_note/ai_worker/1.1.16/ffw_nav2.gif" alt="AI Worker Navigation">

📦 **Physical AI Tools**
> **Version:** `0.8.0`
> **Released:** 2026-01-19
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.8.0)

- Initial release of physical_ai_bt package
- Implemented rule-based behavior tree system with Sequence control node
- Added trajectory-based actions (MoveArms, MoveLift, MoveHead, Rotate)
- Added TreeLoader for XML-based configuration with auto-execution mode
<img src="/release_note/physical_ai_tools/0.8.0/pat_bt.gif" alt="Physical AI Tools Behavior Tree">
</details>

<details>
<summary>2025.12.18</summary>

📦 **AI Worker**
> **Version:** `1.1.15`
> **Released:** 2025-12-11
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.15)

- Add LiDAR URDF and launch configuration to AI Worker
<img src="/release_note/ai_worker/1.1.15/ffw_rviz2.gif" alt="AI Worker Rviz2">

- Resolved lift unit malfunction
- Support mock hardware. User can launch virtual robot without physical hardware.
</details>

<details>
<summary>2025.12.01</summary>

📦 **Physical AI Tools**
> **Version:** `0.7.2`
> **Released:** 2025-12-01
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.7.2)

- Fixed an issue where the task_index was being merged based on the first episode when merging episodes in the *.parquet data.
</details>

<details>
<summary>2025.11.28</summary>

📦 **Physical AI Tools**
> **Version:** `0.7.1`
> **Released:** 2025-11-28
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.7.1)

- Updated physical_ai_server launch file to use direct rosbridge websocket node instantiation
</details>

<details>
<summary>2025.11.21</summary>

📦 **Physical AI Tools**
> **Version:** `0.7.0`
> **Released:** 2025-11-21
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.7.0)

- Added rosbag_recorder package
- Added rosbag2 recording support when collecting LeRobot datasets
</details>

<details>
<summary>2025.11.20</summary>

📦 **ROBOTIS LAB**
> **Version:** `1.0.0`
> **Released:** 2025-11-17
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/robotis_lab)

- Added comprehensive Docker containerization for consistent development environment
- Docker Infrastructure:

> Created Dockerfile.base based on NVIDIA Isaac Sim container imageImplemented multi-stage build with Isaac Lab and Robotis Lab integration Added docker-compose.yaml with volume management for caches, logs, and datasetsCreated container.sh management script with build, start, enter, stop, clean, and logs commandsImplemented X11 forwarding support for GUI applications through x11.yamlAdded entrypoint.sh for runtime symbolic link setupConfigured .dockerignore files to optimize build context

- ***Dependencies Installation:***
    - Automated CycloneDDS build and installation from third_party submodule
    - Integrated robotis_dds_python installation from third_party submodule
    - Created separate Python virtual environment for LeRobot with version 0.3.3
    - Installed Isaac Lab and all required dependencies
- ***Container Features:***
    - Volume persistence for Isaac Sim caches, logs, and data
    - Bind mounts for source code, scripts, datasets, and third_party submodules
    - GPU acceleration support via NVIDIA Container Toolkit
    - Bash history preservation across container sessions
    - Pre-configured environment variables and command aliases
    - Network host mode for seamless communication
- ***Configuration:***
    - Added .env.base for centralized environment configuration
    - Support for Isaac Sim 5.1.0 as default version
    - Customizable paths and container naming
</details>


<details>
<summary>2025.11.06</summary>

📦 **ROBOTIS LAB**
> **Version:** `0.2.0`
> **Released:** 2025-10-28
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/robotis_lab)

**Add Sim2Real pipeline for OMY robot with LeRobot dataset conversion**

- Folder Structure Refactor:
    - Tasks are now separated and organized into two categories:
        - real_world_tasks – for real robot execution
        - simulator_tasks – for simulation environments
- Sim2Real Pipeline Implementation:
    - Task Recording: Added functionality to record demonstrations for the OMY plastic bottle pick-and-place task in simulation.
    - Sub-task Annotation: Introduced annotation tools for splitting demonstrations into meaningful sub-tasks, improving policy learning efficiency.
    - Action Representation Conversion: Converted control commands from joint-space to IK-based end-effector pose commands for better real-world transfer.
    - Data Augmentation: Added augmentation techniques to increase dataset diversity and enhance policy generalization.
    - Dataset Conversion: Integrated data conversion to the LeRobot dataset format, enabling compatibility with LeRobot’s training framework.
- ROS 2 Integration:
    - Modified to receive the leader’s /joint_trajectory values using the robotis_dds_python library without any ROS 2 dependency.

- Sim2Sim video
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/release_note/robotis_lab/0.2.0/sim2sim.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>
- Sim2Real video
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/release_note/robotis_lab/0.2.0/sim2real.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>
</details>

<details>
<summary>2025.10.27</summary>

📦 **Physical AI Tools**
> **Version:** `0.6.13`
> **Released:** 2025-10-27
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.6.13)

- Fixed physical_ai_server crash when querying user ID without locally registered HuggingFace token
- Changed to skip automatic HF user ID loading on Record page when Push to Hub is disabled
</details>

<details>
<summary>2025.10.23</summary>

📦 **AI Worker**
> **Version:** `1.1.14`
> **Released:** 2025-10-14
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.14)
Corrected the battery monitoring feature when exceeding the maximum voltage

📦 **Physical AI Tools**
> **Version:** `0.6.12`
> **Released:** 2025-10-21
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.6.12)

- Enhanced SendTrainingCommand.srv with resume functionality.
- Added GetTrainingInfo.srv for retrieving training configuration from saved model checkpoints.
- Added training resume functionality.
- Improved performance of dataset episode deletion by implementing batch deletion.
<img src="/release_note/physical_ai_tools/0.6.12/pat_check_point.png" alt="Physical AI Tools Checkpoint">
<img src="/release_note/physical_ai_tools/0.6.12/pat_check_point_button.png" alt="Physical AI Tools Checkpoint Button">
</details>

<details>
<summary>2025.08.27</summary>

📦 **AI Worker**
> **Version:** `1.1.11`
> **Released:** 2025-08-27
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.11)

Added tactile switch feature to joystick controller.
- Data recording has become more convenient with the joystick.
<img src="/release_note/ai_worker/1.1.11/ffw_joystick.png" alt="AI Worker Joystick">

Applied Dynamic Brake for Dynamixel Y.
- Safety has been improved by allowing the arm to move down slowly in case of a forced program shutdown.

📦 **Physical AI Tools**
> **Version:** `0.6.8`
> **Released:** 2025-08-21
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.6.8)

Basic Dataset Editing in Web UI.
- Add support for merging multiple datasets and deleting unwanted episodes from datasets
- Enable deletion of unwanted episodes from datasets
<img src="/release_note/physical_ai_tools/0.6.8/pat_edit_data.png" alt="Physical AI Tools Dataset Editing">

Dataset Recording Improvements.
- Beep sound notification at episode recording start
- Retry current episode / Move to next episode using leader device button controls (AI Worker only)
</details>

<details>
<summary>2025.08.14</summary>

📦 **AI Worker**
> **Version:** `1.1.10`
> **Released:** 2025-08-14
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.10)

Added start/pause feature for ffw_joint_trajectory_command_broadcaster.
- Pushing the trigger will start and stop the teleop.

Added ffw_robot_manager ROS 2 controller.
- Head LED Control
  - FFW Robot Manager detects Dynamixel hardware error and changes LED.
  <img src="/release_note/ai_worker/1.1.10/ffw_head_led.gif" alt="AI Worker Head LED">

- Dynamixel Torque Control
  - Provides torque disable option when the Dynamixel hardware error is detected.

Added ZED mini to URDF.
<img src="/release_note/ai_worker/1.1.10/ffw_zed_urdf.png" alt="AI Worker ZED URDF">

Support MoveIt for FFW SG2.
<img src="/release_note/ai_worker/1.1.10/ffw_moveit.gif" alt="AI Worker MoveIt">


📦 **Physical AI Tools**
> **Version:** `0.6.6`
> **Released:** 2025-08-12
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.6.6)

Training loss display in Web UI.
- Added current training loss value display to the Training page in Web UI.
<img src="/release_note/physical_ai_tools/0.6.6/ffw_training_loss.png" alt="Physical AI Tools Training Loss">

Enhanced policy model selection for inference.
- Added file browsing functionality to make policy model path selection more intuitive and convenient when performing inference.
<img src="/release_note/physical_ai_tools/0.6.6/ffw_policy_path.png" alt="Physical AI Tools Policy Path">


📦 **ROBOTIS LAB**
> **Version:** `0.1.2`
> **Released:** 2025-08-11
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/robotis_lab)

Added FFW BG2 Pick-and-Place Imitation Learning Environment.
- Built an imitation learning environment for cylindrical rod pick-and-place using the FFW BG2 robot.
<img src="/release_note/robotis_lab/0.1.2/ffw_imitation_learning_env.png" alt="Robotis Lab Imitation Learning Environment">
</details>


<details>
<summary>2025.07.17</summary>

📦 **AI Worker**
> **Version:** `1.1.8`
> **Released:** 2025-07-14
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.8)

Added slow start feature for joint_trajectory_command_broadcaster.
- Modified joystick controller to enable lift control while in swerve mode.

📦 **Physical AI Tools**
> **Version:** `0.6.0`
> **Released:** 2025-07-16
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.6.0)

Added the training module.
- Model policy training is now supported via the Web GUI.
<img src="/release_note/physical_ai_tools/0.6.0/ffw_training.png" alt="Physical AI Tools Training">

Added an evaluation for trained models.  
<img src="/release_note/physical_ai_tools/0.6.0/ffw_policy_evaluation.png" alt="Physical AI Tools Policy Evaluation">  
<img src="/release_note/physical_ai_tools/0.6.0/ffw_policy_evaluation_2.png" alt="Physical AI Tools Policy Evaluation">

Added multi-tasking support.
- Multi-tasking data collection is now supported via the Web GUI.
<img src="/release_note/physical_ai_tools/0.6.0/ffw_multi_task.png" alt="Physical AI Tools Multi-Task">

- It is possible to specify an arbitrary number of task instructions from now on.
<img src="/release_note/physical_ai_tools/0.6.0/ffw_multi_task_2.png" alt="Physical AI Tools Multi-Task">


📦 **ROBOTIS LAB**  
> **Version:** `0.1.1`  
> **Released:** 2025-07-16  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/robotis_lab)

Added Sim2Real functionality to the OMY reach task.  
- Development and deployment of Sim2Real capabilities with the OMY robot and NVIDIA Isaac Sim.  
  - ISAAC SIM  
  <img src="/release_note/robotis_lab/0.1.1/ffw_isaac_sim.png" alt="ROBOTIS Lab Isaac Sim">  
  - Real World  
  <img src="/release_note/robotis_lab/0.1.1/ffw_real_world.png" alt="ROBOTIS Lab Real World">
</details>


<details>
<summary>2025.07.03</summary>

📦 **AI Worker**
> **Version:** `1.1.5`
> **Released:** 2025-06-30
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.5)

Updated FFW-SG2 model with latest configurations and improvements.
- Integrated ROS 2 controller for swerve drive support. 
- Added joystick control functionality for swerve drive.
- Updated robot configuration for improved system stability.
<img src="/release_note/ai_worker/1.1.5/ffw_sg2.png" alt="AI Worker SG2">


📦 **Physical AI Tools**
> **Version:** `0.5.7`
> **Released:** 2025-06-30
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.5.7)

Enhanced policy inference via Web UI.
- Enable AI model inference directly from the inference page without CLI.
<img src="/release_note/physical_ai_tools/0.5.7/ffw_inference.png" alt="Physical AI Tools Inference">


📦 **ROBOTIS LAB**
> **Version:** `0.1.0`
> **Released:** 2025-07-01
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/robotis_lab)

Released robotis_lab repository.
- Newly created simulation and RL/IL research toolkit for Robotis robots built on Isaac Lab.
<img src="/release_note/robotis_lab/0.1.0/ffw_robotis_lab.png" alt="Robotis Lab">

- Added instructions for the AI Worker homepage.
<img src="/release_note/robotis_lab/0.1.0/ffw_robotis_lab_documentation.png" alt="Robotis Lab">
</details>


<details>
<summary>2025.06.19</summary>

📦 **AI Worker**
> **Version:** `1.0.9`
> **Released:** 2025-06-18
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.0.9)

Added Gazebo simulation support for the AI Worker.
- Included inertial properties in the URDF and XACRO files for the follower model.
<img src="/release_note/ai_worker/1.0.9/ffw_gazebo_sim.png" alt="AI Worker Gazebo Sim">


📦 **Physical AI Tools**
> **Version:** `0.5.3`
> **Released:** 2025-06-18
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.5.3)

Web UI-based ROS 2 data acquisition system for LeRobot.
- From CLI to UI: Data collection now operates through an intuitive web-based UI using Physical AI Manager.
- From fixed configs to flexible launch: Robot types can now be selected dynamically.
- From manual commands to task flow: Users can input tasks, and the system sends commands to the Physical AI Server.
- From raw topic reading to buffered capture: Improved image acquisition using efficient buffering.
<img src="/release_note/physical_ai_tools/0.5.3/ffw_data_collection.png" alt="Physical AI Tools Data Collection">

- Episode tracking during data collection.
- Real-time monitoring of system resource (CPU, RAM and disk usage).
- Easy parameter configuration for each data session.
- Simple control via Start / Stop / Retry / Next / Finish buttons.
</details>


<details>
<summary>2025.06.05</summary>

📦 **AI Worker**
> **Version:** `1.0.8`
> **Released:** 2025-06-02
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.0.8)

AI Worker Hardware REV-4 Release.
<img src="/release_note/ai_worker/1.0.8/ffw_rev4.png" alt="AI Worker REV-4" style="width: 100%; ">

- **REV-4**: New outer design, Orin relocated to the chest, ZED camera mounted on the head, and wrist camera with tilt functionality
- **REV-3**: Improved wrist joint
- **REV-2**: Internal development version
- **REV-1**: Initial version equipped with the INSPIRE hand


📦 **Physical AI Tools**
> **Version:** `0.5.2`
> **Released:** 2025-05-29
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.5.2)

Physical AI Manager – web app for data collection, training, and inference.
- Added a web-based UI tool that shows live image streaming during data collection. You can monitor real-time image streaming through your web browser.
<img src="/release_note/physical_ai_tools/0.5.2/ffw_physical_ai_manager.png" alt="Physical AI Manager">


📦 **AI Worker Website**
> 🔗 [Website](https://ai.robotis.com/)

**AI.ROBOTIS.COM launched**
- Management of AI.ROBOTIS.COM has begun. The site includes hardware, software, setup guides, and imitation learning manuals for the AI Worker.
<img src="/release_note/ai_worker/ffw_website.png" alt="Website">
</details>
-->
