# Release Notes

<details>
<summary>2025.08.14</summary>

📦 **AI Worker**  
> **Version:** `1.1.10`  
> **Released:** 2025-08-13  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.10)

Added start/pause feature for ffw_joint_trajectory_command_broadcaster.
- Pushing the trigger will start and stop the teleop

Added ffw_robot_manager ros2 controller.
- Head LED Control
    - FFW Robot Manager Detects Dynamixel Hardware Error and changes led.
<img src="/release_note/ai_worker/ffw_head_led.gif" alt="AI Worker Head LED">


- Dynamixel Torque Control
    - Provides Torque disable option when the Dynamixel Hardware Error is detected.

Added ZED mini to URDF
<img src="/release_note/ai_worker/ffw_zed_urdf.png" alt="AI Worker ZED URDF">

Support Moveit for FFW SG2
<img src="/release_note/ai_worker/ffw_moveit.gif" alt="AI Worker Moveit">


📦 **Physical AI Tools**
> **Version:** `0.6.6`  
> **Released:** 2025-08-12  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.6.6)

Training loss display in Web UI
- Added current training loss value display to the Training page in web UI.
<img src="/release_note/ai_worker/ffw_training_loss.png" alt="Physical AI Tools Training Loss">

Enhanced policy model selection for inference
- Added file browsing functionality to make policy model path selection more intuitive and convenient when performing inference.
<img src="/release_note/ai_worker/ffw_policy_path.png" alt="Physical AI Tools Policy Path">

📦 **ROBOTIS LAB**
> **Version:** `0.1.2`  
> **Released:** 2025-08-11  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/robotis_lab)

Added FFW BG2 Pick-and-Place Imitation Learning Environment.
- Built an imitation learning environment for cylindrical rod pick-and-place using the FFW BG2 robot.
<img src="/release_note/ai_worker/ffw_imitation_learning_env.png" alt="Robotis Lab Imitation Learning Environment">
</details>


<details>
<summary>2025.07.17</summary>

📦 **AI Worker**  
> **Version:** `1.1.8`  
> **Released:** 2025-07-14  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.8)

Added slow start feature for joint_trajectory_command_broadcaster.
- Modified joystick controller to enable lift control while in swerve mode.
<img src="/release_note/ai_worker/ffw_slow_start.gif" alt="AI Worker Slow Start">

📦 **Physical AI Tools**
> **Version:** `0.6.0`  
> **Released:** 2025-07-16  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.6.0)

Added the training module.
- Model policy training is now supported via the Web GUI.
<img src="/release_note/ai_worker/ffw_training.png" alt="Physical AI Tools Training">

Added an evaluation for trained models.
<img src="/release_note/ai_worker/ffw_policy_evaluation.png" alt="Physical AI Tools Policy Evaluation">
<img src="/release_note/ai_worker/ffw_policy_evaluation_2.png" alt="Physical AI Tools Policy Evaluation">

Added the training module.
- Model policy training is now supported via the Web GUI.
<img src="/release_note/ai_worker/ffw_multi_task.png" alt="Physical AI Tools Multi-Task">
    - Multi-tasking data collection is now supported via the Web GUI.

- It is possible to specify an arbitrary number of task instructions from now on.
<img src="/release_note/ai_worker/ffw_multi_task_2.png" alt="Physical AI Tools Multi-Task">

📦 **ROBOTIS LAB**
> **Version:** `0.1.1`  
> **Released:** 2025-07-16  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/robotis_lab)

Added Sim2Real functionality to the OMY reach task.
- Development and deployment of Sim2Real capabilities with the OMY robot and NVIDIA Isaac Sim
    - ISAAC SIM
<img src="/release_note/ai_worker/ffw_isaac_sim.png" alt="Robotis Lab Isaac Sim">
    - Real World
<img src="/release_note/ai_worker/ffw_real_world.png" alt="Robotis Lab Real World">
</details>


<details>
<summary>2025.07.03</summary>

📦 **AI Worker**  
> **Version:** `1.1.5`  
> **Released:** 2025-06-30  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.1.5)

Updated FFW-SG2 model with latest configurations and improvements.
- Integrated ROS 2 controller for swerve drive support
- Added joystick control functionality for Swerve drive
- Updated robot configuration for improved system stability
<img src="/release_note/ai_worker/ffw_sg2.png" alt="AI Worker SG2">

📦 **Physical AI Tools**
> **Version:** `0.5.7`  
> **Released:** 2025-06-30  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.5.7)

Enhanced Policy Inference via Web UI.
- Enable AI model inference directly from the inference page without CLI
<img src="/release_note/ai_worker/ffw_inference.png" alt="Physical AI Tools Inference">

📦 **ROBOTIS LAB**
> **Version:** `0.1.0`  
> **Released:** 2025-07-01  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/robotis_lab)

Released robotis_lab Repository.
- Newly Created Simulation and RL/IL Research Toolkit for Robotis Robots Built on Isaac Lab
<img src="/release_note/ai_worker/ffw_robotis_lab.png" alt="Robotis Lab">
- Added instructions for the AI Worker homepage
<img src="/release_note/ai_worker/ffw_robotis_lab_documentation.png" alt="Robotis Lab">
</details>


<details>
<summary>2025.06.19</summary>

📦 **AI Worker**  
> **Version:** `1.0.9`  
> **Released:** 2025-06-18  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.0.9)


Added Gazebo simulation support for the AI Worker.
    - Included inertial properties in the URDF and XACRO files for the follower model.
<img src="/release_note/ai_worker/ffw_gazebo_sim.png" alt="AI Worker Gazebo Sim.">

📦 **Physical AI Tools**
> **Version:** `0.5.3`  
> **Released:** 2025-06-18  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.5.3)

Web UI-Based ROS2 Data Acquisition System for LeRobot.
- From CLI to UI: Data collection now operates through an intuitive web-based UI using Physical AI Manager
- From fixed configs to flexible launch: Robot types can now be selected dynamically
- From manual commands to task flow: Users can input tasks, and the system sends commands to the Physical AI Server
- From raw topic reading to buffered capture: Improved image acquisition using efficient buffering
<img src="/release_note/ai_worker/ffw_data_collection.png" alt="Physical AI Tools Data Collection">
    - Episode tracking during data collection
    - Real-time monitoring of system resource (CPU, RAM and disk usage)
    - Easy parameter configuration for each data session
    - Simple control via Start / Stop / Retry / Next / Finish buttons
</details>


<details>
<summary>2025.06.05</summary>

📦 **AI Worker**  
> **Version:** `1.0.8`  
> **Released:** 2025-06-02  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/ai_worker/releases/tag/1.0.8)


AI Worker Hardware REV-4 Release.
<img src="/release_note/ai_worker/ffw_rev4.png" alt="AI Worker REV-4">

- **REV-4**: New outer design, Orin relocated to the chest, ZED camera mounted on the head, and wrist camera with tilt functionality
- **REV-3**: Improved wrist joint
- **REV-2**: Internal development version
- **REV-1**: Initial version equipped with the INSPIRE hand


📦 **Physical AI Tools**
> **Version:** `0.5.2`  
> **Released:** 2025-05-29  
> 🔗 [Repository](https://github.com/ROBOTIS-GIT/physical_ai_tools/releases/tag/0.5.2)

Physical AI Manager - web app for data collection, training, and inference.
- Added a web-based UI tool that shows live image streaming during data collection. You can monitor real-time image streaming through your web browser.
<img src="/release_note/ai_worker/ffw_physical_ai_manager.png" alt="Physical AI Manager">

📦 **AI Worker Website**
> 🔗 [Website](https://ai.robotis.com/)

**AI.ROBOTIS.COM launched**  
- Management of AI.ROBOTIS.COM has begun. The site includes hardware, software, setup guides, and imitation learning manuals for the AI WORKER.
<img src="/release_note/ai_worker/ffw_website.png" alt="Website">
</details>
