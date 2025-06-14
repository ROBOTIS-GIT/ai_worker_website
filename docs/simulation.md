# Simulation

## Overview
The AI Worker simulation environment is designed to test and validate robot functionalities in a controlled virtual setting. This allows for comprehensive evaluation of its capabilities across a variety of tasks before deployment on the physical hardware. The simulation ecosystem leverages industry-standard tools including ROS 2, Google DeepMind's MuJoCo, and NVIDIA Isaac Sim, providing diverse options for development and research.

## Simulation for Reinforcement Learning
Our simulation environments are particularly well-suited for reinforcement learning (RL). They provide realistic physics and sensor feedback, enabling the AI Worker to learn and adapt its behaviors through interaction. This is crucial for developing complex manipulation and navigation skills.

## Available Simulation Models and Environments

The AI Worker can be simulated in various environments using different model formats. These models include all necessary components of the AI Worker, such as the mobile base, dual arms, and sensors, allowing for comprehensive testing.

### Gazebo / RViz (URDF/SDF)
For seamless integration with the ROS ecosystem, URDF (Unified Robot Description Format) and SDF (Simulation Description Format) models are provided. These are primarily used with Gazebo for physics simulation and RViz for 3D visualization of robot states and sensor data.

![AI Worker URDF model in RViz or Gazebo](/simulation/simulation_urdf.png)

*   **Model Formats**: [URDF](https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/URDF-Main.html), [SDF](http://sdformat.org/)
*   **Primary Tools**: [Gazebo Sim](https://gazebosim.org/), [RViz](https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/RViz-Main.html)
*   **Purpose**: General robotics simulation, ROS-based algorithm testing, visualization.
*   **Link**: [AI Worker URDF on GitHub](https://github.com/ROBOTIS-GIT/ai_worker/tree/main/ffw_description/urdf)

#### How to Launch

You can launch Gazebo and RViz individually, or launch RViz together with the physical hardware.

*   **Launch Gazebo simulation:**
    Use this command to run a dynamic simulation of the AI Worker in the Gazebo simulator.
    ```bash
    # Launch Gazebo with the AI Worker Follower model (FFW-BG2)
    ros2 launch ffw_bringup ffw_bg2_follower_gazebo.launch.py
    ```

*   **View model in RViz only:**
    Use this command when you want to visualize the robot model in RViz without running a full simulation or the physical hardware.
    ```bash
    # Launch RViz with the AI Worker model
    ros2 launch ffw_description model_view.launch.py
    ```

*   **Launch RViz with physical hardware:**
    Use this command when you want to operate the physical AI Worker hardware and monitor its status in RViz. This command will launch RViz alongside the hardware interface.
    ```bash
    # Launch the AI Worker Follower (FFW-BG2) hardware interface and RViz
    ros2 launch ffw_bringup ffw_bg2_follower_ai.launch.py
    ```
    *   **Note:** This command is the same as the `Follower` launch command in the [`Teleoperation Guide`](/operation#option-2-separate-launches) and will also launch the cameras by default. To run without cameras, you can add the `launch_cameras:=false` argument.

### MuJoCo (MJCF)
For high-fidelity physics simulation, particularly favored in reinforcement learning research and biomechanics, models in MJCF (MuJoCo XML Format) are available. MuJoCo offers fast and accurate physics, ideal for training complex behaviors.

![AI Worker MJCF model in MuJoCo](/simulation/simulation_mujoco.png)

*   **Model Format**: [MJCF](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
*   **Primary Tool**: [MuJoCo (by Google DeepMind)](https://mujoco.org/)
*   **Purpose**: High-fidelity physics simulation, reinforcement learning, advanced dynamics research.
*   **Link**: [ROBOTIS MuJoCo Menagerie on GitHub](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)

### NVIDIA Isaac Sim (USD)
Support for NVIDIA Isaac Sim is an exciting upcoming feature. Isaac Sim utilizes the USD (Universal Scene Description) format for photorealistic rendering and advanced physics simulation, tightly integrated with NVIDIA's AI and robotics SDKs.

![AI Worker USD model in NVIDIA Isaac Sim (Coming Soon)](/simulation/simulation_isaac_sim.png)

*   **Model Format**: [USD](https://docs.isaacsim.omniverse.nvidia.com/latest/omniverse_usd/open_usd.html)
*   **Primary Tool**: [NVIDIA Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/)
*   **Purpose**: Photorealistic simulation, advanced physics, synthetic data generation, AI-driven robotics development.
*   **Status**: We are actively developing support for NVIDIA Isaac Sim, with a targeted release in alignment with the launch of NVIDIA Isaac Sim 5.0.
*   **Link**: (Coming Soon)
