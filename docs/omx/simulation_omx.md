# Simulation

## Overview

OMX supports comprehensive simulation environments for testing and development. This guide covers the available simulation platforms and how to use them effectively.

## Supported Platforms

### Gazebo
- Full physics simulation with realistic dynamics
- Support for sensors and environment modeling
- Integration with MoveIt for motion planning

### Isaac Sim/Lab
- NVIDIA's high-performance simulation platform
- GPU-accelerated physics and rendering
- Advanced AI training capabilities

### MuJoCo
- Fast physics simulation for research
- Support for reinforcement learning
- Lightweight and efficient

## Getting Started

### Prerequisites
- ROS 2 Humble or later
- Docker container (recommended)
- GPU support (for Isaac Sim)

### Basic Setup
```bash
# Launch OMX in Gazebo
ros2 launch open_manipulator_gazebo omx_3m_gazebo.launch.py

# Launch with RViz
ros2 launch open_manipulator_gazebo omx_3m_gazebo_rviz.launch.py
```

## Features

### Physics Simulation
- Realistic joint dynamics and constraints
- Collision detection and response
- Gravity and friction modeling

### Sensor Simulation
- Camera and depth sensor simulation
- Force/torque sensor support
- Environmental sensor integration

### Control Interface
- ROS 2 topic-based control
- Joint trajectory planning
- Real-time feedback

## Examples

### Basic Movement
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class OMXSimController(Node):
    def __init__(self):
        super().__init__('omx_sim_controller')
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/omx/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
    def move_to_position(self, joint_positions):
        msg = Float64MultiArray()
        msg.data = joint_positions
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = OMXSimController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Resources

- [Gazebo Documentation](http://gazebosim.org/docs)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [OMX GitHub Repository](https://github.com/ROBOTIS-GIT/open_manipulator) 