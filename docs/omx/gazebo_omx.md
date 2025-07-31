# Gazebo

## Overview

OMX supports simulation in Gazebo, a powerful robotics simulation environment. This allows you to test and develop your robotics applications in a virtual environment before deploying them on physical hardware.

## Installation

### Prerequisites
- Ubuntu 22.04 or later
- ROS 2 Humble or later
- Gazebo Classic (gazebo) or Gazebo (gz)

### Install Gazebo
```bash
# Install Gazebo Classic
sudo apt update
sudo apt install gazebo libgazebo-dev

# Or install Gazebo (newer version)
sudo apt install gz
```

### Install OMX Gazebo Packages
```bash
# Clone the repository
git clone https://github.com/ROBOTIS-GIT/open_manipulator

# Build the workspace
cd open_manipulator
colcon build --symlink-install
source install/setup.bash
```

## Launching OMX in Gazebo

### Basic Launch
```bash
# Launch OMX-3M in Gazebo
ros2 launch open_manipulator_gazebo omx_3m_gazebo.launch.py

# Launch OMX-F3M in Gazebo
ros2 launch open_manipulator_gazebo omx_f3m_gazebo.launch.py
```

### Launch with GUI
```bash
# Launch with RViz for visualization
ros2 launch open_manipulator_gazebo omx_3m_gazebo_rviz.launch.py
```

## World Files

### Default World
The default world includes:
- OMX manipulator
- Ground plane
- Basic lighting
- Physics engine configuration

### Custom Worlds
You can create custom worlds by modifying the world files located in:
```
open_manipulator_gazebo/worlds/
```

## Control

### Joint State Publisher
```bash
# Publish joint states
ros2 run joint_state_publisher joint_state_publisher
```

### MoveIt Integration
```bash
# Launch MoveIt with OMX
ros2 launch open_manipulator_moveit omx_3m_moveit.launch.py
```

### Teleop Control
```bash
# Launch teleop node
ros2 run open_manipulator_teleop open_manipulator_teleop_keyboard
```

## Visualization

### RViz
```bash
# Launch RViz with OMX configuration
ros2 launch open_manipulator_description omx_3m_rviz.launch.py
```

### Gazebo GUI
- Use the Gazebo GUI for 3D visualization
- Control camera angles and views
- Inspect joint states and properties

## Troubleshooting

### Common Issues

#### Gazebo Not Starting
1. Check if Gazebo is properly installed
2. Verify ROS 2 environment is sourced
3. Check for missing dependencies

#### Model Not Loading
1. Ensure URDF files are in the correct location
2. Check file permissions
3. Verify model paths in launch files

#### Physics Issues
1. Adjust physics parameters in world file
2. Check collision models
3. Verify joint limits and dynamics

## Advanced Configuration

### Physics Parameters
Modify physics parameters in the world file:
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

### Sensor Configuration
Configure sensors in the URDF:
```xml
<sensor type="camera" name="camera">
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### Plugin Configuration
Add custom plugins for advanced functionality:
```xml
<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
  <robotNamespace>/omx</robotNamespace>
</plugin>
```

## Examples

### Basic Movement
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class OMXController(Node):
    def __init__(self):
        super().__init__('omx_controller')
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/omx/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
    def move_joints(self, joint_positions):
        msg = Float64MultiArray()
        msg.data = joint_positions
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = OMXController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Trajectory Planning
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class OMXTrajectoryController(Node):
    def __init__(self):
        super().__init__('omx_trajectory_controller')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/omx/joint_trajectory_controller/joint_trajectory',
            10
        )
        
    def execute_trajectory(self, waypoints):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        for i, waypoint in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.time_from_start.sec = i
            msg.points.append(point)
            
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = OMXTrajectoryController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Resources

- [Gazebo Documentation](http://gazebosim.org/docs)
- [ROS 2 Gazebo Tutorials](https://docs.ros.org/en/humble/Tutorials/Gazebo.html)
- [OMX GitHub Repository](https://github.com/ROBOTIS-GIT/open_manipulator) 