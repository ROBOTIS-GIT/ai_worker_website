# Operation

## Basic Operation

### Powering On
1. Connect the power supply (24VDC)
2. Press the power button until the indicator turns white
3. Wait for the system to boot

### Connecting via SSH
- Use the hostname based on the serial number (e.g., `omx-SNPR44B9999.local`)
- Default user: `root`

### Starting ROS 2 Nodes
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash
# Source workspace
source ~/open_manipulator/install/setup.bash
# Launch main node
ros2 launch open_manipulator_bringup omx_3m_bringup.launch.py
```

## Safety Precautions
- Ensure the workspace is clear before operation
- Do not touch the manipulator while powered
- Use emergency stop if needed

## Shutdown Procedure
1. Stop all running nodes
2. Power off the system using the power button

## Maintenance
- Regularly check for loose cables and connectors
- Clean the manipulator with a dry cloth
- Update firmware and software as recommended

## Troubleshooting
- See the [Issues](./issues_omx.md) page for common problems and solutions
- Contact support for unresolved issues

## Documentation
- [Official Documentation](https://docs.robotis.com)
- [GitHub Repository](https://github.com/ROBOTIS-GIT/open_manipulator) 