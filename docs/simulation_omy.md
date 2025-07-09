# Launch Gazebo

> ⚠️ **Note:** It is recommended to run on a PC other than OMY.


## 1. Prepare the OM container

Refer to `Setup Guide` to configure the OM container environment.


## 2. Enable GUI access

Open a new terminal and run the following command:

```bash
xhost +
```


## 3. Access the running container

```bash
./docker/container.sh enter
```


## 4. Launch Gazebo

Launch the simulation environment using:

```bash
ros2 launch open_manipulator_bringup omy_f3m_gazebo.launch.py
```


## 5. Run MoveIt and GUI in simulation

Refer to the Operation page to control the robot in Gazebo.


## Simulation Views

### OMY_F3M

![OMY F3M Simulation](/simulation/omy_f3m_gazebo.png)


### OMY_3M

![OMY 3M Simulation](/simulation/omy_3m_gazebo.png)

