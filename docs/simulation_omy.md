# Launch Gazebo

::: info
It is recommended to run this on a PC other than the OMY-AI device.
:::

## 1. Prepare the OM Container

Refer to the `Setup Guide` to configure the OM container environment.


## 2. Enable GUI Access

Open a new terminal and run the following command:

```bash
xhost +
```


## 3. Access the Running Container

```bash
./docker/container.sh enter
```


## 4. Launch Gazebo

Launch the simulation environment using:

```bash
ros2 launch open_manipulator_bringup omy_f3m_gazebo.launch.py
```


## 5. Run MoveIt and GUI in Simulation

Refer to the **Operation** page to control the robot within the Gazebo environment.


## Simulation Views

### OMY-AI_F3M

![OMY-AI F3M Simulation](/simulation/omy_f3m_gazebo.png)


### OMY-AI_3M

![OMY-AI 3M Simulation](/simulation/omy_3m_gazebo.png)

