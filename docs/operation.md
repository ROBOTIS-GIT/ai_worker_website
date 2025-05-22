# Teleoperation
- Place the `Follower` in a spacious area.
- Wear the `Leader` device.


![Leader's wearing appearance](/quick_start_guide/leader_wearing_appearance.png)


###### How to wear the `Leader` :
1. Put both arms through the `Leader`'s `shoulder straps`.
2. Fasten the chest belt buckle and `hip belt buckle`.
3. Adjust the length of the `shoulder straps`, `chest belt`, and `hip belt` so that the `Leader` is securely fixed on your back.
4. Face the `red sticker` attached to the `Leader` to set the initial position.

### Running Teleoperation

1. Open a new terminal and run the following command:

- To start teleoperation, run the launch file:
    ```bash
    ros2 launch ffw_bringup ffw_teleop_with_rh.launch.py
    ```
    or
    ```bash
    bringup
    ```

- If you want to run the `Leader` and `Follower` separately in different terminals:
    1. To launch the teleoperation `Leader`:
         ```bash
         ros2 launch ffw_bringup hardware_leader_with_rh.launch.py
         ```
         or
         ``` bash
         leader
         ```
    2. To launch the teleoperation `Follower`:
         ```bash
         ros2 launch ffw_bringup hardware_follower_with_rh.launch.py
         ```
         or
         ``` bash
         follower
         ```
