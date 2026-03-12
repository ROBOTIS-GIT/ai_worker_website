# Manual Packing Pose Procedure for OMY

## Connect OMY with Wizard 2.0
1. Prepare a PC or laptop with [Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) installed.
2. Connect the Debug USB-C port on the main body panel to the PC using a USB-C cable.
3. Turn on OMY.
4. On the PC, check in Device Manager that the Virtual COM port is recognized.
![connect omy wizard](/specifications/omy/connect_omy_wizard.png)

## Search for OMY on Wizard 2.0
1. In the Wizard 2.0 settings window, select the COM port recognized by the PC.
![search omy on wizard 1](/specifications/omy/search_omy_on_wizard_1.png)
2. Click the DYNAMIXEL search button in Wizard 2.0.
3. Confirm that `OMY-HAT (ID 200)` is recognized.
![search omy on wizard 2](/specifications/omy/search_omy_on_wizard_2.png)

## Search for DYNAMIXEL-Y on Wizard 2.0
1. Turn on `DXL Power Enable (512)` in the `OMY-HAT` control table.
![search dynamixelY 1](/specifications/omy/search_dynamixel_y_1.png)
2. In `Settings -> Search` in Wizard 2.0, check the `6 Mbps` baud rate and set the ID range to search from `1` to `10`.
![search dynamixelY 2](/specifications/omy/search_dynamixel_y_2.png)
3. Click the DYNAMIXEL search button in Wizard 2.0.
4. Confirm that six DYNAMIXEL-Y actuators (ID `1` to `6`) are recognized.
![search dynamixelY 3](/specifications/omy/search_dynamixel_y_3.png)

## Reset DYNAMIXEL-Y multi-turn alarm
1. Check the ID of the DYNAMIXEL-Y actuator that raised the alarm.
2. In the Packet Monitoring window of Wizard 2.0, open the COM port and send the `Clear` command.
![reset multi-turn alarm](/specifications/omy/reset_multiturn_1.png)
3. Confirm that the multi-turn alarm has been cleared.
![reset multi-turn alarm](/specifications/omy/reset_multiturn_2.png)

## Manual operation of DYNAMIXEL-Y
1. Change `Operating Mode (33)` of the target DYNAMIXEL-Y to `Current`.
2. Turn the torque on.
::: warning
Caution: The robot may fall in the direction of gravity, so it must be held by hand.
:::
![omy manual operation 1](/specifications/omy/omy_manual_operation_1.png)
3. Move the target axis to the marked home position.
4. Turn the torque off.
5. Repeat steps 1 to 4 for each axis until the robot reaches the home position or the packaging position (see the [next block](#initialize-the-home-position)).
![omy manual operation 2](/specifications/omy/omy_manual_operation_2.png)

## Initialize the home position
1. Move the robot to the home position.
2. In the Packet Monitoring window of Wizard 2.0, open the COM port and send the `Clear` command to IDs `1` through `6`.
![initialize home position 1](/specifications/omy/reset_multiturn_1.png)
3. Check that the current position of each joint is within `1` degree.
4. For any joint outside that range, refer to the [previous block](#manual-operation-of-dynamixel-y) and manually move it to the home position, then repeat step 2.
![initialize home position 2](/specifications/omy/initialize_home_position.png)

## OMY home position
Move the robot to the home position by aligning it with the home-position slit.
![home position slit](/specifications/omy/home_position_slit_omy.png)

## OMY packaging position
1. Confirm that the robot is in the home position.
2. Move joint 3 to `150` degrees toward the base panel.
3. Move joint 4 to `30` degrees.
![packaging position slit](/specifications/omy/packaging_position_omy.png)