# Dataset Preparation with Web UI

## Prerequisites

This section describes the necessary setup steps before starting data preparation.  
We will use a total of three terminals, each responsible for a different ROS 2 node:
- **[Terminal 1 - Teleoperation Node](/omy/dataset_preparation_with_web_ui_omy#_1-launch-the-ros-2-teleoperation-node)**  
This terminal launches the ROS 2 teleoperation node that controls the manipulator.

- **[Terminal 2 – Camera Node](/omy/dataset_preparation_with_web_ui_omy#_2-camera-setup)**  
This terminal launches the RealSense camera node (or another supported camera) to stream visual data.

- **[Terminal 3 – Physical AI Server](/omy/dataset_preparation_with_web_ui_omy#_3-launch-physical-ai-server)**  
This terminal starts the backend server that connects to the Web UI.

Make sure to follow the setup instructions for each terminal in the order shown.

::: info
If you're setting up for the first time, please navigate to the path where the Docker script is located and run `./container.sh start` before using `./container.sh enter` to access the container.
:::

### 1. Launch the ROS 2 teleoperation node
a. Access the Robot PC either directly or via SSH. For SSH connection instructions, refer to the [SSH connection](/omy/setup_guide_omy#ssh-connection).  

b. Enter the Docker container:
```bash
cd /data/docker/open_manipulator/docker && ./container.sh enter
```
c. Then, launch the ROS 2 teleoperation node using the appropriate command for your robot type:
```bash
ros2 launch open_manipulator_bringup omy_ai.launch.py
```
::: warning
Executing the code will cause OMY to move immediately. Please stay clear and be cautious.
:::

### 2. Camera Setup

a. Open a new terminal on your host machine and enter the Docker container:
```bash
cd open_manipulator/docker && ./container.sh enter
```
b. Set a consistent `ROS_DOMAIN_ID` across terminals to enable ROS 2 node communication.
```bash
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
source ~/.bashrc
```
c. Launch the camera node:
```bash
ros2 launch realsense2_camera rs_launch.py camera_name:='cam_wrist'
```

You can also use other camera models such as ZED2 or USB cameras, if needed.


### 3. Launch Physical AI Server

::: info
The _Physical AI Server_ is the backend that connects with the Web UI. It should be running to use the interface for data recording.
:::

#### a. Start and enter the Docker container:

Open a new terminal on your host machine and move to **phisical_ai_tools/docker** directory. 

```bash
cd physical_ai_tools/docker
```

Start the **Physical AI Tools** Docker container with the following command:

::: info
Run `./container.sh start` only when starting the container for the first time.  
If you have already started the container, you can skip this step.
:::

```bash
./container.sh start
```

Enter the Docker container:
```bash
./container.sh enter
```

#### b. Setup ROS DOMAIN ID
Set a consistent `ROS_DOMAIN_ID` across terminals to enable ROS 2 node communication.
```bash
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
source ~/.bashrc
```
#### c. Launch Physical AI Server

Launch Physical AI Server with the following command

```bash
ros2 launch physical_ai_server physical_ai_server_bringup.launch.py
```
Or, use shortcut command:

```bash
ai_server
```

### 4. Open the Web UI

::: info
This step must be performed on the **host machine** (or another device on the same network).
:::


#### Access the Web UI in Your Browser
Please enter the IP address of the PC running Physical AI into your web browser.  
If you are running the server on the same computer, you can simply enter `localhost` instead.


For example:
- `192.168.0.1`  

  <img src="/imitation_learning/web_ui_home_page_address.png" alt="Web UI" style="width: 100%; ">
  <p style="text-align: center;"><em>Enter the IP Address</em></p>  

Once accessed, the following screen will appear with the `Disconnected` status.

  <img src="/imitation_learning/web_ui_initial_home_page.png" alt="Web UI" style="width: 100%; ">
  <p style="text-align: center;"><em>Initial Screen</em></p>  


## Record your dataset

### 1. Select the Robot Type

On the **Home** page, select the type of robot you are using.

<div class="UI" style="width: 60%; margin: 0 auto;">
  <img src="/imitation_learning/web_ui_robot_type_selection.png" alt="Web UI" style="width: 100%; ">
</div>

After clicking the **Set Robot Type** button, the status will change to `Connected`.

   <img src="/imitation_learning/web_ui_connected_home_page.png" alt="Web UI" style="width: 100%; ">
  <p style="text-align: center;"><em>Connected Screen</em></p>  

### 2. Go to `Record` page

::: info
You cannot access **Record** page unless a robot type has been selected on the **Home** page.
Please ensure that the robot type is selected before proceeding.
:::
After the status changes to `Connected`, access to the **Record** page will be available through the button on the left sidebar.  

The **Record** page is divided into three main sections:

- **Image Streaming Area** (Center): Displays real-time camera streams from the robot.

- **Task Info Panel** (Right side): Enter task name, task instruction, time values, etc. [(details)](#_4-enter-task-information)

- **Control Panel** (Bottom): Start/stop recording and manage data collection. [(details)](#_5-start-recording)

The selected robot type is also displayed in the top left corner.

  <img src="/imitation_learning/web_ui_record_page.png" alt="Web UI" style="width: 100%; ">
  <p style="text-align: center;"><em>Record Screen</em></p> 

### 3. Visualize RGB images from the cameras:

The image stream is displayed automatically upon entering the **Record** page.
You can remove the current stream and select a different one as needed.

To change the image topic:

1. Click the `+` button in the Image Streaming Area.
2. Choose a topic from the popup window.

  <div class="UI" style="width: 60%; margin: 0 auto;">
  <img src="/imitation_learning/web_ui_select_image_topic.png" alt="Web UI" style="width: 100%; ">
</div>

### 4. Enter Task Information:

Fill out the task-related fields in the **Task Info Panel**, located on the right side of the **Record** page. The panel will initially appear blank, as shown in the left image, and should be completed to match the filled example shown on the right.
<div style="display: flex; justify-content: center; gap: 40px;">
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui_task_info_initial.png" alt="Web UI" style="width: 100%; max-width: 300px;">
    <p><em>Initial Panel</em></p>
    
  </div>
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui_task_info_filled.png" alt="Web UI" style="width: 94%; max-width: 300px;">
    <p><em>Filled Panel</em></p>
  </div>
</div>


Refer to the descriptions below to complete each field in the Task Information Panel:

::: details :point_right: Task Information Field Descriptions
| Item                 | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| -------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Task Name**        | It is used for the folder name of the dataset                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| **Task Instruction** | It is a sentence that instructs the robot what action to perform, such as "pick and place object"                                                                                                                                                                                                                                                                                                                                                                                 |
| **Push to hub**      | If you want to push the dataset to the Hugging Face Hub, check this box. This allows sharing and training. <br>**To push to the Hugging Face Hub, you need to:**<br>1. Have a Hugging Face account<br>2. Have the necessary permissions to push to the repository                                                                                                                                                                                                                 |
| **Private Mode**     | Only available when "Push to hub" is checked. Keeps your dataset private on Hugging Face.                                                                                                                                                                                                                                                                                                                                                                                         |
| **User ID**          | Your Hugging Face account username, also used as the folder name for the dataset.<br>- If **not** using Hugging Face, you can use any name.<br>- If using Hugging Face, click `Load` to select from registered User IDs.<br>- If no User IDs or want a different one, click `Change` and enter your Hugging Face token.<br>- The Hugging Face token must have both **read** and **write** permissions enabled.<img src="/imitation_learning/web_ui_enter_hf_token.png" alt="Web UI" style="width: 100%; "><br>- If you have a registered account but haven't checked "Push to hub", you can either:<br>1. Load a registered User ID<br>2. Manually enter any User ID  |
| **FPS**              | Frame rate for dataset recording. Recommended value is 15.                                                                                                                                                                                                                                                                                                                                                                                                                        |
| **Tags**             | Keywords to categorize and organize your dataset in the hub. Multiple tags can be added. Useful for searching or filtering later.                                                                                                                                                                                                                                                                                                                                                 |
| **Warmup Time**      | Duration (in seconds) to warm up the robot before starting the recording                                                                                                                                                                                                                                                                                                                                                                                                          |
| **Episode Time**     | Duration (in seconds) to record each episode                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| **Reset Time**       | Time allocated (in seconds) for resetting between episodes                                                                                                                                                                                                                                                                                                                                                                                                                        |
| **Num Episodes**     | Total number of episodes to record                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| **Optimized Save**   | If enabled, uses RAM for faster dataset encoding processing                                                                                                                                                                                                                                                                                                                                                                                                                       |
:::

### 5. Start Recording:

To begin recording, use the **Control Panel** located at the bottom of the **Record** page:

  <img src="/imitation_learning/web_ui_control_panel.png" alt="Web UI" style="width: 100%; ">

#### Step 1: Click the `Start` Button

Click the `Start` button to begin the recording session. The system will automatically:

   - Warm up the robot for the specified **Warm-up Time**
   - Record each episode for the specified **Episode Time**
   - Wait for the specified **Reset Time** between episodes
   - Repeat the above steps for the specified **Number of Episodes**

#### Step 2: Monitor and Control During Recording


While recording is in progress, the following controls are available:

   - The `Stop` button saves the current episode in progress and stops the recording. If you press the `Start` button again, recording will resume from the next episode.
   - The `Retry` button cancels the current episode and restarts recording for that episode
   - The `Next` button ends the current episode early and moves to the next episode
   - The `Finish` button ends the recording session and saves the current dataset, regardless of the remaining number of episodes

::: info
- The current recording stage is displayed in the control panel:

  - 📍 **Ready to start** — Standby mode before recording begins
  - 🔥 **Warm-up in progress** — Robot is warming up
  - 🔴 **Recording in progress** — Capturing data
  - 🏠 **Reset in progress** — Reset time between episodes
  - 💾 **Saving...** — Encoding and saving the episode
:::

::: tip
- Before you start, keep the robot in a safe and ready position during warm-up
- Make sure all required fields in the Task Info Panel are filled in
- Keep the robot in a safe and ready position during warm-up
- Monitor progress and status updates through the Web UI
- System resources (CPU, RAM, Storage) are displayed during recording
:::

#### Step 3. After recording:
   - The dataset will be saved on the **host machine**, not inside the **Docker container**.
   - If "Push to hub" is enabled, the dataset will be uploaded to Hugging Face.
   - The recorded dataset can be accessed at the following path from the **host environment**:

```bash
~/physical_ai_tools/docker/huggingface/lerobot
```


## Dataset Visualization

Once data collection is complete, you can preview and inspect your recorded dataset using the following steps.  
a. Access the **physical_ai_tools** Docker environment:
```bash
cd physical_ai_tools/docker && ./container.sh enter
```
b. Move to the directory where the dataset is stored inside the container:
```bash
cd /root/ros2_ws/src/physical_ai_tools/lerobot
```
c. Run the following command to launch the dataset visualization tool:
```bash
python lerobot/scripts/visualize_dataset_html.py \
  --host 0.0.0.0 \
  --port 9091 \
  --repo-id YOUR_REPO_ID
```
  - Replace **YOUR_REPO_ID** with the dataset ID found under the **User ID** section in the right sidebar.
  - For example: `ROBOTIS/omy_f3m_Test` 
  <div class="UI" style="width: 50%; margin: 0 auto;">
  <img src="/imitation_learning/web_ui_task_info_repo_id.png" alt="Web UI" style="width: 100%; ">
</div>

You should see an output similar to the following:

```
Fetching 4 files: 100%|██████████| 4/4 [00:00<00:00, 3457.79it/s]
.gitattributes: 100%|██████████| 2.46k/2.46k [00:00<00:00, 45.9MB/s]
Fetching 126 files: 100%|██████████| 126/126 [00:00<00:00, 266.66it/s]
Resolving data files: 100%|██████████| 30/30 [00:00<00:00, 662258.53it/s]
INFO 2025-05-15 16:18:07 set_html.py:364 Output directory already exists. Loading from it: '/tmp/lerobot_visualize_dataset_uo6ddbb1'
 * Serving Flask app 'visualize_dataset_html'
 * Debug mode: off
INFO 2025-05-15 16:18:07 _internal.py:97 WARNING: This is a development server. Do not use it in a production deployment. Use a production WSGI server instead.
 * Running on http://127.0.0.1:9091
INFO 2025-05-15 16:18:07 _internal.py:97 Press CTRL+C to quit
```
Access http://127.0.0.1:9091 to view the dataset. You should see a web interface similar to the one below:
<img src="/imitation_learning/data_visualization.png" alt="Web UI" style="width: 100%; ">


::: tip
Once the server is running, open [http://127.0.0.1:9091](http://127.0.0.1:9091) in your browser to preview the dataset.
:::

::: tip
On a device connected to the same network as the host machine, open `http://omy-{serial number}.local:9091` in your browser to preview the dataset.

For example, `http://omy-SNPR48A0000.local:9091`.
:::
