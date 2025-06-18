# Dataset Preparation with Web UI

## Prerequisites

Access the `Robot PC` either directly or via SSH, and follow the steps below.
(Refer to the [Setup Guide](/setup) for instructions on how to connect via SSH.)

### 1. Launch the ROS 2 teleoperation node:

Open a new terminal and enter the Docker container

```bash
cd ai_worker
./docker/container.sh enter
```

Launch the ROS 2 teleoperation node

```bash
ffw_bg2_ai
```

### 2. Launch Physical AI Server:

Open another terminal and enter the Docker container

```bash
cd ai_worker
./docker/container.sh enter
```

Launch Physical AI Server with the following command

```bash
ros2 launch physical_ai_server physical_ai_server_bringup.launch.py
```

or use the shortcut command:

```bash
ai_server
```

### 3. Open Web UI (Physical AI Manager):

> [!WARNING] On the host machine, not inside the Docker container

a. Check the AI Worker's serial number

In this example, the serial number is `SNPR48A0000`.

b. Open your web browser and go to `http://ffw-{serial number}.local`, replacing `{serial number}` with the serial number from the previous step.

In this example, the address becomes `http://ffw-SNPR48A0000.local`.

Once connected, you should see the web UI as shown below.

  <img src="/imitation_learning/web_ui_home_page.png" alt="Web UI" style="width: 100%; ">

## Record your dataset

### 1. Select robot type

Select robot type in `Home` page

  <img src="/imitation_learning/web_ui_robot_type_selection.png" alt="Web UI" style="width: 50%; ">

### 2. Go to `Record` page

> [!note]  
> You cannot go to **Record** page unless you have set the robot type on the **Home** page.  
> Please make sure to select the robot type on the Home page.

The Record page consists of three main sections:

- **Image Streaming Area** (Center): View real-time streams from the robot cameras.

- **Task Info Panel** (Right side): Enter task name, task instruction, time values, etc. [(details)](#_4-enter-task-information)

- **Control Panel** (Bottom): Start/stop recording and manage data collection. [(details)](#_5-start-recording)

The selected robot type is also displayed in the top left corner.

  <img src="/imitation_learning/web_ui_record_page.png" alt="Web UI" style="width: 100%; ">

### 3. Visualize RGB images from the cameras:

The image streaming will be displayed automatically. You can remove the currently displayed image stream and select a different image stream to display. To select an image topic, click the **+** button and choose from the popup window.

  <img src="/imitation_learning/web_ui_select_image_topic.png" alt="Web UI" style="width: 50%; ">

### 4. Enter Task Information:

Enter the task information in the panel located on the right side of the page.

  <img src="/imitation_learning/web_ui_task_info.png" alt="Web UI" style="width: 50%; ">

| Item                 | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| -------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Task Name**        | It is used for the folder name of the dataset                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| **Task Instruction** | It is a sentence that instructs the robot what action to perform, such as "pick and place object"                                                                                                                                                                                                                                                                                                                                                                                 |
| **Push to hub**      | If you want to push the dataset to the Hugging Face Hub, check this box. This allows sharing and training. <br>**To push to the Hugging Face Hub, you need to:**<br>1. Have a Hugging Face account<br>2. Have the necessary permissions to push to the repository                                                                                                                                                                                                                 |
| **Private Mode**     | Only available when "Push to hub" is checked. Keeps your dataset private on Hugging Face.                                                                                                                                                                                                                                                                                                                                                                                         |
| **User ID**          | Your Hugging Face account username, also used as the folder name for the dataset.<br>- If **not** using Hugging Face, you can use any name.<br>- If using Hugging Face, click "Load" to select from registered User IDs.<br>- If no User IDs or want a different one, click "Change" and enter your Hugging Face token.<br>- If you have a registered account but haven't checked "Push to hub", you can either:<br>1. Load a registered User ID<br>2. Manually enter any User ID |
| **FPS**              | Frame rate for dataset recording. Recommended value is 15.                                                                                                                                                                                                                                                                                                                                                                                                                        |
| **Tags**             | Keywords to categorize and organize your dataset in the hub. Multiple tags can be added. Useful for searching or filtering later.                                                                                                                                                                                                                                                                                                                                                 |
| **Warmup Time**      | Duration (in seconds) to warm up the robot before starting the recording                                                                                                                                                                                                                                                                                                                                                                                                          |
| **Episode Time**     | Duration (in seconds) to record each episode                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| **Reset Time**       | Time allocated (in seconds) for resetting between episodes                                                                                                                                                                                                                                                                                                                                                                                                                        |
| **Num Episodes**     | Total number of episodes to record                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| **Optimized Save**   | If enabled, uses RAM for faster dataset encoding processing                                                                                                                                                                                                                                                                                                                                                                                                                       |

### 5. Start Recording:

To start recording, use the control panel at the bottom of the page:

  <img src="/imitation_learning/web_ui_control_panel.png" alt="Web UI" style="width: 100%; ">

1. Click the `Start` button to begin recording. The system will:

   - Warm up the robot for the specified warmup time
   - Record each episode for the specified episode time
   - Reset between episodes for the specified reset time
   - Repeat for the specified number of episodes

2. During recording:

   - The `Stop` button allows you to stop recording at any time
   - The `Retry` button cancels the current episode and restarts recording for that episode
   - The `Next` button ends the current episode early and moves to the next episode
   - The `Finish` button ends the recording session and saves the current dataset, regardless of the remaining number of episodes

::: info

- The current recording stage is displayed in the control panel:

  - 📍 Ready to start (Standby)
  - 🔥 Warmup in progress (Warmup Time)
  - 🏠 Reset in progress (Reset Time)
  - 🔴 Recording in progress (Episode Time)
  - 💾 Saving... (Encoding the episode)

  :::

  ::: tip

- Make sure all required fields are filled before starting
- Keep the robot in a safe position during warmup
- Monitor the recording progress through the web UI
- You can monitor system resources (CPU, RAM, Storage) during recording
  :::

3. After recording:
   - The dataset will be saved locally
   - If "Push to hub" is enabled, the dataset will be uploaded to Hugging Face
   - You can find the recorded dataset in the specified location

## Dataset Visualization

Once data collection is complete, you can preview and inspect your recorded dataset using the following command:

```bash
cd /root/ros2_ws/src/physical_ai_tools/lerobot
```

```bash
python lerobot/scripts/visualize_dataset_html.py \
  --host 0.0.0.0 \
  --port 9091 \
  --repo-id ${HF_USER}/ffw_test
```

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

::: tip
Once the server is running, open [http://127.0.0.1:9091](http://127.0.0.1:9091) in your browser to preview the dataset.
:::
