# Dataset Preparation

## Record Your Datasets
Access the `Robot PC` either directly or via SSH, and follow the steps below.
(Refer to the [Setup Guide](/setup) for instructions on how to connect via SSH.)
### 1. Open a terminal and start the Docker container:
```bash
cd ai_worker
./docker/container.sh enter
```

### 2. Launch the ROS 2 teleoperation node inside the Docker container:
```bash
ffw_bg2_ai
```

### 3. Visualize RGB images from the cameras (on the host machine, not inside the Docker container):

  a. Check the AI Worker's serial number

  In this example, the serial number is `SNPR48A0000`.

  b. Open your web browser and go to `http://ffw-{serial number}.local`, replacing `{serial number}` with the serial number from the previous step.

  In this example, the address becomes `http://ffw-SNPR48A0000.local`.

  Once connected, you should see the web UI as shown below.

  <img src="/imitation_learning/web_ui.png" alt="Web UI" style="width: 100%; ">

  c. Click the '+' button to open a pop-up where you can select a camera image topic as shown below:

  <img src="/imitation_learning/web_ui_topic_selection.png" alt="Web UI Topic Selection" style="width: 50%; ">

  d. For example, to visualize the 'camera_left/camera_left/color/image_rect_raw' topic, simply click the button. Once selected, the image stream will appear as shown below:

  <img src="/imitation_learning/web_ui_after_topic_selection.png" alt="Web UI" style="width: 100%; ">

  ::: tip
  Image topics:

  - Left wrist camera: /camera_left/camera_left/color/image_rect_raw

  - Right wrist camera: /camera_right/camera_right/color/image_rect_raw

  - Head camera: /zed/zed_node/rgb/image_rect_color
  :::


### 4. Open a new terminal and navigate to the `lerobot` directory:

```bash
cd ai_worker
./docker/container.sh enter
```

```bash
cd /root/ros2_ws/src/physical_ai_tools/lerobot
```

### 5. Authenticate with Hugging Face

> **Note:** If you do not wish to use Hugging Face, you may skip this step and proceed to the next section, "Prerequisite without Hugging Face."

To create a Hugging Face dataset, you first need to log in using a **write access token**, which can be generated from your [Hugging Face settings](https://huggingface.co/settings/tokens):

```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN} --add-to-git-credential
```

Store your Hugging Face username in a variable:

```bash
export HF_USER=$(huggingface-cli whoami | head -n 1)
echo $HF_USER
```
You should see an output similar to the following:

```
YourUserName
```
#### Prerequisite without Hugging Face

If you do not intend to use the Hugging Face Hub, you can still define a placeholder username for local dataset handling:
```bash
export HF_USER=AnyNameYouWant
echo $HF_USER
```
You should see an output like:

```
AnyNameYouWant
```

::: tip
- Make sure to replace `${HF_USER}` with your actual Hugging Face username.
:::

### 6. Run the following command to start recording your Hugging Face dataset:
```bash
cd /root/ros2_ws/src/physical_ai_tools/lerobot
```

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=ffw \
  --control.type=record \
  --control.single_task="pick and place objects" \
  --control.fps=15 \
  --control.repo_id=${HF_USER}/ffw_test \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=30 \
  --control.reset_time_s=10 \
  --control.num_episodes=10 \
  --control.push_to_hub=true \
  --control.play_sounds=false
```

::: tip
- To save the dataset locally without uploading to the Hugging Face Hub, set `--control.push_to_hub=false`. This option is essential if you choose not to use Hugging Face.
- If you are controlling the robot remotely via VSCode or SSH, the `arrow keys` may not work during data recording due to a pynput limitation. In this case, it's recommended to set --control.episode_time_s and --control.reset_time_s appropriately.
- To use the `arrow keys` for teleoperation, connect a monitor and keyboard directly to the robot (Right arrow key: Save immediately, Left arrow key: Cancel).
:::

#### Key Parameters to Customize

To create your own dataset, here are some important parameters you may want to adjust:

| Parameter                  | Description | Example |
|----------------------------|-------------|---------|
| `--control.repo_id`        | The Hugging Face dataset repository ID in the format `<username>/<dataset_name>` | `username/ffw_pick_place` |
| `--control.single_task`    | The name of the task you are performing | "pick and place objects" |
| `--control.fps`            | Frame rate for dataset recording | 15 (recommended) |
| `--control.episode_time_s` | Duration (in seconds) to record each episode | 30-60 for simple tasks |
| `--control.reset_time_s`   | Time allocated (in seconds) for resetting between episodes | 10-20 seconds |
| `--control.num_episodes`   | Total number of episodes to record | 10-50 depending on task complexity |

Of course, you can modify additional parameters as needed to fit your specific use case.

The dataset is located at:

`~/ai_worker/docker/huggingface/lerobot/`

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
