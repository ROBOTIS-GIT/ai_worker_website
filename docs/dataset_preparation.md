# Dataset Preparation

## Preparation Before Dataset Creation

### Authenticate with Hugging Face

> **Note:** If you do not wish to use Hugging Face, you may skip this step. Instructions for preparing the dataset without Hugging Face will be provided in the following sections.

To create a Hugging Face dataset, you first need to log in using a **write access token**, which can be generated from your [Hugging Face settings](https://huggingface.co/settings/tokens):

```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN} --add-to-git-credential
```

Store your Hugging Face username in a variable:

```bash
export HF_USER=$(huggingface-cli whoami | head -n 1)
echo $HF_USER
```

## Record Your Datasets

### 1. Open a terminal and run Docker container:
```bash
cd ai_worker
./docker/container.sh enter
```

### 2. Launch the ROS 2 teleoperation node inside the Docker container:
```bash
bringup
```

### 3. Visualize RGB images from the cameras:

  1. You can find the AI Worker's serial number in the `~/.serial_number` file. This serial number also serves as the device's `hostname`, which you'll use to access the web interface.

  2. Open your web browser and go to `http://{hostname}`, replacing `{hostname}` with the serial number you found in step 1. You should then see the web UI, as shown below.

  <img src="/imitation_learning/web_ui.png" alt="Web UI" style="width: 100%; ">

  3. Click the '+' button to open a pop-up where you can select a camera image topic as shown below:

  <img src="/imitation_learning/web_ui_topic_selection.png" alt="Web UI Topic Selection" style="width: 50%; ">

  4. For example, to visualize the 'camera_left/camera_left/color/image_rect_raw' topic, simply click the button.

### 4. Open a new terminal and navigate to the `lerobot` directory:

```bash
cd ai_worker
./docker/container.sh enter
```

```bash
cd /root/colcon_ws/src/physical_ai_tools/lerobot
```

### 5. Run the following command to start recording your Hugging Face dataset:

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=ffw \
  --control.type=record \
  --control.single_task="pick and place objects" \
  --control.fps=30 \
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
- Make sure to replace `${HF_USER}` with your actual Hugging Face username.
- To save the dataset locally without uploading to the Hugging Face Hub, set `--control.push_to_hub=false`. This option is essential if you choose not to use Hugging Face.
:::

#### Key Parameters to Customize

To create your own dataset, here are some important parameters you may want to adjust:

| Parameter                  | Description | Example |
|----------------------------|-------------|---------|
| `--control.repo_id`        | The Hugging Face dataset repository ID in the format `<username>/<dataset_name>` | `username/ffw_pick_place` |
| `--control.single_task`    | The name of the task you're performing | "pick and place objects" |
| `--control.fps`            | Frame rate for dataset recording | 15 (recommended) |
| `--control.episode_time_s` | Duration (in seconds) to record each episode | 30-60 for simple tasks |
| `--control.reset_time_s`   | Time allocated (in seconds) for resetting between episodes | 10-20 seconds |
| `--control.num_episodes`   | Total number of episodes to record | 10-50 depending on task complexity |

Of course, you can modify additional parameters as needed to fit your specific use case.

## Dataset Visualization

Once data collection is complete, you can preview and inspect your recorded dataset using the following command:

```bash
python lerobot/scripts/visualize_dataset_html.py \
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
 * Running on http://127.0.0.1:9090
INFO 2025-05-15 16:18:07 _internal.py:97 Press CTRL+C to quit
```

::: tip
Once the server is running, open [http://127.0.0.1:9090](http://127.0.0.1:9090) in your browser to preview the dataset.
:::
