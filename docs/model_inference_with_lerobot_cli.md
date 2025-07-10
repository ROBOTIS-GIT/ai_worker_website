# Model Inference with LeRobot CLI

Once your model is trained, you can deploy it on the AI Worker for inference.

## Model Deployment and Inference

### 1. Transfer Model to Robot PC
::: info
If you trained your model directly on NVIDIA Jetson AGX Orin, you can skip this step.
:::
Change ownership of the model directory. This step must be performed **on the robot PC**, **not inside the Docker container**:
```bash
sudo chown -R robotis ./
```
Move your model folder from your local PC to the model directory on the Robot PC using `scp`:
```bash
scp -r <your model folder's directory> robotis@<your robot's serial number>.local:~/ai_worker/docker/lerobot/outputs/train
```

### 2. Open a Terminal and Enter the Docker container
:::tabs key:robot-type
== BG2 Type
cd ai_worker && ./docker/container.sh enter
== SG2 Type
cd ai_worker && ./docker/container.sh enter
== OMY
cd open_manipulator && ./docker/container.sh enter
:::

### 3. Launch the ROS 2 Follower Node
:::tabs key:robot-type
== BG2 Type
ffw_bg2_follower_ai
== SG2 Type
ffw_sg2_follower_ai
== OMY
ros2 launch open_manipulator_bringup hardware_y_follower.launch.py
:::

### 4. Run Inference

#### a. Open a New Terminal and Run Docker Container
Open a terminal on the Jetson device and enter the Docker container:
:::tabs key:robot-type
== BG2 Type
cd ai_worker && ./docker/container.sh enter
== SG2 Type
cd ai_worker && ./docker/container.sh enter
== OMY
cd open_manipulator && ./docker/container.sh enter
:::

#### b. Navigate to the `LeRobot` Directory
```bash
cd /root/ros2_ws/src/physical_ai_tools/lerobot
```

#### c. Run the Following Command for Evaluation
```bash
python lerobot/scripts/control_robot.py \
  --robot.type=ffw \
  --control.type=record \
  --control.single_task="pick and place objects" \
  --control.fps=15 \
  --control.repo_id=${HF_USER}/eval_ffw_test \
  --control.tags='["tutorial"]' \
  --control.episode_time_s=20 \
  --control.reset_time_s=10 \
  --control.num_episodes=10 \
  --control.push_to_hub=true \
  --control.policy.path=outputs/train/act_ffw_test/checkpoints/last/pretrained_model \
  --control.play_sounds=false
```
::: details :point_right: Key Inference Parameters
| Parameter | Description |
|-----------|-------------|
| `--control.type=record` | Records the policy performance for later evaluation |
| `--control.policy.path` | Path to your trained model checkpoint |
| `--control.episode_time_s` | Duration of each inference episode (in seconds) |
| `--control.repo_id` | Hugging Face repo where evaluation results will be saved |
:::

## Visualizing Inference Results

After running inference, you can visualize the results using the same visualization tool used for datasets:

```bash
python lerobot/scripts/visualize_dataset_html.py \
  --host 0.0.0.0 \
  --port 9091 \
  --repo-id ${HF_USER}/eval_ffw_test
```

Then open [http://127.0.0.1:9091](http://127.0.0.1:9091) in your browser to see how your model performed.

::: tip
If you have another device connected to the same network as the host machine, open `http://{robot type}-{serial number}.local:9091` in your browser to see how your model performed.

For example, `http://ffw-SNPR48A0000.local:9091`.
:::

## Troubleshooting

- **Out of memory errors**: Try reducing the batch size with `--train.batch_size=16` or lower
- **Low performance**: Consider collecting more diverse training data or increasing training duration
- **Robot not responding**: Ensure the follower node is running and communication is established
- **Training divergence**: Check your dataset quality and try decreasing the learning rate
