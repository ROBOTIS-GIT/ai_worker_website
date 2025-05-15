# Model Training

Run the following command to start training a policy using your dataset. You can train your policy either on your local PC or on an NVIDIA Jetson Orin device.

### a. NVIDIA Jetson Orin:

Open a new terminal and navigate to the `lerobot` directory:

```bash
container
cd /root/colcon_ws/src/physical_ai_tools/lerobot
```

Then run the following command:

```bash
python lerobot/scripts/train.py \
  --dataset.repo_id=${HF_USER}/ffw_test \
  --policy.type=act \
  --output_dir=outputs/train/act_ffw_test \
  --job_name=act_ffw_test \
  --policy.device=cuda \
  --log_freq=100 \
  --save_freq=1000
```

### b. Your PC

First, follow the [LeRobot installation instructions](https://github.com/ROBOTIS-GIT/lerobot) to set up the framework locally. Once installed, you can train the policy using the same command:

```bash
python lerobot/scripts/train.py \
  --dataset.repo_id=${HF_USER}/ffw_test \
  --policy.type=act \
  --output_dir=outputs/train/act_ffw_test \
  --job_name=act_ffw_test \
  --policy.device=cuda \
  --log_freq=100 \
  --save_freq=1000
```

### (Optional) Upload Checkpoint to Hugging Face

To upload the latest trained checkpoint to the Hugging Face Hub, run:

```bash
huggingface-cli upload ${HF_USER}/act_ffw_test \
  outputs/train/act_ffw_test/checkpoints/last/pretrained_model
```

# Model Inference

Launch the ROS 2 follower node:

```bash
container
follower
```

To evaluate your trained policy, open a new terminal and navigate to the lerobot directory:
```bash
container
cd /root/colcon_ws/src/physical_ai_tools/lerobot
```

Then run the following command. It will evaluate the policy using the `record` mode and save the results for visualization. Make sure to specify the pretrained checkpoint path using the `--control.policy.path` argument:

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=ffw \
  --control.type=record \
  --control.single_task="pick and place objects" \
  --control.fps=30 \
  --control.repo_id=${HF_USER}/eval_ffw_test \
  --control.tags='["tutorial"]' \
  --control.episode_time_s=20 \
  --control.reset_time_s=10 \
  --control.num_episodes=10 \
  --control.push_to_hub=true \
  --control.policy.path=outputs/train/act_ffw_test/checkpoints/last/pretrained_model \
  --control.play_sounds=false
```
