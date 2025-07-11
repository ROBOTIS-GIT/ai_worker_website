# Imitation Learning

## Overview

This document provides an overview of the complete ROS 2-based imitation learning pipeline built on the AI Worker and the Hugging Face Hub and features a user-friendly web GUI designed to streamline interaction and enhance accessibility.

### 1. Data Collection

Human operators use a wearable skeletal leader device to demonstrate motions, collecting image and joint position data. The web-based GUI plays a key role in streamlining the process by providing real-time visualization and control. The collected data can be uploaded to and downloaded from the Hugging Face Hub. 

### 2. Data Visualization

Collected data is visualized to inspect motion trajectories and images, helping to identify potential errors prior to training.

### 3. Model Training

The verified dataset is then used to train an action policy model. Training can be performed on local GPUs or on embedded platforms such as the NVIDIA Jetson. The resulting model can be uploaded to and downloaded from the Hugging Face Hub.

### 4. Model Inference

Once trained, the models are deployed on the AI Worker to execute real-time inference for tasks such as picking, placing, and obstacle avoidance.

## End-to-End Imitation Learning Workflow
- The diagram below shows the full imitation learning workflow using the AI Worker and Hugging Face.


<img src="/imitation_learning/end_to_end_imitation_learning_workflow.png" alt="Imitation Learning Workflow" style="width: 100%; ">

## Tutorial Videos: End-to-End Imitation Learning Workflow

The videos illustrate the full imitation learning workflow using the AI Worker, including teleoperation, dataset creation, and real-time inference with a trained model:

- Full imitation learning workflow using the **ROBOTIS AI Worker** and the **LeRobot CLI**.

<YouTube videoId="hnJpFX2G3P4" />

- Dataset Schema

The dataset follows the standard [🤗 Hugging Face datasets format](https://huggingface.co/docs/datasets/index) and contains imitation learning demonstrations collected from the AI Worker via ROS 2 teleoperation using the [LeRobot](https://github.com/huggingface/lerobot) framework.

| Field              | Type            | Description          |
|--------------------|-----------------|----------------------|
| `action`           | `List[float32]` | Leader state vector  |
| `observation.state`| `List[float32]` | Follower state vector |
| `observation.images.cam_head`        | `Image`   | RGB image from the head-mounted camera |
| `observation.images.cam_wrist_1`     | `Image`   | RGB image from the first wrist camera |
| `observation.images.cam_wrist_2`     | `Image`   | RGB image from the second wrist camera |
| `timestamp`                          | `float32` | Time (in seconds) when the step was recorded |
| `frame_index`                        | `int64`   | Index of the frame within an episode |
| `episode_index`                      | `int64`   | Index of the episode |
| `index`                              | `int64`   | Global index across the dataset |
| `task_index`                         | `int64`   | Task identifier |
