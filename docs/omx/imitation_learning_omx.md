# Imitation Learning

## Overview

This document provides an overview of the complete ROS 2-based imitation learning pipeline built on the OMX and the Hugging Face Hub. OMX offers two powerful approaches for imitation learning:

<div style='display: flex; justify-content: flex-start; gap: 30px;'>
<a href="/omx/dataset_preparation_omx.html" class="button-shortcut">
Option 1<br>Physical AI tools (Recommended)
</a>

<a href="/omx/lerobot_imitation_learning_omx.html" class="button-shortcut">
Option 2<br>Lerobot
</a>
</div>

### 1. Data Collection

Human operators use a leader device to demonstrate motions, collecting image and joint position data.
The collected data can be uploaded to and downloaded from the Hugging Face Hub.

### 2. Data Visualization

Collected data is visualized to inspect motion trajectories and images, helping to identify potential errors prior to training.

### 3. Model Training

The verified dataset is then used to train an action policy model. Training can be performed on local GPUs or on embedded platforms such as the NVIDIA Jetson. The resulting model can be uploaded to and downloaded from the Hugging Face Hub.

### 4. Model Inference

Once trained, the models are deployed on the OMX to execute real-time inference for tasks such as picking, placing

## End-to-End Imitation Learning Workflow
- The diagram below shows the full imitation learning workflow using the OMX and Hugging Face.


<img src="/imitation_learning/end_to_end_imitation_learning_workflow.png" alt="Imitation Learning Workflow" style="width: 100%; ">

<!-- ## Tutorial Videos: End-to-End Imitation Learning Workflow

The video illustrates the full imitation learning workflow using the OMX, including teleoperation, dataset creation, and real-time inference with a trained model:

- Full imitation learning workflow using **OMX** and the **Web GUI**.
- LeRobot native workflow is also available for advanced users.

<YouTube videoId="3x-eN36pNns" /> -->

## Dataset Schema

The dataset follows the standard [🤗 Hugging Face datasets format](https://huggingface.co/docs/datasets/index) and contains imitation learning demonstrations collected from the OMX via ROS 2 teleoperation using the [LeRobot](https://github.com/huggingface/lerobot) framework.

| Field              | Type            | Description          |
|--------------------|-----------------|----------------------|
| `action`           | `List[float32]` | Leader state vector  |
| `observation.state`| `List[float32]` | Follower state vector |
| `observation.images.camera1`     | `Image`   | RGB image from the first wrist camera |
| `observation.images.camera2`     | `Image`   | RGB image from the second wrist camera |
| `timestamp`                          | `float32` | Time (in seconds) when the step was recorded |
| `frame_index`                        | `int64`   | Index of the frame within an episode |
| `episode_index`                      | `int64`   | Index of the episode |
| `index`                              | `int64`   | Global index across the dataset |
| `task_index`                         | `int64`   | Task identifier |
