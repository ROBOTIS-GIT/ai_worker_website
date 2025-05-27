# Imitation Learning

## Overview

### General Information

The dataset follows the standard [🤗 Hugging Face datasets format](https://huggingface.co/docs/datasets/index) and contains imitation learning demonstrations collected from the AI Worker via ROS 2 teleoperation using the [lerobot](https://github.com/huggingface/lerobot) framework.

### Tutorial Video: End-to-End Imitation Learning Workflow

The following video demonstrates the complete process of recording a dataset with AI Worker, from teleoperation to dataset creation to inference using a model trained on the collected data:

<YouTube videoId="hnJpFX2G3P4" />

### Dataset Schema

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
