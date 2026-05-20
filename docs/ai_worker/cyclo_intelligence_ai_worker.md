# Cyclo Intelligence

**Cyclo Intelligence** is the AI workflow system for AI Worker. Through a web UI you can select the robot type, record robot data, convert datasets, manage Hugging Face datasets and models, train policies, and deploy trained models for inference. The UI works with ROS 2 services and topics, while `cyclo_data`, `orchestrator`, and the Cyclo Brain policy containers run the data and model runtime in the background.

Cyclo Manager is used to bring up and manage containers. Cyclo Intelligence is the workspace used after the robot stack is running.

## Start Cyclo Intelligence

Start from the Cyclo Intelligence repository on the AI Worker Orin.

`ROBOT PC`
```bash
cd /home/robotis/cyclo_intelligence
git pull
./docker/container.sh start
```

::: tip Cyclo Manager
If Cyclo Intelligence is managed through Cyclo Manager, open the Cyclo Manager System page and start the Cyclo Intelligence service from there.
:::

Open the web UI in your browser.

```text
http://<robot-ip>/
```

For local access on the robot:

```text
http://127.0.0.1/
```

## Home Page

On the Home page, select the robot type before using Record, Training, or Inference.

![Cyclo Intelligence Home Page](/advanced_features/cyclo_intelligence/home_robot_type.png)

1. Click `Refresh Robot Type List`.
2. Select the current robot in `Select Robot Type`.
3. Click `Set Robot Type`.
4. Check that the UI changes to the connected state.

The selected robot type is used by the orchestrator, recording pipeline, training options, and Cyclo Brain inference runtime.

## Record Page

The Record page collects robot data and stores it through `cyclo_data`.

The current Record page is organized around a top status and control bar, a live image grid, a lower monitoring area, and a collapsible Task Information panel.

![Record Page with Rosbag Replay](/advanced_features/cyclo_intelligence/record_page_rosbag_replay.png)

| Area | Description |
| --- | --- |
| Top status and control bar | Robot type, joystick mode, system status, and recording control buttons. |
| Center image grid | Camera streams selected from the robot topics. |
| Lower monitoring area | 3D robot viewer and Record Topic Monitor. |
| Right Task Information panel | Task number, task name, task instruction, ROBOTIS license option, and session preparation. |

### cyclo_data Recording Pipeline

`cyclo_data` owns the data plane for Cyclo Intelligence. The UI sends a record command to the orchestrator, and the orchestrator forwards the request to `cyclo_data`. From there, `cyclo_data` handles the bag writer, video encoding, dataset conversion, editing, visualization, and Hugging Face operations.

| Stage | Owner | Description |
| --- | --- | --- |
| Record | `cyclo_data` + `rosbag_recorder` | Records robot topics and camera streams into a task folder. |
| Encode | `cyclo_data.recorder` | Writes camera streams as MP4 files and tracks recording metadata. |
| Convert | `cyclo_data.converter` | Converts MCAP and MP4 data into LeRobot dataset formats. |
| Manage | `cyclo_data.editor` / `cyclo_data.hub` | Edits datasets and uploads or downloads Hugging Face repositories. |
| Visualize | `cyclo_data.visualization` | Serves replay and dataset visualization data to the UI. |

The recording status is published by `cyclo_data` on `/data/recording/status`.

| Status | Meaning |
| --- | --- |
| `READY` | The recorder is idle and ready for a new session. |
| `RECORDING` | The current episode is being recorded. |
| `SAVING` | The episode is being finalized and encoded. |
| `CONVERTING` | The recorded data is being converted from rosbag and MP4 into LeRobot dataset output. |
| `PAUSED` | Recording is paused and can be resumed. |

### Recording Workflow

1. Open the `Record` page after setting the robot type.
2. Select or confirm the camera topics shown in the center view.
3. Fill out `Task Num`, `Task Name`, and `Task Instruction`.
4. Enable `Add License` only for ROBOTIS internal datasets that should include the ROBOTIS Apache 2.0 license header.
5. Click the session preparation button to arm the task on the orchestrator.
6. Use `Record`, `Save`, or `Discard` from the top control bar as needed.

::: info
Recorded rosbag data is stored under `/workspace/rosbag2` inside the Cyclo Intelligence container. On the host, this path is bind-mounted from `docker/workspace` in the Cyclo Intelligence repository.
:::

::: tip
The recent `cyclo_data` update stabilizes repeated `START` and `STOP` recording, hardens background video-transcode worker shutdown, and fixes LeRobot v3.0 aggregated video timing and frame-count validation.
:::

## Dataset Tools

The dataset tools use the same `cyclo_data` backend as recording.

Use `Convert Dataset` to convert a rosbag2 task folder into MP4 and LeRobot dataset outputs.

![Convert Dataset](/advanced_features/cyclo_intelligence/data_tools_convert_dataset.png)

| Tool | Description |
| --- | --- |
| Dataset merge | Combines multiple rosbag task folders into a single output dataset. |
| Delete episodes | Removes selected episodes from a dataset. |
| Convert rosbag2 | Converts recorded rosbag2 data to MP4 and LeRobot dataset output. |
| Hugging Face upload/download/delete | Manages dataset and model repositories through the configured Hugging Face endpoint. |

Long-running data operations publish progress through Cyclo Intelligence status topics so the UI can show conversion and Hub progress without blocking the recording workflow.

## Model Deploy

Cyclo Brain is the model deployment and inference runtime inside Cyclo Intelligence. It runs policy backends as Docker containers and exposes them to the UI through one Inference page.

![Cyclo Brain Inference Page](/advanced_features/cyclo_intelligence/inference_lerobot_setup.png)

### Runtime Architecture

Cyclo Brain splits each policy backend into two processes.

| Process | Responsibility |
| --- | --- |
| `main-runtime` | Receives external inference commands, owns session state, manages the action chunk buffer, and publishes robot command topics. |
| `engine-process` | Loads the policy model, reads robot observations, and computes action chunks. It does not publish robot commands directly. |

The architecture map below shows the command, observation, engine, buffer, and action flow inside the manual page width.

<CycloBrainArchitecture />

The runtime flow is:

1. The UI sends a `LOAD`, `START`, `STOP`, or `UNLOAD` style command.
2. `main-runtime` receives the command and updates the session state.
3. `main-runtime` asks `engine-process` to load a model or compute actions through an internal Zenoh service.
4. `engine-process` reads robot observations, runs model inference, and returns an action chunk.
5. `main-runtime` pushes the chunk into the action buffer and publishes one robot command per control-loop tick.

LeRobot and GR00T use the same runtime contract. The model loading, preprocessing, and prediction implementation are handled by each backend.

### Model Selection

On the Inference page, select the policy backend and model family from `Task Information > Model`.

![Cyclo Brain Model Selection](/advanced_features/cyclo_intelligence/inference_model_list.png)

| Group | Model | Backend | Description |
| --- | --- | --- | --- |
| LeRobot | `ACT` | `lerobot_server` | Loads a LeRobot ACT checkpoint. |
| LeRobot | `SmolVLA` | `lerobot_server` | Loads a LeRobot SmolVLA checkpoint and uses task instructions. |
| LeRobot | `XVLA` | `lerobot_server` | Loads a LeRobot XVLA checkpoint and uses task instructions. |
| LeRobot | `Pi0` | `lerobot_server` | Loads a LeRobot Pi0 checkpoint and uses task instructions. |
| LeRobot | `Pi0.5` | `lerobot_server` | Loads a LeRobot Pi0.5 checkpoint and uses task instructions. |
| LeRobot | `Diffusion` | `lerobot_server` | Loads a LeRobot Diffusion checkpoint. |
| GR00T | `GR00T N1.7` | `groot_server` | Loads an NVIDIA Isaac GR00T N1.7 checkpoint and uses task instructions. |

The following entries can appear in the UI but are disabled until their runtime path is validated.

- `GreenVLA`
- `OpenPI`
- `RLDX-1`

### Docker Backend Control

After selecting a model, the matching Docker backend control appears in the right panel.

| Button | Behavior |
| --- | --- |
| `ON` | Creates or starts the matching policy container. If it is already running, it restarts the container to reset the runtime. |
| `Restart` | Restarts the policy container. Use this after model-load failures, stale services, or CUDA memory cleanup needs. |
| `OFF` | Stops the policy container without deleting the container or image. |

Two process states are shown for each backend.

| Process | `Up` means | If `Down` or `Unknown` |
| --- | --- | --- |
| `Main` | The `main-runtime` service is alive. | Press `Restart` to bring the backend up again. |
| `Engine` | The `engine-process` service is alive. | Model loading or inference calls may fail. Press `Restart` first. |

`Main Up` and `Engine Up` mean the backend processes are alive. They do not mean that a model is already loaded. Model loading happens after pressing `Start` with the current `Policy Path`.

### LeRobot Deployment

![LeRobot Inference Setup](/advanced_features/cyclo_intelligence/inference_lerobot_instruction.png)

1. Select a policy under the `LeRobot` group.
2. Check that `LeRobot Docker` is `Running`.
3. Check that both `Main` and `Engine` are `Up`.
4. Enter a Hugging Face repo ID or a local checkpoint path in `Policy Path`.
5. For language-conditioned policies, enter `Task Instruction`.
6. Set `Inference Hz` to match the model action generation rate.
7. Set `Control Hz` to match the robot command publish rate.
8. Click `Start`.

Example Hugging Face repo ID:

```text
Dongkkka/Act_test_20k
```

Example local checkpoint path:

```text
/policy_checkpoints/lerobot/Act_test_20k
```

ACT and Diffusion do not use task instructions, so the task instruction field is hidden for those model families.

### GR00T N1.7 Deployment

![GR00T N1.7 Inference Setup](/advanced_features/cyclo_intelligence/inference_groot_instruction.png)

1. Select `GR00T N1.7` under the `GR00T` group.
2. Check that `GR00T Docker` is `Running`.
3. Check that both `Main` and `Engine` are `Up`.
4. Enter the task instruction in `Task Instruction`.
5. Enter the model path in `Policy Path`.
6. Click `Start`.
7. To update the instruction during inference, edit the text and click `Update Task Instruction`.

Example Hugging Face repo ID:

```text
Dongkkka/cyclo_intelligence_groot_n1.7_model
```

Example local checkpoint path:

```text
/policy_checkpoints/groot/cyclo_intelligence_groot_n1.7_model
```

### Model File Locations

Policy checkpoint folders are bind-mounted into each policy container.

| Backend | Host path | Container path |
| --- | --- | --- |
| LeRobot | `/home/robotis/cyclo_intelligence/cyclo_brain/policy/lerobot/checkpoints` | `/policy_checkpoints/lerobot` |
| GR00T | `/home/robotis/cyclo_intelligence/cyclo_brain/policy/groot/checkpoints` | `/policy_checkpoints/groot` |

If the host model is here:

```text
/home/robotis/cyclo_intelligence/cyclo_brain/policy/lerobot/checkpoints/Act_test_20k
```

Enter this path in the UI:

```text
/policy_checkpoints/lerobot/Act_test_20k
```

## Inference Controls

The top Inference control bar is shared by all backends.

| Button | Description |
| --- | --- |
| `Start` | Loads the model and starts inference. In paused state, resumes inference when the policy path is unchanged. |
| `Stop` | Pauses inference. The model remains loaded in memory. |
| `Clear` | Stops inference and clears the model, session, and action buffer. |
| `Record` | Starts recording inference results when recording is enabled. |
| `Save` | Saves the current inference recording. |
| `Discard` | Discards the current inference recording. |

If `Start` is disabled, check the Docker backend state in the right panel. Common causes are `OFF`, `Image missing`, `Warming up`, `Main Down`, or `Engine Down`.

## Troubleshooting

### Start Is Disabled

Check the backend status in the right panel.

- `Image missing`: pull or install the required Docker image.
- `Stopped` or `Not created`: click `ON`.
- `Warming up`: wait until both `Main` and `Engine` are `Up`.
- `Main Down` or `Engine Down`: click `Restart`.

### Model Load Failed

1. Check that `Policy Path` is correct.
2. If you use a local path, make sure it is a container path such as `/policy_checkpoints/...`.
3. If you use a Hugging Face repo ID, check network access and token permissions.
4. Check that the selected UI model family matches the checkpoint type.
5. Click `Clear` to reset the inference session.
6. If it still fails, restart the Docker backend and try again.

### Recording Conversion Failed

1. Check that the task folder exists under `/workspace/rosbag2`.
2. Check storage space before retrying conversion.
3. If a LeRobot output already exists for the same source dataset, delete or rename the old output before running conversion again.
4. Retry the conversion from the dataset tools page.
