# Dataset Preparation - Visualization

Once data collection is complete, you can preview and inspect your recorded dataset using the following steps.  

## Launch the Dataset Visualization Tool:

Enter the **physical_ai_tools** Docker container

`ROBOT PC`
```bash
cd ~/ai_worker && ./docker/container.sh enter
```

Navigate to the **physical_ai_tools/lerobot** directory inside the container:

`ROBOT PC` `🐋 AI WORKER`
```bash
cd /root/ros2_ws/src/physical_ai_tools/lerobot
```

Run the following command to launch the dataset visualization tool:

```bash
python lerobot/scripts/visualize_dataset_html.py \
  --host 0.0.0.0 \
  --port 9091 \
  --repo-id YOUR_REPO_ID
```
  - Replace **YOUR_REPO_ID** with the dataset ID found under the **User ID** section in the right sidebar.
  - For example: `ROBOTIS/ffw_sg2_f3m_Test` 
  <div class="UI" style="width: 50%; margin: 0 auto;">
  <img src="/imitation_learning/web_ui_task_info_repo_id.png" alt="Web UI" style="width: 100%; ">
</div>

<Todo: 위 이미지 AI worker 버전으로 변경 필요>

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
Access one of the following URLs to view the dataset:
- `http://ffw-{serial number}.local:9091`
- `http://{IP address of ROBOT PC}:9091`


You should see a web interface similar to the one below:
<img src="/imitation_learning/data_visualization.png" alt="Web UI" style="width: 100%; ">
