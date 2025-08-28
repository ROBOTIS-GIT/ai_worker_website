# Dataset Preparation - Visualization

Once data collection is complete, you can preview and inspect your recorded dataset using the following steps.

## Launch the Dataset Visualization Tool:

Enter the **physical_ai_tools** Docker container:

`USER PC`
```bash
cd physical_ai_tools/docker && ./container.sh enter
```

Move to the **physical_ai_tools/lerobot** directory inside the container:

`USER PC` `🐋 PHYSICAL AI TOOLS`
```bash
cd /root/ros2_ws/src/physical_ai_tools/lerobot/src
```

Run the following command to launch the dataset visualization tool:

```bash
python lerobot/scripts/visualize_dataset_html.py \
  --host 0.0.0.0 \
  --port 9091 \
  --repo-id YOUR_REPO_ID
```
  - Replace **YOUR_REPO_ID** with the dataset ID found under the **User ID** section in the right sidebar.
  - For example: `ROBOTIS/omx_f_Test`
  <div class="UI" style="width: 50%; margin: 0 auto;">
  <img src="/imitation_learning/web_ui/record/task_info_repo_id.png" alt="Web UI" style="width: 100%; ">
</div>

You should see an output similar to the following:

```bash
WARNING:root:'torchcodec' is not available in your platform, falling back to 'pyav' as a default decoder
INFO 2025-07-28 06:55:31 set_html.py:364 Output directory already exists. Loading from it: '/tmp/lerobot_visualize_dataset_772b0wso'
 * Serving Flask app 'visualize_dataset_html'
 * Debug mode: off
INFO 2025-07-28 06:55:31 _internal.py:97 WARNING: This is a development server. Do not use it in a production deployment. Use a production WSGI server instead.
 * Running on all addresses (0.0.0.0)
 * Running on http://127.0.0.1:9091
 * Running on http://192.168.50.127:9091
INFO 2025-07-28 06:55:31 _internal.py:97 Press CTRL+C to quit
```
Access http://127.0.0.1:9091 to view the dataset. You should see a web interface similar to the one below:
<img src="/imitation_learning/data_visualization.png" alt="Web UI" style="width: 100%; ">


::: tip
Once the server is running, open [http://127.0.0.1:9091](http://127.0.0.1:9091) in your browser to preview the dataset.
:::
