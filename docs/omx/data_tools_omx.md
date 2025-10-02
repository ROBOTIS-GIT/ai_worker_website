# Data Tools

The **Data Tools** page provides useful tools for editing and managing datasets/models as follows: 

<img src="/imitation_learning/web_ui/edit_dataset/page.png" alt="Web UI" style="width: 100%; ">

| Tool | Description |
|---------|-------------|
| **Hugging Face Upload & Download** | Upload your datasets/models to Hugging Face or download existing ones |
| **Merging Datasets** | Combine multiple datasets into a single dataset |
| **Removing Episodes** | Remove unwanted episodes from your dataset |


## Hugging Face Upload & Download

You can upload collected datasets or trained models to Hugging Face. You can also download datasets or models from Hugging Face.

### 1. Upload

First, select the Hugging Face user id and data type for upload

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/hugging_face/user_id_and_data_type.png" alt="Web UI" style="width: 50%;">
</div>

::: info
If there is no registered User ID, click the `Change` button and enter your Hugging Face token to register a User ID.
:::

Set the toggle button to upload.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/hugging_face/section_selector_upload.png" alt="Web UI" style="width: 30%;">
</div>

Select the path of the dataset or model you want to upload in the right panel.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/hugging_face/enter_local_directory.png" alt="Web UI" style="width: 80%;">
</div>

<div style="display: flex; flex-direction: column; align-items: center; gap: 10px;">
  <div style="height: 10px;">
  </div>
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui/edit_dataset/folder_button.png" alt="Web UI" style="width: 100%; max-width: 40px;">
  </div>
  <div style="font-size: 30px; text-align: center;">
    ↓
  </div>
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui/edit_dataset/hugging_face/browse_local_dir_for_upload.png" alt="Web UI" style="width: 100%; max-width: 500px;">
  </div>
</div>


Enter the repository name to be created on Hugging Face.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/hugging_face/upload_repo_id.png" alt="Web UI" style="width: 80%;">
</div>

Click the `Upload` button. The upload progress will be displayed.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/hugging_face/upload_progress.png" alt="Web UI" style="width: 80%;">
</div>

::: info
You can cancel the upload by clicking `Cancel` during the upload process. However, the repository will still be created on Hugging Face.

After canceling, clicking the Upload button again will resume the upload from where it left off.
:::


### 2. Download

First, select the Hugging Face user id and data type for download

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/hugging_face/user_id_and_data_type.png" alt="Web UI" style="width: 50%;">
</div>

Set the toggle button to download.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/hugging_face/section_selector_download.png" alt="Web UI" style="width: 30%;">
</div>

Enter the repository to download.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/hugging_face/enter_download_repo.png" alt="Web UI" style="width: 80%;">
</div>

Click the `Download` button.

- **Dataset**: Progress bar is displayed
  <img src="/imitation_learning/web_ui/edit_dataset/hugging_face/download_progress.png" alt="Web UI" style="width: 80%;">

- **Model**: Spinner is displayed
  <img src="/imitation_learning/web_ui/edit_dataset/hugging_face/download_spinner.png" alt="Web UI" style="width: 50%;">


::: info
You can cancel the download by clicking `Cancel` during the download process. However, the folder created in the local path will remain.
:::

::: info
The download paths are as follows (based on the Docker container’s file system):

- **Dataset:** `/root/.cache/huggingface/lerobot/`
- **Model:** `/root/ros2_ws/src/physical_ai_tools/lerobot/outputs/train/`
:::

::: tip
⚠️ **Network recommendation**

Policy models are typically large in size. When downloading a policy model, a **wired Ethernet connection** is **highly recommended** to ensure stable and fast download speeds.
:::


## Merging Datasets
You can merge multiple datasets into a single dataset. 

### 1. Enter Datasets to Merge

In the `Merge Datasets` > `Enter Datasets to Merge` section:

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/enter_datasets_to_merge.png" alt="Web UI" style="width: 70%;">
</div>

Click the **folder button** to open the file explorer and select your desired dataset. (You can also type the path directly in the text input field)


<div style="display: flex; flex-direction: column; align-items: center; gap: 10px;">
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui/edit_dataset/folder_button.png" alt="Web UI" style="width: 100%; max-width: 40px;">
  </div>
  <div style="font-size: 30px; text-align: center;">
    ↓
  </div>
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui/edit_dataset/browse_merge_input.png" alt="Web UI" style="width: 100%; max-width: 550px;">
  </div>
</div>

Click the `+ Add Dataset` button to add a new dataset input field and select your dataset.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/add_new_field.png" alt="Web UI" style="width: 90%;">
</div>

Repeat the above steps to add as many datasets as needed.

### 2. Enter Output Path

In the `Enter Output Path` section, select the path where the merged dataset result will be saved.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/enter_output_path.png" alt="Web UI" style="width: 70%; ">
</div>

Click the **folder button** to open the file explorer and select your desired folder. (You can also type the path directly in the text input field)

<div style="display: flex; flex-direction: column; align-items: center; gap: 10px;">
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui/edit_dataset/folder_button.png" alt="Web UI" style="width: 100%; max-width: 40px;">
  </div>
  <div style="font-size: 30px; text-align: center;">
    ↓
  </div>
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui/edit_dataset/output_dir.png" alt="Web UI" style="width: 100%; max-width: 550px;">
  </div>
</div>


Enter the name for the merged result dataset folder.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/enter_output_folder_name.png" alt="Web UI" style="width: 80%; max-width: 550px;">
</div>

Finally, click the `Merge` button to merge the datasets. 

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/merge_button.png" alt="Web UI" style="width: 20%; max-width: 550px;">
</div>

## Removing Episodes

You can remove episodes that were incorrectly recorded during the data collection process.

### 1. Select Dataset to Edit

In the `Delete Episodes` section, click the **folder button** to open the file explorer and select your desired dataset. (You can also type the path directly in the text input field)

<div style="display: flex; flex-direction: column; align-items: center; gap: 10px;">
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui/edit_dataset/folder_button.png" alt="Web UI" style="width: 100%; max-width: 40px;">
  </div>
  <div style="font-size: 30px; text-align: center;">
    ↓
  </div>
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui/edit_dataset/browse_merge_input.png" alt="Web UI" style="width: 100%; max-width: 550px;">
  </div>
</div>

### 2. Specify Episodes to Remove

Enter the episode numbers you want to remove in the input field. You can remove multiple episodes at once by separating the numbers with commas.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/delete_episodes_write_episode_num.png" alt="Web UI" style="width: 80%; max-width: 550px;">
</div>

::: info
After deletion, the remaining episode numbers and metadata will be automatically reorganized and reindexed.
:::

Finally, click the `Delete` button to remove the specified episodes from the dataset.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/delete_button.png" alt="Web UI" style="width: 20%; max-width: 550px;">
</div>
