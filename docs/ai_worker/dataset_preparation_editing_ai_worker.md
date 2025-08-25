# Dataset Preparation - Editing

After collecting data, you can modify your dataset by merging multiple datasets or removing unwanted episodes. This section guides you through these basic dataset operations.

## Merging Datasets
You can merge multiple datasets into a single dataset. 

### 1. Enter datasets to merge

Navigate to the `Edit Dataset` page.

<img src="/imitation_learning/web_ui/edit_dataset/page.png" alt="Web UI" style="width: 100%; ">

In the `Merge Datasets` > `Enter Datasets to Merge` section:

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/enter_datasets_to_merge.png" alt="Web UI" style="width: 70%;">
</div>

Click the **folder button** to open the file explorer and select your desired dataset. (You can also type the path directly in the text input field)


<div style="display: flex; flex-direction: column; align-items: center; gap: 10px;">
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui/edit_dataset/folder_button.png" alt="Web UI" style="width: 100%; max-width: 60px;">
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

### 2. Enter output path

In the `Enter Output Path` section, select the path where the merged dataset result will be saved.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/enter_output_path.png" alt="Web UI" style="width: 70%; ">
</div>

Click the **folder button** to open the file explorer and select your desired folder. (You can also type the path directly in the text input field)

<div style="display: flex; flex-direction: column; align-items: center; gap: 10px;">
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui/edit_dataset/folder_button.png" alt="Web UI" style="width: 100%; max-width: 60px;">
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

### 1. Select dataset to edit

Navigate to the `Edit Dataset` page.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui/edit_dataset/delete_episodes.png" alt="Web UI" style="width: 80%; max-width: 550px;">
</div>

In the `Delete Episodes` section, click the **folder button** to open the file explorer and select your desired dataset. (You can also type the path directly in the text input field)

<div style="display: flex; flex-direction: column; align-items: center; gap: 10px;">
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui/edit_dataset/folder_button.png" alt="Web UI" style="width: 100%; max-width: 60px;">
  </div>
  <div style="font-size: 30px; text-align: center;">
    ↓
  </div>
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui/edit_dataset/browse_merge_input.png" alt="Web UI" style="width: 100%; max-width: 550px;">
  </div>
</div>

### 2. Specify episodes to remove

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
