# Dataset Preparation - Editing

After collecting data, you can modify your dataset by combining multiple datasets or removing unwanted episodes. This section guides you through these basic dataset operations.

## Combining Datasets
You can combine multiple datasets into a single dataset. 

### 1. Enter datasets to merge

Navigate to the `Edit Dataset` page.

<img src="/imitation_learning/web_ui_edit_dataset.png" alt="Web UI" style="width: 100%; ">

In the `Merge Datasets` > `Enter Datasets to Merge` section:

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui_edit_dataset_enter_datasets_to_merge.png" alt="Web UI" style="width: 70%;">
</div>

Click the **folder button** to open the file explorer and select your desired dataset. (You can also type the path directly in the text input field)


<div style="display: flex; flex-direction: column; align-items: center; gap: 10px;">
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui_edit_dataset_folder_button.png" alt="Web UI" style="width: 100%; max-width: 60px;">
  </div>
  <div style="font-size: 30px; text-align: center;">
    ↓
  </div>
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui_edit_dataset_browse_merge_input.png" alt="Web UI" style="width: 100%; max-width: 550px;">
  </div>
</div>

Click the `+ Add Dataset` button to add a new dataset input field and select your dataset.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui_edit_dataset_add_new_field.png" alt="Web UI" style="width: 90%;">
</div>

Repeat the above steps to add as many datasets as needed.

### 2. Enter output path

In the Enter Output Path section, select the path where the merged dataset result will be saved.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui_edit_dataset_enter_output_path.png" alt="Web UI" style="width: 70%; ">
</div>

Click the **folder button** to open the file explorer and select your desired folder, or type the path directly in the text input field.

<div style="display: flex; flex-direction: column; align-items: center; gap: 10px;">
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui_edit_dataset_folder_button.png" alt="Web UI" style="width: 100%; max-width: 60px;">
  </div>
  <div style="font-size: 30px; text-align: center;">
    ↓
  </div>
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui_edit_dataset_output_dir.png" alt="Web UI" style="width: 100%; max-width: 550px;">
  </div>
</div>


Enter the name for the merged result dataset folder.

<div style="display: flex; justify-content: center;">
  <img src="/imitation_learning/web_ui_edit_dataset_output_folder_name.png" alt="Web UI" style="width: 80%; max-width: 550px;">
</div>

Click the Merge button to combine the datasets. 

## Removing Unwanted Episodes
