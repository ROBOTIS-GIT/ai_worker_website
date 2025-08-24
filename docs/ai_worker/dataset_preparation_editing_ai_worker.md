# Dataset Preparation - Editing

After collecting data, you can modify your dataset by combining multiple datasets or removing unwanted episodes. This section guides you through these basic dataset operations.

## Combining Datasets
You can combine multiple datasets into a single dataset. 

Navigate to the `Edit Dataset` page.

<img src="/imitation_learning/web_ui_edit_dataset.png" alt="Web UI" style="width: 100%; ">

In the `Merge Datasets` > `Enter Datasets` section:

<img src="/imitation_learning/web_ui_edit_dataset_enter_datasets_to_merge.png" alt="Web UI" style="width: 70%; ">

Click the folder button to open the file explorer and select your desired dataset. 

(You can also type the path directly in the text input field)


<div style="display: flex; justify-start: center; gap: 10px;">
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui_edit_dataset_folder_button.png" alt="Web UI" style="width: 100%; max-width: 80px;">
  </div>
  <div style="font-size: 30px">
  →
  </div>
  <div style="text-align: center;">
    <img src="/imitation_learning/web_ui_edit_dataset_browse_merge_input.png" alt="Web UI" style="width: 100%; max-width: 550px;">
  </div>
</div>

Click the `+ Add Dataset` button to add a new dataset input field and select your dataset.

Repeat the above steps to add as many datasets as needed.

In the Enter Output Path section, select the path where the merged dataset result will be saved.
Click the folder button to open the file explorer and select your desired folder, or type the path directly in the text input field.

<img src="/imitation_learning/web_ui_edit_dataset_enter_output_path.png" alt="Web UI" style="width: 70%; ">

Enter the name for the merged result dataset folder.

Click the Merge button to combine the datasets. 

## Removing Unwanted Episodes
