# Dataset Preparation

The following sections describe the dataset preparation workflow for imitation learning, which consists of three main stages: **Prerequisites**, **Recording**, and **Visualization**.

- The **Prerequisites** stage outlines the necessary setup steps before you begin recording your dataset.  
- The **Recording** stage explains how to collect data using the Web UI.  
  Two recording modes are available. The table below summarizes the key differences between these modes:

<table>
  <thead>
    <tr>
      <th style="width: 50%;">Single Task Mode</th>
      <th style="width: 50%;">Multi-Task Mode</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>Focused on collecting data for a single specific task</td>
      <td>Designed for collecting data across multiple tasks</td>
    </tr>
    <tr>
      <td>Allows time to reset the environment between recordings</td>
      <td>No reset time; ideal for segmented motions or diverse objects</td>
    </tr>
    <tr>
      <td>Requires a predefined number of episodes and recording duration</td>
      <td>No predefined limits; record freely until complete</td>
    </tr>
  </tbody>
</table>

You will learn more about how to use each mode in the **Recording** section.

- The **Visualization** stage focuses on verifying data quality by allowing users to inspect the recorded dataset.
