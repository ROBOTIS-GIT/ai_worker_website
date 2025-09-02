---
next: setup_guide
nextText: Setup Guide
---

# Assembly Guide

Step-by-Step Assembly Guide for OMX

::: info
- Assemble exactly as shown in the **official manual video** (posture, **cable routing**, **torque**)
- When **all joints are at 0°**, the posture must match the following reference image exactly.
<div style="text-align: center; font-size: 3.8rem; line-height: 1; margin: 50px 0 8px; letter-spacing: 2px;">⭣   ⭣</div>
<div style="max-width: 1000px; margin: 0 auto; display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; align-items: center;">
  <div style="width: 100%; aspect-ratio: 1 / 1; display: flex; align-items: center; justify-content: center;">
    <img src="/quick_start_guide/omx/omx_f_initial.webp" alt="Reference posture at all joints = 0° (Left)" style="width: 100%; height: 100%; object-fit: contain; display: block;" />
  </div>
  <div style="width: 100%; aspect-ratio: 1 / 1; display: flex; align-items: center; justify-content: center;">
    <img src="/quick_start_guide/omx/omx_l_initial.webp" alt="Reference posture at all joints = 0° (Right)" style="width: 100%; height: 100%; object-fit: contain; display: block;" />
  </div>
</div>
:::

::: info
- We provide the camera frame and mounting bolts; <br> the camera itself is **not included in the package**. You can purchase it from the **ROBOTIS Shop**.

<div style="max-width: 650px; margin: 12px auto; display: flex; align-items: center; justify-content: center;">
  <img src="/assembly_guide/camera_info.jpg" alt="Example of anti-slip tape applied" style="width: 100%; height: auto; object-fit: contain; display: block; border-radius: 6px;" />
</div>
:::

## Overview
OMX-AI is operated as a leader–follower system. Each arm has a dedicated role and naming:

- Leader: OMX-L (omx-l)
  - Role: Human teleoperation input device that generates reference motions
- Follower: OMX-F (omx-f)
  - Role: Executes the leader's motion precisely in real time

<!-- styles consolidated to docs/.vitepress/theme/custom.css -->


## Leader — Assembly Guide

### Motor Configuration (Leader)
<div style="overflow-x: auto;">
  <table style="width: 100%; text-align: center; border-collapse: separate; border-spacing: 10px 8px;">
    <thead>
      <tr>
        <th style="padding: 8px 12px;">Joint</th>
        <th style="padding: 8px 12px;">Motor</th>
        <th style="padding: 8px 12px;">ID</th>
      </tr>
    </thead>
    <tbody>
      <tr><td style="padding: 8px 12px;">1</td><td style="padding: 8px 12px;">XL330-M288</td><td style="padding: 8px 12px;">1</td></tr>
      <tr><td style="padding: 8px 12px;">2</td><td style="padding: 8px 12px;">XL330-M288</td><td style="padding: 8px 12px;">2</td></tr>
      <tr><td style="padding: 8px 12px;">3</td><td style="padding: 8px 12px;">XL330-M288</td><td style="padding: 8px 12px;">3</td></tr>
      <tr><td style="padding: 8px 12px;">4</td><td style="padding: 8px 12px;">XL330-M288</td><td style="padding: 8px 12px;">4</td></tr>
      <tr><td style="padding: 8px 12px;">5</td><td style="padding: 8px 12px;">XL330-M288</td><td style="padding: 8px 12px;">5</td></tr>
      <tr><td style="padding: 8px 12px;">6</td><td style="padding: 8px 12px;">XL330-M077</td><td style="padding: 8px 12px;">6</td></tr>
    </tbody>
  </table>
  </div>

::: warning
- Always orient **the motor horn upward as shown in the photo.**<br>Assemble exactly as in the official manual video.

<div style="max-width: 360px; margin: 12px auto; display: flex; align-items: center; justify-content: center;">
  <img src="/assembly_guide/motor_horn.png" alt="Motor horn orientation" style="width: 85%; height: auto; object-fit: contain; display: block; border-radius: 6px;" />
</div>
:::

### Step 1
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/omx_l/OMX-L_1.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>

### Step 2
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/omx_l/OMX-L_2.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>

### Step 3
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/omx_l/OMX-L_3.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>

### Step 4
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/omx_l/OMX-L_4.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>

### Step 5
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/omx_l/OMX-L_5.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>

### Step 6
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/omx_l/OMX-L_6.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>


## Follower — Assembly Guide

### Motor Configuration (Follower)
<div style="overflow-x: auto;">
  <table style="width: 100%; text-align: center; border-collapse: separate; border-spacing: 10px 8px;">
    <thead>
      <tr>
        <th style="padding: 8px 12px;">Joint</th>
        <th style="padding: 8px 12px;">Motor</th>
        <th style="padding: 8px 12px;">ID</th>
      </tr>
    </thead>
    <tbody>
      <tr><td style="padding: 8px 12px;">1</td><td style="padding: 8px 12px;">XL430-W250-T</td><td style="padding: 8px 12px;">11</td></tr>
      <tr><td style="padding: 8px 12px;">2</td><td style="padding: 8px 12px;">XL430-W250-T</td><td style="padding: 8px 12px;">12</td></tr>
      <tr><td style="padding: 8px 12px;">3</td><td style="padding: 8px 12px;">XL430-W250-T</td><td style="padding: 8px 12px;">13</td></tr>
      <tr><td style="padding: 8px 12px;">4</td><td style="padding: 8px 12px;">XL330-M288</td><td style="padding: 8px 12px;">14</td></tr>
      <tr><td style="padding: 8px 12px;">5</td><td style="padding: 8px 12px;">XL330-M288</td><td style="padding: 8px 12px;">15</td></tr>
      <tr><td style="padding: 8px 12px;">6</td><td style="padding: 8px 12px;">XL330-M288</td><td style="padding: 8px 12px;">16</td></tr>
    </tbody>
  </table>
  </div>

::: warning
- Always orient **the motor horn upward as shown in the photo.**<br>Assemble exactly as in the official manual video.

<div style="max-width: 360px; margin: 12px auto; display: flex; align-items: center; justify-content: center;">
  <img src="/assembly_guide/motor_horn.png" alt="Motor horn orientation" style="width: 85%; height: auto; object-fit: contain; display: block; border-radius: 6px;" />
</div>
:::

### Step 1
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/omx_f/OMX-F_1.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>

### Step 2
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/omx_f/OMX-F_2.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>

### Step 3
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/omx_f/OMX-F_3.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>

### Step 4
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/omx_f/OMX-F_4.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>

### Step 5
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/omx_f/OMX-F_5.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>

### Step 6
<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/omx_f/OMX-F_6.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>

<!-- ---

### Video: Assembly Cable & Camera
Below is a short video that summarizes the assembly steps and shows how to connect camera and cables.

<YouTube videoId="dQw4w9WgXcQ" /> -->

🎉 Assembly Complete

Your OMX assembly is complete. Continue to the next step.

## Appendix

:::: tip
- Applying anti-slip tape to the gripper/contact surfaces, as shown below, can significantly improve grasp efficiency.

<div style="max-width: 650px; margin: 12px auto; display: flex; align-items: center; justify-content: center;">
  <img src="/assembly_guide/anti-slip.jpg" alt="Example of anti-slip tape applied" style="width: 100%; height: auto; object-fit: contain; display: block; border-radius: 6px;" />
</div>
::::
