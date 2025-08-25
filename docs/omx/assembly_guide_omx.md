---
next: setup_guide
nextText: Setup Guide
---

# Assembly Guide (OMX)

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

## Overview
OMX-AI is operated as a leader–follower system. Each arm has a dedicated role and naming:

- Leader: OMX-L (omx-l)
  - Role: Human teleoperation input device that generates reference motions
- Follower: OMX-F (omx-f)
  - Role: Executes the leader's motion precisely in real time

### Motor Configuration Summary
The following table summarizes the actuator models per joint for both arms.

| Joint | OMX-L (Leader) Motor | OMX-F (Follower) Motor |
|:---:|:-----------------------:|:-------------------------:|
| 1    | XL330-M288           | XL430-W250-T            |
| 2    | XL330-M288           | XL430-W250-T            |
| 3    | XL330-M288           | XL430-W250-T            |
| 4    | XL330-M288           | XL330-M288             |
| 5    | XL330-M288           | XL330-M288             |
| 6    | XL330-M077           | XL330-M288             |


<!-- styles consolidated to docs/.vitepress/theme/custom.css -->


## Leader — Assembly Guide <span class="chip-soft chip-amber"><span class="dot"></span> OMX‑L</span>

<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/Joint1_v2.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>


## Follower — Assembly Guide <span class="chip-soft chip-pink"><span class="dot"></span> OMX‑F</span>

<div class="video-container">
  <video controls preload="metadata" style="width: 100%; max-width: 900px; border-radius: 10px;">
    <source src="/assembly_guide/Joint1_v2.mp4" type="video/mp4" />
    Your browser does not support the video tag.
  </video>
</div>

<!-- ---

### Video: Assembly Cable & Camera
Below is a short video that summarizes the assembly steps and shows how to connect camera and cables.

<YouTube videoId="dQw4w9WgXcQ" /> -->

::: tip 🎉 Assembly Complete
Your OMX assembly is complete. Continue to the Setup Guide to power on, configure networking, and run initial checks.
:::