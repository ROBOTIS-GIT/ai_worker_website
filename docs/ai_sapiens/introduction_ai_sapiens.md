---
title: K0 - AI Sapiens
---

<AISapiensIntroHero />

<div class="ai-sapiens-page">

# K0 - The First Step in AI Sapiens



:::tip

<div class="ai-sapiens-at-a-glance">

<img src="/ai_sapiens/k0_no_background.png" alt="K0" class="ai-sapiens-at-a-glance-img" />

<div class="ai-sapiens-at-a-glance-spec">

|  | |
| :-- | :-- |
| **Height** | 1.3 m (1300 mm) |
| **Weight** | 34 kg |
| **DOF** | 23 (whole-body) |
| **Actuators** | Dynamixel Q<br>(QM-060 ×14, QM-080 ×9) |
| **Stack** | Fully open source:<br>hardware BoM, CAD (STEP) files, source code, simulation assets, and tutorials. |

</div>

</div>

:::

A humanoid only earns the name when motion survives the real world: contact, backlash, power limits, and sensor noise. **AI Sapiens** is ROBOTIS’s **humanoid platform** built on **Dynamixel Q**, so policies trained with **imitation learning** and **reinforcement learning** can be deployed on real hardware, not only in simulation.

**Physical AI**, here, means intelligence that **closes the loop** on hardware, sensing, actuation, and real-world dynamics, not only in simulation.

**K0** is the first humanoid in that line: the baseline hardware and software others extend, with training artifacts and documentation shipped **open source** so research, education, and product work share one reproducible starting point, not a black box.

## Key Features

<div class="features">
  <div class="feature-item">
    <div class="feature-icon">🧠</div>
    <h3>Platform for Physical AI</h3>
    <p>Documentation, simulation, and control oriented toward policies that you can train and refine for real contact and dynamics, not toy demos.</p>
  </div>

  <div class="feature-item">
    <div class="feature-icon">🌐</div>
    <h3>Fully Open-Source Platform</h3>
    <p>Repositories and learning materials are released to be forked, audited, and built on, with no guesswork about what runs on the robot.</p>
  </div>

  <div class="feature-item">
    <div class="feature-icon">⚙️</div>
    <h3>Powered by Dynamixel Q</h3>
    <p>QDD(Quasi-Direct Drive) actuators built in-house: efficiency, low backlash, and torque-level control suited to dynamic balancing and manipulation.</p>
  </div>

  <div class="feature-item">
    <div class="feature-icon">🦾</div>
    <h3>23 DOF Full-Body Motion</h3>
    <p>Head, arms, legs, and waist mapped for locomotion and upper-body tasks. The joint layout matches the detailed spec tables below.</p>
  </div>

</div>

## Teaser Video

<YouTube videoId="eFRftVHCwUs" />
<YouTube videoId="2xAJzQ8V9p8" />

## Reinforcement Learning

AI Sapiens is developing reinforcement learning-based locomotion technology using DYNAMIXEL-Q actuators, which help minimize the sim-to-real gap. Training is conducted in NVIDIA Isaac Sim, enabling precise physical simulation and large-scale iterative learning. Through this approach, AI Sapiens is demonstrating strong real-world applicability in humanoid robot environments.
By closely connecting simulation and real-world deployment, AI Sapiens is advancing more stable, precise, and next-generation humanoid locomotion technology.

<img src="/ai_sapiens/sim_walking.gif" alt="Sim Walking" width="600" />
<img src="/ai_sapiens/sim_walking2.gif" alt="Sim Walking" width="600" />

## Imitation Learning

AI Sapiens enables a complete pipeline for imitation learning, covering data collection, training, and inference. Data can be captured through the leader–follower system, allowing human motions and demonstrations to be directly recorded and structured for learning. Physical AI tools make it easy to train models and run inference, reducing the complexity of deploying learned behaviors on real hardware.

Through this pipeline, policies trained from demonstrations can be seamlessly transferred to real robots, enabling natural and adaptive behaviors. The system is designed to bridge the gap between data collection and real-world execution, allowing users to iterate quickly and develop robust robotic skills with minimal overhead.

<img src="/ai_sapiens/leader_and_k0.png" alt="Imitation Learning" width="600" />

## Specification

### Exploded View

<img src="/ai_sapiens/k0_exploded_view.png" alt="K0 exploded view" width="600" />

### Physical Specification
K0 is a humanoid platform with a **height of 1300 mm** and a **weight of 34 kg**.
It features a **total of 23 degrees of freedom**, **5 in each arm**, **6 in each leg**, and **1 in the waist**.
The system supports a **maximum arm payload of 3 kg**.

| Item | Specification |
|:-----|:----------------|
| Height | 1300 mm |
| Weight | 34 kg |
| Degrees of freedom | 23 total<br>(each arm: 5, each leg: 6, waist: 1) |
| Max arm payload | 3 kg |

### Actuators
**AI Sapiens K0** is powered by **DYNAMIXEL-Q** actuators, including the **QM-060** and **QM-080**, which are part of ROBOTIS’s latest generation of **QDD** (Quasi-Direct Drive) actuators. Building on its long-standing expertise in robotic actuators, ROBOTIS developed QDD technology and applied it to humanoid systems such as AI Sapiens.

**DYNAMIXEL-Q** is a **QDD (Quasi-Direct Drive)** actuator that combines a **low gear reduction ratio** with a **high-torque** motor and integrated control electronics. It enables **high backdrivability**, **low impedance**, and **precise torque control**, making it well-suited for dynamic and compliant motions in humanoid and robotic applications.

::: info
* **DYNAMIXEL-Q** is scheduled for release in the second half of the year.(The schedule is subject to change)

* The details on this page are subject to change as the product approaches launch.

* For any additional inquiries, contact us at [contactus2@robotis.com](mailto:contactus2@robotis.com)
:::

:::tip QM-060
<div class="dynamixel-q-at-a-glance">

<img src="/ai_sapiens/thumb_dynamixel_qm060.png" alt="dynamixel_q"/>

<div class="motor-spec">

| Item | Specification |
|:-----|:----------------|
| Motor Type | Inner Rotor |
| Input Voltage | 48 V |
| Gear ratio | 12:1 / 16:1 / 20:1 |
| Size | Ø58 x 60 mm |
| Weight | 450 g |
| Hollow Bore Diameter | Ø5 mm |

</div>
</div>
:::

:::tip QM-080
<div class="dynamixel-q-at-a-glance">

<img src="/ai_sapiens/thumb_dynamixel_qm080.png" alt="dynamixel_q"/>

<div class="motor-spec">

| Item | Specification |
|:-----|:----------------|
| Motor Type | Inner Rotor |
| Input Voltage | 48 V |
| Gear ratio | 12:1 / 16:1 / 20:1 |
| Size | Ø78 x 65 mm |
| Weight | 850 g |
| Hollow Bore Diameter | Ø7 mm |

</div>
</div>
:::


### Compute

For computation, K0 is equipped with **Jetson Orin NX 16GB (ONX 16GB)**, featuring an **NVIDIA Ampere GPU**, an **Arm Cortex-A78AE v8.2 64-bit CPU**, and AI performance of up to **100 Sparse INT8 TOPS** / **50 Dense INT8 TOPS**.

| Item | Specification |
|:-----|:----------------|
| Module | Jetson Orin NX 16GB (ONX 16GB) |
| CPU | Arm Cortex-A78AE v8.2 64-bit CPU |
| GPU | NVIDIA Ampere GPU |
| AI Performance | Up to 100 Sparse INT8 TOPS / 50 Dense INT8 TOPS |

### Interfaces

In terms of connectivity, it supports **Wi-Fi 5** and **Bluetooth 5.0** (RTL8822CE).
The platform provides multiple interfaces, including **2 Ethernet** ports, **USB 2.0 ×2** (USB-A), **USB 3.0 ×2** (1 USB-C, 1 USB-A), and **power outputs** of **24 V** (up to **10 A**) and **12 V** (up to **10 A**).

| Item | Specification |
|:-----|:----------------|
| Wireless | Wi-Fi 5, Bluetooth 5.0 (RTL8822CE) |
| Ethernet | ×2 |
| USB 2.0 | ×2 (USB-A) |
| USB 3.0 | ×2 (1× USB-C, 1× USB-A) |
| Power outputs | 24 V (up to 10 A), 12 V (up to 10 A) |

### Power

K0 is powered by a **46.8 V**, **9000 mAh** battery.

| Item | Specification |
|:-----|:----------------|
| Battery | 46.8 V, 9000 mAh |

## Costume Options


AI Sapiens lets you create a unique humanoid using a variety of customizable exterior parts, including hats, head covers, and body covers.

:::info
The default front and rear body covers and head are provided as **open source 3D printable files**, giving you the flexibility to modify and extend the design as needed.
:::

<img src="/ai_sapiens/costume_set1.png" alt="K0 customizable" width="600" />
<img src="/ai_sapiens/costume_set2.png" alt="K0 customizable" width="600" />

## Open Source

AI Sapiens is being prepared as a fully open platform. The resources below will be released in stages.

<div class="ai-sapiens-open-source-blocks">
<div class="ai-sapiens-coming-soon-banner">Coming Soon</div>

### Hardware BoM

:::tip
The complete Bill of Materials for K0 (parts list, quantities, and references) will be published for reproducible builds.
:::

### STEP Files

:::tip
Mechanical STEP files for core assemblies will be provided so teams can inspect, modify, and integrate hardware quickly.
:::

### Source Code

:::tip
Core software repositories, setup instructions, and example pipelines will be released for direct use and extension.
:::

### Simulation Assets

:::tip
Simulation assets and environment packages will be shared to support training, validation, and sim-to-real workflows.
:::

### Tutorials

:::tip
Step-by-step tutorials from bring-up to policy deployment will be added so new users can get started fast.
:::

</div>

## More Information and Questions

>If you have any questions or would like more information, please reach out on our Discord and we will respond as quickly as possible.
>You can also learn more about ROBOTIS products and services on our Discord.
>[Join our Discord](https://discord.gg/robotis)
</div>

