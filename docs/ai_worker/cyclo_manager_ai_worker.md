# Cyclo Manager

**Cyclo Manager** is an integrated control system for the Cyclo ecosystem. Through a web UI you can bring up the robot, use a topic viewer for ROS 2 traffic, manage Docker-backed services, and open noVNC sessions in the browser. A management API, Docker Compose layouts, and a pip-installable CLI (for example `cyclo_manager up`) support the same workflow on field hardware and developer PCs.

## Install

**Cyclo Manager** runs inside Docker containers. The Python package **`cyclo-manager`** on PyPI provides the **`cyclo_manager`** CLI, which uses Docker images and Compose to bring up and configure the container stack.  

:::info
**Cyclo Manager** is operated on the **AI Worker Orin**. So you should install it on the **AI Worker Orin**.
:::


1. Connect to the AI Worker Orin via SSH.
```bash
ssh robotis@ffw-SNPR48A0000.local
```

2. Install the Python package `cyclo-manager` on the AI Worker Orin.

```bash
pip install cyclo-manager
```

3. Set up the environment variables and create the containers:

```bash
export PATH="$HOME/.local/bin:$PATH"
cyclo_manager up
```

::: tip CLI commands
Use the **`cyclo_manager`** CLI from your terminal:

```bash
cyclo_manager up   # optional: --pull to fetch the latest images from the registry
cyclo_manager down
cyclo_manager update
```

- `cyclo_manager up` :
  - Starts the **cyclo_manager** (API) and **cyclo_manager_ui** containers. It also **creates** the **novnc-server** and **zenoh-daemon** containers but **does not start** them yet (start them later when needed).

- `cyclo_manager down` :
  - **Stops and removes** the containers that **up** created for this stack (same role as **docker compose down**).

- `cyclo_manager update` :
  - Runs **down**, upgrades the CLI with **pip install -U cyclo-manager**, then runs **up** again so you are on the latest package and images.
:::

## App Page
Open the Cyclo Manager **web UI** on **port 3000** in your browser.

:::info
**Connect using the robot IP address or the mDNS hostname.**  
http://192.168.6.2:3000  
http://ffw-SNPR48A0000.local:3000
:::
![Cyclo Manager App Page](/advanced_features/cyclo_manager/app_page.png)

- `Cyclo Manager` : Navigates to the Cyclo Manager Home page where you can select the container to manage.
- `Physical AI Tools` : Navigates to the Physical AI Tools page.

## Home Page

![Cyclo Manager Home Page](/advanced_features/cyclo_manager/home_page.png)

- This page shows the containers supported by Cyclo Manager that are running on the system.
- Select the container to manage and navigate to the corresponding page.
- If the source inside the corresponding container is not at the latest version, a notification appears indicating that an update is available.

## System Page

![System Page](/advanced_features/cyclo_manager/system_page.png)  

:::info Button Descriptions
<div class="cyclo-bringup-legend-stack">
<div class="cyclo-bringup-legend">
<div class="cyclo-bringup-legend__text">
<p>▶️ : Bring up the robot</p>
<p>⚙️ : Set parameters</p>
<p>📄 : See Logs</p>
</div>
<div class="cyclo-bringup-legend__media">

<img src="/advanced_features/cyclo_manager/robot_bringup_button.png" alt="Robot bringup control buttons" loading="lazy" />

</div>
</div>
<div class="cyclo-bringup-legend">
<div class="cyclo-bringup-legend__text">
<p>▶️ : Run button</p>
<p>📄 : See Logs</p>
</div>
<div class="cyclo-bringup-legend__media">

<img src="/advanced_features/cyclo_manager/server_run_button.png" alt="Server run and log controls" loading="lazy" />

</div>
</div>
</div>
:::

![System Bringup](/advanced_features/cyclo_manager/system_page_bringup.png)
- You can bring up the robot and view logs.
- You can run the Physical AI server and view logs.
- You can run the Zenoh daemon container (zenohd).

![Parameter Setup](/advanced_features/cyclo_manager/parameter_setup.png)
- Click the gear icon (⚙️) to configure the robot parameters.
- You can also set the pack position by specifying the initial position parameter in `pack_position.yaml`.

The gear icon opens the **Follower Bringup (SG2) Launch Arguments** dialog. Default values:

| Parameter | Default | Description |
| :--- | :--- | :--- |
| Start RViz | `false` | Launch RViz with the physical hardware.|
| Use Simulation | `false` | Use simulation environment timestamp.|
| Use Mock Hardware | `false` | Use mock hardware.|
| Mock Sensor Commands | `false` | Use mock sensor. You can bring up the robot without sensors. |
| Port Name | `/dev/follower` | Port name for the hardware.|
| Launch Cameras | `true` | Whether to launch cameras.|
| Launch Lidar | `true` | Whether to launch lidar.|
| Init Position | `true` | Whether to initialize position.|
| Model | `ffw_sg2_rev1_follower` | Model name.|
| Use Head EEF Tracker | `false` | Whether to use the head EEF tracker. If enabled, head will track the middle of the hand.|
| Init Position File | `pack_position.yaml` | File name for the initial position file.|
| ROS2 Control Type | `ffw_sg2_follower` | ROS2 control type.|

:::info Log Clear Button Description
<div class="cyclo-bringup-legend-stack">
<div class="cyclo-bringup-legend">
<div class="cyclo-bringup-legend__text">
<p>Logs are stored in the AI Worker container at <code>/var/log/ai_worker_bringup/current</code>.</p>
<p>You can delete stored logs using the Clear button.</p>
</div>
<div class="cyclo-bringup-legend__media">

<img src="/advanced_features/cyclo_manager/log_clear_button.png" alt="log clear button" loading="lazy" />

</div>
</div>
</div>
:::

## Topic Page
![Topic Page](/advanced_features/cyclo_manager/topic_page.png)
On the Topic page, you can inspect the topic list and each topic’s data.  
The Topic list button makes the topic list visible.  

:::info
**If the topic list is not visible, check that the ROS_DOMAIN_ID matches between the Cyclo Manager container and the AI Worker container.**  
**You can change ROS_DOMAIN_ID in the Cyclo Manager container’s `.bashrc`. After editing it, restart the Cyclo Manager container.**  
:::

## Docker Page
![Docker Page](/advanced_features/cyclo_manager/docker_page.png)
On the Docker page, you can manage the list of Docker containers running on the PC.  
You can start, stop, and restart containers.  
![Modifying Bashrc Cyclo Manager](/advanced_features/cyclo_manager/modifying_bashrc_cyclo_manager.png)
Click the **Settings** button, open the **`.bashrc`** section, and you can edit the bashrc file for the container.  

:::info
**Environment variables in the container are managed in the `.bashrc` file.**  
:::

![Terminal Page](/advanced_features/cyclo_manager/terminal_page.png)
- Click the terminal button to open the terminal page.
- You can use the bash terminal to run commands in the container.
- You can also manage the process list in the container.

## noVNC Page
![noVNC Page](/advanced_features/cyclo_manager/novnc_page.png)
On the noVNC page, you can use **Dynamixel Wizard 2.0** after you start the noVNC container with the controls above.  
This lets you diagnose and control motors with the wizard in the browser, without attaching a display or laptop to the AI Worker.  

<style>
/* Robot bringup legend: text left, screenshot right (this page only) */
.cyclo-bringup-legend-stack {
  display: flex;
  flex-direction: column;
  gap: 1.25rem;
}

.cyclo-bringup-legend {
  display: flex;
  flex-direction: row;
  align-items: flex-start;
  gap: 1.25rem;
}

.cyclo-bringup-legend__text {
  flex: 1;
  min-width: 0;
}

.cyclo-bringup-legend__text p {
  margin: 0.35rem 0;
}

.cyclo-bringup-legend__text p:first-child {
  margin-top: 0;
}

.cyclo-bringup-legend__text p:last-child {
  margin-bottom: 0;
}

.cyclo-bringup-legend__media {
  flex-shrink: 0;
  width: min(300px, 42%);
}

.cyclo-bringup-legend__media img {
  width: 100%;
  height: auto;
  display: block;
  border-radius: 6px;
}

@media (max-width: 640px) {
  .cyclo-bringup-legend {
    flex-direction: column;
  }

  .cyclo-bringup-legend__media {
    width: 100%;
    max-width: 360px;
  }
}
</style>
