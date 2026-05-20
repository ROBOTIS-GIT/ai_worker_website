# Cyclo Manager

**Cyclo Manager** is the web-based control panel on the **AI Worker Orin**. Install it with the CLI, open the UI on port **3000**, then use the sections below to bring up the robot, inspect ROS 2 topics, manage containers, and run tools such as Dynamixel Wizard 2.0 via noVNC.

## Contents

<!-- no toc -->

- [Install Cyclo Manager CLI](#install-cyclo-manager-cli)
- [Open the Cyclo Manager Web UI](#open-the-cyclo-manager-web-ui)
- [Cyclo Manager Home Page](#cyclo-manager-home-page)
- [Bring up the Robot (Follower/Leader)](#bring-up-the-robot-follower-leader)
- [Bring up the Physical AI Server and Zenoh Daemon](#bring-up-the-physical-ai-server-and-zenoh-daemon)
- [Set Robot Pack Position](#set-robot-pack-position)
- [View ROS 2 Topics and Messages](#view-ros-2-topics-and-messages)
- [Change ROS_DOMAIN_ID](#change-ros-domain-id)
- [Run Commands in a Container](#run-commands-in-a-container)
- [Use Dynamixel Wizard 2.0 on noVNC](#use-dynamixel-wizard-2-0-on-novnc)

## Install Cyclo Manager CLI
**Cyclo Manager** runs inside Docker containers. The Python package **`cyclo-manager`** on PyPI provides the **`cyclo_manager`** CLI, which uses Docker images and Compose to bring up and configure the container stack.  

:::info
**Cyclo Manager** runs on the **AI Worker Orin**. Install it on the **AI Worker Orin**.
:::


1. Connect to the AI Worker Orin via SSH.

Replace `SNPR48A0000` with the serial number printed on the back of the robot body.

```bash
ssh robotis@ffw-SNPR48A0000.local
```

2. Install the Python package `cyclo-manager` on the AI Worker Orin.

```bash
pip install cyclo-manager
```

3. Add the CLI to your `PATH` and create the containers.

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

- `cyclo_manager up`:
  - Starts the **cyclo_manager** (API) and **cyclo_manager_ui** containers. It also **creates** the **novnc-server** and **zenoh-daemon** containers but **does not start** them yet (start them later when needed).

- `cyclo_manager down`:
  - **Stops and removes** the containers created by `cyclo_manager up` for this stack (same role as **docker compose down**).

- `cyclo_manager update`:
  - Runs **down**, upgrades the CLI with **pip install -U cyclo-manager**, then runs **up** again so you are on the latest package and images.
:::

## Open the Cyclo Manager Web UI

Open the Cyclo Manager **web UI** on **port 3000** in your browser.

:::info
**Connect using the robot IP address or the mDNS hostname.** Replace `SNPR48A0000` with your robot serial number.

http://192.168.6.2:3000  
http://ffw-SNPR48A0000.local:3000
:::
![Cyclo Manager App Page](/advanced_features/cyclo_manager/app_page.png)

This is the main page of the Cyclo Manager web UI. From here, you can open the Cyclo Manager or Physical AI Tools pages.

![Sidebar](/advanced_features/cyclo_manager/side_bar.png)

The top-left toolbar provides quick access to the main areas of the web UI:

- **Theme toggle**: Switch between light and dark UI themes.
- **Home**: Return to the app page.
- **M**: Navigate to the Cyclo Manager home page.
- **P**: Navigate to the Physical AI Tools page.

## Cyclo Manager Home Page
![Cyclo Manager Home Page](/advanced_features/cyclo_manager/home_page.png)

This page shows containers running on the system that Cyclo Manager manages. Select a container to manage it.

![Container Card](/advanced_features/cyclo_manager/home_page_container_card.png)

In the container card, you can check the container status and start or stop the container. After you open a container, use the left sidebar to open **System**, **Topic**, **Docker**, or **noVNC**.

## Bring up the Robot (Follower/Leader)

From the **Home** page, select the robot container to open the **System** page. On the **System** page, you can bring up the robot, the Physical AI server, and the Zenoh daemon.

![System Page](/advanced_features/cyclo_manager/system_page.png)  

![Robot Bringup Button](/advanced_features/cyclo_manager/robot_bringup_button.png)

- **Robot Type Selection**: Select the robot model to bring up (SG2, BG2, SH5, BH5).
- **Robot Bringup Button**: Bring up the robot.
- **Parameter Setup**: Open launch parameters (gear icon ⚙️).
- **Logs**: View robot bring-up logs.
- **Status Light**: Red indicates the robot is not running. Green indicates the robot is running.

![System Bringup](/advanced_features/cyclo_manager/system_page_bringup.png)

After you click the **Robot Bringup** button, the robot stack starts and the 3D model is displayed. Click **Logs** to view bring-up output.

:::info Log Clear Button Description
<div class="cyclo-bringup-legend">
<div class="cyclo-bringup-legend__text">
<p>Robot bring-up logs are stored in the AI Worker container at <code>/var/log/ai_worker_bringup/current</code>.</p>
<p>On the System page, you can delete stored robot bring-up logs using the <strong>Clear</strong> button.</p>
</div>
<div class="cyclo-bringup-legend__media">

<img src="/advanced_features/cyclo_manager/log_clear_button.png" alt="log clear button" loading="lazy" />

</div>
</div>
:::

## Bring up the Physical AI Server and Zenoh Daemon

On the **System** page, click the **Bringup** button in the Physical AI Server and Zenoh Daemon section to start both services.

![Physical AI Server and Zenoh Daemon Bringup Button](/advanced_features/cyclo_manager/server_run_button.png)

- **Bringup Button**: Bring up the Physical AI server and Zenoh daemon.
- **Logs**: View service logs.
- **Status Light**: Red indicates the services are not running. Green indicates they are running.

## Set Robot Pack Position

![Robot parameter gear button](/advanced_features/cyclo_manager/robot_parameter_button.png)
Click the gear icon (⚙️) to configure the robot parameters on the **System** page.

![Parameter setup dialog](/advanced_features/cyclo_manager/parameter_setup.png)

Set **Init Position File** to `pack_position.yaml`.

![Robot bringup and stop buttons](/advanced_features/cyclo_manager/robot_bringup_button_red_box.png)

Then bring up the robot by clicking the **Robot Bringup** button. The robot moves to the pack position defined in `pack_position.yaml` during bring-up. When you are done, click the **Robot Stop** button to stop the robot.

## View ROS 2 Topics and Messages

From the **Home** page, open the robot container, then select **Topic** in the left sidebar.

![Topic page](/advanced_features/cyclo_manager/topic_page.png)

Click **Topic list** to show available ROS 2 topics and inspect message data.

If the topic list is empty, check that **`ROS_DOMAIN_ID`** matches between the Cyclo Manager container and the AI Worker container. To change it, see [Change ROS_DOMAIN_ID](#change-ros-domain-id).

## Change ROS_DOMAIN_ID

From the **Home** page, open a container, then select **Docker** in the left sidebar.

![Docker page](/advanced_features/cyclo_manager/docker_page.png)

Click the gear icon (⚙️) and open the **`.bashrc`** section. Set `ROS_DOMAIN_ID` to the same value on every container that must communicate over ROS 2 (for example `export ROS_DOMAIN_ID=30`).

![Editing container .bashrc](/advanced_features/cyclo_manager/modifying_bashrc_cyclo_manager.png)

After you save **`.bashrc`**, click **Restart** to restart the container.

:::info
**Environment variables in the container are managed in the `.bashrc` file.**
:::

## Run Commands in a Container

From the **Docker** page, click **Terminal** to open the in-container shell.

![Terminal page](/advanced_features/cyclo_manager/terminal_page.png)

Use a bash terminal to run commands and manage processes inside the container without using SSH to access the container.

## Use Dynamixel Wizard 2.0 on noVNC

From the **Home** page, open the robot container, then select **noVNC** in the left sidebar.

1. Click the **Start** control at the top of the page to start the noVNC container.
2. Reload the page (for example press **F5**).
3. Open the **Dynamixel Wizard 2.0** icon on the desktop.

![noVNC page](/advanced_features/cyclo_manager/novnc_page.png)

You can diagnose and control motors in the browser without attaching a display or laptop to the AI Worker.

<style>
/* Log clear legend: text left, screenshot right (this page only) */
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
