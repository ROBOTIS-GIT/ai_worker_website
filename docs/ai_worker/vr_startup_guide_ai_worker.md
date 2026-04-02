# VR Startup Guide

## 1. Clone the repository

1. At the `USER PC`, clone the repository

```bash
cd ~/
git clone -b jazzy https://github.com/ROBOTIS-GIT/robotis_applications.git
cd robotis_applications
```

2. At the `Robot PC`, clone the repository
```bash
ssh robotis@ffw-SNPR48A0000.local (replace SNPR48A0000 with the serial number printed on the back of the robot body)
```
```bash
cd ~/
git clone -b jazzy https://github.com/ROBOTIS-GIT/robotis_applications.git
cd robotis_applications
```

## 2. Start the Docker container

```bash
cd robotis_applications/docker && ./container.sh start
```

## 3. Shell alias (optional)

If your workflow uses a `vr` alias to start VR-related commands inside the container or host, add it to `~/.bashrc` (or your shell config) as described in your team documentation.

```bash
alias vr='...'   # replace with the command from your project docs
```

## 4. Open the Vuer page (Quest browser)

Use the headset browser (or a browser on the same network, if applicable) and go to:

```text
https://{pc_ip}:8012?ws=wss://{pc_ip}:8012
```

Example when the worker PC is at sg2's Orin `192.168.6.2`:

```text
https://192.168.6.2:8012?ws=wss://192.168.6.2:8012
```

Replace `{pc_ip}` with the actual IP of the machine hosting the Vuer server.

### Self-signed HTTPS warning

If the browser shows a certificate warning, use **Advanced** → proceed to the site (wording may vary by browser).

![Browser advanced / certificate bypass](/public/vr/advance_webpage.png)

![Proceed to site (unsafe)](/public/vr/proceed_webpage.png)

### Enter VR

Click **Enter VR**.

![Enter VR button](/public/vr/enter_vr_webpage.png)

When **passthrough** is active and you see axis markers on your hands, the session is ready.

![Passthrough with hand axes](/public/vr/pass_meta.png)

**Notice**: if you stop the vuer server, you need to refresh the page and clicking the **Enter VR** button again.


# Troubleshooting
1. If ROS communication is not working: check the ROS_DOMAIN_ID. (ROS_DOMAIN_ID is set to 30 in the container.)
2. If the Vuer server is not running: check the logs in the terminal.
3. If value updates are slow: check your Wi-Fi connection. Network performance has a major effect. A wired connection is recommended.

