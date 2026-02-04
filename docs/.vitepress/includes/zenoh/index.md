# Zenoh

ROBOTIS projects (AI Worker, OMY, OMX, and Hands) use **Zenoh** as the ROS 2 communication layer by setting the RMW to **RMW_ZENOH**.

## What is Zenoh?

**Zenoh** is an open-source protocol and middleware for efficient, location-transparent data distribution. It provides high-performance pub/sub and query abstractions across heterogeneous systems—from embedded devices to the cloud.

## RMW_ZENOH

In ROS 2, the **RMW** (ROS Middleware) is the layer that implements topics, services, and actions. We use **RMW_ZENOH** (`rmw_zenoh_cpp`) so that ROS 2 uses Zenoh instead of DDS for transport.

- **Same ROS 2 API** — Your nodes, topics, and launch files stay the same; only the underlying transport changes.
- **Set the RMW** — Use Zenoh by setting:
  ```bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  ```
- **Zenoh router** — For discovery, a Zenoh router (`zenohd` / `rmw_zenohd`) is typically run in the network. See the [official Zenoh docs](https://zenoh.io/) and [ROS 2 Zenoh documentation](https://docs.ros.org/en/rolling/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.html) for setup.

## Why Zenoh?

Zenoh is chosen for our stack to improve scalability, latency, and flexibility in wired and wireless setups, while keeping full compatibility with the ROS 2 ecosystem. Setup and usage details are covered in each product's **Setup Guide** and **Quick Start Guide**.

## Further reading

- [Zenoh — official site](https://zenoh.io/)
- [ROS 2 — Working with Zenoh](https://docs.ros.org/en/rolling/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.html)
- [rmw_zenoh (GitHub)](https://github.com/ros2/rmw_zenoh)
