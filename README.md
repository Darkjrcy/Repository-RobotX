# UAV Landing on USV Platforms Simulation

This repository simulates **UAV landing maneuvers** onto **USV (Unmanned Surface Vehicle) marine platforms** using **Gazebo Sim (Garden)**. It integrates the [VRX](https://github.com/osrf/vrx/tree/humble) environment with [PX4-Autopilot](https://github.com/Darkjrcy/PX4-Autopilot/tree/my-px4-v1.15.4) tools.

---

## Project Overview

The simulation environment focuses on high-fidelity maritime physics and autonomous drone control, primarily relying on **ROS 2 Humble** and **Gazebo Garden**.

### Repository Structure
- **`PX4-Autopilot`**: Contains PX4 version 1.15.4, featuring realistic UAV models compatible with Gazebo Garden.
- **`ros_ws`**: The primary ROS 2 workspace containing all packages related to landing maneuvers and USV simulation.
- **`gazebo_models`**: A library for generating SDF AprilTag models for use within Gazebo worlds.
- **`april-tag-imgs`**: A collection of AprilTag examples across various families for visual docking.
- **`Obsidian.zip`**: A comprehensive Obsidian Vault containing detailed documentation on simulation logic, startup procedures, and deep-dive customization parameters.

---

## Dependencies

### 1. Core Simulation Setup (Gazebo Garden)

> [!CAUTION]
> If your ROS 2 Humble installation defaults to Gazebo Ignition, you must purge the old libraries to avoid conflicts with Garden:

```bash
# 1. Remove legacy Gazebo/Ignition libraries
sudo apt remove --purge \
  "ros-humble-ros-gz*" \
  "ros-humble-ign-*" \
  "ros-humble-gazebo-ros*" \
  "gazebo*"
sudo apt autoremove --purge

# 2. Install Garden-specific bridges
sudo apt update
sudo apt install \
  ros-humble-ros-gzgarden-bridge \
  ros-humble-ros-gzgarden-sim \
  ros-humble-ros-gzgarden-interfaces
