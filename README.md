
# READ.ME file

---

## Project Overview

This repository simulates **UAV landing maneuvers** onto **USV marine platforms** using **Gazebo Sim (Garden)**. It integrates the [VRX](https://github.com/osrf/vrx/tree/humble) environment with [PX4-Autopilot](https://github.com/Darkjrcy/PX4-Autopilot/tree/my-px4-v1.15.4) tools.
### Repository Structure
- **`PX4-Autopilot`**: Contains PX4 version 1.15.4, featuring realistic UAV models compatible with Gazebo Garden.
- **`april-tag-imgs`**: A collection of AprilTag examples across various families for visual docking.
- **`gazebo_models`**: A library for generating SDF AprilTag models to be used within Gazebo worlds.
- **`ros_ws`**: The primary ROS 2 workspace containing all packages related to landing maneuvers and USV simulation.
- **`Obsidian.zip`**: A comprehensive Obsidian Vault containing detailed documentation on simulation logic, startup procedures, and deep-dive customization parameters.

The simulation primarily relies on **ROS 2 Humble** and **Gazebo Garden**.

---
## Dependencies
### 1. Core Simulation Setup (Gazebo Garden)

> [!CAUTION] If your ROS 2 Humble installation defaults to Gazebo Ignition, you must purge the old libraries to avoid conflicts with Garden:

``` bash
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
```
### 2. PX4-ROS 2 Communication
The system supports two different approaches for interfacing with the PX4-Autopilot. **Choose the one that fits your workflow:**

#### Option A: MAVSDK (MAVLink)
Used for high-level command execution. Requires building from source:
``` bash
git clone https://github.com/mavlink/MAVSDK.git
cd MAVSDK && git checkout v2.0 # Adjust version as needed
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

#### Option B: Micro XRCE-DDS (Middleware)
Provides a direct bridge between ROS 2 topics and PX4 internals:
``` bash
sudo snap install micro-xrce-dds-agent --edge
```

---

### 3. Computer Vision (AprilTag Recognition)
To enable autonomous landing, the system requires the official AprilTag detection library and its ROS 2 wrappers.
**Step 1: Install C Library**
``` bash
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag && mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

**Step 2: Install ROS 2 Integration**
``` bash
sudo apt update
sudo apt install ros-humble-apriltag \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs \
  ros-humble-apriltag-msgs
```

---
## Launch Files
The simulation is managed through two primary launch files located within the `robotx_bringup` package. 
### 1. Standard Simulation: `main_usv_px4.launch.py`
This is he base launch file. it initializes the VRX competition environment alongside a PX4-controlled UAV.
**What it does:**
- **Environment:** Launches the VRX world with specified `gz-ros` bridges.
- **Synchronization:** Waits for the Gazebo world to load before spawning the UAV.
- **Bridge & Communication:** Automatically starts the **Micro XRCE-DDS Agent** and initializes a ROS 2 node to track the USV’s position.
**To Launch:** After sourcing ROS 2 Humble and building the workspace with `--merge-install`:
``` bash
colcon build --merge-install
source install/setup.bash
ros2 launch robotx_bringup main_usv_px4.launch.py
```

### 2. Vision-Aided Simulation: `usv_px4_apriltags.launch.py`
This file extends the main launch by adding visual markers to the USV platform for precision landing.
**What it does:**
- **Inheritance:** Includes all functionality from `main_usv_px4.launch.py`.
- **Dynamic Markers:** Spawns custom AprilTag models onto the WAM-V at specified positions.
- **Scalability:** While configured for `marker0` by default, the launch file is designed to handle multiple tags simultaneously.
> [!IMPORTANT] Ensure the AprilTag models have been added to your `wamv_description` models folder before running this file.

**To Launch:**
``` bash
ros2 launch robotx_bringup usv_px4_apriltags.launch.py
```

---
## Execution
To start the landing maneuver simulation, follow these steps:

### 1. Launch the Environment
Choose the launch file that includes AprilTag support for visual docking:
``` bash
ros2 launch robotx_bringup usv_px4_apriltags.launch.py
```

### 2. Execute the Landing Strategy
Once the environment is loaded, run the landing controller. This node sends position commands to the UAV using **Offboard Mode** via the **Micro XRCE-DDS** bridge:
``` bash
ros2 run px4_exec landing_strategy
```
> [!NOTE] **Work in Progress:** AprilTag detection and position estimation are currently under development. You can run the experimental detector node using the command below, but note that the estimation logic is not yet finalized: 

 ``` bash
 ros2 run px4_exec apriltag_detector
```
---

##  Customization
Most launch files and executable in this repository utilize custom parameters for tuning. For a deep dive into the available parameters, library architecture, and development history, please refer to the notes within the **Obsidian Vault** included in this repository.
