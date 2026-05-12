# Autonomous Navigation of a Wheelchair Using a Camera and a 3D LiDAR

Thesis project — Universidad Nacional de Colombia.  
**Author:** Felipe Forero (anforero@unal.edu.co)

---

## Overview

This repository contains all software developed for the autonomous navigation of a wheelchair platform equipped with a **ZED 2i** stereo camera and a **RoboSense RS-Helios-16P** 3D LiDAR. The system is developed and deployed natively on two platforms running **ROS 2 Humble** side by side:

- **x86 PC** — development, simulation, and bag processing (Ubuntu 22.04, NVIDIA GPU)
- **NVIDIA Jetson** — onboard real-time execution

The work covers four main areas:

| Area | Location | Description |
|------|----------|-------------|
| MPC Control | `MPC_MATLAB/` | Model Predictive Control design and simulation in MATLAB/Simulink |
| CAD Model | `CAD_MODELS/` | Wheelchair Fusion 360 model and exported STL meshes |
| Gazebo Simulation | `thesis_ws/src/gazebo/` | Ignition Gazebo simulation with diff-drive and LiDAR sensors |
| Sensor Fusion | `thesis_ws/src/sensorFusion/` | LiDAR–camera point cloud merge and object association |

---

## Repository Structure

```
Thesis/
├── CAD_MODELS/
│   └── ACTUAL_SET_UP/            # Fusion 360 (.f3d) and STL of the wheelchair
│
├── MPC_MATLAB/                   # MATLAB/Simulink MPC controllers
│   ├── RobotModel.mlx            # Kinematic model derivation
│   ├── RobotSim.slx              # Simulink simulation
│   ├── MPC.m / FHOCP.m           # Finite-horizon OCP solvers
│   └── ParkingRobotModel.mlx     # Parking manoeuvre model
│
├── Wc_Integration/               # Initial Docker configs (kept for reference)
│   └── docker/
│       ├── x86_PC/
│       └── ARM_Jetson/
│
└── thesis_ws/
    └── src/
        ├── drivers/              # Hardware driver submodules
        │   ├── rslidar_msg       # RoboSense message definitions
        │   ├── rslidar_sdk       # RoboSense ROS 2 driver
        │   └── zed-ros2-wrapper  # ZED 2i ROS 2 wrapper (v4.2.5)
        │
        ├── gazebo/
        │   └── wheelchair_actual_set_up_description/  # Simulation package
        │
        ├── sensorFusion/
        │   ├── lidar_camera_fusion_pcl/               # Point cloud merge node
        │   └── lidar_camera_object_association/       # Object–LiDAR association node
        │
        └── bags/
            └── fusion_sensor/    # Bag post-processing scripts
                ├── TimestampsCorrector.py
                └── convert.yaml
```

---

## Hardware Setup

| Sensor / Board | Model | Interface |
|----------------|-------|-----------|
| Stereo camera | ZED 2i | USB 3.0 |
| 3D LiDAR | RoboSense RS-Helios-16P | Ethernet (MSOP) |
| Onboard computer | NVIDIA Jetson | — |
| Development PC | x86 with NVIDIA GPU | — |

---

## Software Requirements

Both platforms run the same software stack installed natively:

- **OS:** Ubuntu 22.04
- **ROS:** ROS 2 Humble (`ros-humble-desktop`)
- **ZED SDK:** Installed from the Stereolabs installer (provides CUDA-accelerated depth)
- **Simulator (x86 only):** Ignition Gazebo 6 (Fortress) via `ros_gz_sim`
- **MATLAB R2023a+** — MPC design only, x86 PC

### System Dependencies

```bash
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-xacro \
  ros-humble-zed-msgs \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  python3-colcon-common-extensions \
  python3-rosdep \
  libpcap-dev \
  libyaml-cpp-dev \
  libeigen3-dev
```

---

## Building the Workspace

Run this on both the x86 PC and the Jetson after cloning the repository.

```bash
cd thesis_ws

# Initialise hardware driver submodules
git submodule update --init --recursive

# Install remaining ROS dependencies
rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Gazebo Simulation (x86 PC)

The `wheelchair_actual_set_up_description` package provides a full Ignition Gazebo simulation of the wheelchair.

**Features:**
- Differential-drive controller (`diff_drive_controller`) via `ros2_control`
- Front and rear SICK TiM561-equivalent LiDAR sensors (`gpu_lidar`, 270° FOV, 15 Hz)
- Topic bridge between Ignition and ROS 2 (`ignition.msgs` namespace)

### Launch

```bash
ros2 launch wheelchair_actual_set_up_description gazebo.launch.py
# Headless:
ros2 launch wheelchair_actual_set_up_description gazebo.launch.py gui:=false
```

**Bridged topics:**

| ROS 2 Topic | Direction | Type |
|-------------|-----------|------|
| `/cmd_vel` | ROS → Gz | `geometry_msgs/Twist` |
| `/odom` | Gz → ROS | `nav_msgs/Odometry` |
| `/scan_front` | Gz → ROS | `sensor_msgs/LaserScan` |
| `/scan_rear` | Gz → ROS | `sensor_msgs/LaserScan` |
| `/tf` | Gz → ROS | `tf2_msgs/TFMessage` |
| `/clock` | Gz → ROS | `rosgraph_msgs/Clock` |

---

## Sensor Fusion Pipeline

The pipeline processes data from the ZED 2i and the RS-Helios-16P and has two stages.

### Bag Pre-processing

Real-session bags from the LiDAR may have misaligned `header.stamp` fields. Run the corrector before replaying:

```bash
cd thesis_ws/src/bags/fusion_sensor
python3 TimestampsCorrector.py
# Input:  session2Real_decompressed/
# Output: session2Real_fixed/
```

### Stage 1 — Point cloud merge (`lidar_camera_fusion_pcl`)

Merges the RS-Helios-16P point cloud (`/rslidar_points`) with the ZED 2i registered depth cloud (`/zed/zed_node/point_cloud/cloud_registered`) into a single dense cloud in a common TF frame. An optional voxel-grid filter pre-processes the ZED cloud before merging.

```bash
ros2 launch lidar_camera_fusion_pcl fusion_pcl.launch.py \
  bag:=src/bags/fusion_sensor/session2Real_fixed/session2Real_fixed_0.db3
```

**Key parameters** (`config/fusion_pcl_params.yaml`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_frame` | `zed_camera_center` | Common output frame |
| `voxel_size` | `0.05` | Voxel leaf size [m] (0 = off) |
| `slop` | `0.05` | ApproximateTime sync tolerance [s] |

### Stage 2 — Object association (`lidar_camera_object_association`)

Associates ZED 2i object detections (`/zed/zed_node/obj_det/objects`) with LiDAR points. For each detected bounding box, the node crops the LiDAR scan to the oriented bounding box (OBB), colours the surviving points by object ID, and publishes:

- `/fused/object_points` — XYZRGB coloured point cloud
- `/fused/object_markers` — OBB wireframes + ID labels for RViz2

```bash
ros2 launch lidar_camera_object_association fusion.launch.py \
  bag_path:=src/bags/fusion_sensor/session2Real_fixed/session2Real_fixed_0.db3
```

**Key parameters** (`config/params.yaml`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `lidar_frame` | `rslidar` | LiDAR TF frame |
| `camera_frame` | `zed_left_camera_frame` | ZED optical frame |
| `sync_slop` | `0.10` | Max timestamp difference for sync [s] |
| `cropbox_margin` | `0.05` | OBB expansion margin [m] |
| `min_object_points` | `3` | Minimum LiDAR points per object |

---

## MPC Control (MATLAB/Simulink)

The `MPC_MATLAB/` directory contains the Model Predictive Control design for the wheelchair.

| File | Description |
|------|-------------|
| `RobotModel.mlx` | Kinematic unicycle model derivation |
| `RobotSim.slx` | Closed-loop Simulink simulation |
| `FHOCP.m` / `FHOCP2.m` | Finite-horizon optimal control problem solvers |
| `MPC.m` | MPC controller implementation |
| `ParkingRobotModel.mlx` | Parking manoeuvre model |
| `RobotSimParking.slx` | Parking Simulink simulation |

Open any `.mlx` or `.slx` file in MATLAB R2023a or later.

---

## Notes on Large Data Files

ROS bag recordings (`.db3`) are excluded from version control via `.gitignore` because they are several gigabytes each. Session bags are stored locally under:

```
thesis_ws/src/bags/fusion_sensor/session*/
thesis_ws/src/bags/zed_sessions/
```
