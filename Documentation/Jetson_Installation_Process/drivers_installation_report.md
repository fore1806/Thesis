# Sensor Drivers Installation Report — Jetson Deployment

**Project:** Autonomous Navigation of a Wheelchair Using a Camera and a 3D LiDAR
**Platform:** NVIDIA Jetson Orin Nano (ARM64, JetPack 6.2.2 / L4T 36.5)
**ROS 2:** Humble Hawksbill · **Date:** 2026-05-28

---

## 1. Overview

This report documents the native (bare-metal, no Docker) installation and verification of the two sensor drivers used onboard the wheelchair platform:

- **Stereolabs ZED 2i** stereo camera (via `zed-ros2-wrapper`)
- **RoboSense RS-Helios-16P** 3D LiDAR (via `rslidar_sdk` + `rslidar_msg`)

Both drivers were built inside the ROS 2 workspace at `~/Thesis/thesis_ws` and verified to publish data correctly.

---

## 2. Third-party driver versions

These drivers are cloned per-platform and are git-ignored in the repository. Exact commits are recorded for reproducibility.

| Driver | Repository | Branch | Commit / Tag |
|---|---|---|---|
| zed-ros2-wrapper | stereolabs/zed-ros2-wrapper | master | `87187ba` (release_5.3) |
| rslidar_sdk | RoboSense-LiDAR/rslidar_sdk | main | `78d2abb` (tag v1.5.19) |
| rslidar_msg | RoboSense-LiDAR/rslidar_msg | master | `fe8a95c` (v1.5.8–1.5.10) |

Clone location: `~/Thesis/thesis_ws/src/drivers/`

---

## 3. ZED 2i Camera

### 3.1 SDK
- **ZED SDK:** 5.3.0 for L4T 36.5 (CUDA 12.6), installed under `/usr/local/zed/`
- Installer: `ZED_SDK_Tegra_L4T36.5_v5.3.0.zstd.run`
- AI module: installed · Python API (`pyzed`): installed and verified
- ROS wrapper option during SDK install: skipped (wrapper built separately from GitHub)

### 3.2 Runtime dependencies resolved
Missing shared libraries were installed manually after the SDK install:
- `libturbojpeg`, `libturbojpeg0-dev` (required by `pyzed`)
- Qt5 runtime libraries (`libqt5network5`, `libqt5sql5`, `libqt5gui5`, etc.) — required by the ZED GUI tools (`ZED_Diagnostic`, `ZED_Explorer`)

### 3.3 Verification
- `python3 -c "import pyzed.sl as sl"` → SDK version 5.3.0
- `ZED_Diagnostic` ran; reported "No GMSL driver detected" — expected for the USB ZED 2i (GMSL applies to ZED-X only)
- Camera detected on USB 3.0: serial `39158856`, FW 1523
- Node launched with: `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i`
- Camera opened at 1280x720 (HD720); neural depth model optimized on first launch
- Published topics verified, including:
  - `/zed/zed_node/rgb/color/rect/image`
  - `/zed/zed_node/depth/depth_registered`
  - `/zed/zed_node/point_cloud/cloud_registered`
  - `/zed/zed_node/imu/data`, `/zed/zed_node/odom`, `/zed/zed_node/pose`

> **Note:** The depth topic publishes `sensor_msgs/msg/Image` with `32FC1` encoding (float depth in metres). For 3D visualization, use the `point_cloud/cloud_registered` topic with an RViz2 PointCloud2 display rather than the raw depth image.

> **Performance note:** Point-cloud rate observed ~6–7 Hz with neural depth active. Can be increased by lowering depth mode or resolution in the ZED `common.yaml` if needed.

---

## 4. RS-Helios-16P LiDAR

### 4.1 Driver configuration
The `rslidar_sdk` v1.5.19 auto-detects ROS 2 via `find_package(ament_cmake)` — no manual `CMakeLists.txt` edit was required (unlike older versions). The `package.xml` uses `$ROS_VERSION` conditionals to select `ament_cmake` automatically.

Key `config/config.yaml` settings:
- `lidar_type: RSHELIOS_16P`
- `msop_port: 6699`, `difop_port: 7788`
- `send_point_cloud_ros: true`
- Output topic: `/rslidar_points`

### 4.2 Network configuration
| Parameter | Value |
|---|---|
| Jetson interface | `enP8p1s0` |
| Jetson IP | `192.168.1.102/24` (static) |
| LiDAR IP | `192.168.1.200` (factory default) |
| Link | 100 Mb/s Full Duplex |

The LiDAR connects directly to the Jetson Ethernet port; internet is provided separately over Wi-Fi (`wlP1p1s0`).

### 4.3 Verification
- Raw UDP packets confirmed with `tcpdump` before building:
  - MSOP (port 6699): streaming, 1248-byte packets, no kernel drops
  - DIFOP (port 7788): streaming at ~1 Hz
- Node launched with: `ros2 launch rslidar_sdk start.py`
- `/rslidar_points` published at ~10 Hz with valid point data

---

## 5. Build dependencies (apt issue resolved)

During `rosdep install`, a "held broken packages" error occurred. Root cause: several runtime libraries were one patch version behind the `-dev` packages they were paired with, and the installed versions were being kept back.

**Resolution:** refreshed the apt index (`sudo rm -rf /var/lib/apt/lists/* && sudo apt update`) and installed the matching runtime versions explicitly:
```
libssl3=3.0.2-0ubuntu1.23
libpq5=14.23-0ubuntu0.22.04.1
libmysqlclient21=8.0.45-0ubuntu0.22.04.1
libopenjp2-7=2.4.0-6ubuntu0.5
```
After this, `ros-humble-pcl-ros` and `ros-humble-pcl-conversions` installed cleanly, and `rosdep install` completed successfully.

Gazebo simulation packages were excluded on the Jetson (hardware-only deployment) by placing a `COLCON_IGNORE` file in `wheelchair_actual_set_up_description/` and scoping `rosdep` to `src/drivers` and `src/sensorFusion` only.

---

## 6. Build strategy (memory-safe on 8 GB Orin Nano)

Built incrementally to avoid out-of-memory freezes:
```
MAKEFLAGS="-j2" colcon build \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+
```
Build order: `rslidar_msg` → `rslidar_sdk` → `zed_wrapper` (+deps) → fusion packages.
All packages compiled successfully.

---

## 7. Status

| Component | Status |
|---|---|
| ZED SDK 5.3.0 | Installed & verified |
| zed-ros2-wrapper build | Success |
| ZED 2i publishing (image, depth, cloud, IMU) | Verified |
| rslidar_sdk + rslidar_msg build | Success |
| LiDAR publishing `/rslidar_points` @ ~10 Hz | Verified |
| Initial Fusion packages build | Success |
