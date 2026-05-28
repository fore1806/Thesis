# ROS 2 Humble Installation Report — Jetson Orin Nano

**Project:** Wheelchair Robotics — Autonomous Navigation & Perception Stack
**Report generated:** 2026-05-27 10:32 CEST
**Phase completed:** Phase 1 of 6 (System Preparation + ROS 2 base install)

---

## 1. System Identification

| Attribute | Value |
|---|---|
| Hostname | `ubuntu` |
| User | `wheelchairrob` |
| Hardware | NVIDIA Jetson Orin Nano (8GB) |
| CPU | ARM Cortex-A78AE, 6 cores @ aarch64 |
| RAM | 7.4 GiB physical |
| Storage | NVMe SSD `/dev/nvme0n1p1` — 233 GB total, 185 GB free (17% used) |
| OS | Ubuntu 22.04.5 LTS (Jammy) |
| Kernel | `5.15.185-tegra` (PREEMPT, built 2026-01-15) |
| JetPack | **6.2.2+b24** |
| L4T (Tegra) | **R36 Revision 5.0** (build date 2026-01-16) |
| CUDA | **12.6.68** (V12.6.r12.6) |

---

## 2. Pre-flight Configuration Applied

### 2.1 Swap configuration

- **16 GiB swap file** created at `/mnt/swapfile` on the NVMe SSD.
- Persisted in `/etc/fstab` for automatic mount on boot.
- Priority `-2` (default file-based swap priority).
- **Verification:** `swapon --show` confirms active.

> **Rationale:** The Orin Nano 8GB has insufficient RAM for full ROS 2 + Nav2 + ZED workspace compilation. Swap on NVMe (not eMMC/SD) avoids flash wear and provides acceptable throughput during builds.

### 2.2 ZRAM disabled

- Service `nvzramconfig` disabled via `systemctl disable nvzramconfig`.
- **Verification:** `systemctl is-enabled nvzramconfig` returns `disabled`.

> **Rationale:** ZRAM competes with the real NVMe swap and causes memory thrashing under heavy compilation loads.

### 2.3 Boot target

- **Current state:** `graphical.target` (GUI active).
- **Note:** Originally switched to `multi-user.target` (headless mode) during initial setup, then reverted by user for convenience during interactive configuration.
- **Action required before heavy `colcon build`:** Temporarily switch to multi-user mode with `sudo systemctl isolate multi-user.target` to free ~1 GB RAM during compilation.

### 2.4 Power mode

- **Current:** `15W` mode (reported as mode 0 by `nvpmodel -q`).
- ⚠️ **Action required:** Reapply MAXN with:
  ```bash
  sudo nvpmodel -m 0
  sudo jetson_clocks
  ```

---

## 3. ROS 2 Humble Installation

### 3.1 Repository configuration

- ROS 2 official apt repository added: `http://packages.ros.org/ros2/ubuntu jammy main`.
- GPG key installed at `/usr/share/keyrings/ros-archive-keyring.gpg`.
- Locale set to `en_US.UTF-8`.

### 3.2 Packages installed

| Package | Version | Purpose |
|---|---|---|
| `ros-humble-ros-base` | `0.10.0-1jammy.20260426.092717` | Core ROS 2 Humble runtime (no GUI tools) |
| `ros-dev-tools` | `1.0.1` | Developer toolkit (colcon, rosdep, vcstool, etc.) |

> **Note:** `ros-desktop-full` intentionally **not** installed to save RAM and disk. RViz2 and similar GUI tools can be added on-demand via `apt install ros-humble-rviz2` when needed.

### 3.3 Build toolchain

| Tool | Version | Notes |
|---|---|---|
| GCC/G++ | 11.4.0 (Ubuntu 11.4.0-1ubuntu1~22.04.3) | C++17/C++20 capable |
| CMake | 3.22.1 | Stock Jammy version, sufficient for ROS 2 Humble |
| Python | 3.10.12 | ROS 2 Humble's required Python version |

### 3.4 Colcon ecosystem (installed plugins)

| Plugin | Version | Status |
|---|---|---|
| `colcon-core` | 0.20.1 | up-to-date |
| `colcon-cmake` | 0.2.29 | up-to-date |
| `colcon-ros` | 0.5.0 | up-to-date |
| `colcon-clean` | 0.2.1 | up-to-date |
| `colcon-mixin` | 0.2.3 | up-to-date |

> A newer `colcon-parallel-executor 0.4.0` is available, but not needed — we'll use the sequential executor for memory safety on this hardware.

### 3.5 Rosdep

- Initialized: `/etc/ros/rosdep/sources.list.d/20-default.list` present.
- User cache populated in `~/.ros/rosdep/`.
- Updated successfully (run as user, not root — correct behavior).

### 3.6 Shell integration

- `source /opt/ros/humble/setup.bash` appended to `~/.bashrc`.
- `ROS_DISTRO` environment variable resolves to `humble`.

---

## 4. Verification Tests Passed

| Test | Result |
|---|---|
| `ros2 --help` returns full subcommand list | ✅ Pass |
| `g++ --version` reports 11.4.0 | ✅ Pass |
| `cmake --version` reports 3.22.1 | ✅ Pass |
| `colcon version-check` shows all plugins healthy | ✅ Pass |
| `ros2 pkg prefix rclcpp` returns `/opt/ros/humble` | ✅ Pass |
| `rclcpp` C++ headers present at `/opt/ros/humble/include/rclcpp/rclcpp/` | ✅ Pass (rclcpp.hpp, node.hpp, publisher.hpp, etc.) |

---

## 5. Resource Snapshot at Report Time

| Resource | State |
|---|---|
| RAM used / total | 2.5 GiB / 7.4 GiB (4.8 GiB available) |
| Swap used / total | 0 B / 15 GiB |
| Root filesystem | 36 GB used / 233 GB total |
| Power mode | 15W (mode 0) |
| Boot target | graphical.target |

---

## 6. Pending Phases

| Phase | Status | Notes |
|---|---|---|
| 1. ROS 2 Humble base install | ✅ **Complete** | Verified working |
| 2. ZED SDK 5.3 + zed-ros2-wrapper | ⏭️ **Next** | Use L4T 36.5 build, CUDA 12.6 |
| 3. RoboSense rslidar_sdk (RS-Helios-16P) | ⏳ Pending | Requires Ethernet reconfiguration for 192.168.1.x subnet |
| 4. Nav2, robot_localization, PCL stack | ⏳ Pending | Mostly apt binaries on ARM64 |
| 5. Safe workspace compilation | ⏳ Pending | `MAKEFLAGS="-j2" --executor sequential --parallel-workers 1` |
| 6. End-to-end sensor verification | ⏳ Pending | Topic rates, TF tree, joint RViz2 visualization |

---

## 7. Operational Notes & Known Gotchas

### Reversion commands (if needed)

```bash
# Re-enable GUI on boot
sudo systemctl set-default graphical.target

# Disable GUI on boot (recommended before heavy builds)
sudo systemctl set-default multi-user.target

# Temporarily stop GUI without changing default (RAM recovery during one session)
sudo systemctl isolate multi-user.target

# Remove swap (NOT recommended)
sudo swapoff /mnt/swapfile
sudo sed -i '/swapfile/d' /etc/fstab
sudo rm /mnt/swapfile

# Re-enable ZRAM (NOT recommended)
sudo systemctl enable nvzramconfig
```

### Monitoring commands during builds

```bash
# Real-time hardware monitoring (RAM, GPU, thermals)
sudo tegrastats

# RAM-focused with refresh
watch -n 1 free -h

# Active build processes
htop
```

### Network reservation for RoboSense LiDAR

The RS-Helios-16P will require `eth0` configured statically at **192.168.1.102/24** (UDP ports 6699 MSOP, 7788 DIFOP). When that phase begins, either:
- Free `eth0` from current use, or
- Add a USB-Ethernet adapter (Realtek RTL8153 / ASIX AX88179 recommended) for non-LiDAR network traffic.

---

## 8. Replication Quick-Reference

To replicate this exact state on another Jetson Orin Nano with JetPack 6.2.2:

```bash
# 1. Pre-flight
sudo nvpmodel -m 0 && sudo jetson_clocks
sudo fallocate -l 16G /mnt/swapfile
sudo chmod 600 /mnt/swapfile
sudo mkswap /mnt/swapfile
sudo swapon /mnt/swapfile
echo '/mnt/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
sudo systemctl disable nvzramconfig

# 2. Locale + repo
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Install
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-ros-base ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  python3-colcon-mixin build-essential cmake git

# 4. Rosdep
sudo rosdep init
rosdep update   # WITHOUT sudo

# 5. Shell integration
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

*End of report. Next phase: ZED SDK 5.3 installation for L4T 36.5.*
