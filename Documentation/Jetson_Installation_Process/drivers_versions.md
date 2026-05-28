# Driver Versions — Jetson Deployment

**Captured:** 2026-05-27  
**Platform:** Jetson Orin Nano (ARM64, JetPack 6.2.2, L4T 36.5)  
**ROS 2 distribution:** Humble Hawksbill  
**ZED SDK:** 5.3.0

---

## Pinned versions of third-party drivers

These drivers are cloned manually per platform (not tracked in this repo via `.gitignore`).  
For reproducibility, the exact commits used are documented below.

### ZED ROS 2 Wrapper
- **Repository:** https://github.com/stereolabs/zed-ros2-wrapper
- **Branch:** `master`
- **Commit SHA:** `87187ba`
- **Notes:** Aligned with ZED SDK release 5.3. Compatible with the installed SDK 5.3.0.
- **Submodule:** `zed-ros2-interfaces` (cloned recursively)

### RoboSense LiDAR SDK
- **Repository:** https://github.com/RoboSense-LiDAR/rslidar_sdk
- **Branch:** `main`
- **Commit SHA:** `78d2abb`
- **Tag:** `v1.5.19`
- **Notes:** Stable tagged release. Includes `rs_driver` as submodule.

### RoboSense ROS 2 Messages
- **Repository:** https://github.com/RoboSense-LiDAR/rslidar_msg
- **Branch:** `master`
- **Commit SHA:** `fe8a95c`
- **Tags:** `v1.5.8`, `v1.5.9`, `v1.5.10`
- **Notes:** Same SHA as the original x86 setup — full consistency between platforms.

---

## Reproducing this exact setup

```bash
cd ~/Thesis/thesis_ws/src/drivers

git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd zed-ros2-wrapper && git checkout 87187ba && cd ..

git clone --recursive https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk && git checkout v1.5.19 && cd ..

git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
cd rslidar_msg && git checkout fe8a95c && cd ..
```

---

## Checking for updates later

To see if newer versions are available without pulling them:

```bash
cd ~/Thesis/thesis_ws/src/drivers/zed-ros2-wrapper && git fetch && git log HEAD..origin/master --oneline
cd ../rslidar_sdk && git fetch --tags && git tag -l | tail -5
cd ../rslidar_msg && git fetch --tags && git tag -l | tail -5
```
