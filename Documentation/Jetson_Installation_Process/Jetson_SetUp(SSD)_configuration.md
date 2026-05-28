# OS & Storage Installation Report — Jetson Orin Nano

**Project:** Autonomous Navigation of a Wheelchair Using a Camera and a 3D LiDAR
**Hardware:** NVIDIA Jetson Orin Nano (8GB) Super Developer Kit
**Date:** 2026-05-28

---

## 1. Overview

This report documents how the Jetson Orin Nano was set up to boot natively from an NVMe SSD (instead of a microSD card) and how Ubuntu / JetPack were installed. It also records practical lessons learned during the process that are easy to miss but important for a clean install.

**Reference followed:** "The RIGHT Way to Boot NVIDIA Jetson Orin Nano from SSD (No Hacks Needed)" — https://youtu.be/f9lDqQ0QwOM

This method uses the official **NVIDIA SDK Manager** running on an x86 Linux host to flash JetPack directly onto the SSD, avoiding the unofficial SD-card-then-migrate workarounds.

---

## 2. Hardware used

| Component | Detail |
|---|---|
| Board | Jetson Orin Nano 8GB Super Developer Kit |
| Storage | Verbatim Vi3000 PCIe NVMe M.2 SSD, 256 GB |
| Slot | M.2 Key M (PCIe NVMe — SATA M.2 drives are NOT supported) |
| Host PC for flashing | x86 machine running Ubuntu (20.04 or 22.04) |
| Connection | USB-C cable (host ↔ Jetson) for flashing in recovery mode |

> **Note on the SSD:** The Verbatim Vi3000 is a PCIe NVMe drive, which is the required type. The Orin Nano's M.2 Key M slot does not accept SATA drives.

---

## 3. Procedure summary

### 3.1 Physical SSD install
1. Power off the Jetson and remove from any enclosure.
2. Insert the Verbatim Vi3000 NVMe SSD into the M.2 Key M slot, seat it, and secure with the standoff screw.

### 3.2 Flashing with NVIDIA SDK Manager (from x86 host)
1. On the x86 Linux host, install and log into **NVIDIA SDK Manager** (requires a free NVIDIA developer account).
2. Put the Jetson into **Force Recovery Mode** (jumper the FC REC and GND pins on the button header, then power on / connect USB-C to the host).
3. In SDK Manager, select the target device: **Jetson Orin Nano (8GB Developer Kit)**.
4. Choose **JetPack 6.2** as the SDK version.
5. In storage/target options, select the **NVMe SSD** as the target storage device (not eMMC / SD).
6. Accept terms, enter the host password when prompted, and let SDK Manager download and flash. This step takes a while (downloads are several GB).

> **Warning:** Flashing **formats the target storage** — any existing data on the SSD is erased and not recoverable. Around 40 GB of free space is needed on the host for the BSP and disk image files.

### 3.3 First boot & Ubuntu setup (oem-config)
After flashing, the Jetson boots from the SSD into the Ubuntu first-time setup (oem-config) wizard. Steps: accept the license, choose language / keyboard / timezone, then create the user account and set the device name.

---

## 4. Lessons learned (important, easy to miss)

### 4.1 Do NOT use uppercase in the username
During Ubuntu account creation, **use a lowercase username only** (e.g. `wheelchairrob`, not `WheelchairRob`).

- Linux usernames are conventionally lowercase. Uppercase characters can cause warnings or be rejected by the account creation step, and can lead to inconsistencies later with home-directory paths, SSH, and scripts.
- The full/display name can contain uppercase and spaces; the **username** (login + home folder) should stay lowercase, no spaces.

### 4.2 Ensure internet connectivity before installing JetPack components
Once Ubuntu is up on the Jetson, **connect to the internet (Wi-Fi or Ethernet) BEFORE installing the JetPack runtime/SDK components.**

- If SDK Manager's "Step 03" (component install on the target) is run, or if components are installed later via `sudo apt install nvidia-jetpack`, the device needs a working internet connection to pull CUDA, cuDNN, TensorRT, OpenCV, and related libraries.
- Without connectivity, the CUDA / JetPack dependencies fail to install or install incompletely, leaving a broken or partial development environment that is painful to diagnose afterwards.
- **Recommended:** verify connectivity first (e.g. `ping -c 3 archive.ubuntu.com`), then proceed with component installation.

### 4.3 Installing JetPack components on-device (if skipped during flashing)
If only the OS was flashed (custom install, SDK components skipped), install the full JetPack meta-package on the Jetson once it has internet:
```bash
sudo apt update
sudo apt install nvidia-jetpack
```
This is often faster than installing components through SDK Manager over USB.

---

## 5. Result

| Item | Value |
|---|---|
| Boot device | Verbatim Vi3000 NVMe SSD (256 GB) — root filesystem on `/dev/nvme0n1p1` |
| OS | Ubuntu 22.04.5 LTS (Jammy) |
| JetPack | 6.2.2 |
| L4T | R36 Rev 5.0 |
| CUDA | 12.6 |
| Username | lowercase (`wheelchairrob`) |

The Jetson boots directly from the SSD with no microSD card required, giving significantly more storage (256 GB vs a typical SD card) and much faster disk I/O — important for compiling large ROS 2 workspaces and recording sensor data.

---

## 6. Tips for next time / troubleshooting

- **Recovery mode heat:** the board can get warm while sitting in Force Recovery Mode (the fan does not spin in that mode). This is normal; complete the flash promptly.
- **No VM for flashing:** flashing from a virtual machine is unreliable (USB re-enumeration issues). Use a native x86 Linux host.
- **Host disk space:** keep ~40 GB+ free on the host for image files.
- **OpenCV + CUDA:** the OpenCV that ships from the NVIDIA repository is not CUDA-enabled by default; a custom build is required if CUDA-accelerated OpenCV is needed.
- **Verify after first boot:** `nvcc --version` (CUDA), `sudo apt-cache show nvidia-jetpack | grep Version` (JetPack), and `cat /etc/nv_tegra_release` (L4T) confirm a healthy install.
