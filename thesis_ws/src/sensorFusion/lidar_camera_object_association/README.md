# lidar_camera_Object_Association  (C++)

ROS 2 Humble **C++17** package for sensor fusion between a **ZED 2i** stereo
camera and a **RoboSense RS-Helios-16P** LiDAR.

---

## Architecture

```
/rslidar_points ─────────────────────────────────────────────────┐
                                                                  ▼
                                                    ┌──────────────────────┐
/zed/.../objects ─── ApproximateTimeSynchronizer ──►│  association_node    ├──► /fused/object_points  (PointCloud2 XYZRGB)
                                                    └──────────┬───────────┘
/tf + /tf_static ────────── tf2::Buffer ────────────────────►  │            └──► /fused/object_markers (MarkerArray OBB)
```

**Per detected object:**

| Step | Detail |
|------|--------|
| **TF2 transform** | Looks up `zed_left_camera_frame → rslidar` (latest available). Converts the `TransformStamped` to an `Eigen::Isometry3d` via `tf2_eigen`. |
| **Corner transform** | Multiplies all 8 ZED OBB keypoint corners by the 4×4 transform matrix. |
| **CropBox filter** | Projects each LiDAR point onto the three OBB principal axes; retains points inside `[0, edge_len] + margin`. Pure Eigen – **no PCL dependency**. |
| **RGB colouring** | Assigns a deterministic colour by `label_id % 8` and packs it into the `FLOAT32 rgb` field (RViz2 `RGB8` convention). |
| **Markers** | Publishes `LINE_LIST` OBB wireframes + `TEXT_VIEW_FACING` ID labels with configurable lifetime. |

---

## File Map

```
lidar_camera_Object_Association/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── lidar_camera_Object_Association/
│       └── association_utils.hpp     ← colour palette + CropBox (header-only)
├── src/
│   └── association_node.cpp          ← main fusion logic
├── config/
│   └── params.yaml                   ← all tunable parameters
├── launch/
│   └── fusion.launch.py              ← bag + node + RViz2
└── rviz/
    └── fusion.rviz                   ← pre-built RViz2 profile
```

---

## Dependencies

| Package | Source |
|---------|--------|
| `rclcpp`, `tf2_ros`, `tf2_eigen`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `message_filters` | ROS 2 Humble |
| `zed_msgs` | [ZED ROS 2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper) |
| `Eigen3` | `sudo apt install libeigen3-dev` |

Install `zed_msgs`:
```bash
# Pre-built (if available in your ROS index)
sudo apt install ros-humble-zed-msgs

# Or build from source alongside the wrapper
```

---

## Build

```bash
cd <your_ws>

# Make sure ZED ROS 2 wrapper is also in src/ (provides zed_msgs)
colcon build --packages-select lidar_camera_Object_Association \
             --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

---

## Run

```bash
# Full stack: bag playback + fusion node + RViz2
ros2 launch lidar_camera_Object_Association fusion.launch.py

# Override bag path
ros2 launch lidar_camera_Object_Association fusion.launch.py \
  bag_path:=/absolute/path/to/session2Real_0.db3

# Override params file
ros2 launch lidar_camera_Object_Association fusion.launch.py \
  params_file:=/path/to/custom_params.yaml

# Run node only (bag already playing externally)
ros2 run lidar_camera_Object_Association association_node \
  --ros-args --params-file install/lidar_camera_Object_Association/share/lidar_camera_Object_Association/config/params.yaml
```

---

## Configuration (`config/params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `lidar_topic` | `/rslidar_points` | LiDAR PointCloud2 input |
| `objects_topic` | `/zed/zed_node/obj_det/objects` | ZED detections |
| `lidar_frame` | `rslidar` | Fixed LiDAR frame |
| `camera_frame` | `zed_left_camera_frame` | ZED optical frame |
| `sync_slop` | `0.10` | Max timestamp difference for sync [s] |
| `sync_queue_size` | `10` | Per-topic buffer depth |
| `cropbox_margin` | `0.05` | OBB expansion margin [m] |
| `min_object_points` | `3` | Skip objects with fewer surviving points |
| `marker_lifetime_s` | `0.20` | RViz2 marker auto-delete time [s] |

---

## RViz2 Displays

| Display | Topic | Notes |
|---------|-------|-------|
| PointCloud2 (grey) | `/rslidar_points` | Raw scan, flat colour |
| PointCloud2 (colour) | `/fused/object_points` | Object-coloured, sphere style |
| MarkerArray | `/fused/object_markers` | OBB wireframes + ID labels |
| TF | – | Sensor coordinate frames |

---

## Troubleshooting

**No output on `/fused/object_points`**
- Verify TF tree: `ros2 run tf2_tools view_frames` – both frames must be connected.
- Increase `sync_slop` (try `0.15` or `0.20`) if bag timestamps are misaligned.
- Check `ros2 topic hz /rslidar_points` and `ros2 topic hz /zed/zed_node/obj_det/objects` to confirm both are publishing.

**Wrong frame IDs**
- Inspect the bag: `ros2 bag info session2Real_0.db3` and read the `frame_id` from a sample message.
- Update `lidar_frame` / `camera_frame` in `params.yaml` accordingly.

**Build error: `zed_msgs` not found**
- Build the ZED ROS 2 wrapper from source in the same workspace, or install the pre-built Debian package.
