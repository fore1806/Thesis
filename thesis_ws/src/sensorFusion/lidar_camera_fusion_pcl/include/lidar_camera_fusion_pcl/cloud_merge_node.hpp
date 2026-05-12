#pragma once

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_camera_fusion_pcl
{

// ─────────────────────────────────────────────────────────────────────────────
// Point type used throughout this package.
// XYZ + intensity (from LiDAR) + RGB (from ZED, packed as float).
// ─────────────────────────────────────────────────────────────────────────────
using PointT = pcl::PointXYZRGB;
using Cloud  = pcl::PointCloud<PointT>;

// ─────────────────────────────────────────────────────────────────────────────
// Sync policy: approximate-time with two PointCloud2 inputs
// ─────────────────────────────────────────────────────────────────────────────
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::PointCloud2,   // /rslidar_points
  sensor_msgs::msg::PointCloud2>;  // /zed/zed_node/point_cloud/cloud_registered

using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

// ─────────────────────────────────────────────────────────────────────────────
// CloudMergeNode
//
// Subscribes to both point clouds, transforms them to `target_frame_`, and
// publishes their concatenation as a single PointCloud2 on /fused_cloud.
// ─────────────────────────────────────────────────────────────────────────────
class CloudMergeNode : public rclcpp::Node
{
public:
  explicit CloudMergeNode(const rclcpp::NodeOptions & options);

private:
  // ── Callback ────────────────────────────────────────────────────────────────
  void fusionCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & zed_msg);

  // ── Helpers ─────────────────────────────────────────────────────────────────
  bool transformCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in,
    sensor_msgs::msg::PointCloud2 & out);

  Cloud::Ptr fromROS(const sensor_msgs::msg::PointCloud2 & msg) const;
  void       toROS(
    const Cloud & cloud,
    sensor_msgs::msg::PointCloud2 & msg,
    const std_msgs::msg::Header & header) const;

  // ── Parameters ──────────────────────────────────────────────────────────────
  std::string target_frame_;   // common frame (e.g. "base_link")
  double      tf_timeout_s_;   // TF lookup timeout in seconds
  int         queue_size_;     // sync queue size
  double      slop_s_;         // ApproximateTime slop in seconds
  double      voxel_size_;     // voxel leaf size for optional downsampling (0 = off)
  bool        apply_voxel_;    // enable voxel filter on fused output

  // ── TF ──────────────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── Subscribers / Sync ──────────────────────────────────────────────────────
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> zed_sub_;
  std::shared_ptr<Synchronizer>                               sync_;

  // ── Publisher ───────────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_pub_;

  // ── Diagnostics ─────────────────────────────────────────────────────────────
  uint64_t frame_count_{0};
};

} 