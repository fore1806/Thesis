#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace lidar_camera_fusion_pcl
{

// ─────────────────────────────────────────────────────────────────────────────
// CloudFilterNode
//
// Standalone utility: subscribes to any PointCloud2, applies a passthrough
// range filter on Z (height) and optionally a voxel downsample, and publishes
// the result. Useful for pre-filtering noisy ZED points before merge.
// ─────────────────────────────────────────────────────────────────────────────
class CloudFilterNode : public rclcpp::Node
{
public:
    explicit CloudFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Subscription and Publishing Results
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;

    // Parameters
    double z_min_, z_max_, voxel_size_;
    bool   apply_voxel_;
};

}