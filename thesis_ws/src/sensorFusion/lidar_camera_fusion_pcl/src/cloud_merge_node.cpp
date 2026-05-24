#include "lidar_camera_fusion_pcl/cloud_merge_node.hpp"

#include <chrono>

#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/header.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace lidar_camera_fusion_pcl
{

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
CloudMergeNode::CloudMergeNode(const rclcpp::NodeOptions & options)
: Node("cloud_merge_node", options)
{
    // Parameters Declaration
    this->declare_parameter("target_frame",    "base_link"); // To be updated to base_link
    this->declare_parameter("tf_timeout_s",    0.1);
    this->declare_parameter("queue_size",      10);
    this->declare_parameter("slop_s",          0.05);   // Maximum time shift between sensors data
    this->declare_parameter("voxel_size",      0.05);   // metres; 0 = disabled
    this->declare_parameter("apply_voxel",     true);
    this->declare_parameter("lidar_topic",     "/rslidar_points");
    this->declare_parameter("zed_cloud_topic", "/zed/zed_node/point_cloud/cloud_registered");
    this->declare_parameter("fused_topic",     "/fused_cloud");

    // Getting parameters
    target_frame_  = this->get_parameter("target_frame").as_string();
    tf_timeout_s_  = this->get_parameter("tf_timeout_s").as_double();
    queue_size_    = this->get_parameter("queue_size").as_int();
    slop_s_        = this->get_parameter("slop_s").as_double();
    voxel_size_    = this->get_parameter("voxel_size").as_double();
    apply_voxel_   = this->get_parameter("apply_voxel").as_bool();

    // Topics
    const auto lidar_topic     = this->get_parameter("lidar_topic").as_string();
    const auto zed_cloud_topic = this->get_parameter("zed_cloud_topic").as_string();
    const auto fused_topic     = this->get_parameter("fused_topic").as_string();

    RCLCPP_INFO(get_logger(),
        "[CloudMerge] target_frame='%s'  slop=%.3f s  voxel=%.3f m  voxel_enabled=%s",
        target_frame_.c_str(), slop_s_, voxel_size_, apply_voxel_ ? "true" : "false");

    // ── TF2 ───────────────────────────────────────────────────────────────────
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ── Subscribers ───────────────────────────────────────────────────────────
    // Using rclcpp default QoS (reliable, depth 10).
    // LiDAR bags are typically BEST_EFFORT — adjust if your bag uses that.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_size_));  
    lidar_sub_.subscribe(this, lidar_topic, qos.get_rmw_qos_profile());
    zed_sub_.subscribe(this, zed_cloud_topic, qos.get_rmw_qos_profile());

    // ── ApproximateTime synchroniser ──────────────────────────────────────────
    sync_ = std::make_shared<Synchronizer>(
        SyncPolicy(queue_size_), lidar_sub_, zed_sub_); 
      
    // slop in nanoseconds for rclcpp::Duration
    sync_->setMaxIntervalDuration(
        rclcpp::Duration::from_seconds(slop_s_));
    
    // Callback Definition
    sync_->registerCallback(
        std::bind(&CloudMergeNode::fusionCallback, this,
        std::placeholders::_1, std::placeholders::_2)); 
    
    // Publisher Configuration
    fused_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        fused_topic, rclcpp::QoS(10));
       
    RCLCPP_INFO(get_logger(),
      "[CloudMerge] Listening on '%s' and '%s'. Publishing on '%s'.",
      lidar_topic.c_str(), zed_cloud_topic.c_str(), fused_topic.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// fusionCallback
// ─────────────────────────────────────────────────────────────────────────────
void CloudMergeNode::fusionCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & zed_msg)
{
    ++frame_count_;

    // Transforming the two clouds to the Base Link
    sensor_msgs::msg::PointCloud2 lidar_tf_points, zed_tf_points;

    if (!transformCloud(lidar_msg, lidar_tf_points)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[CloudMerge] Frame %lu: TF failed for lidar cloud — skipping.", frame_count_);
        return;
    }

    if (!transformCloud(zed_msg, zed_tf_points)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[CloudMerge] Frame %lu: TF failed for ZED cloud — skipping.", frame_count_);
        return;
    }

    // Converting to PCL
    // The ZED cloud is XYZRGB natively.
    // The LiDAR cloud (XYZI) is cast to XYZRGB with white colour so the
    // merged cloud has a uniform type. You can later use intensity for colouring.
    Cloud::Ptr lidar_pcl = fromROS(lidar_tf_points);
    Cloud::Ptr zed_pcl   = fromROS(zed_tf_points);

    // Concatenation
    Cloud::Ptr merged(new Cloud);
    *merged = *lidar_pcl + *zed_pcl;

    // Voxel Downsampling to cast the data in small cubes and reduce the amount of data transferred
    if (apply_voxel_ && voxel_size_ > 0.0) {
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(merged);
      vg.setLeafSize(
        static_cast<float>(voxel_size_),
        static_cast<float>(voxel_size_),
        static_cast<float>(voxel_size_));
      Cloud::Ptr filtered(new Cloud);
      vg.filter(*filtered);
      merged = filtered;
    }

    // Publishing Phase
    sensor_msgs::msg::PointCloud2 out_msg;
    std_msgs::msg::Header header;
    // Using LiDAR stamp as reference
    header.stamp    = lidar_msg->header.stamp;
    header.frame_id = target_frame_;
    // Transforming the PCL message to ROS
    toROS(*merged, out_msg, header);
    fused_pub_->publish(out_msg);

    if (frame_count_ % 50 == 0) {
        RCLCPP_INFO(get_logger(),
            "[CloudMerge] Frame %lu: lidar=%zu  zed=%zu  fused=%zu pts",
            frame_count_,
            lidar_pcl->size(),
            zed_pcl->size(),
            merged->size());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// transformCloud — look up the TF and apply it to the ROS message in place
// ─────────────────────────────────────────────────────────────────────────────
bool CloudMergeNode::transformCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in,
    sensor_msgs::msg::PointCloud2 & out)
{
    // Fast path: already in target frame
    if (in->header.frame_id == target_frame_) {
      out = *in;
      return true;
    }

    try {
        auto timeout = tf2::durationFromSec(tf_timeout_s_);
        tf_buffer_->transform(*in, out, target_frame_, timeout);
        return true;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(),
            "[CloudMerge] TF lookup '%s' -> '%s': %s",
            in->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
        return false;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// fromROS — sensor_msgs::PointCloud2 -> pcl::PointCloud<PointXYZRGB>
// LiDAR points without RGB get white colour (255,255,255).
// ─────────────────────────────────────────────────────────────────────────────
Cloud::Ptr CloudMergeNode::fromROS(const sensor_msgs::msg::PointCloud2 & msg) const
{
    // Try to convert directly (works if the cloud already has rgb field)
    Cloud::Ptr cloud(new Cloud);
    try {
        pcl::fromROSMsg(msg, *cloud);
    } catch (const std::exception &) {
        // Fallback: interpret as XYZ only and assign white colour
        pcl::PointCloud<pcl::PointXYZ> xyz;
        pcl::fromROSMsg(msg, xyz);
        cloud->reserve(xyz.size());
        for (const auto & p : xyz) {
            PointT pt;
            pt.x = p.x; pt.y = p.y; pt.z = p.z;
            pt.r = 200; pt.g = 200; pt.b = 200;  // neutral gray for LiDAR-only points
            cloud->push_back(pt);
        }
        cloud->width    = static_cast<uint32_t>(cloud->size());
        cloud->height   = 1;
        cloud->is_dense = false;
    }
    return cloud;
}

// ─────────────────────────────────────────────────────────────────────────────
// toROS — pcl::PointCloud<PointXYZRGB> -> sensor_msgs::PointCloud2
// ─────────────────────────────────────────────────────────────────────────────
void CloudMergeNode::toROS(
    const Cloud & cloud,
    sensor_msgs::msg::PointCloud2 & msg,
    const std_msgs::msg::Header & header) const
{
    pcl::toROSMsg(cloud, msg);
    msg.header = header;
}

}  // namespace lidar_camera_fusion_b

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_camera_fusion_pcl::CloudMergeNode)