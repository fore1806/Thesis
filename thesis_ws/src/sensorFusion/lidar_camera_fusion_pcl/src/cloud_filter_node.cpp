#include "lidar_camera_fusion_pcl/cloud_filter_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

namespace lidar_camera_fusion_pcl
{
// Starting the Node cloud filter, employed to filter out the not usable data
// e.g. too high/low data
CloudFilterNode::CloudFilterNode(const rclcpp::NodeOptions & options)
: Node("cloud_filter_node", options)
{
    // Default Parameters
    this->declare_parameter("input_topic",  "/zed/zed_node/point_cloud/cloud_registered");
    this->declare_parameter("output_topic", "/zed/cloud_filtered");
    this->declare_parameter("z_min",        -2.0);
    this->declare_parameter("z_max",         5.0);
    this->declare_parameter("apply_voxel",  true);
    this->declare_parameter("voxel_size",   0.03);

    const auto in_topic  = this->get_parameter("input_topic").as_string();
    const auto out_topic = this->get_parameter("output_topic").as_string();
    z_min_        = this->get_parameter("z_min").as_double();
    z_max_        = this->get_parameter("z_max").as_double();
    apply_voxel_  = this->get_parameter("apply_voxel").as_bool();
    voxel_size_   = this->get_parameter("voxel_size").as_double();

    // Subscription to the specified input_topic
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        in_topic, rclcpp::QoS(10),
        std::bind(&CloudFilterNode::cloudCallback, this, std::placeholders::_1));
    
    // Publishing to the specified output_topic
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(),
        "[CloudFilter] '%s' → '%s'  z=[%.1f, %.1f]  voxel=%.3f m",
        in_topic.c_str(), out_topic.c_str(), z_min_, z_max_, voxel_size_);
}

// Callback (Processing)
void CloudFilterNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // We keep the XYZ and RGB information
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Transforming the ROS message to PCL
    pcl::fromROSMsg(*msg, *cloud);

    // Passthrough on Z (removing Ceiling and floor)
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(static_cast<float>(z_min_), static_cast<float>(z_max_));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pass.filter(*filtered);

    // Reducing the number of data points by clustering them in small cubes
    if (apply_voxel_ && voxel_size_ > 0.0) {
      pcl::VoxelGrid<pcl::PointXYZRGB> vg;
      vg.setInputCloud(filtered);
      const float leaf = static_cast<float>(voxel_size_);
      vg.setLeafSize(leaf, leaf, leaf);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds(new pcl::PointCloud<pcl::PointXYZRGB>);
      vg.filter(*ds);
      filtered = ds;
    }

    // Publishing the Filtered Result
    sensor_msgs::msg::PointCloud2 out;
    // Transforming the PCL message to ROS
    pcl::toROSMsg(*filtered, out);
    // Keeping the Original message Data
    out.header = msg->header;
    pub_->publish(out);
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_camera_fusion_pcl::CloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}