// =============================================================================
//  association_node.cpp
//  LiDAR–Camera Object Association  –  ZED 2i + RoboSense RS-Helios-16P
//
//  Pipeline
//  --------
//  1.  Subscribe to /rslidar_points and /zed/.../objects via
//      message_filters::ApproximateTimeSynchronizer.
//  2.  For every ZED detection, transform the 8 OBB corner keypoints from
//      zed_left_camera_frame → rslidar using tf2.
//  3.  CropBox-filter the raw LiDAR cloud (pure Eigen, no PCL dependency).
//  4.  Publish /fused/object_points  (PointCloud2 with per-object RGB colour).
//  5.  Publish /fused/object_markers (MarkerArray with OBB wireframes).
// =============================================================================

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

// ── ROS 2 core ──────────────────────────────────────────────────────────────
#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

// ── TF2 ─────────────────────────────────────────────────────────────────────
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"

// ── Messages ────────────────────────────────────────────────────────────────
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "zed_msgs/msg/objects_stamped.hpp"

// ── Eigen ────────────────────────────────────────────────────────────────────
#include <Eigen/Core>
#include <Eigen/Geometry>

// ── Package utilities ────────────────────────────────────────────────────────
#include "lidar_camera_object_association/association_utils.hpp"

namespace lca = lidar_camera_association;

using sensor_msgs::msg::PointCloud2;
using zed_msgs::msg::ObjectsStamped;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  PointCloud2, ObjectsStamped>;

// =============================================================================
class AssociationNode : public rclcpp::Node
// =============================================================================
{
public:
  explicit AssociationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
  : Node("association_node", options)
  {
    // ── Declare & read parameters ──────────────────────────────────────────
    lidar_topic_    = declare_parameter<std::string>("lidar_topic",       "/rslidar_points");
    objects_topic_  = declare_parameter<std::string>("objects_topic",     "/zed/zed_node/obj_det/objects");
    lidar_frame_    = declare_parameter<std::string>("lidar_frame",       "rslidar");
    camera_frame_   = declare_parameter<std::string>("camera_frame",      "zed_left_camera_frame");
    sync_slop_      = declare_parameter<double>     ("sync_slop",         0.10);
    sync_queue_     = declare_parameter<int>        ("sync_queue_size",   10);
    cropbox_margin_ = declare_parameter<float>      ("cropbox_margin",    0.05f);
    min_pts_        = declare_parameter<int>        ("min_object_points", 3);
    marker_lifetime_= declare_parameter<double>     ("marker_lifetime_s", 0.20);

    // ── TF2 ───────────────────────────────────────────────────────────────
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    // ── QoS profiles ──────────────────────────────────────────────────────
    rclcpp::SensorDataQoS sensor_qos;
    rclcpp::QoS reliable_qos{rclcpp::KeepLast(5)};

    // ── Subscribers (message_filters) ─────────────────────────────────────
    lidar_sub_.subscribe(this, lidar_topic_,   sensor_qos.get_rmw_qos_profile());
    objects_sub_.subscribe(this, objects_topic_, reliable_qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(static_cast<unsigned>(sync_queue_)),
      lidar_sub_, objects_sub_);
    sync_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(sync_slop_));
    sync_->registerCallback(&AssociationNode::sync_callback, this);

    // ── Publishers ────────────────────────────────────────────────────────
    fused_pub_  = create_publisher<PointCloud2>  ("/fused/object_points",  reliable_qos);
    marker_pub_ = create_publisher<MarkerArray>  ("/fused/object_markers", reliable_qos);

    RCLCPP_INFO(get_logger(),
      "AssociationNode ready  |  lidar_frame='%s'  camera_frame='%s'  slop=%.3fs",
      lidar_frame_.c_str(), camera_frame_.c_str(), sync_slop_);
  }

private:
  // ── Parameters ────────────────────────────────────────────────────────────
  std::string lidar_topic_, objects_topic_;
  std::string lidar_frame_, camera_frame_;
  double sync_slop_;
  int    sync_queue_;
  float  cropbox_margin_;
  int    min_pts_;
  double marker_lifetime_;

  // ── TF2 ───────────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── message_filters ───────────────────────────────────────────────────────
  message_filters::Subscriber<PointCloud2>      lidar_sub_;
  message_filters::Subscriber<ObjectsStamped>   objects_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<PointCloud2>::SharedPtr  fused_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr  marker_pub_;

  // =========================================================================
  // Synchronised callback
  // =========================================================================
  void sync_callback(
    const PointCloud2::ConstSharedPtr   & cloud_msg,
    const ObjectsStamped::ConstSharedPtr & objs_msg)
  {
    if (objs_msg->objects.empty()) return;

    // ── Lookup transform: camera_frame → lidar_frame ─────────────────────
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        lidar_frame_, camera_frame_,
        tf2::TimePointZero,                           // latest available
        tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed: %s", ex.what());
      return;
    }

    // Convert TransformStamped → Eigen Isometry3d
    const Eigen::Isometry3d T = tf2::transformToEigen(tf_stamped);
    const Eigen::Matrix4f   Tf = T.matrix().cast<float>();

    // ── Parse raw LiDAR cloud into Eigen matrix (N×3) ─────────────────────
    Eigen::MatrixX3f pts = cloud_to_eigen(*cloud_msg);
    if (pts.rows() == 0) return;

    // ── Per-object processing ─────────────────────────────────────────────
    // Accumulate coloured points: struct { float x,y,z,rgb }
    std::vector<std::array<float, 4>> coloured_pts;
    coloured_pts.reserve(static_cast<std::size_t>(pts.rows()));

    MarkerArray markers;

    // DELETEALL marker to clear stale markers
    Marker del;
    del.header  = cloud_msg->header;
    del.action  = Marker::DELETEALL;
    markers.markers.push_back(del);

    int marker_id = 0;

    for (const auto & obj : objs_msg->objects) {
      const auto & bb = obj.bounding_box_3d;
      if (bb.corners.size() < 8) continue;

      // ── Transform 8 OBB corners into LiDAR frame ─────────────────────
      Eigen::Matrix<float, 8, 3> corners_lid;
      for (int k = 0; k < 8; ++k) {
        Eigen::Vector4f c_cam(
          static_cast<float>(bb.corners[k].kp[0]),
          static_cast<float>(bb.corners[k].kp[1]),
          static_cast<float>(bb.corners[k].kp[2]),
          1.0f);
        Eigen::Vector4f c_lid = Tf * c_cam;
        corners_lid.row(k) = c_lid.head<3>();
      }

      // ── CropBox filter ────────────────────────────────────────────────
      auto mask = lca::cropbox_filter(pts, corners_lid, cropbox_margin_);

      int inside_count = 0;
      for (bool b : mask) if (b) ++inside_count;

      if (inside_count < min_pts_) continue;

      // ── Colour ────────────────────────────────────────────────────────
      lca::Colour col = lca::colour_for_id(obj.label_id);
      float rgb_f = lca::pack_rgb(col.r, col.g, col.b);

      for (int i = 0; i < pts.rows(); ++i) {
        if (!mask[static_cast<std::size_t>(i)]) continue;
        coloured_pts.push_back({pts(i, 0), pts(i, 1), pts(i, 2), rgb_f});
      }

      // ── OBB wireframe marker ──────────────────────────────────────────
      markers.markers.push_back(
        make_box_marker(cloud_msg->header, marker_id++, corners_lid, col, obj.label_id));

      // ── Text label ────────────────────────────────────────────────────
      markers.markers.push_back(
        make_label_marker(cloud_msg->header, marker_id++, corners_lid, obj.label_id));

      RCLCPP_DEBUG(get_logger(),
        "Obj %d: %d pts inside bbox", obj.label_id, inside_count);
    }

    // ── Publish fused cloud ───────────────────────────────────────────────
    if (!coloured_pts.empty()) {
      fused_pub_->publish(build_pointcloud2(cloud_msg->header, coloured_pts));
    }

    // ── Publish markers ───────────────────────────────────────────────────
    marker_pub_->publish(markers);
  }

  // =========================================================================
  // Parse PointCloud2 → Eigen MatrixX3f  (skip NaNs)
  // =========================================================================
  static Eigen::MatrixX3f cloud_to_eigen(const PointCloud2 & msg)
  {
    std::vector<std::array<float, 3>> tmp;
    tmp.reserve(msg.width * msg.height);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z))
        continue;
      tmp.push_back({*it_x, *it_y, *it_z});
    }

    Eigen::MatrixX3f pts(static_cast<int>(tmp.size()), 3);
    for (int i = 0; i < static_cast<int>(tmp.size()); ++i) {
      pts(i, 0) = tmp[static_cast<std::size_t>(i)][0];
      pts(i, 1) = tmp[static_cast<std::size_t>(i)][1];
      pts(i, 2) = tmp[static_cast<std::size_t>(i)][2];
    }
    return pts;
  }

  // =========================================================================
  // Build PointCloud2 from x,y,z,rgb tuples
  // =========================================================================
  static PointCloud2 build_pointcloud2(
    const std_msgs::msg::Header & header,
    const std::vector<std::array<float, 4>> & pts)
  {
    PointCloud2 msg;
    msg.header     = header;
    msg.height     = 1;
    msg.width      = static_cast<uint32_t>(pts.size());
    msg.is_dense   = true;
    msg.is_bigendian = false;
    msg.point_step = 16;
    msg.row_step   = msg.point_step * msg.width;

    auto add_field = [&](const std::string & name, uint32_t offset, uint8_t dtype) {
      sensor_msgs::msg::PointField f;
      f.name     = name;
      f.offset   = offset;
      f.datatype = dtype;
      f.count    = 1;
      msg.fields.push_back(f);
    };
    add_field("x",   0,  sensor_msgs::msg::PointField::FLOAT32);
    add_field("y",   4,  sensor_msgs::msg::PointField::FLOAT32);
    add_field("z",   8,  sensor_msgs::msg::PointField::FLOAT32);
    add_field("rgb", 12, sensor_msgs::msg::PointField::FLOAT32);

    msg.data.resize(static_cast<std::size_t>(msg.row_step));
    for (std::size_t i = 0; i < pts.size(); ++i) {
      std::memcpy(msg.data.data() + i * 16, pts[i].data(), 16);
    }
    return msg;
  }

  // =========================================================================
  // Build LINE_LIST OBB wireframe marker
  // =========================================================================
  Marker make_box_marker(
    const std_msgs::msg::Header & header,
    int id,
    const Eigen::Matrix<float, 8, 3> & corners,
    const lca::Colour & col,
    int obj_id) const
  {
    // 12 edges of the box
    static constexpr int kEdges[12][2] = {
      {0,1},{1,2},{2,3},{3,0},   // bottom
      {4,5},{5,6},{6,7},{7,4},   // top
      {0,4},{1,5},{2,6},{3,7},   // verticals
    };

    Marker mk;
    mk.header  = header;
    mk.ns      = "object_boxes";
    mk.id      = id;
    mk.type    = Marker::LINE_LIST;
    mk.action  = Marker::ADD;
    mk.scale.x = 0.04;
    mk.color.r = col.r;  mk.color.g = col.g;
    mk.color.b = col.b;  mk.color.a = 0.9f;
    mk.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);

    for (const auto & e : kEdges) {
      geometry_msgs::msg::Point pa, pb;
      pa.x = static_cast<double>(corners(e[0], 0));
      pa.y = static_cast<double>(corners(e[0], 1));
      pa.z = static_cast<double>(corners(e[0], 2));
      pb.x = static_cast<double>(corners(e[1], 0));
      pb.y = static_cast<double>(corners(e[1], 1));
      pb.z = static_cast<double>(corners(e[1], 2));
      mk.points.push_back(pa);
      mk.points.push_back(pb);
    }
    // Unused but required
    (void)obj_id;
    return mk;
  }

  // =========================================================================
  // Build TEXT_VIEW_FACING label marker
  // =========================================================================
  Marker make_label_marker(
    const std_msgs::msg::Header & header,
    int id,
    const Eigen::Matrix<float, 8, 3> & corners,
    int obj_id) const
  {
    Eigen::Vector3f centre = corners.colwise().mean();

    Marker mk;
    mk.header  = header;
    mk.ns      = "object_labels";
    mk.id      = id;
    mk.type    = Marker::TEXT_VIEW_FACING;
    mk.action  = Marker::ADD;
    mk.scale.z = 0.25;
    mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0f;
    mk.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
    mk.pose.position.x = static_cast<double>(centre.x());
    mk.pose.position.y = static_cast<double>(centre.y());
    mk.pose.position.z = static_cast<double>(centre.z()) + 0.5;
    mk.pose.orientation.w = 1.0;
    mk.text = "ID:" + std::to_string(obj_id);
    return mk;
  }
};

// =============================================================================
// main
// =============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AssociationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
