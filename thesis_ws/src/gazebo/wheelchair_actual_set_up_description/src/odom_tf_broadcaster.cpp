#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

/*
 * Reads /odom and re-broadcasts it as the odom → base_footprint TF.
 *
 * The Ignition DiffDrive plugin's <tf_topic> is unreliable in Fortress;
 * this node provides the TF link explicitly from the odometry message that
 * the bridge already delivers reliably.
 */
class OdomTFBroadcaster : public rclcpp::Node
{
public:
  OdomTFBroadcaster()
  : Node("odom_tf_broadcaster"),
    br_(std::make_shared<tf2_ros::TransformBroadcaster>(*this))
  {
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = msg->header.stamp;
        t.header.frame_id = msg->header.frame_id;   // "odom"
        t.child_frame_id  = msg->child_frame_id;    // "base_footprint"
        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;
        t.transform.rotation      = msg->pose.pose.orientation;
        br_->sendTransform(t);
      });
  }

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
