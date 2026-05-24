#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class WorldMarkersPublisher : public rclcpp::Node
{
public:
  WorldMarkersPublisher() : Node("world_markers_publisher")
  {
    // Transient-local (latched): RViz receives markers even when it starts after this node
    auto qos = rclcpp::QoS(1).transient_local();
    pub_ = create_publisher<MarkerArray>("/world_markers", qos);
    pub_->publish(build_markers());
  }

private:
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_;

  static Marker make_box(int id, const std::string & frame,
                         double x, double y, double z, double yaw,
                         double sx, double sy, double sz,
                         float r, float g, float b)
  {
    Marker m;
    m.header.frame_id = frame;
    m.ns = "world";
    m.id = id;
    m.type = Marker::CUBE;
    m.action = Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.pose.orientation.z = std::sin(yaw / 2.0);
    m.pose.orientation.w = std::cos(yaw / 2.0);
    m.scale.x = sx;
    m.scale.y = sy;
    m.scale.z = sz;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
    return m;
  }

  static Marker make_cylinder(int id, const std::string & frame,
                               double x, double y, double z,
                               double radius, double height,
                               float r, float g, float b)
  {
    Marker m;
    m.header.frame_id = frame;
    m.ns = "world";
    m.id = id;
    m.type = Marker::CYLINDER;
    m.action = Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.pose.orientation.w = 1.0;
    m.scale.x = radius * 2.0;
    m.scale.y = radius * 2.0;
    m.scale.z = height;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
    return m;
  }

  MarkerArray build_markers()
  {
    MarkerArray arr;
    int id = 0;

    // Walls — match wheelchair_world.sdf poses exactly
    arr.markers.push_back(make_box(id++, "odom",  0.0,  5.0, 0.5, 0.0,    10.0, 0.2, 1.0, 0.75f, 0.65f, 0.5f));
    arr.markers.push_back(make_box(id++, "odom",  0.0, -5.0, 0.5, 0.0,    10.0, 0.2, 1.0, 0.75f, 0.65f, 0.5f));
    arr.markers.push_back(make_box(id++, "odom",  5.0,  0.0, 0.5, M_PI_2, 10.0, 0.2, 1.0, 0.75f, 0.65f, 0.5f));
    arr.markers.push_back(make_box(id++, "odom", -5.0,  0.0, 0.5, M_PI_2, 10.0, 0.2, 1.0, 0.75f, 0.65f, 0.5f));

    // Pillars (cylinders)
    arr.markers.push_back(make_cylinder(id++, "odom", 3.0, 0.0, 0.5, 0.15, 1.0, 0.2f, 0.6f, 0.2f));
    arr.markers.push_back(make_cylinder(id++, "odom", 1.5, 2.0, 0.5, 0.15, 1.0, 0.2f, 0.6f, 0.2f));

    // Crates
    arr.markers.push_back(make_box(id++, "odom",  2.0, -1.5, 0.3, 0.4, 0.6, 0.6, 0.6, 0.6f, 0.4f, 0.1f));
    arr.markers.push_back(make_box(id++, "odom", -2.5,  1.0, 0.3, 0.0, 0.6, 0.6, 0.6, 0.6f, 0.4f, 0.1f));

    return arr;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldMarkersPublisher>());
  rclcpp::shutdown();
  return 0;
}
