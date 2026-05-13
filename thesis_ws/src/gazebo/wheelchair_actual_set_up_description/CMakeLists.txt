cmake_minimum_required(VERSION 3.8)
project(wheelchair_actual_set_up_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(topic_relay src/topic_relay.cpp)
ament_target_dependencies(topic_relay rclcpp)

add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster rclcpp nav_msgs geometry_msgs tf2_ros)

install(TARGETS topic_relay odom_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    urdf
    meshes
    launch
    config
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()