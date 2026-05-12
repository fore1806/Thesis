cmake_minimum_required(VERSION 3.8)
project(wheelchair_actual_set_up_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(topic_relay src/topic_relay.cpp)
ament_target_dependencies(topic_relay rclcpp)

install(TARGETS topic_relay
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    urdf
    meshes
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()