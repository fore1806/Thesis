cmake_minimum_required(VERSION 3.8)
project(lidar_camera_fusion_pcl)

cmake_policy(SET CMP0074 NEW)

# ── Compile options ────────────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O2)

# ── Find dependencies ──────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)

# ── Shared dependency list (used by both targets) ─────────────────────────────
set(DEPS
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_eigen
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  message_filters
)

# ── Fusion node (composable component) ────────────────────────────────────────
add_library(cloud_merge_component SHARED
  src/cloud_merge_node.cpp
)
target_include_directories(cloud_merge_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(cloud_merge_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(cloud_merge_component ${DEPS})
rclcpp_components_register_node(
  cloud_merge_component
  PLUGIN "lidar_camera_fusion_pcl::CloudMergeNode"
  EXECUTABLE cloud_merge_node
)

# ── Passthrough filter utility node ───────────────────────────────────────────
add_executable(cloud_filter_node
  src/cloud_filter_node.cpp
)
target_include_directories(cloud_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(cloud_filter_node
  ${PCL_LIBRARIES}
)
ament_target_dependencies(cloud_filter_node ${DEPS})

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS cloud_merge_component cloud_merge_node cloud_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

# ── Ament export ──────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(cloud_merge_component)
ament_export_dependencies(${DEPS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
