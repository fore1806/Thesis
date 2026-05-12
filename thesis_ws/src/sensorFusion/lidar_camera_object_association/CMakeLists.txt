cmake_minimum_required(VERSION 3.14)
project(lidar_camera_object_association)

# ── C++17 required for std::filesystem and structured bindings ──────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ── Find dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(message_filters    REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(tf2_eigen          REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(zed_msgs           REQUIRED)
find_package(Eigen3             REQUIRED)

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(association_node
  src/association_node.cpp
)

target_include_directories(association_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(association_node
  rclcpp
  message_filters
  tf2_ros
  tf2_eigen
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  zed_msgs
)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS association_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
