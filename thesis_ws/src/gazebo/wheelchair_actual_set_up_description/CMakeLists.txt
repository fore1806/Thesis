cmake_minimum_required(VERSION 3.8)
project(wheelchair_actual_set_up_description)

find_package(ament_cmake REQUIRED)

# Install every folder that actually exists in the package
install(
  DIRECTORY
    urdf
    meshes
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()