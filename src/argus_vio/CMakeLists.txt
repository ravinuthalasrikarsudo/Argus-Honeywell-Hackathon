cmake_minimum_required(VERSION 3.8)
project(argus_vio)

find_package(ament_cmake REQUIRED)

# Install VINS-Fusion configs (main + per-camera intrinsics) and launch files.
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
