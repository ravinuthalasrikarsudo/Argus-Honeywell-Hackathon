cmake_minimum_required(VERSION 3.10)
project(argus_sim)

find_package(ament_cmake REQUIRED)

# World + model assets, discoverable via GZ_SIM_RESOURCE_PATH env hook.
install(
  DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Runtime helper: Zone-B light flicker driver (gz transport).
install(
  PROGRAMS scripts/flicker_light.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Prepend share/argus_sim/{worlds,models} onto GZ_SIM_RESOURCE_PATH so the world
# resolves by filename (`gz sim warehouse_corridor.sdf`) and the drone resolves by
# URI (`model://argus_drone`). Use a colcon .dsv hook: colcon substitutes the
# correct per-package install prefix (a .sh hook using $AMENT_CURRENT_PREFIX picks
# up a stale prefix during colcon dsv sourcing).
ament_environment_hooks("hooks/${PROJECT_NAME}.dsv.in")

ament_package()
