#!/usr/bin/env bash
# ARGUS :: colcon_build.sh  —  build the workspace (run INSIDE the container).
#
#   docker/run.sh docker/colcon_build.sh     # from the host, one-shot
#   ./docker/colcon_build.sh                 # from a container shell
#
# Pins the source-built Ceres 2.1.0 (/usr/local) so VINS gets ceres::Manifold.
set -euo pipefail
WS=/home/vittal/argus
cd "$WS"

set +u
source /opt/ros/humble/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "[colcon_build] Ceres: $(ls -d /usr/local/lib/cmake/Ceres 2>/dev/null || echo MISSING)"
colcon build --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args -DCeres_DIR=/usr/local/lib/cmake/Ceres -DCMAKE_BUILD_TYPE=Release

# loop_fusion's DBoW vocabulary (support_files/) is not copied into the colcon install
# tree, so the node aborts (vector::reserve) loading a missing file. Link it where the
# node looks: share/loop_fusion/../support_files -> share/support_files.
ln -sfn "$WS/third_party/VINS-Fusion-ROS2/support_files" \
        "$WS/install/loop_fusion/share/support_files" 2>/dev/null || true

echo "[colcon_build] done. source $WS/install/setup.bash"
