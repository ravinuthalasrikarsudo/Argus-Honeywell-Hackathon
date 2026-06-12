#!/usr/bin/env bash
#
# ARGUS Day-1 :: build.sh
# Builds the ARGUS workspace with colcon (symlink-install) and reports the
# resulting warning count. Optional package names build only a subset.
#
# Usage:  scripts/build.sh [pkg ...]
#   scripts/build.sh                       # whole workspace
#   scripts/build.sh argus_sim argus_msgs  # selected packages only
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARGUS_WS="$(dirname "${SCRIPT_DIR}")"

# Only the base ROS env is needed to build. Do NOT source the overlay we are
# about to rebuild (it may be stale or half-written). ROS setup.bash is not
# nounset-safe, so relax -u just across it.
set +u
source /opt/ros/humble/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd "${ARGUS_WS}"

args=(--symlink-install --event-handlers console_direct+)
if [ "$#" -gt 0 ]; then
  args+=(--packages-select "$@")
fi

echo "[build] colcon build ${args[*]}"
colcon build "${args[@]}"

echo "[build] done. Source the overlay with:"
echo "          source ${ARGUS_WS}/install/setup.bash"
