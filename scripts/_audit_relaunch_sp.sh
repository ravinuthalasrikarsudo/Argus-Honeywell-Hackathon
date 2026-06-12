#!/usr/bin/env bash
# Audit helper: hot-restart the SuperPoint node so the overlay QoS fix (BEST_EFFORT)
# takes effect under the already-running rqt + bag. Safe self-match (this script's
# argv does not contain the node module pattern).
set -uo pipefail
WS=/home/vittal/argus
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1

pkill -9 -f "argus_superpoint.superpoint_node" 2>/dev/null || true
sleep 3

SP=$HOME/.venvs/argus-sp/bin/python
NVLIBS=$(find "$HOME/.venvs/argus-sp/lib" -maxdepth 3 -type d -path "*nvidia*lib" 2>/dev/null | tr '\n' ':')
export LD_LIBRARY_PATH="${NVLIBS}${LD_LIBRARY_PATH:-}"
nohup env PYTHONPATH="$WS/src/argus_superpoint:${PYTHONPATH:-}" "$SP" \
  -m argus_superpoint.superpoint_node --ros-args -p use_sim_time:=true \
  > "$WS/data/eval/_demo_sp.log" 2>&1 &
disown
echo "relaunched superpoint node (BEST_EFFORT overlay) pid $!"
