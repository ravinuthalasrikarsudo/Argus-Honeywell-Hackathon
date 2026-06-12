#!/usr/bin/env bash
# SuperPoint live end-to-end test: replay corridor images, run the SuperPoint node,
# capture overlay PNGs + the node's measured Hz. Confirms criterion #3 end-to-end
# (node plumbing + keypoints found in the low-texture corridor + RViz overlay).
set -uo pipefail
WS=/home/vittal/argus
SP=$HOME/.venvs/argus-sp/bin/python
BAG="${1:-$WS/data/bags/scenario_A_fwd}"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
NVLIBS=$(find "$HOME/.venvs/argus-sp/lib" -maxdepth 3 -type d -path "*nvidia*lib" 2>/dev/null | tr '\n' ':')
export LD_LIBRARY_PATH="${NVLIBS}${LD_LIBRARY_PATH:-}"
set -u

echo "[splive] starting SuperPoint node..."
PYTHONPATH="$WS/src/argus_superpoint:${PYTHONPATH:-}" "$SP" -m argus_superpoint.superpoint_node \
  --ros-args -p use_sim_time:=true > "$WS/data/eval/_sp_node.log" 2>&1 &
sleep 8
echo "[splive] starting overlay saver..."
PYTHONPATH="$WS/src/argus_superpoint:${PYTHONPATH:-}" "$SP" "$WS/scripts/_sp_save_overlay.py" \
  "$WS/data/eval/superpoint" 8 > "$WS/data/eval/_sp_saver.log" 2>&1 &
sleep 3
echo "[splive] replaying cam0 from $BAG ..."
ros2 bag play "$BAG" --clock --rate 2.0 \
  --topics /argus/cam0/image_raw /argus/cam0/camera_info /clock > "$WS/data/eval/_sp_play.log" 2>&1 &
PLAY=$!
sleep 45
echo "[splive] teardown..."
kill -INT "$PLAY" 2>/dev/null || true
sleep 2
pkill -9 -f "argus_superpoint.superpoint_node" 2>/dev/null || true
pkill -9 -f "_sp_save_overlay" 2>/dev/null || true
echo "=== SuperPoint node Hz report ==="
grep -E "rate=|provider|loaded|kpts" "$WS/data/eval/_sp_node.log" | tail -8
echo "=== overlay PNGs saved ==="
ls -la "$WS/data/eval/superpoint"/*.png 2>/dev/null | tail -8 || echo "  (none)"
