#!/usr/bin/env bash
# Quick diagnostic: is VINS publishing keyframes to /keyframe_pose? (loop_fusion
# starves with 0 loop candidates if not.) Launch loop stack, replay a slice, sample.
set -uo pipefail
WS=/home/vittal/argus
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

ros2 launch argus_vio argus_vio_loop.launch.py > /tmp/_diag_stack.log 2>&1 &
sleep 12
ros2 bag play "$WS/data/bags/scenario_C_shuttle" --clock --rate 0.6 > /tmp/_diag_play.log 2>&1 &
PLAY=$!
sleep 22
echo "=== /keyframe_pose rate (10s sample) ==="
timeout 12 ros2 topic hz /keyframe_pose 2>&1 | tail -4 || echo "  (no keyframes / topic silent)"
echo "=== /keyframe_point rate (6s) ==="
timeout 8 ros2 topic hz /keyframe_point 2>&1 | tail -3 || true
echo "=== relevant topics present ==="
ros2 topic list 2>/dev/null | grep -E "keyframe|loop|odom_loop" || true
echo "=== teardown ==="
kill -INT "$PLAY" 2>/dev/null || true
sleep 2
pkill -9 -f vins_node 2>/dev/null || true
pkill -9 -f loop_fusion_node 2>/dev/null || true
echo "diag done"
