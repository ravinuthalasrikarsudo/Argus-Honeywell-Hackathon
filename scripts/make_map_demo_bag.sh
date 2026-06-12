#!/usr/bin/env bash
# Bake a CLEAN VINS map bag (mt0@0.15) for the RViz map demo. Records trajectory +
# point-cloud map so demo.sh map can replay it smoothly (no live VINS on stage = no
# divergence risk). One-off; re-run only if the sensor bag changes.
set -uo pipefail
WS=/home/vittal/argus
SRC="${1:-$WS/data/bags/baseline_live_day6}"
OUT="${2:-$WS/data/bags/map_demo}"
RATE="${RATE:-0.15}"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

bash "$WS/scripts/_killsim.sh" >/dev/null 2>&1; sleep 2
rm -rf "$OUT"

echo "[bake] launch VINS (mt0)..."
ros2 launch argus_vio argus_vio.launch.py > /tmp/mapbake_vins.log 2>&1 &
VP=$!
sleep 6

echo "[bake] record map topics..."
ros2 bag record -s sqlite3 -o "$OUT" \
  /argus/vio/path /argus/vio/odom /argus/vio/odom_optimized \
  /argus/vio/point_cloud /argus/vio/margin_cloud /argus/vio/key_poses \
  /argus/vio/loop_closures /argus/ground_truth/pose /clock > /tmp/mapbake_rec.log 2>&1 &
RP=$!
sleep 2

echo "[bake] replay $SRC @ $RATE (builds the map)..."
ros2 bag play "$SRC" --clock --rate "$RATE"
sleep 5

echo "[bake] teardown..."
kill -INT "$RP" 2>/dev/null || true; sleep 3
kill -INT "$VP" 2>/dev/null || true; sleep 2
pkill -9 -f vins_node 2>/dev/null || true
bash "$WS/scripts/_killsim.sh" >/dev/null 2>&1

echo "[bake] result:"
ros2 bag info "$OUT" 2>/dev/null | grep -iE "Duration|point_cloud|margin|path|odom|Count" | head -20
echo "[bake] DONE -> $OUT"
