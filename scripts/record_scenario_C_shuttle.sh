#!/usr/bin/env bash
# ARGUS :: record_scenario_C_shuttle.sh
#
# Records a multi-leg SENSOR bag flown as a NO-YAW forward/back shuttle
# (fly_shuttle.py). Replaces the U-turn recorder, whose in-place yaw turns
# diverged VINS. The shuttle is always translating along the optical axis
# (well-conditioned) and re-observes each place from the same viewpoint on the
# return legs (DBoW loop-closure cue). This is BOTH the long-path drift gate and
# the loop-closure validation bag.
#
# Usage:  bash scripts/record_scenario_C_shuttle.sh [BAG_DIR]
# Env: LAPS(3) LEG_M(16) SPEED(0.5) SETTLE_S(12)
set -uo pipefail

WS=/home/vittal/argus
BAG="${1:-$WS/data/bags/scenario_C_shuttle}"
LAPS="${LAPS:-3}"
LEG_M="${LEG_M:-16}"
SPEED="${SPEED:-0.5}"
SETTLE_S="${SETTLE_S:-12}"
LOG="$WS/data/bags/_record_shuttle.log"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

mkdir -p "$WS/data/bags"
rm -rf "$BAG"

echo "[recS] pre-launch cleanup of stray sim procs..."
pkill -9 -f "gz sim -s" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "camera_info_patch" 2>/dev/null || true
sleep 2

echo "[recS] launching sim (headless)..."
ros2 launch argus_bringup argus_sim.launch.py headless:=true >"$LOG" 2>&1 &
LAUNCH_PID=$!
echo "[recS] settling ${SETTLE_S}s for gz + bridge + drone spawn..."
sleep "$SETTLE_S"

if ! ros2 topic list 2>/dev/null | grep -q "/argus/ground_truth/pose"; then
  echo "[recS] ERROR: /argus topics not up after settle; see $LOG" >&2
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 4; pkill -9 -f "gz-sim|ruby|parameter_bridge|camera_info_patch" 2>/dev/null || true
  exit 1
fi

echo "[recS] recording sensor+GT -> $BAG (raw ros2 bag record)"
ros2 bag record -s sqlite3 -o "$BAG" \
  /argus/cam0/image_raw /argus/cam0/camera_info \
  /argus/cam1/image_raw /argus/cam1/camera_info \
  /argus/imu /argus/ground_truth/pose /clock >>"$LOG" 2>&1 &
REC_PID=$!
sleep 2

echo "[recS] flying ${LAPS} round-trips x ${LEG_M}m, speed=${SPEED}, no-yaw shuttle..."
python3 "$WS/scripts/fly_shuttle.py" --laps "$LAPS" --leg-m "$LEG_M" --speed "$SPEED" 2>&1 | tee "$WS/data/bags/_flyer_shuttle.log"

echo "[recS] flight done; stopping recorder (SIGINT -> finalize metadata)..."
kill -INT "$REC_PID" 2>/dev/null || true
sleep 6

echo "[recS] teardown..."
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 6
pkill -INT -f "ros2 launch argus_bringup" 2>/dev/null || true
pkill -9 -f "gz-sim|ruby|parameter_bridge|camera_info_patch" 2>/dev/null || true
sleep 2

if [ -d "$BAG" ]; then
  echo "[recS] DONE. shuttle bag -> $BAG"
  du -sh "$BAG" 2>/dev/null || true
else
  echo "[recS] ERROR: bag not produced" >&2
  exit 1
fi
