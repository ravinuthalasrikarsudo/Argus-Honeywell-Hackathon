#!/usr/bin/env bash
# ARGUS :: record_scenario_C_loop.sh
#
# Multi-lap out-and-back traversal for long-path drift + loop-closure validation
# (Scenario C, also the spec-faithful "<1.5% over ~200m" gate). The drone flies
# forward (camera facing +x) to ~x=22, then BACKWARD (still facing +x) to ~x=1.5,
# repeated LAPS times. Same-ORIENTATION revisits let loop_fusion's DBoW recognize
# places and fire loop closures that BOUND accumulated drift. Smooth --ramp at
# each turnaround avoids the kinematic velocity-step IMU impulses.
#
# Usage:  bash scripts/record_scenario_C_loop.sh [BAG_DIR]
# Env: SPEED(0.7) LEG_S(70) LAPS(3) RAMP_S(2) HOVER_START_S(5) HOVER_END_S(8) SETTLE_S(12)
set -uo pipefail

WS=/home/vittal/argus
BAG="${1:-$WS/data/bags/scenario_C_loop}"
SPEED="${SPEED:-0.7}"
LEG_S="${LEG_S:-70}"
LAPS="${LAPS:-3}"
RAMP_S="${RAMP_S:-2}"
HOVER_START_S="${HOVER_START_S:-5}"
HOVER_END_S="${HOVER_END_S:-8}"
SETTLE_S="${SETTLE_S:-12}"
LOG="$WS/data/bags/_record_C.log"
# Record long enough to cover start hover + all legs (+ per-leg slack) + end hover.
REC_S=$(( HOVER_START_S + LAPS * 2 * LEG_S + HOVER_END_S + 30 ))

# ROS env (guarded: ROS setup.bash trips `set -u` on AMENT_TRACE_SETUP_FILES).
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

mkdir -p "$WS/data/bags"
rm -rf "$BAG"

echo "[recC] launching sim (headless)..."
ros2 launch argus_bringup argus_sim.launch.py headless:=true >"$LOG" 2>&1 &
LAUNCH_PID=$!
echo "[recC] settling ${SETTLE_S}s for gz + bridge + drone spawn..."
sleep "$SETTLE_S"

if ! ros2 topic list 2>/dev/null | grep -q "/argus/ground_truth/pose"; then
  echo "[recC] ERROR: /argus topics not up after settle; see $LOG" >&2
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 4; pkill -9 -f "gz-sim|ruby|parameter_bridge|camera_info_patch" 2>/dev/null || true
  exit 1
fi

echo "[recC] recording sensor+GT for ${REC_S}s -> $BAG"
ros2 run argus_bringup record_bag -o "$BAG" -d "$REC_S" >>"$LOG" 2>&1 &
REC_PID=$!

echo "[recC] start hover ${HOVER_START_S}s (VINS init on static stereo)..."
sleep "$HOVER_START_S"

for lap in $(seq 1 "$LAPS"); do
  echo "[recC] lap ${lap}/${LAPS}: forward leg (${LEG_S}s wall)..."
  ros2 run argus_bringup drive_drone --pattern forward  --speed "$SPEED" --ramp "$RAMP_S" --duration "$LEG_S"
  echo "[recC] lap ${lap}/${LAPS}: backward leg (${LEG_S}s wall, same heading)..."
  ros2 run argus_bringup drive_drone --pattern backward --speed "$SPEED" --ramp "$RAMP_S" --duration "$LEG_S"
done

echo "[recC] laps done; end hover, waiting for recorder to finalize..."
wait "$REC_PID" 2>/dev/null || true

echo "[recC] teardown..."
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 6
pkill -INT -f "ros2 launch argus_bringup" 2>/dev/null || true
pkill -9 -f "gz-sim|ruby|parameter_bridge|camera_info_patch" 2>/dev/null || true
sleep 2

if [ -d "$BAG" ]; then
  echo "[recC] DONE. loop bag -> $BAG"
  du -sh "$BAG" 2>/dev/null || true
else
  echo "[recC] ERROR: bag not produced" >&2
  exit 1
fi
