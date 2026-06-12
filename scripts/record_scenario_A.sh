#!/usr/bin/env bash
# ARGUS :: record_scenario_A.sh
#
# Records the canonical Scenario A ("easy" textured corridor) SENSOR bag with a
# CLEAN flight profile: a brief start hover (lets VINS initialize on a static
# stereo view), a smooth accel ramp to cruise, a straight forward traverse
# through zones A->B->C, a smooth decel ramp, and an end hover (lets the
# estimator lock the final keyframes). The ramps avoid the one-step velocity
# jumps of the Day-2 baseline bag, whose kinematic VelocityControl steps
# produced large IMU acceleration impulses at motion onset/offset -> VIO
# start/stop transients (the dominant Day-3 residual error).
#
# Usage:  bash scripts/record_scenario_A.sh [BAG_DIR]
# Env: SPEED(0.5) DRIVE_S(95) RAMP_S(2) HOVER_START_S(5) HOVER_END_S(8) SETTLE_S(12)
set -uo pipefail

WS=/home/vittal/argus
BAG="${1:-$WS/data/bags/scenario_A}"
SPEED="${SPEED:-0.5}"
DRIVE_S="${DRIVE_S:-95}"
RAMP_S="${RAMP_S:-2}"
HOVER_START_S="${HOVER_START_S:-5}"
HOVER_END_S="${HOVER_END_S:-8}"
SETTLE_S="${SETTLE_S:-12}"
LOG="$WS/data/bags/_record_A.log"
REC_S=$(( HOVER_START_S + DRIVE_S + HOVER_END_S ))

# ROS env (guarded: ROS setup.bash trips `set -u` on AMENT_TRACE_SETUP_FILES).
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

mkdir -p "$WS/data/bags"
rm -rf "$BAG"

# Pre-launch cleanup: kill any stray gz/bridge from a prior run whose teardown
# failed (ghost sim pollutes /argus topics on the shared domain + thrashes RAM).
echo "[recA] pre-launch cleanup of stray sim procs..."
pkill -9 -f "gz sim -s" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "camera_info_patch" 2>/dev/null || true
sleep 2

echo "[recA] launching sim (headless)..."
ros2 launch argus_bringup argus_sim.launch.py headless:=true >"$LOG" 2>&1 &
LAUNCH_PID=$!
echo "[recA] settling ${SETTLE_S}s for gz + bridge + drone spawn..."
sleep "$SETTLE_S"

# Confirm the contract is live before recording (fail fast if the sim died).
if ! ros2 topic list 2>/dev/null | grep -q "/argus/ground_truth/pose"; then
  echo "[recA] ERROR: /argus topics not up after settle; see $LOG" >&2
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 4; pkill -9 -f "gz-sim|ruby|parameter_bridge|camera_info_patch" 2>/dev/null || true
  exit 1
fi

echo "[recA] recording sensor+GT for ${REC_S}s -> $BAG"
ros2 run argus_bringup record_bag -o "$BAG" -d "$REC_S" >>"$LOG" 2>&1 &
REC_PID=$!

echo "[recA] start hover ${HOVER_START_S}s (VINS init on static stereo)..."
sleep "$HOVER_START_S"

echo "[recA] forward: speed=${SPEED} ramp=${RAMP_S}s drive=${DRIVE_S}s wall..."
ros2 run argus_bringup drive_drone --pattern forward --speed "$SPEED" --ramp "$RAMP_S" --duration "$DRIVE_S"

echo "[recA] decel done; end hover, waiting for recorder to finalize..."
wait "$REC_PID" 2>/dev/null || true

echo "[recA] teardown..."
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 6
pkill -INT -f "ros2 launch argus_bringup" 2>/dev/null || true
pkill -9 -f "gz-sim|ruby|parameter_bridge|camera_info_patch" 2>/dev/null || true
sleep 2

if [ -d "$BAG" ]; then
  echo "[recA] DONE. scenario A bag -> $BAG"
  du -sh "$BAG" 2>/dev/null || true
else
  echo "[recA] ERROR: bag not produced" >&2
  exit 1
fi
