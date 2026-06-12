#!/usr/bin/env bash
# ARGUS Day-4 :: record_scenario_D.sh
#
# Record the Scenario D ("lights-off") SENSOR bag under LIGHT load (sim + flight
# only, no VINS/health) so the replayed bag gives VINS a full per-frame budget.
#
# The flight is the PROVEN-good Day-3 profile: drive_drone --pattern forward with a
# smooth accel ramp (a one-step 0->cruise velocity jump produces a large kinematic
# IMU impulse that wrecks VINS init -> divergence). The Zone-B blackout is fired by
# a TIMED background subshell (decoupled from the flight controller), and the
# recorder self-stops on its --duration so rosbag2 finalizes metadata.yaml cleanly.
#
# Usage:  bash scripts/record_scenario_D.sh [BAG_DIR]
# Env: SPEED(0.5) RAMP_S(2) HOVER_S(5) DRIVE_S(58) BLACKOUT_DELAY(18) DARK_S(8) SETTLE_S(12)
set -uo pipefail

WS=/home/vittal/argus
BAG="${1:-$WS/data/bags/scenario_D}"
SPEED="${SPEED:-0.5}"
RAMP_S="${RAMP_S:-2}"
HOVER_S="${HOVER_S:-5}"
DRIVE_S="${DRIVE_S:-58}"
BLACKOUT_DELAY="${BLACKOUT_DELAY:-18}"   # after drive start: ~x=10 (Zone B entry)
DARK_S="${DARK_S:-8}"
SETTLE_S="${SETTLE_S:-12}"
LOG="$WS/data/bags/_record_D.log"
REC_S=$(( HOVER_S + DRIVE_S + 6 ))

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

mkdir -p "$WS/data/bags"; rm -rf "$BAG"

echo "[recD] pre-launch cleanup..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "camera_info_patch" 2>/dev/null || true
sleep 2

echo "[recD] launching sim (headless)..."
ros2 launch argus_bringup argus_sim.launch.py headless:=true >"$LOG" 2>&1 &
LAUNCH_PID=$!
echo "[recD] settling ${SETTLE_S}s..."
sleep "$SETTLE_S"
if ! ros2 topic list 2>/dev/null | grep -q "/argus/ground_truth/pose"; then
  echo "[recD] ERROR: /argus topics not up; see $LOG" >&2
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 4; pkill -9 -f "gz sim|ruby|parameter_bridge|camera_info_patch" 2>/dev/null || true
  exit 1
fi

echo "[recD] recording sensor+GT for ${REC_S}s -> $BAG"
ros2 run argus_bringup record_bag -o "$BAG" -d "$REC_S" >>"$LOG" 2>&1 &
REC_PID=$!
sleep 2

echo "[recD] start hover ${HOVER_S}s (VINS init)..."
sleep "$HOVER_S"

# Timed Zone-B blackout, decoupled from the flight controller.
echo "[recD] arming blackout: off at +${BLACKOUT_DELAY}s for ${DARK_S}s..."
(
  sleep "$BLACKOUT_DELAY"
  bash "$WS/scripts/blackout.sh" off >>"$LOG" 2>&1
  sleep "$DARK_S"
  bash "$WS/scripts/blackout.sh" on  >>"$LOG" 2>&1
) &
BLACK_PID=$!

echo "[recD] forward drive (ramp ${RAMP_S}s, speed ${SPEED}, ${DRIVE_S}s)..."
ros2 run argus_bringup drive_drone --pattern forward --speed "$SPEED" --ramp "$RAMP_S" --duration "$DRIVE_S"

echo "[recD] drive done; ensuring lights on + waiting for recorder to finalize..."
wait "$BLACK_PID" 2>/dev/null || true
bash "$WS/scripts/blackout.sh" on >/dev/null 2>&1 || true
wait "$REC_PID" 2>/dev/null || true

echo "[recD] teardown..."
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 6
pkill -9 -f "gz sim|ruby|parameter_bridge|camera_info_patch" 2>/dev/null || true
sleep 2

if [ -d "$BAG" ]; then
  echo "[recD] DONE -> $BAG"; du -sh "$BAG" 2>/dev/null || true
  ros2 bag info "$BAG" 2>/dev/null | grep -E "Duration|image_raw|imu \||metadata" || true
else
  echo "[recD] ERROR: bag not produced" >&2; exit 1
fi
