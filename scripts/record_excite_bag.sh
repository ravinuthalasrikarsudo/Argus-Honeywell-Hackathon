#!/usr/bin/env bash
# ARGUS :: record_excite_bag.sh  (Day-6)
#
# Like record_baseline_bag.sh, but flies a short multi-axis EXCITATION PRE-ROLL
# (vertical bob + lateral) BEFORE the forward drive. The pre-roll gives VINS-Fusion
# strong, multi-axis accel transients at init -> gravity direction + accel bias are
# well-observed -> robust, low-pitch-error init -> flat-Z trajectory (no altitude ramp).
# Pure-forward flights starve init (gravity ambiguous on a single translation axis).
#
# Usage:  bash scripts/record_excite_bag.sh [BAG_DIR]
# Env:    SPEED(0.8) DRIVE_S(70) REC_S(92) SETTLE_S(12)
set -uo pipefail

WS=/home/vittal/argus
BAG="${1:-$WS/data/bags/excite_ABC}"
SPEED="${SPEED:-0.8}"
DRIVE_S="${DRIVE_S:-70}"
REC_S="${REC_S:-92}"
SETTLE_S="${SETTLE_S:-12}"
LOG="$WS/data/bags/_record_excite.log"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

mkdir -p "$WS/data/bags"
rm -rf "$BAG"

echo "[rec] killing any stray sims first (clean slate)..."
bash "$WS/scripts/_killsim.sh" >/dev/null 2>&1
sleep 2

echo "[rec] launching sim (headless)..."
ros2 launch argus_bringup argus_sim.launch.py headless:=true >"$LOG" 2>&1 &
LAUNCH_PID=$!
echo "[rec] settling ${SETTLE_S}s..."
sleep "$SETTLE_S"

if ! ros2 topic list 2>/dev/null | grep -q "/argus/ground_truth/pose"; then
  echo "[rec] ERROR: /argus topics not up; see $LOG" >&2
  kill -INT "$LAUNCH_PID" 2>/dev/null || true; sleep 4
  pkill -9 -f "gz-sim|ruby|parameter_bridge|camera_info_patch" 2>/dev/null || true
  exit 1
fi

echo "[rec] recording for ${REC_S}s -> $BAG"
ros2 run argus_bringup record_bag -o "$BAG" -d "$REC_S" >>"$LOG" 2>&1 &
REC_PID=$!
sleep 2

echo "[rec] EXCITATION pre-roll (vertical bob + lateral; --vx 0 = pure axis)..."
ros2 run argus_bringup drive_drone --vx 0 --vz  0.4 --duration 2.0
ros2 run argus_bringup drive_drone --vx 0 --vz -0.4 --duration 2.0
ros2 run argus_bringup drive_drone --vx 0 --vy  0.3 --duration 1.8
ros2 run argus_bringup drive_drone --vx 0 --vy -0.3 --duration 1.8

echo "[rec] forward drive: speed=${SPEED} for ${DRIVE_S}s..."
ros2 run argus_bringup drive_drone --pattern forward --speed "$SPEED" --duration "$DRIVE_S"

echo "[rec] drive done; finalizing recorder..."
wait "$REC_PID" 2>/dev/null || true

echo "[rec] teardown (via _killsim -> kills 'gz sim' space-form too)..."
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 4
pkill -INT -f "ros2 launch argus_bringup" 2>/dev/null || true
bash "$WS/scripts/_killsim.sh" >/dev/null 2>&1
sleep 2

if [ -d "$BAG" ]; then
  echo "[rec] DONE -> $BAG"; du -sh "$BAG" 2>/dev/null || true
else
  echo "[rec] ERROR: bag not produced" >&2; exit 1
fi
