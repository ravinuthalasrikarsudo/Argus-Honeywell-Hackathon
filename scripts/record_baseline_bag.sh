#!/usr/bin/env bash
# ARGUS :: record_baseline_bag.sh
#
# Orchestrates the Day-1 sim stack to produce the baseline SENSOR bag: launch the
# warehouse sim headless, fly the kinematic drone forward through zones A->B->C,
# and record the frozen /argus/* sensor + ground-truth contract topics. This bag
# is the reproducible input for offline VIO evaluation (run_vio_offline.sh).
#
# Usage:  bash scripts/record_baseline_bag.sh [BAG_DIR]
# Env:    SPEED (m/s, 0.8) DRIVE_S (wall s, 70) REC_S (wall s, 80) SETTLE_S (12)
#
# Note: timings are WALL seconds; under sim RTF~0.42 (iGPU) the drone covers
# ~0.42*SPEED*DRIVE_S metres of sim distance. Defaults aim ~x=1.5 -> ~x=25 (zone C).
set -uo pipefail

WS=/home/vittal/argus
BAG="${1:-$WS/data/bags/baseline_ABC}"
SPEED="${SPEED:-0.8}"
DRIVE_S="${DRIVE_S:-70}"
REC_S="${REC_S:-80}"
SETTLE_S="${SETTLE_S:-12}"
RAMP="${RAMP:-0.0}"   # Day-6: ramp DEFAULT OFF. A gentle ramp reduces early motion ->
                      # starves VINS init ("feature tracking not enough") -> can crash.
                      # Step-start inits reliably (mild z-ramp). Real init fix = excitation
                      # pre-roll (multi-axis motion before forward), not a ramp.
LOG="$WS/data/bags/_record.log"

# ROS env (guarded: ROS setup.bash trips `set -u` on AMENT_TRACE_SETUP_FILES).
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

echo "[rec] settling ${SETTLE_S}s for gz + bridge + drone spawn..."
sleep "$SETTLE_S"

# Confirm the contract is live before recording (fail fast if the sim died).
if ! ros2 topic list 2>/dev/null | grep -q "/argus/ground_truth/pose"; then
  echo "[rec] ERROR: /argus topics not up after settle; see $LOG" >&2
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 4; pkill -9 -f "gz-sim|ruby|parameter_bridge|camera_info_patch" 2>/dev/null || true
  exit 1
fi

echo "[rec] recording sensor+GT topics for ${REC_S}s -> $BAG"
ros2 run argus_bringup record_bag -o "$BAG" -d "$REC_S" >>"$LOG" 2>&1 &
REC_PID=$!
sleep 2

echo "[rec] flying forward: speed=${SPEED} ramp=${RAMP} for ${DRIVE_S}s wall..."
ros2 run argus_bringup drive_drone --pattern forward --speed "$SPEED" --duration "$DRIVE_S" --ramp "$RAMP"

echo "[rec] drive done; waiting for recorder to finalize..."
wait "$REC_PID" 2>/dev/null || true

echo "[rec] teardown..."
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 4
pkill -INT -f "ros2 launch argus_bringup" 2>/dev/null || true
bash "$WS/scripts/_killsim.sh" >/dev/null 2>&1
sleep 2

if [ -d "$BAG" ]; then
  echo "[rec] DONE. baseline bag -> $BAG"
  du -sh "$BAG" 2>/dev/null || true
else
  echo "[rec] ERROR: bag not produced" >&2
  exit 1
fi
