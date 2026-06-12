#!/usr/bin/env bash
# ARGUS :: record_scenario_C_uturn.sh
#
# Records a multi-lap SENSOR bag flown with GT-feedback 180-deg U-turns
# (fly_uturn_laps.py) instead of backward-flight reversals (which diverge VINS).
# This is the long-path drift gate: ~LAPS*2*LEG_M metres of accumulated travel
# with rotational excitation at each turn.
#
# Usage:  bash scripts/record_scenario_C_uturn.sh [BAG_DIR]
# Env: LAPS(3) LEG_M(14) SPEED(0.6) REC_S(700 upper bound) SETTLE_S(12)
set -uo pipefail

WS=/home/vittal/argus
BAG="${1:-$WS/data/bags/scenario_C_uturn}"
LAPS="${LAPS:-3}"
LEG_M="${LEG_M:-14}"
SPEED="${SPEED:-0.6}"
REC_S="${REC_S:-700}"        # safety upper bound; flyer finishes earlier and we stop the recorder
SETTLE_S="${SETTLE_S:-12}"
LOG="$WS/data/bags/_record_uturn.log"

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
# failed. A ghost sim keeps publishing /argus/ground_truth/pose on the shared
# domain (two drones -> GT jumps -> GT-feedback flight breaks) and thrashes RAM.
# Safe from pkill self-match: this script's own cmdline contains none of these.
echo "[recU] pre-launch cleanup of stray sim procs..."
pkill -9 -f "gz sim -s" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "camera_info_patch" 2>/dev/null || true
sleep 2

echo "[recU] launching sim (headless)..."
ros2 launch argus_bringup argus_sim.launch.py headless:=true >"$LOG" 2>&1 &
LAUNCH_PID=$!
echo "[recU] settling ${SETTLE_S}s for gz + bridge + drone spawn..."
sleep "$SETTLE_S"

if ! ros2 topic list 2>/dev/null | grep -q "/argus/ground_truth/pose"; then
  echo "[recU] ERROR: /argus topics not up after settle; see $LOG" >&2
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 4; pkill -9 -f "gz-sim|ruby|parameter_bridge|camera_info_patch" 2>/dev/null || true
  exit 1
fi

# Raw `ros2 bag record` (NOT the record_bag console tool): it finalizes the bag
# metadata cleanly on SIGINT, which the console tool does NOT -- the U-turn flight
# is variable-length so we stop the recorder on flight completion, not a timer.
echo "[recU] recording sensor+GT -> $BAG (raw ros2 bag record)"
ros2 bag record -s sqlite3 -o "$BAG" \
  /argus/cam0/image_raw /argus/cam0/camera_info \
  /argus/cam1/image_raw /argus/cam1/camera_info \
  /argus/imu /argus/ground_truth/pose /clock >>"$LOG" 2>&1 &
REC_PID=$!
sleep 2

echo "[recU] flying ${LAPS} laps x ${LEG_M}m, speed=${SPEED}, GT-feedback U-turns..."
python3 "$WS/scripts/fly_uturn_laps.py" --laps "$LAPS" --leg-m "$LEG_M" --speed "$SPEED" 2>&1 | tee "$WS/data/bags/_flyer.log"

echo "[recU] flight done; stopping recorder (SIGINT -> finalize metadata)..."
kill -INT "$REC_PID" 2>/dev/null || true
sleep 6

echo "[recU] teardown..."
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 6
pkill -INT -f "ros2 launch argus_bringup" 2>/dev/null || true
pkill -9 -f "gz-sim|ruby|parameter_bridge|camera_info_patch" 2>/dev/null || true
sleep 2

if [ -d "$BAG" ]; then
  echo "[recU] DONE. uturn bag -> $BAG"
  du -sh "$BAG" 2>/dev/null || true
else
  echo "[recU] ERROR: bag not produced" >&2
  exit 1
fi
