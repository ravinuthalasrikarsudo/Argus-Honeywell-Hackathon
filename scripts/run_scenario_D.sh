#!/usr/bin/env bash
# ARGUS Day-4 :: run_scenario_D.sh
#
# LIVE Scenario D ("lights-off") run: sim + VINS + health monitor + a health-aware
# forward flight that blacks out Zone B mid-traverse. Records GT + VIO odom + the
# VIOHealth stream so the health LOST episode and (in C3) the recovery hold can be
# evaluated and plotted.
#
# Architecture (clean, no cmd_vel contention): the health node only DETECTS and
# FLAGS (recovery_hold_cmd=false); the flight node ACTS on the flag -- in C3 it
# yields its forward command to a hold while recovery_active is true. The monitor
# detects, the planner reacts.
#
#   C3  recovery ON : health enable_recovery=true,  flight --yield-recovery
#                     -> drone holds in the dark, auto-resumes when re-acquired.
#   C1  recovery OFF: health enable_recovery=false, flight drives straight through
#                     -> dead-reckons blind through the blackout (the baseline).
#
# Usage:  bash scripts/run_scenario_D.sh [c3|c1]
# Env: SPEED(0.5) BLACKOUT_X(10) DARK_S(6) STOP_X(27) SETTLE_S(12) VINS_SETTLE_S(8)
set -uo pipefail

WS=/home/vittal/argus
MODE="${1:-c3}"
SPEED="${SPEED:-0.5}"
BLACKOUT_X="${BLACKOUT_X:-10}"
DARK_S="${DARK_S:-6}"
STOP_X="${STOP_X:-27}"
SETTLE_S="${SETTLE_S:-12}"
VINS_SETTLE_S="${VINS_SETTLE_S:-8}"

case "$MODE" in
  c3) ENABLE_RECOVERY=true;  YIELD="--yield-recovery" ;;
  c1) ENABLE_RECOVERY=false; YIELD="" ;;
  *)  echo "usage: run_scenario_D.sh [c3|c1]" >&2; exit 2 ;;
esac

OUT="$WS/data/eval/scenario_D/$MODE"
BAG="$OUT/bag"
LOG="$OUT/run.log"

# NOTE: do NOT export ROS_LOCALHOST_ONLY=1 (it breaks VINS stereo, see memory).
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

mkdir -p "$OUT"
rm -rf "$BAG"

cleanup_stray() {
  pkill -9 -f "gz sim" 2>/dev/null || true
  pkill -9 -f "parameter_bridge" 2>/dev/null || true
  pkill -9 -f "camera_info_patch" 2>/dev/null || true
  pkill -9 -f "vins_node" 2>/dev/null || true
  pkill -9 -f "health_monitor" 2>/dev/null || true
}
echo "[D:$MODE] pre-launch cleanup..."
cleanup_stray; sleep 2

echo "[D:$MODE] launching sim (headless)..."
ros2 launch argus_bringup argus_sim.launch.py headless:=true >"$LOG" 2>&1 &
LAUNCH_PID=$!
echo "[D:$MODE] settling ${SETTLE_S}s (gz + bridge + drone spawn)..."
sleep "$SETTLE_S"
if ! ros2 topic list 2>/dev/null | grep -q "/argus/ground_truth/pose"; then
  echo "[D:$MODE] ERROR: /argus topics not up; see $LOG" >&2
  kill -INT "$LAUNCH_PID" 2>/dev/null || true; sleep 4; cleanup_stray; exit 1
fi

echo "[D:$MODE] launching VINS-Fusion..."
ros2 launch argus_vio argus_vio.launch.py >"$OUT/vins.log" 2>&1 &
VINS_PID=$!
sleep "$VINS_SETTLE_S"

echo "[D:$MODE] launching health monitor (enable_recovery=$ENABLE_RECOVERY)..."
ros2 run argus_health health_monitor --ros-args \
  -p use_sim_time:=true -p enable_recovery:=$ENABLE_RECOVERY -p recovery_hold_cmd:=false \
  >"$OUT/health.log" 2>&1 &
HEALTH_PID=$!
sleep 2

echo "[D:$MODE] recording GT + VIO + health -> $BAG"
ros2 bag record -s sqlite3 -o "$BAG" \
  /argus/ground_truth/pose /argus/vio/odom /argus/vio/odom_optimized \
  /argus/vio/health /argus/health/recovery_active /clock >>"$LOG" 2>&1 &
REC_PID=$!
sleep 2

echo "[D:$MODE] flying (blackout_x=$BLACKOUT_X dark_s=$DARK_S stop_x=$STOP_X yield='$YIELD')..."
python3 "$WS/scripts/fly_scenario_D.py" \
  --speed "$SPEED" --blackout-x "$BLACKOUT_X" --dark-s "$DARK_S" --stop-x "$STOP_X" \
  $YIELD --blackout-script "$WS/scripts/blackout.sh" 2>&1 | tee -a "$LOG" | grep -E "phase|LIGHTS|YIELD|recovery|finished" || true

echo "[D:$MODE] flight done; safety lights-on + flush..."
bash "$WS/scripts/blackout.sh" on >/dev/null 2>&1 || true
sleep 4

echo "[D:$MODE] teardown..."
kill -INT "$REC_PID" 2>/dev/null || true
sleep 3
kill -INT "$HEALTH_PID" 2>/dev/null || true
kill -INT "$VINS_PID" 2>/dev/null || true
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 6
cleanup_stray
sleep 2

# --- Quick report ---
echo ""
echo "[D:$MODE] health recovery events:"
grep -E "RECOVERY|recovery_count" "$OUT/health.log" 2>/dev/null | tail -8 || echo "  (none)"
echo "[D:$MODE] recorded bag:"
ros2 bag info "$BAG" 2>/dev/null | grep -E "Duration|health|ground_truth|Messages:" || echo "  (no bag)"
echo "[D:$MODE] VINS throw img1 count: $(grep -c 'throw img1' "$OUT/vins.log" 2>/dev/null || echo 0)"
echo "[D:$MODE] DONE -> $OUT"
