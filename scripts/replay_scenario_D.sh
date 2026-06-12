#!/usr/bin/env bash
# ARGUS Day-4 :: replay_scenario_D.sh
#
# Deterministic OFFLINE evaluation of the Scenario D sensor bag: replay it through
# VINS-Fusion + the health monitor with a full per-frame compute budget (no live
# sim competing), and record GT + VIO odom + the VIOHealth stream for analysis.
# This is the clean Pillar-3 detection result (NOMINAL -> blackout LOST -> recover),
# the offline analogue of run_vio_offline.sh.
#
# Usage:  bash scripts/replay_scenario_D.sh [c3|c1] [SENSOR_BAG]
# Env: RATE(0.4) VINS_SETTLE_S(8)
set -uo pipefail

WS=/home/vittal/argus
MODE="${1:-c3}"
SRC="${2:-$WS/data/bags/scenario_D}"
RATE="${RATE:-0.4}"
VINS_SETTLE_S="${VINS_SETTLE_S:-8}"

case "$MODE" in
  c3) ENABLE_RECOVERY=true ;;
  c1) ENABLE_RECOVERY=false ;;
  *)  echo "usage: replay_scenario_D.sh [c3|c1] [bag]" >&2; exit 2 ;;
esac

OUT="$WS/data/eval/scenario_D/offline_$MODE"
BAG="$OUT/bag"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

mkdir -p "$OUT"; rm -rf "$BAG"
[ -d "$SRC" ] || { echo "[replayD] ERROR: bag not found: $SRC" >&2; exit 1; }

cleanup() { pkill -9 -f "vins_node" 2>/dev/null || true; pkill -9 -f "health_monitor" 2>/dev/null || true; }
cleanup; sleep 1

echo "[replayD:$MODE] launching VINS-Fusion..."
ros2 launch argus_vio argus_vio.launch.py >"$OUT/vins.log" 2>&1 &
VINS_PID=$!
sleep "$VINS_SETTLE_S"
kill -0 "$VINS_PID" 2>/dev/null || { echo "[replayD] VINS died" >&2; tail -15 "$OUT/vins.log"; exit 1; }

echo "[replayD:$MODE] launching health monitor (enable_recovery=$ENABLE_RECOVERY)..."
ros2 run argus_health health_monitor --ros-args \
  -p use_sim_time:=true -p enable_recovery:=$ENABLE_RECOVERY -p recovery_hold_cmd:=false \
  >"$OUT/health.log" 2>&1 &
HEALTH_PID=$!
sleep 2

echo "[replayD:$MODE] recording GT + VIO + health -> $BAG"
ros2 bag record -s sqlite3 -o "$BAG" \
  /argus/ground_truth/pose /argus/vio/odom /argus/vio/odom_optimized \
  /argus/vio/health /argus/health/recovery_active /clock >>"$OUT/rec.log" 2>&1 &
REC_PID=$!
sleep 2

echo "[replayD:$MODE] replaying $SRC @${RATE} (blocks)..."
ros2 bag play "$SRC" --clock --rate "$RATE"

echo "[replayD:$MODE] flush 8s..."
sleep 8

echo "[replayD:$MODE] teardown..."
kill -INT "$REC_PID" 2>/dev/null || true; sleep 3
kill -INT "$HEALTH_PID" "$VINS_PID" 2>/dev/null || true; sleep 3
cleanup

echo "[replayD:$MODE] throw img1: $(grep -c 'throw img1' "$OUT/vins.log" 2>/dev/null || echo 0)"
grep -E "RECOVERY ENGAGED|RECOVERY CLEARED" "$OUT/health.log" 2>/dev/null | tail -6 || true
ros2 bag info "$BAG" 2>/dev/null | grep -E "health|odom \||ground_truth" || true
echo "[replayD:$MODE] DONE -> $OUT"
