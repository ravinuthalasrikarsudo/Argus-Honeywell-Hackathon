#!/usr/bin/env bash
# ARGUS :: _health_smoke.sh  (Day-4 dev smoke, not a canonical deliverable)
#
# Replay a sensor bag -> VINS-Fusion -> argus_health monitor, RECORD the health +
# point_cloud topics with the efficient C++ recorder (no live python subscribers,
# so the estimator keeps a full CPU budget), then analyze the recording offline.
# Goal: confirm real topic wiring against a live estimator and capture the real
# inlier feature-count distribution used to tune the health thresholds. No Gazebo.
#
# Usage:  bash scripts/_health_smoke.sh [SENSOR_BAG]
# Env:    RATE (0.4)  SETTLE_S (8)
set -uo pipefail

WS=/home/vittal/argus
SRC_BAG="${1:-$WS/data/bags/baseline_ABC}"
RATE="${RATE:-0.4}"
SETTLE_S="${SETTLE_S:-8}"
OUT="$WS/data/eval/_health_smoke"
REC="$OUT/rec"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
# NOTE: do NOT set ROS_LOCALHOST_ONLY=1 here. Forcing Cyclone onto the loopback
# interface ("lo is not multicast-capable: disabling multicast") drops the large
# stereo image frames, desyncing VINS ("throw img1" on every cam1) so it never
# initializes. The known-good Day-2 run_vio_offline.sh omits it too.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

mkdir -p "$OUT"
rm -rf "$REC"
[ -d "$SRC_BAG" ] || { echo "[smoke] ERROR: bag not found: $SRC_BAG" >&2; exit 1; }

echo "[smoke] launching VINS-Fusion..."
ros2 launch argus_vio argus_vio.launch.py >"$OUT/vins.log" 2>&1 &
VINS_PID=$!
sleep "$SETTLE_S"
if ! kill -0 "$VINS_PID" 2>/dev/null; then
  echo "[smoke] ERROR: VINS died during warmup; tail:" >&2; tail -20 "$OUT/vins.log" >&2; exit 1
fi

H_PID=""
if [ "${WITH_HEALTH:-1}" = "1" ]; then
  echo "[smoke] launching health monitor (recovery hold OFF for smoke)..."
  ros2 run argus_health health_monitor --ros-args \
    -p use_sim_time:=true -p recovery_hold_cmd:=false >"$OUT/health.log" 2>&1 &
  H_PID=$!
  sleep 2
else
  echo "[smoke] WITH_HEALTH=0 -> health monitor NOT launched (control run)"
fi

echo "[smoke] recording health + point_cloud + odom_optimized..."
ros2 bag record -s sqlite3 -o "$REC" \
  /argus/vio/health /argus/vio/point_cloud /argus/vio/odom_optimized /clock \
  >"$OUT/record.log" 2>&1 &
REC_PID=$!
sleep 2

echo "[smoke] replaying $SRC_BAG @${RATE} (blocks)..."
ros2 bag play "$SRC_BAG" --clock --rate "$RATE"

echo "[smoke] replay done; flush 8s..."
sleep 8

echo "[smoke] teardown..."
kill -INT "$REC_PID" 2>/dev/null || true
sleep 3
[ -n "$H_PID" ] && kill -INT "$H_PID" 2>/dev/null || true
kill -INT "$VINS_PID" 2>/dev/null || true
sleep 3
pkill -9 -f "vins_node" 2>/dev/null || true

# --- Convergence guard: did VINS actually produce optimized output? ---
THROW=$(grep -c "throw img1" "$OUT/vins.log" 2>/dev/null || echo 0)
echo ""
echo "[smoke] VINS 'throw img1' count: $THROW"
echo "[smoke] recorded bag info:"
ros2 bag info "$REC" 2>/dev/null | grep -E "Messages:|point_cloud|health|odom_optimized" || echo "  (no bag)"

echo ""
echo "########## OFFLINE ANALYSIS ##########"
~/.venvs/argus-eval/bin/python "$WS/scripts/_health_sniff.py" --bag "$REC" --out "$OUT/timeline.csv" 2>&1 | grep -vE "multicast"
echo "######################################"
