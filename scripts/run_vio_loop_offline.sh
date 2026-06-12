#!/usr/bin/env bash
# ARGUS :: run_vio_loop_offline.sh
#
# Offline VIO + loop-closure pass: replay a SENSOR bag through VINS-Fusion AND
# the loop_fusion pose-graph (argus_vio_loop.launch.py), recording ground truth,
# the raw optimized estimate, the loop-CORRECTED estimate, and the pose-graph
# path. Use for long-path / Scenario C (multi-lap) drift validation.
#
# Usage:  bash scripts/run_vio_loop_offline.sh [SRC_SENSOR_BAG] [OUT_EVAL_BAG]
# Env:    RATE(0.4)  SETTLE_S(10, loop_fusion loads the DBoW vocab)  FLUSH_S(12)
set -uo pipefail

WS=/home/vittal/argus
SRC_BAG="${1:-$WS/data/bags/scenario_C_loop}"
EVAL_BAG="${2:-$WS/data/bags/vio_eval_loop}"
RATE="${RATE:-0.4}"
SETTLE_S="${SETTLE_S:-10}"
FLUSH_S="${FLUSH_S:-12}"
LOG="$WS/data/eval/_vins_loop.log"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

mkdir -p "$WS/data/eval"
if [ ! -d "$SRC_BAG" ]; then
  echo "[loop] ERROR: source bag not found: $SRC_BAG" >&2
  exit 1
fi
rm -rf "$EVAL_BAG"

# Pre-flight (day-7): GUI viewers must NOT be attached during eval replays on
# RAM-limited hosts. A reliable subscriber on the 5.5 MB/frame image_track (or
# the cumulative Path topics) forces DDS writer retention inside vins_node ->
# ~8 GB anon RSS -> kernel OOM kill mid-bag. Watch the live demo instead, or
# re-attach viewers only to demo.sh sessions.
if pgrep -f "rqt_image_vie[w]|rviz[2]" >/dev/null 2>&1; then
  echo "[loop] WARNING: killing attached GUI viewers (rqt/rviz) — they OOM vins_node on long replays" >&2
  for p in $(pgrep -f "rqt_image_vie[w]|rviz[2]"); do kill -9 "$p" 2>/dev/null || true; done
  sleep 2
fi

# Eval config = production config with show_track: 0 (the per-frame 5.5 MB
# annotated track image leaks in the port -> OOM on long bags; day-7).
EVAL_CFG="$WS/install/argus_vio/share/argus_vio/config/argus_stereo_imu_eval_config.yaml"
echo "[loop] starting VINS-Fusion + loop_fusion (eval config, show_track off)..."
ros2 launch argus_vio argus_vio_loop.launch.py config:="$EVAL_CFG" >"$LOG" 2>&1 &
VINS_PID=$!
sleep "$SETTLE_S"

if ! kill -0 "$VINS_PID" 2>/dev/null; then
  echo "[loop] ERROR: estimator stack exited during warmup; see $LOG" >&2
  tail -30 "$LOG" >&2 || true
  exit 1
fi

# Eval-critical topics ONLY. The cumulative Path topics (/argus/vio/path,
# base_path, loop_closures) grow QUADRATICALLY in total bytes; a reliable
# subscriber (recorder or RViz) forces the DDS writer inside vins_node to
# retain them -> 8 GB anon RSS and a kernel OOM kill on long bags (day-7;
# day-5 hit the same wall on the recorder side). run_eval.py reads message
# header stamps, so /clock is not needed either.
echo "[loop] recording GT + raw + loop-corrected odom -> $EVAL_BAG"
ros2 bag record -s sqlite3 -o "$EVAL_BAG" \
  /argus/ground_truth/pose /argus/vio/odom_optimized \
  /argus/vio/odom_loop >>"$LOG" 2>&1 &
REC_PID=$!
sleep 2

echo "[loop] replaying sensor bag at rate=${RATE} (blocks until done)..."
ros2 bag play "$SRC_BAG" --clock --rate "$RATE"

echo "[loop] replay done; flushing ${FLUSH_S}s (pose-graph optimization tail)..."
sleep "$FLUSH_S"

echo "[loop] teardown..."
kill -INT "$REC_PID" 2>/dev/null || true
sleep 3
kill -INT "$VINS_PID" 2>/dev/null || true
sleep 3
pkill -9 -f "vins_node" 2>/dev/null || true
pkill -9 -f "loop_fusion_node" 2>/dev/null || true

# Loop-closure stats from loop_fusion's log: "detect loop with" = an ACCEPTED
# loop edge (DBoW candidate that passed findConnection's PnP-RANSAC geometric
# check). NOTE (day-7): "optimize pose graph" is the optimiser thread's
# PERIODIC tick in this ROS2 port — it fires with zero loop edges and must
# not be counted as a correction.
DET=$(grep -c "detect loop with" "$LOG" 2>/dev/null || echo 0)
echo "[loop] accepted loop closures (detect loop with): $DET (see $LOG)"

if [ -d "$EVAL_BAG" ]; then
  echo "[loop] DONE. eval bag -> $EVAL_BAG"
else
  echo "[loop] ERROR: eval bag not produced; see $LOG" >&2
  exit 1
fi
