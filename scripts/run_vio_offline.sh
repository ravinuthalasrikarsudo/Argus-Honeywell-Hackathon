#!/usr/bin/env bash
# ARGUS :: run_vio_offline.sh
#
# Offline VIO evaluation pass: replay a baseline SENSOR bag through the
# VINS-Fusion estimator (argus_vio) and record ground truth + VIO odometry into
# an EVAL bag. Offline replay is deterministic and decoupled from sim RTF, so the
# estimator gets a full per-frame compute budget (replay --rate < 1).
#
# Usage:  bash scripts/run_vio_offline.sh [SRC_SENSOR_BAG] [OUT_EVAL_BAG]
# Env:    RATE (replay speed, 0.4)  SETTLE_S (VINS warmup, 6)  FLUSH_S (3)
set -uo pipefail

WS=/home/vittal/argus
SRC_BAG="${1:-$WS/data/bags/baseline_ABC}"
EVAL_BAG="${2:-$WS/data/bags/vio_eval}"
RATE="${RATE:-0.15}"   # Day-6: 0.4 dropped frames -> non-determinism; with multiple_thread:0
                       # (single-thread, deterministic) VINS needs a slow replay to keep up
                       # (no drops). 0.15 is safe for <=15Hz cameras. See day6 log.
SETTLE_S="${SETTLE_S:-6}"
FLUSH_S="${FLUSH_S:-10}"   # Day-3: let VINS finish optimizing the tail keyframes
                           # before teardown (was 3; short flush left the final
                           # zone-C window un-optimized -> end-of-trajectory ramp)
LOG="$WS/data/eval/_vins.log"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

mkdir -p "$WS/data/eval"
if [ ! -d "$SRC_BAG" ]; then
  echo "[vio] ERROR: source bag not found: $SRC_BAG" >&2
  exit 1
fi
rm -rf "$EVAL_BAG"

# Day-7 hardening: eval config (show_track: 0 — the per-frame track image
# leaks in the port) and no GUI spectators (reliable subscribers on heavy
# topics force DDS retention inside vins_node -> OOM on long bags).
EVAL_CFG="$WS/install/argus_vio/share/argus_vio/config/argus_stereo_imu_eval_config.yaml"
if pgrep -f "rqt_image_vie[w]|rviz[2]" >/dev/null 2>&1; then
  echo "[vio] WARNING: killing attached GUI viewers (they OOM vins_node on long replays)" >&2
  for p in $(pgrep -f "rqt_image_vie[w]|rviz[2]"); do kill -9 "$p" 2>/dev/null || true; done
  sleep 2
fi

# glibc tuning (day-7): Ceres' per-solve small-allocation churn fragments the
# malloc arenas — RSS ratchets ~3.6 MB per processed frame (count-based,
# image-size-independent; heap-internal, not mmap'd) until the 14 GB host
# OOM-kills vins_node ~60% into a 207 m bag. Single arena + low mmap
# threshold + aggressive trim lets the allocator return solver memory.
export MALLOC_ARENA_MAX=1
export MALLOC_MMAP_THRESHOLD_=131072
export MALLOC_TRIM_THRESHOLD_=8388608

echo "[vio] starting VINS-Fusion (argus_vio, eval config)..."
ros2 launch argus_vio argus_vio.launch.py config:="$EVAL_CFG" >"$LOG" 2>&1 &
VINS_PID=$!
sleep "$SETTLE_S"

if ! kill -0 "$VINS_PID" 2>/dev/null; then
  echo "[vio] ERROR: vins_node exited during warmup; see $LOG" >&2
  tail -20 "$LOG" >&2 || true
  exit 1
fi

# No cumulative /argus/vio/path (quadratic growth -> DDS writer retention
# inside vins_node -> OOM on long bags; day-7) and no /clock (run_eval.py
# reads header stamps).
echo "[vio] recording GT + VIO odom -> $EVAL_BAG"
ros2 bag record -s sqlite3 -o "$EVAL_BAG" \
  /argus/ground_truth/pose /argus/vio/odom /argus/vio/odom_optimized \
  >>"$LOG" 2>&1 &
REC_PID=$!
sleep 2

echo "[vio] replaying sensor bag at rate=${RATE} (blocks until done)..."
ros2 bag play "$SRC_BAG" --clock --rate "$RATE"

echo "[vio] replay done; flushing ${FLUSH_S}s..."
sleep "$FLUSH_S"

echo "[vio] teardown..."
kill -INT "$REC_PID" 2>/dev/null || true
sleep 3
kill -INT "$VINS_PID" 2>/dev/null || true
sleep 2
pkill -9 -f "vins_node" 2>/dev/null || true

if [ -d "$EVAL_BAG" ]; then
  echo "[vio] DONE. eval bag -> $EVAL_BAG"
else
  echo "[vio] ERROR: eval bag not produced; see $LOG" >&2
  exit 1
fi
