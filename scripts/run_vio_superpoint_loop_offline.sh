#!/usr/bin/env bash
# ARGUS :: run_vio_superpoint_loop_offline.sh
#
# C2 (SuperPoint) + loop_fusion offline pass for Scenario A/C on the shuttle.
# = run_vio_loop_offline.sh (VINS + loop_fusion pose-graph) but with the SuperPoint
# node feeding learned keypoints and config:=argus_stereo_imu_superpoint_config.yaml.
# Records GT + odom_optimized (before) + odom_loop (after) so run_day5_evals.sh can
# slice Scenario A (first leg) and C (before/after loop closure).
#
# Usage:  bash scripts/run_vio_superpoint_loop_offline.sh [SRC_SENSOR_BAG] [OUT_EVAL_BAG]
# Env:    RATE(0.4)  SP_SETTLE_S(15)  SETTLE_S(10, DBoW vocab)  FLUSH_S(12)
# NOTE: SP + VINS + loop_fusion + play + record is memory-heavy on 7.4 GB. Day-3
# saw loop_fusion OOM at TEARDOWN (data still complete). If it dies mid-run, fall
# back to run_vio_superpoint_offline.sh on the shuttle (odom_optimized only = A + C-before).
set -uo pipefail

WS=/home/vittal/argus
SRC_BAG="${1:-$WS/data/bags/scenario_C_shuttle}"
EVAL_BAG="${2:-$WS/data/bags/vio_eval_sp_shuttle}"
RATE="${RATE:-0.4}"
SP_SETTLE_S="${SP_SETTLE_S:-15}"
SETTLE_S="${SETTLE_S:-10}"
FLUSH_S="${FLUSH_S:-12}"
SP_VENV="${SP_VENV:-$HOME/.venvs/argus-sp}"
CFG="$WS/src/argus_vio/config/argus_stereo_imu_superpoint_config.yaml"
LOG="$WS/data/eval/_vins_sp_loop.log"
SPLOG="$WS/data/eval/_superpoint_loop.log"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

mkdir -p "$WS/data/eval"
if [ ! -d "$SRC_BAG" ]; then
  echo "[c2loop] ERROR: source bag not found: $SRC_BAG" >&2
  exit 1
fi
if [ ! -f "$CFG" ]; then
  echo "[c2loop] ERROR: SuperPoint config not found: $CFG" >&2
  exit 1
fi
rm -rf "$EVAL_BAG"

echo "[c2loop] starting SuperPoint front-end (overlay off)..."
NVLIBS=$(find "$SP_VENV/lib" -maxdepth 3 -type d -path "*nvidia*lib" 2>/dev/null | tr '\n' ':')
LD_LIBRARY_PATH="${NVLIBS}${LD_LIBRARY_PATH:-}" \
PYTHONPATH="$WS/src/argus_superpoint:${PYTHONPATH:-}" \
  "$SP_VENV/bin/python" -m argus_superpoint.superpoint_node --ros-args \
  -p use_sim_time:=true -p publish_overlay:=false >"$SPLOG" 2>&1 &
SP_PID=$!
sleep "$SP_SETTLE_S"

if ! kill -0 "$SP_PID" 2>/dev/null; then
  echo "[c2loop] ERROR: SuperPoint node exited during warmup; see $SPLOG" >&2
  tail -30 "$SPLOG" >&2 || true
  exit 1
fi
if grep -q "CUDAExecutionProvider" "$SPLOG"; then
  echo "[c2loop] SuperPoint running on CUDA EP."
else
  echo "[c2loop] WARN: SuperPoint may be on CPU (slow); check $SPLOG" >&2
fi

echo "[c2loop] starting VINS-Fusion + loop_fusion (use_superpoint:1)..."
ros2 launch argus_vio argus_vio_loop.launch.py config:="$CFG" >"$LOG" 2>&1 &
VINS_PID=$!
sleep "$SETTLE_S"

if ! kill -0 "$VINS_PID" 2>/dev/null; then
  echo "[c2loop] ERROR: estimator stack exited during warmup; see $LOG" >&2
  tail -30 "$LOG" >&2 || true
  kill -INT "$SP_PID" 2>/dev/null || true
  exit 1
fi

echo "[c2loop] recording GT + optimized + loop-corrected odom -> $EVAL_BAG"
# Trimmed to the 3 eval-critical topics (was OOM-corrupting the 502s shuttle bag on
# 7.4GB RAM: the cumulative /path,/base_path,/loop_closures Path msgs + 250Hz
# /argus/vio/odom + /clock blew the recorder buffer -> SIGKILL -> malformed db3).
# run_eval.py reads message header stamps (not /clock); loop-edge count comes from
# the "optimize pose graph" tally in $LOG.
ros2 bag record -s sqlite3 -o "$EVAL_BAG" \
  /argus/ground_truth/pose /argus/vio/odom_optimized /argus/vio/odom_loop \
  >>"$LOG" 2>&1 &
REC_PID=$!
sleep 2

echo "[c2loop] replaying shuttle at rate=${RATE} (~21 min wall at 502s/0.4; blocks)..."
ros2 bag play "$SRC_BAG" --clock --rate "$RATE"

echo "[c2loop] replay done; flushing ${FLUSH_S}s (pose-graph tail)..."
sleep "$FLUSH_S"

echo "[c2loop] teardown..."
kill -INT "$REC_PID" 2>/dev/null || true
sleep 3
kill -INT "$VINS_PID" 2>/dev/null || true
kill -INT "$SP_PID" 2>/dev/null || true
sleep 3
pkill -9 -f "vins_node" 2>/dev/null || true
pkill -9 -f "loop_fusion_node" 2>/dev/null || true

DET=$(grep -c "detect loop with" "$LOG" 2>/dev/null || echo 0)
OPT=$(grep -c "optimize pose graph" "$LOG" 2>/dev/null || echo 0)
echo "[c2loop] DBoW loop candidates: $DET ; accepted pose-graph corrections: $OPT"
echo "[c2loop] SuperPoint match/fallback tally:"
grep "\[C2\] superpoint frames" "$LOG" | tail -1 || echo "[c2loop]   (no tally -- check $LOG)"

if [ -d "$EVAL_BAG" ]; then
  echo "[c2loop] DONE. eval bag -> $EVAL_BAG"
else
  echo "[c2loop] ERROR: eval bag not produced; see $LOG" >&2
  exit 1
fi
