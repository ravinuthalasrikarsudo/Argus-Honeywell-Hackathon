#!/usr/bin/env bash
# ARGUS :: run_vio_superpoint_offline.sh
#
# C2 (SuperPoint front-end) offline VIO eval. Mirror of run_vio_offline.sh, but:
#   * also starts the SuperPoint node (argus_superpoint, overlay OFF) on the dGPU,
#   * points VINS at argus_stereo_imu_superpoint_config.yaml (use_superpoint:1),
# so the estimator's new features come from the learned detector instead of Harris.
# Replays a SENSOR bag -> records GT + VIO odom into an EVAL bag (deterministic,
# RTF-decoupled at --rate < 1; SuperPoint at 16.8 Hz keeps up easily).
#
# Usage:  bash scripts/run_vio_superpoint_offline.sh [SRC_SENSOR_BAG] [OUT_EVAL_BAG]
# Env:    RATE(0.4)  SP_SETTLE_S(15, ONNX+CUDA init)  SETTLE_S(6)  FLUSH_S(10)
set -uo pipefail

WS=/home/vittal/argus
SRC_BAG="${1:-$WS/data/bags/baseline_ABC}"
EVAL_BAG="${2:-$WS/data/bags/vio_eval_sp}"
RATE="${RATE:-0.4}"
SP_SETTLE_S="${SP_SETTLE_S:-15}"
SETTLE_S="${SETTLE_S:-6}"
FLUSH_S="${FLUSH_S:-10}"
SP_VENV="${SP_VENV:-$HOME/.venvs/argus-sp}"
CFG="$WS/src/argus_vio/config/argus_stereo_imu_superpoint_config.yaml"
LOG="$WS/data/eval/_vins_sp.log"
SPLOG="$WS/data/eval/_superpoint.log"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

mkdir -p "$WS/data/eval"
if [ ! -d "$SRC_BAG" ]; then
  echo "[c2] ERROR: source bag not found: $SRC_BAG" >&2
  exit 1
fi
if [ ! -f "$CFG" ]; then
  echo "[c2] ERROR: SuperPoint config not found: $CFG" >&2
  exit 1
fi
rm -rf "$EVAL_BAG"

echo "[c2] starting SuperPoint front-end (overlay off)..."
NVLIBS=$(find "$SP_VENV/lib" -maxdepth 3 -type d -path "*nvidia*lib" 2>/dev/null | tr '\n' ':')
LD_LIBRARY_PATH="${NVLIBS}${LD_LIBRARY_PATH:-}" \
PYTHONPATH="$WS/src/argus_superpoint:${PYTHONPATH:-}" \
  "$SP_VENV/bin/python" -m argus_superpoint.superpoint_node --ros-args \
  -p use_sim_time:=true -p publish_overlay:=false >"$SPLOG" 2>&1 &
SP_PID=$!
sleep "$SP_SETTLE_S"

if ! kill -0 "$SP_PID" 2>/dev/null; then
  echo "[c2] ERROR: SuperPoint node exited during warmup; see $SPLOG" >&2
  tail -30 "$SPLOG" >&2 || true
  exit 1
fi
if grep -q "CUDAExecutionProvider" "$SPLOG"; then
  echo "[c2] SuperPoint running on CUDA EP."
else
  echo "[c2] WARN: SuperPoint may be on CPU (slow); check $SPLOG" >&2
fi

echo "[c2] starting VINS-Fusion (use_superpoint:1)..."
ros2 launch argus_vio argus_vio.launch.py config:="$CFG" >"$LOG" 2>&1 &
VINS_PID=$!
sleep "$SETTLE_S"

if ! kill -0 "$VINS_PID" 2>/dev/null; then
  echo "[c2] ERROR: vins_node exited during warmup; see $LOG" >&2
  tail -20 "$LOG" >&2 || true
  kill -INT "$SP_PID" 2>/dev/null || true
  exit 1
fi

echo "[c2] recording GT + VIO odom -> $EVAL_BAG"
ros2 bag record -s sqlite3 -o "$EVAL_BAG" \
  /argus/ground_truth/pose /argus/vio/odom /argus/vio/odom_optimized \
  /argus/vio/path /clock >>"$LOG" 2>&1 &
REC_PID=$!
sleep 2

echo "[c2] replaying sensor bag at rate=${RATE} (blocks until done)..."
ros2 bag play "$SRC_BAG" --clock --rate "$RATE"

echo "[c2] replay done; flushing ${FLUSH_S}s..."
sleep "$FLUSH_S"

echo "[c2] teardown..."
kill -INT "$REC_PID" 2>/dev/null || true
sleep 3
kill -INT "$VINS_PID" 2>/dev/null || true
kill -INT "$SP_PID" 2>/dev/null || true
sleep 2
pkill -9 -f "vins_node" 2>/dev/null || true

echo "[c2] SuperPoint match/fallback tally:"
grep "\[C2\] superpoint frames" "$LOG" | tail -1 || echo "[c2]   (no tally line -- check $LOG)"

if [ -d "$EVAL_BAG" ]; then
  echo "[c2] DONE. eval bag -> $EVAL_BAG"
else
  echo "[c2] ERROR: eval bag not produced; see $LOG" >&2
  exit 1
fi
