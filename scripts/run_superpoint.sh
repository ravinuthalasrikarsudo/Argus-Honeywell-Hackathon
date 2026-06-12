#!/usr/bin/env bash
# ARGUS :: run_superpoint.sh
#
# Launch the standalone SuperPoint front-end under the GPU venv (onnxruntime-gpu),
# with the ROS env sourced so rclpy / sensor_msgs resolve. The colcon console
# entry point cannot be used directly because it runs under system python3, which
# has no onnxruntime; this wrapper runs ~/.venvs/argus-sp/bin/python on the node
# module instead (the venv is --system-site-packages, so ROS python is visible).
#
# Usage:  bash scripts/run_superpoint.sh [extra --ros-args ...]
# Env:    SP_VENV (~/.venvs/argus-sp)  FORCE_CPU(0)  INFER_W(0) INFER_H(0)
set -uo pipefail

WS=/home/vittal/argus
SP_VENV="${SP_VENV:-$HOME/.venvs/argus-sp}"
FORCE_CPU="${FORCE_CPU:-0}"
INFER_W="${INFER_W:-0}"
INFER_H="${INFER_H:-0}"

set +u
source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

# onnxruntime-gpu[cuda,cudnn] ships the CUDA/cuDNN .so under the venv's nvidia/*
# wheels; expose them so the CUDA EP loads (preload_dlls handles most, this is belt
# + braces for older loaders).
NVLIBS=$(find "$SP_VENV/lib" -maxdepth 3 -type d -path "*nvidia*lib" 2>/dev/null | tr '\n' ':')
export LD_LIBRARY_PATH="${NVLIBS}${LD_LIBRARY_PATH:-}"

cd "$WS"
PYTHONPATH="$WS/src/argus_superpoint:${PYTHONPATH:-}" exec "$SP_VENV/bin/python" \
  -m argus_superpoint.superpoint_node --ros-args \
  -p use_sim_time:=true \
  -p force_cpu:="$([ "$FORCE_CPU" = "1" ] && echo true || echo false)" \
  -p infer_width:="$INFER_W" -p infer_height:="$INFER_H" \
  "$@"
