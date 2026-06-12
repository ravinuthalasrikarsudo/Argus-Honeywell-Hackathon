#!/usr/bin/env bash
# ARGUS :: run.sh  —  launch the argus:humble container with GPU + GUI (run on the HOST).
#
#   docker/run.sh                 # open an interactive shell in the container
#   docker/run.sh <cmd...>        # run a one-off command (still GPU+X11 capable)
#
# The project is bind-mounted at the SAME path the repo hardcodes (/home/vittal/argus)
# so the src/VINS-Fusion-ROS2 symlink and config paths resolve. The container runs as
# uid/gid 1000 so files it writes (build/, install/, bags) stay owned by host `vittal`.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # -> /home/vittal/argus
IMAGE="${IMAGE:-argus:humble}"
NAME="${NAME:-argus}"

# Host is Wayland; XWayland exposes an X11 socket at $DISPLAY for the GUI apps
# (Gazebo GUI, RViz, rqt). Allow local container clients to connect.
xhost +local:root >/dev/null 2>&1 || true
xhost +local: >/dev/null 2>&1 || true

# Reuse a running container if present, else start a fresh one.
if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "[run] attaching to running container '$NAME'"
  exec docker exec -it "$NAME" bash -lc "${*:-bash}"
fi

ARGS=(
  --rm -it
  --name "$NAME"
  --gpus all
  -e NVIDIA_DRIVER_CAPABILITIES=all
  -e NVIDIA_VISIBLE_DEVICES=all
  -e DISPLAY="${DISPLAY:-:0}"
  -e QT_X11_NO_MITSHM=1
  # This laptop has an AMD 780M iGPU + an RTX 4050. gz/Qt's default EGL probe hits the
  # iGPU and fails ("failed to create dri2 screen") -> blank GUI. Force the NVIDIA GL path
  # (the one glxgears uses) and Qt onto X11, so Gazebo/RViz/rqt render correctly.
  -e QT_QPA_PLATFORM=xcb
  -e __NV_PRIME_RENDER_OFFLOAD=1
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia
  -e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  -e XDG_RUNTIME_DIR=/tmp/runtime-vittal
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -v "$REPO":/home/vittal/argus
  --network host
  --ipc host
)

if [ "$#" -eq 0 ]; then
  exec docker run "${ARGS[@]}" "$IMAGE" bash
else
  exec docker run "${ARGS[@]}" "$IMAGE" bash -lc "$*"
fi
