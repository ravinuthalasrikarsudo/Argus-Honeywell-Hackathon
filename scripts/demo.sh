#!/usr/bin/env bash
# ARGUS :: demo.sh -- one-command visual demos (WSLg GUI windows on Windows).
#
#   bash scripts/demo.sh superpoint   # SuperPoint keypoints overlaid on corridor video (rqt)
#   bash scripts/demo.sh fly          # live Gazebo GUI + drone auto-flies the corridor
#   bash scripts/demo.sh rviz         # RViz: VIO trajectory + loop-closure path (replay)
#
# Ctrl-C to stop a demo (it cleans up its own processes).
set -uo pipefail
WS=/home/vittal/argus
MODE="${1:-help}"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

cleanup() {
  echo "[demo] stopping..."
  kill -INT ${PIDS:-} 2>/dev/null || true
  sleep 2
  pkill -9 -f "argus_superpoint.superpoint_node" 2>/dev/null || true
  pkill -9 -f "rqt_image_view" 2>/dev/null || true
  pkill -9 -f "bag play" 2>/dev/null || true
  pkill -2 -f "ros2 launch argus_bringup" 2>/dev/null || true
  pkill -9 -f "gz-sim|ruby|parameter_bridge|camera_info_patch" 2>/dev/null || true
  pkill -9 -f "fly_shuttle" 2>/dev/null || true
}
trap cleanup INT TERM EXIT
PIDS=""

case "$MODE" in
  superpoint)
    SP=$HOME/.venvs/argus-sp/bin/python
    NVLIBS=$(find "$HOME/.venvs/argus-sp/lib" -maxdepth 3 -type d -path "*nvidia*lib" 2>/dev/null | tr '\n' ':')
    export LD_LIBRARY_PATH="${NVLIBS}${LD_LIBRARY_PATH:-}"
    echo "[demo] starting SuperPoint node (RTX 4050)..."
    PYTHONPATH="$WS/src/argus_superpoint:${PYTHONPATH:-}" "$SP" -m argus_superpoint.superpoint_node \
      --ros-args -p use_sim_time:=true > "$WS/data/eval/_demo_sp.log" 2>&1 &
    PIDS="$PIDS $!"
    sleep 8
    echo "[demo] opening rqt_image_view on /argus/superpoint/overlay ..."
    ros2 run rqt_image_view rqt_image_view /argus/superpoint/overlay > /dev/null 2>&1 &
    PIDS="$PIDS $!"
    sleep 4
    echo "[demo] looping corridor video (Zone B blank walls included). Ctrl-C to stop."
    ros2 bag play "$WS/data/bags/scenario_A_fwd" --loop --rate 0.6 \
      --topics /argus/cam0/image_raw /argus/cam0/camera_info /clock --clock
    ;;

  fly)
    echo "[demo] launching Gazebo GUI + bridge + drone (this is the warehouse sim)..."
    ros2 launch argus_bringup argus_sim.launch.py headless:=false > "$WS/data/eval/_demo_sim.log" 2>&1 &
    PIDS="$PIDS $!"
    echo "[demo] waiting 20s for Gazebo + drone spawn (iGPU render is slow)..."
    sleep 20
    if ! ros2 topic list 2>/dev/null | grep -q "/argus/ground_truth/pose"; then
      echo "[demo] still bringing up; waiting 15s more..."; sleep 15
    fi
    echo "[demo] opening the DRONE-CAMERA view (rqt) -- the corridor as the drone sees it,"
    echo "[demo] moving = the drone flying through the warehouse. (Gazebo window is the 3D world;"
    echo "[demo] mouse-drag to orbit, scroll to zoom -- the corridor is enclosed so default view is the box.)"
    ros2 run rqt_image_view rqt_image_view /argus/cam0/image_raw > /dev/null 2>&1 &
    PIDS="$PIDS $!"
    sleep 4
    echo "[demo] flying the drone down the corridor (watch the rqt camera window). Ctrl-C to stop."
    python3 "$WS/scripts/fly_shuttle.py" --laps 1 --leg-m 16 --speed 0.5
    echo "[demo] flight done; sim still up. Ctrl-C to close."
    sleep 600
    ;;

  rviz)
    echo "[demo] opening RViz (VIO path = green, loop closures = orange)..."
    rviz2 -d "$WS/install/argus_bringup/share/argus_bringup/rviz/argus_demo.rviz" > /dev/null 2>&1 &
    PIDS="$PIDS $!"
    sleep 5
    echo "[demo] replaying the loop run -- VIO trajectory builds + loop closure fires (orange)."
    echo "[demo] watch ~2-3 min then Ctrl-C. (RVIZRATE=${RVIZRATE:-6} speeds the replay.)"
    ros2 bag play "$WS/data/bags/vio_eval_shuttle2" --rate "${RVIZRATE:-6.0}" --clock
    echo "[demo] replay done; trajectory stays up. Ctrl-C to close."
    sleep 600
    ;;

  map)
    if [ ! -d "$WS/data/bags/map_demo" ]; then
      echo "[demo] map bag missing -- bake it once: bash scripts/make_map_demo_bag.sh"
      trap - INT TERM EXIT; exit 1
    fi
    echo "[demo] publishing static TF so RViz fixed frame 'world' exists (bags have no /tf)..."
    ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id vio_origin \
      --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 > /dev/null 2>&1 &
    PIDS="$PIDS $!"
    sleep 2
    echo "[demo] RViz: trajectory (green) + warehouse POINT-CLOUD MAP"
    echo "[demo]   (yellow = current features in view, cyan = accumulated 3D map)..."
    rviz2 -d "$WS/src/argus_bringup/rviz/argus_map.rviz" > /dev/null 2>&1 &
    PIDS="$PIDS $!"
    sleep 5
    echo "[demo] replaying the baked VIO map (looping) -> watch trajectory + point cloud build. Ctrl-C to stop."
    ros2 bag play "$WS/data/bags/map_demo" --clock --loop --rate "${MAPRATE:-3.0}"
    ;;

  *)
    echo "usage: bash scripts/demo.sh {fly|map|rviz|superpoint}"
    echo "  fly         live Gazebo + drone camera view (drone flying the corridor)"
    echo "  map         RViz: VIO trajectory + warehouse point-cloud map (from extracted features)"
    echo "  rviz        RViz: VIO trajectory + loop-closure path building"
    echo "  superpoint  SuperPoint features overlaid on corridor video (rqt window)"
    trap - INT TERM EXIT
    ;;
esac
