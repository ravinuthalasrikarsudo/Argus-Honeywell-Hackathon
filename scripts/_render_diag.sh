#!/usr/bin/env bash
# Render-lighting diagnostic: launch argus_sim each mode, probe cam0 mean brightness.
# Isolates headless(-s, EGL offscreen) vs GUI(WSLg GLX) sensor lighting.
set +e
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/argus/install/setup.bash 2>/dev/null
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

run_mode() {
  local mode="$1"
  echo "===== MODE headless:=${mode} ====="
  bash ~/argus/scripts/_killsim.sh >/dev/null 2>&1
  sleep 2
  ros2 launch argus_bringup argus_sim.launch.py headless:=${mode} > /tmp/launch_${mode}.log 2>&1 &
  local lpid=$!
  sleep 22
  python3 ~/argus/scripts/_cam_brightness_live.py
  kill -INT ${lpid} 2>/dev/null
  sleep 3
  bash ~/argus/scripts/_killsim.sh >/dev/null 2>&1
  sleep 2
}

run_mode true
run_mode false
echo "===== DONE ====="
