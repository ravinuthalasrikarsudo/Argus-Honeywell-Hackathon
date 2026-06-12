#!/usr/bin/env bash
# Kill ALL stray sim/render/light processes (pattern-free of this script's own name
# so pkill -f cannot self-match). Use between sim runs to free the iGPU.
set +e
pkill -9 -f "gz sim"
pkill -9 -f "gz-sim"
pkill -9 -f "ros_gz"
pkill -9 -x ruby
pkill -9 -f "parameter_bridge"
pkill -9 -f "camera_info_patch"
pkill -9 -f "blackout.sh"
pkill -9 -f "flicker_light"
pkill -9 -f "vins_node"
pkill -9 -f "health_monitor"
sleep 3
echo "remaining gz/ruby: $(pgrep -f 'gz-sim|gz sim' | wc -l) / $(pgrep -x ruby | wc -l)"
