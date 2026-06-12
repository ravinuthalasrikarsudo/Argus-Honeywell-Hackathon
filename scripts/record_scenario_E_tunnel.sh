#!/usr/bin/env bash
# ARGUS :: record_scenario_E_tunnel.sh
#
# Records the Scenario E SENSOR bag: one (or more) continuous CCW laps of the
# 202.8 m tunnel_circuit stadium (fly_circuit.py), the DP7 long-distance drift
# gate ("< 1.5 % over 200 m"). The lap ends back at the spawn with a few metres
# of overrun, so the pose graph can close the loop exactly where drift is
# measured. No stops, no reversals, no in-place yaw anywhere in the profile.
#
# Usage:  bash scripts/record_scenario_E_tunnel.sh [BAG_DIR]
# Env: LAPS(1) SPEED(0.8) EXCITE(0) SETTLE_S(14) EXTRA_S(40)
#
# EXCITE=1 is a DOCUMENTED NEGATIVE (day-7): the smooth vertical-sinusoid
# preamble's velocity zero-crossings corrupt VINS gravity init (first pose
# ~12 km out). Production profile is the corridor-proven plain step-start.
set -uo pipefail

WS=/home/vittal/argus
BAG="${1:-$WS/data/bags/scenario_E_tunnel}"
LAPS="${LAPS:-1}"
SPEED="${SPEED:-0.8}"
EXCITE="${EXCITE:-0}"
SETTLE_S="${SETTLE_S:-14}"
EXTRA_S="${EXTRA_S:-40}"   # recorder margin beyond the estimated flight time
LOG="$WS/data/bags/_record_E.log"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
set -u

mkdir -p "$WS/data/bags"
rm -rf "$BAG"

echo "[recE] pre-launch cleanup of stray sim procs..."
bash "$WS/scripts/_killsim.sh" >/dev/null 2>&1
sleep 2

echo "[recE] launching tunnel_circuit sim (headless)..."
ros2 launch argus_bringup argus_sim.launch.py headless:=true world:=tunnel_circuit >"$LOG" 2>&1 &
LAUNCH_PID=$!
echo "[recE] settling ${SETTLE_S}s for gz + bridge + drone spawn..."
sleep "$SETTLE_S"

if ! ros2 topic list 2>/dev/null | grep -q "/argus/ground_truth/pose"; then
  echo "[recE] ERROR: /argus topics not up after settle; see $LOG" >&2
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 4; bash "$WS/scripts/_killsim.sh" >/dev/null 2>&1
  exit 1
fi

# RTF-aware recorder window: flight covers LAPS*202.8+4 m at SPEED (sim s);
# the tunnel world paces itself at RTF 0.5 (recording robustness — see the
# world file), so wall time ~= sim time / 0.45 with margin. fly_circuit exits
# on its own when done; the recorder is SIGINTed right after (no hover tail).
FLIGHT_SIM_S=$(python3 -c "print(int(($LAPS*202.83+4)/$SPEED + 12))")
REC_S=$(python3 -c "print(int($FLIGHT_SIM_S/0.45 + $EXTRA_S))")
# VIO-essential topics only: stereo images dominate the rate; cam_info / lidar /
# rangefinder / cmd_vel are not consumed by the offline VIO+eval pass.
echo "[recE] recording 5 VIO topics for up to ${REC_S}s wall -> $BAG"
timeout -s INT "$REC_S" ros2 bag record -s sqlite3 -o "$BAG" \
  /argus/cam0/image_raw /argus/cam1/image_raw \
  /argus/imu /argus/ground_truth/pose /clock >>"$LOG" 2>&1 &
REC_PID=$!
sleep 2

EXC_FLAG=""
[ "$EXCITE" = "1" ] && EXC_FLAG="--excite"
echo "[recE] flying ${LAPS} lap(s) of 202.83 m at ${SPEED} m/s ${EXC_FLAG}..."
python3 "$WS/scripts/fly_circuit.py" --laps "$LAPS" --speed "$SPEED" $EXC_FLAG

echo "[recE] flight done; stopping recorder..."
kill -INT "$REC_PID" 2>/dev/null || true
wait "$REC_PID" 2>/dev/null || true

echo "[recE] teardown..."
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 4
pkill -INT -f "ros2 launch argus_bringup" 2>/dev/null || true
bash "$WS/scripts/_killsim.sh" >/dev/null 2>&1
sleep 2

if [ -d "$BAG" ] && [ -f "$BAG/metadata.yaml" ]; then
  echo "[recE] DONE. scenario E bag -> $BAG"
  du -sh "$BAG" 2>/dev/null || true
else
  echo "[recE] ERROR: bag not produced or recorder died early; see $LOG" >&2
  exit 1
fi
