#!/usr/bin/env bash
#
# ARGUS Day-1 :: flicker_light.sh
# Flickers the Zone-B ceiling fixture (light_b_flicker) in warehouse_corridor while
# the sim is running, via the gz light_config service. gz-harmonic ships no autonomous
# flicker plugin, so the world declares the light static-bright and this driver toggles
# it at runtime (gz-transport only -- no ROS dependency).
#
# Usage:  flicker_light.sh [world_name] [light_name]
# Stop:   Ctrl-C  (light is restored to bright on exit).
#
set -u

WORLD="${1:-warehouse_corridor}"
LIGHT="${2:-light_b_flicker}"
SVC="/world/${WORLD}/light_config"

# Shared, fixed light parameters (must match the SDF declaration).
POSE='pose: { position: { x: 17.5, y: 0.0, z: 2.8 } }'
ATTEN='range: 14.0, attenuation_constant: 0.3, attenuation_linear: 0.08, attenuation_quadratic: 0.01'
SPEC='specular: { r: 0.2, g: 0.2, b: 0.2, a: 1.0 }'

req_bright="name: \"${LIGHT}\", type: POINT, ${POSE}, diffuse: { r: 0.95, g: 0.93, b: 0.85, a: 1.0 }, ${SPEC}, ${ATTEN}, cast_shadows: false, intensity: 1.0"
req_dim="name: \"${LIGHT}\", type: POINT, ${POSE}, diffuse: { r: 0.05, g: 0.05, b: 0.05, a: 1.0 }, ${SPEC}, ${ATTEN}, cast_shadows: false, intensity: 0.05"

set_light() {
  gz service -s "${SVC}" \
    --reqtype gz.msgs.Light --reptype gz.msgs.Boolean \
    --timeout 300 --req "$1" >/dev/null 2>&1
}

restore() { set_light "$req_bright"; echo "[flicker] restored ${LIGHT} to bright."; exit 0; }
trap restore INT TERM

echo "[flicker] driving ${LIGHT} via ${SVC}  (Ctrl-C to stop)"
# Irregular cadence -> looks like a failing fluorescent tube, and stresses VIO.
while true; do
  set_light "$req_dim";    sleep 0.07
  set_light "$req_bright"; sleep 0.18
  set_light "$req_dim";    sleep 0.05
  set_light "$req_bright"; sleep 0.45
done
