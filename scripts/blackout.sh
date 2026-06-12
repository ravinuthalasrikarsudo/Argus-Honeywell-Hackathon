#!/usr/bin/env bash
# ARGUS Day-4 :: blackout.sh
#
# Turn ALL warehouse_corridor lights off (or back on) at runtime via the gz
# light_config service -- the Scenario D "lights-off" event. Kills the 6 ceiling
# point lights AND the directional fill (the fill alone keeps the scene visible,
# so it must go too for the stereo cameras to actually go dark). gz-transport
# only, no ROS dependency (callable as a subprocess from the flight node).
#
# Usage:  blackout.sh off|on [world_name]
set -u

ACTION="${1:-off}"
WORLD="${2:-warehouse_corridor}"
SVC="/world/${WORLD}/light_config"
SPEC='specular: { r: 0.05, g: 0.05, b: 0.05, a: 1.0 }'

set_light() {
  gz service -s "$SVC" \
    --reqtype gz.msgs.Light --reptype gz.msgs.Boolean \
    --timeout 400 --req "$1" >/dev/null 2>&1
}

# Blackout level: a DIM, not pitch-black, so feature tracking craters (-> LOST)
# while VINS still survives and can re-acquire when lights return. Total darkness
# (0 features) sends Ceres into a NaN blow-up -> km-scale divergence that never
# recovers, which masks the recover-to-NOMINAL half of the demo.
OFF_DIFF="${OFF_DIFF:-0.02}"
OFF_INTEN="${OFF_INTEN:-0.02}"

# emit NAME TYPE POSE EXTRA DR DG DB INTEN
#   ON  -> use the light's native diffuse + intensity
#   off -> dim diffuse + dim intensity
emit() {
  local name=$1 type=$2 pose=$3 extra=$4 dr=$5 dg=$6 db=$7 inten=$8
  if [ "$ACTION" = "off" ]; then dr=$OFF_DIFF; dg=$OFF_DIFF; db=$OFF_DIFF; inten=$OFF_INTEN; fi
  set_light "name: \"$name\", type: $type, pose: { $pose }, diffuse: { r: $dr, g: $dg, b: $db, a: 1.0 }, $SPEC, $extra cast_shadows: false, intensity: $inten"
}

ATTEN='range: 14.0, attenuation_constant: 0.3, attenuation_linear: 0.08, attenuation_quadratic: 0.01,'

# Directional fill (note: needs a direction, no attenuation).
emit fill           DIRECTIONAL "position: { x: 15, y: 0, z: 6 }"  "direction: { x: 0.2, y: 0.15, z: -1 }," 0.35 0.35 0.35 0.5
# 6 ceiling point lights.
emit light_a1       POINT "position: { x: 2.5,  y: 0, z: 2.8 }" "$ATTEN" 0.95 0.93 0.85 1.0
emit light_a2       POINT "position: { x: 7.5,  y: 0, z: 2.8 }" "$ATTEN" 0.95 0.93 0.85 1.0
emit light_b1       POINT "position: { x: 12.5, y: 0, z: 2.8 }" "$ATTEN" 0.95 0.93 0.85 1.0
emit light_b_flicker POINT "position: { x: 17.5, y: 0, z: 2.8 }" "$ATTEN" 0.95 0.93 0.85 1.0
emit light_c1       POINT "position: { x: 22.5, y: 0, z: 2.8 }" "$ATTEN" 0.95 0.93 0.85 1.0
emit light_c2       POINT "position: { x: 27.5, y: 0, z: 2.8 }" "$ATTEN" 0.95 0.93 0.85 1.0

echo "[blackout] lights -> ${ACTION} (world=${WORLD})"
