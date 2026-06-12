#!/usr/bin/env bash
#
# ARGUS Day-1 :: flicker.sh
# Convenience wrapper that drives the Zone-B ceiling fixture while a sim is
# running -- without needing to know the install path of the packaged
# flicker_light.sh. Run this in a SECOND terminal after run.sh is up. Arguments
# pass through (world name, light name). Ctrl-C restores the light to bright.
#
# Usage:  scripts/flicker.sh [world_name] [light_name]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh"

# Prefer the installed copy; fall back to the source tree if not built yet.
flicker="$(ros2 pkg prefix argus_sim 2>/dev/null)/share/argus_sim/scripts/flicker_light.sh"
if [ ! -x "${flicker}" ]; then
  flicker="${ARGUS_WS}/src/argus_sim/scripts/flicker_light.sh"
fi

if [ ! -f "${flicker}" ]; then
  echo "[flicker] ERROR: flicker_light.sh not found (build the workspace first)." >&2
  exit 1
fi

echo "[flicker] using ${flicker}"
exec bash "${flicker}" "$@"
