#!/usr/bin/env bash
#
# ARGUS Day-1 :: view.sh
# Opens a live view of an ARGUS camera in rqt-image-view. Needs a running sim +
# bridge and a GUI (WSLg). Defaults to the left/reference camera (cam0).
#
# Usage:  scripts/view.sh [ros_image_topic]
#   scripts/view.sh                          # /argus/cam0/image_raw
#   scripts/view.sh /argus/cam1/image_raw    # right camera
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh"

TOPIC="${1:-/argus/cam0/image_raw}"
echo "[view] rqt-image-view on ${TOPIC}"
exec ros2 run rqt_image_view rqt_image_view "${TOPIC}"
