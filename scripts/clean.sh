#!/usr/bin/env bash
#
# ARGUS Day-1 :: clean.sh
# Removes the colcon artifacts (build/ install/ log/) for a clean rebuild. The
# source tree is left untouched.
#
# Usage:  scripts/clean.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARGUS_WS="$(dirname "${SCRIPT_DIR}")"

cd "${ARGUS_WS}"
echo "[clean] removing build/ install/ log/ under ${ARGUS_WS}"
rm -rf build install log
echo "[clean] done. Rebuild with scripts/build.sh"
