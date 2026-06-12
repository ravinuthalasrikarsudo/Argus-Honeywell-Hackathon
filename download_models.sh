#!/usr/bin/env bash
# ARGUS :: download_models.sh  -- fetch the SuperPoint / LightGlue ONNX weights.
# Weights are NOT committed (>5 MB); this script reproduces them. Source:
# fabio-sim/LightGlue-ONNX release v0.1.3 (dynamic-shape, symbolic-shape-inferred).
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE="https://github.com/fabio-sim/LightGlue-ONNX/releases/download/v0.1.3"
echo "[models] -> $DIR"
curl -fSL -o "$DIR/superpoint_1024.onnx"               "$BASE/superpoint_1024.onnx"
curl -fSL -o "$DIR/superpoint_lightglue_end2end.onnx"  "$BASE/superpoint_lightglue_end2end.onnx"
echo "[models] done:"
ls -la "$DIR"/*.onnx
