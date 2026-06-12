#!/usr/bin/env bash
# ARGUS :: run_day3_evals.sh  -- Day-3 baseline (C1 = KLT, no recovery) eval matrix.
#
# Produces the Scenario A / B / C drift numbers + paper plots into the ablation
# dir layout data/eval/<config>/<scenario>/. Reuses two recordings (no re-fly):
#   * vio_eval          = VINS on baseline_ABC straight A->B->C run (Day-2 bag)
#   * vio_eval_shuttle  = VINS+loop_fusion on the no-yaw shuttle (Scenario C)
#
# Scenario A (easy, drift gate)   : clean forward leg, KLT, target < 1.5%.
# Scenario B (hard, blank walls)  : Zone-B slice -- baseline expected to struggle
#                                   (this is the Day-4 SuperPoint payoff scenario).
# Scenario C (loop)               : full shuttle, BEFORE (odom_optimized) vs
#                                   AFTER (odom_loop) pose-graph correction.
set -uo pipefail

WS=/home/vittal/argus
PY=~/.venvs/argus-eval/bin/python
EVAL=$WS/scripts/run_eval.py
OUT=$WS/data/eval/C1_klt
BASE=$WS/data/bags/vio_eval
SHUT=$WS/data/bags/vio_eval_shuttle
cd "$WS"

echo "=================== Scenario A (easy / drift gate) ==================="
# Headline: clean first forward leg of the slow shuttle (0.5 m/s -> dense frames).
$PY "$EVAL" --bag "$SHUT" --vio-topic /argus/vio/odom_optimized \
  --max-dist-m 15 --skip-start-m 2 \
  --out-root "$OUT" --run-id scenario_A
echo
echo "--- Scenario A cross-check: Zone-A slice of the baseline straight run ---"
$PY "$EVAL" --bag "$BASE" --vio-topic /argus/vio/odom_optimized \
  --max-dist-m 11 \
  --out-root "$OUT" --run-id scenario_A_baselinerun || true

echo "=================== Scenario B (hard / blank walls) ==================="
$PY "$EVAL" --bag "$BASE" --vio-topic /argus/vio/odom_optimized \
  --skip-start-m 8 --max-dist-m 11 \
  --out-root "$OUT" --run-id scenario_B || true

echo "=================== Scenario C (loop) -- BEFORE loop closure ==================="
$PY "$EVAL" --bag "$SHUT" --vio-topic /argus/vio/odom_optimized \
  --out-root "$OUT" --run-id scenario_C_beforeloop

echo "=================== Scenario C (loop) -- AFTER loop closure ==================="
$PY "$EVAL" --bag "$SHUT" --vio-topic /argus/vio/odom_loop \
  --out-root "$OUT" --run-id scenario_C_afterloop || true

echo
echo "===== Day-3 eval matrix summary (drift_pct + kitti) ====="
for s in scenario_A scenario_A_baselinerun scenario_B scenario_C_beforeloop scenario_C_afterloop; do
  f="$OUT/$s/metrics.json"
  if [ -f "$f" ]; then
    echo "--- $s"
    grep -E "drift_pct_ate|drift_pct_final|kitti_drift_pct_mean|kitti_drift_pct_by_len|path_length" "$f"
  fi
done
