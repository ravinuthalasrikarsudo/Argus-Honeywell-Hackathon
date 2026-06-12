#!/usr/bin/env bash
# ARGUS :: run_day5_evals.sh  -- Day-5 C2 (SuperPoint front-end) eval matrix.
#
# Mirrors run_day3_evals.sh slice-for-slice but on the C2 eval bags (VINS driven
# by SuperPoint detections instead of Harris), writing into data/eval/C2_superpoint/.
# Because the slices (--max-dist-m / --skip-start-m) are identical to the C1 run,
# compare_c1_c2.py can diff the two matrices 1:1.
#
#   vio_eval_sp          = VINS+SuperPoint on baseline_ABC straight A->B->C run
#   vio_eval_sp_shuttle  = VINS+SuperPoint on the no-yaw shuttle (Scenario A/C)
#
# Scenario A (easy)  : clean first forward leg of the shuttle.
# Scenario B (hard)  : Zone-B blank-wall slice of the straight run -- THE SuperPoint
#                      payoff scenario (Harris starves on textureless walls).
# Scenario C (loop)  : shuttle BEFORE (odom_optimized) vs AFTER (odom_loop); only
#                      if a SuperPoint loop bag exists.
set -uo pipefail

WS=/home/vittal/argus
PY=~/.venvs/argus-eval/bin/python
EVAL=$WS/scripts/run_eval.py
OUT=$WS/data/eval/C2_superpoint
BASE=$WS/data/bags/vio_eval_sp
SHUT=$WS/data/bags/vio_eval_sp_shuttle
cd "$WS"

run_slice () {  # args: <bag> <vio-topic> <run-id> <extra eval args...>
  local bag="$1"; local topic="$2"; local rid="$3"; shift 3
  if [ ! -d "$bag" ]; then
    echo "[day5] SKIP $rid (bag missing: $bag)"; return 0
  fi
  echo "=================== $rid ==================="
  "$PY" "$EVAL" --bag "$bag" --vio-topic "$topic" --out-root "$OUT" --run-id "$rid" "$@" || true
}

echo "############### Day-5 C2 (SuperPoint) eval matrix ###############"
run_slice "$SHUT" /argus/vio/odom_optimized scenario_A             --max-dist-m 15 --skip-start-m 2
run_slice "$BASE" /argus/vio/odom_optimized scenario_A_baselinerun --max-dist-m 11
run_slice "$BASE" /argus/vio/odom_optimized scenario_B             --skip-start-m 8 --max-dist-m 11
run_slice "$SHUT" /argus/vio/odom_optimized scenario_C_beforeloop
run_slice "$SHUT" /argus/vio/odom_loop      scenario_C_afterloop

echo
echo "===== Day-5 C2 eval matrix summary (drift_pct + kitti) ====="
for s in scenario_A scenario_A_baselinerun scenario_B scenario_C_beforeloop scenario_C_afterloop; do
  f="$OUT/$s/metrics.json"
  if [ -f "$f" ]; then
    echo "--- $s"
    grep -E "drift_pct_ate|drift_pct_final|kitti_drift_pct_mean|kitti_drift_pct_by_len|path_length" "$f"
  fi
done
echo
echo "[day5] next: ~/.venvs/argus-eval/bin/python scripts/compare_c1_c2.py"
