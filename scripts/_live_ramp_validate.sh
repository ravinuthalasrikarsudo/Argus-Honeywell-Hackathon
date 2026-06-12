#!/usr/bin/env bash
# Full live validation: record a FRESH ramped lit bag (smooth accel -> clean VINS
# gravity init), replay it mt0@rate0.2, eval drift. Target <1.5% on live data.
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/argus/install/setup.bash 2>/dev/null
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
cd ~/argus

echo "===== 1/3 record fresh ramped lit bag ====="
RAMP=2.0 bash scripts/record_baseline_bag.sh ~/argus/data/bags/baseline_ramp_day6 2>&1 | tail -2

echo "===== 2/3 replay through VINS (mt0, rate0.2) ====="
RATE=0.2 bash scripts/run_vio_offline.sh ~/argus/data/bags/baseline_ramp_day6 ~/argus/data/bags/vio_eval_ramp_day6 2>&1 | tail -1

echo "===== 3/3 eval drift (opt topic, skip2 max24) ====="
~/.venvs/argus-eval/bin/python scripts/run_eval.py --bag ~/argus/data/bags/vio_eval_ramp_day6 --run-id ramp_day6 \
  --vio-topic /argus/vio/odom_optimized --skip-start-m 2.0 --max-dist-m 24.0 2>&1 \
  | grep -iE "path_length|ate_rmse|drift_pct_ate|drift_pct_final|kitti_drift_pct_mean"
echo "===== LIVE RAMP VALIDATE DONE ====="
