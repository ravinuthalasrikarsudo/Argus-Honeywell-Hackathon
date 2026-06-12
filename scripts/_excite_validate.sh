#!/usr/bin/env bash
# Day-6 init-fix attempt: excitation-preroll record -> mt0@0.15 replay -> eval.
# Target: flat-Z trajectory, drift <1.5%.
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/argus/install/setup.bash 2>/dev/null
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
cd ~/argus

echo "===== 1/3 record excitation-preroll bag (15Hz cams) ====="
bash scripts/record_excite_bag.sh ~/argus/data/bags/excite_day6 2>&1 | tail -2
echo "--- bag rate ---"
ros2 bag info ~/argus/data/bags/excite_day6 2>/dev/null | grep -iE "duration|cam0/image|imu"

echo "===== 2/3 replay mt0@0.15 ====="
bash scripts/run_vio_offline.sh ~/argus/data/bags/excite_day6 ~/argus/data/bags/vio_eval_excite_day6 2>&1 | tail -1

echo "===== 3/3 eval (opt topic, skip3 max24) ====="
~/.venvs/argus-eval/bin/python scripts/run_eval.py --bag ~/argus/data/bags/vio_eval_excite_day6 --run-id excite_day6 \
  --vio-topic /argus/vio/odom_optimized --skip-start-m 3.0 --max-dist-m 24.0 2>&1 \
  | grep -iE "n_poses_synced|path_length|ate_rmse|drift_pct_ate|drift_pct_final|kitti_drift_pct_mean"
echo "===== EXCITE VALIDATE DONE ====="
