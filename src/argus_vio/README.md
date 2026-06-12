# argus_vio — stereo-inertial VIO configuration & launch (Pillar 2)

The VIO deliverable: wires the vendored, locally-patched
[VINS-Fusion-ROS2](../../third_party/VINS-Fusion-ROS2) stereo-inertial
estimator onto the frozen ARGUS topic contract.

| File | Role |
|------|------|
| `config/argus_stereo_imu_config.yaml` | Production estimator config (KLT/Harris front-end, `multiple_thread: 0` for determinism, EuRoC-calibrated IMU noise, `g_norm: 9.8`) |
| `config/argus_stereo_imu_superpoint_config.yaml` | Ablation C2 config — identical except `use_superpoint: 1` |
| `config/argus_cam{0,1}_pinhole.yaml` | Camera intrinsics (1280×720, fx=fy=640, 0.12 m baseline) |
| `launch/argus_vio.launch.py` | vins_node remapped onto `/argus/vio/*` |
| `launch/argus_vio_loop.launch.py` | vins_node + loop_fusion (DBoW2 pose graph) → `/argus/vio/odom_loop` |

Outputs: `/argus/vio/odom` (250 Hz IMU-propagated), `/argus/vio/odom_optimized`
(keyframe-rate, **eval this**), `/argus/vio/odom_loop` (loop-corrected).

Run offline (deterministic, decoupled from sim RTF):

```bash
bash scripts/run_vio_offline.sh <sensor_bag> <eval_bag>        # VIO only
bash scripts/run_vio_loop_offline.sh <sensor_bag> <eval_bag>   # VIO + loop closure
```

See root README §4 for the pipeline description and §8 for results.
