# argus_msgs — custom interface definitions

Two messages, frozen by the Day-1 contract (`docs/CONTRACT.md`):

- **`msg/VIOHealth.msg`** — the health monitor's composite state:
  `status` (0=INITIALIZING 1=NOMINAL 2=DEGRADED 3=LOST), `confidence`,
  `num_inlier_features`, `avg_parallax`, `estimated_drift_rate`,
  `position_covariance_trace`, `imu_excitation_ok`, `processing_latency_ms`.
- **`msg/UncertaintyMap.msg`** — header + per-voxel uncertainty array for the
  occupancy mapper.

Verify they resolve: `ros2 interface show argus_msgs/msg/VIOHealth`
(acceptance point 2 checks this automatically).
