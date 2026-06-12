# argus_health — VIO health monitor & failure recovery (Pillar 3)

Standalone watchdog over the VIO's *public topics* (no estimator coupling):
state machine `INITIALIZING → NOMINAL ⇄ DEGRADED ⇄ LOST`, with autonomous
recovery (zero-velocity hold) when LOST persists.

Signals derived: inlier feature count (`/argus/vio/point_cloud`), motion-aware
parallax proxy, IMU-vs-optimized drift-rate EMA, IMU excitation, odom staleness,
composite confidence ∈ [0, 1]. Publishes `argus_msgs/VIOHealth` on
`/argus/vio/health` plus `/argus/health/recovery_active`.

```bash
ros2 launch argus_health argus_health.launch.py
python3 scripts/_health_selftest.py   # scripted-timeline self-test, no sim needed
```

Validated in Scenario D (mid-flight Zone-B blackout): status reaches LOST,
recovery engages, and clears on re-lighting — see root README §5 and §8.4.
