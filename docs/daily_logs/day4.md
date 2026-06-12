# ARGUS — Day 4 log

**Pillar 3: VIO health monitor + recovery, Scenario D (lights-off), dashboard.**

Goal: stand up `argus_health` — a watchdog that reports an `argus_msgs/VIOHealth`
status from the live VINS-Fusion output and raises a recovery signal when tracking
fails — validate it on a lights-off scenario, and surface it on a dashboard.

---

## What was built

* **`src/argus_health/`** (ament_python) — `health_monitor.py`. A standalone node
  that derives the full `VIOHealth` schema from VINS's *public* `/argus/vio/*`
  topics (no third_party patch). Pure rclpy → runs under the normal `ros2 run`.
  - **num_tracked/inlier_features** ← `point_cloud` size (window inliers), median-5
    smoothed (the per-keyframe count is a noisy step function).
  - **estimated_drift_rate / position_covariance_trace** ← growth of the
    imu_propagate-vs-optimized position divergence (≈0 healthy, climbs when the
    estimator falls back to IMU-only dead-reckoning).
  - **avg_parallax** ← focal·speed·keyframe_dt / mean_depth (report-only proxy).
  - **imu_excitation_ok** ← VIO speed (primary; the Day-1 IMU is noise-free so a
    constant-velocity cruise reads zero accel/gyro variance) + IMU variance.
  - **liveness** ← WALL-CLOCK age of the high-rate `/argus/vio/odom`. The sim runs
    at RTF<1 and VINS lags several seconds in *sim* time under load, so a sim-time
    staleness test false-trips LOST on a working-but-lagging estimator.
  - **status** INITIALIZING / NOMINAL / DEGRADED / LOST with hysteresis (LOST must
    persist 0.4 s to engage recovery; non-LOST 1.5 s to clear — de-bounces the
    flicker when features hover at the threshold in low-texture zones).
  - **recovery**: on sustained LOST, raise `/argus/health/recovery_active` (+ a
    `/argus/health/recovery_event` log) and count activations. A full
    return-to-pose maneuver is planner territory (Pillar 4, cut); detect + flag +
    (optional) zero-cmd_vel hold + auto-resume is the realistic Pillar-3 behaviour.
  - Thresholds (`feat_nominal 80 / feat_degraded 40 / feat_lost 5`) calibrated
    against a live VINS run on `baseline_ABC` (inlier median ~84).
  - pubs `/argus/vio/health`, `/argus/health/recovery_active` (Bool),
    `/argus/health/recovery_event` (String); launch arg `enable_recovery` selects
    the C1/C3 ablation cell.

* **Scripts** (`scripts/`): `fly_scenario_D.py` (health-aware blackout flight,
  bounded-hold deadlock breaker), `blackout.sh` (gz light_config kill/restore, dim
  not pitch-black), `run_scenario_D.sh` (live), `record_scenario_D.sh` (clean
  sensor capture), `replay_scenario_D.sh` (offline VINS+health), `analyze_scenario_D.py`
  (ablation + timeline plot), `make_scenario_D_synth.py` (synthetic lights-off),
  `build_dashboard.py` (self-contained HTML dashboard), plus dev probes
  (`_health_selftest.py`, `_health_sniff.py`, `_bag_brightness.py`, `_killsim.sh`).

---

## Validation

* **State-machine self-test** (`_health_selftest.py`, no sim): synthetic timeline
  INITIALIZING → NOMINAL → LOST → recovery → NOMINAL, **8/8 PASS**.
* **Live-VINS smoke** (replay `baseline_ABC` → VINS → health): **83% NOMINAL /
  14% DEGRADED / 0.4% LOST** — the target shape for a normal lit flight (DEGRADED
  confined to the blank-wall Zone B, ~no false LOST). Confirms real topic/QoS
  wiring and calibrates the thresholds.

## Scenario D — lights-off

A forward traverse through A→B→C; partway into Zone B the lights cut, the stereo
cameras go dark, feature tracking starves, VINS degrades. The monitor must flip
NOMINAL→LOST over the blackout and (C3) raise recovery.

* **Mechanism.** Built the live gz path (`blackout.sh` kills the 6 ceiling lights +
  the directional fill; `run_scenario_D.sh` flies + records). **The headless ogre2
  render regressed mid-day to unlit frames** (cam0 mean brightness ~0.5 vs ~190 on
  the Day-2 bag; survived a WSL restart → not a transient GPU-context issue), which
  starves VINS regardless of the lights. So the evaluable artifact uses **synthetic
  darkening**: `make_scenario_D_synth.py` darkens the cam0/cam1 payloads of the
  known-good lit `baseline_ABC` bag over the Zone-B window (x∈[10,14], factor 0.03)
  — the same physical effect on the estimator, deterministic, render-independent.
* **Result (offline replay).** Clean arc: **NOMINAL through lit Zone A (~100
  inliers) → LOST the moment the cameras go dark (~0 inliers) → recover** when the
  lights return. VIO dead-reckons ~89 m through the 12 s blackout (no loop closure
  to correct). See `data/eval/scenario_D/scenario_D_health.png`.

### Lights-off recovery ablation (offline, same VIO — deterministic replay)

| metric | C3 (recovery on) | C1 (recovery off) |
|---|---|---|
| NOMINAL % | 33.2 | 33.7 |
| LOST % | 33.0 | 33.3 |
| time-in-LOST (s) | 64.7 | 64.8 |
| **recovery activations** | **4** | **0** |
| time recovery flagged (s) | 75.1 | 0.0 |
| VIO max drift (m) | 89.3 | 87.6 |

C3 detects the blackout and raises recovery 4×; C1 only reports LOST. (Offline
replay cannot actuate the hold, so the trajectory is identical; the live runs
demonstrated the drone-hold action before the render regressed.)

## Dashboard

`build_dashboard.py` → `data/eval/argus_dashboard.html` — a **self-contained**
(no server / install / CDN) HTML dashboard: KPI cards, the Scenario-D health
timeline, the C1-vs-C3 recovery ablation table, and the cross-scenario drift gate
(A/B/C vs 1.5%). Streamlit was the original plan but pip is SSL-blocked here and a
static bundle is more portable for a demo.

---

## Key debugging

* **`ROS_LOCALHOST_ONLY=1` breaks VINS stereo replay** — forces Cyclone onto `lo`
  (no multicast) → drops cam0 frames → VINS throws every cam1 (`throw img1`), never
  initialises. Never export it for VINS runs (the known-good offline scripts omit
  it). 3× reproduced.
* **Sim-time vs wall-time liveness** — keying liveness off message age in *sim*
  time false-trips LOST when VINS lags under load; switched to wall-clock.
* **Headless render regression** — fresh sim renders unlit (geometry present, ~0
  brightness), survived a WSL restart. Worked around with synthetic darkening;
  needs a host-side GPU/driver look for live demos.

## Deferred / next

* SuperPoint → VINS integration (ablation C2) — the Day-3 standalone front-end
  feeding VINS's feature tracker (deep third_party C++).
* Live ogre2 render fix (GPU/driver) so the live gz-blackout demo renders lit.
* Loop closure on Scenario D to correct the ~89 m blackout drift.
