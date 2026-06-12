# Day 2 — VIO Baseline and Eval Harness

**Date:** 2026-06-03
**Objective (master plan §8 Day 2):** VINS-Fusion stereo-inertial running on the
ARGUS sim, producing a trajectory; an `evo`-based eval harness producing
publication-quality plots + drift numbers.

**Result: ALL 4 acceptance criteria MET.** VIO runs end-to-end, trajectory +
drift reported, harness validated on the baseline bag.

---

## Acceptance criteria

| # | Criterion | Result |
|---|-----------|--------|
| 1 | `/argus/vio/odom` ≥ 20 Hz during sim | **✓ ~250 Hz** (8390 msgs / 33.6 s sim; high-rate IMU-propagated odom) |
| 2 | Trajectory plot vs ground truth (any quality) | **✓** `data/eval/day2_baseline/trajectory.png` (XY + XZ, VIO coloured by APE) |
| 3 | Drift % computed + reported (target deferred to D6) | **✓** 20.1% (ATE) / 31.6% (final) over 23.6 m |
| 4 | Eval harness runs e2e on the baseline bag | **✓** `run_eval.py` produces plots + `metrics.json` |

Drift is high — expected. Master plan: Day-2 wants drift *measured, any quality*;
Day-3 tunes Scenario A to <5%, D6 to <1.5%. Root cause of the 20% baseline:
low IMU excitation (constant-velocity forward flight), 13 Hz iGPU-limited cameras,
and untuned IMU noise. These are exactly the Day-3 levers.

---

## Deliverables built

### `src/argus_vio/` (new package, ament_cmake)
- `config/argus_stereo_imu_config.yaml` — VINS config matching the frozen contract:
  topics → `/argus/{imu,cam0,cam1}`, 1280×720, EuRoC IMU noise (Day-1 spec),
  `g_norm=9.8` (sim gravity), `estimate_extrinsic:0` (exact extrinsics).
- `config/argus_cam{0,1}_pinhole.yaml` — fx=fy=640, cx=640, cy=360, zero distortion.
- **Extrinsics** (computed): body(IMU)=FLU, cam optical=RDF →
  `R_body_cam` rows `(0,0,1)/(-1,0,0)/(0,-1,0)`; `t_cam0=(0.10,0.06,0)`,
  `t_cam1=(0.10,-0.06,0)`; baseline 0.12 m, cam0=left=reference (matches contract).
  VINS log confirmed it loaded these exactly.
- `launch/argus_vio.launch.py` — starts `vins_node`, `use_sim_time:=true`, Cyclone
  pinned; remaps onto `/argus/vio/*` schema.

### `third_party/VINS-Fusion-ROS2/` (zinuok port, symlinked into `src/`)
- Cloned `zinuok/VINS-Fusion-ROS2` (master plan named xukuanHIT "or equivalent";
  zinuok is a maintained Humble equivalent). Build `camera_models`+`vins`+`loop_fusion`;
  **`global_fusion` skipped** (GPS fusion — we are GPS-denied).
- **2 patches** (documented as deviations; see below).

### `scripts/`
- `run_eval.py` — evo-based eval. Reads bag via pure-python `rosbags` (no rclpy →
  fully decoupled from ROS runtime, no system-numpy shadowing). SE(3) Umeyama
  align with origin-align fallback on degenerate (straight-line) trajectories;
  APE/RPE with short-trajectory fallback; drift% (ATE + final); paper-style plots.
- `run_ablation.py` — Day-2 skeleton: frozen C1–C5 × A–D grid (master plan §9) +
  working `aggregate` mode (scans `data/eval/*/metrics.json` → table + bar chart).
- `record_baseline_bag.sh` — orchestrates sim + forward flight A→B→C + record
  (closes the unbuilt **Day-1 deliverable #7**). Produces `data/bags/baseline_ABC`.
- `run_vio_offline.sh` — replay baseline bag → VINS → record GT + odom into
  `data/bags/vio_eval`. Offline replay = deterministic, decoupled from sim RTF.

### Environment
- **Ceres 2.1.0** built from source → `/usr/local` (static `libceres.a`).
  Ubuntu 22.04 ships 2.0.0 (too old: no `ceres::Manifold`); 2.2 too new (forces
  C++17, clashes with the port's CXX14). 2.1 = Manifold API + C++14. Colcon pinned
  via `-DCeres_DIR=/usr/local/lib/cmake/Ceres`. Static lib → no clash with apt 2.0.
- **Eval venv** `~/.venvs/argus-eval` (outside workspace): evo 1.36.5 + rosbags +
  numpy 2.2.6, isolated from system numpy 1.21.5 (ROS-safe).

---

## Results

**Baseline sensor bag** `data/bags/baseline_ABC` (5.4 GB, gitignored):
79.6 s wall ≈ 33 s sim (RTF~0.42), 1040 stereo pairs (~13 Hz, iGPU-limited),
8588 IMU (~108 Hz), 2863 GT. Drone flew x=1.5 → ~24 m (zones A→B→C).

**VIO eval** `data/eval/day2_baseline/metrics.json`:
```
path_length      23.6 m
ate_rmse          4.76 m      ate_max   7.46 m
rpe_rmse          0.42 m/m    rpe_max   1.00 m/m
drift_pct_ate     20.1 %      drift_pct_final  31.6 %
odom rate        ~250 Hz (high-rate) ; optimized ~15 Hz
align            origin (Umeyama degenerate on straight-line flight)
```

---

## Decisions / deviations (carry forward)

1. **VINS port = zinuok/VINS-Fusion-ROS2** (Humble-maintained; xukuanHIT equivalent).
2. **Ceres 2.1.0 from source → /usr/local (static).** Pin colcon with `-DCeres_DIR`.
3. **VINS subscription QoS patch** (`vins/src/rosNodeTest.cpp`): IMU + both image
   subs changed `SensorDataQoS()` (BEST_EFFORT) → `QoS(KeepLast(N)).reliable()`.
   BEST_EFFORT subs received **nothing** from `ros2 bag play` / the gz bridge
   (both publish RELIABLE) over Cyclone — endpoints matched, "compatible", but no
   delivery; `ros2 topic hz` saw data while VINS stalled at "waiting for image and
   imu". RELIABLE byte-matches both bag + live bridge.
4. **`/argus/vio/odom` = VINS `imu_propagate`** (IMU-rate ~250 Hz real-time odom) to
   satisfy the 30 Hz schema + downstream (health/planner need high-rate pose). The
   image-rate optimized estimate (~15 Hz, iGPU-capped) is on **`/argus/vio/odom_optimized`**.
5. **global_fusion skipped** (GPS, not applicable).
6. **Offline-replay eval** at `--rate 0.4` (every frame processed; repeatable under RTF<1).

## WSL/build gotchas learned (also in memory)
- `/tmp` is wiped on WSL VM cold-boot (systemd-tmpfiles) → never stage long work in
  `/tmp`; use `/home/vittal`.
- `wsl -- bash <script>` 127'd on PATH; `wsl -- bash -lc '<file>'` (login shell) works.
- colcon default built vins+loop_fusion in parallel at `make -j16` → **OOM**
  (`cc1plus Killed`) on 7.4 GB. Fix: `--executor sequential` + `MAKEFLAGS=-j2`.
- `$()`/`$VAR`-in-loops mangle across host→wsl; author script files instead.

## Deferred (not Day-2 scope)
- Drift tuning to <5% (Day 3, Scenario A); IMU-noise scaling is the #1 lever.
- Named scenarios A/B/C codified in `data/scenarios/` (Day 3).
- Loop closure validation (Day 6); `loop_fusion` is built but not yet exercised.
- Plot aesthetics (squashed equal-aspect XY on straight-line; legend overlap) — D10.
