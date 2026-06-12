# ARGUS — Day 5 log

**Pillar 1 (cont.): SuperPoint → VINS front-end integration + C1-vs-C2 ablation.**

Goal: take the Day-3 standalone SuperPoint detector and actually drive VINS-Fusion's
feature front-end with it (ablation cell **C2**), then measure it head-to-head against
the Day-3 KLT/Harris baseline (**C1**) on the same scenarios. The hypothesis: learned
features survive the low-texture Zone-B walls better than `goodFeaturesToTrack`, so C2
should win on the hard scenes.

**Outcome: hypothesis rejected.** The integration works end-to-end (SuperPoint feeds
VINS continuously, `matched=6493 fallback=7` on the shuttle), but C2 is **worse than C1
on every scenario** and **diverges catastrophically on the long shuttle**. KLT/Harris
(C1) remains the production front-end. This is a clean, rigorous **negative result** —
documented below with the root-cause diagnosis.

---

## What was built

* **SuperPoint → VINS C++ integration** in `third_party/VINS-Fusion-ROS2/vins/`
  (gitignored port; patch must be re-applied if re-cloned — see CONTRACT deviations):
  - `estimator/parameters.{h,cpp}`: new `use_superpoint` config flag → `USE_SUPERPOINT`
    (default 0 → C1 behaviour byte-for-byte unchanged).
  - `rosNodeTest.cpp`: subscribes `/argus/vio/keypoints` (RELIABLE QoS, dev #7),
    buffers `PointCloud2 (u,v,score)` by stamp, and just before each `inputImage()`
    hands the stamp-matched detections to the tracker (`feedSuperpoint()`), with a
    bounded 0.5 s wait for the async SuperPoint node + a match/fallback tally.
  - `featureTracker/feature_tracker.cpp`: `setExternalKeypoints()` /
    `selectExternalKeypoints()` — when `USE_SUPERPOINT` and this frame's detections
    arrived, **new** features are seeded from the learned detector instead of
    `goodFeaturesToTrack`; the LK temporal tracker, IDs, stereo matching, RANSAC and
    IMU pipeline are **unchanged**. Selection respects the `setMask()` MIN_DIST
    exclusion discs and enforces MIN_DIST spacing between picks (= Harris's masked +
    spaced behaviour, seeded by SuperPoint, score-sorted).
* **`src/argus_vio/config/argus_stereo_imu_superpoint_config.yaml`** — identical to the
  C1 config except `use_superpoint: 1`. Keeps the ablation apples-to-apples (same
  intrinsics, extrinsics, IMU noise, solver + keyframe params; only the feature source
  differs).
* **Scripts:** `run_vio_superpoint_loop_offline.sh` (SuperPoint node + VINS +
  loop_fusion offline pass on the shuttle → records GT + `odom_optimized` + `odom_loop`),
  `run_day5_evals.sh` (C2 eval matrix into `data/eval/C2_superpoint/`, slice-for-slice
  identical to `run_day3_evals.sh`), `compare_c1_c2.py` (1:1 diff table + trajectory
  overlays + ATE bar chart).

---

## Result — C1 (KLT/Harris) vs C2 (SuperPoint)

Matched slices (`data/eval/compare_c1_c2/summary.md`). ATE drift %, lower = better:

| scenario (matched slice) | C1 KLT | C2 SuperPoint | verdict |
|---|---:|---:|---|
| A — baseline straight, 11 m | **3.92 %** | 8.13 % | C2 ~2× worse |
| B — Zone-B blank walls (SP's expected payoff) | **2.51 %** | 2.52 % | tied — no payoff |
| C — shuttle, 92 m (before loop) | **11.08 %** | 1190.6 % | C2 **diverged** |
| C — shuttle, 92 m (after loop) | **11.08 %** | 1192.9 % | loop can't rescue |

Reference (unchanged Day-3 headlines, different slices): C1 Scenario A = 0.73 % over
24 m; C1 C6 shuttle 2.18 % → 1.31 %. The C2 Scenario-A 15 m shuttle-leg slice = 27.4 %.

**SuperPoint integration is live, not a fallback:** the run tally is
`[C2] superpoint frames matched=6493 fallback=7` — SuperPoint drove ~99.9 % of frames;
the result is genuinely C2, not KLT in disguise.

## Root-cause diagnosis (why C2 diverges)

The shuttle estimate is **exploded from the very first recorded pose**, not progressively
drifting:

* First VIO pose (t≈5.4 s): `x=535.8, y=-556.8` while ground truth is `x=1.5, y=0`
  (drone still near spawn). Trajectory ranges out to `x=-2074, y=+983` over a path whose
  GT stays within `x∈[1.5, 17.2], y=0`.
* KITTI segment drift is uniformly ~3600–4150 % at **all** window lengths including 5 m
  → the estimate is wrong at every scale, i.e. **VINS initialisation / triangulation
  failed and the optimiser blew up**, it did not slowly accumulate error.

This is **not a wiring bug** — `setExternalKeypoints` / `selectExternalKeypoints` and the
tracker hook were audited and are correct (mask-respecting, MIN_DIST-spaced, LK still
tracks temporally; SuperPoint coords are published at native 1280×720 and rescaled if
inferred downscaled). The problem is **architectural**:

> SuperPoint detections are optimised for **descriptor-matching repeatability**, not for
> **Lucas–Kanade optical-flow trackability**. Harris `goodFeaturesToTrack` deliberately
> selects points with strong bidirectional gradients precisely *because* LK tracks them
> well. Feeding SuperPoint points into the same LK flow tracker yields weaker frame-to-
> frame tracks and poorly-conditioned stereo triangulation → fragile init. On the easy
> straight run it survives but drifts ~2× worse; on the longer shuttle (with motion
> reversals) the init explodes.

The **correct** way to exploit SuperPoint in VINS is a learned **descriptor-matching**
front-end (SuperPoint + SuperGlue / LightGlue) that replaces LK flow entirely — a major
estimator rework, out of scope for this hackathon. Logged as future work.

## Decision

**KLT/Harris (C1) stays the ARGUS production front-end.** Pillar 1's headline numbers
(Scenario A 0.73 % drift, loop closure 2.18 %→1.31 %) are all C1 and stand. C2 is
retained in-tree behind `use_superpoint:0` (off by default → zero impact on C1) as a
documented ablation and a starting point for a future descriptor-matching front-end.

---

## Key debugging / gotchas

* **Recorder OOM corrupted the first shuttle capture** (7.4 GB RAM). `ros2 bag record`
  of the full topic set (`/clock` + cumulative `/path`,`/base_path`,`/loop_closures`
  Path msgs + 250 Hz `/argus/vio/odom`) blew the recorder buffer → SIGKILL →
  `database disk image is malformed` (unrecoverable; no `sqlite3` recover available).
  **Fix:** trimmed `run_vio_superpoint_loop_offline.sh`'s record to the 3 eval-critical
  topics (`/argus/ground_truth/pose`, `/argus/vio/odom_optimized`, `/argus/vio/odom_loop`)
  — clean teardown, `metadata.yaml` written, 7.6 MB bag. `run_eval.py` reads message
  header stamps, so `/clock` is not needed for offline eval.
* The `[ERROR] process has died ... exit code -9` lines in the run log are the script's
  own teardown `pkill -9 -f vins_node` (expected), **not** a mid-run OOM.
* Loop closure with the SP front-end fired (7 pose-graph corrections) but could not
  rescue an already-diverged trajectory (1190.6 % → 1192.9 %).

## Artifacts

* `data/eval/C2_superpoint/<scenario>/metrics.json` — C2 eval matrix.
* `data/eval/C1_klt/{scenario_A_baselinerun,scenario_C_beforeloop,scenario_C_afterloop}`
  — C1 cells regenerated at matched slices for the head-to-head (Day-3 headline
  `scenario_A` 24 m run left untouched).
* `data/eval/compare_c1_c2/{summary.md, traj_*.png, drift_bar_c1_c2.png}` — comparison.

## Deferred / next

* Learned **descriptor-matching** front-end (SuperPoint + LightGlue) replacing LK flow —
  the architecturally-correct way to test the learned-features hypothesis.
* Live ogre2 render fix (WSLg/iGPU) so live sim renders lit again (carried from Day 4).
