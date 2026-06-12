# Day 7 — Ubuntu 26.04 bring-up, texture regression, and the literal 200 m gate

**Date:** 2026-06-10
**Objective:** verify every DP7 deliverable end-to-end on the new Ubuntu 26.04 +
RTX 4050 host, and close the one spec gap the project had never demonstrated:
**VIO drift < 1.5 % over 200 m** (longest prior run: 92.7 m shuttle).

> Status tokens: ✅ fixed/verified · ⚠️ open/limited · ❌ failed experiment.

---

## 1. Environment bring-up on Ubuntu 26.04 — ✅ acceptance 11/11

ROS 2 Humble has no 26.04 packages; the stack runs in the `argus:humble`
container (`docker/run.sh` / `docker/demo.sh`), GPU passthrough verified
(`nvidia-smi` inside the container sees the RTX 4050). Full acceptance:

```
ros2 run argus_bringup acceptance --full   →  ACCEPTED, 11/11 gated PASS
RTF = 0.902 under full world + sensors (was 0.42 WSL/iGPU, 0.84 WSL day-6)
```

Host-side eval venv (`~/.venvs/argus-eval`, Python 3.14) imports evo/rosbags/
matplotlib cleanly. The record → replay → eval loop was re-validated live
(fresh corridor bag → `run_vio_offline.sh` → `run_eval.py`).

## 2. detail.png texture regression — ✅ found + fixed  ← the big one

The README docs commit (`1dd00af`, June 8) **accidentally deleted
`src/argus_sim/worlds/detail.png`** — the PBR checker+speckle albedo that gives
the walls/floor their trackable detail. `install/` only holds a symlink into
`src/`, so every sim run since June 8 rendered **untextured flat-colour
walls** (ogre2 silently drops a missing albedo map). The Day-1 contract
explicitly freezes this texture; the README "live capture" GIFs and any run
recorded on this host were feature-starved without anyone noticing.

Restored byte-identical from the initial commit (`75f2c6b`). A/B on the same
flight profile (fresh corridor straight, mt0, 0.8 m/s):

| | untextured (regressed) | textured (restored) |
|---|---|---|
| KITTI segment drift (mean) | 5.91 % | **4.03 %** |
| ATE drift % | 7.68 % | 7.57 % |

Relative (KITTI) drift improves immediately; absolute ATE stays dominated by
two other effects, now isolated (below).

## 3. Fresh-run ATE error budget — ✅ root causes isolated

The fresh corridor runs sit at ~7.6 % ATE, far above day-6's 3.4–4.2 %. The
trajectory plot decomposes it:

* **XY tracking is excellent** (±0.05 m over the whole corridor).
* **Pitch-init tilt → Z-ramp**: VIO z ramps ~3.5 m across the run (≈ 7° tilt
  this pass; day-6 measured 1.3°). Same documented gravity-init limitation,
  worse with the step-start + untextured init view.
* **Hover-tail drift**: the recorder kept running ~10 s after the drive ended;
  a stationary, zero-excitation VINS keeps integrating → the synced tail
  poses inflate ATE RMSE and final drift (the "final drift 4.96 m" is almost
  entirely tail). Mission-segment metrics should end at flight end.

Mitigations baked into Scenario E (not config tuning): smooth 6 s vertical-
sinusoid excitation preamble (C¹-continuous — day-6's failed pre-roll had
sharp reversals), recorder SIGINTed the moment the flight script exits, and
near-field texture everywhere at init.

## 4. Scenario E — the literal 200 m drift gate — ✅ designed + flown

**World** `tunnel_circuit.sdf` (generated): 202.83 m stadium tunnel — two 70 m
straights + two r = 10 m semicircular end-caps, 6 × 3.5 m section. The world
IS the experiment design:

* **Segmented 5 m wall panels** re-tile `detail.png` per panel (an SDF box maps
  the albedo once per face — the corridor's single 30 m wall box stretched it
  into invisibility). Checker squares subtend 30–60 px at flight range: ideal
  Harris/KLT scale.
* **Arch ribs every ~10 m** + colour signage every ~12.7 m: near-field
  parallax + locally distinctive DBoW landmarks.
* **End-caps flown, not turned**: wz = v/R = 0.08 rad/s while translating at
  0.8 m/s — yaw with continuous parallax (the U-turn divergence mode is
  geometrically impossible on this course).
* **Closed circuit**: the lap ends 4 m past the spawn → loop_fusion can close
  the loop exactly where drift is measured.

**Flight** `fly_circuit.py`: GT-feedback path follower (GT steers the vehicle
only — the VIO never sees it), kinematic dry-run max lateral error 5 mm/lap.
Live flight matched: e = 0.00 m at lap end, z hold 1.000 m, 206.8 m flown.

**Recording** `record_scenario_E_tunnel.sh` → 28 GB sensor bag (rgb8 1280×720
stereo @ 15 Hz + 250 Hz IMU + GT). Offline replay `run_vio_loop_offline.sh`
at RATE 0.15 (the proven deterministic envelope for mt0).

## 5. ❌ Excitation preamble — failed experiment (iteration 1)

The 6 s smooth vertical-sinusoid preamble **corrupted VINS init outright**: the
first published pose was already ~12 km from the origin, with a wrong world
tilt that integrated a runaway fictitious acceleration (ATE meaningless;
KITTI drift uniformly enormous at every window → wrong from t0, the day-5
"exploded init" signature, not accumulated drift). Tracking after the bogus
init was locally smooth — the damage was entirely in the init window.

Conclusion sharpened from Day 6: it is not the *sharpness* of the reversals —
**any vertical velocity zero-crossings during the init window break the
gravity/velocity SFM init** in this regime. The smooth sinusoid had a zero
crossing every second. `--excite` stays in `fly_circuit.py` as a documented
negative flag (default off); the production Scenario E profile is the
corridor-proven plain step-start.

Bonus finding: with the diverged estimate, loop_fusion produced **0 DBoW
candidates** over the full lap — to be re-assessed on a sane run (the tiled
texture is a perceptual-aliasing risk for appearance-based loop detection;
the colour signage is the intended disambiguator).

## 6. ❌→✅ Recorder OOM on the 14 GB host (iteration 2) — fixed structurally

Iteration 2 (step-start) never produced a bag: at RTF ~0.9 the 5-topic stereo
rgb8 stream writes **~80 MB/s**; with the page cache pre-warmed by the 40 min
iteration-1 replay, writeback stalled, the sim collapsed to RTF ≈ 0.2
mid-flight, and the recorder was killed at ~16 GB — **no `metadata.yaml`**
(the same signature as the day-5 shuttle OOM, different mechanism: this host
has 14 GB RAM, the WSL box had more headroom).

Structural fix, not a retry-and-hope:
* `tunnel_circuit.sdf` now **self-paces at RTF 0.5** (physics step and all
  sensor rates unchanged — sim-time data identical; wall disk rate halved).
* The Scenario E recorder captures **only the 5 topics** the offline VIO +
  eval pass consumes (`cam0/cam1 image_raw, imu, ground_truth/pose, /clock`).
* Replay-rate equivalence: bag play paces by *recorded wall spacing*, so
  `RATE=0.3` on an RTF-0.5 bag delivers frames to VINS at the same wall rate
  as the proven `RATE=0.15` on an RTF-1.0 bag (≈ 2.25 frames/s).
* Pipeline scripts now **gate on `metadata.yaml`** before replaying.

Debug rule added to the playbook: a bag directory without `metadata.yaml`
means the recorder died — check `free -g` and look for the RTF collapse, not
just the recorder log (it ends mid-sentence).

## 7. ❌ Ramped cruise start — failed experiment (iteration 3)

Iteration 3 (step "start" still wrapped in a 4 s sine speed ramp) initialised
with a **~26° pitch tilt**: VIO z climbed 0.49 m per metre travelled while XY
stayed clean (y error −0.13 m at x = 22). This is the day-6 ramp finding at
full strength — a soft ramp gives the init window weak, smoothly-varying
excitation, and the gravity/velocity SFM locks a tilted frame. The corridor's
proven init regime is a **hard step to cruise speed**; `fly_circuit.py` now
steps to 0.8 m/s instantly (the gentle end-of-lap brake is kept — it is far
outside the init window).

Counter clarification while debugging this: in the ROS 2 port,
loop_fusion's `optimize pose graph` log line is the optimiser thread's
**periodic tick** (fires with zero loop edges); the accepted-loop indicator is
`detect loop with`. `run_vio_loop_offline.sh` now counts only the latter.

## 8. ❌→✅ vins_node OOM via cumulative Path topics (iteration 4)

Step-start fixed half the tilt (26° → ~13°, XY still clean through the first
end-cap: y = 20.2 vs GT 20.0 on the return straight) — and then **vins_node
was kernel-OOM-killed at 8.2 GB anon RSS** ~150 sim-s into the replay.

Mechanism: `nav_msgs/Path` on `/argus/vio/path` (and base_path /
loop_closures) is **cumulative** — every message carries the entire history,
so total bytes grow quadratically. Reliable subscribers (the eval recorder
*and* the RViz viewer attached for the live demo) force the DDS writer inside
vins_node to retain those samples → unbounded anonymous memory in the
estimator process. Iteration 1 survived the same bag length only because no
RViz was attached. Day-5 hit the recorder-side version of this and trimmed
the SuperPoint script; the C1 loop script still carried the Path topics.

Fixes: both offline eval scripts now record **only** gt + odom_optimized
(+ odom_loop); no `/clock` (run_eval.py uses header stamps); RViz is not
attached to eval replays. Iteration-4 also showed the tilt persists with
bare walls, motivating the v2 world's **init garden** (five pillar/crate
clusters at |y| = 2 m over the first 14 m — the obstacle-rich corridor inits
at ~1.3°; bare tunnel walls gave 13–26°).

## 9. ✅ ROOT CAUSE — the contract IMU has been dynamics-blind all along

Iteration 5 (init garden) still initialised at ~11° tilt — three consistent
reproductions ruled out the "init lottery". Reading the **raw IMU stream from
the bag** ended the theorising:

```
rest        ax=+0.00000 std=0.000000   az=+9.80000 std=0.000000   gz=+0.00000
straight    ax=+0.00000 std=0.000000   az=+9.80000 std=0.000000   gz=+0.00000
ARC 1       ax=+0.00002                az=+9.80000                gz=+0.07999  ← gyro exact (v/R)
```

The accelerometer reports **(0, 0, +9.8) constant, zero noise, forever** — the
0.8 m/s velocity step, the end-cap centripetal force, every excitation
experiment: invisible. Mechanism: the contract drone is kinematic
(`<gravity>false</gravity>` + gz `VelocityControl` *sets* link velocity each
step), so the body never has dynamics for the gz IMU system to measure. The
**gyro is exact** (angular velocity is imposed directly and read back), which
is why XY/yaw tracking is excellent everywhere. The acceptance suite only
checks the IMU **at rest** (point 7) — where a dead accelerometer is
indistinguishable from a perfect one.

Every consequence clicks into place:
* VINS gravity/velocity init must explain vision's sudden motion with an
  accelerometer that says "stationary" → it tilts gravity (the 1.3–26°
  Z-ramps of days 6–7, magnitude set by how the step straddles keyframes).
* Day-6's excitation pre-rolls "changed nothing / made init worse" — the IMU
  never saw the excitation, only vision did.
* The IMU-heavy health metrics (`imu_excitation_ok`) were reading a constant.

**Fix** — `scripts/synth_imu_from_gt.py`: rewrite recorded bags with the
acceleration a real IMU would measure for the recorded trajectory
(Savitzky–Golay double-derivative of the GT stream, body-rotated,
gravity-reacted) **plus the contract noise model on both channels**
(σ_acc = 0.002·√250 = 0.032 m/s², σ_gyr = 1.7e-4·√250 = 0.0027 rad/s) — i.e.
exactly what the gz IMU plugin would emit given true dynamics, and strictly
*harder* than the old noiseless constant. Deterministic (seeded). Validated
on the corridor bag: rest = 9.8 ± 0.032, the launch step appears as a
physical ~5 m/s² pulse, gyro noise on spec. This is the standard synthesis
used by simulated VIO benchmarks; documented as a contract-deviation fix.

## 10. ❌→✅ One more OOM: the spectator killed the run

The first synth-IMU replay initialised beautifully — **z flat at ~4 cm over
the first 10 m (≈ 0.13° vs 11–26° before)** — then vins_node was OOM-killed
again at 8.5 GB, sim t ≈ 154. Solver costs were *flat* (~52 ms median in
every decile) until a single 19,000 ms catastrophe at the end: not
degradation — a memory-pressure stall.

The accumulator: an **rqt_image_view left attached to
`/argus/vio/image_track`** (5.5 MB per stereo overlay frame). Its reliable
subscription forces the DDS writer inside vins_node to retain frames; over
~1,500 frames that is ~8 GB inside the estimator process. The viewer had
been watching across two replays — the same mechanism as the §8 Path-topic
OOM, through a different heavy topic. (Iteration 4's death had BOTH RViz and
the recorder attached; trimming the recorder alone was not enough.)

**Rule, now enforced in-script:** `run_vio_loop_offline.sh` pre-flight kills
any attached rqt/rviz before replaying — GUI viewers belong to `demo.sh`
sessions, never to eval replays on a 14 GB host.

## 11. Results — Scenario E (iteration 6, attempt 3: clean room)

*pending*

## 12. Repo / deliverable hygiene — ✅

* `third_party/VINS-Fusion-ROS2` was **gitignored** → a fresh clone could not
  build the VIO. Now vendored in-repo (largest file 58 MB DBoW vocab, under
  GitHub's limit); `src/VINS-Fusion-ROS2` symlink made **relative**.
* Removed a 254 MB `core.442` (rviz2 crash dump) from the repo root; core
  dumps + bag dirs now gitignored.
* Per-package READMEs added to all 7 ROS 2 packages.
* `data/scenarios/scenario_E_tunnel_200m.yaml` added (reproduction commands
  inline, like A–D).
* **Fresh-clone build test PASSED**: `git clone` → clean container →
  `colcon build` = 10/10 packages in 2 min 37 s (stderr only from the usual
  upstream camera_models/loop_fusion/vins warnings). The clone-and-build
  deliverable path is verified end to end.
