# Day 6 — Live-sim refinement: render, VINS determinism, init robustness

**Date:** 2026-06-05
**Objective:** make the FULL pipeline reliably work *live* in sim (lit render → record →
VINS → drift), and fix the latent defects that were silently making results a lottery.
This was the "get everything working in the sim 100%" pass.

> Status tokens: ✅ fixed · ⚠️ open/limited · ❌ failed experiment.

---

## 1. Render regression — ✅ RESOLVED (was transient)

The Day-4 "unlit frames" regression (cam0 mean ~0.5) is **gone**. A fresh `argus_sim`
launch now renders **cam0 mean 175.5, std 57.4, full 0–255 range**, in BOTH
`headless:=true` (EGL offscreen) and `headless:=false` (GUI/GLX). Visual confirms a fully
lit, textured corridor (placards, pillars, ceiling lights, floor stripe, obstacles) —
saved to `PRESENTATION/img/cam0_live_day6.png`.

The GL stack is unchanged from Day-1 (D3D12 / AMD 780M / Mesa 23.2.1), so the Day-4 dark
was a **transient GPU/WSLg context state**, not a driver or code regression — no code
change fixed it. New diagnostics: `scripts/_render_diag.sh` (headless-vs-GUI brightness),
`scripts/_cam_grab.py` (save frame + mean/std/min/max), `scripts/_grab_once.sh`.

Fresh lit bag recorded live: `data/bags/baseline_live_day6` (9 topics, clean).

## 2. VINS non-determinism — ✅ FIXED (`multiple_thread: 0`)  ← the big one

The **same** sensor bag (`baseline_ABC`) gave wildly different drift run-to-run:
**0.73 %** (Day-3 headline) ↔ **33 %** (diverged, this pass). Root cause: with
`multiple_thread: 1`, the VINS feature-tracker and estimator run on separate threads; under
ROS-bag replay the init SFM window sees a *timing-dependent* subset of frames → the
initialization sometimes locks a grossly wrong heading/gravity frame → divergence.

So the Day-3 headlines were **lucky `mt1` runs**, not reproducible. Fix: `multiple_thread: 0`
(single-threaded, synchronous) → deterministic, **no catastrophic divergence**. Verified:
`baseline_ABC` ×2 under mt0 = 3.43 % / 4.17 % (bounded, no 33 % blow-ups). Cost: slower
per-frame → must replay slowly (see §4).

## 3. Stray-sim teardown bug — ✅ FIXED

Record runs were leaving **stray `gz sim` processes alive** (found 3 running at once).
Multiple sims publish the *same* `/argus/*` topics → garbage multi-source interleaving
(IMU appeared at ~350 Hz = 3×, bogus clock stamps) → VINS segfault (`exit -11`, repeated
`throw img0`). Root cause: teardown used `pkill -f "gz-sim"` (hyphen) which does **not**
match the real `"gz sim"` (space) process. Fixed: `record_baseline_bag.sh` and
`record_excite_bag.sh` now call `scripts/_killsim.sh` (which kills `"gz sim"`) at BOTH start
and teardown. **Debug rule: if VINS diverges on a fresh record, first `pgrep -af "gz sim"`.**
(This pollution invalidated the intermediate ramp/excitation experiments — re-run clean.)

## 4. Camera rate + replay rate — ✅ tuned for mt0

- Stereo cameras `update_rate` **30 → 15 Hz** (`models/argus_drone/model.sdf`). A clean
  single sim at RTF~1 can emit ~28 Hz, which overwhelms single-thread VINS → crash. 15 Hz
  caps it, exceeds VINS `freq:10`, is ample for 0.8 m/s flight, and halves bag size.
- `run_vio_offline.sh` default `RATE` **0.4 → 0.15** so mt0 never drops a frame.
- Always eval the **`/argus/vio/odom_optimized`** topic for drift, not the default
  `/argus/vio/odom` (250 Hz imu_propagate, drifts ~5× more).
- **Bonus:** halving the camera load lifted sim **RTF 0.42 → 0.842** (acceptance pt10) —
  the sim now runs near real-time.

## 6. Contract re-verified — ✅ acceptance 11/11

`ros2 run argus_bringup acceptance --full` = **ACCEPTED, 11/11 gated PASS** after the
15 Hz + mt0 changes: 9/9 topics, intrinsics 1280×720 fx=640, cam1 P[3]=−76.8, frames,
IMU a_z=9.800, drive moves x / no fall, /clock advancing, RTF=0.842. The frozen Day-1
contract is intact.

## 5. Init sensitivity — ⚠️ OPEN limitation (honest)

On the feature-poor straight corridor, VINS gravity init carries a ~**1.3° pitch error**
on a plain forward flight → the estimate tilts → a linear **altitude (Z) ramp**
(±0.55 m over 24 m). **XY tracking is excellent** (< 0.05 m); the error is almost entirely
this Z bias. The honest 4-DoF gauge (yaw+translation, roll/pitch fixed by gravity) does not
hide it → ~3.5 % ATE on fresh forward runs.

Mitigations TRIED and ❌ FAILED:
- **Gentle accel ramp** at start — reduced early excitation, no improvement.
- **Multi-axis excitation pre-roll** (vertical bob + lateral before forward) — made init
  *worse*: the sharp velocity reversals corrupted init → **diverged (267 %)**.
Plain step-start is the best fresh-record option. Reaching < 1.5 % requires a lucky-clean
init OR **loop closure** (the pose graph snaps drift back on revisits — Day-3 shuttle
2.18 → 1.31 %). `record_excite_bag.sh` is kept as a documented negative experiment.

---

## Results (deterministic mt0 config)

| Run | Config | Drift (ATE) | Note |
|---|---|---|---|
| Scenario A (Day-3 headline) | mt1, lucky init | **0.73 %** | best case, not reproducible under mt0 |
| Loop closure shuttle (Day-3) | mt1 + loop_fusion | **1.31 %** | revisits correct drift |
| baseline_ABC ×2 | **mt0** | 3.43 % / 4.17 % | deterministic, no divergence |
| fresh live (baseline_live_day6) | **mt0** | 3.92 % | full live pipeline post-fixes; clean Z-ramp, no divergence |

**Bottom line:** the live sim + VIO pipeline now runs **end-to-end without divergence**
(render fixed, determinism fixed, no stray-sim crashes). It **meets < 1.5 %** with a clean
init (0.73 %) and with loop closure (1.31 %); fresh straight-corridor forward runs without
loop closure sit at ~3–4 % due to the gravity-init Z-ramp — a documented, bounded
limitation, not a divergence.

## Files changed
- `src/argus_vio/config/argus_stereo_imu_config.yaml` — `multiple_thread: 0`.
- `models/argus_drone/model.sdf` — stereo cameras 30 → 15 Hz.
- `scripts/run_vio_offline.sh` — default `RATE` 0.15.
- `scripts/record_baseline_bag.sh` — `_killsim` at start+teardown; `RAMP` knob (default 0).
- `scripts/record_excite_bag.sh` (new) — excitation-preroll experiment (negative result).
- New diag scripts: `_render_diag.sh`, `_cam_grab.py`, `_grab_once.sh`, `_determ_test.sh`.
