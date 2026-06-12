# ARGUS Day-1 — Frozen Interface Contract

This document is the **authoritative, downstream-facing specification** for the
ARGUS Day-1 simulation foundation. Everything here is **frozen**: the later
pillars (VIO, mapping, planning, …) are written against these topics, frames,
units, intrinsics and message schemas, so they must not change without an
explicit contract revision and a sweep of every consumer.

If the world, the drone model, the bridge or the message definitions ever
disagree with this document, **this document is the bug report** — fix the code
or revise the contract deliberately; do not let them silently drift.

- Conventions: **ENU** world frame, **FLU** body frame, **SI units** throughout.
- Authoritative sources in the tree:
  - World — `src/argus_sim/worlds/warehouse_corridor.sdf`
  - Drone — `src/argus_sim/models/argus_drone/model.sdf`
  - Bridge — `src/argus_bringup/config/argus_bridge.yaml` + `camera_info_patch`
  - Messages — `src/argus_msgs/msg/*.msg`

---

## 1. Frames and units

| Item | Value |
|------|-------|
| World frame | ENU. `+X` = corridor length (0 → 30 m), `+Y` = width (−2.5 → +2.5 m), `+Z` = up (0 → 3 m) |
| World frame name | `warehouse_corridor` |
| Body frame | FLU — `+X` forward, `+Y` left, `+Z` up |
| Body frame name | `base_link` (at the model origin ⇒ model pose ≡ `base_link` pose) |
| Units | metres, radians, seconds, m/s, rad/s, m/s² (SI) |
| Camera optical frames | `cam0_optical_frame` (left), `cam1_optical_frame` (right) |
| IMU frame | `imu_link` |

There is deliberately **no `world → base_link` TF** (see deviation #4). Ground
truth is published as a *topic*.

---

## 2. World contract — `warehouse_corridor.sdf`

- Dimensions: 30 m (X) × 5 m (Y) × 3 m (Z) corridor. Floor top at `z = 0`,
  ceiling bottom at `z = 3`.
- Zones along X: **A** `[0, 10]`, **B** `[10, 20]`, **C** `[20, 30]`
  (floor stripes + wall placards mark them).
- Drone start pose: **`(1.5, 0, 1.0)`**.
- Lights: point lights `light_a1`, `light_a2`, `light_b1`, `light_b_flicker`,
  `light_c1`, `light_c2`, plus one directional `fill`.
  - `light_b_flicker` (Zone B) is declared **static-bright** in the SDF and
    driven at runtime by `scripts/flicker.sh` → `flicker_light.sh` via the
    `/world/warehouse_corridor/light_config` service (`gz.msgs.Light`,
    intensity toggled 1.0 ↔ 0.05). gz-harmonic ships no autonomous flicker
    plugin, so flicker is an external driver, not a world property.
- Obstacles: 7 static obstacles in the corridor.
- Physics: **dartsim**, `max_step_size = 0.004` (250 Hz), RTF target 1.0
  (deviation #5).
- Systems loaded by the world: `Physics`, `SceneBroadcaster`, `UserCommands`,
  `Contact`, `Imu`, and `Sensors` (ogre2 render engine). Because the world
  already declares `Sensors` and `Imu`, the drone model carries no world edits.
- Resolves by name: once `argus_sim` is built and sourced,
  `gz sim warehouse_corridor.sdf` finds the world via the
  `GZ_SIM_RESOURCE_PATH` install hook.

> **Frozen:** zone boundaries, spawn pose, world name, frame/unit convention and
> light names are all part of the contract. The drone is **spawned at runtime by
> the launch**, never baked into the world, so the world file stays a stable,
> standalone artifact.

---

## 3. Drone contract — `models/argus_drone/model.sdf`

Kinematic stereo + IMU drone (SDF 1.10). Body frame FLU.

### Links

| Link | Offset from `base_link` | Role |
|------|-------------------------|------|
| `base_link` | (0, 0, 0) | Canonical body link (model origin) |
| `cam0_link` | (+0.10, +0.06, 0) | Left camera |
| `cam1_link` | (+0.10, −0.06, 0) | Right camera |
| `imu_link`  | (0, 0, 0) | IMU |

Cameras and IMU are attached by fixed joints. The 0.12 m camera separation is
the stereo **baseline**.

### Kinematic behaviour

Per-link `<gravity>false</gravity>` plus a model-attached `VelocityControl`
plugin: the drone hovers and obeys `cmd_vel`, and never falls (verified: `z`
stays at 1.0 at rest). A `PosePublisher` (`publish_nested_model_pose=true`,
`use_pose_vector_msg=false`, 100 Hz) emits a single ground-truth pose.

### Sensors

| Sensor | Spec |
|--------|------|
| Stereo cameras | 2 × 1280×720, hfov 90°, 30 Hz, ogre2 |
| IMU | 250 Hz, noise-free for Day-1 (at rest reads `linear_acceleration.z = +9.8`) |

### Intrinsics (both cameras identical, verified)

| Parameter | Value |
|-----------|-------|
| Resolution | 1280 × 720 |
| `fx`, `fy` | 640 |
| `cx` | 640 |
| `cy` | 360 |
| Distortion | 0 (all coefficients) |
| Rectification | identity |
| Baseline | 0.12 m |
| `P[3]` from gz | 0 on **both** cameras (gz cannot encode the baseline — see dev #3) |

### gz topics emitted by the model

These already match the `/argus/*` ROS contract; the Phase-5 bridge is a 1:1
mapping plus the #2 / #3 fixes.

| gz topic | gz type | Rate |
|----------|---------|-----:|
| `/argus/cam0/image_raw` + `/argus/cam0/camera_info` | image / camera_info | 30 Hz |
| `/argus/cam1/image_raw` + `/argus/cam1/camera_info` | image / camera_info | 30 Hz |
| `/argus/imu` | `gz.msgs.IMU` | 250 Hz |
| `/model/argus_drone/pose` | `gz.msgs.Pose` | 100 Hz |
| `/model/argus_drone/cmd_vel` | `gz.msgs.Twist` (in) | — |

> The image topic name MUST stay **one level deep** under each camera namespace:
> gz derives the `camera_info` topic from the image topic's parent namespace, so
> flattening it would collide both cameras on a single `/argus/camera_info`.

### Frame IDs

| Data | `frame_id` |
|------|-----------|
| `cam0` image + info | `cam0_optical_frame` |
| `cam1` image + info | `cam1_optical_frame` |
| IMU | `imu_link` |
| Ground-truth pose | `frame_id` = world (`warehouse_corridor`), `child_frame_id` = `argus_drone` |

> Frame IDs are set via the gz custom element `<gz_frame_id>`. libsdformat prints
> a benign *"gz_frame_id is not defined in SDF"* warning on every load (three
> times), but gz-sim honours it and the frame_ids are confirmed correct. This
> warning is expected, not an error.

---

## 4. Bridge contract — `argus_bringup`

ROS ↔ gz via:

```bash
ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=<install>/argus_bringup/config/argus_bridge.yaml
```

Nine bridge entries (all `gz.msgs.*`), plus the `camera_info_patch` node. **The
launch starts BOTH** the bridge and the patch — without the patch, the right
`CameraInfo` keeps `P[3] = 0` and downstream stereo is wrong.

### GZ → ROS

| gz topic | ROS topic | ROS type | Notes |
|----------|-----------|----------|-------|
| `/clock` | `/clock` **and** `/argus/clock` | `rosgraph_msgs/Clock` | dev #2 |
| `/argus/imu` | `/argus/imu` | `sensor_msgs/Imu` | |
| `/argus/cam0/image_raw` | `/argus/cam0/image_raw` | `sensor_msgs/Image` | |
| `/argus/cam1/image_raw` | `/argus/cam1/image_raw` | `sensor_msgs/Image` | |
| `/argus/cam0/camera_info` | `/argus/cam0/camera_info` | `sensor_msgs/CameraInfo` | passthrough, `P[3]=0` |
| `/argus/cam1/camera_info` | `/argus/cam1/camera_info_gz` → `/argus/cam1/camera_info` | `sensor_msgs/CameraInfo` | republished by `camera_info_patch` with `P[3] = −fx·baseline = −76.8` (dev #3) |
| `/model/argus_drone/pose` | `/argus/ground_truth/pose` | `geometry_msgs/PoseStamped` | dev #4; `frame_id` = `warehouse_corridor` |

### ROS → GZ

| ROS topic | gz topic | ROS type |
|-----------|----------|----------|
| `/argus/cmd_vel` | `/model/argus_drone/cmd_vel` | `geometry_msgs/Twist` |

**Downstream rule:** ignore the intermediate `/argus/cam1/camera_info_gz`;
consume `/argus/cam1/camera_info`. `cam0` is the left / stereo-reference camera
(`P[3] = 0`). Verified live: `pose.x` advances 1.5 → 2.8 under a 0.5 m/s
`cmd_vel`.

---

## 5. Message schemas — `argus_msgs`

Authoritative source: `src/argus_msgs/msg/*.msg`. Self-designed for Day-1 (the
brief delegated the exact schema). Frozen facts downstream must honour:

### `VIOHealth.msg`

```
std_msgs/Header header
uint8 status                       # enum below
float32 confidence                 # [0..1]
int32  num_tracked_features
int32  num_inlier_features
float32 avg_parallax
float64 position_covariance_trace
float64 estimated_drift_rate
bool   imu_excitation_ok
float64 processing_latency_ms
# status enum:
#   STATUS_INITIALIZING = 0
#   STATUS_NOMINAL      = 1
#   STATUS_DEGRADED     = 2
#   STATUS_LOST         = 3
```

### `UncertaintyMap.msg`

```
std_msgs/Header header
geometry_msgs/Pose pose
float32[36] covariance             # 6x6 ROW-MAJOR, order x,y,z,roll,pitch,yaw; float32 (not float64)
float32 position_uncertainty
float32 orientation_uncertainty
```

> Check the exact field names and ordering with
> `ros2 interface show argus_msgs/msg/VIOHealth` (and `UncertaintyMap`) — that
> output is the ground truth if it ever disagrees with the table above.

---

## 6. Deviations from the original brief

Five intentional deviations, each minimal and contract-preserving. Recorded here
so they are never silently "corrected" downstream.

### #1 — Gazebo Garden → Harmonic
Garden reached end-of-life (Nov 2024); Harmonic is the current LTS (supported to
2028). Same `gz::sim::systems` plugin namespaces and SDF; only the apt package
names differ (`gz-harmonic`, `ros-humble-ros-gzharmonic`). Binary is `gz sim`
(`gz-sim8`).

### #2 — `/clock` bridged in addition to `/argus/clock`
The schema specified `/argus/clock`, but standard ROS nodes running with
`use_sim_time` subscribe to `/clock`. Without `/clock` bridged, sim time would
silently fail to propagate. Both topics are published.

### #3 — Stereo baseline republished into `cam1.P[3]`
Gazebo treats the two cameras as independent and does not encode the stereo
baseline, leaving `P[3] = 0` on both. The `camera_info_patch` node republishes
the right camera's `CameraInfo` with `P[3] = −fx·baseline = −640 × 0.12 = −76.8`,
which is what stereo consumers expect. `cam0` (left) is the reference, `P[3] = 0`.

### #4 — No `world → base_link` TF
Ground truth is a **topic** (`/argus/ground_truth/pose`,
`geometry_msgs/PoseStamped`), not a TF transform. The `world → base_link` TF
edge is intentionally left unpublished so that VIO can own the future
`odom → base_link` estimate without fighting a ground-truth TF.

### #5 — ODE → dartsim physics
gz-harmonic's gz-physics ships dartsim / bullet / bullet-featherstone / tpe —
there is **no ODE engine** (ODE belonged to gazebo-classic). The world uses the
gz-sim default **dartsim** (most accurate and best-supported; correct for a
kinematic drone over a static world). The SDF carries `<physics type="dart">`
with `max_step_size = 0.004` (250 Hz physics, supporting IMU ≤ 250 Hz) and no
`<engine>` override (default = dartsim).

---

## 7. Performance note (not a contract term, but load-bearing)

Under WSLg, gz `ogre2` rendering runs on the **integrated AMD Radeon 780M GPU
plus shared system RAM**, not the discrete RTX 4050. The Phase-9 acceptance
suite measured **RTF = 0.42** under the full world with both 720p cameras and
the IMU; sensor rates scale with it (IMU ≈ 103 Hz of 250, cameras ≈ 12.6 of 30).
The discrete GPU's VRAM is not the bottleneck — iGPU + shared RAM is. RTF is
reported, not gated (a known platform limit). If it matters downstream, reduce
cosmetic simulation load **before** changing anything in this contract.
