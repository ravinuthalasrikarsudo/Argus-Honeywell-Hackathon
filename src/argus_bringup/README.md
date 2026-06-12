# argus_bringup — launch, bridge, drive & acceptance (Pillar 1)

Top-level orchestration of the simulation stack and the formal acceptance gate.

| Entry point | Role |
|-------------|------|
| `launch/argus_sim.launch.py` | One-shot bringup: gz-sim world + drone spawn (1.5, 0, 1.0) + `ros_gz` parameter bridge + camera-info patch. Args: `world`, `headless`, `use_sim_time`, `spawn_delay`. |
| `config/argus_bridge.yaml` | The frozen 11-topic gz↔ROS bridge contract. |
| `argus_bringup/camera_info_patch.py` | Deviation #3: republishes cam1 CameraInfo with `P[3] = -fx·baseline` (Gazebo emits 0). |
| `argus_bringup/acceptance.py` | 10-point + 2-bonus scorecard: build, interfaces, topic flow, intrinsics, frames, IMU sanity, drive test, clocks, RTF, contract bag. `ros2 run argus_bringup acceptance --full` → expect 11/11 gated PASS. |
| `argus_bringup/drive_drone.py` | CLI flight pattern helper (forward/backward/square/yaw), body-FLU `cmd_vel`. |
| `argus_bringup/record_bag.py` | Contract-topic rosbag recorder (clean SIGINT finalization). |
| `argus_bringup/check_stack.py` | Day-to-day quick probe of a running stack. |

Quick start (inside the Docker container or a sourced native install):

```bash
ros2 launch argus_bringup argus_sim.launch.py headless:=true
ros2 run argus_bringup acceptance            # runtime points against the live stack
ros2 run argus_bringup drive_drone --pattern forward --speed 0.5 --duration 20
```
