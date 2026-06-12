# ARGUS — Native Ubuntu setup (from the migration pack)

Target: **Ubuntu 22.04**, user **`vittal`**, project at **`/home/vittal/argus`**
(same username + path → the `src/VINS-Fusion-ROS2 → third_party/...` symlink stays valid;
hardcoded paths in configs resolve). If you use a different user/path, `grep -rl
/home/vittal/argus ~/argus` and fix.

GPU: native Ubuntu uses the real driver (amdgpu for the 780M, or the NVIDIA driver for
the RTX 4050) — no WSLg D3D12 layer, so Gazebo/RViz render stable + lit. For speed, install
the NVIDIA driver and run rendering on the RTX 4050.

---

## 0. Extract the pack
```bash
mkdir -p ~/argus && tar xzf argus_code.tar.gz -C ~/    # extracts to ~/argus
cd ~/argus
```

## 1. ROS 2 Humble + Gazebo Harmonic + tooling (apt)
```bash
# ROS 2 Humble (follow docs.ros.org if the repo isn't added yet), then:
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools \
  ros-humble-rmw-cyclonedds-cpp python3-colcon-common-extensions \
  ros-humble-xacro ros-humble-rviz2 ros-humble-rqt-image-view

# Gazebo Harmonic + ros_gz bridge
sudo apt install -y gz-harmonic ros-humble-ros-gzharmonic

# Ceres build deps + misc
sudo apt install -y cmake libeigen3-dev libgoogle-glog-dev libgflags-dev \
  libsuitesparse-dev libceres-dev python3-pip python3.10-venv dos2unix mesa-utils
```

## 2. Ceres 2.1.0 from source (apt ships 2.0 — too old: no `ceres::Manifold`)
```bash
cd /tmp && wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz
tar xzf ceres-solver-2.1.0.tar.gz && cd ceres-solver-2.1.0
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j$(nproc) && sudo make install      # installs to /usr/local
```

## 3. Python venvs (OUTSIDE the workspace)
```bash
# eval venv (drift/plots): evo + rosbags + numpy
python3 -m venv ~/.venvs/argus-eval
~/.venvs/argus-eval/bin/pip install -r ~/argus/requirements-eval.txt

# superpoint venv: onnxruntime-gpu (bundles CUDA12/cuDNN). --system-site-packages so rclpy resolves
python3 -m venv --system-site-packages ~/.venvs/argus-sp
~/.venvs/argus-sp/bin/pip install -r ~/argus/requirements-sp.txt
```
(SuperPoint ONNX models are already in `models/superpoint/`; else re-run
`models/superpoint/download_models.sh`.)

## 4. Build the workspace (pin the source Ceres)
```bash
cd ~/argus
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --cmake-args -DCeres_DIR=/usr/local/lib/cmake/Ceres -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
The VINS QoS patch + the C2 SuperPoint patch are already in `third_party/` (they travel in
the pack — no re-clone, no re-apply needed).

## 5. Verify
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run argus_bringup acceptance --full          # expect 11/11 gated PASS
# live (render is reliable on native Ubuntu):
ros2 launch argus_bringup argus_sim.launch.py     # Gazebo, lit corridor
```

## 6. Re-record demo bags on Ubuntu (render works → fresh lit bags)
```bash
bash scripts/record_baseline_bag.sh ~/argus/data/bags/baseline_live
bash scripts/run_vio_offline.sh ~/argus/data/bags/baseline_live ~/argus/data/bags/vio_eval
~/.venvs/argus-eval/bin/python scripts/run_eval.py --bag ~/argus/data/bags/vio_eval \
  --run-id A --vio-topic /argus/vio/odom_optimized --skip-start-m 2.0 --max-dist-m 24.0
```
(Optional: the WSL demo bags `baseline_live_day6` + `map_demo` are in the separate
`argus_bags.tar.gz` if you want the exact pre-recorded demos without re-recording.)

## Notes carried from WSL build
- `multiple_thread: 0` in `src/argus_vio/config/argus_stereo_imu_config.yaml` (deterministic).
- Cameras 15 Hz, `run_vio_offline` RATE 0.15 (was WSL/iGPU tuning; on native+RTX you can
  raise both — RTF is far higher).
- Never `export ROS_LOCALHOST_ONLY=1` (breaks VINS stereo over Cyclone).
