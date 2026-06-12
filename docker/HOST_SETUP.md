# ARGUS — Docker host setup (Ubuntu 26.04 host → Humble container)

The host is **Ubuntu 26.04**, too new for ROS 2 Humble (apt only ships Humble for 22.04).
So ARGUS runs inside a **Ubuntu 22.04 + Humble + Gazebo Harmonic** container, using the
host's **RTX 4050** for rendering via the NVIDIA Container Toolkit. This file is the
containerised version of `../MIGRATE_SETUP.md` (which assumed native 22.04 apt).

Host facts (verified): RTX 4050, NVIDIA driver 595.58.03 (CUDA 13.2), `nvidia-smi` works
natively; Docker + toolkit were NOT installed; project lives at `/home/vittal/argus`
(user `vittal`, uid 1000).

---

## Step 1 — Install Docker + NVIDIA Container Toolkit (HOST, needs sudo)

`sudo` on this machine is interactive, so run these yourself. In the Claude Code prompt you
can prefix a line with `!` to run it in this session (output comes back to the chat), or run
them in a normal terminal.

```bash
# Docker Engine (official repo)
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
# NOTE: Docker has no 26.04 repo yet — pin to the 24.04 (noble) repo; the .debs are generic.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu noble stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# NVIDIA Container Toolkit (its repo is distro-independent)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# let `vittal` run docker without sudo (log out/in, or `newgrp docker`, to apply)
sudo usermod -aG docker vittal
```

### Verify GPU passthrough
```bash
newgrp docker            # or open a new terminal so the docker group applies
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```
Expect the RTX 4050 table. If it fails, GPU passthrough is broken — fix before continuing.

---

## Step 2 — Build the image
```bash
cd /home/vittal/argus
docker/build_image.sh          # builds argus:humble (pulls Humble, compiles Ceres 2.1 — slow)
```

## Step 3 — Build the workspace (inside the container)
```bash
docker/run.sh docker/colcon_build.sh
```

## Live demo (one command)
```bash
docker/demo.sh                 # sim + VIO mapping + RViz + onboard camera view, then auto-fly
docker/demo.sh --no-fly        # ... bring it all up but fly the drone yourself
docker rm -f argus             # stop everything
```
Opens four windows (all on the RTX 4050): Gazebo chase-cam following the drone down the lit
corridor, RViz building the point-cloud MAP + trajectory in real time, and rqt_image_view of
the drone's onboard camera with the tracked-feature overlay — then flies the corridor while
VINS maps it live. See the header of `docker/demo.sh` for the design notes (persistent
container, NVIDIA EGL path, loop_fusion vocabulary fix).

## Step 4 — Launch a shell / verify
```bash
docker/run.sh                  # interactive shell; ROS + overlay auto-sourced
```
Inside the container:
```bash
ros2 run argus_bringup acceptance --full                 # expect 11/11 gated PASS
ros2 launch argus_bringup argus_sim.launch.py            # Gazebo, lit corridor
bash scripts/record_baseline_bag.sh data/bags/baseline_live
bash scripts/run_vio_offline.sh data/bags/baseline_live data/bags/vio_eval
~/.venvs/argus-eval/bin/python scripts/run_eval.py --bag data/bags/vio_eval \
  --run-id A --vio-topic /argus/vio/odom_optimized --skip-start-m 2.0 --max-dist-m 24.0
```

## Notes / gotchas carried over
- **Never** `export ROS_LOCALHOST_ONLY=1` — breaks VINS stereo over Cyclone DDS.
- Keep `multiple_thread: 0` in `src/argus_vio/config/argus_stereo_imu_config.yaml` (deterministic).
- `requirements-sp.txt` is a system pip freeze — the image installs only the GPU subset
  (`docker/requirements-sp-min.txt`): onnxruntime-gpu + nvidia-cu12 + opencv.
- The container mounts the project at `/home/vittal/argus` and runs as uid 1000 so the
  VINS symlink + hardcoded paths resolve and written files stay owned by host `vittal`.
- Host is Wayland → XWayland handles GUI; `run.sh` calls `xhost +local:` for you.
- With the RTX you may raise cameras 15→30 Hz (`models/argus_drone/model.sdf`) and replay
  `RATE` 0.15→0.4-1.0 (`scripts/run_vio_offline.sh`). Test incrementally; keep mt0.
```
