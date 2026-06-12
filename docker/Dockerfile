# ARGUS — Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic container
#
# The HOST is Ubuntu 26.04, which has no apt packages for ROS 2 Humble (Humble
# targets 22.04). So the whole stack runs inside this container; the host only
# provides Docker + the NVIDIA Container Toolkit + the GPU.
#
# This image is the containerised form of MIGRATE_SETUP.md (which assumed a
# native 22.04 apt install). Key adaptations:
#   - base image already IS Ubuntu 22.04 + Humble (osrf/ros:humble-desktop-full)
#   - a `vittal` user (uid/gid 1000) with HOME=/home/vittal is created so the
#     repo's hardcoded /home/vittal paths + the src/VINS-Fusion-ROS2 symlink and
#     the ~/.venvs / $HOME/.venvs script references all resolve, AND files written
#     into the mounted /home/vittal/argus keep host uid 1000 ownership.
#   - argus-sp venv installs ONLY the GPU subset (onnxruntime-gpu + nvidia-cu12 +
#     opencv) — NOT the full requirements-sp.txt, which is a system pip freeze
#     full of apt-only packages (see feedback_gotchas: requirements-sp).
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- Gazebo Harmonic apt repo + all apt deps (ROS gz bridge, DDS, GUI, Ceres build deps) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release wget ca-certificates sudo && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-humble-ros-gzharmonic \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rviz2 \
        ros-humble-rqt-image-view \
        ros-humble-xacro \
        python3-colcon-common-extensions \
        cmake build-essential git \
        libeigen3-dev libgoogle-glog-dev libgflags-dev libsuitesparse-dev \
        python3-venv python3-pip dos2unix mesa-utils \
        libgl1-mesa-dri && \
    rm -rf /var/lib/apt/lists/*

# --- Ceres 2.1.0 from source -> /usr/local (apt ships 2.0; VINS needs ceres::Manifold) ---
RUN cd /tmp && \
    wget -q https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.tar.gz \
        -O ceres-2.1.0.tar.gz && \
    tar xzf ceres-2.1.0.tar.gz && \
    cd ceres-solver-2.1.0 && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF && \
    make -j"$(nproc)" && make install && ldconfig && \
    cd /tmp && rm -rf ceres-solver-2.1.0 ceres-2.1.0.tar.gz

# --- create vittal:1000 with HOME=/home/vittal (matches repo's absolute paths) ---
RUN groupadd -g 1000 vittal && \
    useradd -u 1000 -g 1000 -m -d /home/vittal -s /bin/bash vittal && \
    groupadd -f render && \
    usermod -aG sudo,video,render vittal && \
    echo "vittal ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/vittal && \
    chmod 0440 /etc/sudoers.d/vittal

# --- Python venvs (baked into the image at /home/vittal/.venvs, OUTSIDE the mount) ---
COPY requirements-eval.txt /tmp/requirements-eval.txt
COPY docker/requirements-sp-min.txt /tmp/requirements-sp-min.txt

USER vittal
WORKDIR /home/vittal

# eval venv: pure-python (evo, rosbags, numpy, matplotlib, pandas, scipy) — full file installs fine
RUN python3 -m venv /home/vittal/.venvs/argus-eval && \
    /home/vittal/.venvs/argus-eval/bin/pip install --no-cache-dir -U pip && \
    /home/vittal/.venvs/argus-eval/bin/pip install --no-cache-dir -r /tmp/requirements-eval.txt

# superpoint venv: --system-site-packages so rclpy resolves; install ONLY the GPU subset
RUN python3 -m venv --system-site-packages /home/vittal/.venvs/argus-sp && \
    /home/vittal/.venvs/argus-sp/bin/pip install --no-cache-dir -U pip && \
    /home/vittal/.venvs/argus-sp/bin/pip install --no-cache-dir -r /tmp/requirements-sp-min.txt

# --- GPU passthrough hints + ROS env (NEVER set ROS_LOCALHOST_ONLY — breaks VINS stereo) ---
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=42

# auto-source ROS + the workspace overlay (once built) for every interactive shell
RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'if [ -f /home/vittal/argus/install/setup.bash ]; then source /home/vittal/argus/install/setup.bash; fi' \
    'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' \
    'export ROS_DOMAIN_ID=42' \
    >> /home/vittal/.bashrc

WORKDIR /home/vittal/argus
CMD ["/bin/bash"]
