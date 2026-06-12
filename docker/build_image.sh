#!/usr/bin/env bash
# ARGUS :: build_image.sh  —  build the argus:humble Docker image (run on the HOST).
#
# Needs Docker access (docker group or sudo). Pulls osrf/ros:humble-desktop-full,
# installs Gazebo Harmonic + deps, compiles Ceres 2.1.0 (slow), and bakes the two
# venvs. Expect 15-40 min on first build depending on network/CPU.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # -> /home/vittal/argus
IMAGE="${IMAGE:-argus:humble}"

echo "[build_image] context=$REPO  image=$IMAGE"
docker build -t "$IMAGE" -f "$REPO/docker/Dockerfile" "$REPO"
echo "[build_image] done -> $IMAGE"
