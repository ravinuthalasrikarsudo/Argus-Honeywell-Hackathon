#!/usr/bin/env bash
#
# ARGUS Day-1 :: env.sh
# Sourceable environment for the ARGUS workspace. Mirrors the ~/.bashrc ARGUS
# block but is self-contained, so non-interactive shells and the other helper
# scripts get a correct ROS env regardless of how they were started.
#
# Usage:  source scripts/env.sh
#
# Source it (do not execute): it must mutate the *calling* shell's environment.
# No `set -u` here on purpose -- the upstream ROS setup scripts reference unset
# variables, and tainting the caller's shell options would be rude.

# Resolve the workspace root from this file's own location (location-independent).
_ARGUS_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARGUS_WS="$(dirname "${_ARGUS_ENV_DIR}")"
export ARGUS_WS

# The upstream ROS/ament setup scripts are not `set -u` (nounset) safe -- they
# read variables such as AMENT_TRACE_SETUP_FILES before defining them. If the
# caller sourced us under `set -u` (the helper scripts do), relax it across the
# sourcing and restore the caller's prior setting afterwards.
case $- in
  *u*) _argus_had_nounset=1; set +u ;;
  *)   _argus_had_nounset=0 ;;
esac

source /opt/ros/humble/setup.bash
if [ -f "${ARGUS_WS}/install/setup.bash" ]; then
  source "${ARGUS_WS}/install/setup.bash"
fi

if [ "${_argus_had_nounset}" = 1 ]; then set -u; fi
unset _argus_had_nounset

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
