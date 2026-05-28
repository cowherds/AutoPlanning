#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
CTRL_DIR="${ROOT_DIR}/Controller/src"
SIM_DIR="${ROOT_DIR}/Simulator/src"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Please source ROS2 first (e.g. humble)."
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${CTRL_DIR}/install_ros2/setup.bash"
source "${SIM_DIR}/install_ros2/setup.bash"
set -u

ros2 launch sensor_simulator sensor_simulator.launch.py "$@"
