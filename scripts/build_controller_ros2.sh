#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
CTRL_DIR="${ROOT_DIR}/Controller/src"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Please source ROS2 first (e.g. humble)."
  exit 1
fi

bash "${ROOT_DIR}/scripts/select_ros_version.sh" ROS2

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

cd "${CTRL_DIR}"
colcon --log-base log_ros2 build --symlink-install \
  --base-paths utils/quadrotor_msgs so3_control so3_quadrotor_simulator utils/yopo_bringup \
  --build-base build_ros2 \
  --install-base install_ros2 \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3

echo
echo "Controller ROS2 build finished."
echo "Source overlay with:"
echo "  source ${CTRL_DIR}/install_ros2/setup.bash"
