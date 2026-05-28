#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
CTRL_UTILS_DIR="${ROOT_DIR}/Controller/src/utils"
CTRL_WS_DIR="${ROOT_DIR}/Controller/src"
YOPO_DIR="${ROOT_DIR}/policy/YOPO"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Please source your ROS2 installation first."
  echo "Example: source /opt/ros/humble/setup.bash"
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
bash "${ROOT_DIR}/scripts/select_ros_version.sh" ROS2

if [[ -f "${CTRL_WS_DIR}/install_ros2/setup.bash" ]]; then
  set +u
  source "${CTRL_WS_DIR}/install_ros2/setup.bash"
  set -u
elif [[ -f "${CTRL_UTILS_DIR}/install/setup.bash" ]]; then
  set +u
  source "${CTRL_UTILS_DIR}/install/setup.bash"
  set -u
else
  echo "ROS2 controller workspace is not built. Run:"
  echo "  bash ${ROOT_DIR}/scripts/build_controller_ros2.sh"
  exit 1
fi

cd "${YOPO_DIR}"
python script/test_yopo_ros.py --ros_version ros2 "$@"
