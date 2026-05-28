#!/usr/bin/env bash
# Run YOPO planner against ROS2 (sources controller workspace for quadrotor_msgs).
set -euo pipefail
YOPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "${YOPO_DIR}/../.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${REPO_ROOT}/Controller/src/install_ros2/setup.bash"
exec python "${YOPO_DIR}/script/test_yopo_ros.py" --ros_version ros2 "$@"
