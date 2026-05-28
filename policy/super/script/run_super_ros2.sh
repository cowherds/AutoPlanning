#!/usr/bin/env bash
# Run SUPER-compat planner against ROS2 (sources controller workspace for quadrotor_msgs).
set -euo pipefail
SUPER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "${SUPER_DIR}/../.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ -f "${REPO_ROOT}/Controller/src/install_ros2/setup.bash" ]]; then
  source "${REPO_ROOT}/Controller/src/install_ros2/setup.bash"
elif [[ -f "${REPO_ROOT}/Controller/src/utils/install/setup.bash" ]]; then
  source "${REPO_ROOT}/Controller/src/utils/install/setup.bash"
else
  echo "[run_super_ros2] Missing controller setup.bash" >&2
  exit 1
fi
exec python3 "${SUPER_DIR}/super_compat_planner.py" "$@"
