#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
CTRL_DIR="${ROOT_DIR}/Controller/src"
SIM_DIR="${ROOT_DIR}/Simulator/src"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Please source ROS2 first (e.g. humble)."
  exit 1
fi

bash "${ROOT_DIR}/scripts/select_ros_version.sh" ROS2

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${CTRL_DIR}/install_ros2/setup.bash" ]]; then
  source "${CTRL_DIR}/install_ros2/setup.bash"
fi
set -u

cd "${SIM_DIR}"
if [[ -n "${YOPO_CUDA_ARCH:-}" || -n "${YOPO_CUDA_ARCH_FLAGS:-}" ]]; then
  colcon --log-base log_ros2 build --symlink-install \
    --base-paths . \
    --build-base build_ros2 \
    --install-base install_ros2 \
    --cmake-args \
      -DPython3_EXECUTABLE=/usr/bin/python3 \
      ${YOPO_CUDA_ARCH:+-DYOPO_CUDA_ARCH=${YOPO_CUDA_ARCH}} \
      ${YOPO_CUDA_ARCH_FLAGS:+-DYOPO_CUDA_ARCH_FLAGS=${YOPO_CUDA_ARCH_FLAGS}}
else
  colcon --log-base log_ros2 build --symlink-install \
    --base-paths . \
    --build-base build_ros2 \
    --install-base install_ros2 \
    --cmake-args \
      -DPython3_EXECUTABLE=/usr/bin/python3
fi

echo
echo "Simulator ROS2 build finished."
echo "Source overlay with:"
echo "  source ${SIM_DIR}/install_ros2/setup.bash"
