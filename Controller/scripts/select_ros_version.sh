#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
CTRL_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd -P)

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 [ROS1|ROS2]"
  exit 1
fi
ROS_VERSION=$(echo "$1" | tr '[:upper:]' '[:lower:]')
if [[ "${ROS_VERSION}" != "ros1" && "${ROS_VERSION}" != "ros2" ]]; then
  echo "Invalid ROS version: $1 (expected ROS1 or ROS2)"
  exit 1
fi

copy_template() {
  local package_dir="$1"
  local cmake_src="${package_dir}/ros/${ROS_VERSION}.CMakeLists.txt"
  local package_src="${package_dir}/ros/${ROS_VERSION}.package.xml"
  if [[ ! -f "${cmake_src}" || ! -f "${package_src}" ]]; then
    echo "[skip] $(basename "${package_dir}") has no ${ROS_VERSION} templates."
    return 0
  fi
  cp -f "${cmake_src}" "${package_dir}/CMakeLists.txt"
  cp -f "${package_src}" "${package_dir}/package.xml"
  echo "[ok] switched $(basename "${package_dir}") to ${ROS_VERSION}"
}

copy_template "${CTRL_ROOT}/src/utils/quadrotor_msgs"
copy_template "${CTRL_ROOT}/src/so3_control"
copy_template "${CTRL_ROOT}/src/so3_quadrotor_simulator"

echo "Controller switch completed for ${ROS_VERSION}."
