#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
SIM_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd -P)

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 [ROS1|ROS2]"
  exit 1
fi
ROS_VERSION=$(echo "$1" | tr '[:upper:]' '[:lower:]')
if [[ "${ROS_VERSION}" != "ros1" && "${ROS_VERSION}" != "ros2" ]]; then
  echo "Invalid ROS version: $1 (expected ROS1 or ROS2)"
  exit 1
fi

cmake_src="${SIM_ROOT}/src/ros/${ROS_VERSION}.CMakeLists.txt"
package_src="${SIM_ROOT}/src/ros/${ROS_VERSION}.package.xml"
if [[ ! -f "${cmake_src}" || ! -f "${package_src}" ]]; then
  echo "Missing ROS templates for sensor_simulator (${ROS_VERSION})"
  exit 1
fi

cp -f "${cmake_src}" "${SIM_ROOT}/src/CMakeLists.txt"
cp -f "${package_src}" "${SIM_ROOT}/src/package.xml"
echo "Simulator switch completed for ${ROS_VERSION}."
