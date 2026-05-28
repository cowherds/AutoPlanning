#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"

source_safe() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

source_safe /opt/ros/humble/setup.bash

if [[ -f "${REPO_ROOT}/Controller/src/install_ros2/setup.bash" ]]; then
  source_safe "${REPO_ROOT}/Controller/src/install_ros2/setup.bash"
fi

if [[ -f "${REPO_ROOT}/Simulator/src/install_ros2/setup.bash" ]]; then
  source_safe "${REPO_ROOT}/Simulator/src/install_ros2/setup.bash"
fi

REPORT_FILE="${REPO_ROOT}/super_parity_report.json"
LOG_FILE="${REPO_ROOT}/super_parity_regression.log"

echo "[regression] launching super_compat stack..."
ros2 launch yopo_bringup system.launch.py planner_method:=super_compat >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

sleep 10
echo "[regression] running metric collection..."
python3 "${SCRIPT_DIR}/super_parity_regression.py" --output "${REPORT_FILE}"
echo "[regression] done. report=${REPORT_FILE}"
