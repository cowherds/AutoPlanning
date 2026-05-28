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

LOG_FILE="${REPO_ROOT}/super_compat_test.log"
echo "[test] launching super_compat system..."
ros2 launch yopo_bringup system.launch.py planner_method:=super_compat >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

sleep 8

echo "[test] publish one goal..."
timeout 5 ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 8.0, y: 2.0, z: 2.0}, orientation: {w: 1.0}}}" >/dev/null

echo "[test] check /sim/odom ..."
timeout 10 ros2 topic echo /sim/odom --once >/dev/null
if timeout 5 ros2 topic info /lidar_points >/dev/null 2>&1; then
  echo "[test] check /lidar_points ..."
  timeout 10 ros2 topic echo /lidar_points --once >/dev/null
else
  echo "[test] check /lidar_points_world ..."
  timeout 10 ros2 topic echo /lidar_points_world --once >/dev/null
fi
echo "[test] check /so3_control/pos_cmd ..."
timeout 10 ros2 topic echo /so3_control/pos_cmd --once >/dev/null

echo "[test] wait for reach + stable hover from /super_compat/diag/metrics ..."
python3 <<'PY'
import ast
import subprocess
import time

deadline = time.time() + 120.0
hold_need = 5
hold = 0
last_dbg = 0.0

def read_metrics():
    proc = subprocess.run(
        ["bash", "-lc", "timeout 4 ros2 topic echo /super_compat/diag/metrics --once"],
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        return None
    data = None
    for line in proc.stdout.splitlines():
        line = line.strip()
        if line.startswith("data:"):
            payload = line.split("data:", 1)[1].strip()
            try:
                data = ast.literal_eval(payload)
            except Exception:
                data = None
    return data

while time.time() < deadline:
    metrics = read_metrics()
    if not metrics or len(metrics) < 8:
        continue
    d_goal = float(metrics[0])
    speed_xy = float(metrics[2])
    reached = float(metrics[7]) > 0.5
    stable = d_goal < 0.35 and speed_xy < 0.35
    if reached or stable:
        hold += 1
    else:
        hold = 0
    now = time.time()
    if now - last_dbg > 5.0:
        print(f"[test] diag: d_goal={d_goal:.3f}, speed={speed_xy:.3f}, reached={reached}, hold={hold}/{hold_need}")
        last_dbg = now
    if hold >= hold_need:
        print("[test] PASS: reached goal and held stable hover.")
        raise SystemExit(0)

print("[test] FAIL: did not reach and hold goal within timeout.")
raise SystemExit(1)
PY
