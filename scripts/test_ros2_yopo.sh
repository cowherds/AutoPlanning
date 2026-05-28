#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
CTRL_UTILS_DIR="${ROOT_DIR}/Controller/src/utils"
YOPO_DIR="${ROOT_DIR}/policy/YOPO"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Please source your ROS2 installation first."
  echo "Example: source /opt/ros/humble/setup.bash"
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

echo "[1/5] Switch packages to ROS2 templates"
bash "${ROOT_DIR}/scripts/select_ros_version.sh" ROS2

echo "[2/5] Build ROS2 interface package (quadrotor_msgs)"
cd "${CTRL_UTILS_DIR}"
colcon build --symlink-install --packages-select quadrotor_msgs

echo "[3/5] Source generated setup"
set +u
source "${CTRL_UTILS_DIR}/install/setup.bash"
set -u

echo "[4/5] Python dependency check for ROS2 runtime bridge"
python3 - <<'PY'
import importlib
mods = ["rclpy", "sensor_msgs_py", "nav_msgs.msg", "geometry_msgs.msg", "sensor_msgs.msg"]
missing = [m for m in mods if importlib.util.find_spec(m) is None]
if missing:
    raise SystemExit("Missing modules: " + ", ".join(missing))
print("ROS2 Python deps check passed.")
PY

echo "[5/5] YOPO ROS2 dry-run bridge check"
cd "${YOPO_DIR}"
python3 - <<'PY'
import importlib
import importlib.util
import sys
sys.path.insert(0, ".")
compat = importlib.import_module("ros_compat")
version = compat.detect_ros_version(force_version="ros2")
PositionCommand = compat.import_position_command(version)
print("Loaded ROS bridge successfully.")
print("PositionCommand type:", PositionCommand.__module__ + "." + PositionCommand.__name__)
print("Use runtime command:")
print("  conda activate yopo && python script/test_yopo_ros.py --ros_version ros2 --weight saved/YOPO_1/epoch50.pth")
PY

echo "ROS2 compatibility smoke test completed."
