#!/usr/bin/env python3
"""Launch SUPER-compat planner only (sim/controller must run separately)."""
from pathlib import Path
import argparse
import shutil
import signal
import subprocess
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="Launch super_compat planner via ROS2 launch.")
    parser.add_argument(
        "--world-lidar",
        action="store_true",
        help="Use /lidar_points_world with world_frame_cloud:=true (RViz/debug profile).",
    )
    parser.add_argument(
        "--mission-file",
        type=str,
        default="",
        help="Optional mission file path/name for super waypoint mission mode.",
    )
    args = parser.parse_args()

    ros2_bin = shutil.which("ros2")
    if ros2_bin is None:
        print("[test_super] 'ros2' not found in PATH.")
        print("[test_super] Please source ROS2 first, e.g. source /opt/ros/humble/setup.bash")
        return 1

    repo_root = Path(__file__).resolve().parents[3]
    launch_file = (
        repo_root
        / "Controller"
        / "src"
        / "utils"
        / "yopo_bringup"
        / "launch"
        / "super_compat_only.launch.py"
    )
    if not launch_file.exists():
        print(f"[test_super] Launch file not found: {launch_file}")
        return 1

    setup_candidates = [
        repo_root / "Controller" / "src" / "install_ros2" / "setup.bash",
        repo_root / "Controller" / "src" / "utils" / "install" / "setup.bash",
        repo_root / "install" / "setup.bash",
    ]
    workspace_setup = next((p for p in setup_candidates if p.exists()), None)
    if workspace_setup is None:
        print("[test_super] Missing ROS2 workspace setup.bash.")
        for candidate in setup_candidates:
            print(f"[test_super] - {candidate}")
        print("[test_super] Please build Controller ROS2 workspace first.")
        return 1

    if args.world_lidar:
        lidar_args = "lidar_topic:=/lidar_points_world world_frame_cloud:=true"
        profile = "world-lidar (RViz/debug)"
    else:
        lidar_args = "lidar_topic:=/lidar_points world_frame_cloud:=false"
        profile = "body-lidar (full-stack default)"

    mission_arg = ""
    if args.mission_file.strip():
        mission_arg = f" mission_file:={args.mission_file.strip()}"

    launch_cmd = (
        "unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH; "
        "source /opt/ros/humble/setup.bash; "
        f"source '{workspace_setup}'; "
        f"ros2 launch '{launch_file}' "
        "odom_topic:=/sim/odom "
        f"{lidar_args} "
        "ctrl_topic:=/so3_control/pos_cmd "
        "use_default_goal:=false"
        f"{mission_arg}"
    )
    cmd = ["bash", "-lc", launch_cmd]
    print("[test_super] Launching SUPER-compat method:")
    print(f"[test_super] profile: {profile}")
    print(f"[test_super] ros2 launch {launch_file} ...")
    print("[test_super] This script starts planner only (no simulator/controller).")

    try:
        proc = subprocess.Popen(cmd)
    except FileNotFoundError:
        print("[test_super] Failed to execute ros2 command.")
        return 1

    def _forward_signal(sig_num, _frame):
        if proc.poll() is None:
            proc.send_signal(sig_num)

    signal.signal(signal.SIGINT, _forward_signal)
    signal.signal(signal.SIGTERM, _forward_signal)

    return proc.wait()


if __name__ == "__main__":
    sys.exit(main())
