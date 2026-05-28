from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# Default YOPO python package: <repo>/policy/YOPO
_DEFAULT_YOPO = str(Path(__file__).resolve().parents[5] / "policy" / "YOPO")


def generate_launch_description():
    yopo_root = LaunchConfiguration("yopo_root")
    weight = LaunchConfiguration("weight")

    yopo_root_arg = DeclareLaunchArgument(
        "yopo_root",
        default_value=_DEFAULT_YOPO,
        description="Absolute path to the policy/YOPO/ folder.",
    )
    weight_arg = DeclareLaunchArgument(
        "weight",
        default_value="saved/YOPO_1/epoch50.pth",
        description="Path to model weights (.pth), relative to yopo_root unless absolute.",
    )

    controller_stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("yopo_bringup"), "launch", "controller_stage.launch.py"])
        )
    )

    sensor_sim = Node(
        package="sensor_simulator",
        executable="sensor_simulator",
        name="sensor_simulator_node",
        output="screen",
    )

    yopo_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            [
                "cd ",
                yopo_root,
                " && source /opt/ros/${ROS_DISTRO:-humble}/setup.bash",
                " && conda run -n yopo bash script/run_yopo_ros2.sh ",
                " --weight=",
                weight,
            ],
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        yopo_root_arg,
        weight_arg,
        controller_stage,
        sensor_sim,
        yopo_node,
    ])
