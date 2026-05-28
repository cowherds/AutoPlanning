from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

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
        description="Weights path relative to yopo_root unless absolute.",
    )

    run_yopo = ExecuteProcess(
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
        emulate_tty=False,
    )

    return LaunchDescription([yopo_root_arg, weight_arg, run_yopo])
