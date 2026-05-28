import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

_DEFAULT_YOPO = str(Path(__file__).resolve().parents[5] / "policy" / "YOPO")
_DEFAULT_SUPER = str(Path(__file__).resolve().parents[5] / "policy" / "super")
_DEFAULT_CONTROLLER_SETUP = str(
    Path(__file__).resolve().parents[5] / "Controller" / "src" / "install_ros2" / "setup.bash"
)
_DEFAULT_CONTROLLER_SETUP_ALT = str(
    Path(__file__).resolve().parents[5] / "Controller" / "src" / "utils" / "install" / "setup.bash"
)
_DEFAULT_SUPER_PARAM = str(
    Path(__file__).resolve().parents[5] / "policy" / "super" / "config" / "super_compat.yaml"
)


def generate_launch_description():
    yopo_root = LaunchConfiguration("yopo_root")
    weight = LaunchConfiguration("weight")
    planner_method = LaunchConfiguration("planner_method")
    super_root = LaunchConfiguration("super_root")
    odom_topic = LaunchConfiguration("odom_topic")
    lidar_topic = LaunchConfiguration("lidar_topic")
    ctrl_topic = LaunchConfiguration("ctrl_topic")
    world_frame_cloud = LaunchConfiguration("world_frame_cloud")
    flight_height = LaunchConfiguration("flight_height")
    max_speed = LaunchConfiguration("max_speed")
    goal_tolerance = LaunchConfiguration("goal_tolerance")
    map_resolution = LaunchConfiguration("map_resolution")
    planning_horizon = LaunchConfiguration("planning_horizon")
    obstacle_inflation = LaunchConfiguration("obstacle_inflation")
    map_decay_sec = LaunchConfiguration("map_decay_sec")
    start_clearance = LaunchConfiguration("start_clearance")
    mission_file = LaunchConfiguration("mission_file")
    super_param_file = LaunchConfiguration("super_param_file")

    yopo_root_arg = DeclareLaunchArgument(
        "yopo_root",
        default_value=_DEFAULT_YOPO,
        description="Absolute path to the policy/YOPO/ folder.",
    )
    weight_arg = DeclareLaunchArgument(
        "weight",
        default_value="saved/YOPO_1/epoch50.pth",
        description="Weights .pth path (relative to yopo_root unless absolute).",
    )
    planner_method_arg = DeclareLaunchArgument(
        "planner_method",
        default_value="yopo",
        description="Planner branch to run: yopo | super_compat",
    )
    super_root_arg = DeclareLaunchArgument(
        "super_root",
        default_value=_DEFAULT_SUPER,
        description="Absolute path to the policy/super/ folder.",
    )
    odom_topic_arg = DeclareLaunchArgument("odom_topic", default_value="/sim/odom")
    lidar_topic_arg = DeclareLaunchArgument("lidar_topic", default_value="/lidar_points")
    ctrl_topic_arg = DeclareLaunchArgument("ctrl_topic", default_value="/so3_control/pos_cmd")
    world_frame_cloud_arg = DeclareLaunchArgument(
        "world_frame_cloud",
        default_value="false",
        description=(
            "super_compat: true if lidar_topic publishes world-frame points "
            "(pair with /lidar_points_world); false for body-frame /lidar_points."
        ),
    )
    flight_height_arg = DeclareLaunchArgument(
        "flight_height",
        default_value="3.0",
        description="Fixed planner flight height (m); clamped by flight_constraints.",
    )
    max_speed_arg = DeclareLaunchArgument(
        "max_speed",
        default_value="10.0",
        description="Planner max speed in m/s.",
    )
    goal_tolerance_arg = DeclareLaunchArgument(
        "goal_tolerance",
        default_value="0.8",
        description="super_compat: horizontal distance to goal treated as arrived (m).",
    )
    map_resolution_arg = DeclareLaunchArgument(
        "map_resolution",
        default_value="0.35",
        description="super_compat map grid resolution in meters.",
    )
    planning_horizon_arg = DeclareLaunchArgument(
        "planning_horizon",
        default_value="16.0",
        description="super_compat local planning horizon in meters.",
    )
    obstacle_inflation_arg = DeclareLaunchArgument(
        "obstacle_inflation",
        default_value="0.7",
        description="super_compat obstacle inflation radius in meters.",
    )
    map_decay_sec_arg = DeclareLaunchArgument(
        "map_decay_sec",
        default_value="2.0",
        description="super_compat occupancy decay time in seconds.",
    )
    start_clearance_arg = DeclareLaunchArgument(
        "start_clearance",
        default_value="1.0",
        description="super_compat start-area clearance radius in meters.",
    )
    mission_file_arg = DeclareLaunchArgument(
        "mission_file",
        default_value="__none__",
        description="Optional mission file for SUPER waypoint mode; __none__ disables mission mode.",
    )
    super_param_file_arg = DeclareLaunchArgument(
        "super_param_file",
        default_value=_DEFAULT_SUPER_PARAM,
        description="ROS2 params file for super_compat_planner.",
    )

    sim_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so3_quadrotor_simulator"),
                "launch",
                "simulator_attitude_control.launch.py",
            )
        )
    )

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sensor_simulator"),
                "launch",
                "sensor_simulator.launch.py",
            )
        )
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
                " --flight_height=",
                flight_height,
            ],
        ],
        output="screen",
        emulate_tty=False,
        condition=IfCondition(PythonExpression(["'", planner_method, "' == 'yopo'"])),
    )

    super_compat_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            [
                "source /opt/ros/${ROS_DISTRO:-humble}/setup.bash",
                " && if [ -f '",
                _DEFAULT_CONTROLLER_SETUP,
                "' ]; then source '",
                _DEFAULT_CONTROLLER_SETUP,
                "'; elif [ -f '",
                _DEFAULT_CONTROLLER_SETUP_ALT,
                "' ]; then source '",
                _DEFAULT_CONTROLLER_SETUP_ALT,
                "'; else echo '[system.launch] Missing controller setup.bash' >&2; exit 1; fi",
                " && if [[ '",
                world_frame_cloud,
                "' == 'true' && '",
                lidar_topic,
                "' != '/lidar_points_world' ]]; then echo '[system.launch] world_frame_cloud=true requires /lidar_points_world' >&2; exit 1; fi",
                " && if [[ '",
                world_frame_cloud,
                "' == 'false' && '",
                lidar_topic,
                "' == '/lidar_points_world' ]]; then echo '[system.launch] /lidar_points_world requires world_frame_cloud=true' >&2; exit 1; fi",
                " && python3 ",
                super_root,
                "/super_compat_planner.py",
                " --ros-args",
                " --params-file ",
                super_param_file,
                " -p odom_topic:=",
                odom_topic,
                " -p lidar_topic:=",
                lidar_topic,
                " -p ctrl_topic:=",
                ctrl_topic,
                " -p world_frame_cloud:=",
                world_frame_cloud,
                " -p flight_height:=",
                flight_height,
                " -p max_speed:=",
                max_speed,
                " -p goal_tolerance:=",
                goal_tolerance,
                " -p map_resolution:=",
                map_resolution,
                " -p planning_horizon:=",
                planning_horizon,
                " -p obstacle_inflation:=",
                obstacle_inflation,
                " -p map_decay_sec:=",
                map_decay_sec,
                " -p start_clearance:=",
                start_clearance,
                " -p mission_file:=",
                mission_file,
            ],
        ],
        output="screen",
        condition=IfCondition(PythonExpression(["'", planner_method, "' == 'super_compat'"])),
    )

    return LaunchDescription([
        yopo_root_arg,
        weight_arg,
        planner_method_arg,
        super_root_arg,
        odom_topic_arg,
        lidar_topic_arg,
        ctrl_topic_arg,
        world_frame_cloud_arg,
        flight_height_arg,
        max_speed_arg,
        goal_tolerance_arg,
        map_resolution_arg,
        planning_horizon_arg,
        obstacle_inflation_arg,
        map_decay_sec_arg,
        start_clearance_arg,
        mission_file_arg,
        super_param_file_arg,
        sim_ctrl_launch,
        sensor_launch,
        yopo_node,
        super_compat_node,
    ])
