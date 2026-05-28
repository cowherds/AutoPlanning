from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


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
    odom_topic = LaunchConfiguration("odom_topic")
    lidar_topic = LaunchConfiguration("lidar_topic")
    ctrl_topic = LaunchConfiguration("ctrl_topic")
    world_frame_cloud = LaunchConfiguration("world_frame_cloud")
    use_default_goal = LaunchConfiguration("use_default_goal")
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
    super_root = LaunchConfiguration("super_root")

    return LaunchDescription(
        [
            DeclareLaunchArgument("odom_topic", default_value="/sim/odom"),
            DeclareLaunchArgument(
                "lidar_topic",
                default_value="/lidar_points",
                description="Body-frame cloud: /lidar_points; world-frame: /lidar_points_world.",
            ),
            DeclareLaunchArgument("ctrl_topic", default_value="/so3_control/pos_cmd"),
            DeclareLaunchArgument(
                "world_frame_cloud",
                default_value="false",
                description="Must match lidar_topic frame (false=/lidar_points, true=/lidar_points_world).",
            ),
            DeclareLaunchArgument("use_default_goal", default_value="false"),
            DeclareLaunchArgument("flight_height", default_value="3.0"),
            DeclareLaunchArgument("max_speed", default_value="10.0"),
            DeclareLaunchArgument("goal_tolerance", default_value="0.8"),
            DeclareLaunchArgument("map_resolution", default_value="0.35"),
            DeclareLaunchArgument("planning_horizon", default_value="16.0"),
            DeclareLaunchArgument("obstacle_inflation", default_value="0.7"),
            DeclareLaunchArgument("map_decay_sec", default_value="2.0"),
            DeclareLaunchArgument("start_clearance", default_value="1.0"),
            DeclareLaunchArgument(
                "mission_file",
                default_value="__none__",
                description="Optional mission file; use __none__ to disable mission mode.",
            ),
            DeclareLaunchArgument(
                "super_param_file",
                default_value=_DEFAULT_SUPER_PARAM,
                description="ROS2 params file for super_compat_planner.",
            ),
            DeclareLaunchArgument(
                "super_root",
                default_value=_DEFAULT_SUPER,
                description="Absolute path to the policy/super/ folder.",
            ),
            ExecuteProcess(
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
                        "'; else echo '[super_compat_only.launch] Missing controller setup.bash' >&2; exit 1; fi",
                        " && if [[ '",
                        world_frame_cloud,
                        "' == 'true' && '",
                        lidar_topic,
                        "' != '/lidar_points_world' ]]; then echo '[super_compat_only.launch] world_frame_cloud=true requires /lidar_points_world' >&2; exit 1; fi",
                        " && if [[ '",
                        world_frame_cloud,
                        "' == 'false' && '",
                        lidar_topic,
                        "' == '/lidar_points_world' ]]; then echo '[super_compat_only.launch] /lidar_points_world requires world_frame_cloud=true' >&2; exit 1; fi",
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
                        " -p use_default_goal:=",
                        use_default_goal,
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
            ),
        ]
    )
