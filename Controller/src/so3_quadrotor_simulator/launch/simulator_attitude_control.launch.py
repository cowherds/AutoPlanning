from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    init_x = LaunchConfiguration("init_x")
    init_y = LaunchConfiguration("init_y")
    init_z = LaunchConfiguration("init_z")

    sim_node = Node(
        package="so3_quadrotor_simulator",
        executable="quadrotor_simulator_so3",
        name="quadrotor_simulator_so3",
        output="screen",
        parameters=[{
            "rate.odom": 100.0,
            "simulator.init_state_x": init_x,
            "simulator.init_state_y": init_y,
            "simulator.init_state_z": init_z,
        }],
        remappings=[
            ("odom", "/sim/odom"),
            ("imu", "/sim/imu"),
            ("cmd", "so3_cmd"),
            ("force_disturbance", "force_disturbance"),
            ("moment_disturbance", "moment_disturbance"),
        ],
    )

    controller_node = Node(
        package="so3_control",
        executable="network_control_node",
        name="network_controller_node",
        output="screen",
        parameters=[{
            "is_simulation": True,
            "hover_thrust": 0.375,
            "kx_xy": 5.7,
            "kx_z": 6.2,
            "kv_xy": 3.4,
            "kv_z": 4.0,
        }],
        remappings=[
            ("odom", "/sim/odom"),
            ("imu", "/sim/imu"),
            ("position_cmd", "/so3_control/pos_cmd"),
            ("so3_cmd", "so3_cmd"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("init_x", default_value="0.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_z", default_value="2.0"),
        sim_node,
        controller_node,
    ])
