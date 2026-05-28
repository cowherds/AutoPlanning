from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hover_thrust = LaunchConfiguration("hover_thrust")

    controller_node = Node(
        package="so3_control",
        executable="network_control_node",
        name="network_controller_node",
        output="screen",
        parameters=[{
            "is_simulation": False,
            "hover_thrust": hover_thrust,
            "kx_xy": 5.7,
            "kx_z": 6.2,
            "kv_xy": 3.4,
            "kv_z": 4.0,
        }],
        remappings=[
            ("odom", "/vins_estimator/imu_propagate"),
            ("imu", "/mavros/imu/data_raw"),
            ("position_cmd", "/so3_control/pos_cmd"),
            ("so3_cmd", "so3_cmd"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("hover_thrust", default_value="0.38"),
        controller_node,
    ])
