from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    simulator = Node(
        package="so3_quadrotor_simulator",
        executable="quadrotor_simulator_so3",
        name="quadrotor_simulator_so3",
        output="screen",
        parameters=[
            {
                "rate.odom": 100.0,
                "simulator.init_state_x": 0.0,
                "simulator.init_state_y": 0.0,
                "simulator.init_state_z": 2.0,
            }
        ],
        remappings=[
            ("odom", "/sim/odom"),
            ("imu", "/sim/imu"),
            ("cmd", "so3_cmd"),
            ("force_disturbance", "force_disturbance"),
            ("moment_disturbance", "moment_disturbance"),
        ],
    )

    controller = Node(
        package="so3_control",
        executable="network_control_node",
        name="network_controller_node",
        output="screen",
        parameters=[
            {
                "is_simulation": True,
                "hover_thrust": 0.375,
                "kx_xy": 5.7,
                "kx_z": 6.2,
                "kv_xy": 3.4,
                "kv_z": 4.0,
            }
        ],
        remappings=[
            ("odom", "/sim/odom"),
            ("imu", "/sim/imu"),
            ("position_cmd", "/so3_control/pos_cmd"),
            ("so3_cmd", "so3_cmd"),
        ],
    )

    return LaunchDescription([simulator, controller])
