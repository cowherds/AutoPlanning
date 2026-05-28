from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sensor_node = Node(
        package="sensor_simulator",
        executable="sensor_simulator",
        name="sensor_simulator_node_ros2",
        output="screen",
    )
    return LaunchDescription([sensor_node])
