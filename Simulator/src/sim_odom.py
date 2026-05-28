#!/usr/bin/env python3
"""Simple test odometry publisher (ROS2) — use quadrotor_simulator_so3 in real tests."""
import math

import rclpy
from geometry_msgs.msg import Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")
        self._pub = self.create_publisher(Odometry, "/sim/odom", 10)
        self._timer = self.create_timer(1.0 / 30.0, self._tick)
        self._start = self.get_clock().now()
        self._y = 2.0
        self._z = 1.6
        self._velocity = 2.0

    def _tick(self):
        now = self.get_clock().now()
        elapsed = (now - self._start).nanoseconds * 1e-9
        x = self._velocity * elapsed
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position = Point(x=x, y=self._y, z=self._z)
        msg.pose.pose.orientation = euler_to_quat(0.0, 0.0, 0.0)
        msg.twist.twist.linear = Vector3(x=self._velocity, y=0.0, z=0.0)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
