import os
import time


class RosCompatError(RuntimeError):
    pass


def detect_ros_version(force_version=None):
    """This repository targets ROS2 only."""
    requested = (force_version or os.environ.get("YOPO_ROS_VERSION", "ros2")).strip().lower()
    if requested not in {"auto", "ros2"}:
        raise RosCompatError(
            f"Invalid YOPO_ROS_VERSION / --ros_version '{requested}'; this tree supports ros2 only."
        )
    try:
        import rclpy  # noqa: F401
    except Exception as e:
        raise RosCompatError("ROS2 (rclpy) is required but not available.") from e
    return "ros2"


class Ros2Adapter:
    def __init__(self, node_name):
        import rclpy
        from rclpy.node import Node

        rclpy.init(args=None)
        self._rclpy = rclpy
        self.node = Node(node_name)

    def create_publisher(self, msg_type, topic, queue_size=1):
        return self.node.create_publisher(msg_type, topic, queue_size)

    def create_subscription(self, msg_type, topic, callback, queue_size=1):
        return self.node.create_subscription(msg_type, topic, callback, queue_size)

    def create_timer(self, period_sec, callback):
        return self.node.create_timer(period_sec, lambda: callback(None))

    def now(self):
        return self.node.get_clock().now().to_msg()

    @staticmethod
    def sleep(sec):
        time.sleep(sec)

    def spin(self):
        try:
            self._rclpy.spin(self.node)
        finally:
            self.node.destroy_node()
            self._rclpy.shutdown()

    def logwarn(self, msg):
        self.node.get_logger().warning(msg)

    @staticmethod
    def has_connections(pub):
        return pub.get_subscription_count() > 0


def make_ros_adapter(node_name, force_version=None):
    ros_version = detect_ros_version(force_version=force_version)
    return ros_version, Ros2Adapter(node_name)


def import_point_cloud2(ros_version):
    from sensor_msgs_py import point_cloud2
    return point_cloud2


def import_position_command(ros_version):
    from quadrotor_msgs.msg import PositionCommand
    return PositionCommand
