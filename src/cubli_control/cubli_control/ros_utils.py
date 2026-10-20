"""Small rclpy helpers shared by the nodes."""

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException


def spin_until_shutdown(node):
    """Spin until Ctrl+C / SIGTERM, without a traceback on the shutdown race.

    If the signal lands while spin() is building its next wait set, rclpy raises a plain
    RCLError instead of ExternalShutdownException; that is a normal shutdown too.
    """
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if node.context.ok():
            raise
