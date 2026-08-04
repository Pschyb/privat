"""Small ROS logging/time shim used by the unchanged algorithm modules.

The computational code only relied on ``rospy.loginfo`` and
``rospy.Time.now``. Keeping those two calls behind this shim avoids coupling
thousands of lines of numerical code to a Node instance while all actual ROS
entities are implemented natively with rclpy.
"""

from rclpy.clock import Clock
from rclpy.logging import get_logger


def _format(message, args):
    text = str(message)
    if not args:
        return text
    try:
        return text % args
    except (TypeError, ValueError):
        return " ".join([text, *(str(arg) for arg in args)])


class _Time:
    @staticmethod
    def now():
        return Clock().now().to_msg()


class _RospyCompatibility:
    Time = _Time

    @staticmethod
    def loginfo(message, *args):
        get_logger("rose2.algorithm").info(_format(message, args))


rospy = _RospyCompatibility()
