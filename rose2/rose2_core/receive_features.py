"""Example ROS 2 subscriber that prints decoded ROSE2 features."""

import array
import pickle
from pprint import pprint

import rclpy
from rclpy.node import Node

from rose2.msg import ROSE2Features


def _unpickle(values):
    return pickle.loads(array.array("b", values).tobytes())


class ReceiverFeatures(Node):
    def __init__(self):
        super().__init__("rose2_feature_receiver")
        self.subscription = self.create_subscription(
            ROSE2Features, "features_ROSE2", self.process_features, 1
        )
        self.get_logger().info("Waiting for ROSE2 features...")

    def process_features(self, features):
        print("EXTENDED LINES")
        for line in features.lines:
            pprint(vars(_unpickle(line.bytes)))
        print("EDGES")
        for edge in features.edges:
            pprint(vars(_unpickle(edge.bytes)))
        print("ROOMS")
        for room in features.rooms:
            print(_unpickle(room.bytes))
        print("CONTOUR")
        print(features.contour)


def main(args=None):
    rclpy.init(args=args)
    node = ReceiverFeatures()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
