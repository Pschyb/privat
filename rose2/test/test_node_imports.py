from types import SimpleNamespace

import rclpy


def test_ros2_nodes_import():
    from rose2_core.feature_extractor_rose import FeatureExtractorROSE
    from rose2_core.feature_extractor_rose2 import FeatureExtractorROSE2

    assert FeatureExtractorROSE.__name__ == "FeatureExtractorROSE"
    assert FeatureExtractorROSE2.__name__ == "FeatureExtractorROSE2"


def test_ros2_nodes_construct_and_destroy():
    from rose2_core.feature_extractor_rose import FeatureExtractorROSE
    from rose2_core.feature_extractor_rose2 import FeatureExtractorROSE2

    rose_node = None
    rose2_node = None
    rclpy.init(args=[])
    try:
        rose_node = FeatureExtractorROSE()
        rose2_node = FeatureExtractorROSE2()
        assert rose_node.get_name() == "rose"
        assert rose2_node.get_name() == "rose2"
        assert rose2_node.pub_once is False
        assert rose2_node.feature_publisher.topic_name == "/features_ROSE2"
        assert (
            rose2_node.feature_publisher_lowercase.topic_name
            == "/features_rose2"
        )
        assert rose2_node.room_publisher.topic_name == "/rooms"
        assert rose2_node.status_publisher.topic_name == "/rose2/status"
        assert rose2_node.status == "waiting_for_rose_features"
    finally:
        if rose2_node is not None:
            rose2_node.destroy_node()
        if rose_node is not None:
            rose_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_rose2_results_are_republished_on_both_feature_topics():
    from rose2.msg import ROSE2Features
    from rose2_core.feature_extractor_rose2 import FeatureExtractorROSE2
    from visualization_msgs.msg import Marker, MarkerArray

    recorders = {
        name: Recorder()
        for name in (
            "edge_publisher",
            "line_publisher",
            "room_publisher",
            "feature_publisher",
            "feature_publisher_lowercase",
        )
    }
    status_publications = []
    node = SimpleNamespace(
        **recorders,
        _publish_status=lambda: status_publications.append(True),
        publish_pending=True,
        output_features=ROSE2Features(),
        edge_markers=MarkerArray(),
        line_marker=Marker(),
        room_markers=MarkerArray(),
        pub_once=False,
    )

    FeatureExtractorROSE2._publish_if_ready(node)
    FeatureExtractorROSE2._publish_if_ready(node)

    assert len(status_publications) == 2
    assert node.publish_pending is True
    assert len(node.feature_publisher.messages) == 2
    assert len(node.feature_publisher_lowercase.messages) == 2
    assert len(node.room_publisher.messages) == 2

    node.pub_once = True
    FeatureExtractorROSE2._publish_if_ready(node)
    assert node.publish_pending is False
