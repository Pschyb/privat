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
    finally:
        if rose2_node is not None:
            rose2_node.destroy_node()
        if rose_node is not None:
            rose_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
