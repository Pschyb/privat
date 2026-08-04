def test_ros2_nodes_import():
    from rose2_core.feature_extractor_rose import FeatureExtractorROSE
    from rose2_core.feature_extractor_rose2 import FeatureExtractorROSE2

    assert FeatureExtractorROSE.__name__ == "FeatureExtractorROSE"
    assert FeatureExtractorROSE2.__name__ == "FeatureExtractorROSE2"
