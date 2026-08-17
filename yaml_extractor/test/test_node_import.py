def test_node_imports_with_generated_rose2_interfaces():
    from yaml_extractor.live_segmented_map_exporter import (
        LiveSegmentedMapExporter,
        main,
    )

    assert LiveSegmentedMapExporter is not None
    assert callable(main)
