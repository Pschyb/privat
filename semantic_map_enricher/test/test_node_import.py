def test_go2_mapper_imports_with_generated_ros_interfaces():
    from semantic_map_enricher.go2_object_mapper import Go2ObjectMapper

    assert Go2ObjectMapper.__name__ == "Go2ObjectMapper"
