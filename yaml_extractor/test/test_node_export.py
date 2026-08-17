import array
import pickle

import pytest
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from shapely.geometry import Polygon
from yaml_extractor.live_segmented_map_exporter import LiveSegmentedMapExporter

from rose2.msg import Room, ROSE2Features


@pytest.fixture
def exporter_node():
    rclpy.init()
    node = LiveSegmentedMapExporter()
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_ros_messages_are_exported_to_expected_yaml(exporter_node, tmp_path):
    source_map = OccupancyGrid()
    source_map.info.width = 20
    source_map.info.height = 10
    source_map.info.resolution = 0.5
    source_map.info.origin.position.x = -2.0
    source_map.info.origin.position.y = -1.0
    source_map.info.origin.orientation.w = 1.0

    polygon = Polygon([(0, 0), (4, 0), (4, 3), (0, 3)])
    payload = array.array("b")
    payload.frombytes(pickle.dumps(polygon, protocol=pickle.HIGHEST_PROTOCOL))
    room = Room()
    room.bytes = payload
    features = ROSE2Features()
    features.rooms = [room]

    output_path = tmp_path / "map15_segments_from_rose2.yaml"
    exporter_node.latest_map = source_map
    exporter_node.latest_features = features
    exporter_node.map_yaml = "/maps/map15.yaml"
    exporter_node.output_yaml = str(output_path)

    assert exporter_node._write_yaml() == str(output_path)
    with output_path.open(encoding="utf-8") as handle:
        document = yaml.safe_load(handle)

    assert document["map_yaml"] == "/maps/map15.yaml"
    assert document["origin"] == [-2.0, -1.0, 0.0]
    assert document["resolution"] == 0.5
    assert document["segmentation_method"] == "rose2_live"
    assert len(document["segments"]) == 1
    assert document["segments"][0]["id"] == 1
    assert document["segments"][0]["area_cells"] == 12
    assert document["segments"][0]["area_m2"] == pytest.approx(3.0)
    assert document["segments"][0]["center_m"] == pytest.approx([-1.0, -0.25])
