import array
import math
import pickle
from types import SimpleNamespace

import pytest
from yaml_extractor.exporter_core import (
    build_segment,
    decode_room,
    grid_point_to_map,
    resolve_output_yaml,
)


class FakeCoordinates:
    @property
    def xy(self):
        return ([0.0, 4.0, 4.0, 0.0, 0.0], [0.0, 0.0, 3.0, 3.0, 0.0])


class FakePolygon:
    area = 12.0
    bounds = (0.0, 0.0, 4.0, 3.0)
    centroid = SimpleNamespace(x=2.0, y=1.5)
    exterior = SimpleNamespace(coords=FakeCoordinates())


def make_origin(x=-20.0, y=-20.0, yaw=0.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y),
        orientation=SimpleNamespace(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        ),
    )


def test_build_segment_preserves_original_yaml_schema_without_y_flip():
    segment = build_segment(FakePolygon(), 0, 0.05, make_origin())

    assert segment["id"] == 1
    assert segment["area_cells"] == 12
    assert segment["area_m2"] == pytest.approx(0.03)
    assert segment["center_px"] == [2.0, 1.5]
    assert segment["center_m"] == pytest.approx([-19.9, -19.925])
    assert segment["corners_m"][0] == pytest.approx([-20.0, -19.85])
    assert segment["polygon_m"][0] == pytest.approx([-20.0, -20.0])
    assert segment["room_name"] == "unknown"
    assert segment["objects"] == []


def test_grid_point_to_map_applies_origin_yaw():
    point = grid_point_to_map(
        2.0,
        3.0,
        0.5,
        make_origin(x=10.0, y=-2.0, yaw=math.pi / 2.0),
    )
    assert point == pytest.approx([8.5, -1.0])


def test_decode_room_accepts_signed_int8_ros_payload():
    expected = {"room": 7}
    payload = array.array("b")
    payload.frombytes(pickle.dumps(expected, protocol=pickle.HIGHEST_PROTOCOL))
    message = SimpleNamespace(bytes=payload)
    assert decode_room(message) == expected


def test_output_path_is_derived_beside_map_yaml():
    assert resolve_output_yaml("", "/maps/map15.yaml") == (
        "/maps/map15_segments_from_rose2.yaml"
    )
    assert resolve_output_yaml("/tmp/result.yaml", "/maps/map15.yaml") == (
        "/tmp/result.yaml"
    )
