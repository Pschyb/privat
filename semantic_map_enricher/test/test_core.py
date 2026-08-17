import math

import pytest
import yaml

from semantic_map_enricher.core import (
    SemanticMapModel,
    atomic_write_yaml,
    find_containing_segment,
    parse_detected_objects,
    point_in_polygon,
    rotate_point_by_quaternion,
)


def make_document():
    return {
        "segments": [
            {
                "id": 1,
                "polygon_m": [[0, 0], [4, 0], [4, 4], [0, 4], [0, 0]],
                "center_m": [2, 2],
                "objects": [],
            },
            {
                "id": 2,
                "polygon_m": [[5, 0], [9, 0], [9, 4], [5, 4], [5, 0]],
                "center_m": [7, 2],
                "objects": [],
            },
        ]
    }


def test_object_parser_accepts_yolo_string_encodings():
    assert parse_detected_objects("chair, table; Chair") == ["chair", "table"]
    assert parse_detected_objects('["chair", "dining table"]') == [
        "chair",
        "dining table",
    ]
    assert parse_detected_objects(
        '{"detections": [{"class_name": "chair", "confidence": 0.9}, '
        '{"label": "table"}]}'
    ) == ["chair", "table"]
    assert parse_detected_objects('{"chair": 0.91, "table": 0.84}') == [
        "chair",
        "table",
    ]


def test_point_in_polygon_includes_boundary():
    polygon = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
    assert point_in_polygon(2.0, 2.0, polygon)
    assert point_in_polygon(0.0, 2.0, polygon)
    assert not point_in_polygon(4.1, 2.0, polygon)


def test_quaternion_rotation_for_tf_pose_conversion():
    yaw = math.pi / 2.0
    rotated = rotate_point_by_quaternion(
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0),
    )
    assert rotated == pytest.approx((0.0, 1.0, 0.0))


def test_room_lookup_has_no_nearest_room_fallback():
    segments = make_document()["segments"]
    assert find_containing_segment(segments, 1.0, 1.0)[0] == 0
    assert find_containing_segment(segments, 6.0, 1.0)[0] == 1
    assert find_containing_segment(segments, 4.5, 1.0) == (None, None)


def test_room_change_resets_session_without_copying_old_objects():
    document = make_document()
    model = SemanticMapModel(document)

    transition = model.update_position(1.0, 1.0)
    assert transition.current_id == 1
    assert model.add_detection_payload("chair") == ["chair"]
    assert model.add_detection_payload("chair") == []

    transition = model.update_position(6.0, 1.0)
    assert transition.changed
    assert transition.previous_id == 1
    assert transition.current_id == 2
    # Merely changing the pose cannot carry the cached chair into room 2.
    assert document["segments"][1]["objects"] == []
    assert model.add_detection_payload("table") == ["table"]

    assert document["segments"][0]["objects"] == ["chair"]
    assert document["segments"][1]["objects"] == ["table"]


def test_outside_position_clears_room_history_and_blocks_detections():
    model = SemanticMapModel(make_document())
    model.update_position(1.0, 1.0)
    model.add_detection_payload("chair")

    transition = model.update_position(4.5, 1.0)
    assert transition.current_id is None
    assert model.add_detection_payload("person") == []

    transition = model.update_position(6.0, 1.0)
    assert transition.current_id == 2
    assert model.add_detection_payload("person") == ["person"]


def test_legacy_object_string_is_migrated_to_list():
    document = make_document()
    document["segments"][0]["objects"] = "chair, table"
    model = SemanticMapModel(document)
    assert model.dirty
    assert document["segments"][0]["objects"] == ["chair", "table"]


def test_atomic_yaml_write(tmp_path):
    output = tmp_path / "semantic.yaml"
    document = make_document()
    atomic_write_yaml(str(output), document)
    with output.open(encoding="utf-8") as handle:
        assert yaml.safe_load(handle) == document
