import numpy as np
import pytest

from semantic_map_enricher.depth_filter import (
    ImageDetection,
    depth_image_to_meters,
    detections_in_distance_range,
    median_detection_distance,
    parse_spatial_detections,
)


def test_spatial_parser_requires_bounding_boxes():
    assert parse_spatial_detections("chair, table") == []
    assert parse_spatial_detections('["chair", "table"]') == []


def test_spatial_parser_accepts_xyxy_and_named_boxes():
    detections = parse_spatial_detections(
        '{"detections": ['
        '{"label": "chair", "bbox": [10, 20, 110, 220]},'
        '{"class_name": "table", "bbox": {'
        '"x_min": 200, "y_min": 100, "x_max": 500, "y_max": 400}}'
        "]}"
    )
    assert detections == [
        ImageDetection("chair", 10.0, 20.0, 110.0, 220.0),
        ImageDetection("table", 200.0, 100.0, 500.0, 400.0),
    ]


def test_spatial_parser_accepts_vision_style_center_box():
    payload = """
    objects:
      - class: person
        bbox:
          center:
            position: {x: 320, y: 240}
          size_x: 100
          size_y: 200
    """
    assert parse_spatial_detections(payload) == [
        ImageDetection("person", 270.0, 140.0, 370.0, 340.0)
    ]


def test_decode_realsense_16uc1_with_row_padding():
    raw = np.array(
        [[1000, 2000, 0, 9999], [1500, 2500, 3500, 9999]],
        dtype="<u2",
    )
    decoded = depth_image_to_meters(
        raw.tobytes(), height=2, width=3, step=8, encoding="16UC1"
    )
    assert decoded == pytest.approx(
        np.array([[1.0, 2.0, 0.0], [1.5, 2.5, 3.5]], dtype=np.float32)
    )


def test_decode_32fc1_keeps_metres():
    raw = np.array([[0.5, 1.25], [2.5, np.nan]], dtype="<f4")
    decoded = depth_image_to_meters(
        raw.tobytes(), height=2, width=2, step=8, encoding="32FC1"
    )
    np.testing.assert_allclose(decoded, raw, equal_nan=True)


def test_median_depth_ignores_invalid_pixels():
    depth = np.array(
        [
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 1.8, 2.0, 0.0],
            [0.0, 2.2, np.nan, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float32,
    )
    detection = ImageDetection("chair", 0.0, 0.0, 4.0, 4.0)
    assert median_detection_distance(
        depth, detection, roi_fraction=1.0, min_valid_pixels=3
    ) == pytest.approx(2.0)


def test_only_detections_inside_distance_range_are_returned():
    depth = np.zeros((6, 12), dtype=np.float32)
    depth[:, 0:4] = 0.8
    depth[:, 4:8] = 2.5
    depth[:, 8:12] = 4.2
    detections = [
        ImageDetection("near", 0, 0, 4, 6),
        ImageDetection("accepted", 4, 0, 8, 6),
        ImageDetection("far", 8, 0, 12, 6),
    ]
    accepted = detections_in_distance_range(
        detections,
        depth,
        minimum_m=1.0,
        maximum_m=3.0,
        roi_fraction=1.0,
        min_valid_pixels=5,
    )
    assert accepted == [(detections[1], pytest.approx(2.5))]
