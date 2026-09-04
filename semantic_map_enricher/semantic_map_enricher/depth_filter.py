"""Parse image-space detections and measure their RealSense depth."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable

import numpy as np
import yaml


_CONTAINER_KEYS = ("detections", "objects", "predictions", "results")
_LABEL_KEYS = ("class_name", "class", "label", "name")


@dataclass(frozen=True)
class ImageDetection:
    """One YOLO detection with an axis-aligned pixel bounding box."""

    label: str
    x_min: float
    y_min: float
    x_max: float
    y_max: float


def _number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _label(value: Any) -> str | None:
    if not isinstance(value, str):
        return None
    cleaned = " ".join(value.strip().split())
    return cleaned or None


def _xyxy_from_mapping(value: dict[str, Any]) -> tuple[float, ...] | None:
    key_sets = (
        ("x_min", "y_min", "x_max", "y_max"),
        ("xmin", "ymin", "xmax", "ymax"),
        ("x1", "y1", "x2", "y2"),
        ("left", "top", "right", "bottom"),
    )
    for keys in key_sets:
        if not all(key in value for key in keys):
            continue
        numbers = tuple(_number(value[key]) for key in keys)
        if all(number is not None for number in numbers):
            return numbers  # type: ignore[return-value]

    if all(key in value for key in ("x", "y", "width", "height")):
        x = _number(value["x"])
        y = _number(value["y"])
        width = _number(value["width"])
        height = _number(value["height"])
        if None not in (x, y, width, height):
            assert x is not None and y is not None
            assert width is not None and height is not None
            return x, y, x + width, y + height

    size_x = _number(value.get("size_x", value.get("width")))
    size_y = _number(value.get("size_y", value.get("height")))
    center = value.get("center")
    if isinstance(center, dict):
        position = center.get("position", center)
        if isinstance(position, dict):
            center_x = _number(position.get("x"))
            center_y = _number(position.get("y"))
        else:
            center_x = center_y = None
    else:
        center_x = _number(value.get("center_x"))
        center_y = _number(value.get("center_y"))
    if None not in (center_x, center_y, size_x, size_y):
        assert center_x is not None and center_y is not None
        assert size_x is not None and size_y is not None
        return (
            center_x - size_x / 2.0,
            center_y - size_y / 2.0,
            center_x + size_x / 2.0,
            center_y + size_y / 2.0,
        )
    return None


def _bounding_box(value: dict[str, Any]) -> tuple[float, ...] | None:
    for key in ("bbox", "box", "xyxy"):
        raw_box = value.get(key)
        if isinstance(raw_box, (list, tuple)) and len(raw_box) == 4:
            numbers = tuple(_number(item) for item in raw_box)
            if all(number is not None for number in numbers):
                return numbers  # type: ignore[return-value]
        if isinstance(raw_box, dict):
            parsed = _xyxy_from_mapping(raw_box)
            if parsed is not None:
                return parsed
    return _xyxy_from_mapping(value)


def _detection_values(value: Any) -> Iterable[dict[str, Any]]:
    if isinstance(value, (list, tuple)):
        for item in value:
            yield from _detection_values(item)
        return
    if not isinstance(value, dict):
        return
    if any(key in value for key in _LABEL_KEYS):
        yield value
        return
    for key in _CONTAINER_KEYS:
        if key in value:
            yield from _detection_values(value[key])


def parse_spatial_detections(payload: str) -> list[ImageDetection]:
    """Parse YOLO labels plus pixel ``bbox`` values from JSON/YAML text.

    The preferred bounding-box form is ``[x_min, y_min, x_max, y_max]``.
    Common named-coordinate and centre/size mappings are accepted as well.
    Detections without a label or bounding box are deliberately omitted.
    """
    try:
        decoded = yaml.safe_load(payload)
    except yaml.YAMLError:
        return []

    detections: list[ImageDetection] = []
    for value in _detection_values(decoded):
        label = next(
            (
                cleaned
                for key in _LABEL_KEYS
                if key in value
                if (cleaned := _label(value[key])) is not None
            ),
            None,
        )
        box = _bounding_box(value)
        if label is None or box is None:
            continue
        x_min, y_min, x_max, y_max = box
        left, right = sorted((float(x_min), float(x_max)))
        top, bottom = sorted((float(y_min), float(y_max)))
        if right <= left or bottom <= top:
            continue
        detections.append(ImageDetection(label, left, top, right, bottom))
    return detections


def depth_image_to_meters(
    data: Any,
    height: int,
    width: int,
    step: int,
    encoding: str,
    is_bigendian: bool = False,
) -> np.ndarray:
    """Decode a ``sensor_msgs/Image`` depth buffer into metres.

    RealSense Z16 images arrive through ROS as ``16UC1`` millimetres. Float
    depth images use ``32FC1`` metres. Row padding described by ``step`` is
    respected so the function does not depend on cv_bridge.
    """
    if height <= 0 or width <= 0:
        raise ValueError("depth image dimensions must be positive")

    normalized_encoding = encoding.strip().lower()
    endian = ">" if is_bigendian else "<"
    if normalized_encoding in ("16uc1", "mono16"):
        dtype = np.dtype(f"{endian}u2")
        scale = 0.001
    elif normalized_encoding == "32fc1":
        dtype = np.dtype(f"{endian}f4")
        scale = 1.0
    else:
        raise ValueError(
            f"unsupported depth encoding {encoding!r}; expected 16UC1 or 32FC1"
        )

    row_step = int(step) or int(width) * dtype.itemsize
    minimum_step = int(width) * dtype.itemsize
    if row_step < minimum_step:
        raise ValueError("depth image step is smaller than one pixel row")
    required_bytes = (int(height) - 1) * row_step + minimum_step
    buffer = memoryview(data)
    if buffer.nbytes < required_bytes:
        raise ValueError("depth image data is shorter than dimensions and step")

    pixels = np.ndarray(
        shape=(int(height), int(width)),
        dtype=dtype,
        buffer=buffer,
        strides=(row_step, dtype.itemsize),
    )
    return pixels.astype(np.float32) * scale


def median_detection_distance(
    depth_m: np.ndarray,
    detection: ImageDetection,
    roi_fraction: float = 0.5,
    min_valid_pixels: int = 20,
) -> float | None:
    """Return median valid depth in the central part of a detection box."""
    if depth_m.ndim != 2:
        raise ValueError("depth image must be a two-dimensional array")
    if not 0.0 < roi_fraction <= 1.0:
        raise ValueError("roi_fraction must be in the interval (0, 1]")
    if min_valid_pixels <= 0:
        raise ValueError("min_valid_pixels must be positive")

    center_x = (detection.x_min + detection.x_max) / 2.0
    center_y = (detection.y_min + detection.y_max) / 2.0
    half_width = (detection.x_max - detection.x_min) * roi_fraction / 2.0
    half_height = (detection.y_max - detection.y_min) * roi_fraction / 2.0

    height, width = depth_m.shape
    x_min = max(0, min(width, math.floor(center_x - half_width)))
    x_max = max(0, min(width, math.ceil(center_x + half_width)))
    y_min = max(0, min(height, math.floor(center_y - half_height)))
    y_max = max(0, min(height, math.ceil(center_y + half_height)))
    if x_max <= x_min or y_max <= y_min:
        return None

    region = depth_m[y_min:y_max, x_min:x_max]
    valid = region[np.isfinite(region) & (region > 0.0)]
    if valid.size < min_valid_pixels:
        return None
    return float(np.median(valid))


def detections_in_distance_range(
    detections: Iterable[ImageDetection],
    depth_m: np.ndarray,
    minimum_m: float,
    maximum_m: float,
    roi_fraction: float = 0.5,
    min_valid_pixels: int = 20,
) -> list[tuple[ImageDetection, float]]:
    """Return detections with a valid median depth inside an inclusive range."""
    accepted: list[tuple[ImageDetection, float]] = []
    for detection in detections:
        distance = median_detection_distance(
            depth_m,
            detection,
            roi_fraction=roi_fraction,
            min_valid_pixels=min_valid_pixels,
        )
        if distance is not None and minimum_m <= distance <= maximum_m:
            accepted.append((detection, distance))
    return accepted
