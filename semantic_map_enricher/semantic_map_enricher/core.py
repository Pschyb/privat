"""ROS-independent helpers for assigning object labels to ROSE2 rooms."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import re
import tempfile
from typing import Any, Iterable

import yaml


_CONTAINER_KEYS = (
    "objects",
    "detections",
    "labels",
    "class_names",
    "classes",
    "predictions",
    "results",
)
_LABEL_KEYS = ("class_name", "class", "label", "name")
_SEPARATOR = re.compile(r"[,;\n]+")


def _clean_label(value: Any) -> str | None:
    if not isinstance(value, str):
        return None
    label = " ".join(value.strip().split())
    return label or None


def _labels_from_value(value: Any) -> list[str]:
    if value is None:
        return []

    if isinstance(value, str):
        return [
            label
            for part in _SEPARATOR.split(value)
            if (label := _clean_label(part)) is not None
        ]

    if isinstance(value, (list, tuple, set)):
        labels: list[str] = []
        for item in value:
            labels.extend(_labels_from_value(item))
        return labels

    if isinstance(value, dict):
        for key in _LABEL_KEYS:
            if key in value:
                label = _clean_label(value[key])
                if label is not None:
                    return [label]

        labels = []
        for key in _CONTAINER_KEYS:
            if key in value:
                labels.extend(_labels_from_value(value[key]))
        if labels:
            return labels

        # Also accept a compact confidence mapping such as
        # {"chair": 0.91, "table": 0.84}.
        if value and all(
            isinstance(item, (int, float, bool)) for item in value.values()
        ):
            return [
                label
                for key in value
                if (label := _clean_label(key)) is not None
            ]

    return []


def deduplicate_labels(labels: Iterable[str]) -> list[str]:
    """Deduplicate labels case-insensitively while preserving their order."""
    unique: list[str] = []
    seen: set[str] = set()
    for value in labels:
        label = _clean_label(value)
        if label is None:
            continue
        key = label.casefold()
        if key in seen:
            continue
        seen.add(key)
        unique.append(label)
    return unique


def parse_detected_objects(payload: str) -> list[str]:
    """Parse common ``std_msgs/String`` encodings used by YOLO nodes.

    Supported payloads include one label, comma/semicolon/newline-separated
    labels, YAML/JSON lists, and dictionaries containing ``objects`` or
    ``detections``.
    """
    text = payload.strip()
    if not text:
        return []

    try:
        decoded = yaml.safe_load(text)
    except yaml.YAMLError:
        decoded = text
    return deduplicate_labels(_labels_from_value(decoded))


def normalize_existing_objects(value: Any) -> list[str]:
    """Convert legacy string-valued ``objects`` fields to YAML lists."""
    if isinstance(value, str):
        return parse_detected_objects(value)
    return deduplicate_labels(_labels_from_value(value))


def rotate_point_by_quaternion(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[float, float, float]:
    """Rotate a point by a quaternion."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError("quaternion has zero length")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def segment_polygon(segment: dict[str, Any]) -> list[tuple[float, float]] | None:
    """Return the metric ROSE2 polygon, falling back to bounding corners."""
    raw_polygon = segment.get("polygon_m")
    if not isinstance(raw_polygon, list) or len(raw_polygon) < 3:
        raw_polygon = segment.get("corners_m")
    if not isinstance(raw_polygon, list) or len(raw_polygon) < 3:
        return None

    polygon: list[tuple[float, float]] = []
    for raw_point in raw_polygon:
        if not isinstance(raw_point, (list, tuple)) or len(raw_point) != 2:
            return None
        try:
            polygon.append((float(raw_point[0]), float(raw_point[1])))
        except (TypeError, ValueError):
            return None
    return polygon


def _point_on_segment(
    x: float,
    y: float,
    start: tuple[float, float],
    end: tuple[float, float],
    epsilon: float = 1e-8,
) -> bool:
    ax, ay = start
    bx, by = end
    squared_length = (bx - ax) ** 2 + (by - ay) ** 2
    if squared_length <= epsilon * epsilon:
        return (x - ax) ** 2 + (y - ay) ** 2 <= epsilon * epsilon

    cross = (x - ax) * (by - ay) - (y - ay) * (bx - ax)
    scale = max(1.0, abs(bx - ax), abs(by - ay))
    if abs(cross) > epsilon * scale:
        return False

    dot = (x - ax) * (bx - ax) + (y - ay) * (by - ay)
    if dot < -epsilon:
        return False
    return dot <= squared_length + epsilon


def point_in_polygon(x: float, y: float, polygon: list[tuple[float, float]]) -> bool:
    """Return true for points inside or on the boundary of a polygon."""
    inside = False
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        if _point_on_segment(x, y, start, end):
            return True

        x1, y1 = start
        x2, y2 = end
        if (y1 > y) == (y2 > y):
            continue
        intersection_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
        if intersection_x >= x:
            inside = not inside
    return inside


def polygon_area(polygon: list[tuple[float, float]]) -> float:
    area_twice = 0.0
    for index, (x1, y1) in enumerate(polygon):
        x2, y2 = polygon[(index + 1) % len(polygon)]
        area_twice += x1 * y2 - x2 * y1
    return abs(area_twice) * 0.5


def find_containing_segment(
    segments: list[dict[str, Any]],
    x: float,
    y: float,
) -> tuple[int | None, dict[str, Any] | None]:
    """Find the room containing a map-frame point.

    No nearest-centre fallback is used: a robot outside every room must not
    contaminate the nearest room. If polygons overlap, the smaller polygon is
    preferred because it is the more specific region.
    """
    candidates: list[tuple[float, int, dict[str, Any]]] = []
    for index, segment in enumerate(segments):
        polygon = segment_polygon(segment)
        if polygon is None or not point_in_polygon(x, y, polygon):
            continue
        candidates.append((polygon_area(polygon), index, segment))

    if not candidates:
        return None, None
    _, index, segment = min(candidates, key=lambda item: (item[0], item[1]))
    return index, segment


def segment_id(segment: dict[str, Any], index: int) -> int:
    raw_id = segment.get("id", index + 1)
    try:
        return int(raw_id)
    except (TypeError, ValueError) as error:
        raise ValueError(f"segment id {raw_id!r} is not an integer") from error


@dataclass(frozen=True)
class RoomTransition:
    changed: bool
    previous_id: int | None
    current_id: int | None


class SemanticMapModel:
    """Mutable semantic-map state with room-local detection history."""

    def __init__(self, document: dict[str, Any]):
        if not isinstance(document, dict):
            raise ValueError("YAML root must be a mapping")
        segments = document.get("segments")
        if not isinstance(segments, list):
            raise ValueError("YAML must contain a 'segments' list")
        if not all(isinstance(segment, dict) for segment in segments):
            raise ValueError("every segment must be a mapping")

        self.document = document
        self.segments: list[dict[str, Any]] = segments
        self.current_index: int | None = None
        self.session_seen: set[str] = set()
        self.dirty = False

        for segment in self.segments:
            normalized = normalize_existing_objects(segment.get("objects", []))
            if segment.get("objects") != normalized:
                segment["objects"] = normalized
                self.dirty = True

    @property
    def current_segment(self) -> dict[str, Any] | None:
        if self.current_index is None:
            return None
        return self.segments[self.current_index]

    @property
    def current_segment_id(self) -> int | None:
        segment = self.current_segment
        if segment is None or self.current_index is None:
            return None
        return segment_id(segment, self.current_index)

    def update_position(self, x: float, y: float) -> RoomTransition:
        previous_id = self.current_segment_id
        new_index, _ = find_containing_segment(self.segments, x, y)
        changed = new_index != self.current_index
        if changed:
            self.current_index = new_index
            # This is the crucial room boundary: observations remembered for
            # the previous room are never reused in the new room.
            self.session_seen.clear()
        return RoomTransition(changed, previous_id, self.current_segment_id)

    def add_detection_payload(self, payload: str) -> list[str]:
        return self.add_objects(parse_detected_objects(payload))

    def add_objects(self, labels: Iterable[str]) -> list[str]:
        segment = self.current_segment
        if segment is None:
            return []

        objects = normalize_existing_objects(segment.get("objects", []))
        segment["objects"] = objects
        existing = {label.casefold() for label in objects}
        added: list[str] = []

        for label in deduplicate_labels(labels):
            key = label.casefold()
            if key in self.session_seen:
                continue
            self.session_seen.add(key)
            if key in existing:
                continue
            objects.append(label)
            existing.add(key)
            added.append(label)

        if added:
            self.dirty = True
        return added


def load_semantic_map(path: str) -> SemanticMapModel:
    with open(path, encoding="utf-8") as handle:
        document = yaml.safe_load(handle)
    return SemanticMapModel(document)


def atomic_write_yaml(path: str, document: dict[str, Any]) -> None:
    """Replace a YAML file atomically so interruption cannot truncate it."""
    output_dir = os.path.dirname(os.path.abspath(path))
    os.makedirs(output_dir, exist_ok=True)
    descriptor, temporary_path = tempfile.mkstemp(
        prefix=f".{os.path.basename(path)}.",
        suffix=".tmp",
        dir=output_dir,
        text=True,
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            yaml.safe_dump(document, handle, sort_keys=False, allow_unicode=True)
        os.replace(temporary_path, path)
    except Exception:
        if os.path.exists(temporary_path):
            os.unlink(temporary_path)
        raise
