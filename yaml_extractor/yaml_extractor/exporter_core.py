"""ROS-independent conversion helpers for the ROSE2 YAML exporter."""

from __future__ import annotations

import math
import os
import pickle
from typing import Any


def resolve_output_yaml(output_yaml: str, map_yaml: str) -> str:
    """Return the explicit output path or derive one beside ``map_yaml``."""
    if output_yaml:
        return output_yaml

    base_dir = os.path.dirname(map_yaml)
    base_name = os.path.splitext(os.path.basename(map_yaml))[0]
    file_name = f"{base_name}_segments_from_rose2.yaml"
    return os.path.join(base_dir, file_name) if base_dir else file_name


def yaw_from_quaternion(quaternion: Any) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def grid_point_to_map(
    px: float,
    py: float,
    resolution: float,
    origin: Any,
) -> list[float]:
    """Transform a ROSE2 grid point into the map frame.

    The current ROS 2 port keeps polygons in OccupancyGrid coordinates, where
    both x and y increase with the map frame.  There is therefore no image-y
    flip.  The OccupancyGrid origin rotation is applied as well.
    """
    local_x = float(px) * float(resolution)
    local_y = float(py) * float(resolution)
    yaw = yaw_from_quaternion(origin.orientation)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    map_x = float(origin.position.x) + cosine * local_x - sine * local_y
    map_y = float(origin.position.y) + sine * local_x + cosine * local_y
    return [float(map_x), float(map_y)]


def polygon_pixel_points(polygon: Any) -> list[tuple[float, float]]:
    x_values, y_values = polygon.exterior.coords.xy
    return [
        (float(x_value), float(y_value)) for x_value, y_value in zip(x_values, y_values)
    ]


def polygon_map_points(
    polygon: Any,
    resolution: float,
    origin: Any,
) -> list[list[float]]:
    return [
        grid_point_to_map(x_value, y_value, resolution, origin)
        for x_value, y_value in polygon_pixel_points(polygon)
    ]


def bounding_corners_map(
    polygon: Any,
    resolution: float,
    origin: Any,
) -> list[list[float]]:
    min_x, min_y, max_x, max_y = polygon.bounds
    corners = [
        (min_x, max_y),
        (max_x, max_y),
        (max_x, min_y),
        (min_x, min_y),
    ]
    return [
        grid_point_to_map(x_value, y_value, resolution, origin)
        for x_value, y_value in corners
    ]


def decode_room(room_message: Any) -> Any:
    """Decode the signed-int8 pickle payload used by ``rose2/msg/Room``."""
    values = room_message.bytes
    if hasattr(values, "tobytes"):
        payload = values.tobytes()
    else:
        payload = bytes(int(value) & 0xFF for value in values)
    return pickle.loads(payload)


def build_segment(
    polygon: Any,
    index: int,
    resolution: float,
    origin: Any,
) -> dict[str, Any]:
    """Build one segment using the schema of the original exporter node."""
    room_id = (int(index) % 99) + 1
    area_cells_float = float(polygon.area)
    area_cells = max(0, round(area_cells_float))
    centroid = polygon.centroid
    centroid_x = float(centroid.x)
    centroid_y = float(centroid.y)

    return {
        "id": room_id,
        "area_cells": area_cells,
        "area_m2": float(area_cells * resolution * resolution),
        "center_px": [centroid_x, centroid_y],
        "center_m": grid_point_to_map(centroid_x, centroid_y, resolution, origin),
        "corners_m": bounding_corners_map(polygon, resolution, origin),
        "polygon_m": polygon_map_points(polygon, resolution, origin),
        "room_name": "unknown",
        "objects": [],
    }
