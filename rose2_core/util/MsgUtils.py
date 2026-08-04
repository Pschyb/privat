"""Conversion and visualization helpers for ROSE2 ROS messages."""

import array
import colorsys
import pickle
import random
from copy import deepcopy

import numpy as np
from geometry_msgs.msg import Point, Point32, Polygon, PolygonStamped, Quaternion
from nav_msgs.msg import MapMetaData, OccupancyGrid
from scipy.spatial.transform import Rotation
from shapely.geometry.polygon import orient
from shapely.ops import triangulate
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from rose2.msg import Contour, Edge, ExtendedLine, Room
from rose2_core.my_grid_map import MyGridMap
from rose2_core.ros_compat import rospy


def from_occupancy_grid_to_image(occupancy_grid):
    """Convert a standard trinary OccupancyGrid to a grayscale image."""
    height = int(occupancy_grid.info.height)
    width = int(occupancy_grid.info.width)
    data = np.asarray(occupancy_grid.data, dtype=np.int16).reshape(height, width)
    image = np.full((height, width), 200, dtype=np.uint8)
    image[data == 0] = 255
    image[data >= 50] = 0
    return image


def from_image_to_occupancy_grid(image, origin, resolution, stamp=None):
    """Convert a binary wall image to a valid trinary OccupancyGrid.

    Non-zero pixels represent structural walls and become occupied cells
    (100); zero pixels become free cells (0). This avoids putting unsigned
    image values such as 255 into OccupancyGrid's signed int8 data field.
    """
    image = np.asarray(image)
    grid = OccupancyGrid()
    grid.header = Header(frame_id="map")
    if stamp is not None:
        grid.header.stamp = stamp
    grid.info = MapMetaData()
    grid.info.resolution = float(resolution)
    grid.info.height = image.shape[0]
    grid.info.width = image.shape[1]
    grid.info.origin = origin
    grid.data = np.where(image > 0, 100, 0).astype(np.int8).ravel().tolist()
    return grid


def fromOccupancyGridToImg(occupancy_grid):
    """Backward-compatible alias used by the upstream algorithm."""
    return from_occupancy_grid_to_image(occupancy_grid)


def fromOccupancyGridRawToImg(occupancy_grid):
    """Backward-compatible alias; ROS 2 always uses valid trinary grids."""
    return from_occupancy_grid_to_image(occupancy_grid)


def fromImgMapToOccupancyGridRaw(image, origin, resolution):
    """Backward-compatible alias for the corrected trinary conversion."""
    return from_image_to_occupancy_grid(image, origin, resolution)


def _pickle_as_int8(value):
    return array.array("b", pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL))


def make_extendedline_msg(line):
    message = ExtendedLine()
    message.bytes = _pickle_as_int8(line)
    return message


def make_edge_msg(edge):
    message = Edge()
    message.bytes = _pickle_as_int8(edge)
    return message


def make_room_msg(room):
    message = Room()
    message.bytes = _pickle_as_int8(room)
    return message


def make_contour_msg(vertices):
    message = Contour()
    if vertices is not None:
        message.vertices = [float(value) for value in np.ravel(vertices)]
    return message


def _base_marker(namespace, marker_id, marker_type, origin, resolution):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace
    marker.id = int(marker_id)
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose = deepcopy(origin)
    orientation = marker.pose.orientation
    if not any((orientation.x, orientation.y, orientation.z, orientation.w)):
        orientation.w = 1.0
    marker.scale.x = float(resolution)
    marker.color.a = 1.0
    return marker


def delete_all_marker():
    marker = Marker()
    marker.action = Marker.DELETEALL
    return marker


def make_edge_marker(edge, marker_id, origin, resolution):
    marker = _base_marker("edges", marker_id, Marker.LINE_LIST, origin, resolution)
    color = random.Random(marker_id)
    marker.color.r = color.random()
    marker.color.g = color.random()
    marker.color.b = color.random()
    marker.points = [
        Point(x=float(edge.x1 * resolution), y=float(edge.y1 * resolution)),
        Point(x=float(edge.x2 * resolution), y=float(edge.y2 * resolution)),
    ]
    return marker


def make_lines_marker(lines, origin, resolution):
    marker = _base_marker("lines", 50, Marker.LINE_LIST, origin, resolution)
    marker.color.r = 1.0
    for line in lines:
        marker.points.extend(
            [
                Point(x=float(line.x1 * resolution), y=float(line.y1 * resolution)),
                Point(x=float(line.x2 * resolution), y=float(line.y2 * resolution)),
            ]
        )
    return marker


def _room_color(marker_id):
    """Return a bright, deterministic color for one room."""
    hue = (0.02 + marker_id * 0.618033988749895) % 1.0
    return colorsys.hsv_to_rgb(hue, 0.72, 1.0)


def _set_room_color(marker, marker_id, alpha):
    marker.color.r, marker.color.g, marker.color.b = _room_color(marker_id)
    marker.color.a = float(alpha)


def _room_polygons(room):
    geometry = room if room.is_valid else room.buffer(0)
    if geometry.is_empty:
        return []
    if geometry.geom_type == "Polygon":
        return [geometry]
    return [
        part
        for part in getattr(geometry, "geoms", [])
        if part.geom_type == "Polygon" and not part.is_empty
    ]


def _room_triangles(room):
    for polygon in _room_polygons(room):
        for triangle in triangulate(polygon):
            if triangle.area > 0.0 and polygon.covers(triangle):
                yield triangle


def make_room_fill_marker(room, marker_id, origin, resolution):
    marker = _base_marker(
        "rooms_fill", marker_id, Marker.TRIANGLE_LIST, origin, resolution
    )
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    _set_room_color(marker, marker_id, 0.45)
    for triangle in _room_triangles(room):
        triangle = orient(triangle, sign=1.0)
        marker.points.extend(
            Point(x=float(x * resolution), y=float(y * resolution), z=0.025)
            for x, y in list(triangle.exterior.coords)[:3]
        )
    return marker


def make_room_marker(room, marker_id, origin, resolution):
    marker = _base_marker(
        "rooms_outline", marker_id, Marker.LINE_LIST, origin, resolution
    )
    marker.scale.x = max(float(resolution) * 2.0, 0.03)
    _set_room_color(marker, marker_id, 1.0)
    for polygon in _room_polygons(room):
        for ring in [polygon.exterior, *polygon.interiors]:
            coordinates = list(ring.coords)
            for start, end in zip(coordinates[:-1], coordinates[1:]):
                marker.points.extend(
                    [
                        Point(
                            x=float(start[0] * resolution),
                            y=float(start[1] * resolution),
                            z=0.04,
                        ),
                        Point(
                            x=float(end[0] * resolution),
                            y=float(end[1] * resolution),
                            z=0.04,
                        ),
                    ]
                )
    return marker


def _local_point_to_world(origin, x, y, resolution, z):
    offset = np.array([x * resolution, y * resolution, z], dtype=float)
    quaternion = np.array(
        [
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        ],
        dtype=float,
    )
    norm = np.linalg.norm(quaternion)
    if norm > 0.0:
        offset = Rotation.from_quat(quaternion / norm).apply(offset)
    return Point(
        x=float(origin.position.x + offset[0]),
        y=float(origin.position.y + offset[1]),
        z=float(origin.position.z + offset[2]),
    )


def make_room_label_marker(room, marker_id, origin, resolution):
    marker = _base_marker(
        "rooms_label", marker_id, Marker.TEXT_VIEW_FACING, origin, resolution
    )
    polygons = _room_polygons(room)
    if not polygons:
        return marker
    center = max(polygons, key=lambda polygon: polygon.area).representative_point()
    marker.pose.position = _local_point_to_world(
        origin, center.x, center.y, resolution, 0.10
    )
    marker.pose.orientation = Quaternion(w=1.0)
    marker.scale.z = max(float(resolution) * 10.0, 0.40)
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.text = f"Room {marker_id + 1}"
    return marker


def make_room_marker_array(rooms, origin, resolution):
    result = MarkerArray()
    result.markers.append(delete_all_marker())
    for marker_id, room in enumerate(rooms or []):
        fill = make_room_fill_marker(room, marker_id, origin, resolution)
        outline = make_room_marker(room, marker_id, origin, resolution)
        label = make_room_label_marker(room, marker_id, origin, resolution)
        if fill.points:
            result.markers.append(fill)
        if outline.points:
            result.markers.append(outline)
        if label.text:
            result.markers.append(label)
    return result


def make_room_polygon(room, origin, resolution):
    """Retained for consumers that still want PolygonStamped messages."""
    x_values, y_values = room.exterior.coords.xy
    polygon = Polygon(
        points=[
            Point32(
                x=float(x * resolution + origin.position.x),
                y=float(y * resolution + origin.position.y),
            )
            for x, y in zip(x_values, y_values)
        ]
    )
    return PolygonStamped(
        header=Header(frame_id="map", stamp=rospy.Time.now()), polygon=polygon
    )


def make_direction_markers(main_directions, origin, resolution):
    result = MarkerArray()
    result.markers.append(delete_all_marker())
    pairs = zip(main_directions[0::2], main_directions[1::2])
    for marker_id, direction in enumerate(pairs):
        marker = _base_marker(
            "directions", marker_id, Marker.ARROW, origin, resolution
        )
        quaternion = Rotation.from_euler(
            "xyz", [direction[0], 0.0, direction[1]]
        ).as_quat()
        marker.pose.orientation = Quaternion(
            x=float(quaternion[0]),
            y=float(quaternion[1]),
            z=float(quaternion[2]),
            w=float(quaternion[3]),
        )
        marker.scale.x = 5.0
        marker.scale.y = float(resolution)
        marker.scale.z = float(resolution)
        marker.color.g = 1.0
        result.markers.append(marker)
    return result
