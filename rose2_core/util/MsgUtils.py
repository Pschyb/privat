"""Conversion and visualization helpers for ROSE2 ROS messages."""

import array
import pickle
import random

import numpy as np
from geometry_msgs.msg import Point, Point32, Polygon, PolygonStamped, Quaternion
from nav_msgs.msg import MapMetaData, OccupancyGrid
from scipy.spatial.transform import Rotation
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
    marker.pose = origin
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


def make_room_marker(room, marker_id, origin, resolution):
    marker = _base_marker("rooms", marker_id, Marker.LINE_STRIP, origin, resolution)
    marker.scale.x = max(float(resolution) * 2.0, 0.03)
    color = random.Random(10_000 + marker_id)
    marker.color.r = color.random()
    marker.color.g = color.random()
    marker.color.b = color.random()
    x_values, y_values = room.exterior.coords.xy
    marker.points = [
        Point(x=float(x * resolution), y=float(y * resolution), z=0.02)
        for x, y in zip(x_values, y_values)
    ]
    return marker


def make_room_marker_array(rooms, origin, resolution):
    result = MarkerArray()
    result.markers.append(delete_all_marker())
    for marker_id, room in enumerate(rooms or []):
        result.markers.append(make_room_marker(room, marker_id, origin, resolution))
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
