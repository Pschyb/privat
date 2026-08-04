from geometry_msgs.msg import Pose
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry import Polygon
from visualization_msgs.msg import Marker

from rose2_core.util.MsgUtils import make_room_marker_array


def make_origin():
    origin = Pose()
    origin.position.x = 2.0
    origin.position.y = -1.0
    origin.orientation.w = 1.0
    return origin


def marker_by_namespace(markers, namespace):
    return next(marker for marker in markers if marker.ns == namespace)


def test_room_markers_are_filled_outlined_and_labelled():
    room = Polygon([(0, 0), (4, 0), (4, 3), (0, 3)])
    origin = make_origin()
    resolution = 0.5

    marker_array = make_room_marker_array([room], origin, resolution)

    assert marker_array.markers[0].action == Marker.DELETEALL
    fill = marker_by_namespace(marker_array.markers, "rooms_fill")
    outline = marker_by_namespace(marker_array.markers, "rooms_outline")
    label = marker_by_namespace(marker_array.markers, "rooms_label")

    assert fill.type == Marker.TRIANGLE_LIST
    assert len(fill.points) == 6
    assert 0.0 < fill.color.a < 1.0
    assert outline.type == Marker.LINE_LIST
    assert len(outline.points) == 8
    assert outline.color.a == 1.0
    assert label.type == Marker.TEXT_VIEW_FACING
    assert label.text == "Room 1"
    assert label.color.a == 1.0

    fill_color = (fill.color.r, fill.color.g, fill.color.b)
    outline_color = (outline.color.r, outline.color.g, outline.color.b)
    assert fill_color == outline_color
    assert max(fill_color) >= 0.9

    label_local_x = (label.pose.position.x - origin.position.x) / resolution
    label_local_y = (label.pose.position.y - origin.position.y) / resolution
    assert room.covers(ShapelyPoint(label_local_x, label_local_y))


def test_concave_room_fill_contains_only_interior_triangles():
    room = Polygon([(0, 0), (4, 0), (4, 1), (1, 1), (1, 4), (0, 4)])
    resolution = 0.25
    marker_array = make_room_marker_array([room], make_origin(), resolution)
    fill = marker_by_namespace(marker_array.markers, "rooms_fill")

    assert fill.points
    assert len(fill.points) % 3 == 0
    for index in range(0, len(fill.points), 3):
        triangle = Polygon(
            [
                (
                    fill.points[offset].x / resolution,
                    fill.points[offset].y / resolution,
                )
                for offset in range(index, index + 3)
            ]
        )
        assert room.covers(triangle)


def test_empty_room_result_clears_previous_markers():
    marker_array = make_room_marker_array([], make_origin(), 0.05)
    assert len(marker_array.markers) == 1
    assert marker_array.markers[0].action == Marker.DELETEALL
