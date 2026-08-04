import cv2
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid

from rose2_core.my_grid_map import MyGridMap
from rose2_core.rose_v2_repo.parameters import ParameterObj
from rose2_core.util import layout
from rose2_core.util.MsgUtils import (
    from_image_to_occupancy_grid,
    from_occupancy_grid_to_image,
    from_structural_occupancy_grid_to_image,
)


def make_grid():
    grid = OccupancyGrid()
    grid.info.width = 2
    grid.info.height = 2
    grid.info.resolution = 0.05
    grid.data = [0, 100, -1, 0]
    return grid


def test_trinary_grid_to_image():
    image = from_occupancy_grid_to_image(make_grid())
    np.testing.assert_array_equal(
        image, np.array([[255, 0], [200, 255]], dtype=np.uint8)
    )


def test_binary_image_to_valid_occupancy_grid():
    image = np.array([[0, 1], [255, 0]], dtype=np.uint8)
    grid = from_image_to_occupancy_grid(image, Pose(), 0.05)
    assert grid.info.width == 2
    assert grid.info.height == 2
    assert list(grid.data) == [0, 100, 100, 0]
    assert all(-1 <= value <= 100 for value in grid.data)


def test_clean_grid_preserves_original_rose2_binary_semantics():
    image = from_structural_occupancy_grid_to_image(make_grid())
    np.testing.assert_array_equal(
        image, np.array([[0, 1], [0, 0]], dtype=np.uint8)
    )


def test_rose2_hough_receives_walls_instead_of_free_space(monkeypatch):
    structural_image = from_structural_occupancy_grid_to_image(make_grid())
    captured = {}

    def fake_hough(image, *args):
        captured["image"] = image.copy()
        return np.array([[[0, 0, 1, 1]]], dtype=np.int32)

    monkeypatch.setattr(cv2, "HoughLinesP", fake_hough)
    rose2_input = cv2.bitwise_not(structural_image)
    walls, _ = layout.start_canny_and_hough(rose2_input, ParameterObj())

    np.testing.assert_array_equal(captured["image"], structural_image)
    np.testing.assert_array_equal(walls, np.array([[0, 0, 1, 1]]))


def test_grid_wrapper_uses_python3_integer_coordinates():
    wrapped = MyGridMap(make_grid())
    assert wrapped.getCoordinates(3) == (1, 1)
    assert wrapped.getIndex(1, 1) == 3
    assert wrapped.getData(1, 0) == 100
    assert wrapped.areNeighbour(0, 3)
