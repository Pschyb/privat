import cv2
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid

from rose2_core.my_grid_map import MyGridMap
from rose2_core.rose_v2_repo.parameters import ParameterObj
from rose2_core.util import layout, voronoi
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


def test_cell_dbscan_uses_keyword_only_sklearn_api():
    distance_matrix = np.array([[0.0, 1.0], [1.0, 0.0]])

    labels = layout.clustering_dbscan_cells(
        eps=0.85, min_samples=1, X=distance_matrix
    )

    np.testing.assert_array_equal(labels, np.array([0, 1]))


def test_voronoi_graph_uses_current_two_value_skan_api():
    skeleton = np.zeros((3, 3), dtype=bool)
    skeleton[1, :] = True

    graph, coordinates = voronoi.skeleton_to_networkx_graph(skeleton)

    assert graph.number_of_nodes() == 3
    assert graph.number_of_edges() == 2
    assert coordinates.shape == (3, 2)
    np.testing.assert_array_equal(
        coordinates, np.array([[1, 0], [1, 1], [1, 2]])
    )


def test_voronoi_preprocessing_does_not_write_a_shared_temp_file(
    tmp_path, monkeypatch
):
    image = np.zeros((9, 9), dtype=np.uint8)
    image[4, 2:7] = 255
    image_path = tmp_path / "metric_map.png"
    assert cv2.imwrite(str(image_path), image)

    def reject_temp_file(*args, **kwargs):
        raise AssertionError("Voronoi preprocessing must stay in memory")

    monkeypatch.setattr(voronoi.cv2, "imwrite", reject_temp_file)
    parameters = ParameterObj()
    parameters.blur = 1
    graph, coordinates = voronoi.compute_voronoi_graph(
        str(image_path),
        parameters,
        False,
        "",
        parameters.bormann,
        filepath=str(tmp_path) + "/",
    )

    assert coordinates.shape[1] == 2
    assert graph.number_of_nodes() >= 0


def test_grid_wrapper_uses_python3_integer_coordinates():
    wrapped = MyGridMap(make_grid())
    assert wrapped.getCoordinates(3) == (1, 1)
    assert wrapped.getIndex(1, 1) == 3
    assert wrapped.getData(1, 0) == 100
    assert wrapped.areNeighbour(0, 3)
