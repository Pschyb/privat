"""ROS 2 node implementing the first (structural filtering) ROSE stage."""

import warnings

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from PIL import Image as ImagePIL
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from skimage.util import img_as_ubyte
from visualization_msgs.msg import MarkerArray

from rose2.msg import ROSEFeatures
from rose2.srv import ROSE
from rose2_core.rose_v1_repo.fft_structure_extraction import (
    FFTStructureExtraction,
)
from rose2_core.util import MsgUtils as mu


TRANSIENT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class FeatureExtractorROSE(Node):
    def __init__(self):
        super().__init__("rose")
        self.declare_parameter("pub_once", True)
        self.declare_parameter("filter", 0.18)
        self.declare_parameter("map_topic", "map")

        self.pub_once = bool(self.get_parameter("pub_once").value)
        self.filter_level = float(self.get_parameter("filter").value)

        self.features = None
        self.clean_map = None
        self.main_directions = []
        self.publish_pending = False
        self.processing = False

        self.feature_publisher = self.create_publisher(
            ROSEFeatures, "features_ROSE", TRANSIENT_QOS
        )
        self.clean_map_publisher = self.create_publisher(
            OccupancyGrid, "clean_map", TRANSIENT_QOS
        )
        self.direction_publisher = self.create_publisher(
            MarkerArray, "direction_markers", TRANSIENT_QOS
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._map_callback,
            TRANSIENT_QOS,
        )
        self.service = self.create_service(ROSE, "ROSESrv", self._service_callback)
        self.timer = self.create_timer(1.0, self._publish_if_ready)
        self.get_logger().info("[ROSE] waiting for map...")

    def _map_callback(self, occupancy_grid):
        self.process_map(occupancy_grid)

    def process_map(self, occupancy_grid):
        if self.processing:
            self.get_logger().warning("[ROSE] map ignored while processing")
            return False

        self.processing = True
        try:
            origin = occupancy_grid.info.origin
            resolution = occupancy_grid.info.resolution
            image_map = mu.from_occupancy_grid_to_image(occupancy_grid)
            grid_map = img_as_ubyte(ImagePIL.fromarray(image_map))

            rose = FFTStructureExtraction(grid_map, peak_height=0.2, par=50)
            rose.process_map()
            if len(rose.main_directions) <= 2:
                self.get_logger().warning(
                    "[ROSE] skipped map: not enough dominant directions"
                )
                return False

            rose.simple_filter_map(self.filter_level)
            rose.generate_initial_hypothesis_simple()
            try:
                rose.find_walls_flood_filing()
            except ValueError as error:
                self.get_logger().warning(
                    f"[ROSE] skipped map: walls could not be extracted ({error})"
                )
                return False

            analysed_map = rose.analysed_map.astype(np.uint8)
            self.main_directions = [float(value) for value in rose.main_directions]
            self.clean_map = mu.from_image_to_occupancy_grid(
                analysed_map,
                origin,
                resolution,
                stamp=self.get_clock().now().to_msg(),
            )
            self.features = ROSEFeatures()
            self.features.original_map = occupancy_grid
            self.features.clean_map = self.clean_map
            self.features.directions = self.main_directions
            self.publish_pending = True
            self.get_logger().info("[ROSE] structural map computed")
            return True
        except Exception as error:  # Keep a bad map from terminating the node.
            self.get_logger().error(f"[ROSE] processing failed: {error}")
            return False
        finally:
            self.processing = False

    def _service_callback(self, request, response):
        self.process_map(request.map)
        if self.features is not None:
            response.features = self.features
        return response

    def _publish_if_ready(self):
        if not self.publish_pending or self.features is None:
            return
        self.feature_publisher.publish(self.features)
        self.clean_map_publisher.publish(self.clean_map)
        self.direction_publisher.publish(
            mu.make_direction_markers(
                self.main_directions,
                self.features.original_map.info.origin,
                self.features.original_map.info.resolution,
            )
        )
        if self.pub_once:
            self.publish_pending = False


def main(args=None):
    warnings.filterwarnings("ignore")
    rclpy.init(args=args)
    node = FeatureExtractorROSE()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
