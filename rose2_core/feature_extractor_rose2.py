"""ROS 2 node implementing the ROSE2 line/edge/room stage."""

import os
import warnings

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from rose2.msg import ROSE2Features, ROSEFeatures
from rose2.srv import ROSE2
from rose2_core.rose_v2_repo import minibatch, parameters
from rose2_core.util import MsgUtils as mu


TRANSIENT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class FeatureExtractorROSE2(Node):
    def __init__(self):
        super().__init__("rose2")
        defaults = {
            "pub_once": False,
            "spatial_clustering_threshold": 5.0,
            "lines_threshold": 0.0,
            "lines_distance": 20.0,
            "edges_threshold": 0.0,
            "use_voronoi": False,
            "output_directory": "/tmp/rose2_voronoi",
            "voronoi_closeness": 10,
            "voronoi_blur": 8,
            "voronoi_iterations": 5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.pub_once = bool(self.get_parameter("pub_once").value)
        self.use_voronoi = bool(self.get_parameter("use_voronoi").value)
        self.output_directory = str(self.get_parameter("output_directory").value)

        self.algorithm_parameters = parameters.ParameterObj()
        self.algorithm_parameters.spatialClusteringLineSegmentsThreshold = float(
            self.get_parameter("spatial_clustering_threshold").value
        )
        self.algorithm_parameters.th1 = float(
            self.get_parameter("lines_threshold").value
        )
        self.algorithm_parameters.distance_extended_segment = float(
            self.get_parameter("lines_distance").value
        )
        self.algorithm_parameters.threshold_edges = float(
            self.get_parameter("edges_threshold").value
        )
        self.algorithm_parameters.voronoi_closeness = int(
            self.get_parameter("voronoi_closeness").value
        )
        self.algorithm_parameters.blur = int(
            self.get_parameter("voronoi_blur").value
        )
        self.algorithm_parameters.iterations = int(
            self.get_parameter("voronoi_iterations").value
        )

        if self.use_voronoi:
            os.makedirs(self.output_directory, exist_ok=True)
        self.output_directory = os.path.join(self.output_directory, "")

        self.input_features = None
        self.output_features = None
        self.line_marker = Marker()
        self.edge_markers = MarkerArray()
        self.room_markers = MarkerArray()
        self.publish_pending = False
        self.processing = False

        self.subscription = self.create_subscription(
            ROSEFeatures, "features_ROSE", self._features_callback, TRANSIENT_QOS
        )
        self.feature_publisher = self.create_publisher(
            ROSE2Features, "features_ROSE2", TRANSIENT_QOS
        )
        self.feature_publisher_lowercase = self.create_publisher(
            ROSE2Features, "features_rose2", TRANSIENT_QOS
        )
        self.line_publisher = self.create_publisher(
            Marker, "extended_lines", TRANSIENT_QOS
        )
        self.edge_publisher = self.create_publisher(
            MarkerArray, "edges", TRANSIENT_QOS
        )
        self.room_publisher = self.create_publisher(
            MarkerArray, "rooms", TRANSIENT_QOS
        )
        self.status_publisher = self.create_publisher(
            String, "rose2/status", TRANSIENT_QOS
        )
        self.service = self.create_service(
            ROSE2, "ROSE2Srv", self._service_callback
        )
        self.timer = self.create_timer(1.0, self._publish_if_ready)
        self.status = "waiting_for_rose_features"
        self._publish_status()
        self.get_logger().info("[ROSE2] waiting for ROSE features...")

    def _features_callback(self, features):
        self.process_features(features)

    def process_features(self, features):
        if self.processing:
            self.get_logger().warning("[ROSE2] features ignored while processing")
            return False
        self.processing = True
        self._set_status("processing")
        try:
            self.input_features = features
            clean_map = mu.from_structural_occupancy_grid_to_image(
                features.clean_map
            )
            original_map = mu.from_occupancy_grid_to_image(features.original_map)
            self.algorithm_parameters.comp = list(features.directions)

            algorithm = minibatch.Minibatch()
            algorithm.start_main(
                parameters,
                self.algorithm_parameters,
                clean_map,
                original_map,
                self.output_directory,
                self.use_voronoi,
            )
            self._make_messages(algorithm)
            self.publish_pending = True
            line_count = len(self.output_features.lines)
            edge_count = len(self.output_features.edges)
            room_count = len(self.output_features.rooms)
            self._set_status(
                f"ready: {room_count} rooms, {edge_count} edges, "
                f"{line_count} extended lines"
            )
            message = (
                f"[ROSE2] computed {room_count} rooms, {edge_count} edges and "
                f"{line_count} extended lines; publishing /rooms, "
                "/features_rose2 and /features_ROSE2"
            )
            if room_count:
                self.get_logger().info(message)
            else:
                self.get_logger().warning(message)
            return True
        except Exception as error:  # Keep a bad map from terminating the node.
            self._set_status(f"error: {error}")
            self.get_logger().error(f"[ROSE2] processing failed: {error}")
            return False
        finally:
            self.processing = False

    def _make_messages(self, algorithm):
        origin = self.input_features.original_map.info.origin
        resolution = self.input_features.original_map.info.resolution
        extended_lines = list(algorithm.extended_segments_th1_merged)
        edges = list(algorithm.edges_th1)
        rooms = list(algorithm.rooms_th1 or [])

        self.line_marker = mu.make_lines_marker(extended_lines, origin, resolution)
        self.edge_markers = MarkerArray()
        self.edge_markers.markers.append(mu.delete_all_marker())
        for marker_id, edge in enumerate(edges):
            self.edge_markers.markers.append(
                mu.make_edge_marker(edge, marker_id, origin, resolution)
            )
        self.room_markers = mu.make_room_marker_array(rooms, origin, resolution)

        self.output_features = ROSE2Features()
        self.output_features.lines = [
            mu.make_extendedline_msg(line) for line in extended_lines
        ]
        self.output_features.edges = [mu.make_edge_msg(edge) for edge in edges]
        self.output_features.rooms = [mu.make_room_msg(room) for room in rooms]
        self.output_features.contour = mu.make_contour_msg(
            getattr(algorithm, "vertices", None)
        )

    def _service_callback(self, request, response):
        self.process_features(request.features)
        if self.output_features is not None:
            response.features = self.output_features
        return response

    def _publish_status(self):
        self.status_publisher.publish(String(data=self.status))

    def _set_status(self, status):
        self.status = status
        self._publish_status()

    def _publish_if_ready(self):
        self._publish_status()
        if not self.publish_pending or self.output_features is None:
            return
        self.edge_publisher.publish(self.edge_markers)
        self.line_publisher.publish(self.line_marker)
        self.room_publisher.publish(self.room_markers)
        self.feature_publisher.publish(self.output_features)
        self.feature_publisher_lowercase.publish(self.output_features)
        if self.pub_once:
            self.publish_pending = False


def main(args=None):
    warnings.filterwarnings("ignore")
    rclpy.init(args=args)
    node = FeatureExtractorROSE2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
