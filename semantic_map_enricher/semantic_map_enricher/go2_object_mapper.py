"""Assign YOLO object labels to ROSE2 rooms using the Unitree Go2 pose."""

from __future__ import annotations

import os
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from semantic_map_enricher.core import (
    SemanticMapModel,
    atomic_write_yaml,
    load_semantic_map,
    rotate_point_by_quaternion,
)
from semantic_map_enricher.depth_filter import (
    depth_image_to_meters,
    detections_in_distance_range,
    parse_spatial_detections,
)


DETECTION_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class Go2ObjectMapper(Node):
    """Enrich a ROSE2 YAML with room-local YOLO object lists."""

    def __init__(self) -> None:
        super().__init__("go2_object_mapper")

        robot_namespace = os.environ.get("ROBOT_NS", "go2_unit_001").strip("/")
        default_depth_topic = (
            f"/{robot_namespace}/{robot_namespace}/"
            "aligned_depth_to_color/image_raw"
        )

        self.declare_parameter("yaml_path", "")
        self.declare_parameter(
            "pose_topic", "/lio_sam_ros2/mapping/re_location_odometry"
        )
        self.declare_parameter("detected_objects_topic", "/detected_objects")
        self.declare_parameter("segment_id_topic", "/current_segment_id")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("transform_pose_to_map", True)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("max_pose_age_sec", 2.0)
        self.declare_parameter("auto_save", True)
        self.declare_parameter("distance_filter_enabled", True)
        self.declare_parameter("depth_image_topic", default_depth_topic)
        self.declare_parameter("min_object_distance_m", 0.2)
        self.declare_parameter("max_object_distance_m", 3.0)
        self.declare_parameter("max_depth_age_sec", 0.5)
        self.declare_parameter("depth_roi_fraction", 0.5)
        self.declare_parameter("min_valid_depth_pixels", 20)

        self.yaml_path = str(self.get_parameter("yaml_path").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.detected_objects_topic = str(
            self.get_parameter("detected_objects_topic").value
        )
        self.segment_id_topic = str(self.get_parameter("segment_id_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value).lstrip("/")
        self.transform_pose_to_map = bool(
            self.get_parameter("transform_pose_to_map").value
        )
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.max_pose_age_sec = float(
            self.get_parameter("max_pose_age_sec").value
        )
        self.auto_save = bool(self.get_parameter("auto_save").value)
        self.distance_filter_enabled = bool(
            self.get_parameter("distance_filter_enabled").value
        )
        self.depth_image_topic = str(
            self.get_parameter("depth_image_topic").value
        )
        self.min_object_distance_m = float(
            self.get_parameter("min_object_distance_m").value
        )
        self.max_object_distance_m = float(
            self.get_parameter("max_object_distance_m").value
        )
        self.max_depth_age_sec = float(
            self.get_parameter("max_depth_age_sec").value
        )
        self.depth_roi_fraction = float(
            self.get_parameter("depth_roi_fraction").value
        )
        self.min_valid_depth_pixels = int(
            self.get_parameter("min_valid_depth_pixels").value
        )

        if not self.yaml_path:
            raise ValueError("parameter 'yaml_path' must point to the ROSE2 YAML")
        if not os.path.isfile(self.yaml_path):
            raise FileNotFoundError(f"semantic YAML does not exist: {self.yaml_path}")
        if not self.map_frame:
            raise ValueError("parameter 'map_frame' must not be empty")
        if self.tf_timeout_sec < 0.0:
            raise ValueError("parameter 'tf_timeout_sec' must be non-negative")
        if self.max_pose_age_sec <= 0.0:
            raise ValueError("parameter 'max_pose_age_sec' must be positive")
        if self.distance_filter_enabled and not self.depth_image_topic:
            raise ValueError(
                "parameter 'depth_image_topic' must not be empty when the "
                "distance filter is enabled"
            )
        if self.min_object_distance_m < 0.0:
            raise ValueError("parameter 'min_object_distance_m' must be non-negative")
        if self.max_object_distance_m <= self.min_object_distance_m:
            raise ValueError(
                "parameter 'max_object_distance_m' must be greater than "
                "'min_object_distance_m'"
            )
        if self.max_depth_age_sec <= 0.0:
            raise ValueError("parameter 'max_depth_age_sec' must be positive")
        if not 0.0 < self.depth_roi_fraction <= 1.0:
            raise ValueError(
                "parameter 'depth_roi_fraction' must be in the interval (0, 1]"
            )
        if self.min_valid_depth_pixels <= 0:
            raise ValueError("parameter 'min_valid_depth_pixels' must be positive")

        try:
            self.model: SemanticMapModel = load_semantic_map(self.yaml_path)
        except Exception as error:  # noqa: BLE001
            raise ValueError(
                f"could not load semantic YAML '{self.yaml_path}': {error}"
            ) from error
        self._last_pose_received_at: float | None = None
        self._last_position: tuple[float, float] | None = None
        self._last_tf_warning_at = 0.0
        self._last_missing_pose_warning_at = 0.0
        self._last_depth_warning_at = 0.0
        self._latest_depth_m = None
        self._last_depth_received_at: float | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Odometry,
            self.pose_topic,
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            self.detected_objects_topic,
            self._on_detected_objects,
            DETECTION_QOS,
        )
        if self.distance_filter_enabled:
            self.create_subscription(
                Image,
                self.depth_image_topic,
                self._on_depth_image,
                qos_profile_sensor_data,
            )
        self.segment_id_publisher = self.create_publisher(
            Int32, self.segment_id_topic, 10
        )
        self.create_service(Trigger, "~/save", self._on_save)
        self.create_service(Trigger, "~/reload", self._on_reload)

        self.get_logger().info(
            f"loaded {len(self.model.segments)} ROSE2 rooms from {self.yaml_path}"
        )
        self.get_logger().info(
            f"Go2 pose: {self.pose_topic} (nav_msgs/msg/Odometry), "
            f"YOLO objects: {self.detected_objects_topic} (std_msgs/msg/String), "
            f"map frame: {self.map_frame}"
        )
        if self.distance_filter_enabled:
            self.get_logger().info(
                f"D435i distance filter: {self.min_object_distance_m:.2f} to "
                f"{self.max_object_distance_m:.2f} m using "
                f"{self.depth_image_topic}"
            )
        else:
            self.get_logger().warning(
                "D435i distance filter disabled; label-only YOLO payloads are accepted"
            )

    def _pose_in_map(self, message: Odometry) -> tuple[float, float] | None:
        position = message.pose.pose.position
        source_frame = message.header.frame_id.lstrip("/")
        if not source_frame or source_frame == self.map_frame:
            return float(position.x), float(position.y)

        if not self.transform_pose_to_map:
            self._warn_tf_throttled(
                f"ignoring pose in frame '{source_frame}'; expected '{self.map_frame}'"
            )
            return None

        try:
            # The robot and host clocks can differ. The newest available TF is
            # safer here than querying with the Unitree message timestamp.
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            ).transform
            rotated = rotate_point_by_quaternion(
                float(position.x),
                float(position.y),
                float(position.z),
                float(transform.rotation.x),
                float(transform.rotation.y),
                float(transform.rotation.z),
                float(transform.rotation.w),
            )
            return (
                rotated[0] + float(transform.translation.x),
                rotated[1] + float(transform.translation.y),
            )
        except (TransformException, ValueError) as error:
            self._warn_tf_throttled(
                f"cannot transform Go2 pose from '{source_frame}' to "
                f"'{self.map_frame}': {error}"
            )
            return None

    def _warn_tf_throttled(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_tf_warning_at >= 5.0:
            self.get_logger().warning(text)
            self._last_tf_warning_at = now

    def _on_odometry(self, message: Odometry) -> None:
        position = self._pose_in_map(message)
        if position is None:
            return

        self._last_pose_received_at = time.monotonic()
        self._last_position = position
        transition = self.model.update_position(*position)
        if not transition.changed:
            return

        published_id = (
            transition.current_id if transition.current_id is not None else -1
        )
        self.segment_id_publisher.publish(Int32(data=published_id))
        if transition.current_id is None:
            self.get_logger().info(
                f"Go2 is outside all room polygons at x={position[0]:.2f}, "
                f"y={position[1]:.2f}; detection history cleared"
            )
        else:
            self.get_logger().info(
                f"entered segment {transition.current_id} at x={position[0]:.2f}, "
                f"y={position[1]:.2f}; room-local detection history reset"
            )

    def _pose_is_fresh(self) -> bool:
        if self._last_pose_received_at is None:
            return False
        return time.monotonic() - self._last_pose_received_at <= self.max_pose_age_sec

    def _warn_missing_pose_throttled(self) -> None:
        now = time.monotonic()
        if now - self._last_missing_pose_warning_at >= 5.0:
            self.get_logger().warning(
                "ignoring YOLO objects because no recent Go2 pose is available"
            )
            self._last_missing_pose_warning_at = now

    def _warn_depth_throttled(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_depth_warning_at >= 5.0:
            self.get_logger().warning(text)
            self._last_depth_warning_at = now

    def _on_depth_image(self, message: Image) -> None:
        try:
            depth_m = depth_image_to_meters(
                message.data,
                message.height,
                message.width,
                message.step,
                message.encoding,
                bool(message.is_bigendian),
            )
        except ValueError as error:
            self._warn_depth_throttled(f"ignoring D435i depth image: {error}")
            return
        self._latest_depth_m = depth_m
        self._last_depth_received_at = time.monotonic()

    def _depth_is_fresh(self) -> bool:
        if self._latest_depth_m is None or self._last_depth_received_at is None:
            return False
        return (
            time.monotonic() - self._last_depth_received_at
            <= self.max_depth_age_sec
        )

    def _on_detected_objects(self, message: String) -> None:
        # Deliberately do not process a cached object value from the pose
        # callback. Only a newly received YOLO message can change the YAML.
        if not self._pose_is_fresh() or self.model.current_segment is None:
            self._warn_missing_pose_throttled()
            return

        accepted_with_distance = []
        if self.distance_filter_enabled:
            detections = parse_spatial_detections(message.data)
            if not detections:
                self._warn_depth_throttled(
                    "ignoring YOLO message because it contains no pixel bounding "
                    "boxes; publish structured detections with label and bbox"
                )
                return
            if not self._depth_is_fresh():
                self._warn_depth_throttled(
                    "ignoring YOLO message because no recent aligned D435i depth "
                    "image is available"
                )
                return

            accepted_with_distance = detections_in_distance_range(
                detections,
                self._latest_depth_m,
                self.min_object_distance_m,
                self.max_object_distance_m,
                roi_fraction=self.depth_roi_fraction,
                min_valid_pixels=self.min_valid_depth_pixels,
            )
            added = self.model.add_objects(
                detection.label for detection, _ in accepted_with_distance
            )
        else:
            added = self.model.add_detection_payload(message.data)
        if not added:
            return

        segment_id_value = self.model.current_segment_id
        if self.distance_filter_enabled:
            measured = ", ".join(
                f"{detection.label}={distance:.2f} m"
                for detection, distance in accepted_with_distance
                if detection.label in added
            )
            self.get_logger().info(
                f"segment {segment_id_value}: added depth-filtered objects "
                f"{added} ({measured})"
            )
        else:
            self.get_logger().info(
                f"segment {segment_id_value}: added objects {added}"
            )
        if self.auto_save:
            self._save()

    def _save(self) -> None:
        atomic_write_yaml(self.yaml_path, self.model.document)
        self.model.dirty = False
        self.get_logger().info(f"updated semantic YAML: {self.yaml_path}")

    def _on_save(self, request: Trigger.Request, response: Trigger.Response):
        del request
        try:
            self._save()
            response.success = True
            response.message = self.yaml_path
        except Exception as error:  # noqa: BLE001
            response.success = False
            response.message = str(error)
            self.get_logger().error(f"could not save semantic YAML: {error}")
        return response

    def _on_reload(self, request: Trigger.Request, response: Trigger.Response):
        del request
        if self.model.dirty:
            response.success = False
            response.message = "unsaved in-memory changes; save before reloading"
            return response
        try:
            self.model = load_semantic_map(self.yaml_path)
            self._last_pose_received_at = None
            self._last_position = None
            response.success = True
            response.message = f"reloaded {len(self.model.segments)} segments"
            self.get_logger().info(response.message)
        except Exception as error:  # noqa: BLE001
            response.success = False
            response.message = str(error)
            self.get_logger().error(f"could not reload semantic YAML: {error}")
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Go2ObjectMapper | None = None
    executor: MultiThreadedExecutor | None = None
    try:
        node = Go2ObjectMapper()
        # A second executor thread lets TF data arrive while an odometry
        # callback is briefly waiting for a transform.
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (FileNotFoundError, ValueError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"go2_object_mapper: {error}")
        raise SystemExit(1) from error
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
