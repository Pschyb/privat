"""Export live ROSE2 room polygons and their map metadata to YAML."""

from __future__ import annotations

import os
import tempfile
from typing import Any

import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Trigger

from rose2.msg import ROSE2Features, ROSEFeatures
from yaml_extractor.exporter_core import (
    build_segment,
    decode_room,
    resolve_output_yaml,
    yaw_from_quaternion,
)

LATCHED_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class LiveSegmentedMapExporter(Node):
    """Combine ROSE2 polygons with map metadata and write a YAML document."""

    def __init__(self) -> None:
        super().__init__("rose2_yaml_extractor")

        self.declare_parameter("output_yaml", "")
        self.declare_parameter("map_yaml", "rose2_live.yaml")
        self.declare_parameter("features_topic", "/features_ROSE2")
        self.declare_parameter("rose_features_topic", "/features_ROSE")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("write_once", True)
        self.declare_parameter("auto_save", True)

        self.output_yaml = str(self.get_parameter("output_yaml").value)
        self.map_yaml = str(self.get_parameter("map_yaml").value)
        self.features_topic = str(self.get_parameter("features_topic").value)
        self.rose_features_topic = str(self.get_parameter("rose_features_topic").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.write_once = bool(self.get_parameter("write_once").value)
        self.auto_save = bool(self.get_parameter("auto_save").value)

        self.latest_features: ROSE2Features | None = None
        self.latest_map: OccupancyGrid | None = None
        self._map_from_rose = False
        self._exported = False

        self.create_subscription(
            ROSE2Features,
            self.features_topic,
            self._on_features,
            LATCHED_QOS,
        )
        self.create_subscription(
            ROSEFeatures,
            self.rose_features_topic,
            self._on_rose_features,
            LATCHED_QOS,
        )
        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._on_map,
            LATCHED_QOS,
        )
        self.create_service(Trigger, "~/save", self._on_save)
        self._timer = self.create_timer(0.5, self._try_export)

        self.get_logger().info(
            "waiting for ROSE2 rooms on "
            f"{self.features_topic} and map metadata on "
            f"{self.rose_features_topic} or {self.map_topic}"
        )

    def _on_features(self, message: ROSE2Features) -> None:
        self.latest_features = message

    def _on_rose_features(self, message: ROSEFeatures) -> None:
        # This is the exact OccupancyGrid used to produce the ROSE2 result and
        # therefore takes precedence over the fallback /map subscription.
        self.latest_map = message.original_map
        self._map_from_rose = True

    def _on_map(self, message: OccupancyGrid) -> None:
        if not self._map_from_rose:
            self.latest_map = message

    def _ready(self) -> bool:
        return self.latest_features is not None and self.latest_map is not None

    def _build_segments(self) -> list[dict[str, Any]]:
        if not self._ready():
            return []

        assert self.latest_features is not None
        assert self.latest_map is not None
        resolution = float(self.latest_map.info.resolution)
        origin = self.latest_map.info.origin
        segments: list[dict[str, Any]] = []

        for index, room_message in enumerate(self.latest_features.rooms):
            try:
                room = decode_room(room_message)
                segments.append(build_segment(room, index, resolution, origin))
            # A malformed pickle or geometry must not suppress other rooms.
            except Exception as error:  # noqa: BLE001
                self.get_logger().warning(
                    f"could not decode/export room {index + 1}: {error}"
                )

        return segments

    def _document(self) -> dict[str, Any]:
        assert self.latest_map is not None
        origin = self.latest_map.info.origin
        return {
            "map_yaml": self.map_yaml,
            "origin": [
                float(origin.position.x),
                float(origin.position.y),
                float(yaw_from_quaternion(origin.orientation)),
            ],
            "resolution": float(self.latest_map.info.resolution),
            "segments": self._build_segments(),
            "subsegment_tile_size_m": 0.0,
            "segmentation_method": "rose2_live",
            "source_topics": {
                "features_topic": self.features_topic,
                "rose_features_topic": self.rose_features_topic,
                "map_topic": self.map_topic,
            },
        }

    def _write_yaml(self) -> str:
        if not self._ready():
            raise RuntimeError("ROSE2 rooms or map metadata have not arrived yet")

        output_yaml = resolve_output_yaml(self.output_yaml, self.map_yaml)
        output_dir = os.path.dirname(output_yaml) or "."
        os.makedirs(output_dir, exist_ok=True)

        descriptor, temporary_path = tempfile.mkstemp(
            prefix=f".{os.path.basename(output_yaml)}.",
            suffix=".tmp",
            dir=output_dir,
            text=True,
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                yaml.safe_dump(self._document(), handle, sort_keys=False)
            os.replace(temporary_path, output_yaml)
        except Exception:
            if os.path.exists(temporary_path):
                os.unlink(temporary_path)
            raise

        self.get_logger().info(f"exported segmented-map YAML to {output_yaml}")
        return output_yaml

    def _attempt_write(self) -> bool:
        try:
            self._write_yaml()
        # Keep the long-running ROS node available after a filesystem or
        # serialization failure so a manual retry can succeed.
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"YAML export failed: {error}")
            return False
        self._exported = True
        if self.write_once:
            self._timer.cancel()
        return True

    def _try_export(self) -> None:
        if not self.auto_save or not self._ready():
            return
        if self._exported and self.write_once:
            return
        self._attempt_write()

    def _on_save(self, request: Trigger.Request, response: Trigger.Response):
        del request
        if not self._ready():
            response.success = False
            response.message = "ROSE2 rooms or map metadata have not arrived yet"
            return response

        response.success = self._attempt_write()
        response.message = (
            resolve_output_yaml(self.output_yaml, self.map_yaml)
            if response.success
            else "YAML export failed; see node log"
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiveSegmentedMapExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
