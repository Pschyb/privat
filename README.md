# ROSE2 and YAML Extractor for ROS 2 Humble

This repository contains a ROS 2 Humble port of
[aislabunimi/ROSE2](https://github.com/aislabunimi/ROSE2). ROSE removes
non-structural clutter from an occupancy grid and estimates dominant map
directions. ROSE2 then extracts extended wall lines, edges and room polygons.

The original algorithms and GPL-3.0 license are preserved. ROS integration is
implemented natively with `rclpy`, ROS 2 interfaces, Python launch files and
ROS 2 QoS settings. The repository is a ROS 2 multi-package repository:

```text
privat/
├── rose2/             # ROSE/ROSE2 algorithm, interfaces and RViz output
├── yaml_extractor/    # ROSE2 room-polygon YAML exporter
├── semantic_map_enricher/ # Assign YOLO objects to Go2's current room
├── maps/              # Shared source maps (existing paths remain valid)
└── requirements-humble.txt
```

Keeping the packages as siblings lets `colcon` discover each package.
Nesting `yaml_extractor` below the old package root would make package
discovery unreliable because the repository root was itself already a ROS
package.

## Supported platform

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10

## Installation

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-pip python3-rosdep

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Pschyb/privat.git rose2

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update --rosdistro humble
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
python3 -m pip install --user -r src/rose2/requirements-humble.txt
colcon build --symlink-install \
  --packages-select rose2 yaml_extractor semantic_map_enricher
source install/setup.bash
```

The pinned Python versions match Python 3.10 on Ubuntu 22.04. They also avoid
the known `numba`/`coverage` incompatibility seen with some newer combinations.

## Run

Start a map publisher that provides `nav_msgs/msg/OccupancyGrid` on `/map`,
then run:

```bash
ros2 launch rose2 rose.launch.py
```

Without RViz:

```bash
ros2 launch rose2 rose.launch.py rviz:=false
```

Enable the optional, slower Voronoi refinement:

```bash
ros2 launch rose2 rose.launch.py use_voronoi:=true \
  output_directory:=/tmp/rose2_voronoi
```

The `/map` and intermediate-feature subscriptions use reliable,
transient-local QoS, so latched maps from `nav2_map_server` are received even
when ROSE2 starts later.

ROSE2 can take some time to compute the segmentation. Its current state and
the final result can be checked from another sourced terminal:

```bash
ros2 topic echo /rose2/status
ros2 topic echo /rooms --once
ros2 topic echo /features_rose2 --once
```

The default RViz configuration displays every detected room as a
semi-transparent colored area with a solid outline and a room-number label.
ROSE2 republishes the finished visualization and feature messages once per
second, so subscribers started after the computation still receive them.

## ROS interfaces

| Direction | Name | Type |
|---|---|---|
| Subscribe | `/map` | `nav_msgs/msg/OccupancyGrid` |
| Publish | `/clean_map` | `nav_msgs/msg/OccupancyGrid` |
| Publish | `/features_ROSE` | `rose2/msg/ROSEFeatures` |
| Publish | `/features_rose2` | `rose2/msg/ROSE2Features` |
| Publish | `/features_ROSE2` | `rose2/msg/ROSE2Features` (legacy alias) |
| Publish | `/rose2/status` | `std_msgs/msg/String` |
| Publish | `/direction_markers` | `visualization_msgs/msg/MarkerArray` |
| Publish | `/extended_lines` | `visualization_msgs/msg/Marker` |
| Publish | `/edges` | `visualization_msgs/msg/MarkerArray` |
| Publish | `/rooms` | `visualization_msgs/msg/MarkerArray` |
| Service | `/ROSESrv` | `rose2/srv/ROSE` |
| Service | `/ROSE2Srv` | `rose2/srv/ROSE2` |

Unlike the ROS 1 package, `/rooms` uses the standard ROS 2 `MarkerArray`
message. Each room contains fill triangles, an outline and a text label. The
external `jsk_recognition_msgs` and `jsk_rviz_plugins` dependencies are
therefore no longer required.

## Export room segmentation to YAML

`yaml_extractor` consumes the actual interfaces of this port:

- `/features_ROSE2` (`rose2/msg/ROSE2Features`) supplies the room polygons.
- `/features_ROSE` supplies the exact source OccupancyGrid metadata.
- `/map` is a fallback source for the map metadata.

Start it before or after ROSE2. All subscriptions use reliable,
transient-local QoS:

```bash
ros2 launch yaml_extractor yaml_extractor.launch.py \
  map_yaml:=/home/paul/rose2_ws/src/rose2/maps/Freiburg_Building_079/map15.yaml \
  output_yaml:=/home/paul/rose2_ws/src/rose2/maps/Freiburg_Building_079/map15_segments_from_rose2.yaml
```

With `write_once:=true` (default), the file is written once as soon as both
the ROSE2 polygons and map metadata have arrived. Set `auto_save:=false` for
manual saving:

```bash
ros2 service call /rose2_yaml_extractor/save std_srvs/srv/Trigger '{}'
```

The resulting document retains the schema of the original exporter:

```yaml
map_yaml: /path/to/map15.yaml
origin: [-20.0, -20.0, 0.0]
resolution: 0.05
segments:
  - id: 1
    area_cells: 1234
    area_m2: 3.085
    center_px: [320.5, 140.25]
    center_m: [-3.975, -12.9875]
    corners_m: []
    polygon_m: []
    room_name: unknown
    objects: []
subsegment_tile_size_m: 0.0
segmentation_method: rose2_live
source_topics:
  features_topic: /features_ROSE2
  rose_features_topic: /features_ROSE
  map_topic: /map
```

Room area is derived directly from the ROSE2 Shapely polygon and rounded to
grid cells, matching the original `area_cells`/`area_m2` schema. Coordinates
are transformed according to the OccupancyGrid origin and yaw; unlike an
image export, the ROS 2 polygon y-coordinate must not be flipped.

## Add YOLO objects to the current Go2 room

`semantic_map_enricher` updates the `objects` list of the room that currently
contains the Unitree Go2. It uses the official Unitree SLAM relocation output
by default:

| Direction | Name | Type |
|---|---|---|
| Subscribe | `/lio_sam_ros2/mapping/re_location_odometry` | `nav_msgs/msg/Odometry` |
| Subscribe | `/detected_objects` | `std_msgs/msg/String` |
| Publish | `/current_segment_id` | `std_msgs/msg/Int32` (`-1` outside all rooms) |
| Service | `/go2_object_mapper/save` | `std_srvs/srv/Trigger` |
| Service | `/go2_object_mapper/reload` | `std_srvs/srv/Trigger` |

Unitree's
[official SLAM example](https://github.com/unitreerobotics/unitree_slam/blob/main/unitree_slam_example/demo_mid360.cpp)
subscribes to
`lio_sam_ros2/mapping/re_location_odometry` for the real-time relocated robot
pose. ROS 2 hides the DDS `rt/` prefix, so the ROS topic begins with `/lio...`.
The mapping-only pose is `/lio_sam_ros2/mapping/odometry`; it can be selected
with the launch argument shown below.

Run the node with the YAML previously created by `yaml_extractor`:

```bash
ros2 launch semantic_map_enricher go2_semantic_map.launch.py \
  yaml_path:=/home/paul/rose2_ws/src/rose2/maps/my_map_segments_from_rose2.yaml
```

If object detections use a different topic, select it through the parameter:

```bash
ros2 launch semantic_map_enricher go2_semantic_map.launch.py \
  yaml_path:=/absolute/path/to/my_map_segments_from_rose2.yaml \
  detected_objects_topic:=/my_yolo/detected_objects
```

For enrichment while the Go2 is still mapping instead of relocating:

```bash
ros2 launch semantic_map_enricher go2_semantic_map.launch.py \
  yaml_path:=/absolute/path/to/my_map_segments_from_rose2.yaml \
  pose_topic:=/lio_sam_ros2/mapping/odometry
```

The YOLO adapter must publish `std_msgs/msg/String`. The node accepts a single
label, comma-separated labels, a YAML/JSON list, or a detection dictionary:

```text
chair
chair, table
["chair", "dining table"]
{"detections": [{"class_name": "chair"}, {"label": "table"}]}
```

Objects are stored as a real YAML list and deduplicated case-insensitively:

```yaml
segments:
  - id: 1
    polygon_m: [[...], [...], [...]]
    room_name: unknown
    objects:
      - chair
      - dining table
```

Room assignment deliberately uses only `polygon_m` (or `corners_m` as a
fallback). There is no nearest-centre fallback, so detections made in an
unsegmented doorway or outside the map do not contaminate a nearby room. If
the odometry frame is not `map`, the node transforms the pose through TF.

On every room transition the in-memory detection history is cleared. A pose
callback never writes the last object message: only a newly received YOLO
message can modify the current room. Therefore objects from the old room are
not copied merely because the Go2 crosses a room boundary. Object lists
already stored in previously visited rooms remain in the YAML. Writes are
atomic, so an interrupted save cannot leave a truncated map file.

## Parameters

Parameters can be supplied through the launch file or directly with
`--ros-args -p name:=value`.

| Node | Parameter | Default | Meaning |
|---|---|---:|---|
| `rose` | `filter` | `0.18` | Structural-pixel filtering level |
| `rose` | `pub_once` | `true` | Publish each computed result once |
| `rose2` | `pub_once` | `false` | Repeatedly publish finished ROSE2 results |
| `rose2` | `spatial_clustering_threshold` | `5.0` | Spatial line-cluster threshold |
| `rose2` | `lines_threshold` | `0.0` | Minimum extended-line weight |
| `rose2` | `lines_distance` | `20.0` | Extended-line merge distance |
| `rose2` | `edges_threshold` | `0.0` | Minimum edge weight |
| `rose2` | `use_voronoi` | `false` | Enable Voronoi room refinement |
| `rose2` | `output_directory` | `/tmp/rose2_voronoi` | Voronoi image output |
| `yaml_extractor` | `output_yaml` | empty | Explicit output file path |
| `yaml_extractor` | `map_yaml` | `rose2_live.yaml` | Source-map path recorded in the YAML and used to derive the default output path |
| `yaml_extractor` | `features_topic` | `/features_ROSE2` | ROSE2 room-polygon topic |
| `yaml_extractor` | `rose_features_topic` | `/features_ROSE` | Preferred source for exact map metadata |
| `yaml_extractor` | `map_topic` | `/map` | Fallback map-metadata topic |
| `yaml_extractor` | `write_once` | `true` | Stop automatic writes after the first successful export |
| `yaml_extractor` | `auto_save` | `true` | Write automatically when both inputs arrive |
| `go2_object_mapper` | `yaml_path` | empty | Existing ROSE2 semantic YAML to update |
| `go2_object_mapper` | `pose_topic` | `/lio_sam_ros2/mapping/re_location_odometry` | Unitree relocation odometry |
| `go2_object_mapper` | `detected_objects_topic` | `/detected_objects` | YOLO labels as `std_msgs/msg/String` |
| `go2_object_mapper` | `segment_id_topic` | `/current_segment_id` | Current room ID; `-1` means outside |
| `go2_object_mapper` | `map_frame` | `map` | Coordinate frame used by `polygon_m` |
| `go2_object_mapper` | `transform_pose_to_map` | `true` | Transform non-map odometry through TF |
| `go2_object_mapper` | `tf_timeout_sec` | `0.2` | Maximum TF lookup duration |
| `go2_object_mapper` | `max_pose_age_sec` | `2.0` | Reject detections without a recent pose |
| `go2_object_mapper` | `auto_save` | `true` | Save after new labels are added |

The launch argument for the ROSE2 `pub_once` parameter is
`rose2_pub_once:=true`.

## ROS 1 to ROS 2 API changes

- Message fields use ROS 2-compatible `snake_case` names (`original_map`,
  `clean_map`).
- ROS 1 XML launch files were replaced by `launch/rose.launch.py`.
- Nodes use timers rather than blocking `rospy.Rate` loops.
- Clean maps are valid trinary `OccupancyGrid` messages (`0` and `100`) rather
  than raw unsigned image bytes in an `int8[]` field.

## License and attribution

GPL-3.0. See [LICENSE](LICENSE). The segmentation algorithms originate from
the upstream ROSE2 project by Gabriele Somaschini and Matteo Luperto.
