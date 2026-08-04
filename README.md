# ROSE2 for ROS 2 Humble

This repository contains a ROS 2 Humble port of
[aislabunimi/ROSE2](https://github.com/aislabunimi/ROSE2). ROSE removes
non-structural clutter from an occupancy grid and estimates dominant map
directions. ROSE2 then extracts extended wall lines, edges and room polygons.

The original algorithms and GPL-3.0 license are preserved. ROS integration is
implemented natively with `rclpy`, ROS 2 interfaces, Python launch files and
ROS 2 QoS settings.

## Supported platform

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Pschyb/privat.git rose2
cd rose2

python3 -m pip install --user -r requirements-humble.txt

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rose2
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

## ROS interfaces

| Direction | Name | Type |
|---|---|---|
| Subscribe | `/map` | `nav_msgs/msg/OccupancyGrid` |
| Publish | `/clean_map` | `nav_msgs/msg/OccupancyGrid` |
| Publish | `/features_ROSE` | `rose2/msg/ROSEFeatures` |
| Publish | `/features_ROSE2` | `rose2/msg/ROSE2Features` |
| Publish | `/direction_markers` | `visualization_msgs/msg/MarkerArray` |
| Publish | `/extended_lines` | `visualization_msgs/msg/Marker` |
| Publish | `/edges` | `visualization_msgs/msg/MarkerArray` |
| Publish | `/rooms` | `visualization_msgs/msg/MarkerArray` |
| Service | `/ROSESrv` | `rose2/srv/ROSE` |
| Service | `/ROSE2Srv` | `rose2/srv/ROSE2` |

Unlike the ROS 1 package, `/rooms` uses the standard ROS 2 `MarkerArray`
message. The external `jsk_recognition_msgs` and `jsk_rviz_plugins`
dependencies are therefore no longer required.

## Parameters

Parameters can be supplied through the launch file or directly with
`--ros-args -p name:=value`.

| Node | Parameter | Default | Meaning |
|---|---|---:|---|
| `rose` | `filter` | `0.18` | Structural-pixel filtering level |
| `rose` | `pub_once` | `true` | Publish each computed result once |
| `rose2` | `spatial_clustering_threshold` | `5.0` | Spatial line-cluster threshold |
| `rose2` | `lines_threshold` | `0.0` | Minimum extended-line weight |
| `rose2` | `lines_distance` | `20.0` | Extended-line merge distance |
| `rose2` | `edges_threshold` | `0.0` | Minimum edge weight |
| `rose2` | `use_voronoi` | `false` | Enable Voronoi room refinement |
| `rose2` | `output_directory` | `/tmp/rose2_voronoi` | Voronoi image output |

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
