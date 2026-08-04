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
