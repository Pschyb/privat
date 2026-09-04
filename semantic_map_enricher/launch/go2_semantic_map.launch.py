import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_namespace = os.environ.get("ROBOT_NS", "go2_unit_001").strip("/")
    default_depth_topic = (
        f"/{robot_namespace}/{robot_namespace}/"
        "aligned_depth_to_color/image_raw"
    )
    yaml_path = LaunchConfiguration("yaml_path")
    pose_topic = LaunchConfiguration("pose_topic")
    detected_objects_topic = LaunchConfiguration("detected_objects_topic")
    segment_id_topic = LaunchConfiguration("segment_id_topic")
    map_frame = LaunchConfiguration("map_frame")
    transform_pose_to_map = LaunchConfiguration("transform_pose_to_map")
    tf_timeout_sec = LaunchConfiguration("tf_timeout_sec")
    max_pose_age_sec = LaunchConfiguration("max_pose_age_sec")
    auto_save = LaunchConfiguration("auto_save")
    distance_filter_enabled = LaunchConfiguration("distance_filter_enabled")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    min_object_distance_m = LaunchConfiguration("min_object_distance_m")
    max_object_distance_m = LaunchConfiguration("max_object_distance_m")
    max_depth_age_sec = LaunchConfiguration("max_depth_age_sec")
    depth_roi_fraction = LaunchConfiguration("depth_roi_fraction")
    min_valid_depth_pixels = LaunchConfiguration("min_valid_depth_pixels")

    return LaunchDescription(
        [
            DeclareLaunchArgument("yaml_path", default_value=""),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/lio_sam_ros2/mapping/re_location_odometry",
            ),
            DeclareLaunchArgument(
                "detected_objects_topic", default_value="/detected_objects"
            ),
            DeclareLaunchArgument(
                "segment_id_topic", default_value="/current_segment_id"
            ),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("transform_pose_to_map", default_value="true"),
            DeclareLaunchArgument("tf_timeout_sec", default_value="0.2"),
            DeclareLaunchArgument("max_pose_age_sec", default_value="2.0"),
            DeclareLaunchArgument("auto_save", default_value="true"),
            DeclareLaunchArgument(
                "distance_filter_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "depth_image_topic", default_value=default_depth_topic
            ),
            DeclareLaunchArgument("min_object_distance_m", default_value="0.2"),
            DeclareLaunchArgument("max_object_distance_m", default_value="3.0"),
            DeclareLaunchArgument("max_depth_age_sec", default_value="0.5"),
            DeclareLaunchArgument("depth_roi_fraction", default_value="0.5"),
            DeclareLaunchArgument("min_valid_depth_pixels", default_value="20"),
            Node(
                package="semantic_map_enricher",
                executable="go2_object_mapper",
                name="go2_object_mapper",
                output="screen",
                parameters=[
                    {
                        "yaml_path": yaml_path,
                        "pose_topic": pose_topic,
                        "detected_objects_topic": detected_objects_topic,
                        "segment_id_topic": segment_id_topic,
                        "map_frame": map_frame,
                        "transform_pose_to_map": ParameterValue(
                            transform_pose_to_map, value_type=bool
                        ),
                        "tf_timeout_sec": ParameterValue(
                            tf_timeout_sec, value_type=float
                        ),
                        "max_pose_age_sec": ParameterValue(
                            max_pose_age_sec, value_type=float
                        ),
                        "auto_save": ParameterValue(auto_save, value_type=bool),
                        "distance_filter_enabled": ParameterValue(
                            distance_filter_enabled, value_type=bool
                        ),
                        "depth_image_topic": depth_image_topic,
                        "min_object_distance_m": ParameterValue(
                            min_object_distance_m, value_type=float
                        ),
                        "max_object_distance_m": ParameterValue(
                            max_object_distance_m, value_type=float
                        ),
                        "max_depth_age_sec": ParameterValue(
                            max_depth_age_sec, value_type=float
                        ),
                        "depth_roi_fraction": ParameterValue(
                            depth_roi_fraction, value_type=float
                        ),
                        "min_valid_depth_pixels": ParameterValue(
                            min_valid_depth_pixels, value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
