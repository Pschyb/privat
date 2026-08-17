from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    yaml_path = LaunchConfiguration("yaml_path")
    pose_topic = LaunchConfiguration("pose_topic")
    detected_objects_topic = LaunchConfiguration("detected_objects_topic")
    segment_id_topic = LaunchConfiguration("segment_id_topic")
    map_frame = LaunchConfiguration("map_frame")
    transform_pose_to_map = LaunchConfiguration("transform_pose_to_map")
    tf_timeout_sec = LaunchConfiguration("tf_timeout_sec")
    max_pose_age_sec = LaunchConfiguration("max_pose_age_sec")
    auto_save = LaunchConfiguration("auto_save")

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
                    }
                ],
            ),
        ]
    )
