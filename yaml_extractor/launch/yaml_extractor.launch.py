from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    output_yaml = LaunchConfiguration("output_yaml")
    map_yaml = LaunchConfiguration("map_yaml")
    features_topic = LaunchConfiguration("features_topic")
    rose_features_topic = LaunchConfiguration("rose_features_topic")
    map_topic = LaunchConfiguration("map_topic")
    write_once = LaunchConfiguration("write_once")
    auto_save = LaunchConfiguration("auto_save")

    return LaunchDescription(
        [
            DeclareLaunchArgument("output_yaml", default_value=""),
            DeclareLaunchArgument("map_yaml", default_value="rose2_live.yaml"),
            DeclareLaunchArgument("features_topic", default_value="/features_ROSE2"),
            DeclareLaunchArgument(
                "rose_features_topic", default_value="/features_ROSE"
            ),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("write_once", default_value="true"),
            DeclareLaunchArgument("auto_save", default_value="true"),
            Node(
                package="yaml_extractor",
                executable="yaml_extractor",
                name="rose2_yaml_extractor",
                output="screen",
                parameters=[
                    {
                        "output_yaml": output_yaml,
                        "map_yaml": map_yaml,
                        "features_topic": features_topic,
                        "rose_features_topic": rose_features_topic,
                        "map_topic": map_topic,
                        "write_once": ParameterValue(write_once, value_type=bool),
                        "auto_save": ParameterValue(auto_save, value_type=bool),
                    }
                ],
            ),
        ]
    )
