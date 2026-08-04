from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    pub_once = LaunchConfiguration("pub_once")
    rose2_pub_once = LaunchConfiguration("rose2_pub_once")
    filter_level = LaunchConfiguration("filter")
    use_voronoi = LaunchConfiguration("use_voronoi")
    output_directory = LaunchConfiguration("output_directory")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("pub_once", default_value="true"),
            DeclareLaunchArgument("rose2_pub_once", default_value="false"),
            DeclareLaunchArgument("filter", default_value="0.18"),
            DeclareLaunchArgument("use_voronoi", default_value="false"),
            DeclareLaunchArgument(
                "output_directory", default_value="/tmp/rose2_voronoi"
            ),
            Node(
                package="rose2",
                executable="feature_extractor_rose",
                name="rose",
                output="screen",
                parameters=[
                    {
                        "pub_once": ParameterValue(pub_once, value_type=bool),
                        "filter": ParameterValue(filter_level, value_type=float),
                    }
                ],
            ),
            Node(
                package="rose2",
                executable="feature_extractor_rose2",
                name="rose2",
                output="screen",
                parameters=[
                    {
                        "pub_once": ParameterValue(
                            rose2_pub_once, value_type=bool
                        ),
                        "spatial_clustering_threshold": 5.0,
                        "lines_threshold": 0.0,
                        "lines_distance": 20.0,
                        "edges_threshold": 0.0,
                        "use_voronoi": ParameterValue(
                            use_voronoi, value_type=bool
                        ),
                        "output_directory": output_directory,
                        "voronoi_closeness": 2,
                        "voronoi_blur": 8,
                        "voronoi_iterations": 5,
                    }
                ],
            ),
            Node(
                condition=IfCondition(rviz),
                package="rviz2",
                executable="rviz2",
                name="rose2_rviz",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [FindPackageShare("rose2"), "rviz", "rose2.rviz"]
                    ),
                ],
            ),
        ]
    )
