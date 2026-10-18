import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("pb2025_nav_bringup"),
        "rviz",
        "nav2_default_view.rviz",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        include("sentry_hardware", "lidar.launch.py"),
        include("sentry_description", "description.launch.py", {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }),
        include("sentry_perception", "perception.launch.py"),
        include("sentry_localization", "mapping.launch.py"),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", rviz_config],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
    ])
