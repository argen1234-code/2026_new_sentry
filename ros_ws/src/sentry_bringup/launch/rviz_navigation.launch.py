"""Real-robot mapping/navigation stack with RViz as the only visualizer.

This entry point deliberately has no Gazebo, ros_gz, SDF, or simulation-clock
dependency.  It is intended for a Livox-equipped robot (or a recorded ROS bag
publishing the same sensor topics).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def include(package, launch_file, arguments=None, condition=None):
    source = os.path.join(get_package_share_directory(package), "launch", launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(source),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    nav_share = get_package_share_directory("pb2025_nav_bringup")
    localization_share = get_package_share_directory("sentry_localization")
    params_file = os.path.join(nav_share, "config", "reality", "nav2_params.yaml")
    # navigation_launch.py is also used by multi-robot setups and expects its
    # caller to expand this placeholder.  This entry point is deliberately a
    # single, root-namespace robot, so resolve it before passing the file on.
    resolved_params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ""},
    )
    rviz_config = os.path.join(nav_share, "rviz", "nav2_default_view.rviz")
    map_default = os.path.join(nav_share, "map", "reality", "RMUL.yaml")
    pcd_default = os.path.join(nav_share, "pcd", "reality", "RMUL.pcd")

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam = LaunchConfiguration("slam")
    rviz = LaunchConfiguration("rviz")
    namespace = LaunchConfiguration("namespace")

    # Point-LIO and slam_toolbox are started here so their real-robot topic and
    # frame configuration stays independent of the simulation launch files.
    mapping = include(
        "sentry_localization",
        "mapping.launch.py",
        {"use_sim_time": use_sim_time},
        condition=IfCondition(slam),
    )
    localization = include(
        "sentry_localization",
        "localization.launch.py",
        {
            "use_sim_time": use_sim_time,
            "map": LaunchConfiguration("map"),
            "prior_pcd_file": LaunchConfiguration("prior_pcd_file"),
        },
        condition=LaunchConfigurationEquals("slam", "false"),
    )

    # navigation_launch supplies terrain processing, odometry/scan adapters,
    # and the Nav2 planner/controller servers.  The point-cloud-to-scan node
    # below is kept here because it is also required by slam_toolbox in mapping
    # mode and is intentionally a real-robot-only node.
    navigation = include(
        "pb2025_nav_bringup",
        "navigation_launch.py",
        {
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": resolved_params_file,
            "use_composition": "False",
        },
    )

    cloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[resolved_params_file],
        remappings=[("cloud_in", "terrain_map_ext"), ("scan", "obstacle_scan")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "slam",
                default_value="true",
                description="Build a live map when true; use map/PCD localization when false",
            ),
            DeclareLaunchArgument("map", default_value=map_default),
            DeclareLaunchArgument("prior_pcd_file", default_value=pcd_default),
            DeclareLaunchArgument("rviz", default_value="true"),
            include("sentry_hardware", "lidar.launch.py", {"frame_id": "front_mid360"}),
            include(
                "sentry_description",
                "description.launch.py",
                {"use_sim_time": use_sim_time},
            ),
            mapping,
            localization,
            cloud_to_scan,
            navigation,
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(rviz),
                arguments=["-d", rviz_config],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
        ]
    )
