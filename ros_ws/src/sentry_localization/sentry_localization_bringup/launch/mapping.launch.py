"""Mapping pipeline used by the real-robot startup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params = ParameterFile(RewrittenYaml(
        source_file=params_file, root_key="",
        param_rewrites={"use_sim_time": use_sim_time}, convert_types=True,
    ), allow_substs=True)

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            get_package_share_directory("pb2025_nav_bringup"), "config", "reality", "nav2_params.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        Node(package="nav2_map_server", executable="map_saver_server",
             name="map_saver", output="screen", parameters=[params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_slam", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "autostart": autostart,
                          "node_names": ["map_saver"]}]),
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
             name="pointcloud_to_laserscan", output="screen", parameters=[params],
             remappings=[("cloud_in", "terrain_map_ext"), ("scan", "obstacle_scan")]),
        Node(package="slam_toolbox", executable="sync_slam_toolbox_node",
             name="slam_toolbox", output="screen", parameters=[params],
             remappings=[("/map", "map"), ("/map_metadata", "map_metadata"),
                         ("/map_updates", "map_updates")]),
        Node(package="point_lio", executable="pointlio_mapping", name="point_lio",
             output="screen", parameters=[params, {"prior_pcd.enable": False,
                                                   "pcd_save.pcd_save_en": True}]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="static_transform_publisher_map2odom", output="screen",
             arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                        "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                        "--frame-id", "map", "--child-frame-id", "odom"]),
    ])
