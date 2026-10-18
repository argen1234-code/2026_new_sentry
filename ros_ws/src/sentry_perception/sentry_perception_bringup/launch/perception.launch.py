import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = get_package_share_directory("sentry_perception")
    params = os.path.join(share, "config", "perception.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("lidar_frame", default_value="front_mid360"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("robot_base_frame", default_value="gimbal_yaw"),
        Node(package="loam_interface", executable="loam_interface_node", name="loam_interface", output="screen", parameters=[params]),
        Node(package="terrain_analysis", executable="terrainAnalysis", name="terrain_analysis", output="screen", parameters=[params]),
        Node(package="terrain_analysis_ext", executable="terrainAnalysisExt", name="terrain_analysis_ext", output="screen", parameters=[params]),
        Node(package="sensor_scan_generation", executable="sensor_scan_generation_node", name="sensor_scan_generation", output="screen", parameters=[{"lidar_frame": LaunchConfiguration("lidar_frame"), "base_frame": LaunchConfiguration("base_frame"), "robot_base_frame": LaunchConfiguration("robot_base_frame")}]),
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node", name="pointcloud_to_laserscan", output="screen", parameters=[params], remappings=[("cloud_in", "terrain_map_ext"), ("scan", "obstacle_scan")]),
    ])
