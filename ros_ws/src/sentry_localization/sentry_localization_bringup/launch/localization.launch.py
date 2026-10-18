import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = get_package_share_directory("sentry_localization")
    point_lio = os.path.join(share, "config", "point_lio_mid360.yaml")
    gicp = os.path.join(share, "config", "small_gicp.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("prior_pcd_file", default_value=""),
        DeclareLaunchArgument("map", default_value=""),
        Node(package="point_lio", executable="pointlio_mapping", name="point_lio", output="screen", parameters=[point_lio, {"prior_pcd.enable": True, "prior_pcd.prior_pcd_map_path": LaunchConfiguration("prior_pcd_file")}]),
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen", parameters=[{"yaml_filename": LaunchConfiguration("map"), "use_sim_time": False}]),
        Node(package="small_gicp_relocalization", executable="small_gicp_relocalization_node", name="small_gicp_relocalization", output="screen", parameters=[gicp, {"prior_pcd_file": LaunchConfiguration("prior_pcd_file")}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization", output="screen", parameters=[{"autostart": True, "node_names": ["map_server"]}]),
    ])
