"""Mapping pipeline used by the real-robot startup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.descriptions import ParameterFile
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params_file = LaunchConfiguration("nav2_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params = ParameterFile(RewrittenYaml(
        source_file=params_file, root_key="",
        param_rewrites={"use_sim_time": use_sim_time}, convert_types=True,
    ), allow_substs=True)

    # slam_toolbox is a lifecycle node: without configure+activate it stays
    # unconfigured and never publishes /map.  Follow the upstream
    # online_sync_launch.py pattern.
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox", executable="sync_slam_toolbox_node",
        name="slam_toolbox", namespace="", output="screen",
        parameters=[params, {"use_lifecycle_manager": False}],
        remappings=[("/map", "map"), ("/map_metadata", "map_metadata"),
                    ("/map_updates", "map_updates")])

    return LaunchDescription([
        DeclareLaunchArgument("nav2_params_file", default_value=os.path.join(
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
        slam_toolbox_node,
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_toolbox_node),
                transition_id=Transition.TRANSITION_CONFIGURE),
            condition=IfCondition(autostart)),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_toolbox_node,
                start_state="configuring",
                goal_state="inactive",
                entities=[EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE))]),
            condition=IfCondition(autostart)),
        Node(package="point_lio", executable="pointlio_mapping", name="point_lio",
             output="screen", parameters=[params, {"prior_pcd.enable": False,
                                                   "pcd_save.pcd_save_en": True}]),
        # map->odom 由 slam_toolbox 发布（transform_publish_period: 0.02），
        # 不要再加静态 map->odom，否则两个发布者会互相覆盖。
    ])
