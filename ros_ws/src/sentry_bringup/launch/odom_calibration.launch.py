"""轮式里程计 TF 标定专用启动。

只启动标定必需的五样，故意不启 Nav2 / sensor_scan_generation / small_gicp：
  - sensor_scan_generation 也发 odom -> base_footprint，会和本阶段的
    odom_wheel -> base_footprint 抢同一个子帧（tf2 按子帧索引父帧，后写覆盖前写）；
  - Nav2 与 GICP 在这一步没有用处，还能给 Jetson 省内存。

LIO 侧位姿从 loam_interface 的 /lidar_odometry 话题读（frame odom, child
front_mid360）。雷达在 URDF 里 yaw=0，水平方向与 base_footprint 一致。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, filename, arguments=None):
    path = os.path.join(get_package_share_directory(package), "launch", filename)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    point_lio_cfg = os.path.join(
        get_package_share_directory("sentry_localization"),
        "config", "point_lio_mid360.yaml")
    nav2_params = os.path.join(
        get_package_share_directory("pb2025_nav_bringup"),
        "config", "reality", "nav2_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("chassis_model", default_value="test_chassis"),
        DeclareLaunchArgument(
            "enable_motion", default_value="false",
            description="必须显式 true 才允许 /cmd_vel 驱动底盘"),
        GroupAction(
            actions=[
                include("sentry_description", "description.launch.py", {
                    "use_sim_time": "false",
                    "use_rviz": "false",
                    "chassis_model": LaunchConfiguration("chassis_model"),
                }),
            ],
            scoped=True,
        ),
        include("sentry_hardware", "lidar.launch.py", {"frame_id": "front_mid360"}),
        include("usb_bridge", "usb_bridge.launch.py", {
            "enable_motion": LaunchConfiguration("enable_motion"),
            "odom_publish_tf": "true",
        }),
        Node(
            package="point_lio",
            executable="pointlio_mapping",
            name="point_lio",
            output="screen",
            parameters=[point_lio_cfg, {
                "prior_pcd.enable": False,
                "pcd_save.pcd_save_en": False,
            }],
        ),
        Node(
            package="loam_interface",
            executable="loam_interface_node",
            name="loam_interface",
            output="screen",
            parameters=[nav2_params],
        ),
    ])
