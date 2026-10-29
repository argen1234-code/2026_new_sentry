"""Offline Nav2 stack for rosbag replay.

Starts everything except the bag itself, so playback can begin only after the
lifecycle stack is active (see clock_preroll.py and run_variant.py).

Does NOT start: livox driver, usb_bridge, cmd_vel_to_unified, referee, joy,
sentry_decision, terrain_analysis*, loam_interface, sensor_scan_generation, or a
nav2 container -- each of those either needs hardware or would fight the
simulated odometry for TF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))


def include(package, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", filename)),
        launch_arguments=arguments.items(),
    )


def script(name, extra=()):
    return ExecuteProcess(
        cmd=["python3", os.path.join(HERE, f"{name}.py"), "--ros-args"] + list(extra),
        name=name,
        output="screen",
    )


def generate_launch_description():
    nav_share = get_package_share_directory("pb2025_nav_bringup")

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(nav_share, "config", "reality", "nav2_params.yaml")),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(REPO, "maps", "corridor_1025_tape.yaml")),
        DeclareLaunchArgument("map_odom_x", default_value="0.043"),
        DeclareLaunchArgument("map_odom_y", default_value="0.007"),
        DeclareLaunchArgument("map_odom_z", default_value="0.060"),
        DeclareLaunchArgument("map_odom_yaw", default_value="-0.004363"),
        DeclareLaunchArgument("init_x", default_value="-0.03"),
        DeclareLaunchArgument("init_y", default_value="0.005"),
        DeclareLaunchArgument("init_yaw", default_value="0.0"),
        DeclareLaunchArgument("plant_json", default_value=""),
        DeclareLaunchArgument("crop", default_value="true"),
        DeclareLaunchArgument("local_radius", default_value="6.0"),
        DeclareLaunchArgument("ext_radius", default_value="12.0"),
        DeclareLaunchArgument("chassis_model", default_value="omni_round"),
        DeclareLaunchArgument("rviz", default_value="false"),
    ]

    sim = {"use_sim_time": True}

    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen",
        parameters=[{"yaml_filename": LaunchConfiguration("map"),
                     "topic_name": "map", "frame_id": "map", **sim}],
    )

    # Delayed: nav2_lifecycle_manager only waits a few ms for the graph before
    # sending CONFIGURE, and started in the same instant it can beat map_server's
    # node creation -- it then waits on a service that never had a client and
    # hangs forever (/map never published, costmap static layer stays empty).
    map_lifecycle = TimerAction(
        period=3.0,
        actions=[Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_map", output="screen",
            parameters=[{**sim, "autostart": True, "node_names": ["map_server"]}],
        )],
    )

    map_to_odom = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="static_map_to_odom", output="screen",
        arguments=[
            "--x", LaunchConfiguration("map_odom_x"),
            "--y", LaunchConfiguration("map_odom_y"),
            "--z", LaunchConfiguration("map_odom_z"),
            "--yaw", LaunchConfiguration("map_odom_yaw"),
            "--pitch", "0", "--roll", "0",
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
    )

    sim_args = [
        "-p", ["plant_json:=", LaunchConfiguration("plant_json")],
        "-p", ["init_x:=", LaunchConfiguration("init_x")],
        "-p", ["init_y:=", LaunchConfiguration("init_y")],
        "-p", ["init_yaw:=", LaunchConfiguration("init_yaw")],
        "-p", "use_sim_time:=true",
    ]

    kinematic = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "kinematic_sim.py"), "--ros-args"] + sim_args,
        name="kinematic_sim", output="screen",
    )

    crop = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "cloud_crop.py"), "--ros-args",
             "-p", "use_sim_time:=true",
             "-p", ["local_radius:=", LaunchConfiguration("local_radius")],
             "-p", ["ext_radius:=", LaunchConfiguration("ext_radius")]],
        name="cloud_crop", output="screen",
        condition=IfCondition(LaunchConfiguration("crop")),
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", os.path.join(nav_share, "rviz", "nav2_compact_view.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(args + [
        map_server,
        map_lifecycle,
        map_to_odom,
        rviz,
        include("sentry_description", "description.launch.py", {
            "use_sim_time": "true",
            "use_rviz": "false",
            "chassis_model": LaunchConfiguration("chassis_model"),
        }),
        include("pb2025_nav_bringup", "navigation_launch.py", {
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": "true",
            "autostart": "true",
            "use_composition": "False",
            "start_terrain": "false",
            "start_tf_helpers": "false",
        }),
        kinematic,
        crop,
    ])
