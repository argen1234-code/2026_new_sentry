"""Complete real-robot startup matching the competition desktop script."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
from nav2_common.launch import ReplaceString, RewrittenYaml


def include(package, filename, arguments=None, condition=None):
    path = os.path.join(get_package_share_directory(package), "launch", filename)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def visualization_nodes(context, nav_share):
    mode = LaunchConfiguration("visualization").perform(context)
    if mode not in ("rviz", "headless", "foxglove"):
        raise ValueError("visualization must be rviz, headless, or foxglove")

    # Preserve use_rviz:=false from the previous startup command.
    if mode == "rviz" and LaunchConfiguration("use_rviz").perform(context).lower() in (
        "false", "0", "no", "off",
    ):
        mode = "headless"

    if mode == "rviz":
        return [Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", os.path.join(nav_share, "rviz", "nav2_compact_view.rviz")],
        )]
    if mode == "foxglove":
        # Resolve the package before any hardware starts, so a missing bridge
        # does not leave a partially running real-robot launch.
        from ament_index_python.packages import PackageNotFoundError

        try:
            get_package_share_directory("foxglove_bridge")
        except PackageNotFoundError as exc:
            raise RuntimeError(
                "Foxglove mode requires ros-jazzy-foxglove-bridge "
                "(sudo apt install ros-jazzy-foxglove-bridge)"
            ) from exc
        return [Node(
            package="foxglove_bridge", executable="foxglove_bridge",
            name="foxglove_bridge", output="screen",
            parameters=[{
                "address": LaunchConfiguration("foxglove_address"),
                "port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
            }],
        )]
    return []


def generate_launch_description():
    nav_share = get_package_share_directory("pb2025_nav_bringup")
    decision_share = get_package_share_directory("sentry_decision")
    params_file = os.path.join(nav_share, "config", "reality", "nav2_params.yaml")
    # Preserve the old bringup's root-namespace substitution for costmap topics.
    resolved_params = ReplaceString(
        source_file=params_file, replacements={"<robot_namespace>": ""}
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=resolved_params, root_key="",
            param_rewrites={"use_sim_time": use_sim_time, "autostart": autostart},
            convert_types=True,
        ), allow_substs=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument(
            "chassis_model",
            default_value="mecanum",
            choices=["mecanum", "sentry"],
            description="Static chassis model: mecanum training car or sentry steering car",
        ),
        DeclareLaunchArgument(
            "chassis_transport",
            default_value="usb",
            choices=["usb", "udp"],
            description="Chassis command transport: USB CDC bridge or legacy UDP",
        ),
        DeclareLaunchArgument(
            "enable_motion",
            default_value="false",
            description="Allow /cmd_vel to drive the chassis over USB (safety gate)",
        ),
        DeclareLaunchArgument(
            "visualization",
            default_value="rviz",
            description="Visualization backend: rviz, foxglove, or headless",
        ),
        # Kept for compatibility with the previous entry point. Set
        # visualization:=headless or visualization:=foxglove for other modes.
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # Listen on all interfaces so a Foxglove client on another machine can
        # connect through the robot's LAN address. Override this with
        # foxglove_address:=127.0.0.1 for local-only access.
        DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument("udp_port", default_value="19002"),
        DeclareLaunchArgument("referee_device", default_value="/dev/ttyACM0"),
        OpaqueFunction(function=visualization_nodes, args=[nav_share]),
        # Keep the description RViz disabled without overwriting this
        # bringup's own use_rviz launch configuration.
        GroupAction(
            actions=[
                include("sentry_description", "description.launch.py", {
                    "use_sim_time": use_sim_time,
                    "use_rviz": "false",
                    "chassis_model": LaunchConfiguration("chassis_model"),
                }),
            ],
            scoped=True,
        ),
        include("sentry_hardware", "lidar.launch.py", {"frame_id": "front_mid360"}),
        include("usb_bridge", "usb_bridge.launch.py", {
            "enable_motion": LaunchConfiguration("enable_motion"),
        }, condition=LaunchConfigurationEquals("chassis_transport", "usb")),
        Node(package="rclcpp_components", executable="component_container_isolated",
             name="nav2_container", output="screen",
             parameters=[configured_params, {"autostart": autostart}]),
        include("sentry_localization", "mapping.launch.py", {
            "nav2_params_file": resolved_params, "use_sim_time": use_sim_time,
            "autostart": autostart,
        }),
        include("pb2025_nav_bringup", "navigation_launch.py", {
            "params_file": resolved_params, "use_sim_time": use_sim_time,
            "autostart": autostart, "use_composition": "True",
            "container_name": "nav2_container",
        }),
        include("pb2025_nav_bringup", "joy_teleop_launch.py", {
            "joy_config_file": resolved_params, "use_sim_time": use_sim_time,
        }),
        # The desktop script starts these three processes three seconds after navigation.
        TimerAction(period=3.0, actions=[
            Node(package="extra_cmd", executable="cmd_vel_to_udp", name="cmd_vel_to_udp",
                 output="screen", condition=LaunchConfigurationEquals(
                     "chassis_transport", "udp"),
                 parameters=[{"udp_ip": "0.0.0.0",
                              "udp_port": LaunchConfiguration("udp_port")}]),
            Node(package="referee_serial_bridge", executable="referee_serial_bridge_node",
                 name="referee_serial_bridge", output="screen",
                 parameters=[{"device": LaunchConfiguration("referee_device")}]),
            include("sentry_decision", "sentry_decision_launch.py", {
                "params_file": os.path.join(decision_share, "config", "sentry_decision_params.yaml"),
            }),
        ]),
    ])
