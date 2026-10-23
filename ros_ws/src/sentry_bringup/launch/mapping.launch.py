import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
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


def truthy(context, name):
    """Normalize a launch argument once: true/1/yes/on are true, rest false.

    Using this instead of IfCondition avoids the launch condition parser, which
    raises on values such as `yes` even though many people type them.
    """
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        "true", "1", "yes", "on")


def teleop_gate(context):
    keyboard = truthy(context, "keyboard_teleop")
    forced = truthy(context, "enable_motion")
    # The USB motion gate is keyboard_teleop OR enable_motion.  With the gate
    # closed cmd_vel_to_unified publishes status=0 and the MCU falls back to
    # its RC receiver.
    gate = "true" if (keyboard or forced) else "false"
    if keyboard:
        msg = (
            "遥控建图：上位机键盘遥控已启用（运动门控打开）。请在另一个终端运行 "
            "`ros2 run usb_bridge keyboard_teleop`；该节点运行期间（即使空闲）"
            "固件认为 USB 在线，遥控器被屏蔽。"
        )
    else:
        msg = (
            "遥控建图：上位机键盘遥控已禁用，/cmd_vel 不会驱动底盘；"
            "底盘仅接受下位机遥控器（需固件 Input/DBUS 已启用）。"
        )
    return [
        LogInfo(msg=msg),
        include("usb_bridge", "usb_bridge.launch.py", {"enable_motion": gate}),
    ]


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("pb2025_nav_bringup"),
        "rviz",
        "nav2_compact_view.rviz",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config,
            description="RViz config file; use nav2_compact_gray.rviz for the grayscale costmap palette",
        ),
        DeclareLaunchArgument(
            "chassis_model",
            default_value="omni_round",
            choices=["sentry_2026", "mecanum", "sentry", "omni_round"],
            description="Static chassis model: official 2026, mecanum training car, earlier steering sentry, or round omni chassis",
        ),
        DeclareLaunchArgument(
            "keyboard_teleop",
            default_value="true",
            description=(
                "Enable host keyboard teleop while mapping. true opens the USB "
                "motion gate so `ros2 run usb_bridge keyboard_teleop` can drive "
                "the chassis; false closes it, so the host keyboard has no effect "
                "and the chassis is left to the RC receiver."
            ),
        ),
        DeclareLaunchArgument(
            "enable_motion",
            default_value="false",
            description=(
                "Force the USB motion gate open regardless of keyboard_teleop. "
                "Kept for compatibility; keyboard_teleop is the normal switch."
            ),
        ),
        include("sentry_hardware", "lidar.launch.py"),
        OpaqueFunction(function=teleop_gate),
        include("sentry_description", "description.launch.py", {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "chassis_model": LaunchConfiguration("chassis_model"),
        }),
        include("sentry_perception", "perception.launch.py"),
        include("sentry_localization", "mapping.launch.py"),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
    ])
