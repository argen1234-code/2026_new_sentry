"""Publish the competition robot TF tree from the URDF exported by SolidWorks."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_setup(context: LaunchContext):
    description_share = get_package_share_directory("sentry_description")
    chassis_model = context.launch_configurations.get("chassis_model", "mecanum")
    override = context.launch_configurations.get("robot_description_file", "")

    if override:
        urdf_path = override
    elif chassis_model == "sentry":
        urdf_path = os.path.join(description_share, "urdf", "robot.urdf")
    else:
        urdf_path = os.path.join(description_share, "urdf", "mecanum_static.urdf")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("description_params_file")
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={"use_sim_time": LaunchConfiguration("use_sim_time")},
            convert_types=True,
        ),
        allow_substs=True,
    )

    return [
        GroupAction(
            [
                Node(
                    package="joint_state_publisher",
                    executable="joint_state_publisher",
                    name="joint_state_publisher",
                    output="screen",
                    respawn=LaunchConfiguration("use_respawn"),
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                ),
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    output="screen",
                    respawn=LaunchConfiguration("use_respawn"),
                    respawn_delay=2.0,
                    parameters=[configured_params, {"robot_description": robot_description}],
                    arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                ),
                Node(
                    condition=IfCondition(LaunchConfiguration("use_rviz")),
                    package="rviz2",
                    executable="rviz2",
                    name="robot_model_rviz",
                    output="screen",
                    arguments=["-d", LaunchConfiguration("rviz_config_file")],
                ),
            ]
        ),
    ]


def generate_launch_description():
    description_share = get_package_share_directory("sentry_description")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "chassis_model",
            default_value="mecanum",
            choices=["mecanum", "sentry"],
            description="Static robot model: mecanum training chassis or sentry steering chassis",
        ),
        DeclareLaunchArgument(
            "robot_description_file",
            default_value="",
            description="Explicit URDF path; overrides chassis_model when non-empty",
        ),
        DeclareLaunchArgument(
            "description_params_file",
            default_value=os.path.join(description_share, "config", "robot_state_publisher.yaml"),
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=os.path.join(description_share, "rviz", "sentry_description.rviz"),
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_respawn", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        OpaqueFunction(function=launch_setup),
    ])
