import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hardware_share = get_package_share_directory("sentry_hardware")
    default_config = os.path.join(hardware_share, "config", "MID360_config.json")
    rviz_config = os.path.join(hardware_share, "rviz", "lidar.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Livox device network configuration JSON",
            ),
            DeclareLaunchArgument("frame_id", default_value="front_mid360"),
            DeclareLaunchArgument("publish_freq", default_value="20.0"),
            DeclareLaunchArgument("xfer_format", default_value="4"),
            DeclareLaunchArgument("multi_topic", default_value="0"),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Also open RViz2 for TF inspection",
            ),
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_ros_driver2",
                output="screen",
                parameters=[
                    {
                        "user_config_path": LaunchConfiguration("config"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "publish_freq": ParameterValue(
                            LaunchConfiguration("publish_freq"), value_type=float
                        ),
                        "xfer_format": ParameterValue(
                            LaunchConfiguration("xfer_format"), value_type=int
                        ),
                        "multi_topic": ParameterValue(
                            LaunchConfiguration("multi_topic"), value_type=int
                        ),
                        "data_src": 0,
                        "output_data_type": 0,
                        "cmdline_input_bd_code": "livox0000000001",
                        "lvx_file_path": "",
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
                arguments=["-d", rviz_config],
            ),
        ]
    )
