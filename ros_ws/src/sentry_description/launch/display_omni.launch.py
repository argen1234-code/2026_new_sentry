"""Show the standalone test chassis model: robot_state_publisher + RViz.

    ros2 launch sentry_description display_omni.launch.py

Only URDF/TF is published; no drivers, no Nav2. The four wheel joints are
continuous, so joint_state_publisher feeds zeros and the wheels stay at 0 rad.
The wheels carry no geometry here: the CAD mesh already draws them.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("sentry_description")
    xacro_path = os.path.join(share, "urdf", "test_chassis.urdf.xacro")
    robot_description = xacro.process_file(xacro_path).toxml()

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config_file")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=os.path.join(share, "rviz", "test_chassis.rviz"),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="test_chassis_rviz",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
