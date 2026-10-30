from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('usb_bridge'), 'config', 'usb_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('enable_motion', default_value='false',
                              description='Allow /cmd_vel to drive the chassis'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        Node(
            package='usb_bridge',
            executable='usb_bridge',
            name='usb_bridge',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='usb_bridge',
            executable='cmd_vel_to_unified',
            name='cmd_vel_to_unified',
            output='screen',
            parameters=[{
                'enable_motion': ParameterValue(
                    LaunchConfiguration('enable_motion'), value_type=bool),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'max_vx': 1.2,
                'max_vy': 0.6,
                'max_wz': 1.3,
            }],
        ),
    ])
