from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('usb_bridge'), 'config', 'usb_bridge.yaml')
    return LaunchDescription([
        Node(
            package='usb_bridge',
            executable='usb_bridge',
            name='usb_bridge',
            output='screen',
            parameters=[config],
        ),
    ])
