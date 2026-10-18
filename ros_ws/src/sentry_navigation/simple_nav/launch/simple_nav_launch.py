from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('simple_nav'),
        'config',
        'simple_nav_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='simple_nav',
            executable='simple_nav_node',
            name='simple_nav_node',
            parameters=[config],
            output='screen',
        )
    ])
