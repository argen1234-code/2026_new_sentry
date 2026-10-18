#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包的共享目录
    sentry_decision_dir = get_package_share_directory('sentry_decision')
    
    # 声明launch参数
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(sentry_decision_dir, 'config', 'sentry_decision_params.yaml'),
        description='哨兵决策节点参数文件的完整路径'
    )
    
    # 创建节点
    sentry_decision_node = Node(
        package='sentry_decision',
        executable='sentry_decision_node',
        name='sentry_decision_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True
    )
    
    return LaunchDescription([
        params_file_arg,
        sentry_decision_node
    ])
