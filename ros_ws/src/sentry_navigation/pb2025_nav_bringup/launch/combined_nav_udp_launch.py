import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取各包的路径
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")

    # 声明参数
    slam = LaunchConfiguration("slam")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    udp_ip = LaunchConfiguration("udp_ip")
    udp_port = LaunchConfiguration("udp_port")

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="True",
        description="Whether run a SLAM",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    declare_udp_ip_cmd = DeclareLaunchArgument(
        "udp_ip",
        default_value="0.0.0.0",
        description="Target UDP IP address for cmd_vel_to_udp",
    )

    declare_udp_port_cmd = DeclareLaunchArgument(
        "udp_port",
        default_value="19002",
        description="Target UDP port for cmd_vel_to_udp",
    )

    # 1. 导航/建图启动文件
    #    IncludeLaunchDescription 本身不是节点，内部节点崩溃不影响外部其他节点
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rm_navigation_reality_launch.py")
        ),
        launch_arguments={
            "slam": slam,
            "use_robot_state_pub": use_robot_state_pub,
        }.items(),
    )

    # 2. UDP 数据上传节点 (cmd_vel -> UDP)
    #    respawn=True：节点崩溃后自动重启，不影响其他节点
    udp_bridge_node = Node(
        package="extra_cmd",
        executable="cmd_vel_to_udp",
        name="cmd_vel_to_udp",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            "udp_ip": udp_ip,
            "udp_port": udp_port,
        }]
    )

    # 3. 裁判系统串口桥接节点
    #    respawn=True：节点崩溃后自动重启，不影响其他节点
    referee_node = Node(
        package="referee_serial_bridge",
        executable="referee_serial_bridge_node",
        name="referee_serial_bridge",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    ld = LaunchDescription()

    # 添加启动参数
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_udp_ip_cmd)
    ld.add_action(declare_udp_port_cmd)

    # 添加节点和启动文件
    # 注意：ROS2 launch 中各 Node 之间默认互相独立
    #       一个节点退出不会导致其他节点退出
    ld.add_action(navigation_launch)
    ld.add_action(udp_bridge_node)
    ld.add_action(referee_node)

    return ld
