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
        DeclareLaunchArgument('odom_publish_tf', default_value='false',
                              description='Publish odom_wheel -> base_footprint TF'),
        DeclareLaunchArgument('wz_gain', default_value='0.80',
                              description='Scale applied to /cmd_vel wz before the chassis'),
        Node(
            package='usb_bridge',
            executable='usb_bridge',
            name='usb_bridge',
            output='screen',
            parameters=[config, {
                'wheel_odom_publish_tf': ParameterValue(
                    LaunchConfiguration('odom_publish_tf'), value_type=bool),
            }],
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
                # 逐轴钳位作用在 frame_yaw 旋转之后的固件车体系上：一条沿 ROS +x
                # 的直行指令在这里会变成 (±0.707v, ±0.707v)，max_vy 0.6 会把
                # 0.85 m/s 以上的直行指令截掉并带偏方向（2026-11-13 round05b 实测：
                # /cmd_vel 1.20 → 下位机 (−0.87,−0.60)，方向偏 10°）。与 max_vx 对称。
                'max_vy': 1.2,
                'max_wz': 1.3,
                # 2026-11-12 实测（~/bags/wz_sweep_1725）：固件把自转放大
                # 实际 = 1.70*wz + 0.0（wz 0.3->0.65，1.2->1.88 rad/s）。
                # 这里压回 1/1.70；平移增益 1.08 未补偿。
                'wz_gain': ParameterValue(
                    LaunchConfiguration('wz_gain'), value_type=float),
            }],
        ),
    ])
