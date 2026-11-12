# Copyright 2026 The 2026_new_sentry Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Start the wheel/LIO fusion node standalone.

The full stack starts it through sentry_bringup/sentry.launch.py with
use_wheel_fusion:=true. This file is for bag replay or manual debugging.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="回放 bag 时置 true，需配合 ros2 bag play --clock"),
        Node(
            package="wheel_odom_fusion",
            executable="wheel_odom_fusion_node",
            name="wheel_odom_fusion",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
            }],
        ),
    ])
