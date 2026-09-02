#!/usr/bin/env python3
#
# Copyright 2026 O-Robotics
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    return [
        Node(
            package='amr_sweeper_localization',
            executable='visual_odometry_node',
            name='visual_odometry_node',
            namespace=namespace,
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                LaunchConfiguration('visual_odometry_params_file'),
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        )
    ]


def generate_launch_description():
    package_share = FindPackageShare('amr_sweeper_localization')
    params_file = PathJoinSubstitution(
        [package_share, 'config', 'amr_sweeper_visual_odometry.yaml']
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='amr_sweeper',
                description='Namespace for visual odometry nodes',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use ROS time if true',
            ),
            DeclareLaunchArgument(
                'use_simulation',
                default_value='false',
                description='Retained for launch API compatibility; VO can run in simulation.',
            ),
            DeclareLaunchArgument(
                'log_level',
                default_value='info',
                description='Log level for launched nodes',
            ),
            DeclareLaunchArgument(
                'visual_odometry_params_file',
                default_value=params_file,
                description='Parameter file for the visual odometry node',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
