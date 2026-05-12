#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="amr_sweeper",
                description="Namespace for localization nodes",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use ROS time if true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("amr_sweeper_localization"),
                            "launch",
                            "fusioncore.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            )
        ]
    )
