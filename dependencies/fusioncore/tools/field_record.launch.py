"""
Field day bag recording launch file.

Starts FusionCore with the outdoor config and a ros2 bag recorder
capturing all topics needed for the FusionCore vs robot_localization comparison.

Usage:
    ros2 launch tools/field_record.launch.py

Or with a custom config:
    ros2 launch tools/field_record.launch.py \
        fusioncore_config:=/path/to/your_robot.yaml \
        bag_prefix:=run_02
"""

import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


RECORD_TOPICS = [
    "/imu/data",
    "/odom/wheels",
    "/gnss/fix",
    "/gnss/status",
    "/tf_static",
    "/fusion/odom",
    "/fusion/debug/gnss_status",
    "/fusion/debug/filter_health",
    "/diagnostics",
]


def _make_nodes(context, *args, **kwargs):
    config = LaunchConfiguration("fusioncore_config").perform(context)
    bag_prefix = LaunchConfiguration("bag_prefix").perform(context)
    env = LaunchConfiguration("env_config").perform(context)

    params = [config]
    if env:
        params.append(env)

    fusioncore = Node(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        output="screen",
        parameters=params,
    )

    bag_dir = os.path.join(os.getcwd(), bag_prefix)
    recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", bag_dir] + RECORD_TOPICS,
        output="screen",
    )

    return [fusioncore, TimerAction(period=3.0, actions=[recorder])]


def generate_launch_description():
    pkg = get_package_share_directory("fusioncore_ros")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")

    return LaunchDescription([
        DeclareLaunchArgument(
            "fusioncore_config",
            default_value=os.path.join(pkg, "config", "van_outdoor_gps.yaml"),
            description="Path to robot config YAML for the field rover",
        ),
        DeclareLaunchArgument(
            "env_config",
            default_value=os.path.join(pkg, "config", "env_urban.yaml"),
            description="Environment preset (env_open.yaml / env_urban.yaml / env_canopy.yaml)",
        ),
        DeclareLaunchArgument(
            "bag_prefix",
            default_value=f"field_{timestamp}",
            description="Output bag directory name (auto-timestamped if not set)",
        ),
        OpaqueFunction(function=_make_nodes),
    ])
