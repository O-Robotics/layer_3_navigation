#!/usr/bin/env python3

import json
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_json_file(path):
    if not path or not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def _resolve_execution_context(missions_directory, execution_pointer_file, mission_execution_directory):
    if mission_execution_directory:
        return _load_json_file(
            str(Path(mission_execution_directory) / "execution_context.json")
        )

    pointer_path = Path(execution_pointer_file)
    if not pointer_path.is_absolute():
        pointer_path = Path(missions_directory) / pointer_path

    pointer = _load_json_file(str(pointer_path))
    context_path = pointer.get("execution_context_file", "")
    if not context_path:
        mission_run_directory = pointer.get("mission_run_directory", "")
        if mission_run_directory:
            context_path = str(Path(mission_run_directory) / "execution_context.json")

    return _load_json_file(context_path)


def _build_nodes(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    params_file = LaunchConfiguration("mapping_params_file").perform(context)
    missions_directory = LaunchConfiguration("missions_directory").perform(context)
    execution_pointer_file = LaunchConfiguration("execution_pointer_file").perform(context)
    mission_execution_directory = LaunchConfiguration("mission_execution_directory").perform(context)

    mission_context = _resolve_execution_context(
        missions_directory,
        execution_pointer_file,
        mission_execution_directory,
    )
    mission_id = mission_context.get("mission_id", "")
    mission_file = mission_context.get("mission_file", "")
    mission_route_file = mission_context.get("mission_route_file", "")
    mission_run_directory = mission_context.get("mission_run_directory", missions_directory)
    mission_window_start = mission_context.get("mission_window_start", "")
    mission_window_end = mission_context.get("mission_window_end", "")
    actual_path_output_file = mission_context.get(
        "actual_path_file",
        str(Path(mission_run_directory) / "actual_path.geojson"),
    )
    gaussian_output_directory = mission_context.get(
        "gaussian_output_directory",
        str(Path(mission_run_directory) / "gaussian"),
    )

    gaussian_representation_name = (
        f"{mission_id}_gaussian_map" if mission_id else "global_gaussian_map"
    )

    common_runtime_parameters = {
        "use_sim_time": use_sim_time,
        "mission_file": mission_file,
        "mission_route_file": mission_route_file,
        "mission_id": mission_id,
        "mission_output_directory": mission_run_directory,
        "actual_path_output_file": actual_path_output_file,
        "mission_window_start": mission_window_start,
        "mission_window_end": mission_window_end,
        "end_mission_service": "end_mission",
    }

    return [
        Node(
            package="amr_sweeper_mapping",
            executable="slam_node",
            name="amr_sweeper_slam_node",
            namespace=namespace,
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="amr_sweeper_mapping",
            executable="gaussian_node",
            name="amr_sweeper_gaussian_node",
            namespace=namespace,
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                    "output_directory": gaussian_output_directory,
                    "representation_name": gaussian_representation_name,
                },
            ],
        ),
        Node(
            package="amr_sweeper_mapping",
            executable="mapping_node",
            name="amr_sweeper_mapping_node",
            namespace=namespace,
            output="screen",
            parameters=[params_file, common_runtime_parameters],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="amr_sweeper",
                description="Namespace for mapping nodes",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use ROS time if true",
            ),
            DeclareLaunchArgument(
                "mapping_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("amr_sweeper_mapping"), "config", "mapping_params.yaml"]
                ),
                description="Shared parameter file for mapping package nodes.",
            ),
            DeclareLaunchArgument(
                "missions_directory",
                default_value="src/missions_log",
                description="Root missions directory used for active runtime aliases and execution selection.",
            ),
            DeclareLaunchArgument(
                "execution_pointer_file",
                default_value="active_execution.json",
                description="Fallback scheduler-written JSON pointer used when mission_execution_directory is empty.",
            ),
            DeclareLaunchArgument(
                "mission_execution_directory",
                default_value="",
                description="Exact scheduler-selected mission execution folder for the active RUNNING mission.",
            ),
            OpaqueFunction(function=_build_nodes),
        ]
    )
