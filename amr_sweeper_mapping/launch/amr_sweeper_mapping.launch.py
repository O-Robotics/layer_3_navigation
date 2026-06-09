#!/usr/bin/env python3

import json
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_json_file(path):
    if not path or not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def _resolve_execution_context(mission_execution_directory):
    if not mission_execution_directory:
        return {}
    return _load_json_file(
        str(Path(mission_execution_directory) / "execution_context.json")
    )


def _build_nodes(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    params_file = LaunchConfiguration("mapping_params_file").perform(context)
    slam_params_file = LaunchConfiguration("slam_params_file").perform(context)
    mission_execution_directory = LaunchConfiguration("mission_execution_directory").perform(context)
    use_test = LaunchConfiguration("use_test").perform(context).lower() == "true"
    test_output_directory = LaunchConfiguration("test_output_directory").perform(context)
    configured_mission_costmap_yaml = LaunchConfiguration("mission_costmap_yaml").perform(context)
    auto_start_mission = LaunchConfiguration("auto_start_mission").perform(context).lower() == "true"

    mission_context = _resolve_execution_context(mission_execution_directory)
    mission_id = mission_context.get("mission_id", "")
    mission_file = mission_context.get("mission_file", "")
    mission_route_file = mission_context.get("mission_route_file", "")
    mission_run_directory = mission_context.get("mission_run_directory", "")
    mission_window_start = mission_context.get("mission_window_start", "")
    mission_window_end = mission_context.get("mission_window_end", "")
    actual_path_output_file = mission_context.get("actual_path_file", "")
    if not actual_path_output_file and mission_run_directory:
        actual_path_output_file = str(Path(mission_run_directory) / "actual_path.geojson")
    actual_path_navsat_output_file = mission_context.get("actual_path_navsat_file", "")
    if not actual_path_navsat_output_file and mission_run_directory:
        actual_path_navsat_output_file = str(Path(mission_run_directory) / "actual_path_navsat.geojson")
    mission_costmap_yaml = configured_mission_costmap_yaml or mission_context.get(
        "mission_costmap_yaml", ""
    )
    gaussian_output_directory = mission_context.get("gaussian_output_directory", "")
    if not gaussian_output_directory and mission_run_directory:
        gaussian_output_directory = str(Path(mission_run_directory) / "gaussian")
    if use_test and not mission_run_directory and test_output_directory:
        mission_run_directory = test_output_directory
        if not actual_path_output_file:
            actual_path_output_file = str(Path(test_output_directory) / "actual_path.geojson")
        if not actual_path_navsat_output_file:
            actual_path_navsat_output_file = str(Path(test_output_directory) / "actual_path_navsat.geojson")
        if not gaussian_output_directory:
            gaussian_output_directory = test_output_directory

    gaussian_representation_name = (
        f"{mission_id}_gaussian_map" if mission_id else "global_gaussian_map"
    )

    common_runtime_parameters = {
        "use_sim_time": use_sim_time,
        "end_mission_service": "end_mission",
        "auto_start_mission": auto_start_mission,
    }
    if mission_file:
        common_runtime_parameters["mission_file"] = mission_file
    if mission_route_file:
        common_runtime_parameters["mission_route_file"] = mission_route_file
    if mission_id:
        common_runtime_parameters["mission_id"] = mission_id
    if mission_run_directory:
        common_runtime_parameters["mission_output_directory"] = mission_run_directory
    if actual_path_output_file:
        common_runtime_parameters["actual_path_output_file"] = actual_path_output_file
    if actual_path_navsat_output_file:
        common_runtime_parameters["actual_path_navsat_output_file"] = actual_path_navsat_output_file
    if mission_costmap_yaml:
        common_runtime_parameters["mission_costmap_yaml"] = mission_costmap_yaml
    if mission_window_start:
        common_runtime_parameters["mission_window_start"] = mission_window_start
    if mission_window_end:
        common_runtime_parameters["mission_window_end"] = mission_window_end

    actions = []
    if mission_costmap_yaml:
        actions.append(
            LogInfo(
                msg=f"amr_sweeper_mapping using mission costmap from execution context: {mission_costmap_yaml}"
            )
        )
    else:
        actions.append(
            LogInfo(
                msg=(
                    "amr_sweeper_mapping did not receive a mission-specific costmap yaml. "
                    "Standalone launch will continue, and the geojson costmap layer will stay inactive "
                    "until a mission_costmap_yaml is provided."
                )
            )
        )

    actions.extend([
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            namespace=namespace,
            output="screen",
            parameters=[
                slam_params_file,
                {
                    "use_sim_time": use_sim_time,
                    "odom_frame": "odom",
                    "map_frame": "map",
                    "base_frame": "base_footprint",
                    "scan_topic": "depth_camera/scan",
                },
            ],
            remappings=[
                ("scan", "depth_camera/scan"),
                ("pose", "slam/pose"),
            ],
        ),
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
    ])

    return actions


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
                "slam_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("amr_sweeper_mapping"), "config", "slam_toolbox.yaml"]
                ),
                description="slam_toolbox parameter file used for live mapping and map->odom TF publication.",
            ),
            DeclareLaunchArgument(
                "mission_execution_directory",
                default_value="",
                description="Exact scheduler-selected mission execution folder for the active RUNNING mission.",
            ),
            DeclareLaunchArgument(
                "use_test",
                default_value="false",
                description="When true and no mission execution folder is available, write generic mapping test artifacts into test_output_directory.",
            ),
            DeclareLaunchArgument(
                "test_output_directory",
                default_value="src/layer_3_navigation/tests",
                description="Generic test artifact directory used only when use_test is enabled without a mission execution folder.",
            ),
            DeclareLaunchArgument(
                "mission_costmap_yaml",
                default_value="",
                description=(
                    "Exact mission costmap yaml. Leave empty to resolve it from execution_context.json "
                    "or continue standalone with an inactive geojson layer."
                ),
            ),
            DeclareLaunchArgument(
                "auto_start_mission",
                default_value="false",
                description=(
                    "When true, the mapping node will dispatch Nav2 goals after receiving a valid "
                    "mission execution context. RUNNING profiles should enable this explicitly."
                ),
            ),
            OpaqueFunction(function=_build_nodes),
        ]
    )
