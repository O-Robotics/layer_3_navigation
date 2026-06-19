#!/usr/bin/env python3
	
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
	
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


def _resolve_execution_context(mission_execution_directory):
    if not mission_execution_directory:
        return {}
    return _load_json_file(
        str(Path(mission_execution_directory) / "execution_context.json")
    )


def _uses_odom_only_runtime(mission_type):
    mission_type = str(mission_type).lower()
    return mission_type in {"builtin_local_pattern", "vda5050_scheduled_mission_local"}


def _build_nodes(context):
    default_record_map_mission_file = str(
        Path(get_package_share_directory("amr_sweeper_navigation"))
        / "missions"
        / "manual_missions"
        / "RecordMap"
        / "RecordMap.json"
    )
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    params_file = LaunchConfiguration("mapping_params_file").perform(context)
    map_pose_params_file = LaunchConfiguration("map_pose_params_file").perform(context)
    mission_execution_directory = LaunchConfiguration("mission_execution_directory").perform(context)
    configured_mission_file = LaunchConfiguration("mission_file").perform(context)
    configured_mission_id = LaunchConfiguration("mission_id").perform(context)
    configured_mission_type = LaunchConfiguration("mission_type").perform(context).lower()
    use_test = LaunchConfiguration("use_test").perform(context).lower() == "true"
    test_output_directory = LaunchConfiguration("test_output_directory").perform(context)
    configured_mission_costmap_yaml = LaunchConfiguration("mission_costmap_yaml").perform(context)
    auto_start_mission = LaunchConfiguration("auto_start_mission").perform(context).lower() == "true"
    use_gaussian = LaunchConfiguration("use_gaussian").perform(context).lower() == "true"

    mission_context = _resolve_execution_context(mission_execution_directory)
    mission_type = configured_mission_type or str(mission_context.get("mission_type", "")).lower()
    builtin_local_pattern_mode = _uses_odom_only_runtime(mission_type)
    mission_id = configured_mission_id or mission_context.get("mission_id", "")
    mission_file = configured_mission_file or mission_context.get("mission_file", "")
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
    saved_costmap_yaml = mission_context.get("saved_costmap_yaml", "") or mission_costmap_yaml
    # Keep the runtime map artifact separate from the static localization reference map.
    # The mapping coordinator should update the run-folder copy, while map_pose_node should
    # correlate against the mission-folder copy so it does not localize against its own live output.
    static_reference_costmap_yaml = (
        mission_context.get("source_mission_costmap_yaml", "")
        or mission_context.get("persistent_mission_costmap_yaml", "")
        or saved_costmap_yaml
    )
    # For mapping startup seeding, prefer the exact run-specific mission artifact first.
    # Scheduled mission execution copies the selected mission costmap into the run folder
    # and that copy can carry georeference metadata even when the persistent mission-folder
    # artifact is an older local-frame map. Falling back to the persistent file first causes
    # mapping_node to reject startup seeding for map-frame missions.
    startup_saved_costmap_yaml = (
        mission_costmap_yaml
        or mission_context.get("source_mission_costmap_yaml", "")
        or mission_context.get("persistent_mission_costmap_yaml", "")
        or saved_costmap_yaml
    )
    gaussian_output_directory = mission_context.get("gaussian_output_directory", "")
    if not gaussian_output_directory and mission_run_directory:
        gaussian_output_directory = str(Path(mission_run_directory) / "gaussian")
    if not mission_file and not mission_execution_directory and not use_test:
        mission_file = default_record_map_mission_file
        mission_id = "RecordMap"
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
    if builtin_local_pattern_mode:
        common_runtime_parameters["map_frame"] = "odom"
        common_runtime_parameters["pad_live_map_to_minimum_size"] = False
        common_runtime_parameters["publish_seeded_map_to_odom"] = True
        common_runtime_parameters["seeded_map_frame"] = "map"
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
    if startup_saved_costmap_yaml:
        common_runtime_parameters["startup_saved_costmap_yaml"] = startup_saved_costmap_yaml
    if saved_costmap_yaml:
        common_runtime_parameters["saved_costmap_yaml"] = saved_costmap_yaml
    if mission_window_start:
        common_runtime_parameters["mission_window_start"] = mission_window_start
    if mission_window_end:
        common_runtime_parameters["mission_window_end"] = mission_window_end

    actions = []
    map_pose_runtime_parameters = {
        "use_sim_time": use_sim_time,
    }
    if static_reference_costmap_yaml:
        map_pose_runtime_parameters["costmap_yaml_path"] = static_reference_costmap_yaml
    if builtin_local_pattern_mode:
        map_pose_runtime_parameters["map_frame"] = "odom"
        map_pose_runtime_parameters["odom_frame"] = "odom"

    actions.append(
        Node(
            package="amr_sweeper_mapping",
            executable="map_pose_node",
            name="map_pose_node",
            namespace=namespace,
            output="screen",
            parameters=[map_pose_params_file, map_pose_runtime_parameters],
        )
    )

    if use_gaussian and not builtin_local_pattern_mode:
        actions.extend([
            Node(
                package="amr_sweeper_mapping",
                executable="gaussian_node",
                name="gaussian_node",
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
        ])

    actions.append(
        Node(
            package="amr_sweeper_mapping",
            executable="mapping_node",
            name="mapping_node",
            namespace=namespace,
            output="screen",
            parameters=[params_file, common_runtime_parameters],
        )
    )

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
                "map_pose_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("amr_sweeper_mapping"), "config", "map_pose.yaml"]
                ),
                description="Parameter file for the map_pose_node TF publisher.",
            ),
            DeclareLaunchArgument(
                "mission_execution_directory",
                default_value="",
                description="Exact scheduler-selected mission execution folder for the active RUNNING mission.",
            ),
            DeclareLaunchArgument(
                "mission_file",
                default_value="",
                description="Optional direct mission definition file for builtin direct-run profiles.",
            ),
            DeclareLaunchArgument(
                "mission_id",
                default_value="",
                description="Optional mission identifier override for direct-run builtin profiles.",
            ),
            DeclareLaunchArgument(
                "mission_type",
                default_value="",
                description="Optional mission type override used to specialize direct-run builtin profiles.",
            ),
            DeclareLaunchArgument(
                "use_gaussian",
                default_value="true",
                description="Launch gaussian_node when true.",
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
