"""Launch the AMR Sweeper localization and navigation stack."""

# Copyright 2026 O-Robotics

import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_file(package_name: str, launch_file_name: str):
    return PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            'launch',
            launch_file_name,
        ]
    )


def _load_json_file(path: Path) -> dict:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _resolve_execution_context(missions_directory: str, execution_pointer_file: str, mission_execution_directory: str) -> dict:
    if mission_execution_directory:
        return _load_json_file(Path(mission_execution_directory) / "execution_context.json")

    pointer_path = Path(execution_pointer_file)
    if not pointer_path.is_absolute():
        pointer_path = Path(missions_directory) / pointer_path

    pointer = _load_json_file(pointer_path)
    context_path = pointer.get("execution_context_file", "")
    if not context_path:
        mission_run_directory = pointer.get("mission_run_directory", "")
        if mission_run_directory:
            context_path = str(Path(mission_run_directory) / "execution_context.json")
    return _load_json_file(Path(context_path)) if context_path else {}


def _build_launches(context):
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_amr_sweeper_localization = LaunchConfiguration('use_amr_sweeper_localization')
    use_amr_sweeper_visual_odometry = LaunchConfiguration('use_amr_sweeper_visual_odometry')
    use_amr_sweeper_waypoint_follower = LaunchConfiguration('use_amr_sweeper_waypoint_follower')
    use_amr_sweeper_mapping = LaunchConfiguration('use_amr_sweeper_mapping')
    missions_directory = LaunchConfiguration('missions_directory').perform(context)
    execution_pointer_file = LaunchConfiguration('execution_pointer_file').perform(context)
    mission_execution_directory = LaunchConfiguration('mission_execution_directory').perform(context)
    mission_context = _resolve_execution_context(
        missions_directory,
        execution_pointer_file,
        mission_execution_directory,
    )
    mission_costmap_yaml = mission_context.get("mission_costmap_yaml", "")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('amr_sweeper_visual_odometry', 'amr_sweeper_visual_odometry.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(use_amr_sweeper_visual_odometry),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('amr_sweeper_localization', 'fusioncore.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(use_amr_sweeper_localization),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('amr_sweeper_waypoint_follower', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'mission_costmap_yaml': mission_costmap_yaml,
            }.items(),
            condition=IfCondition(use_amr_sweeper_waypoint_follower),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('amr_sweeper_mapping', 'amr_sweeper_mapping.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'missions_directory': missions_directory,
                'execution_pointer_file': execution_pointer_file,
                'mission_execution_directory': mission_execution_directory,
            }.items(),
            condition=IfCondition(use_amr_sweeper_mapping),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_sweeper'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_amr_sweeper_localization', default_value='true'),
        DeclareLaunchArgument('use_amr_sweeper_visual_odometry', default_value='true'),
        DeclareLaunchArgument('use_amr_sweeper_waypoint_follower', default_value='true'),
        DeclareLaunchArgument('use_amr_sweeper_mapping', default_value='true'),
        DeclareLaunchArgument('missions_directory', default_value='src/missions_log'),
        DeclareLaunchArgument('execution_pointer_file', default_value='active_execution.json'),
        DeclareLaunchArgument('mission_execution_directory', default_value=''),
        OpaqueFunction(function=_build_launches),
    ])
