"""Launch the AMR Sweeper localization and navigation stack."""

# Copyright 2026 O-Robotics

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_amr_sweeper_localization = LaunchConfiguration('use_amr_sweeper_localization')
    use_amr_sweeper_visual_odometry = LaunchConfiguration('use_amr_sweeper_visual_odometry')
    use_amr_sweeper_waypoint_follower = LaunchConfiguration('use_amr_sweeper_waypoint_follower')
    use_amr_sweeper_mapping = LaunchConfiguration('use_amr_sweeper_mapping')
    execution_pointer_file = LaunchConfiguration('execution_pointer_file')
    mission_execution_directory = LaunchConfiguration('mission_execution_directory')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_sweeper'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_amr_sweeper_localization', default_value='true'),
        DeclareLaunchArgument('use_amr_sweeper_visual_odometry', default_value='true'),
        DeclareLaunchArgument('use_amr_sweeper_waypoint_follower', default_value='true'),
        DeclareLaunchArgument('use_amr_sweeper_mapping', default_value='true'),
        DeclareLaunchArgument('execution_pointer_file', default_value='active_execution.json'),
        DeclareLaunchArgument('mission_execution_directory', default_value=''),
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
                'execution_pointer_file': execution_pointer_file,
                'mission_execution_directory': mission_execution_directory,
            }.items(),
            condition=IfCondition(use_amr_sweeper_mapping),
        ),
    ])
