"""Launch the AMR Sweeper localization and navigation stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_file(package_name: str, launch_file_name: str):
    return PathJoinSubstitution([
        FindPackageShare(package_name),
        'launch',
        launch_file_name,
    ])


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_localization = LaunchConfiguration('use_localization')
    use_navigation = LaunchConfiguration('use_navigation')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_sweeper'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_localization', default_value='true'),
        DeclareLaunchArgument('use_navigation', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file('amr_sweeper_localization', 'fusioncore.launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(use_localization),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file('amr_sweeper_waypoint_follower', 'bringup_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(use_navigation),
        ),
    ])
