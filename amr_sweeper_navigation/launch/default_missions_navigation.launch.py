import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("amr_sweeper_navigation")
    launch_dir = os.path.join(bringup_dir, "launch")
    console_output_format = "[{severity}] [{time}] [{name}] : {message}"

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    mission_costmap_yaml = LaunchConfiguration("mission_costmap_yaml")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_CONSOLE_OUTPUT_FORMAT", console_output_format),
        DeclareLaunchArgument("namespace", default_value="amr_sweeper", description="Top-level namespace"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use ROS time if true"),
        DeclareLaunchArgument("autostart", default_value="true", description="Automatically startup the nav2 stack"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "config", "default_missions_navigation.yaml"),
            description="Full path to the Nav2 parameter file",
        ),
        DeclareLaunchArgument(
            "mission_costmap_yaml",
            default_value="",
            description="Exact mission run costmap YAML from execution_context.json. Leave empty for builtin local missions.",
        ),
        DeclareLaunchArgument("use_respawn", default_value="false", description="Whether to respawn if a node crashes."),
        DeclareLaunchArgument("log_level", default_value="info", description="log level"),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "navigation_launch.py")),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "mission_costmap_yaml": mission_costmap_yaml,
                    "autostart": autostart,
                    "use_respawn": use_respawn,
                    "log_level": log_level,
                }.items(),
            ),
        ]),
    ])
