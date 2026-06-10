# Backward-compatible alias for the programmed mission Nav2 stack.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory("amr_sweeper_navigation")
    launch_dir = os.path.join(bringup_dir, "launch")
    console_output_format = "[{severity}] [{time}] [{name}] : {message}"

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_CONSOLE_OUTPUT_FORMAT", console_output_format),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "programmed_missions_navigation.launch.py")
                )
            ),
        ]),
    ])
