#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("amr_sweeper_visual_odometry")
    params_file = PathJoinSubstitution(
        [package_share, "config", "amr_sweeper_visual_odometry.yaml"]
    )

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="amr_sweeper",
                description="Namespace for visual odometry nodes",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use ROS time if true",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Log level for launched nodes",
            ),
            DeclareLaunchArgument(
                "visual_odometry_params_file",
                default_value=params_file,
                description="Parameter file for the visual odometry node",
            ),
            Node(
                package="amr_sweeper_visual_odometry",
                executable="visual_odometry_node",
                name="visual_odometry_node",
                namespace=namespace,
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    LaunchConfiguration("visual_odometry_params_file"),
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
        ]
    )
