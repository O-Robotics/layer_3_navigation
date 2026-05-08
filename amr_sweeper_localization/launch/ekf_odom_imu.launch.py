#!/usr/bin/env python3
# Launch file for odometry + IMU EKF debugging
# This launches a single EKF node with wheel odometry and IMU sensor fusion

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launches a single EKF node for odometry + IMU debugging.
    
    Nodes:
      1) ekf_filter_node: publishes odometry/filtered and odom->base_footprint TF
    """

    declare_namespace = DeclareLaunchArgument(
        name="namespace",
        default_value="amr_sweeper",
        description="Namespace for localization nodes",
    )
    use_namespace = LaunchConfiguration("namespace")

    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use ROS time if true",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Resolve the parameter file path from the package share directory
    package_dir = get_package_share_directory("amr_sweeper_localization")
    parameters_file_path = os.path.join(package_dir, "params", "ekf_odom_imu.yaml")

    # EKF node with odometry and IMU inputs
    ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        namespace=use_namespace,
        parameters=[parameters_file_path, {"use_sim_time": use_sim_time}],
        remappings=[
            # Output
            ("odometry/filtered", "odometry/filtered"),
            # Optional services/topics used by some tooling
            ("set_pose", "/set_pose"),
        ],
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_use_sim_time,
            ekf_node,
        ]
    )
