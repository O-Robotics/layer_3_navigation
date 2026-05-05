#!/usr/bin/env python3
# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launches a dual-EKF + navsat_transform setup under a configurable namespace.

    Nodes:
      1) ekf_filter_node_odom (local EKF): publishes odometry/local and odom->base_link TF
      2) navsat_transform_node: publishes odometry/gps and gps/filtered
      3) ekf_filter_node_map (global EKF): publishes odometry/global and map->odom TF (if enabled in YAML)
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

    # Resolve the parameter file path (dual_ekf_navsat.yaml) from the package share directory.
    package_dir = get_package_share_directory("amr_sweeper_localization")
    parameters_file_path = os.path.join(package_dir, "params", "dual_ekf_navsat.yaml")

    # Local EKF (odom world frame): produces odometry/local and (optionally) odom->base_link TF.
    local_ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        namespace=use_namespace,
        parameters=[parameters_file_path, {"use_sim_time": use_sim_time}],
        remappings=[
            # Output
            ("odometry/filtered", "odometry/local"),
            # Optional services/topics used by some tooling
            ("set_pose", "/set_pose"),
            # If your stack publishes cmd_vel for debugging/recording, keep this consistent
            ("cmd_vel", "/amr_sweeper/diff_cont/cmd_vel_unstamped"),
        ],
    )

    def launch_navsat_and_global_ekf(context, *args, **kwargs):
        """
        Launch navsat_transform and the global EKF after the local EKF.
        This is not a hard synchronization on TF/data availability, but it avoids startup races.
        """

        # navsat_transform_node: converts GPS Fix + heading source into an odometry/gps message
        # that can be fused by the global EKF. Also publishes gps/filtered if enabled in YAML.
        navsat_node = launch_ros.actions.Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            namespace=use_namespace,
            parameters=[parameters_file_path, {"use_sim_time": use_sim_time}],
            remappings=[
                # IMPORTANT: Match these to your actual topics.
                # Your IMU topic is /amr_sweeper/imu/data_raw (Imu)
                ("imu", "/amr_sweeper/imu/data_raw"),
                # Your GNSS topic is /amr_sweeper/navsat (NavSatFix)
                ("gps/fix", "/amr_sweeper/navsat"),

                # Outputs under the namespace
                ("gps/filtered", "gps/filtered"),
                ("odometry/gps", "odometry/gps"),

                # Reference odometry for navsat_transform (should be the global EKF output)
                ("odometry/filtered", "odometry/global"),
            ],
        )

        # Global EKF (map world frame): fuses odometry/gps + other sources to produce odometry/global.
        global_ekf_node = launch_ros.actions.Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            namespace=use_namespace,
            parameters=[parameters_file_path, {"use_sim_time": use_sim_time}],
            remappings=[
                # Output
                ("odometry/filtered", "odometry/global"),
                # Optional services/topics
                ("set_pose", "/set_pose"),
            ],
        )

        # Start navsat first so odometry/gps exists before the global EKF tries to subscribe.
        return [navsat_node, global_ekf_node]

    delayed_navsat_and_global = OpaqueFunction(function=launch_navsat_and_global_ekf)

    return LaunchDescription(
        [
            declare_namespace,
            declare_use_sim_time,
            local_ekf_node,
            delayed_navsat_and_global,
        ]
    )
