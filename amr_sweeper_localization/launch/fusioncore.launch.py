#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _launch_fusioncore(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    package_dir = get_package_share_directory("amr_sweeper_localization")
    config_path = os.path.join(package_dir, "params", "fusioncore.yaml")

    node = LifecycleNode(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        namespace=namespace,
        output="screen",
        parameters=[
            config_path,
            {
                "use_sim_time": use_sim_time,
                "base_frame": "base_footprint",
                "odom_frame": "odom",
            },
        ],
        remappings=[
            ("/imu/data", "imu/data_raw"),
            ("/odom/wheels", "diff_cont/odom"),
            ("/gnss/fix", "navsat"),
            ("/fusion/odom", "odometry/fused"),
            ("/fusion/pose", "pose"),
        ],
    )

    configure = TimerAction(
        period=2.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=lambda action: action is node,
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ],
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action is node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf",
        namespace=namespace,
        output="screen",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return [node, configure, activate, map_to_odom]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="amr_sweeper",
                description="Namespace for localization nodes",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use ROS time if true",
            ),
            OpaqueFunction(function=_launch_fusioncore),
        ]
    )
