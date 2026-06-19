#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _topic_if_enabled(enabled: bool, topic: str) -> str:
    return topic if enabled else ""


def _load_localization_parameters() -> dict:
    package_dir = get_package_share_directory("amr_sweeper_localization")
    config_path = os.path.join(package_dir, "config", "amr_sweeper_localization.yaml")
    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    return config.get("/**", {}).get("ros__parameters", {})


def _load_launch_defaults() -> dict[str, str]:
    parameters = _load_localization_parameters()
    return {
        "use_imu": str(parameters.get("use_imu", True)).lower(),
        "use_imu2": str(parameters.get("use_imu2", False)).lower(),
        "use_encoder": str(parameters.get("use_encoder", True)).lower(),
        "use_visual_odometry": str(parameters.get("use_visual_odometry", False)).lower(),
        "use_gnss": str(parameters.get("use_gnss", True)).lower(),
    }


def _launch_fusioncore(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    use_imu = LaunchConfiguration("use_imu").perform(context).lower() == "true"
    use_imu2 = LaunchConfiguration("use_imu2").perform(context).lower() == "true"
    use_encoder = LaunchConfiguration("use_encoder").perform(context).lower() == "true"
    use_visual_odometry = LaunchConfiguration("use_visual_odometry").perform(context).lower() == "true"
    use_gnss = LaunchConfiguration("use_gnss").perform(context).lower() == "true"
    package_dir = get_package_share_directory("amr_sweeper_localization")
    config_path = os.path.join(package_dir, "config", "amr_sweeper_localization.yaml")
    parameters = _load_localization_parameters()

    fusion_overrides = {
        "use_sim_time": use_sim_time,
        "base_frame": "base_footprint",
        "odom_frame": "odom",
        "imu.topic": _topic_if_enabled(use_imu, parameters.get("imu.topic", "imu/data_raw")),
        "imu2.topic": _topic_if_enabled(use_imu2, parameters.get("imu2.topic", "")),
        "encoder.topic": _topic_if_enabled(use_encoder, parameters.get("encoder.topic", "drive_controller/odom")),
        "encoder2.topic": _topic_if_enabled(
            use_visual_odometry, parameters.get("encoder2.topic", "visual_odometry/odom")),
        "gnss.fix2_topic": _topic_if_enabled(use_gnss, parameters.get("gnss.fix2_topic", "")),
        "gnss.heading_topic": _topic_if_enabled(use_gnss, parameters.get("gnss.heading_topic", "")),
    }

    node = LifecycleNode(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        namespace=namespace,
        output="screen",
        parameters=[
            config_path,
            fusion_overrides,
        ],
        remappings=[
            ("/gnss/fix", "gnss/navsat" if use_gnss else "_gnss_disabled"),
            ("/fusion/odom", "localization/odometry_fused"),
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

    return [node, configure, activate]


def generate_launch_description():
    defaults = _load_launch_defaults()
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
            DeclareLaunchArgument(
                "use_visual_odometry",
                default_value=defaults["use_visual_odometry"],
                description="Enable the visual-odometry secondary encoder input",
            ),
            DeclareLaunchArgument(
                "use_imu",
                default_value=defaults["use_imu"],
                description="Enable the primary IMU input",
            ),
            DeclareLaunchArgument(
                "use_imu2",
                default_value=defaults["use_imu2"],
                description="Enable the optional secondary IMU input",
            ),
            DeclareLaunchArgument(
                "use_encoder",
                default_value=defaults["use_encoder"],
                description="Enable the primary wheel-odometry encoder input",
            ),
            DeclareLaunchArgument(
                "use_gnss",
                default_value=defaults["use_gnss"],
                description="Enable the primary GNSS input",
            ),
            OpaqueFunction(function=_launch_fusioncore),
        ]
    )
