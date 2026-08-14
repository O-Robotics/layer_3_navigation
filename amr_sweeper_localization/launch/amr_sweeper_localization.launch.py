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


def _primary_topic_or_disabled(enabled: bool, topic: str, disabled_topic: str) -> str:
    return topic if enabled else disabled_topic


def _fusioncore_no_ntrip_overrides() -> dict:
    return {
        "gnss.min_fix_type": 1,
        "gnss.base_noise_xy": 50.0,
        "gnss.base_noise_z": 75.0,
        "gnss.track_heading_enabled": False,
        # Without RTK, a bare autonomous fix (fix_type=1) is the best GNSS
        # ever gets, and base_noise_xy above is deliberately loosened to
        # reflect that. But the chi2 gate judges a fix purely against that
        # noise, so a bigger R makes the SAME threshold tolerate a
        # proportionally bigger raw error -- ordinary multipath scatter on a
        # parked or slow-moving robot can then walk the fused position
        # several meters while looking perfectly "statistically consistent".
        # gnss.max_speed (fusioncore >=0.3.5's physical-plausibility jump
        # gate) rejects a fix outright, before chi2 runs, if the offset from
        # the filter's own prediction implies a speed above this value.
        # amr_sweeper_drive_controller caps commanded speed at 1.0 m/s
        # (max_linear_velocity); 3.0 m/s leaves headroom for real motion
        # while still catching outright teleports. Margin/sigma_k/drift_k
        # are left at fusioncore's own measured defaults (5.0/5.0/3.0) --
        # drift_k in particular exists so a legitimate fix isn't rejected
        # right after a real GPS blackout, and widens with the filter's own
        # position uncertainty, so it will not reliably catch a few-meter
        # drift during normal (non-blackout) operation on its own. The
        # actual fix for that failure mode is fusioncore_core's ukf.alpha
        # default (now 1.0 as of 0.3.7): the previous 0.1 gave a -99 center
        # sigma-point weight at this filter's state dimension, which is what
        # let a modest GNSS-driven position nudge blow up into a runaway
        # heading estimate. This gate is defense-in-depth on top of that,
        # not a substitute for it.
        "gnss.max_speed": 3.0,
    }


def _load_localization_parameters() -> dict:
    package_dir = get_package_share_directory("amr_sweeper_localization")
    config_path = os.path.join(package_dir, "config", "amr_sweeper_localization.yaml")
    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if "fusioncore" in config:
        return config.get("fusioncore", {}).get("ros__parameters", {})
    return config.get("/**", {}).get("ros__parameters", {})


def _projection_config_path() -> str:
    package_dir = get_package_share_directory("amr_sweeper_localization")
    return os.path.join(package_dir, "config", "odometry_projection.yaml")


def _load_launch_defaults() -> dict[str, str]:
    package_dir = get_package_share_directory("amr_sweeper_localization")
    config_path = os.path.join(package_dir, "config", "amr_sweeper_localization.yaml")
    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}

    defaults = config.get("launch_defaults", {}).get("ros__parameters")
    if isinstance(defaults, dict):
        return {
            "use_imu": str(defaults.get("use_imu", True)).lower(),
            "use_imu2": str(defaults.get("use_imu2", False)).lower(),
            "use_encoder": str(defaults.get("use_encoder", True)).lower(),
            "use_visual_odometry": str(defaults.get("use_visual_odometry", False)).lower(),
            "use_gnss": str(defaults.get("use_gnss", True)).lower(),
        }

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
    use_simulation = LaunchConfiguration("use_simulation").perform(context).lower() == "true"
    use_imu = LaunchConfiguration("use_imu").perform(context).lower() == "true"
    use_imu2 = LaunchConfiguration("use_imu2").perform(context).lower() == "true"
    use_encoder = LaunchConfiguration("use_encoder").perform(context).lower() == "true"
    use_visual_odometry = LaunchConfiguration("use_visual_odometry").perform(context).lower() == "true"
    use_gnss = LaunchConfiguration("use_gnss").perform(context).lower() == "true"
    use_ntrip_client = LaunchConfiguration("use_ntrip_client").perform(context).lower() == "true"
    if use_simulation:
        use_imu2 = False
        use_visual_odometry = False
    parameters = _load_localization_parameters()
    gnss_input_topic = "gnss/fix" if parameters.get("gnss.use_gps_fix", False) else "gnss/navsat"

    fusion_overrides = {
        "use_sim_time": use_sim_time,
        "autostart": False,
        "base_frame": "base_link",
        "odom_frame": "odom",
        "imu.topic": _primary_topic_or_disabled(
            use_imu, parameters.get("imu.topic", "imu/data_raw"), "_imu_disabled"),
        "imu2.topic": _topic_if_enabled(use_imu2, parameters.get("imu2.topic", "")),
        "encoder.topic": _primary_topic_or_disabled(
            use_encoder, parameters.get("encoder.topic", "drive_controller/odom"), "_encoder_disabled"),
        "encoder2.topic": _topic_if_enabled(
            use_visual_odometry, parameters.get("encoder2.topic", "visual_odometry/odom")),
        "gnss.fix2_topic": _topic_if_enabled(use_gnss, parameters.get("gnss.fix2_topic", "")),
        "gnss.heading_topic": _topic_if_enabled(use_gnss, parameters.get("gnss.heading_topic", "")),
        "gnss.azimuth_topic": _topic_if_enabled(use_gnss, parameters.get("gnss.azimuth_topic", "")),
        "publish.tf": False,
    }
    if use_gnss and not use_ntrip_client and not use_simulation:
        fusion_overrides.update(_fusioncore_no_ntrip_overrides())
    projection_node = Node(
        package="amr_sweeper_localization",
        executable="odometry_projection_node",
        name="odometry_projection",
        namespace=namespace,
        output="screen",
        parameters=[
            _projection_config_path(),
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    node = LifecycleNode(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        namespace=namespace,
        output="screen",
        parameters=[
            parameters,
            fusion_overrides,
        ],
        remappings=[
            ("/gnss/fix", gnss_input_topic if use_gnss else "_gnss_disabled"),
            ("/odom/wheels", "drive_controller/odom" if use_encoder else "_encoder_disabled"),
            ("/fusion/odom", "localization/odometry_body"),
            ("/fusion/pose", "localization/pose"),
            ("/fusion/debug/gnss_status", "localization/debug/gnss_status"),
            ("/fusion/debug/filter_health", "localization/debug/filter_health"),
            ("/diagnostics", "diagnostics"),
        ],
    )

    configure = TimerAction(
        period=5.0,
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

    return [projection_node, node, configure, activate]


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
                "use_simulation",
                default_value="false",
                description="Disable real-hardware localization inputs when true",
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
            DeclareLaunchArgument(
                "use_ntrip_client",
                default_value="true",
                description="If false, relax FusionCore RTK gating and reduce GNSS trust to match autonomous fixes.",
            ),
            OpaqueFunction(function=_launch_fusioncore),
        ]
    )
