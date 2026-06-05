"""Launch the AMR Sweeper localization and navigation stack."""

# Copyright 2026 O-Robotics

import json
import os
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

def _launch_file(package_name: str, launch_file_name: str):
    return PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            'launch',
            launch_file_name,
        ]
    )


def _topic_if_enabled(enabled: bool, topic: str) -> str:
    return topic if enabled else ""


def _load_localization_parameters() -> dict:
    package_dir = get_package_share_directory("amr_sweeper_localization")
    config_path = os.path.join(package_dir, "config", "amr_sweeper_localization.yaml")
    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    return config.get("/**", {}).get("ros__parameters", {})


def _load_localization_defaults() -> dict[str, str]:
    parameters = _load_localization_parameters()
    return {
        "use_imu": str(parameters.get("use_imu", True)).lower(),
        "use_imu2": str(parameters.get("use_imu2", False)).lower(),
        "use_encoder": str(parameters.get("use_encoder", True)).lower(),
        "use_visual_odometry": str(parameters.get("use_visual_odometry", False)).lower(),
        "use_gnss": str(parameters.get("use_gnss", True)).lower(),
    }


def _load_json_file(path: Path) -> dict:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _resolve_execution_context(mission_execution_directory: str) -> dict:
    if not mission_execution_directory:
        return {}
    return _load_json_file(Path(mission_execution_directory) / "execution_context.json")


def _build_launches(context):
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    waypoint_follower_startup_delay_sec = float(
        LaunchConfiguration('waypoint_follower_startup_delay_sec').perform(context)
    )
    use_amr_sweeper_localization = LaunchConfiguration('use_amr_sweeper_localization').perform(context).lower() == 'true'
    use_amr_sweeper_visual_odometry = LaunchConfiguration('use_amr_sweeper_visual_odometry')
    use_amr_sweeper_visual_odometry_bool = (
        use_amr_sweeper_visual_odometry.perform(context).lower() == 'true'
    )
    use_imu = LaunchConfiguration('use_imu').perform(context).lower() == 'true'
    use_imu2 = LaunchConfiguration('use_imu2').perform(context).lower() == 'true'
    use_encoder = LaunchConfiguration('use_encoder').perform(context).lower() == 'true'
    use_gnss = LaunchConfiguration('use_gnss').perform(context).lower() == 'true'
    use_amr_sweeper_waypoint_follower = LaunchConfiguration('use_amr_sweeper_waypoint_follower').perform(context).lower() == 'true'
    use_amr_sweeper_mapping = LaunchConfiguration('use_amr_sweeper_mapping').perform(context).lower() == 'true'
    mission_execution_directory = LaunchConfiguration('mission_execution_directory').perform(context)
    auto_start_mission = LaunchConfiguration('auto_start_mission').perform(context)
    use_test = LaunchConfiguration('use_test').perform(context).lower() == 'true'
    test_output_directory = LaunchConfiguration('test_output_directory').perform(context)
    mission_context = _resolve_execution_context(mission_execution_directory)
    mission_costmap_yaml = mission_context.get("mission_costmap_yaml", "")

    actions = [
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
    ]

    waypoint_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('amr_sweeper_waypoint_follower', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'mission_costmap_yaml': mission_costmap_yaml,
        }.items(),
    )
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('amr_sweeper_mapping', 'amr_sweeper_mapping.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'mission_execution_directory': mission_execution_directory,
            'auto_start_mission': auto_start_mission,
            'use_test': 'true' if use_test else 'false',
            'test_output_directory': test_output_directory,
        }.items(),
    )

    delayed_waypoint_follower_launch = TimerAction(
        period=waypoint_follower_startup_delay_sec,
        actions=[waypoint_follower_launch],
    )

    if use_amr_sweeper_localization:
        localization_package_dir = get_package_share_directory("amr_sweeper_localization")
        localization_config_path = os.path.join(
            localization_package_dir, "config", "amr_sweeper_localization.yaml")
        use_sim_time_bool = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
        localization_parameters = _load_localization_parameters()

        fusion_overrides = {
            "use_sim_time": use_sim_time_bool,
            "base_frame": "base_footprint",
            "odom_frame": "odom",
            "imu.topic": _topic_if_enabled(use_imu, localization_parameters.get("imu.topic", "imu/data_raw")),
            "imu2.topic": _topic_if_enabled(use_imu2, localization_parameters.get("imu2.topic", "")),
            "encoder.topic": _topic_if_enabled(
                use_encoder, localization_parameters.get("encoder.topic", "drive_controller/odom")),
            "encoder2.topic": _topic_if_enabled(
                use_amr_sweeper_visual_odometry_bool,
                localization_parameters.get("encoder2.topic", "visual_odometry/odom")),
            "gnss.fix2_topic": _topic_if_enabled(use_gnss, localization_parameters.get("gnss.fix2_topic", "")),
            "gnss.heading_topic": _topic_if_enabled(
                use_gnss, localization_parameters.get("gnss.heading_topic", "")),
            "gnss.azimuth_topic": _topic_if_enabled(
                use_gnss, localization_parameters.get("gnss.azimuth_topic", "")),
        }

        fusioncore_node = LifecycleNode(
            package="fusioncore_ros",
            executable="fusioncore_node",
            name="fusioncore",
            namespace=LaunchConfiguration("namespace").perform(context),
            output="screen",
            parameters=[
                localization_config_path,
                fusion_overrides,
            ],
            remappings=[
                ("/gnss/fix", "gnss/navsat" if use_gnss else "__gnss_disabled"),
                ("/fusion/odom", "odometry/fused"),
                ("/fusion/pose", "pose"),
            ],
        )

        configure_fusioncore = TimerAction(
            period=2.0,
            actions=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action is fusioncore_node,
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )

        activate_fusioncore = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=fusioncore_node,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=lambda action: action is fusioncore_node,
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
            namespace=LaunchConfiguration("namespace").perform(context),
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
            parameters=[{"use_sim_time": use_sim_time_bool}],
        )

        actions.extend([fusioncore_node, configure_fusioncore, activate_fusioncore, map_to_odom])

        gated_entities = []
        if use_amr_sweeper_mapping:
            gated_entities.append(mapping_launch)
            if use_amr_sweeper_waypoint_follower:
                gated_entities.append(delayed_waypoint_follower_launch)
        elif use_amr_sweeper_waypoint_follower:
            gated_entities.append(waypoint_follower_launch)

        if gated_entities:
            actions.append(
                RegisterEventHandler(
                    OnStateTransition(
                        target_lifecycle_node=fusioncore_node,
                        start_state="activating",
                        goal_state="active",
                        entities=gated_entities,
                    )
                )
            )
    else:
        if use_amr_sweeper_mapping:
            actions.append(mapping_launch)
            if use_amr_sweeper_waypoint_follower:
                actions.append(delayed_waypoint_follower_launch)
        elif use_amr_sweeper_waypoint_follower:
            actions.append(waypoint_follower_launch)

    return actions


def generate_launch_description():
    localization_defaults = _load_localization_defaults()
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_sweeper'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_amr_sweeper_localization', default_value='true'),
        DeclareLaunchArgument(
            'use_amr_sweeper_visual_odometry',
            default_value='false'),
        DeclareLaunchArgument('use_imu', default_value=localization_defaults['use_imu']),
        DeclareLaunchArgument('use_imu2', default_value=localization_defaults['use_imu2']),
        DeclareLaunchArgument('use_encoder', default_value=localization_defaults['use_encoder']),
        DeclareLaunchArgument('use_gnss', default_value=localization_defaults['use_gnss']),
        DeclareLaunchArgument('use_amr_sweeper_waypoint_follower', default_value='true'),
        DeclareLaunchArgument('use_amr_sweeper_mapping', default_value='true'),
        DeclareLaunchArgument('missions_directory', default_value='src/missions_log'),
        DeclareLaunchArgument('execution_pointer_file', default_value='active_execution.json'),
        DeclareLaunchArgument(
            'waypoint_follower_startup_delay_sec',
            default_value='3.0',
            description='Seconds to wait after mapping launch starts before bringing up the waypoint follower.',
        ),
        DeclareLaunchArgument('mission_execution_directory', default_value=''),
        DeclareLaunchArgument('auto_start_mission', default_value='false'),
        DeclareLaunchArgument('use_test', default_value='false'),
        DeclareLaunchArgument('test_output_directory', default_value='src/layer_3_navigation/tests'),
        OpaqueFunction(function=_build_launches),
    ])
