"""Launch the AMR Sweeper localization and navigation stack."""

# Copyright 2026 O-Robotics

import json
import os
import re
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, TimerAction
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


def _resolve_navigation_launch_filename(mission_type: str, execution_mode: str) -> str:
    mission_type = str(mission_type).lower()
    execution_mode = str(execution_mode).lower()

    if mission_type == "builtin_local_pattern":
        return "default_missions_navigation.launch.py"
    if (
        mission_type in {"builtin_manual_mapping", "builtin_teleop"} or
        execution_mode in {"manual_mapping", "teleoperation"}
    ):
        return "manual_missions_navigation.launch.py"
    return "programmed_missions_navigation.launch.py"


def _bool_override(context: dict, key: str, default_value: bool) -> bool:
    overrides = context.get("layer_overrides", {})
    if not isinstance(overrides, dict) or key not in overrides:
        return default_value
    return bool(overrides.get(key))


def _bool_context_value(context: dict, key: str, default_value: bool) -> bool:
    if key not in context:
        return default_value
    return bool(context.get(key))


def _collected_artifacts_directory(mission_context: dict, mission_execution_directory: str) -> str:
    collected_directory = str(mission_context.get("collected_artifacts_directory", "")).strip()
    if collected_directory:
        return collected_directory
    if mission_execution_directory:
        return str(Path(mission_execution_directory) / "artifacts")
    return ""


def _record_rosbag_topics_path() -> Path:
    return (
        Path(get_package_share_directory("amr_sweeper_layer_3_navigation_bringup")) /
        "config" /
        "record_rosbag.yaml"
    )


def _load_record_rosbag_topics() -> list[str]:
    path = _record_rosbag_topics_path()
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        return []
    topics = document.get("topics", [])
    if not isinstance(topics, list):
        return []
    normalized_topics: list[str] = []
    for topic in topics:
        if not isinstance(topic, str):
            continue
        value = topic.strip()
        if not value:
            continue
        normalized_topics.append(value)
    return normalized_topics


def _record_rosbag_topic_regex(topics: list[str]) -> str:
    if not topics:
        return ""
    escaped_topics = [re.escape(topic) for topic in topics]
    return "^(" + "|".join(escaped_topics) + ")$"


def _mission_requires_mapping(mission_context: dict, mapping_requested: bool) -> bool:
    if not mapping_requested:
        return False
    return True


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
    use_amr_sweeper_navigation = LaunchConfiguration('use_amr_sweeper_navigation').perform(context).lower() == 'true'
    use_amr_sweeper_mapping = LaunchConfiguration('use_amr_sweeper_mapping').perform(context).lower() == 'true'
    mission_execution_directory = LaunchConfiguration('mission_execution_directory').perform(context)
    mission_file = LaunchConfiguration('mission_file').perform(context)
    mission_id = LaunchConfiguration('mission_id').perform(context)
    mission_type = LaunchConfiguration('mission_type').perform(context)
    mission_costmap_yaml = LaunchConfiguration('mission_costmap_yaml').perform(context)
    auto_start_mission = LaunchConfiguration('auto_start_mission').perform(context)
    record_rosbag = LaunchConfiguration('record_rosbag').perform(context).lower() == 'true'
    use_test = LaunchConfiguration('use_test').perform(context).lower() == 'true'
    test_output_directory = LaunchConfiguration('test_output_directory').perform(context)
    mission_context = _resolve_execution_context(mission_execution_directory)
    use_amr_sweeper_localization = _bool_override(
        mission_context,
        'use_amr_sweeper_localization',
        use_amr_sweeper_localization,
    )
    use_amr_sweeper_visual_odometry_bool = _bool_override(
        mission_context,
        'use_amr_sweeper_visual_odometry',
        use_amr_sweeper_visual_odometry_bool,
    )
    use_amr_sweeper_navigation = _bool_override(
        mission_context,
        'use_amr_sweeper_navigation',
        use_amr_sweeper_navigation,
    )
    use_amr_sweeper_mapping = _bool_override(
        mission_context,
        'use_amr_sweeper_mapping',
        use_amr_sweeper_mapping,
    )
    record_rosbag = _bool_context_value(
        mission_context,
        'record_rosbag',
        record_rosbag,
    )
    effective_mission_type = mission_type or mission_context.get("mission_type", "")
    effective_execution_mode = mission_context.get("execution_mode", "")
    effective_mission_file = mission_file or mission_context.get("mission_file", "")
    effective_mission_id = mission_id or mission_context.get("mission_id", "")
    effective_mission_costmap_yaml = mission_costmap_yaml or mission_context.get("mission_costmap_yaml", "")
    effective_collected_artifacts_directory = _collected_artifacts_directory(
        mission_context,
        mission_execution_directory,
    )
    navigation_launch_filename = _resolve_navigation_launch_filename(
        effective_mission_type,
        effective_execution_mode,
    )
    effective_use_amr_sweeper_mapping = _mission_requires_mapping(
        mission_context,
        use_amr_sweeper_mapping,
    )

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
    if record_rosbag and effective_collected_artifacts_directory:
        rosbag_output_directory = str(Path(effective_collected_artifacts_directory) / 'rosbag')
        rosbag_topics = _load_record_rosbag_topics()
        rosbag_topic_regex = _record_rosbag_topic_regex(rosbag_topics)
        os.makedirs(effective_collected_artifacts_directory, exist_ok=True)
        if rosbag_topic_regex:
            actions.append(
                ExecuteProcess(
                    cmd=[
                        'ros2',
                        'bag',
                        'record',
                        '--regex',
                        rosbag_topic_regex,
                        '-o',
                        rosbag_output_directory,
                    ],
                    output='screen',
                )
            )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('amr_sweeper_navigation', navigation_launch_filename)
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'mission_costmap_yaml': effective_mission_costmap_yaml,
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
            'mission_file': effective_mission_file,
            'mission_id': effective_mission_id,
            'mission_type': effective_mission_type,
            'mission_costmap_yaml': effective_mission_costmap_yaml,
            'auto_start_mission': auto_start_mission,
            'use_test': 'true' if use_test else 'false',
            'test_output_directory': test_output_directory,
        }.items(),
    )

    delayed_navigation_launch = TimerAction(
        period=waypoint_follower_startup_delay_sec,
        actions=[navigation_launch],
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
                ("/gnss/fix", "gnss/navsat" if use_gnss else "_gnss_disabled"),
                ("/fusion/odom", "localization/odometry_fused"),
                ("/fusion/pose", "localization/pose"),
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

        activate_fusioncore = TimerAction(
            period=4.0,
            actions=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action is fusioncore_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )

        actions.extend([fusioncore_node, configure_fusioncore, activate_fusioncore])

        gated_entities = []
        if effective_use_amr_sweeper_mapping:
            gated_entities.append(mapping_launch)
            if use_amr_sweeper_navigation:
                gated_entities.append(delayed_navigation_launch)
        elif use_amr_sweeper_navigation:
            gated_entities.append(navigation_launch)

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
        if effective_use_amr_sweeper_mapping:
            actions.append(mapping_launch)
            if use_amr_sweeper_navigation:
                actions.append(delayed_navigation_launch)
        elif use_amr_sweeper_navigation:
            actions.append(navigation_launch)

    return actions


def generate_launch_description():
    localization_defaults = _load_localization_defaults()
    console_output_format = "[{severity}] [{time}] [{name}] : {message}"
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', console_output_format),
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
        DeclareLaunchArgument('use_amr_sweeper_navigation', default_value='true'),
        DeclareLaunchArgument('use_amr_sweeper_mapping', default_value='true'),
        DeclareLaunchArgument('missions_directory', default_value='missions/logs'),
        DeclareLaunchArgument('execution_pointer_file', default_value='active_execution.json'),
        DeclareLaunchArgument(
            'waypoint_follower_startup_delay_sec',
            default_value='3.0',
            description='Seconds to wait after mapping launch starts before bringing up the navigation stack.',
        ),
        DeclareLaunchArgument('mission_execution_directory', default_value=''),
        DeclareLaunchArgument('mission_file', default_value=''),
        DeclareLaunchArgument('mission_id', default_value=''),
        DeclareLaunchArgument('mission_type', default_value=''),
        DeclareLaunchArgument('mission_costmap_yaml', default_value=''),
        DeclareLaunchArgument('auto_start_mission', default_value='false'),
        DeclareLaunchArgument('record_rosbag', default_value='false'),
        DeclareLaunchArgument('use_test', default_value='false'),
        DeclareLaunchArgument('test_output_directory', default_value='src/layer_3_navigation/tests'),
        OpaqueFunction(function=_build_launches),
    ])
