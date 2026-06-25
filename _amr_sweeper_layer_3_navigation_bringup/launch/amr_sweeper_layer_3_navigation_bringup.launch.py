"""Launch the AMR Sweeper localization and navigation stack."""

# Copyright 2026 O-Robotics

import json
import os
from pathlib import Path
import time
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
import rclpy
from rclpy.node import Node as RclpyNode
from std_msgs.msg import String

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


def _primary_topic_or_disabled(enabled: bool, topic: str, disabled_topic: str) -> str:
    return topic if enabled else disabled_topic


def _normalize_namespace(namespace: str) -> str:
    return namespace.strip().strip('/')


def _qualify_name(namespace: str, relative_name: str) -> str:
    cleaned_namespace = _normalize_namespace(namespace)
    cleaned_name = relative_name.strip().strip('/')
    if not cleaned_namespace:
        return f'/{cleaned_name}'
    return f'/{cleaned_namespace}/{cleaned_name}'


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


def _load_localization_defaults() -> dict[str, str]:
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


def _load_json_file(path: Path) -> dict:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _resolve_execution_context_path(mission_execution_directory: str) -> Path | None:
    if not mission_execution_directory:
        return None
    execution_directory = Path(mission_execution_directory)
    stamped_candidates = sorted(execution_directory.glob("*_context.json"))
    if stamped_candidates:
        return stamped_candidates[0]
    legacy_path = execution_directory / "execution_context.json"
    if legacy_path.exists():
        return legacy_path
    return None


def _resolve_execution_context(mission_execution_directory: str) -> dict:
    context_path = _resolve_execution_context_path(mission_execution_directory)
    if context_path is None:
        return {}
    return _load_json_file(context_path)


def _uses_odom_only_runtime(mission_type: str, execution_mode: str) -> bool:
    mission_type = str(mission_type).lower()
    execution_mode = str(execution_mode).lower()
    return (
        mission_type in {"builtin_local_pattern", "vda5050_scheduled_mission_local"} or
        execution_mode == "teleoperation"
    )


def _resolve_navigation_launch_filename(mission_type: str, execution_mode: str) -> str:
    mission_type = str(mission_type).lower()
    execution_mode = str(execution_mode).lower()

    if mission_type in {"builtin_local_pattern", "vda5050_scheduled_mission_local"}:
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


def _mission_requires_mapping(
    mission_type: str,
    execution_mode: str,
    mission_context: dict,
    mapping_requested: bool,
) -> bool:
    del mission_context

    if mapping_requested:
        return True
    if str(execution_mode).lower() == "manual_mapping":
        return True
    if _uses_odom_only_runtime(mission_type, execution_mode):
        return False

    # Programmed scheduled missions navigate in the map frame, and profile 201
    # expects the mapping package to own map construction and map -> odom.
    # Ignore stale mission overrides
    # that would disable mapping for those runs.
    if str(mission_type).lower() in {"vda5050_scheduled_mission", "scheduled"}:
        return True

    return False


def _build_launches(context):
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    legacy_navigation_startup_delay_sec = float(
        LaunchConfiguration('waypoint_follower_startup_delay_sec').perform(context)
    )
    mapping_startup_delay_sec = float(
        LaunchConfiguration('mapping_startup_delay_sec').perform(context)
    )
    navigation_startup_delay_sec = float(
        LaunchConfiguration('navigation_startup_delay_sec').perform(context)
    )
    if navigation_startup_delay_sec == 3.0 and legacy_navigation_startup_delay_sec != 3.0:
        navigation_startup_delay_sec = legacy_navigation_startup_delay_sec
    if navigation_startup_delay_sec == 3.0:
        navigation_startup_delay_sec = 5.0
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
    use_gaussian = LaunchConfiguration('use_gaussian').perform(context)
    use_test = LaunchConfiguration('use_test').perform(context).lower() == 'true'
    test_output_directory = LaunchConfiguration('test_output_directory').perform(context)
    run_layer_3_system_check = (
        LaunchConfiguration('run_layer_3_system_check').perform(context).lower() == 'true'
    )
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
    effective_mission_type = mission_type or mission_context.get("mission_type", "")
    effective_execution_mode = mission_context.get("execution_mode", "")
    effective_mission_file = mission_file or mission_context.get("mission_file", "")
    effective_mission_id = mission_id or mission_context.get("mission_id", "")
    effective_mission_costmap_yaml = mission_costmap_yaml or mission_context.get("mission_costmap_yaml", "")
    navigation_launch_filename = _resolve_navigation_launch_filename(
        effective_mission_type,
        effective_execution_mode,
    )
    effective_use_amr_sweeper_mapping = _mission_requires_mapping(
        effective_mission_type,
        effective_execution_mode,
        mission_context,
        use_amr_sweeper_mapping,
    )

    if (
        not use_amr_sweeper_mapping and
        effective_use_amr_sweeper_mapping and
        not _uses_odom_only_runtime(effective_mission_type, effective_execution_mode)
    ):
        print(
            "Layer 3 bringup: forcing mapping on because this mission navigates in 'map' and "
            "depends on amr_sweeper_mapping publishing map -> odom."
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
    layer_3_system_check = Node(
        package='amr_sweeper_layer_3_navigation_bringup',
        executable='layer_3_system_check.py',
        name='layer_3_system_check',
        namespace=LaunchConfiguration('namespace').perform(context),
        output='screen',
        parameters=[{
            'global_frame': 'map' if navigation_launch_filename == 'programmed_missions_navigation.launch.py' else 'odom',
            'robot_frame': 'base_footprint',
            'action_name': 'follow_waypoints',
            'passed_topic': 'layer_3/system_check_passed',
        }],
        condition=IfCondition(LaunchConfiguration('run_layer_3_system_check')),
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
            'use_gaussian': use_gaussian,
            'auto_start_mission': auto_start_mission,
            'use_test': 'true' if use_test else 'false',
            'test_output_directory': test_output_directory,
        }.items(),
    )

    delayed_mapping_launch = TimerAction(
        period=mapping_startup_delay_sec,
        actions=[mapping_launch],
    )

    delayed_navigation_launch = TimerAction(
        period=navigation_startup_delay_sec,
        actions=[navigation_launch],
    )

    if use_amr_sweeper_localization:
        use_sim_time_bool = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
        localization_parameters = _load_localization_parameters()
        gnss_input_topic = "gnss/fix" if localization_parameters.get("gnss.use_gps_fix", False) else "gnss/navsat"

        fusion_overrides = {
            "use_sim_time": use_sim_time_bool,
            "base_frame": "base_link",
            "odom_frame": "odom",
            "imu.topic": _primary_topic_or_disabled(
                use_imu, localization_parameters.get("imu.topic", "imu/data_raw"), "_imu_disabled"),
            "imu2.topic": _topic_if_enabled(use_imu2, localization_parameters.get("imu2.topic", "")),
            "encoder.topic": _primary_topic_or_disabled(
                use_encoder, localization_parameters.get("encoder.topic", "drive_controller/odom"),
                "_encoder_disabled"),
            "encoder2.topic": _topic_if_enabled(
                use_amr_sweeper_visual_odometry_bool,
                localization_parameters.get("encoder2.topic", "visual_odometry/odom")),
            "gnss.fix2_topic": _topic_if_enabled(use_gnss, localization_parameters.get("gnss.fix2_topic", "")),
            "gnss.heading_topic": _topic_if_enabled(
                use_gnss, localization_parameters.get("gnss.heading_topic", "")),
            "gnss.azimuth_topic": _topic_if_enabled(
                use_gnss, localization_parameters.get("gnss.azimuth_topic", "")),
            "publish.tf": False,
        }

        odometry_projection_node = Node(
            package="amr_sweeper_localization",
            executable="odometry_projection_node",
            name="odometry_projection",
            namespace=LaunchConfiguration("namespace").perform(context),
            output="screen",
            parameters=[
                _projection_config_path(),
                {
                    "use_sim_time": use_sim_time_bool,
                },
            ],
        )

        fusioncore_node = LifecycleNode(
            package="fusioncore_ros",
            executable="fusioncore_node",
            name="fusioncore",
            namespace=LaunchConfiguration("namespace").perform(context),
            output="screen",
            parameters=[
                localization_parameters,
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

        actions.extend([odometry_projection_node, fusioncore_node, configure_fusioncore, activate_fusioncore])

        gated_entities = []
        if effective_use_amr_sweeper_mapping:
            gated_entities.append(delayed_mapping_launch)
            if use_amr_sweeper_navigation:
                gated_entities.append(delayed_navigation_launch)
        elif use_amr_sweeper_navigation:
            gated_entities.append(navigation_launch)
        if use_amr_sweeper_navigation and run_layer_3_system_check:
            gated_entities.append(layer_3_system_check)

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
            actions.append(delayed_mapping_launch)
            if use_amr_sweeper_navigation:
                actions.append(delayed_navigation_launch)
        elif use_amr_sweeper_navigation:
            actions.append(navigation_launch)
        if use_amr_sweeper_navigation and run_layer_3_system_check:
            actions.append(layer_3_system_check)

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
            description='Deprecated compatibility alias for navigation_startup_delay_sec.',
        ),
        DeclareLaunchArgument(
            'mapping_startup_delay_sec',
            default_value='0.0',
            description='Seconds to wait after localization becomes active before starting mapping.',
        ),
        DeclareLaunchArgument(
            'navigation_startup_delay_sec',
            default_value='3.0',
            description='Seconds to wait after localization becomes active before starting navigation.',
        ),
        DeclareLaunchArgument('mission_execution_directory', default_value=''),
        DeclareLaunchArgument('mission_file', default_value=''),
        DeclareLaunchArgument('mission_id', default_value=''),
        DeclareLaunchArgument('mission_type', default_value=''),
        DeclareLaunchArgument('mission_costmap_yaml', default_value=''),
        DeclareLaunchArgument('use_gaussian', default_value='true'),
        DeclareLaunchArgument('auto_start_mission', default_value='false'),
        DeclareLaunchArgument('use_test', default_value='false'),
        DeclareLaunchArgument('test_output_directory', default_value='src/layer_3_navigation/tests'),
        DeclareLaunchArgument('run_layer_3_system_check', default_value='false'),
        OpaqueFunction(function=_build_launches),
    ])
