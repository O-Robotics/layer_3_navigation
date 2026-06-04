"""Launch the AMR Sweeper localization and navigation stack."""

# Copyright 2026 O-Robotics

import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_file(package_name: str, launch_file_name: str):
    return PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            'launch',
            launch_file_name,
        ]
    )


def _load_json_file(path: Path) -> dict:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _resolve_execution_context(missions_directory: str, execution_pointer_file: str, mission_execution_directory: str) -> dict:
    if mission_execution_directory:
        return _load_json_file(Path(mission_execution_directory) / "execution_context.json")

    pointer_path = Path(execution_pointer_file)
    if not pointer_path.is_absolute():
        pointer_path = Path(missions_directory) / pointer_path

    pointer = _load_json_file(pointer_path)
    context_path = pointer.get("execution_context_file", "")
    if not context_path:
        mission_run_directory = pointer.get("mission_run_directory", "")
        if mission_run_directory:
            context_path = str(Path(mission_run_directory) / "execution_context.json")
    return _load_json_file(Path(context_path)) if context_path else {}


def _build_launches(context):
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_amr_sweeper_localization = LaunchConfiguration('use_amr_sweeper_localization')
    use_amr_sweeper_visual_odometry = LaunchConfiguration('use_amr_sweeper_visual_odometry')
    use_amr_sweeper_waypoint_follower = LaunchConfiguration('use_amr_sweeper_waypoint_follower')
    use_amr_sweeper_mapping = LaunchConfiguration('use_amr_sweeper_mapping')
    missions_directory = LaunchConfiguration('missions_directory').perform(context)
    execution_pointer_file = LaunchConfiguration('execution_pointer_file').perform(context)
    mission_execution_directory = LaunchConfiguration('mission_execution_directory').perform(context)
    mission_context = _resolve_execution_context(
        missions_directory,
        execution_pointer_file,
        mission_execution_directory,
    )
    mission_costmap_yaml = mission_context.get("mission_costmap_yaml", "")

    return [
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('amr_sweeper_localization', 'fusioncore.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(use_amr_sweeper_localization),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('amr_sweeper_waypoint_follower', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'mission_costmap_yaml': mission_costmap_yaml,
            }.items(),
            condition=IfCondition(use_amr_sweeper_waypoint_follower),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('amr_sweeper_mapping', 'amr_sweeper_mapping.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'missions_directory': missions_directory,
                'execution_pointer_file': execution_pointer_file,
                'mission_execution_directory': mission_execution_directory,
            }.items(),
            condition=IfCondition(use_amr_sweeper_mapping),
        ),
    ]

<<<<<<< Updated upstream
=======
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
            'missions_directory': missions_directory,
            'execution_pointer_file': execution_pointer_file,
            'mission_execution_directory': mission_execution_directory,
            'use_test': 'true' if use_test else 'false',
            'test_output_directory': test_output_directory,
        }.items(),
    )

    if use_amr_sweeper_localization:
        localization_package_dir = get_package_share_directory("amr_sweeper_localization")
        localization_config_path = os.path.join(
            localization_package_dir, "config", "amr_sweeper_localization.yaml")
        use_sim_time_bool = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"

        fusioncore_node = LifecycleNode(
            package="fusioncore_ros",
            executable="fusioncore_node",
            name="fusioncore",
            namespace=LaunchConfiguration("namespace").perform(context),
            output="screen",
            parameters=[
                localization_config_path,
                {
                    "use_sim_time": use_sim_time_bool,
                    "base_frame": "base_footprint",
                    "odom_frame": "odom",
                    "encoder.topic": "diff_cont/odom",
                    "encoder2.topic": "visual_odometry/odom" if use_amr_sweeper_visual_odometry_bool else "",
                },
            ],
            remappings=[
                ("/gnss/fix", "gnss/navsat"),
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
        if use_amr_sweeper_waypoint_follower:
            gated_entities.append(waypoint_follower_launch)
        if use_amr_sweeper_mapping:
            gated_entities.append(mapping_launch)

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
        if use_amr_sweeper_waypoint_follower:
            actions.append(waypoint_follower_launch)
        if use_amr_sweeper_mapping:
            actions.append(mapping_launch)

    return actions

>>>>>>> Stashed changes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_sweeper'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_amr_sweeper_localization', default_value='true'),
        DeclareLaunchArgument('use_amr_sweeper_visual_odometry', default_value='true'),
        DeclareLaunchArgument('use_amr_sweeper_waypoint_follower', default_value='true'),
        DeclareLaunchArgument('use_amr_sweeper_mapping', default_value='true'),
        DeclareLaunchArgument('missions_directory', default_value='src/missions_log'),
        DeclareLaunchArgument('execution_pointer_file', default_value='active_execution.json'),
        DeclareLaunchArgument('mission_execution_directory', default_value=''),
        OpaqueFunction(function=_build_launches),
    ])
