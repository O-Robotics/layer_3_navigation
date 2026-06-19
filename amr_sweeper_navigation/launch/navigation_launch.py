# Copyright (c) 2018 Intel Corporation
#
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

import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _normalize_namespace(namespace: str) -> str:
    return namespace.strip().strip('/')


def _qualify_topic(namespace: str, relative_topic: str) -> str:
    cleaned_namespace = _normalize_namespace(namespace)
    cleaned_topic = relative_topic.strip().strip('/')
    if not cleaned_namespace:
        return f'/{cleaned_topic}'
    return f'/{cleaned_namespace}/{cleaned_topic}'


def _rewrite_nav2_params(context) -> dict:
    rewritten_params_path = _materialize_nav2_params(context, include_root_key=False)
    params_data = yaml.safe_load(Path(rewritten_params_path).read_text()) or {}
    params_data["__rewritten_params_path__"] = rewritten_params_path
    params_data["__source_params_file__"] = LaunchConfiguration('params_file').perform(context)
    return params_data


def _load_mission_costmap_resolution(context):
    mission_costmap_yaml = LaunchConfiguration('mission_costmap_yaml').perform(context).strip()
    if not mission_costmap_yaml:
        return None

    mission_costmap_path = Path(mission_costmap_yaml)
    if not mission_costmap_path.is_file():
        return None

    mission_costmap_data = yaml.safe_load(mission_costmap_path.read_text()) or {}
    resolution = mission_costmap_data.get('resolution')
    if resolution is None:
        return None
    return float(resolution)


def _materialize_nav2_params(context, *, include_root_key: bool) -> str:
    namespace_value = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    mission_costmap_yaml = LaunchConfiguration('mission_costmap_yaml')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'costmap_yaml_path': mission_costmap_yaml,
    }

    rewritten_params_path = RewrittenYaml(
        source_file=params_file,
        root_key=namespace_value if include_root_key else None,
        param_rewrites=param_substitutions,
        convert_types=True,
    ).perform(context)

    params_data = yaml.safe_load(Path(rewritten_params_path).read_text()) or {}

    mission_costmap_resolution = _load_mission_costmap_resolution(context)
    if mission_costmap_resolution is not None:
        nav2_root = params_data.get(namespace_value, params_data) if include_root_key else params_data
        global_costmap_params = (
            nav2_root
            .get('global_costmap', {})
            .get('global_costmap', {})
            .get('ros__parameters', {})
        )
        if global_costmap_params:
            global_costmap_params['resolution'] = mission_costmap_resolution

    temp_file = Path(tempfile.gettempdir()) / (
        f"amr_sweeper_nav2_params_{'namespaced' if include_root_key else 'flat'}_{os.getpid()}.yaml"
    )
    temp_file.write_text(yaml.safe_dump(params_data, sort_keys=False))
    return str(temp_file)


def _build_configured_params(context):
    return ParameterFile(
        _materialize_nav2_params(context, include_root_key=True),
        allow_substs=True,
    )


def _validate_nav2_params(context) -> None:
    params_data = _rewrite_nav2_params(context)
    rewritten_params_path = params_data.pop("__rewritten_params_path__", "<unknown>")
    source_params_file = params_data.pop("__source_params_file__", "<unknown>")

    required_keys = [
        "controller_server",
        "local_costmap",
        "smoother_server",
        "planner_server",
        "global_costmap",
        "behavior_server",
        "bt_navigator",
        "bt_navigator_navigate_through_poses_rclcpp_node",
        "bt_navigator_navigate_to_pose_rclcpp_node",
        "waypoint_follower",
        "velocity_smoother",
    ]
    missing_keys = [key for key in required_keys if key not in params_data]
    if missing_keys:
        available_keys = sorted(params_data.keys())
        raise RuntimeError(
            "Nav2 params rewrite produced an unexpected structure. "
            f"Missing top-level keys: {missing_keys}. "
            f"Available top-level keys: {available_keys}. "
            f"Source params file: {source_params_file}. "
            f"Rewritten params file: {rewritten_params_path}."
        )



def _build_nav2_group(context):
    namespace_value = _normalize_namespace(LaunchConfiguration('namespace').perform(context))
    if not namespace_value:
        namespace_value = 'amr_sweeper'

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    enable_controller_server = (
        LaunchConfiguration('enable_controller_server').perform(context).lower() == 'true')
    enable_smoother_server = (
        LaunchConfiguration('enable_smoother_server').perform(context).lower() == 'true')
    enable_planner_server = (
        LaunchConfiguration('enable_planner_server').perform(context).lower() == 'true')
    enable_behavior_server = (
        LaunchConfiguration('enable_behavior_server').perform(context).lower() == 'true')
    enable_bt_navigator = (
        LaunchConfiguration('enable_bt_navigator').perform(context).lower() == 'true')
    enable_waypoint_follower = (
        LaunchConfiguration('enable_waypoint_follower').perform(context).lower() == 'true')
    enable_velocity_smoother = (
        LaunchConfiguration('enable_velocity_smoother').perform(context).lower() == 'true')

    lifecycle_nodes = []
    if enable_controller_server:
        lifecycle_nodes.append('controller_server')
    if enable_smoother_server:
        lifecycle_nodes.append('smoother_server')
    if enable_planner_server:
        lifecycle_nodes.append('planner_server')
    if enable_behavior_server:
        lifecycle_nodes.append('behavior_server')
    if enable_bt_navigator:
        lifecycle_nodes.append('bt_navigator')
    if enable_waypoint_follower:
        lifecycle_nodes.append('waypoint_follower')
    if enable_velocity_smoother:
        lifecycle_nodes.append('velocity_smoother')

    _validate_nav2_params(context)
    configured_params = _build_configured_params(context)

    actions = []

    if enable_controller_server:
        actions.append(Node(
                namespace=namespace_value,
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=[('cmd_vel', 'navigation/cmd_vel_raw'),
                            ('odom', 'localization/odometry_fused'),
                            ('local_plan', 'navigation/controller_server/local_plan'),
                            ('cost_cloud', 'navigation/controller_server/cost_cloud'),
                            ('transition_event', 'navigation/controller_server/transition_event')]))
    if enable_smoother_server:
        actions.append(Node(
                namespace=namespace_value,
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],))
    if enable_planner_server:
        actions.append(Node(
                namespace=namespace_value,
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=[('plan', 'navigation/planner_server/plan'),
                            ('transition_event', 'navigation/planner_server/transition_event')],))
    if enable_behavior_server:
        actions.append(Node(
                namespace=namespace_value,
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],))
    if enable_bt_navigator:
        actions.append(Node(
                namespace=namespace_value,
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=[('goal_pose', 'navigation/bt_navigator/goal_pose'),
                            ('transition_event', 'navigation/bt_navigator/transition_event')],))
    if enable_waypoint_follower:
        actions.append(Node(
                namespace=namespace_value,
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=[('transition_event', 'navigation/waypoint_follower/transition_event')],))
    if enable_velocity_smoother:
        actions.append(Node(
                namespace=namespace_value,
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=[('cmd_vel', 'navigation/cmd_vel_raw'),
                            ('cmd_vel_smoothed', 'navigation/cmd_vel'),
                            ('odom', 'localization/odometry_fused')]))

    actions.append(Node(
        namespace=namespace_value,
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]))

    return [GroupAction(actions=actions)]


def generate_launch_description():
    bringup_dir = get_package_share_directory('amr_sweeper_navigation')
    console_output_format = "[{severity}] [{time}] [{name}] : {message}"

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    colorized_logs_envvar = SetEnvironmentVariable(
        'RCUTILS_COLORIZED_OUTPUT', '1')
    console_output_format_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', console_output_format)

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='amr_sweeper',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use ROS time if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'programmed_missions_navigation.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_mission_costmap_yaml_cmd = DeclareLaunchArgument(
        'mission_costmap_yaml',
        default_value='',
        description='Exact mission run costmap YAML from execution_context.json. Leave empty outside mission execution.')

    declare_enable_controller_server_cmd = DeclareLaunchArgument(
        'enable_controller_server',
        default_value='true',
        description='Launch the Nav2 controller server')

    declare_enable_smoother_server_cmd = DeclareLaunchArgument(
        'enable_smoother_server',
        default_value='true',
        description='Launch the Nav2 smoother server')

    declare_enable_planner_server_cmd = DeclareLaunchArgument(
        'enable_planner_server',
        default_value='true',
        description='Launch the Nav2 planner server')

    declare_enable_behavior_server_cmd = DeclareLaunchArgument(
        'enable_behavior_server',
        default_value='true',
        description='Launch the Nav2 behavior server')

    declare_enable_bt_navigator_cmd = DeclareLaunchArgument(
        'enable_bt_navigator',
        default_value='true',
        description='Launch the Nav2 behavior-tree navigator')

    declare_enable_waypoint_follower_cmd = DeclareLaunchArgument(
        'enable_waypoint_follower',
        default_value='true',
        description='Launch the Nav2 waypoint follower')

    declare_enable_velocity_smoother_cmd = DeclareLaunchArgument(
        'enable_velocity_smoother',
        default_value='true',
        description='Launch the Nav2 velocity smoother')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='false',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_logs_envvar)
    ld.add_action(console_output_format_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mission_costmap_yaml_cmd)
    ld.add_action(declare_enable_controller_server_cmd)
    ld.add_action(declare_enable_smoother_server_cmd)
    ld.add_action(declare_enable_planner_server_cmd)
    ld.add_action(declare_enable_behavior_server_cmd)
    ld.add_action(declare_enable_bt_navigator_cmd)
    ld.add_action(declare_enable_waypoint_follower_cmd)
    ld.add_action(declare_enable_velocity_smoother_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(OpaqueFunction(function=_build_nav2_group))

    return ld
