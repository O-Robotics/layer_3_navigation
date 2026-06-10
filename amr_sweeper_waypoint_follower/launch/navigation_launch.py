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
    namespace_value = LaunchConfiguration('namespace').perform(context)
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    mission_costmap_yaml = LaunchConfiguration('mission_costmap_yaml')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'yaml_filename': map_yaml_file,
        'costmap_yaml_path': mission_costmap_yaml,
        'map_topic': _qualify_topic(namespace_value, 'mapping/map_padded'),
    }

    rewritten_params_path = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True,
    ).perform(context)

    params_data = yaml.safe_load(Path(rewritten_params_path).read_text()) or {}
    params_data["__rewritten_params_path__"] = rewritten_params_path
    params_data["__source_params_file__"] = params_file.perform(context)
    return params_data


def _build_configured_params(context):
    namespace = LaunchConfiguration('namespace')
    namespace_value = namespace.perform(context)
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    mission_costmap_yaml = LaunchConfiguration('mission_costmap_yaml')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'yaml_filename': map_yaml_file,
        'costmap_yaml_path': mission_costmap_yaml,
        'map_topic': _qualify_topic(namespace_value, 'mapping/map_padded'),
    }

    return ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
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

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    _validate_nav2_params(context)
    configured_params = _build_configured_params(context)

    return [GroupAction(
        actions=[
            Node(
                namespace=namespace_value,
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav_raw'),
                            ('odom', 'odometry/fused')]),
            Node(
                namespace=namespace_value,
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],),
            Node(
                namespace=namespace_value,
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],),
            Node(
                namespace=namespace_value,
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],),
            Node(
                namespace=namespace_value,
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],),
            Node(
                namespace=namespace_value,
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],),
            Node(
                namespace=namespace_value,
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav_raw'),
                            ('cmd_vel_smoothed', 'cmd_vel_nav'),
                            ('odom', 'odometry/fused')]),
            Node(
                namespace=namespace_value,
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )]


def generate_launch_description():
    bringup_dir = get_package_share_directory('amr_sweeper_waypoint_follower')
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

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file to inject into the Nav2 config')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_mission_costmap_yaml_cmd = DeclareLaunchArgument(
        'mission_costmap_yaml',
        default_value='',
        description='Exact mission run costmap YAML from execution_context.json. Leave empty outside mission execution.')

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
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mission_costmap_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(OpaqueFunction(function=_build_nav2_group))

    return ld
