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
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _normalize_namespace(namespace: str) -> str:
    return namespace.strip().strip('/')


def _build_namespaced_params_file(context, namespace_value: str) -> str:
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
    }

    rewritten_params_path = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True,
    ).perform(context)

    params_data = yaml.safe_load(Path(rewritten_params_path).read_text()) or {}
    namespaced_params = {
        f"/{namespace_value}/{node_name}": node_params
        for node_name, node_params in params_data.items()
    }

    with tempfile.NamedTemporaryFile(
        mode='w',
        suffix='.yaml',
        prefix='amr_sweeper_nav2_params_',
        delete=False,
    ) as handle:
        yaml.safe_dump(namespaced_params, handle, sort_keys=False)
        return handle.name


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

    configured_params = _build_namespaced_params_file(context, namespace_value)

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

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    colorized_logs_envvar = SetEnvironmentVariable(
        'RCUTILS_COLORIZED_OUTPUT', '1')

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
        default_value='src/missions_log/global_costmap.yaml',
        description='Exact mission costmap YAML from execution_context.json. Leave empty outside mission execution.')

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
