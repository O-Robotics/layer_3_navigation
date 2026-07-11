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
import math

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


def _set_nested(mapping: dict, path: list[str], value) -> None:
    current = mapping
    for key in path[:-1]:
        next_value = current.get(key)
        if not isinstance(next_value, dict):
            return
        current = next_value
    if path[-1] in current:
        current[path[-1]] = value


def _nav2_params_root(params_data: dict, namespace_value: str) -> dict:
    if namespace_value and isinstance(params_data.get(namespace_value), dict):
        return params_data[namespace_value]
    return params_data


def _footprint_minimum_inflation_radius(footprint) -> float | None:
    if isinstance(footprint, str):
        try:
            footprint = yaml.safe_load(footprint)
        except yaml.YAMLError:
            return None
    if not isinstance(footprint, list) or len(footprint) < 2:
        return None

    points = []
    for point in footprint:
        if not isinstance(point, list) or len(point) < 2:
            return None
        try:
            points.append((float(point[0]), float(point[1])))
        except (TypeError, ValueError):
            return None

    largest_cross_section = 0.0
    for first_index, first_point in enumerate(points):
        for second_point in points[first_index + 1:]:
            largest_cross_section = max(
                largest_cross_section,
                math.hypot(first_point[0] - second_point[0], first_point[1] - second_point[1]),
            )
    return largest_cross_section / 2.0


def _qualify_nav2_topics(params_data: dict, namespace_value: str) -> None:
    nav2_params = _nav2_params_root(params_data, namespace_value)

    _set_nested(
        nav2_params,
        ['global_costmap', 'global_costmap', 'ros__parameters', 'static_layer', 'map_topic'],
        _qualify_topic(namespace_value, 'mapping/static_costmap'),
    )
    _set_nested(
        nav2_params,
        ['local_costmap', 'local_costmap', 'ros__parameters', 'obstacle_layer', 'scan', 'topic'],
        _qualify_topic(namespace_value, 'depth_camera/scan'),
    )
    _set_nested(
        nav2_params,
        ['global_costmap', 'global_costmap', 'ros__parameters', 'obstacle_layer', 'scan', 'topic'],
        _qualify_topic(namespace_value, 'depth_camera/scan'),
    )
    _set_nested(
        nav2_params,
        ['behavior_server', 'ros__parameters', 'local_costmap_topic'],
        _qualify_topic(namespace_value, 'local_costmap/costmap_raw'),
    )
    _set_nested(
        nav2_params,
        ['behavior_server', 'ros__parameters', 'global_costmap_topic'],
        _qualify_topic(namespace_value, 'global_costmap/costmap_raw'),
    )
    _set_nested(
        nav2_params,
        ['behavior_server', 'ros__parameters', 'local_footprint_topic'],
        _qualify_topic(namespace_value, 'local_costmap/published_footprint'),
    )
    _set_nested(
        nav2_params,
        ['behavior_server', 'ros__parameters', 'global_footprint_topic'],
        _qualify_topic(namespace_value, 'global_costmap/published_footprint'),
    )


def _read_mission_costmap_extent(mission_costmap_yaml_path: str):
    """Parse a map_server-style costmap YAML (+ its referenced PGM) and return
    (width_m, height_m, resolution, origin_x, origin_y), or None if the file is
    missing or malformed."""
    yaml_path = Path(mission_costmap_yaml_path)
    if not yaml_path.is_file():
        return None
    try:
        costmap_yaml = yaml.safe_load(yaml_path.read_text()) or {}
    except yaml.YAMLError:
        return None

    resolution = costmap_yaml.get('resolution')
    origin = costmap_yaml.get('origin')
    image_name = costmap_yaml.get('image')
    if not resolution or not image_name or not isinstance(origin, list) or len(origin) < 2:
        return None

    image_path = yaml_path.parent / image_name
    try:
        with open(image_path, 'rb') as image_file:
            magic = image_file.readline().strip()
            if magic not in (b'P5', b'P2'):
                return None
            dims_line = image_file.readline()
            while dims_line.strip().startswith(b'#'):
                dims_line = image_file.readline()
            width_cells, height_cells = (int(token) for token in dims_line.split())
    except (OSError, ValueError):
        return None

    resolution = float(resolution)
    return (
        width_cells * resolution,
        height_cells * resolution,
        resolution,
        float(origin[0]),
        float(origin[1]),
    )


def _apply_mission_costmap_extent(
    params_data: dict, namespace_value: str, mission_costmap_yaml_path: str,
) -> None:
    """Size the (non-rolling) global costmap to match the active mission's
    costmap artifact instead of letting Nav2 fall back to its own built-in
    default width/height/origin, which locks in a generic, frame-centered
    canvas that the static layer can never resize away from."""
    if not mission_costmap_yaml_path:
        return
    extent = _read_mission_costmap_extent(mission_costmap_yaml_path)
    if extent is None:
        return
    width_m, height_m, resolution, origin_x, origin_y = extent

    nav2_params = _nav2_params_root(params_data, namespace_value)
    global_costmap_params = (
        nav2_params
        .get('global_costmap', {})
        .get('global_costmap', {})
        .get('ros__parameters', {})
    )
    if not isinstance(global_costmap_params, dict) or global_costmap_params.get('rolling_window', False):
        # Rolling-window profiles keep a small, fixed-size window centered on the
        # robot; only the static whole-map profile should adopt the mission extent.
        return

    # Costmap2DROS declares width/height as integer-typed parameters (cell-grid
    # extent in meters, but typed as int) while resolution/origin are doubles;
    # handing it a float here trips rclcpp's InvalidParameterTypeException.
    global_costmap_params['width'] = int(round(width_m))
    global_costmap_params['height'] = int(round(height_m))
    global_costmap_params['resolution'] = resolution
    global_costmap_params['origin_x'] = origin_x
    global_costmap_params['origin_y'] = origin_y


def _rewrite_bt_xml_paths(params_data: dict, namespace_value: str) -> None:
    nav2_params = _nav2_params_root(params_data, namespace_value)
    bt_xml_path = os.path.join(
        get_package_share_directory('amr_sweeper_navigation'),
        'config',
        'navigate_through_poses_replanning_1hz.xml',
    )
    _set_nested(
        nav2_params,
        ['bt_navigator', 'ros__parameters', 'default_nav_through_poses_bt_xml'],
        bt_xml_path,
    )


def _rewrite_nav2_params(context) -> dict:
    rewritten_params_path = _materialize_nav2_params(context, include_root_key=True)
    params_data = yaml.safe_load(Path(rewritten_params_path).read_text()) or {}
    params_data["__rewritten_params_path__"] = rewritten_params_path
    params_data["__source_params_file__"] = LaunchConfiguration('params_file').perform(context)
    return params_data

def _materialize_nav2_params(context, *, include_root_key: bool) -> str:
    namespace_value = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
    }

    rewritten_params_path = RewrittenYaml(
        source_file=params_file,
        root_key=namespace_value if include_root_key else None,
        param_rewrites=param_substitutions,
        convert_types=True,
    ).perform(context)

    params_data = yaml.safe_load(Path(rewritten_params_path).read_text()) or {}
    _qualify_nav2_topics(params_data, namespace_value)
    _rewrite_bt_xml_paths(params_data, namespace_value)
    _apply_mission_costmap_extent(
        params_data,
        namespace_value,
        LaunchConfiguration('mission_costmap_yaml').perform(context),
    )

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
    namespace_value = LaunchConfiguration('namespace').perform(context)
    nav2_params = _nav2_params_root(params_data, namespace_value)

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
        "velocity_smoother",
    ]
    missing_keys = [key for key in required_keys if key not in nav2_params]
    if missing_keys:
        available_keys = sorted(nav2_params.keys())
        raise RuntimeError(
            "Nav2 params rewrite produced an unexpected structure. "
            f"Missing top-level keys: {missing_keys}. "
            f"Available top-level keys: {available_keys}. "
            f"Source params file: {source_params_file}. "
            f"Rewritten params file: {rewritten_params_path}."
        )

    global_costmap_params = (
        nav2_params
        .get("global_costmap", {})
        .get("global_costmap", {})
        .get("ros__parameters", {})
    )
    configured_plugins = global_costmap_params.get("plugins", [])
    inflation_layer = global_costmap_params.get("inflation_layer", {})
    inflation_radius = inflation_layer.get("inflation_radius")
    minimum_inflation_radius = _footprint_minimum_inflation_radius(
        global_costmap_params.get("footprint")
    )
    if "inflation_layer" not in configured_plugins:
        raise RuntimeError(
            "Global costmap is missing inflation_layer in its plugins list. "
            "SmacPlanner2D non-circular collision checking expects a loaded "
            f"inflation layer. Source params file: {source_params_file}. "
            f"Rewritten params file: {rewritten_params_path}."
        )
    if inflation_layer.get("plugin") != "nav2_costmap_2d::InflationLayer":
        raise RuntimeError(
            "Global costmap inflation_layer plugin is not nav2_costmap_2d::InflationLayer. "
            f"Configured value: {inflation_layer.get('plugin')!r}. "
            f"Source params file: {source_params_file}. "
            f"Rewritten params file: {rewritten_params_path}."
        )
    if minimum_inflation_radius is not None:
        try:
            configured_inflation_radius = float(inflation_radius)
        except (TypeError, ValueError):
            configured_inflation_radius = -1.0
        if configured_inflation_radius < minimum_inflation_radius:
            raise RuntimeError(
                "Global costmap inflation_radius is too small for the configured "
                "non-circular footprint. SmacPlanner2D recommends at least half "
                "of the robot's largest cross-section. "
                f"Configured inflation_radius: {inflation_radius}; "
                f"minimum: {minimum_inflation_radius:.3f}. "
                f"Source params file: {source_params_file}. "
                f"Rewritten params file: {rewritten_params_path}."
            )


def _controller_server_selector_params(context) -> dict:
    params_data = _rewrite_nav2_params(context)
    controller_params = params_data.get("controller_server", {}).get("ros__parameters", {})
    selector_keys = (
        "progress_checker_plugins",
        "goal_checker_plugins",
        "current_progress_checker",
        "current_goal_checker",
    )
    return {
        key: controller_params[key]
        for key in selector_keys
        if key in controller_params
    }



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
    if enable_velocity_smoother:
        lifecycle_nodes.append('velocity_smoother')

    _validate_nav2_params(context)
    configured_params = _build_configured_params(context)
    controller_server_selector_params = _controller_server_selector_params(context)

    actions = []

    nav2_cmd_vel_output_topic = (
        'navigation/cmd_vel_raw' if enable_velocity_smoother else 'navigation/cmd_vel'
    )

    if enable_controller_server:
        actions.append(Node(
                namespace=namespace_value,
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, controller_server_selector_params],
                remappings=[('cmd_vel', nav2_cmd_vel_output_topic),
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

    declare_use_simulation_cmd = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='Simulation mode flag')

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
    ld.add_action(declare_use_simulation_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mission_costmap_yaml_cmd)
    ld.add_action(declare_enable_controller_server_cmd)
    ld.add_action(declare_enable_smoother_server_cmd)
    ld.add_action(declare_enable_planner_server_cmd)
    ld.add_action(declare_enable_behavior_server_cmd)
    ld.add_action(declare_enable_bt_navigator_cmd)
    ld.add_action(declare_enable_velocity_smoother_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(OpaqueFunction(function=_build_nav2_group))

    return ld
