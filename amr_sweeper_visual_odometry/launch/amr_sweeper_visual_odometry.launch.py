#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def _launch_fusioncore(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    config_path = LaunchConfiguration("fusioncore_params_file").perform(context)

    node = LifecycleNode(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        namespace=namespace,
        output="screen",
        parameters=[
            config_path,
            {
                "use_sim_time": use_sim_time,
                "base_frame": "base_footprint",
                "odom_frame": "odom",
            },
        ],
        remappings=[
            ("/imu/data", "imu/data_acc_gyro"),
            ("/odom/wheels", "diff_cont/odom"),
            ("/gnss/fix", "gnss/navsat"),
            ("/fusion/odom", "odometry/fused"),
            ("/fusion/pose", "pose"),
        ],
        condition=IfCondition(LaunchConfiguration("use_fusioncore")),
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
        condition=IfCondition(LaunchConfiguration("use_fusioncore")),
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action is node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(LaunchConfiguration("use_fusioncore")),
    )

    return [node, configure, activate]


def generate_launch_description():
    package_share = FindPackageShare("amr_sweeper_visual_odometry")
    params_file = PathJoinSubstitution(
        [package_share, "config", "monocular_visual_odometry.yaml"]
    )
    fusioncore_params_file = PathJoinSubstitution(
        [package_share, "config", "fusioncore_visual_odometry.yaml"]
    )

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="amr_sweeper",
                description="Namespace for visual odometry nodes",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use ROS time if true",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Log level for launched nodes",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Parameter file for the monocular visual odometry node",
            ),
            DeclareLaunchArgument(
                "use_fusioncore",
                default_value="false",
                description="Launch FusionCore using the local stack config plus visual odometry as a secondary odometry source",
            ),
            DeclareLaunchArgument(
                "fusioncore_params_file",
                default_value=fusioncore_params_file,
                description="Parameter file for the optional FusionCore node",
            ),
            Node(
                package="amr_sweeper_visual_odometry",
                executable="monocular_visual_odometry_node",
                name="monocular_visual_odometry",
                namespace=namespace,
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
            OpaqueFunction(function=_launch_fusioncore),
        ]
    )
