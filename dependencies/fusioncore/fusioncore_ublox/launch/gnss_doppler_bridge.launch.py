from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "navpvt_topic",
            default_value="/ublox/navpvt",
            description="Input ublox_msgs/NavPVT topic"
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/gnss/doppler_vel",
            description="Output nav_msgs/Odometry topic (set gnss.velocity_topic to this in FusionCore config)"
        ),
        DeclareLaunchArgument(
            "min_speed_mps",
            default_value="0.05",
            description="Drop fixes below this ground speed (m/s) to avoid heading corruption at standstill"
        ),
        Node(
            package="fusioncore_ublox",
            executable="gnss_doppler_bridge",
            name="gnss_doppler_bridge",
            output="screen",
            parameters=[{
                "navpvt_topic": LaunchConfiguration("navpvt_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "min_speed_mps": LaunchConfiguration("min_speed_mps"),
            }],
        ),
    ])
