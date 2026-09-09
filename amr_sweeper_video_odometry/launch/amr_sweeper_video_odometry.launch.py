from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    visual_slam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            'num_cameras': 2,
            'rectified_images': True,
            'tracking_mode': 1,
            'base_frame': 'base_link',

            'camera_optical_frames': [
              'depth_camera_infra1_optical_frame',
              'depth_camera_infra2_optical_frame',
            ],

            # Replace with the actual header.frame_id from motion/sample.
            'imu_frame': 'depth_camera_motion_optical_frame',

            'calibration_frequency': 200.0,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'image_jitter_threshold_ms': 70.0,
            'enable_slam_visualization': False,
        }],
        remappings=[
            (
                'visual_slam/image_0',
                '/amr_sweeper/depth_camera/infra1/image_rect_raw',
            ),
            (
                'visual_slam/camera_info_0',
                '/amr_sweeper/depth_camera/infra1/camera_info',
            ),
            (
                'visual_slam/image_1',
                '/amr_sweeper/depth_camera/infra2/image_rect_raw',
            ),
            (
                'visual_slam/camera_info_1',
                '/amr_sweeper/depth_camera/infra2/camera_info',
            ),
            (
                'visual_slam/imu',
                '/amr_sweeper/depth_camera/motion/sample',
            ),
        ],
    )

    container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam],
        output='screen',
    )

    return LaunchDescription([container])

