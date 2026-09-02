// Copyright 2026 O-Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VISUAL_ODOMETRY_NODE_HPP_
#define VISUAL_ODOMETRY_NODE_HPP_

#include <Eigen/Dense>  // NOLINT(build/include_order)
#include <opencv2/core.hpp>  // NOLINT(build/include_order)
#include <tf2/LinearMath/Transform.h>  // NOLINT(build/include_order)

#include <array>  // NOLINT(build/include_order)
#include <deque>  // NOLINT(build/include_order)
#include <memory>  // NOLINT(build/include_order)
#include <optional>  // NOLINT(build/include_order)
#include <string>  // NOLINT(build/include_order)
#include <utility>  // NOLINT(build/include_order)
#include <vector>  // NOLINT(build/include_order)

#include <nav_msgs/msg/odometry.hpp>  // NOLINT(build/include_order)
#include <rclcpp/rclcpp.hpp>  // NOLINT(build/include_order)
#include <sensor_msgs/msg/camera_info.hpp>  // NOLINT(build/include_order)
#include <sensor_msgs/msg/image.hpp>  // NOLINT(build/include_order)
#include <sensor_msgs/msg/imu.hpp>  // NOLINT(build/include_order)
#include <std_msgs/msg/string.hpp>  // NOLINT(build/include_order)
#include <tf2_ros/buffer.h>  // NOLINT(build/include_order)
#include <tf2_ros/transform_broadcaster.h>  // NOLINT(build/include_order)
#include <tf2_ros/transform_listener.h>  // NOLINT(build/include_order)

namespace amr_sweeper_localization
{

class VisualOdometryNode : public rclcpp::Node
{
public:
  explicit VisualOdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct TrackingResult
  {
    bool success{false};
    int tracked_features{0};
    int inliers{0};
    int valid_depth_points{0};
    int rejected_depth_points{0};
    double metric_translation_scale{0.0};
    double delta_yaw_rad{0.0};
    double dt_seconds{0.0};
    double residual_m{0.0};
    double sync_delta_seconds{0.0};
    tf2::Transform delta_base{tf2::Transform::getIdentity()};
    std::vector<cv::Point2f> tracked_points;
    std::string metric_source{"none"};
    std::string reason;
  };

  struct CameraCalibration
  {
    cv::Mat camera_matrix;
    cv::Mat projection_matrix;
    cv::Mat distortion_coefficients;
    std::string distortion_model;

    [[nodiscard]] bool ready() const
    {
      return !camera_matrix.empty();
    }
  };

  struct CameraImageSample
  {
    cv::Mat gray_image;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::string frame_id;
  };

  struct DepthSample
  {
    cv::Mat depth_meters;
    CameraCalibration calibration;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::string frame_id;
  };

  struct CameraState
  {
    explicit CameraState(std::string camera_name)
    : name(std::move(camera_name))
    {
    }

    std::string name;
    std::string image_topic;
    std::string camera_info_topic;
    std::string depth_image_topic;
    std::string depth_camera_info_topic;
    std::string frame_override;
    std::string depth_frame_override;
    double fusion_weight{1.0};
    bool use_depth{false};
    CameraCalibration calibration;
    CameraCalibration depth_calibration;
    cv::Mat latest_gray_image;
    rclcpp::Time latest_image_stamp{0, 0, RCL_ROS_TIME};
    std::string latest_frame_id;
    std::deque<CameraImageSample> image_history;
    std::deque<DepthSample> depth_history;
    cv::Mat previous_gray_image;
    std::vector<cv::Point2f> previous_points;
    std::optional<DepthSample> previous_depth_sample;
    rclcpp::Time previous_image_stamp{0, 0, RCL_ROS_TIME};
    bool have_previous_motion_reference{false};
    tf2::Vector3 previous_motion_reference_position{0.0, 0.0, 0.0};
    double previous_motion_reference_yaw_rad{0.0};
    tf2::Transform base_to_camera{tf2::Transform::getIdentity()};
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscription;
  };

  struct MotionReferenceSample
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::optional<tf2::Vector3> planar_position;
    std::optional<double> yaw_rad;
  };

  struct StereoFrame
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    cv::Mat left_gray_image;
    cv::Mat right_gray_image;
    std::vector<cv::Point2f> left_points;
    MotionReferenceSample motion_reference;
    tf2::Transform base_to_left{tf2::Transform::getIdentity()};
    tf2::Transform base_to_right{tf2::Transform::getIdentity()};
  };

  struct StereoPairState
  {
    std::string name;
    std::string left_camera_name;
    std::string right_camera_name;
    double fusion_weight{1.0};
    double sync_tolerance_seconds{0.02};
    std::shared_ptr<CameraState> left_camera;
    std::shared_ptr<CameraState> right_camera;
    std::optional<StereoFrame> previous_frame;
  };

  struct PendingEstimate
  {
    std::string camera_name;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    double fusion_weight{1.0};
    TrackingResult result;
  };

  void configureCameras();
  void configureStereoPairs();
  void cameraInfoCallback(
    const std::shared_ptr<CameraState> & camera,
    const sensor_msgs::msg::CameraInfo::SharedPtr message);
  void depthCameraInfoCallback(
    const std::shared_ptr<CameraState> & camera,
    const sensor_msgs::msg::CameraInfo::SharedPtr message);
  void imageCallback(
    const std::shared_ptr<CameraState> & camera,
    const sensor_msgs::msg::Image::SharedPtr message);
  void depthImageCallback(
    const std::shared_ptr<CameraState> & camera,
    const sensor_msgs::msg::Image::SharedPtr message);
  void wheelOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr message);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr message);

  void initializeFromFrame(
    const std::shared_ptr<CameraState> & camera,
    const cv::Mat & gray_image,
    const std::optional<DepthSample> & depth_sample,
    const MotionReferenceSample & motion_reference,
    const rclcpp::Time & stamp);
  void initializeStereoFrame(
    StereoPairState & stereo_pair,
    const cv::Mat & left_gray_image,
    const cv::Mat & right_gray_image,
    const MotionReferenceSample & motion_reference,
    const rclcpp::Time & stamp);
  [[nodiscard]] TrackingResult estimateMotion(
    const std::shared_ptr<CameraState> & camera,
    const cv::Mat & current_gray_image,
    const MotionReferenceSample & current_motion_reference,
    const rclcpp::Time & stamp);
  [[nodiscard]] TrackingResult estimateDepthMotion(
    const std::shared_ptr<CameraState> & camera,
    const cv::Mat & current_gray_image,
    const DepthSample & current_depth_sample,
    const MotionReferenceSample & current_motion_reference,
    const rclcpp::Time & stamp);
  [[nodiscard]] TrackingResult estimateStereoMotion(
    StereoPairState & stereo_pair,
    const cv::Mat & current_left_gray_image,
    const cv::Mat & current_right_gray_image,
    const MotionReferenceSample & current_motion_reference,
    const tf2::Transform & base_to_left,
    const tf2::Transform & base_to_right,
    const rclcpp::Time & stamp);
  [[nodiscard]] std::vector<cv::Point2f> detectFeatures(const cv::Mat & gray_image) const;
  [[nodiscard]] std::vector<cv::Point2f> undistortPoints(
    const CameraCalibration & calibration,
    const std::vector<cv::Point2f> & points) const;
  [[nodiscard]] std::optional<MotionReferenceSample> lookupMotionReference(
    const rclcpp::Time & stamp) const;
  bool resolveCameraExtrinsics(
    const std::shared_ptr<CameraState> & camera,
    const std::string & camera_frame,
    const rclcpp::Time & stamp);
  StereoPairState * findStereoPairByCameraName(const std::string & camera_name);
  const StereoPairState * findStereoPairByCameraName(const std::string & camera_name) const;
  [[nodiscard]] bool triangulateStereoCorrespondences(
    const CameraCalibration & left_calibration,
    const CameraCalibration & right_calibration,
    const tf2::Transform & left_to_right,
    const std::vector<cv::Point2f> & left_points,
    const std::vector<cv::Point2f> & right_points,
    std::vector<Eigen::Vector3d> & points_3d,
    std::vector<unsigned char> * valid_mask = nullptr) const;
  [[nodiscard]] bool estimateRigidTransform(
    const std::vector<Eigen::Vector3d> & previous_points,
    const std::vector<Eigen::Vector3d> & current_points,
    tf2::Transform & delta_transform,
    int & inlier_count) const;
  [[nodiscard]] std::optional<DepthSample> findNearestDepthSample(
    const std::shared_ptr<CameraState> & camera,
    const rclcpp::Time & stamp) const;
  [[nodiscard]] std::optional<double> sampleDepthMeters(
    const cv::Mat & depth_meters,
    const cv::Point2f & point) const;
  [[nodiscard]] std::optional<Eigen::Vector3d> backProjectDepthPoint(
    const CameraCalibration & calibration,
    const cv::Point2f & point,
    double depth_meters) const;
  [[nodiscard]] static cv::Mat makeProjectionMatrix(
    const tf2::Matrix3x3 & rotation,
    const tf2::Vector3 & translation);
  void queueEstimate(
    const std::string & camera_name,
    const TrackingResult & result,
    const rclcpp::Time & stamp,
    double fusion_weight);
  void flushPendingEstimates(const rclcpp::Time & reference_stamp, bool force_flush);
  [[nodiscard]] TrackingResult fuseTrackingResults(
    const std::vector<PendingEstimate> & estimates) const;
  void publishOdometry(
    const TrackingResult & result,
    const rclcpp::Time & stamp,
    double dt_seconds);
  void publishStatus(const std::string & status_message) const;
  [[nodiscard]] std::string describeMotionReferenceAvailability(const rclcpp::Time & stamp) const;
  [[nodiscard]] double trackingConfidence(const TrackingResult & result) const;
  [[nodiscard]] Eigen::Matrix3d makeMeasurementCovariance(const TrackingResult & result) const;

  static double normalizeAngle(double angle_rad);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion);
  static double planarDistance(
    const geometry_msgs::msg::Point & lhs,
    const geometry_msgs::msg::Point & rhs);
  static double planarDistance(const tf2::Vector3 & lhs, const tf2::Vector3 & rhs);
  static tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose);
  static geometry_msgs::msg::Pose transformToPose(const tf2::Transform & transform);
  [[nodiscard]] std::array<double, 36> makePoseCovariance(const TrackingResult & result) const;
  [[nodiscard]] std::array<double, 36> makeTwistCovariance(
    const TrackingResult & result,
    double dt_seconds) const;

  std::string wheel_odom_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string legacy_camera_image_topic_;
  std::string legacy_camera_info_topic_;
  std::string legacy_camera_frame_override_;

  bool publish_tf_{false};
  bool force_2d_{true};
  int max_features_{500};
  int min_tracked_features_{90};
  int min_inliers_{40};
  double feature_quality_level_{0.01};
  double feature_min_distance_px_{10.0};
  int corner_refinement_window_px_{5};
  int optical_flow_window_px_{21};
  int optical_flow_max_pyramid_level_{3};
  double optical_flow_max_error_{20.0};
  double ransac_confidence_{0.999};
  double ransac_reprojection_threshold_px_{1.5};
  double min_seconds_between_keyframes_{0.05};
  double motion_reference_lookup_tolerance_seconds_{0.25};
  double motion_reference_history_seconds_{5.0};
  double min_scale_translation_meters_{0.005};
  bool reject_zero_scale_updates_{true};
  double camera_fusion_tolerance_seconds_{0.03};
  double tf_warning_tolerance_ms_{5.0};
  double stereo_match_max_error_{20.0};
  double stereo_image_history_seconds_{0.5};
  double stereo_min_disparity_px_{1.0};
  double stereo_max_reprojection_error_m_{0.20};
  double depth_sync_tolerance_seconds_{0.08};
  double depth_history_seconds_{0.5};
  double depth_min_range_m_{0.20};
  double depth_max_range_m_{8.0};
  int depth_sampling_radius_px_{1};
  int depth_min_inliers_{20};
  double depth_min_valid_ratio_{0.35};
  double depth_max_residual_m_{0.20};
  double depth_max_translation_m_{0.50};
  double depth_max_yaw_rad_{0.35};
  double pose_sigma_floor_m_{0.03};
  double pose_sigma_ceiling_m_{0.50};
  double yaw_sigma_floor_rad_{0.02};
  double yaw_sigma_ceiling_rad_{0.35};
  int min_cameras_per_estimate_{1};
  bool use_imu_yaw_{true};
  bool use_wheel_scale_{true};
  double imu_max_dt_seconds_{0.1};
  double imu_planar_accel_deadband_mps2_{0.15};
  double imu_velocity_damping_per_second_{1.5};
  double imu_stationary_angular_velocity_threshold_rad_s_{0.05};
  std::string fusion_method_{"ekf"};

  bool have_pose_estimate_{false};
  tf2::Transform odom_to_base_{tf2::Transform::getIdentity()};
  bool have_previous_imu_sample_{false};
  rclcpp::Time previous_imu_stamp_{0, 0, RCL_ROS_TIME};
  tf2::Vector3 imu_planar_velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 imu_planar_position_{0.0, 0.0, 0.0};

  std::vector<std::shared_ptr<CameraState>> cameras_;
  std::vector<StereoPairState> stereo_pairs_;
  std::deque<MotionReferenceSample> wheel_motion_reference_history_;
  std::deque<MotionReferenceSample> imu_motion_reference_history_;
  std::deque<PendingEstimate> pending_estimates_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace amr_sweeper_localization

#endif  // VISUAL_ODOMETRY_NODE_HPP_
