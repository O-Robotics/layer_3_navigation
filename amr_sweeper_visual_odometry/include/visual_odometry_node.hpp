#ifndef AMR_SWEEPER_VISUAL_ODOMETRY__VISUAL_ODOMETRY_NODE_HPP_
#define AMR_SWEEPER_VISUAL_ODOMETRY__VISUAL_ODOMETRY_NODE_HPP_

#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace amr_sweeper_visual_odometry
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
    double metric_translation_scale{0.0};
    double delta_yaw_rad{0.0};
    double dt_seconds{0.0};
    tf2::Transform delta_base{tf2::Transform::getIdentity()};
    std::vector<cv::Point2f> tracked_points;
    std::string reason;
  };

  struct CameraCalibration
  {
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    std::string distortion_model;

    [[nodiscard]] bool ready() const
    {
      return !camera_matrix.empty();
    }
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
    std::string frame_override;
    double fusion_weight{1.0};
    CameraCalibration calibration;
    cv::Mat latest_gray_image;
    rclcpp::Time latest_image_stamp{0, 0, RCL_ROS_TIME};
    std::string latest_frame_id;
    cv::Mat previous_gray_image;
    std::vector<cv::Point2f> previous_points;
    rclcpp::Time previous_image_stamp{0, 0, RCL_ROS_TIME};
    bool have_previous_motion_reference{false};
    tf2::Vector3 previous_motion_reference_position{0.0, 0.0, 0.0};
    double previous_motion_reference_yaw_rad{0.0};
    tf2::Transform base_to_camera{tf2::Transform::getIdentity()};
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
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
  void imageCallback(
    const std::shared_ptr<CameraState> & camera,
    const sensor_msgs::msg::Image::SharedPtr message);
  void wheelOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr message);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr message);

  void initializeFromFrame(
    const std::shared_ptr<CameraState> & camera,
    const cv::Mat & gray_image,
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
  double camera_fusion_tolerance_seconds_{0.03};
  double tf_warning_tolerance_ms_{5.0};
  double stereo_match_max_error_{20.0};
  double stereo_min_disparity_px_{1.0};
  double stereo_max_reprojection_error_m_{0.20};
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

}  // namespace amr_sweeper_visual_odometry

#endif  // AMR_SWEEPER_VISUAL_ODOMETRY__VISUAL_ODOMETRY_NODE_HPP_
