#ifndef AMR_SWEEPER_VISUAL_ODOMETRY__MONOCULAR_VISUAL_ODOMETRY_NODE_HPP_
#define AMR_SWEEPER_VISUAL_ODOMETRY__MONOCULAR_VISUAL_ODOMETRY_NODE_HPP_

#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace amr_sweeper_visual_odometry
{

class MonocularVisualOdometryNode : public rclcpp::Node
{
public:
  explicit MonocularVisualOdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
    CameraCalibration calibration;
    cv::Mat previous_gray_image;
    std::vector<cv::Point2f> previous_points;
    rclcpp::Time previous_image_stamp{0, 0, RCL_ROS_TIME};
    nav_msgs::msg::Odometry previous_scale_reference;
    bool have_previous_scale_reference{false};
    bool extrinsics_resolved{false};
    tf2::Transform base_to_camera{tf2::Transform::getIdentity()};
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
  };

  struct PendingEstimate
  {
    std::string camera_name;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    TrackingResult result;
  };

  void configureCameras();
  void cameraInfoCallback(
    const std::shared_ptr<CameraState> & camera,
    const sensor_msgs::msg::CameraInfo::SharedPtr message);
  void imageCallback(
    const std::shared_ptr<CameraState> & camera,
    const sensor_msgs::msg::Image::SharedPtr message);
  void wheelOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr message);

  void initializeFromFrame(
    const std::shared_ptr<CameraState> & camera,
    const cv::Mat & gray_image,
    const nav_msgs::msg::Odometry & scale_reference,
    const rclcpp::Time & stamp);
  [[nodiscard]] TrackingResult estimateMotion(
    const std::shared_ptr<CameraState> & camera,
    const cv::Mat & current_gray_image,
    const nav_msgs::msg::Odometry & current_scale_reference,
    const rclcpp::Time & stamp);
  [[nodiscard]] std::vector<cv::Point2f> detectFeatures(const cv::Mat & gray_image) const;
  [[nodiscard]] std::vector<cv::Point2f> undistortPoints(
    const CameraCalibration & calibration,
    const std::vector<cv::Point2f> & points) const;
  [[nodiscard]] std::optional<nav_msgs::msg::Odometry> lookupWheelOdom(
    const rclcpp::Time & stamp) const;
  bool resolveCameraExtrinsics(
    const std::shared_ptr<CameraState> & camera,
    const std::string & camera_frame);
  void queueEstimate(
    const std::string & camera_name,
    const TrackingResult & result,
    const rclcpp::Time & stamp);
  void flushPendingEstimates(const rclcpp::Time & reference_stamp, bool force_flush);
  [[nodiscard]] TrackingResult fuseTrackingResults(
    const std::vector<PendingEstimate> & estimates) const;
  void publishOdometry(
    const TrackingResult & result,
    const rclcpp::Time & stamp,
    double dt_seconds);
  void publishStatus(const std::string & status_message) const;
  [[nodiscard]] double trackingConfidence(const TrackingResult & result) const;

  static double normalizeAngle(double angle_rad);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion);
  static double planarDistance(
    const geometry_msgs::msg::Point & lhs,
    const geometry_msgs::msg::Point & rhs);
  static tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose);
  static geometry_msgs::msg::Pose transformToPose(const tf2::Transform & transform);
  [[nodiscard]] std::array<double, 36> makePoseCovariance(const TrackingResult & result) const;
  [[nodiscard]] std::array<double, 36> makeTwistCovariance(const TrackingResult & result) const;

  std::string wheel_odom_topic_;
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
  double wheel_lookup_tolerance_seconds_{0.25};
  double wheel_history_seconds_{5.0};
  double min_scale_translation_meters_{0.005};
  double camera_fusion_tolerance_seconds_{0.03};
  double pose_sigma_floor_m_{0.03};
  double pose_sigma_ceiling_m_{0.50};
  double yaw_sigma_floor_rad_{0.02};
  double yaw_sigma_ceiling_rad_{0.35};
  int min_cameras_per_estimate_{1};

  bool have_pose_estimate_{false};
  tf2::Transform odom_to_base_{tf2::Transform::getIdentity()};

  std::vector<std::shared_ptr<CameraState>> cameras_;
  std::deque<nav_msgs::msg::Odometry> wheel_odom_history_;
  std::deque<PendingEstimate> pending_estimates_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace amr_sweeper_visual_odometry

#endif  // AMR_SWEEPER_VISUAL_ODOMETRY__MONOCULAR_VISUAL_ODOMETRY_NODE_HPP_
