// Copyright (c) 2026 O-Robotics

#include "monocular_visual_odometry_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace amr_sweeper_visual_odometry
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kLargeVariance = 1.0e6;

}  // namespace

MonocularVisualOdometryNode::MonocularVisualOdometryNode(const rclcpp::NodeOptions & options)
: Node("monocular_visual_odometry", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  legacy_camera_image_topic_ = declare_parameter(
    "camera_image_topic",
    std::string("usb_cameras/tools_camera/image_raw"));
  legacy_camera_info_topic_ = declare_parameter(
    "camera_info_topic",
    std::string("usb_cameras/tools_camera/tools_camera_info"));
  wheel_odom_topic_ = declare_parameter("wheel_odom_topic", std::string("diff_cont/odom"));
  odom_topic_ = declare_parameter("odom_topic", std::string("visual_odometry/odom"));
  base_frame_ = declare_parameter("base_frame", std::string("base_footprint"));
  odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
  legacy_camera_frame_override_ = declare_parameter("camera_frame", std::string(""));
  publish_tf_ = declare_parameter("publish_tf", false);
  force_2d_ = declare_parameter("force_2d", true);
  max_features_ = declare_parameter("max_features", 500);
  min_tracked_features_ = declare_parameter("min_tracked_features", 90);
  min_inliers_ = declare_parameter("min_inliers", 40);
  feature_quality_level_ = declare_parameter("feature_quality_level", 0.01);
  feature_min_distance_px_ = declare_parameter("feature_min_distance_px", 10.0);
  corner_refinement_window_px_ = declare_parameter("corner_refinement_window_px", 5);
  optical_flow_window_px_ = declare_parameter("optical_flow_window_px", 21);
  optical_flow_max_pyramid_level_ = declare_parameter("optical_flow_max_pyramid_level", 3);
  optical_flow_max_error_ = declare_parameter("optical_flow_max_error", 20.0);
  ransac_confidence_ = declare_parameter("ransac_confidence", 0.999);
  ransac_reprojection_threshold_px_ = declare_parameter("ransac_reprojection_threshold_px", 1.5);
  min_seconds_between_keyframes_ = declare_parameter("min_seconds_between_keyframes", 0.05);
  wheel_lookup_tolerance_seconds_ = declare_parameter("wheel_lookup_tolerance_seconds", 0.25);
  wheel_history_seconds_ = declare_parameter("wheel_history_seconds", 5.0);
  min_scale_translation_meters_ = declare_parameter("min_scale_translation_meters", 0.005);
  camera_fusion_tolerance_seconds_ = declare_parameter("camera_fusion_tolerance_seconds", 0.03);
  pose_sigma_floor_m_ = declare_parameter("pose_sigma_floor_m", 0.03);
  pose_sigma_ceiling_m_ = declare_parameter("pose_sigma_ceiling_m", 0.50);
  yaw_sigma_floor_rad_ = declare_parameter("yaw_sigma_floor_rad", 0.02);
  yaw_sigma_ceiling_rad_ = declare_parameter("yaw_sigma_ceiling_rad", 0.35);
  min_cameras_per_estimate_ = declare_parameter("min_cameras_per_estimate", 1);

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  configureCameras();

  wheel_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    wheel_odom_topic_,
    rclcpp::QoS(50),
    std::bind(&MonocularVisualOdometryNode::wheelOdometryCallback, this, std::placeholders::_1));

  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
  status_publisher_ = create_publisher<std_msgs::msg::String>("visual_odometry/status", rclcpp::QoS(10));

  std::ostringstream camera_summary;
  for (std::size_t index = 0; index < cameras_.size(); ++index) {
    if (index > 0U) {
      camera_summary << ", ";
    }
    camera_summary << cameras_[index]->name << ":" << cameras_[index]->image_topic;
  }

  RCLCPP_INFO(
    get_logger(),
    "Monocular visual odometry configured with cameras=[%s] wheel_odom=%s output=%s",
    camera_summary.str().c_str(),
    wheel_odom_topic_.c_str(),
    odom_topic_.c_str());
}

void MonocularVisualOdometryNode::configureCameras()
{
  const auto configured_camera_names =
    declare_parameter("camera_names", std::vector<std::string>{});

  auto add_camera = [this](
    const std::string & name,
    const std::string & image_topic,
    const std::string & camera_info_topic,
    const std::string & frame_override) {
      auto camera = std::make_shared<CameraState>(name);
      camera->image_topic = image_topic;
      camera->camera_info_topic = camera_info_topic;
      camera->frame_override = frame_override;

      camera->camera_info_subscription = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera->camera_info_topic,
        rclcpp::SensorDataQoS(),
        [this, camera](const sensor_msgs::msg::CameraInfo::SharedPtr message) {
          cameraInfoCallback(camera, message);
        });

      camera->image_subscription = create_subscription<sensor_msgs::msg::Image>(
        camera->image_topic,
        rclcpp::SensorDataQoS(),
        [this, camera](const sensor_msgs::msg::Image::SharedPtr message) {
          imageCallback(camera, message);
        });

      cameras_.push_back(std::move(camera));
    };

  if (configured_camera_names.empty()) {
    add_camera(
      "tool_camera",
      legacy_camera_image_topic_,
      legacy_camera_info_topic_,
      legacy_camera_frame_override_);
    return;
  }

  for (const auto & camera_name : configured_camera_names) {
    const std::string prefix = "cameras." + camera_name + ".";
    const std::string image_topic = declare_parameter(prefix + "image_topic", std::string(""));
    const std::string camera_info_topic =
      declare_parameter(prefix + "camera_info_topic", std::string(""));
    const std::string frame_override =
      declare_parameter(prefix + "camera_frame", std::string(""));

    if (image_topic.empty() || camera_info_topic.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Skipping camera '%s' because its image_topic or camera_info_topic is empty.",
        camera_name.c_str());
      continue;
    }

    add_camera(camera_name, image_topic, camera_info_topic, frame_override);
  }

  if (cameras_.empty()) {
    throw std::runtime_error("No valid cameras were configured for visual odometry.");
  }
}

void MonocularVisualOdometryNode::cameraInfoCallback(
  const std::shared_ptr<CameraState> & camera,
  const sensor_msgs::msg::CameraInfo::SharedPtr message)
{
  if (message->k.size() != 9U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Ignoring camera info for %s without a 3x3 intrinsic matrix.",
      camera->name.c_str());
    return;
  }

  camera->calibration.camera_matrix =
    cv::Mat(3, 3, CV_64F, const_cast<double *>(message->k.data())).clone();
  camera->calibration.distortion_coefficients =
    cv::Mat(
    1,
    static_cast<int>(message->d.size()),
    CV_64F,
    const_cast<double *>(message->d.data())).clone();
  camera->calibration.distortion_model = message->distortion_model;

  if (camera->calibration.distortion_coefficients.empty()) {
    camera->calibration.distortion_coefficients = cv::Mat::zeros(1, 4, CV_64F);
  }
}

void MonocularVisualOdometryNode::wheelOdometryCallback(
  const nav_msgs::msg::Odometry::SharedPtr message)
{
  wheel_odom_history_.push_back(*message);

  const rclcpp::Time newest_stamp(message->header.stamp);
  while (!wheel_odom_history_.empty()) {
    const rclcpp::Time oldest_stamp(wheel_odom_history_.front().header.stamp);
    if ((newest_stamp - oldest_stamp).seconds() <= wheel_history_seconds_) {
      break;
    }
    wheel_odom_history_.pop_front();
  }
}

void MonocularVisualOdometryNode::imageCallback(
  const std::shared_ptr<CameraState> & camera,
  const sensor_msgs::msg::Image::SharedPtr message)
{
  if (!camera->calibration.ready()) {
    publishStatus(camera->name + ":waiting_for_camera_info");
    return;
  }

  const rclcpp::Time stamp(message->header.stamp);
  if (camera->previous_image_stamp.nanoseconds() != 0 &&
    (stamp - camera->previous_image_stamp).seconds() < min_seconds_between_keyframes_)
  {
    return;
  }

  auto scale_reference = lookupWheelOdom(stamp);
  if (!scale_reference.has_value()) {
    publishStatus(camera->name + ":waiting_for_wheel_odom");
    return;
  }

  cv::Mat gray_image;
  try {
    gray_image = cv_bridge::toCvCopy(*message, sensor_msgs::image_encodings::MONO8)->image;
  } catch (const cv_bridge::Exception & error) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Failed to convert image for camera %s: %s",
      camera->name.c_str(),
      error.what());
    publishStatus(camera->name + ":image_conversion_failed");
    return;
  }

  const std::string camera_frame =
    camera->frame_override.empty() ? message->header.frame_id : camera->frame_override;
  if (!camera->extrinsics_resolved && !resolveCameraExtrinsics(camera, camera_frame)) {
    publishStatus(camera->name + ":waiting_for_camera_extrinsics");
    return;
  }

  if (!camera->have_previous_scale_reference || camera->previous_gray_image.empty()) {
    initializeFromFrame(camera, gray_image, scale_reference.value(), stamp);
    publishStatus(camera->name + ":initialized_first_frame");
    return;
  }

  TrackingResult result = estimateMotion(camera, gray_image, scale_reference.value(), stamp);
  if (!result.success) {
    initializeFromFrame(camera, gray_image, scale_reference.value(), stamp);
    publishStatus(camera->name + ":" + result.reason);
    return;
  }

  camera->previous_gray_image = gray_image.clone();
  camera->previous_points = result.tracked_points;
  if (static_cast<int>(camera->previous_points.size()) < min_tracked_features_) {
    camera->previous_points = detectFeatures(camera->previous_gray_image);
  }
  camera->previous_image_stamp = stamp;
  camera->previous_scale_reference = scale_reference.value();
  camera->have_previous_scale_reference = true;

  queueEstimate(camera->name, result, stamp);
  flushPendingEstimates(stamp, cameras_.size() == 1U);

  std::ostringstream status;
  status << "tracking=true camera=" << camera->name
         << " tracked=" << result.tracked_features
         << " inliers=" << result.inliers
         << " scale=" << result.metric_translation_scale;
  publishStatus(status.str());
}

void MonocularVisualOdometryNode::initializeFromFrame(
  const std::shared_ptr<CameraState> & camera,
  const cv::Mat & gray_image,
  const nav_msgs::msg::Odometry & scale_reference,
  const rclcpp::Time & stamp)
{
  camera->previous_gray_image = gray_image.clone();
  camera->previous_points = detectFeatures(camera->previous_gray_image);
  camera->previous_image_stamp = stamp;
  camera->previous_scale_reference = scale_reference;
  camera->have_previous_scale_reference = true;

  if (!have_pose_estimate_) {
    odom_to_base_ = poseToTransform(scale_reference.pose.pose);
    have_pose_estimate_ = true;
  }
}

MonocularVisualOdometryNode::TrackingResult MonocularVisualOdometryNode::estimateMotion(
  const std::shared_ptr<CameraState> & camera,
  const cv::Mat & current_gray_image,
  const nav_msgs::msg::Odometry & current_scale_reference,
  const rclcpp::Time & stamp)
{
  TrackingResult result;

  if (camera->previous_points.empty()) {
    result.reason = "reinitializing_no_features";
    return result;
  }

  std::vector<cv::Point2f> tracked_points;
  std::vector<unsigned char> tracking_status;
  std::vector<float> tracking_errors;
  cv::calcOpticalFlowPyrLK(
    camera->previous_gray_image,
    current_gray_image,
    camera->previous_points,
    tracked_points,
    tracking_status,
    tracking_errors,
    cv::Size(optical_flow_window_px_, optical_flow_window_px_),
    optical_flow_max_pyramid_level_,
    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));

  std::vector<cv::Point2f> previous_valid_points;
  std::vector<cv::Point2f> current_valid_points;
  previous_valid_points.reserve(camera->previous_points.size());
  current_valid_points.reserve(camera->previous_points.size());
  for (std::size_t index = 0; index < camera->previous_points.size(); ++index) {
    if (!tracking_status[index] || tracking_errors[index] > optical_flow_max_error_) {
      continue;
    }
    previous_valid_points.push_back(camera->previous_points[index]);
    current_valid_points.push_back(tracked_points[index]);
  }

  result.tracked_features = static_cast<int>(current_valid_points.size());
  if (result.tracked_features < min_inliers_) {
    result.reason = "reinitializing_tracking_lost";
    return result;
  }

  const std::vector<cv::Point2f> previous_undistorted =
    undistortPoints(camera->calibration, previous_valid_points);
  const std::vector<cv::Point2f> current_undistorted =
    undistortPoints(camera->calibration, current_valid_points);

  cv::Mat inlier_mask;
  const cv::Mat essential_matrix = cv::findEssentialMat(
    previous_undistorted,
    current_undistorted,
    camera->calibration.camera_matrix,
    cv::RANSAC,
    ransac_confidence_,
    ransac_reprojection_threshold_px_,
    inlier_mask);
  if (essential_matrix.empty()) {
    result.reason = "reinitializing_no_essential_matrix";
    return result;
  }

  cv::Mat rotation_matrix;
  cv::Mat translation_vector;
  result.inliers = cv::recoverPose(
    essential_matrix,
    previous_undistorted,
    current_undistorted,
    camera->calibration.camera_matrix,
    rotation_matrix,
    translation_vector,
    inlier_mask);
  if (result.inliers < min_inliers_) {
    result.reason = "reinitializing_low_inlier_count";
    return result;
  }

  std::vector<cv::Point2f> inlier_tracked_points;
  inlier_tracked_points.reserve(current_valid_points.size());
  for (int row = 0; row < inlier_mask.rows; ++row) {
    if (inlier_mask.at<unsigned char>(row, 0U) == 0U) {
      continue;
    }
    inlier_tracked_points.push_back(current_valid_points[static_cast<std::size_t>(row)]);
  }
  result.tracked_points = std::move(inlier_tracked_points);

  result.metric_translation_scale = planarDistance(
    camera->previous_scale_reference.pose.pose.position,
    current_scale_reference.pose.pose.position);
  if (result.metric_translation_scale < min_scale_translation_meters_) {
    result.metric_translation_scale = 0.0;
  }

  cv::Mat scaled_translation = translation_vector.clone();
  const double translation_norm = cv::norm(translation_vector);
  if (translation_norm > 1.0e-6) {
    scaled_translation *= result.metric_translation_scale / translation_norm;
  } else {
    scaled_translation = cv::Mat::zeros(3, 1, CV_64F);
  }

  tf2::Matrix3x3 tf_rotation(
    rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
    rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
    rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
  tf2::Quaternion camera_delta_quaternion;
  tf_rotation.getRotation(camera_delta_quaternion);

  tf2::Transform delta_camera(tf2::Transform::getIdentity());
  delta_camera.setRotation(camera_delta_quaternion);
  delta_camera.setOrigin(
    tf2::Vector3(
      scaled_translation.at<double>(0, 0),
      scaled_translation.at<double>(1, 0),
      scaled_translation.at<double>(2, 0)));

  const tf2::Transform camera_to_base = camera->base_to_camera.inverse();
  tf2::Transform delta_base = camera->base_to_camera * delta_camera * camera_to_base;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(delta_base.getRotation()).getRPY(roll, pitch, yaw);
  result.delta_yaw_rad = normalizeAngle(yaw);

  if (force_2d_) {
    tf2::Transform planar_delta(tf2::Transform::getIdentity());
    planar_delta.setOrigin(
      tf2::Vector3(
        delta_base.getOrigin().x(),
        delta_base.getOrigin().y(),
        0.0));
    tf2::Quaternion planar_quaternion;
    planar_quaternion.setRPY(0.0, 0.0, result.delta_yaw_rad);
    planar_delta.setRotation(planar_quaternion);
    delta_base = planar_delta;
  }

  result.dt_seconds = std::max((stamp - camera->previous_image_stamp).seconds(), 1.0e-3);
  result.delta_base = delta_base;
  result.success = true;
  return result;
}

std::vector<cv::Point2f> MonocularVisualOdometryNode::detectFeatures(const cv::Mat & gray_image) const
{
  std::vector<cv::Point2f> features;
  cv::goodFeaturesToTrack(
    gray_image,
    features,
    max_features_,
    feature_quality_level_,
    feature_min_distance_px_);
  if (features.empty()) {
    return features;
  }

  cv::cornerSubPix(
    gray_image,
    features,
    cv::Size(corner_refinement_window_px_, corner_refinement_window_px_),
    cv::Size(-1, -1),
    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 0.01));
  return features;
}

std::vector<cv::Point2f> MonocularVisualOdometryNode::undistortPoints(
  const CameraCalibration & calibration,
  const std::vector<cv::Point2f> & points) const
{
  if (points.empty()) {
    return {};
  }

  std::vector<cv::Point2f> undistorted_points;
  if (calibration.distortion_model == "equidistant") {
    cv::fisheye::undistortPoints(
      points,
      undistorted_points,
      calibration.camera_matrix,
      calibration.distortion_coefficients,
      cv::noArray(),
      calibration.camera_matrix);
  } else {
    cv::undistortPoints(
      points,
      undistorted_points,
      calibration.camera_matrix,
      calibration.distortion_coefficients,
      cv::noArray(),
      calibration.camera_matrix);
  }
  return undistorted_points;
}

std::optional<nav_msgs::msg::Odometry> MonocularVisualOdometryNode::lookupWheelOdom(
  const rclcpp::Time & stamp) const
{
  if (wheel_odom_history_.empty()) {
    return std::nullopt;
  }

  const auto tolerance = rclcpp::Duration::from_seconds(wheel_lookup_tolerance_seconds_);
  const nav_msgs::msg::Odometry * best_match = nullptr;
  auto best_delta = rclcpp::Duration::from_seconds(std::numeric_limits<double>::max());

  for (const auto & message : wheel_odom_history_) {
    const rclcpp::Time candidate_stamp(message.header.stamp);
    const rclcpp::Duration delta =
      candidate_stamp > stamp ? (candidate_stamp - stamp) : (stamp - candidate_stamp);
    if (delta < best_delta) {
      best_delta = delta;
      best_match = &message;
    }
  }

  if (best_match == nullptr || best_delta > tolerance) {
    return std::nullopt;
  }
  return *best_match;
}

bool MonocularVisualOdometryNode::resolveCameraExtrinsics(
  const std::shared_ptr<CameraState> & camera,
  const std::string & camera_frame)
{
  if (camera_frame.empty()) {
    return false;
  }

  try {
    const geometry_msgs::msg::TransformStamped transform =
      tf_buffer_.lookupTransform(base_frame_, camera_frame, tf2::TimePointZero);
    tf2::fromMsg(transform.transform, camera->base_to_camera);
    camera->extrinsics_resolved = true;
    RCLCPP_INFO(
      get_logger(),
      "Resolved visual odometry camera extrinsics for %s from %s to %s",
      camera->name.c_str(),
      base_frame_.c_str(),
      camera_frame.c_str());
    return true;
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for transform from %s to %s for camera %s: %s",
      base_frame_.c_str(),
      camera_frame.c_str(),
      camera->name.c_str(),
      error.what());
    return false;
  }
}

void MonocularVisualOdometryNode::queueEstimate(
  const std::string & camera_name,
  const TrackingResult & result,
  const rclcpp::Time & stamp)
{
  PendingEstimate estimate;
  estimate.camera_name = camera_name;
  estimate.stamp = stamp;
  estimate.result = result;

  auto insertion_point = pending_estimates_.begin();
  while (
    insertion_point != pending_estimates_.end() &&
    insertion_point->stamp.nanoseconds() <= stamp.nanoseconds())
  {
    ++insertion_point;
  }
  pending_estimates_.insert(insertion_point, std::move(estimate));
}

void MonocularVisualOdometryNode::flushPendingEstimates(
  const rclcpp::Time & reference_stamp,
  bool force_flush)
{
  const bool multi_camera_mode = cameras_.size() > 1U;

  while (!pending_estimates_.empty()) {
    const rclcpp::Time anchor_stamp = pending_estimates_.front().stamp;
    const double anchor_age_seconds =
      reference_stamp.nanoseconds() >= anchor_stamp.nanoseconds() ?
      (reference_stamp - anchor_stamp).seconds() :
      0.0;

    if (!force_flush && multi_camera_mode && anchor_age_seconds < camera_fusion_tolerance_seconds_) {
      break;
    }

    std::vector<PendingEstimate> group;
    while (!pending_estimates_.empty()) {
      const double stamp_delta =
        std::abs((pending_estimates_.front().stamp - anchor_stamp).seconds());
      if (stamp_delta > camera_fusion_tolerance_seconds_) {
        break;
      }
      group.push_back(std::move(pending_estimates_.front()));
      pending_estimates_.pop_front();
    }

    if (group.size() < static_cast<std::size_t>(std::max(1, min_cameras_per_estimate_))) {
      std::ostringstream status;
      status << "skipping_group insufficient_cameras=" << group.size();
      publishStatus(status.str());
      continue;
    }

    TrackingResult fused_result = fuseTrackingResults(group);
    if (!fused_result.success) {
      publishStatus("skipping_group fusion_failed");
      continue;
    }

    rclcpp::Time publish_stamp = group.front().stamp;
    std::ostringstream sources;
    for (std::size_t index = 0; index < group.size(); ++index) {
      if (group[index].stamp > publish_stamp) {
        publish_stamp = group[index].stamp;
      }
      if (index > 0U) {
        sources << ",";
      }
      sources << group[index].camera_name;
    }

    publishOdometry(
      fused_result,
      publish_stamp,
      std::max(fused_result.dt_seconds, 1.0e-3));

    std::ostringstream status;
    status << "tracking=true cameras=" << group.size()
           << " sources=" << sources.str()
           << " inliers=" << fused_result.inliers
           << " scale=" << fused_result.metric_translation_scale;
    publishStatus(status.str());
  }
}

MonocularVisualOdometryNode::TrackingResult MonocularVisualOdometryNode::fuseTrackingResults(
  const std::vector<PendingEstimate> & estimates) const
{
  TrackingResult fused_result;
  if (estimates.empty()) {
    return fused_result;
  }

  tf2::Vector3 translation_sum(0.0, 0.0, 0.0);
  double quaternion_x_sum = 0.0;
  double quaternion_y_sum = 0.0;
  double quaternion_z_sum = 0.0;
  double quaternion_w_sum = 0.0;
  double weight_sum = 0.0;
  double scale_sum = 0.0;
  double dt_sum = 0.0;
  bool have_reference_rotation = false;
  tf2::Quaternion reference_rotation;

  for (const auto & estimate : estimates) {
    double weight = trackingConfidence(estimate.result);
    if (weight <= 0.0) {
      weight = 0.1;
    }

    translation_sum += estimate.result.delta_base.getOrigin() * weight;

    tf2::Quaternion rotation = estimate.result.delta_base.getRotation();
    if (!have_reference_rotation) {
      reference_rotation = rotation;
      have_reference_rotation = true;
    }
    const double dot =
      (reference_rotation.x() * rotation.x()) +
      (reference_rotation.y() * rotation.y()) +
      (reference_rotation.z() * rotation.z()) +
      (reference_rotation.w() * rotation.w());
    if (dot < 0.0) {
      rotation = tf2::Quaternion(-rotation.x(), -rotation.y(), -rotation.z(), -rotation.w());
    }

    quaternion_x_sum += weight * rotation.x();
    quaternion_y_sum += weight * rotation.y();
    quaternion_z_sum += weight * rotation.z();
    quaternion_w_sum += weight * rotation.w();

    scale_sum += weight * estimate.result.metric_translation_scale;
    dt_sum += weight * estimate.result.dt_seconds;
    fused_result.tracked_features += estimate.result.tracked_features;
    fused_result.inliers += estimate.result.inliers;
    weight_sum += weight;
  }

  if (weight_sum <= 1.0e-6) {
    return fused_result;
  }

  tf2::Vector3 fused_translation = translation_sum / weight_sum;
  tf2::Quaternion fused_rotation(
    quaternion_x_sum / weight_sum,
    quaternion_y_sum / weight_sum,
    quaternion_z_sum / weight_sum,
    quaternion_w_sum / weight_sum);
  if (fused_rotation.length2() <= 1.0e-12) {
    fused_rotation.setRPY(0.0, 0.0, 0.0);
  } else {
    fused_rotation.normalize();
  }

  tf2::Transform fused_delta(tf2::Transform::getIdentity());
  fused_delta.setOrigin(fused_translation);
  fused_delta.setRotation(fused_rotation);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(fused_rotation).getRPY(roll, pitch, yaw);
  fused_result.delta_yaw_rad = normalizeAngle(yaw);

  if (force_2d_) {
    tf2::Transform planar_delta(tf2::Transform::getIdentity());
    planar_delta.setOrigin(tf2::Vector3(fused_translation.x(), fused_translation.y(), 0.0));
    tf2::Quaternion planar_quaternion;
    planar_quaternion.setRPY(0.0, 0.0, fused_result.delta_yaw_rad);
    planar_delta.setRotation(planar_quaternion);
    fused_delta = planar_delta;
  }

  fused_result.metric_translation_scale = scale_sum / weight_sum;
  fused_result.dt_seconds = dt_sum / weight_sum;
  fused_result.delta_base = fused_delta;
  fused_result.success = true;
  return fused_result;
}

void MonocularVisualOdometryNode::publishOdometry(
  const TrackingResult & result,
  const rclcpp::Time & stamp,
  double dt_seconds)
{
  odom_to_base_ = odom_to_base_ * result.delta_base;

  nav_msgs::msg::Odometry odometry_message;
  odometry_message.header.stamp = stamp;
  odometry_message.header.frame_id = odom_frame_;
  odometry_message.child_frame_id = base_frame_;
  odometry_message.pose.pose = transformToPose(odom_to_base_);
  odometry_message.pose.covariance = makePoseCovariance(result);

  odometry_message.twist.twist.linear.x = result.delta_base.getOrigin().x() / dt_seconds;
  odometry_message.twist.twist.linear.y = result.delta_base.getOrigin().y() / dt_seconds;
  odometry_message.twist.twist.linear.z = result.delta_base.getOrigin().z() / dt_seconds;
  odometry_message.twist.twist.angular.z = result.delta_yaw_rad / dt_seconds;
  odometry_message.twist.covariance = makeTwistCovariance(result);
  odom_publisher_->publish(odometry_message);

  if (!publish_tf_ || !tf_broadcaster_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_message;
  transform_message.header = odometry_message.header;
  transform_message.child_frame_id = base_frame_;
  transform_message.transform.translation.x = odometry_message.pose.pose.position.x;
  transform_message.transform.translation.y = odometry_message.pose.pose.position.y;
  transform_message.transform.translation.z = odometry_message.pose.pose.position.z;
  transform_message.transform.rotation = odometry_message.pose.pose.orientation;
  tf_broadcaster_->sendTransform(transform_message);
}

void MonocularVisualOdometryNode::publishStatus(const std::string & status_message) const
{
  std_msgs::msg::String message;
  message.data = status_message;
  status_publisher_->publish(message);
}

double MonocularVisualOdometryNode::trackingConfidence(const TrackingResult & result) const
{
  if (result.tracked_features <= 0) {
    return 0.0;
  }
  return std::clamp(
    static_cast<double>(result.inliers) / static_cast<double>(result.tracked_features),
    0.0,
    1.0);
}

double MonocularVisualOdometryNode::normalizeAngle(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double MonocularVisualOdometryNode::yawFromQuaternion(
  const geometry_msgs::msg::Quaternion & quaternion)
{
  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(quaternion, tf_quaternion);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

double MonocularVisualOdometryNode::planarDistance(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

tf2::Transform MonocularVisualOdometryNode::poseToTransform(
  const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  tf2::Transform transform(rotation, tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  return transform;
}

geometry_msgs::msg::Pose MonocularVisualOdometryNode::transformToPose(
  const tf2::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();
  pose.orientation = tf2::toMsg(transform.getRotation());
  return pose;
}

std::array<double, 36> MonocularVisualOdometryNode::makePoseCovariance(
  const TrackingResult & result) const
{
  std::array<double, 36> covariance{};
  covariance.fill(0.0);

  const double bounded_confidence = std::clamp(trackingConfidence(result), 0.1, 1.0);
  const double position_sigma = std::clamp(
    pose_sigma_floor_m_ / bounded_confidence,
    pose_sigma_floor_m_,
    pose_sigma_ceiling_m_);
  const double yaw_sigma = std::clamp(
    yaw_sigma_floor_rad_ / bounded_confidence,
    yaw_sigma_floor_rad_,
    yaw_sigma_ceiling_rad_);

  covariance[0] = position_sigma * position_sigma;
  covariance[7] = position_sigma * position_sigma;
  covariance[14] = kLargeVariance;
  covariance[21] = kLargeVariance;
  covariance[28] = kLargeVariance;
  covariance[35] = yaw_sigma * yaw_sigma;
  return covariance;
}

std::array<double, 36> MonocularVisualOdometryNode::makeTwistCovariance(
  const TrackingResult & result) const
{
  std::array<double, 36> covariance{};
  covariance.fill(0.0);

  const double bounded_confidence = std::clamp(trackingConfidence(result), 0.1, 1.0);
  const double velocity_sigma = std::clamp(
    pose_sigma_floor_m_ / bounded_confidence,
    pose_sigma_floor_m_,
    pose_sigma_ceiling_m_);
  const double yaw_rate_sigma = std::clamp(
    yaw_sigma_floor_rad_ / bounded_confidence,
    yaw_sigma_floor_rad_,
    yaw_sigma_ceiling_rad_);

  covariance[0] = velocity_sigma * velocity_sigma;
  covariance[7] = velocity_sigma * velocity_sigma;
  covariance[14] = kLargeVariance;
  covariance[21] = kLargeVariance;
  covariance[28] = kLargeVariance;
  covariance[35] = yaw_rate_sigma * yaw_rate_sigma;
  return covariance;
}

}  // namespace amr_sweeper_visual_odometry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sweeper_visual_odometry::MonocularVisualOdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
