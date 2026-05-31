// Copyright (c) 2026 O-Robotics

#include "visual_odometry_node.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

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

cv::Mat convertRosImageToMono8(const sensor_msgs::msg::Image & message)
{
  const auto & encoding = message.encoding;

  auto require_step = [&message](std::size_t min_step_bytes) {
      if (message.step < min_step_bytes) {
        throw std::runtime_error("Image step is smaller than expected for its encoding.");
      }
    };

  if (encoding == sensor_msgs::image_encodings::MONO8) {
    require_step(message.width);
    cv::Mat mono(
      static_cast<int>(message.height),
      static_cast<int>(message.width),
      CV_8UC1,
      const_cast<unsigned char *>(message.data.data()),
      message.step);
    return mono.clone();
  }

  if (encoding == sensor_msgs::image_encodings::MONO16) {
    require_step(message.width * sizeof(std::uint16_t));
    cv::Mat mono16(
      static_cast<int>(message.height),
      static_cast<int>(message.width),
      CV_16UC1,
      const_cast<unsigned char *>(message.data.data()),
      message.step);
    cv::Mat mono8;
    mono16.convertTo(mono8, CV_8UC1, 1.0 / 256.0);
    return mono8;
  }

  auto convert_color = [&message, &require_step](int cv_type, int conversion_code) {
      const int channels = CV_MAT_CN(cv_type);
      require_step(static_cast<std::size_t>(message.width) * static_cast<std::size_t>(channels));
      cv::Mat color(
        static_cast<int>(message.height),
        static_cast<int>(message.width),
        cv_type,
        const_cast<unsigned char *>(message.data.data()),
        message.step);
      cv::Mat mono8;
      cv::cvtColor(color, mono8, conversion_code);
      return mono8;
    };

  if (encoding == sensor_msgs::image_encodings::BGR8) {
    return convert_color(CV_8UC3, cv::COLOR_BGR2GRAY);
  }
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    return convert_color(CV_8UC3, cv::COLOR_RGB2GRAY);
  }
  if (encoding == sensor_msgs::image_encodings::BGRA8) {
    return convert_color(CV_8UC4, cv::COLOR_BGRA2GRAY);
  }
  if (encoding == sensor_msgs::image_encodings::RGBA8) {
    return convert_color(CV_8UC4, cv::COLOR_RGBA2GRAY);
  }

  throw std::runtime_error("Unsupported image encoding for visual odometry: " + encoding);
}

}  // namespace

VisualOdometryNode::VisualOdometryNode(const rclcpp::NodeOptions & options)
: Node("visual_odometry_node", options),
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
  imu_topic_ = declare_parameter("motion_reference.imu_topic", std::string("imu/data_raw"));
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
  motion_reference_lookup_tolerance_seconds_ = declare_parameter(
    "motion_reference.lookup_tolerance_seconds", 0.25);
  motion_reference_history_seconds_ = declare_parameter(
    "motion_reference.history_seconds", 5.0);
  min_scale_translation_meters_ = declare_parameter("min_scale_translation_meters", 0.005);
  camera_fusion_tolerance_seconds_ = declare_parameter("camera_fusion_tolerance_seconds", 0.03);
  stereo_match_max_error_ = declare_parameter("stereo_match_max_error", 20.0);
  stereo_min_disparity_px_ = declare_parameter("stereo_min_disparity_px", 1.0);
  stereo_max_reprojection_error_m_ = declare_parameter("stereo_max_reprojection_error_m", 0.20);
  pose_sigma_floor_m_ = declare_parameter("pose_sigma_floor_m", 0.03);
  pose_sigma_ceiling_m_ = declare_parameter("pose_sigma_ceiling_m", 0.50);
  yaw_sigma_floor_rad_ = declare_parameter("yaw_sigma_floor_rad", 0.02);
  yaw_sigma_ceiling_rad_ = declare_parameter("yaw_sigma_ceiling_rad", 0.35);
  min_cameras_per_estimate_ = declare_parameter("min_cameras_per_estimate", 1);
  use_imu_yaw_ = declare_parameter("motion_reference.use_imu_yaw", true);
  use_wheel_scale_ = declare_parameter("motion_reference.use_wheel_scale", true);
  imu_max_dt_seconds_ = declare_parameter("motion_reference.imu_max_dt_seconds", 0.1);
  imu_planar_accel_deadband_mps2_ = declare_parameter(
    "motion_reference.imu_planar_accel_deadband_mps2", 0.15);
  imu_velocity_damping_per_second_ = declare_parameter(
    "motion_reference.imu_velocity_damping_per_second", 1.5);
  imu_stationary_angular_velocity_threshold_rad_s_ = declare_parameter(
    "motion_reference.imu_stationary_angular_velocity_threshold_rad_s", 0.05);
  fusion_method_ = declare_parameter("fusion_method", std::string("ekf"));

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  configureCameras();
  configureStereoPairs();

  if (use_wheel_scale_) {
    wheel_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic_,
      rclcpp::QoS(50),
      std::bind(&VisualOdometryNode::wheelOdometryCallback, this, std::placeholders::_1));
  }

  if (use_imu_yaw_) {
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&VisualOdometryNode::imuCallback, this, std::placeholders::_1));
  }

  if (!use_wheel_scale_ && !use_imu_yaw_) {
    throw std::runtime_error(
            "Visual odometry requires at least one enabled motion reference source.");
  }

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
    "Monocular visual odometry configured with cameras=[%s] wheel_scale=%s imu_yaw=%s output=%s",
    camera_summary.str().c_str(),
    use_wheel_scale_ ? "true" : "false",
    use_imu_yaw_ ? "true" : "false",
    odom_topic_.c_str());
}

void VisualOdometryNode::configureCameras()
{
  auto configured_camera_names =
    declare_parameter("camera_names", std::vector<std::string>{});

  if (configured_camera_names.empty()) {
    configured_camera_names = {
      "tool_camera",
      "depth_camera",
      "front_left_camera",
      "front_right_camera",
      "rear_left_camera",
      "rear_right_camera",
    };
  }

  auto add_camera = [this](
    const std::string & name,
    const std::string & image_topic,
    const std::string & camera_info_topic,
    const std::string & frame_override,
    double fusion_weight) {
      auto camera = std::make_shared<CameraState>(name);
      camera->image_topic = image_topic;
      camera->camera_info_topic = camera_info_topic;
      camera->frame_override = frame_override;
      camera->fusion_weight = fusion_weight;

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

  for (const auto & camera_name : configured_camera_names) {
    const std::string prefix = "cameras." + camera_name + ".";
    const bool enabled = declare_parameter(
      prefix + "enabled",
      camera_name == "tool_camera" || camera_name == "depth_camera");
    if (!enabled) {
      continue;
    }
    const std::string image_topic = declare_parameter(prefix + "image_topic", std::string(""));
    const std::string camera_info_topic =
      declare_parameter(prefix + "camera_info_topic", std::string(""));
    const std::string frame_override =
      declare_parameter(prefix + "camera_frame", std::string(""));
    const double fusion_weight =
      declare_parameter(prefix + "fusion_weight", 1.0);

    if (image_topic.empty() || camera_info_topic.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Skipping camera '%s' because its image_topic or camera_info_topic is empty.",
        camera_name.c_str());
      continue;
    }

    add_camera(camera_name, image_topic, camera_info_topic, frame_override, fusion_weight);
  }

  if (cameras_.empty()) {
    throw std::runtime_error("No valid cameras were configured for visual odometry.");
  }
}

void VisualOdometryNode::configureStereoPairs()
{
  const auto configured_stereo_pair_names =
    declare_parameter("stereo_pairs", std::vector<std::string>{});

  for (const auto & stereo_pair_name : configured_stereo_pair_names) {
    const std::string prefix = "stereo_pairs." + stereo_pair_name + ".";
    StereoPairState stereo_pair;
    stereo_pair.name = stereo_pair_name;
    stereo_pair.left_camera_name = declare_parameter(prefix + "left_camera", std::string(""));
    stereo_pair.right_camera_name = declare_parameter(prefix + "right_camera", std::string(""));
    stereo_pair.fusion_weight = declare_parameter(prefix + "fusion_weight", 1.0);
    stereo_pair.sync_tolerance_seconds = declare_parameter(prefix + "sync_tolerance_seconds", 0.02);

    if (stereo_pair.left_camera_name.empty() || stereo_pair.right_camera_name.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Skipping stereo pair '%s' because left_camera or right_camera is empty.",
        stereo_pair.name.c_str());
      continue;
    }

    auto find_camera = [this](const std::string & camera_name) -> std::shared_ptr<CameraState> {
        for (const auto & camera : cameras_) {
          if (camera->name == camera_name) {
            return camera;
          }
        }
        return nullptr;
      };

    stereo_pair.left_camera = find_camera(stereo_pair.left_camera_name);
    stereo_pair.right_camera = find_camera(stereo_pair.right_camera_name);
    if (!stereo_pair.left_camera || !stereo_pair.right_camera) {
      RCLCPP_WARN(
        get_logger(),
        "Skipping stereo pair '%s' because one or both cameras were not configured.",
        stereo_pair.name.c_str());
      continue;
    }

    stereo_pairs_.push_back(std::move(stereo_pair));
  }
}

void VisualOdometryNode::cameraInfoCallback(
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

void VisualOdometryNode::wheelOdometryCallback(
  const nav_msgs::msg::Odometry::SharedPtr message)
{
  MotionReferenceSample sample;
  sample.stamp = rclcpp::Time(message->header.stamp);
  sample.planar_position = tf2::Vector3(
    message->pose.pose.position.x,
    message->pose.pose.position.y,
    0.0);
  sample.yaw_rad = std::nullopt;

  wheel_motion_reference_history_.push_back(sample);

  const rclcpp::Time newest_stamp = sample.stamp;
  while (!wheel_motion_reference_history_.empty()) {
    const rclcpp::Time oldest_stamp = wheel_motion_reference_history_.front().stamp;
    if ((newest_stamp - oldest_stamp).seconds() <= motion_reference_history_seconds_) {
      break;
    }
    wheel_motion_reference_history_.pop_front();
  }
}

void VisualOdometryNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr message)
{
  const rclcpp::Time stamp(message->header.stamp);
  if (stamp.nanoseconds() == 0) {
    return;
  }

  const double yaw_rad = yawFromQuaternion(message->orientation);

  if (!have_previous_imu_sample_) {
    previous_imu_stamp_ = stamp;
    have_previous_imu_sample_ = true;

    MotionReferenceSample sample;
    sample.stamp = stamp;
    sample.planar_position = std::nullopt;
    sample.yaw_rad = yaw_rad;
    imu_motion_reference_history_.push_back(sample);
    return;
  }

  double dt_seconds = (stamp - previous_imu_stamp_).seconds();
  previous_imu_stamp_ = stamp;
  if (dt_seconds <= 0.0) {
    return;
  }
  dt_seconds = std::min(dt_seconds, imu_max_dt_seconds_);

  tf2::Quaternion orientation;
  tf2::fromMsg(message->orientation, orientation);
  const tf2::Vector3 accel_body(
    message->linear_acceleration.x,
    message->linear_acceleration.y,
    message->linear_acceleration.z);
  tf2::Vector3 accel_world = tf2::quatRotate(orientation, accel_body);
  accel_world -= tf2::Vector3(0.0, 0.0, 9.8);

  tf2::Vector3 planar_accel(accel_world.x(), accel_world.y(), 0.0);
  if (planar_accel.length() < imu_planar_accel_deadband_mps2_) {
    planar_accel = tf2::Vector3(0.0, 0.0, 0.0);
  }

  if (std::abs(message->angular_velocity.z) < imu_stationary_angular_velocity_threshold_rad_s_ &&
    planar_accel.length2() == 0.0)
  {
    imu_planar_velocity_.setValue(0.0, 0.0, 0.0);
  } else {
    imu_planar_velocity_ += planar_accel * dt_seconds;
    const double damping = std::exp(-imu_velocity_damping_per_second_ * dt_seconds);
    imu_planar_velocity_ *= damping;
  }

  imu_planar_position_ += imu_planar_velocity_ * dt_seconds;

  MotionReferenceSample sample;
  sample.stamp = stamp;
  sample.planar_position = std::nullopt;
  sample.yaw_rad = yaw_rad;
  imu_motion_reference_history_.push_back(sample);

  while (!imu_motion_reference_history_.empty()) {
    if ((stamp - imu_motion_reference_history_.front().stamp).seconds() <=
      motion_reference_history_seconds_)
    {
      break;
    }
    imu_motion_reference_history_.pop_front();
  }
}

void VisualOdometryNode::imageCallback(
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

  auto motion_reference = lookupMotionReference(stamp);
  if (!motion_reference.has_value()) {
    publishStatus(camera->name + ":waiting_for_motion_reference");
    return;
  }

  cv::Mat gray_image;
  try {
    gray_image = convertRosImageToMono8(*message);
  } catch (const std::exception & error) {
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
  if (!resolveCameraExtrinsics(camera, camera_frame, stamp)) {
    publishStatus(camera->name + ":waiting_for_camera_extrinsics");
    return;
  }

  camera->latest_gray_image = gray_image.clone();
  camera->latest_image_stamp = stamp;
  camera->latest_frame_id = camera_frame;

  if (StereoPairState * stereo_pair = findStereoPairByCameraName(camera->name)) {
    if (stereo_pair->right_camera.get() == camera.get()) {
      publishStatus(stereo_pair->name + ":right_frame_cached");
      return;
    }

    if (!stereo_pair->right_camera ||
      stereo_pair->right_camera->latest_gray_image.empty() ||
      stereo_pair->right_camera->latest_image_stamp.nanoseconds() == 0)
    {
      publishStatus(stereo_pair->name + ":waiting_for_right_frame");
      return;
    }

    const double right_delta_seconds =
      std::abs((stereo_pair->right_camera->latest_image_stamp - stamp).seconds());
    if (right_delta_seconds > stereo_pair->sync_tolerance_seconds) {
      publishStatus(stereo_pair->name + ":waiting_for_synced_right_frame");
      return;
    }

    if (!resolveCameraExtrinsics(
        stereo_pair->right_camera,
        stereo_pair->right_camera->frame_override.empty() ?
        stereo_pair->right_camera->latest_frame_id :
        stereo_pair->right_camera->frame_override,
        stereo_pair->right_camera->latest_image_stamp))
    {
      publishStatus(stereo_pair->name + ":waiting_for_right_camera_extrinsics");
      return;
    }

    if (!stereo_pair->previous_frame.has_value()) {
      initializeStereoFrame(
        *stereo_pair,
        gray_image,
        stereo_pair->right_camera->latest_gray_image,
        motion_reference.value(),
        stamp);
      publishStatus(stereo_pair->name + ":initialized_first_frame");
      return;
    }

    TrackingResult stereo_result = estimateStereoMotion(
      *stereo_pair,
      gray_image,
      stereo_pair->right_camera->latest_gray_image,
      motion_reference.value(),
      camera->base_to_camera,
      stereo_pair->right_camera->base_to_camera,
      stamp);
    if (!stereo_result.success) {
      initializeStereoFrame(
        *stereo_pair,
        gray_image,
        stereo_pair->right_camera->latest_gray_image,
        motion_reference.value(),
        stamp);
      publishStatus(stereo_pair->name + ":" + stereo_result.reason);
      return;
    }

    initializeStereoFrame(
      *stereo_pair,
      gray_image,
      stereo_pair->right_camera->latest_gray_image,
      motion_reference.value(),
      stamp);
    queueEstimate(stereo_pair->name, stereo_result, stamp, stereo_pair->fusion_weight);
    flushPendingEstimates(stamp, false);

    std::ostringstream status;
    status << "tracking=true stereo_pair=" << stereo_pair->name
           << " tracked=" << stereo_result.tracked_features
           << " inliers=" << stereo_result.inliers
           << " scale=" << stereo_result.metric_translation_scale;
    publishStatus(status.str());
    return;
  }

  if (!camera->have_previous_motion_reference || camera->previous_gray_image.empty()) {
    initializeFromFrame(camera, gray_image, motion_reference.value(), stamp);
    publishStatus(camera->name + ":initialized_first_frame");
    return;
  }

  TrackingResult result = estimateMotion(camera, gray_image, motion_reference.value(), stamp);
  if (!result.success) {
    initializeFromFrame(camera, gray_image, motion_reference.value(), stamp);
    publishStatus(camera->name + ":" + result.reason);
    return;
  }

  camera->previous_gray_image = gray_image.clone();
  camera->previous_points = result.tracked_points;
  if (static_cast<int>(camera->previous_points.size()) < min_tracked_features_) {
    camera->previous_points = detectFeatures(camera->previous_gray_image);
  }
  camera->previous_image_stamp = stamp;
  if (motion_reference->planar_position.has_value()) {
    camera->previous_motion_reference_position = *motion_reference->planar_position;
  }
  if (motion_reference->yaw_rad.has_value()) {
    camera->previous_motion_reference_yaw_rad = *motion_reference->yaw_rad;
  }
  camera->have_previous_motion_reference = true;

  queueEstimate(camera->name, result, stamp, camera->fusion_weight);
  flushPendingEstimates(stamp, cameras_.size() == 1U);

  std::ostringstream status;
  status << "tracking=true camera=" << camera->name
         << " tracked=" << result.tracked_features
         << " inliers=" << result.inliers
         << " scale=" << result.metric_translation_scale;
  publishStatus(status.str());
}

void VisualOdometryNode::initializeFromFrame(
  const std::shared_ptr<CameraState> & camera,
  const cv::Mat & gray_image,
  const MotionReferenceSample & motion_reference,
  const rclcpp::Time & stamp)
{
  camera->previous_gray_image = gray_image.clone();
  camera->previous_points = detectFeatures(camera->previous_gray_image);
  camera->previous_image_stamp = stamp;
  if (motion_reference.planar_position.has_value()) {
    camera->previous_motion_reference_position = *motion_reference.planar_position;
  }
  if (motion_reference.yaw_rad.has_value()) {
    camera->previous_motion_reference_yaw_rad = *motion_reference.yaw_rad;
  }
  camera->have_previous_motion_reference = true;

  if (!have_pose_estimate_) {
    odom_to_base_.setIdentity();
    have_pose_estimate_ = true;
  }
}

void VisualOdometryNode::initializeStereoFrame(
  StereoPairState & stereo_pair,
  const cv::Mat & left_gray_image,
  const cv::Mat & right_gray_image,
  const MotionReferenceSample & motion_reference,
  const rclcpp::Time & stamp)
{
  StereoFrame frame;
  frame.stamp = stamp;
  frame.left_gray_image = left_gray_image.clone();
  frame.right_gray_image = right_gray_image.clone();
  frame.left_points = detectFeatures(frame.left_gray_image);
  frame.motion_reference = motion_reference;
  frame.base_to_left = stereo_pair.left_camera->base_to_camera;
  frame.base_to_right = stereo_pair.right_camera->base_to_camera;
  stereo_pair.previous_frame = std::move(frame);

  if (!have_pose_estimate_) {
    odom_to_base_.setIdentity();
    have_pose_estimate_ = true;
  }
}

VisualOdometryNode::StereoPairState * VisualOdometryNode::findStereoPairByCameraName(
  const std::string & camera_name)
{
  for (auto & stereo_pair : stereo_pairs_) {
    if (stereo_pair.left_camera_name == camera_name || stereo_pair.right_camera_name == camera_name) {
      return &stereo_pair;
    }
  }
  return nullptr;
}

const VisualOdometryNode::StereoPairState * VisualOdometryNode::findStereoPairByCameraName(
  const std::string & camera_name) const
{
  for (const auto & stereo_pair : stereo_pairs_) {
    if (stereo_pair.left_camera_name == camera_name || stereo_pair.right_camera_name == camera_name) {
      return &stereo_pair;
    }
  }
  return nullptr;
}

VisualOdometryNode::TrackingResult VisualOdometryNode::estimateStereoMotion(
  StereoPairState & stereo_pair,
  const cv::Mat & current_left_gray_image,
  const cv::Mat & current_right_gray_image,
  const MotionReferenceSample & current_motion_reference,
  const tf2::Transform & base_to_left,
  const tf2::Transform & base_to_right,
  const rclcpp::Time & stamp)
{
  TrackingResult result;
  if (!stereo_pair.previous_frame.has_value()) {
    result.reason = "reinitializing_no_previous_stereo_frame";
    return result;
  }

  const StereoFrame & previous_frame = *stereo_pair.previous_frame;
  if (previous_frame.left_points.empty()) {
    result.reason = "reinitializing_no_stereo_features";
    return result;
  }

  std::vector<cv::Point2f> current_left_points;
  std::vector<unsigned char> temporal_status;
  std::vector<float> temporal_errors;
  cv::calcOpticalFlowPyrLK(
    previous_frame.left_gray_image,
    current_left_gray_image,
    previous_frame.left_points,
    current_left_points,
    temporal_status,
    temporal_errors,
    cv::Size(optical_flow_window_px_, optical_flow_window_px_),
    optical_flow_max_pyramid_level_,
    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));

  std::vector<cv::Point2f> previous_left_valid_points;
  std::vector<cv::Point2f> current_left_valid_points;
  previous_left_valid_points.reserve(previous_frame.left_points.size());
  current_left_valid_points.reserve(previous_frame.left_points.size());
  for (std::size_t index = 0; index < previous_frame.left_points.size(); ++index) {
    if (!temporal_status[index] || temporal_errors[index] > optical_flow_max_error_) {
      continue;
    }
    previous_left_valid_points.push_back(previous_frame.left_points[index]);
    current_left_valid_points.push_back(current_left_points[index]);
  }

  result.tracked_features = static_cast<int>(current_left_valid_points.size());
  if (result.tracked_features < min_inliers_) {
    result.reason = "reinitializing_stereo_tracking_lost";
    return result;
  }

  auto match_right_points = [this](
    const cv::Mat & left_image,
    const cv::Mat & right_image,
    const std::vector<cv::Point2f> & left_points,
    std::vector<cv::Point2f> & right_points_out,
    std::vector<unsigned char> & status_out) {
      std::vector<float> stereo_errors;
      cv::calcOpticalFlowPyrLK(
        left_image,
        right_image,
        left_points,
        right_points_out,
        status_out,
        stereo_errors,
        cv::Size(optical_flow_window_px_, optical_flow_window_px_),
        optical_flow_max_pyramid_level_,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
      for (std::size_t index = 0; index < status_out.size(); ++index) {
        if (!status_out[index] || stereo_errors[index] > stereo_match_max_error_) {
          status_out[index] = 0U;
        }
      }
    };

  std::vector<cv::Point2f> previous_right_points;
  std::vector<cv::Point2f> current_right_points;
  std::vector<unsigned char> previous_right_status;
  std::vector<unsigned char> current_right_status;
  match_right_points(
    previous_frame.left_gray_image,
    previous_frame.right_gray_image,
    previous_left_valid_points,
    previous_right_points,
    previous_right_status);
  match_right_points(
    current_left_gray_image,
    current_right_gray_image,
    current_left_valid_points,
    current_right_points,
    current_right_status);

  std::vector<cv::Point2f> previous_left_stereo_points;
  std::vector<cv::Point2f> previous_right_stereo_points;
  std::vector<cv::Point2f> current_left_stereo_points;
  std::vector<cv::Point2f> current_right_stereo_points;
  previous_left_stereo_points.reserve(previous_left_valid_points.size());
  previous_right_stereo_points.reserve(previous_left_valid_points.size());
  current_left_stereo_points.reserve(previous_left_valid_points.size());
  current_right_stereo_points.reserve(previous_left_valid_points.size());
  for (std::size_t index = 0; index < previous_left_valid_points.size(); ++index) {
    if (!previous_right_status[index] || !current_right_status[index]) {
      continue;
    }
    const double previous_disparity =
      std::abs(previous_left_valid_points[index].x - previous_right_points[index].x);
    const double current_disparity =
      std::abs(current_left_valid_points[index].x - current_right_points[index].x);
    if (previous_disparity < stereo_min_disparity_px_ || current_disparity < stereo_min_disparity_px_) {
      continue;
    }
    previous_left_stereo_points.push_back(previous_left_valid_points[index]);
    previous_right_stereo_points.push_back(previous_right_points[index]);
    current_left_stereo_points.push_back(current_left_valid_points[index]);
    current_right_stereo_points.push_back(current_right_points[index]);
  }

  if (static_cast<int>(current_left_stereo_points.size()) < min_inliers_) {
    result.reason = "reinitializing_stereo_correspondence_lost";
    return result;
  }

  std::vector<Eigen::Vector3d> previous_points_3d;
  std::vector<Eigen::Vector3d> current_points_3d;
  const tf2::Transform previous_left_to_right =
    previous_frame.base_to_left.inverse() * previous_frame.base_to_right;
  const tf2::Transform current_left_to_right = base_to_left.inverse() * base_to_right;
  if (!triangulateStereoCorrespondences(
        stereo_pair.left_camera->calibration,
        stereo_pair.right_camera->calibration,
        previous_left_to_right,
        previous_left_stereo_points,
        previous_right_stereo_points,
        previous_points_3d))
  {
    result.reason = "reinitializing_stereo_triangulation_failed";
    return result;
  }
  if (!triangulateStereoCorrespondences(
        stereo_pair.left_camera->calibration,
        stereo_pair.right_camera->calibration,
        current_left_to_right,
        current_left_stereo_points,
        current_right_stereo_points,
        current_points_3d))
  {
    result.reason = "reinitializing_stereo_triangulation_failed";
    return result;
  }
  if (previous_points_3d.size() != current_points_3d.size() ||
    previous_points_3d.size() < static_cast<std::size_t>(min_inliers_))
  {
    result.reason = "reinitializing_stereo_triangulation_inconsistent";
    return result;
  }

  tf2::Transform delta_left(tf2::Transform::getIdentity());
  int rigid_inliers = 0;
  if (!estimateRigidTransform(previous_points_3d, current_points_3d, delta_left, rigid_inliers) ||
    rigid_inliers < min_inliers_)
  {
    result.reason = "reinitializing_stereo_rigid_fit_failed";
    return result;
  }

  result.inliers = rigid_inliers;
  result.tracked_points = current_left_stereo_points;
  result.metric_translation_scale = delta_left.getOrigin().length();

  tf2::Transform delta_base = base_to_left * delta_left * base_to_left.inverse();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(delta_base.getRotation()).getRPY(roll, pitch, yaw);
  result.delta_yaw_rad = normalizeAngle(yaw);
  if (use_imu_yaw_ && current_motion_reference.yaw_rad.has_value() &&
    previous_frame.motion_reference.yaw_rad.has_value())
  {
    result.delta_yaw_rad = normalizeAngle(
      *current_motion_reference.yaw_rad - *previous_frame.motion_reference.yaw_rad);
  }

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

  result.dt_seconds = std::max((stamp - previous_frame.stamp).seconds(), 1.0e-3);
  result.delta_base = delta_base;
  result.success = true;
  return result;
}

VisualOdometryNode::TrackingResult VisualOdometryNode::estimateMotion(
  const std::shared_ptr<CameraState> & camera,
  const cv::Mat & current_gray_image,
  const MotionReferenceSample & current_motion_reference,
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

  result.metric_translation_scale = 0.0;
  if (current_motion_reference.planar_position.has_value()) {
    result.metric_translation_scale = planarDistance(
      camera->previous_motion_reference_position,
      *current_motion_reference.planar_position);
    if (result.metric_translation_scale < min_scale_translation_meters_) {
      result.metric_translation_scale = 0.0;
    }
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
  if (use_imu_yaw_ && current_motion_reference.yaw_rad.has_value()) {
    result.delta_yaw_rad = normalizeAngle(
      *current_motion_reference.yaw_rad - camera->previous_motion_reference_yaw_rad);
  }

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

std::vector<cv::Point2f> VisualOdometryNode::detectFeatures(const cv::Mat & gray_image) const
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

std::vector<cv::Point2f> VisualOdometryNode::undistortPoints(
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

cv::Mat VisualOdometryNode::makeProjectionMatrix(
  const tf2::Matrix3x3 & rotation,
  const tf2::Vector3 & translation)
{
  cv::Mat projection = cv::Mat::zeros(3, 4, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      projection.at<double>(row, column) = rotation[row][column];
    }
  }
  projection.at<double>(0, 3) = translation.x();
  projection.at<double>(1, 3) = translation.y();
  projection.at<double>(2, 3) = translation.z();
  return projection;
}

bool VisualOdometryNode::triangulateStereoCorrespondences(
  const CameraCalibration & left_calibration,
  const CameraCalibration & right_calibration,
  const tf2::Transform & left_to_right,
  const std::vector<cv::Point2f> & left_points,
  const std::vector<cv::Point2f> & right_points,
  std::vector<Eigen::Vector3d> & points_3d) const
{
  points_3d.clear();
  if (left_points.size() != right_points.size() || left_points.size() < 4U) {
    return false;
  }

  std::vector<cv::Point2f> left_undistorted;
  std::vector<cv::Point2f> right_undistorted;
  if (left_calibration.distortion_model == "equidistant") {
    cv::fisheye::undistortPoints(
      left_points,
      left_undistorted,
      left_calibration.camera_matrix,
      left_calibration.distortion_coefficients);
  } else {
    cv::undistortPoints(
      left_points,
      left_undistorted,
      left_calibration.camera_matrix,
      left_calibration.distortion_coefficients);
  }
  if (right_calibration.distortion_model == "equidistant") {
    cv::fisheye::undistortPoints(
      right_points,
      right_undistorted,
      right_calibration.camera_matrix,
      right_calibration.distortion_coefficients);
  } else {
    cv::undistortPoints(
      right_points,
      right_undistorted,
      right_calibration.camera_matrix,
      right_calibration.distortion_coefficients);
  }

  tf2::Matrix3x3 rotation_matrix(left_to_right.getRotation());
  tf2::Matrix3x3 identity_rotation;
  identity_rotation.setIdentity();
  const cv::Mat left_projection = makeProjectionMatrix(
    identity_rotation,
    tf2::Vector3(0.0, 0.0, 0.0));
  const cv::Mat right_projection = makeProjectionMatrix(rotation_matrix, left_to_right.getOrigin());

  cv::Mat homogeneous_points;
  cv::triangulatePoints(
    left_projection,
    right_projection,
    left_undistorted,
    right_undistorted,
    homogeneous_points);

  points_3d.reserve(left_points.size());
  for (int column = 0; column < homogeneous_points.cols; ++column) {
    const double w = homogeneous_points.at<double>(3, column);
    if (std::abs(w) < 1.0e-9) {
      continue;
    }

    const Eigen::Vector3d point(
      homogeneous_points.at<double>(0, column) / w,
      homogeneous_points.at<double>(1, column) / w,
      homogeneous_points.at<double>(2, column) / w);
    if (!point.allFinite() || point.z() <= 0.0) {
      continue;
    }
    points_3d.push_back(point);
  }

  return points_3d.size() >= 4U;
}

bool VisualOdometryNode::estimateRigidTransform(
  const std::vector<Eigen::Vector3d> & previous_points,
  const std::vector<Eigen::Vector3d> & current_points,
  tf2::Transform & delta_transform,
  int & inlier_count) const
{
  inlier_count = 0;
  if (previous_points.size() != current_points.size() || previous_points.size() < 4U) {
    return false;
  }

  auto compute_transform = [](const std::vector<Eigen::Vector3d> & source_points,
      const std::vector<Eigen::Vector3d> & target_points,
      Eigen::Matrix3d & rotation_out,
      Eigen::Vector3d & translation_out) {
      Eigen::Vector3d source_centroid = Eigen::Vector3d::Zero();
      Eigen::Vector3d target_centroid = Eigen::Vector3d::Zero();
      for (std::size_t index = 0; index < source_points.size(); ++index) {
        source_centroid += source_points[index];
        target_centroid += target_points[index];
      }
      source_centroid /= static_cast<double>(source_points.size());
      target_centroid /= static_cast<double>(target_points.size());

      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (std::size_t index = 0; index < source_points.size(); ++index) {
        covariance +=
          (source_points[index] - source_centroid) * (target_points[index] - target_centroid).transpose();
      }

      const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        covariance,
        Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
      if (rotation.determinant() < 0.0) {
        Eigen::Matrix3d corrected_v = svd.matrixV();
        corrected_v.col(2) *= -1.0;
        rotation = corrected_v * svd.matrixU().transpose();
      }

      rotation_out = rotation;
      translation_out = target_centroid - (rotation * source_centroid);
    };

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  compute_transform(previous_points, current_points, rotation, translation);

  std::vector<Eigen::Vector3d> inlier_previous_points;
  std::vector<Eigen::Vector3d> inlier_current_points;
  for (std::size_t index = 0; index < previous_points.size(); ++index) {
    const Eigen::Vector3d residual =
      (rotation * previous_points[index]) + translation - current_points[index];
    if (residual.norm() > stereo_max_reprojection_error_m_) {
      continue;
    }
    inlier_previous_points.push_back(previous_points[index]);
    inlier_current_points.push_back(current_points[index]);
  }

  inlier_count = static_cast<int>(inlier_previous_points.size());
  if (inlier_count < 4) {
    return false;
  }

  compute_transform(inlier_previous_points, inlier_current_points, rotation, translation);
  if (!rotation.allFinite() || !translation.allFinite()) {
    return false;
  }

  tf2::Matrix3x3 tf_rotation(
    rotation(0, 0), rotation(0, 1), rotation(0, 2),
    rotation(1, 0), rotation(1, 1), rotation(1, 2),
    rotation(2, 0), rotation(2, 1), rotation(2, 2));
  tf2::Quaternion tf_quaternion;
  tf_rotation.getRotation(tf_quaternion);
  delta_transform.setRotation(tf_quaternion);
  delta_transform.setOrigin(tf2::Vector3(translation.x(), translation.y(), translation.z()));
  return true;
}

std::optional<VisualOdometryNode::MotionReferenceSample>
VisualOdometryNode::lookupMotionReference(
  const rclcpp::Time & stamp) const
{
  const auto tolerance = rclcpp::Duration::from_seconds(motion_reference_lookup_tolerance_seconds_);
  auto lookup_best_match = [&stamp, &tolerance](const auto & history)
    -> const MotionReferenceSample * {
      const MotionReferenceSample * best_match = nullptr;
      auto best_delta = rclcpp::Duration::from_seconds(std::numeric_limits<double>::max());

      for (const auto & sample : history) {
        const rclcpp::Time candidate_stamp(sample.stamp);
        const rclcpp::Duration delta =
          candidate_stamp > stamp ? (candidate_stamp - stamp) : (stamp - candidate_stamp);
        if (delta < best_delta) {
          best_delta = delta;
          best_match = &sample;
        }
      }

      if (best_match == nullptr || best_delta > tolerance) {
        return nullptr;
      }
      return best_match;
    };

  MotionReferenceSample combined_sample;
  combined_sample.stamp = stamp;

  if (use_wheel_scale_) {
    const MotionReferenceSample * wheel_match = lookup_best_match(wheel_motion_reference_history_);
    if (wheel_match == nullptr || !wheel_match->planar_position.has_value()) {
      return std::nullopt;
    }
    combined_sample.planar_position = wheel_match->planar_position;
  }

  if (use_imu_yaw_) {
    const MotionReferenceSample * imu_match = lookup_best_match(imu_motion_reference_history_);
    if (imu_match == nullptr || !imu_match->yaw_rad.has_value()) {
      return std::nullopt;
    }
    combined_sample.yaw_rad = imu_match->yaw_rad;
  }

  return combined_sample;
}

bool VisualOdometryNode::resolveCameraExtrinsics(
  const std::shared_ptr<CameraState> & camera,
  const std::string & camera_frame,
  const rclcpp::Time & stamp)
{
  if (camera_frame.empty()) {
    return false;
  }

  try {
    const geometry_msgs::msg::TransformStamped transform =
      tf_buffer_.lookupTransform(base_frame_, camera_frame, stamp);
    tf2::fromMsg(transform.transform, camera->base_to_camera);
    return true;
  } catch (const tf2::TransformException & error) {
    try {
      const geometry_msgs::msg::TransformStamped latest_transform =
        tf_buffer_.lookupTransform(base_frame_, camera_frame, tf2::TimePointZero);
      tf2::fromMsg(latest_transform.transform, camera->base_to_camera);
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Using latest available transform from %s to %s for camera %s because the exact stamp lookup failed: %s",
        base_frame_.c_str(),
        camera_frame.c_str(),
        camera->name.c_str(),
        error.what());
      return true;
    } catch (const tf2::TransformException &) {
    }

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

void VisualOdometryNode::queueEstimate(
  const std::string & camera_name,
  const TrackingResult & result,
  const rclcpp::Time & stamp,
  double fusion_weight)
{
  PendingEstimate estimate;
  estimate.camera_name = camera_name;
  estimate.stamp = stamp;
  estimate.fusion_weight = fusion_weight;
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

void VisualOdometryNode::flushPendingEstimates(
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

VisualOdometryNode::TrackingResult VisualOdometryNode::fuseTrackingResults(
  const std::vector<PendingEstimate> & estimates) const
{
  TrackingResult fused_result;
  if (estimates.empty()) {
    return fused_result;
  }

  if (fusion_method_ != "ekf") {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Unsupported fusion_method '%s'; falling back to ekf.",
      fusion_method_.c_str());
  }

  Eigen::Vector3d state = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 1.0e3;
  double scale_sum = 0.0;
  double dt_sum = 0.0;
  double weight_sum = 0.0;

  for (const auto & estimate : estimates) {
    const double weight =
      std::max(0.1, trackingConfidence(estimate.result)) * std::max(0.1, estimate.fusion_weight);
    Eigen::Vector3d measurement(
      estimate.result.delta_base.getOrigin().x(),
      estimate.result.delta_base.getOrigin().y(),
      estimate.result.delta_yaw_rad);
    Eigen::Matrix3d measurement_covariance = makeMeasurementCovariance(estimate.result);
    const Eigen::Matrix3d innovation_covariance = covariance + measurement_covariance;
    const Eigen::Matrix3d kalman_gain = covariance * innovation_covariance.inverse();
    Eigen::Vector3d innovation = measurement - state;
    innovation.z() = normalizeAngle(innovation.z());
    state = state + (kalman_gain * innovation);
    state.z() = normalizeAngle(state.z());
    covariance = (Eigen::Matrix3d::Identity() - kalman_gain) * covariance;

    scale_sum += weight * estimate.result.metric_translation_scale;
    dt_sum += weight * estimate.result.dt_seconds;
    fused_result.tracked_features += estimate.result.tracked_features;
    fused_result.inliers += estimate.result.inliers;
    weight_sum += weight;
  }

  if (weight_sum <= 1.0e-6 || !covariance.allFinite()) {
    return fused_result;
  }

  tf2::Transform fused_delta(tf2::Transform::getIdentity());
  fused_delta.setOrigin(tf2::Vector3(state.x(), state.y(), 0.0));
  fused_result.delta_yaw_rad = normalizeAngle(state.z());
  tf2::Quaternion fused_rotation;
  fused_rotation.setRPY(0.0, 0.0, fused_result.delta_yaw_rad);
  fused_delta.setRotation(fused_rotation);

  if (force_2d_) {
    tf2::Transform planar_delta(tf2::Transform::getIdentity());
    planar_delta.setOrigin(tf2::Vector3(state.x(), state.y(), 0.0));
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

void VisualOdometryNode::publishOdometry(
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
  odometry_message.twist.covariance = makeTwistCovariance(result, dt_seconds);
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

void VisualOdometryNode::publishStatus(const std::string & status_message) const
{
  std_msgs::msg::String message;
  message.data = status_message;
  status_publisher_->publish(message);
}

double VisualOdometryNode::trackingConfidence(const TrackingResult & result) const
{
  if (result.tracked_features <= 0) {
    return 0.0;
  }
  return std::clamp(
    static_cast<double>(result.inliers) / static_cast<double>(result.tracked_features),
    0.0,
    1.0);
}

Eigen::Matrix3d VisualOdometryNode::makeMeasurementCovariance(
  const TrackingResult & result) const
{
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  const double bounded_confidence = std::clamp(trackingConfidence(result), 0.1, 1.0);
  const double position_sigma = std::clamp(
    pose_sigma_floor_m_ / bounded_confidence,
    pose_sigma_floor_m_,
    pose_sigma_ceiling_m_);
  const double yaw_sigma = std::clamp(
    yaw_sigma_floor_rad_ / bounded_confidence,
    yaw_sigma_floor_rad_,
    yaw_sigma_ceiling_rad_);
  covariance(0, 0) = position_sigma * position_sigma;
  covariance(1, 1) = position_sigma * position_sigma;
  covariance(2, 2) = yaw_sigma * yaw_sigma;
  return covariance;
}

double VisualOdometryNode::normalizeAngle(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double VisualOdometryNode::yawFromQuaternion(
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

double VisualOdometryNode::planarDistance(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

double VisualOdometryNode::planarDistance(const tf2::Vector3 & lhs, const tf2::Vector3 & rhs)
{
  const double dx = lhs.x() - rhs.x();
  const double dy = lhs.y() - rhs.y();
  return std::sqrt((dx * dx) + (dy * dy));
}

tf2::Transform VisualOdometryNode::poseToTransform(
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

geometry_msgs::msg::Pose VisualOdometryNode::transformToPose(
  const tf2::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();
  pose.orientation = tf2::toMsg(transform.getRotation());
  return pose;
}

std::array<double, 36> VisualOdometryNode::makePoseCovariance(
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

std::array<double, 36> VisualOdometryNode::makeTwistCovariance(
  const TrackingResult & result,
  double dt_seconds) const
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
  const double safe_dt_seconds = std::max(dt_seconds, 1.0e-3);
  const double velocity_sigma = position_sigma / safe_dt_seconds;
  const double yaw_rate_sigma = yaw_sigma / safe_dt_seconds;

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
  auto node = std::make_shared<amr_sweeper_visual_odometry::VisualOdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
