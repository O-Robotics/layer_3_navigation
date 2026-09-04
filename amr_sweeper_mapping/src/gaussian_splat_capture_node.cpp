#include "gaussian_splat_capture_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace amr_sweeper_mapping
{

namespace
{

constexpr char kDefaultPointcloudTopic[] = "depth_camera/depth/color/points";
constexpr char kDefaultWorldFrame[] = "map";
constexpr char kDefaultRepresentationName[] = "global_gaussian_map";
constexpr char kDefaultOdometryTopic[] = "localization/odometry_fused";
constexpr char kDefaultMapPoseStatusTopic[] = "mapping/map_pose_status";

std::vector<std::string> defaultCameraNames()
{
  return {
    "depth_color",
    "front_left_camera",
    "front_right_camera",
    "rear_left_camera",
    "rear_right_camera",
    "tools_camera"};
}

std::vector<std::string> defaultImageTopics()
{
  return {
    "depth_camera/color/image_raw",
    "usb_cameras/front_left_camera/image_raw",
    "usb_cameras/front_right_camera/image_raw",
    "usb_cameras/rear_left_camera/image_raw",
    "usb_cameras/rear_right_camera/image_raw",
    "usb_cameras/tools_camera/image_raw"};
}

std::vector<std::string> defaultCameraInfoTopics()
{
  return {
    "depth_camera/color/camera_info",
    "usb_cameras/front_left_camera/front_left_camera_info",
    "usb_cameras/front_right_camera/front_right_camera_info",
    "usb_cameras/rear_left_camera/rear_left_camera_info",
    "usb_cameras/rear_right_camera/rear_right_camera_info",
    "usb_cameras/tools_camera/tools_camera_info"};
}

std::array<std::uint8_t, 3> unpackPackedRgb(const float rgb_float)
{
  std::uint32_t packed_rgb = 0U;
  std::memcpy(&packed_rgb, &rgb_float, sizeof(float));
  return {
    static_cast<std::uint8_t>((packed_rgb >> 16) & 0xFFU),
    static_cast<std::uint8_t>((packed_rgb >> 8) & 0xFFU),
    static_cast<std::uint8_t>(packed_rgb & 0xFFU)};
}

bool hasField(const sensor_msgs::msg::PointCloud2 & message, const std::string & field_name)
{
  return std::any_of(
    message.fields.begin(),
    message.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & field) {
      return field.name == field_name;
    });
}

double secondsBetween(const rclcpp::Time & lhs, const rclcpp::Time & rhs)
{
  return std::abs((lhs - rhs).seconds());
}

std::string sanitizeFileToken(const std::string & value)
{
  std::string out;
  out.reserve(value.size());
  for (const char character : value) {
    if (
      (character >= 'a' && character <= 'z') ||
      (character >= 'A' && character <= 'Z') ||
      (character >= '0' && character <= '9') ||
      character == '_' || character == '-')
    {
      out.push_back(character);
    } else {
      out.push_back('_');
    }
  }
  return out.empty() ? "camera" : out;
}

std::string bundleDirectoryName(const std::uint64_t bundle_index)
{
  std::ostringstream stream;
  stream << "bundle_" << std::setw(6) << std::setfill('0') << bundle_index;
  return stream.str();
}

std::string pointFieldDatatypeName(const std::uint8_t datatype)
{
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return "int8";
    case sensor_msgs::msg::PointField::UINT8:
      return "uint8";
    case sensor_msgs::msg::PointField::INT16:
      return "int16";
    case sensor_msgs::msg::PointField::UINT16:
      return "uint16";
    case sensor_msgs::msg::PointField::INT32:
      return "int32";
    case sensor_msgs::msg::PointField::UINT32:
      return "uint32";
    case sensor_msgs::msg::PointField::FLOAT32:
      return "float32";
    case sensor_msgs::msg::PointField::FLOAT64:
      return "float64";
    default:
      return "unknown";
  }
}

}  // namespace

std::size_t GaussianVoxelKeyHash::operator()(const GaussianVoxelKey & key) const
{
  const std::size_t hx = std::hash<int>{}(key.x);
  const std::size_t hy = std::hash<int>{}(key.y);
  const std::size_t hz = std::hash<int>{}(key.z);
  return hx ^ (hy << 1U) ^ (hz << 2U);
}

GaussianSplatCaptureNode::GaussianSplatCaptureNode()
: Node("gaussian_splat_capture_node"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  declare_parameter("output_directory", std::string(""));
  declare_parameter("mission_id", std::string(""));
  declare_parameter("gaussian_manifest_file", std::string(""));
  declare_parameter("source_mode", std::string("synchronized_camera_dataset"));
  declare_parameter("representation_name", std::string(kDefaultRepresentationName));
  declare_parameter("world_frame", std::string(kDefaultWorldFrame));
  declare_parameter("odom_topic", std::string(kDefaultOdometryTopic));
  declare_parameter("map_pose_status_topic", std::string(kDefaultMapPoseStatusTopic));
  declare_parameter("camera_names", defaultCameraNames());
  declare_parameter("camera_image_topics", defaultImageTopics());
  declare_parameter("camera_info_topics", defaultCameraInfoTopics());
  declare_parameter("capture_distance_meters", 0.50);
  declare_parameter("capture_yaw_degrees", 15.0);
  declare_parameter("min_capture_period_seconds", 1.0);
  declare_parameter("max_camera_timestamp_skew_seconds", 0.20);
  declare_parameter("max_camera_sample_age_seconds", 1.00);
  declare_parameter("camera_discovery_seconds", 2.00);
  declare_parameter("min_camera_count", 0);
  declare_parameter("enable_pointcloud_capture", true);
  declare_parameter("pointcloud_topic", std::string(kDefaultPointcloudTopic));
  declare_parameter("max_pointcloud_sample_age_seconds", 2.00);
  declare_parameter("enable_voxel_preview", false);
  declare_parameter("voxel_size_meters", 0.10);
  declare_parameter("min_range_meters", 0.30);
  declare_parameter("max_range_meters", 8.00);
  declare_parameter("process_every_nth_point", 12);
  declare_parameter("max_voxels", 150000);
  declare_parameter("max_debug_points", 25000);
  declare_parameter("save_ply", true);
  declare_parameter("save_json", true);
  declare_parameter("status_period_seconds", 2.0);
  declare_parameter("debug_publish_period_seconds", 3.0);
  declare_parameter("artifact_save_period_seconds", 10.0);

  output_directory_ = get_parameter("output_directory").as_string();
  mission_id_ = get_parameter("mission_id").as_string();
  gaussian_manifest_file_ = get_parameter("gaussian_manifest_file").as_string();
  source_mode_ = get_parameter("source_mode").as_string();
  representation_name_ = get_parameter("representation_name").as_string();
  world_frame_ = get_parameter("world_frame").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  map_pose_status_topic_ = get_parameter("map_pose_status_topic").as_string();
  capture_distance_meters_ = std::max(0.0, get_parameter("capture_distance_meters").as_double());
  capture_yaw_degrees_ = std::max(0.0, get_parameter("capture_yaw_degrees").as_double());
  min_capture_period_seconds_ = std::max(0.0,
      get_parameter("min_capture_period_seconds").as_double());
  max_camera_timestamp_skew_seconds_ =
    std::max(0.0, get_parameter("max_camera_timestamp_skew_seconds").as_double());
  max_camera_sample_age_seconds_ =
    std::max(0.0, get_parameter("max_camera_sample_age_seconds").as_double());
  camera_discovery_seconds_ =
    std::max(0.0, get_parameter("camera_discovery_seconds").as_double());
  min_camera_count_ = std::max<int>(0,
      static_cast<int>(get_parameter("min_camera_count").as_int()));
  enable_pointcloud_capture_ = get_parameter("enable_pointcloud_capture").as_bool();
  pointcloud_topic_ = get_parameter("pointcloud_topic").as_string();
  max_pointcloud_sample_age_seconds_ =
    std::max(0.0, get_parameter("max_pointcloud_sample_age_seconds").as_double());
  enable_voxel_preview_ = get_parameter("enable_voxel_preview").as_bool();
  voxel_size_meters_ = get_parameter("voxel_size_meters").as_double();
  min_range_meters_ = get_parameter("min_range_meters").as_double();
  max_range_meters_ = get_parameter("max_range_meters").as_double();
  process_every_nth_point_ =
    std::max<int>(1, static_cast<int>(get_parameter("process_every_nth_point").as_int()));
  max_voxels_ = std::max<int>(1, static_cast<int>(get_parameter("max_voxels").as_int()));
  max_debug_points_ = std::max<int>(1,
      static_cast<int>(get_parameter("max_debug_points").as_int()));
  save_ply_ = get_parameter("save_ply").as_bool();
  save_json_ = get_parameter("save_json").as_bool();

  const auto camera_names = get_parameter("camera_names").as_string_array();
  const auto image_topics = get_parameter("camera_image_topics").as_string_array();
  const auto camera_info_topics = get_parameter("camera_info_topics").as_string_array();
  const std::size_t camera_count =
    std::min(camera_names.size(), std::min(image_topics.size(), camera_info_topics.size()));
  camera_streams_.reserve(camera_count);
  for (std::size_t index = 0; index < camera_count; ++index) {
    CameraStream stream;
    stream.name = camera_names.at(index);
    stream.image_topic = image_topics.at(index);
    stream.camera_info_topic = camera_info_topics.at(index);
    camera_streams_.push_back(std::move(stream));
  }

  for (std::size_t index = 0; index < camera_streams_.size(); ++index) {
    camera_streams_.at(index).image_subscription =
      create_subscription<sensor_msgs::msg::Image>(
        camera_streams_.at(index).image_topic,
        rclcpp::SensorDataQoS(),
      [this, index](sensor_msgs::msg::Image::SharedPtr message) {
        handleCameraImage(index, std::move(message));
      });
    camera_streams_.at(index).camera_info_subscription =
      create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_streams_.at(index).camera_info_topic,
        rclcpp::SensorDataQoS(),
      [this, index](sensor_msgs::msg::CameraInfo::SharedPtr message) {
        handleCameraInfo(index, std::move(message));
      });
  }

  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_,
    10,
    std::bind(&GaussianSplatCaptureNode::handleOdometry, this, std::placeholders::_1));
  map_pose_status_subscription_ = create_subscription<std_msgs::msg::String>(
    map_pose_status_topic_,
    10,
    std::bind(&GaussianSplatCaptureNode::handleMapPoseStatus, this, std::placeholders::_1));

  if (enable_pointcloud_capture_ || enable_voxel_preview_) {
    pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&GaussianSplatCaptureNode::processPointCloud, this, std::placeholders::_1));
  }

  debug_pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("mapping/gaussian/world_points", 1);
  status_publisher_ = create_publisher<std_msgs::msg::String>("mapping/gaussian/status", 10);

  status_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("status_period_seconds").as_double()),
    std::bind(&GaussianSplatCaptureNode::publishStatus, this));
  capture_start_time_ = now();
  capture_timer_ = create_wall_timer(
    std::chrono::duration<double>(0.20),
    std::bind(&GaussianSplatCaptureNode::tryCaptureBundle, this));
  debug_publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("debug_publish_period_seconds").as_double()),
    std::bind(&GaussianSplatCaptureNode::publishDebugPointCloud, this));
  artifact_save_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("artifact_save_period_seconds").as_double()),
    std::bind(&GaussianSplatCaptureNode::saveArtifacts, this));

  RCLCPP_INFO(
    get_logger(),
    "Gaussian capture node ready with %zu camera stream(s), world_frame=%s, output=%s.",
    camera_streams_.size(),
    world_frame_.c_str(),
    output_directory_.empty() ? "<disabled until mission context>" : output_directory_.c_str());
}

GaussianSplatCaptureNode::~GaussianSplatCaptureNode()
{
  try {
    saveArtifacts();
    saveManifest();
  } catch (const std::exception & exception) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to save gaussian artifacts during shutdown: %s",
      exception.what());
  }
}

void GaussianSplatCaptureNode::handleCameraImage(
  const std::size_t camera_index,
  sensor_msgs::msg::Image::SharedPtr message)
{
  if (camera_index >= camera_streams_.size()) {
    return;
  }
  auto & stream = camera_streams_.at(camera_index);
  stream.latest_image = std::move(message);
  stream.image_history.push_back(stream.latest_image);
  while (stream.image_history.size() > 30U) {
    stream.image_history.pop_front();
  }
  ++stream.image_frames;
}

void GaussianSplatCaptureNode::handleCameraInfo(
  const std::size_t camera_index,
  sensor_msgs::msg::CameraInfo::SharedPtr message)
{
  if (camera_index >= camera_streams_.size()) {
    return;
  }
  auto & stream = camera_streams_.at(camera_index);
  stream.latest_camera_info = std::move(message);
  ++stream.camera_info_frames;
}

void GaussianSplatCaptureNode::handleOdometry(nav_msgs::msg::Odometry::SharedPtr message)
{
  latest_odometry_pose_ = message->pose.pose;
  latest_odometry_stamp_ = rclcpp::Time(message->header.stamp);
  latest_odometry_ready_ = true;
}

void GaussianSplatCaptureNode::handleMapPoseStatus(std_msgs::msg::String::SharedPtr message)
{
  latest_map_pose_status_ = message->data;
}

void GaussianSplatCaptureNode::processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
{
  latest_pointcloud_ = message;
  ++pointcloud_frames_;

  if (!enable_voxel_preview_) {
    return;
  }

  if (!hasField(*message, "x") || !hasField(*message, "y") || !hasField(*message, "z")) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Incoming point cloud is missing x/y/z fields.");
    return;
  }

  if (!hasField(*message, "rgb")) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Incoming point cloud is missing rgb field; voxel preview needs colored points.");
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      world_frame_,
      message->header.frame_id,
      message->header.stamp,
      tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Skipping point cloud preview due to missing transform %s -> %s: %s",
      message->header.frame_id.c_str(),
      world_frame_.c_str(),
      exception.what());
    return;
  }

  tf2::Transform cloud_to_world;
  tf2::fromMsg(transform_stamped.transform, cloud_to_world);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*message, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*message, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*message, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_rgb(*message, "rgb");

  int point_index = 0;
  std::size_t integrated_from_frame = 0U;
  const std::size_t point_count =
    static_cast<std::size_t>(message->width) * static_cast<std::size_t>(message->height);

  for (std::size_t index = 0; index < point_count;
    ++index, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
  {
    if ((point_index++ % process_every_nth_point_) != 0) {
      continue;
    }

    const double local_x = *iter_x;
    const double local_y = *iter_y;
    const double local_z = *iter_z;
    if (!std::isfinite(local_x) || !std::isfinite(local_y) || !std::isfinite(local_z)) {
      continue;
    }

    const double range = std::sqrt((local_x * local_x) + (local_y * local_y) + (local_z * local_z));
    if (range < min_range_meters_ || range > max_range_meters_) {
      continue;
    }

    const tf2::Vector3 world_point = cloud_to_world * tf2::Vector3(local_x, local_y, local_z);
    const auto [red, green, blue] = unpackPackedRgb(*iter_rgb);
    upsertVoxel(world_point.x(), world_point.y(), world_point.z(), red, green, blue);
    ++integrated_from_frame;
  }

  if (integrated_from_frame > 0U) {
    ++integrated_frames_;
    integrated_points_ += integrated_from_frame;
    artifact_dirty_ = true;
  }
}

void GaussianSplatCaptureNode::tryCaptureBundle()
{
  if (output_directory_.empty() || !latest_odometry_ready_ || !shouldCaptureForMotion()) {
    return;
  }
  if (secondsBetween(now(), capture_start_time_) < camera_discovery_seconds_) {
    return;
  }

  std::vector<std::pair<std::size_t, sensor_msgs::msg::Image::SharedPtr>> selected_cameras;
  selected_cameras.reserve(camera_streams_.size());
  rclcpp::Time target_stamp(0, 0, get_clock()->get_clock_type());
  bool have_target_stamp = false;

  for (std::size_t index = 0; index < camera_streams_.size(); ++index) {
    const auto & stream = camera_streams_.at(index);
    if (!stream.latest_image) {
      continue;
    }
    const rclcpp::Time image_stamp(stream.latest_image->header.stamp);
    if (!have_target_stamp || image_stamp < target_stamp) {
      target_stamp = image_stamp;
      have_target_stamp = true;
    }
  }

  if (!have_target_stamp) {
    ++rejected_bundles_;
    return;
  }

  rclcpp::Time min_stamp(0, 0, get_clock()->get_clock_type());
  rclcpp::Time max_stamp(0, 0, get_clock()->get_clock_type());
  bool have_stamp = false;

  for (std::size_t index = 0; index < camera_streams_.size(); ++index) {
    const auto & stream = camera_streams_.at(index);
    sensor_msgs::msg::Image::SharedPtr selected_image;
    double best_delta = std::numeric_limits<double>::max();
    for (const auto & image : stream.image_history) {
      const rclcpp::Time image_stamp(image->header.stamp);
      const double delta = secondsBetween(image_stamp, target_stamp);
      if (delta < best_delta) {
        best_delta = delta;
        selected_image = image;
      }
    }
    if (!selected_image || best_delta > max_camera_sample_age_seconds_) {
      continue;
    }

    const rclcpp::Time image_stamp(selected_image->header.stamp);
    if (!have_stamp) {
      min_stamp = image_stamp;
      max_stamp = image_stamp;
      have_stamp = true;
    } else {
      if (image_stamp < min_stamp) {
        min_stamp = image_stamp;
      }
      if (image_stamp > max_stamp) {
        max_stamp = image_stamp;
      }
    }
    selected_cameras.emplace_back(index, selected_image);
  }

  if (selected_cameras.size() < expectedCameraCount()) {
    ++rejected_bundles_;
    return;
  }

  const double timestamp_skew = have_stamp ? (max_stamp - min_stamp).seconds() : 0.0;
  if (secondsBetween(max_stamp, min_stamp) > max_camera_sample_age_seconds_) {
    ++rejected_bundles_;
    return;
  }
  if (timestamp_skew > max_camera_timestamp_skew_seconds_) {
    ++rejected_bundles_;
    return;
  }

  const std::uint64_t bundle_index = capture_bundles_ + 1U;
  const std::string bundle_relative_path = relativeBundlePath(bundle_index);
  const std::filesystem::path bundle_directory =
    std::filesystem::path(output_directory_) / bundle_relative_path;
  std::filesystem::create_directories(bundle_directory);

  nlohmann::json capture;
  capture["bundle_index"] = bundle_index;
  capture["bundle_directory"] = bundle_relative_path;
  capture["stamp"] = stampToString(max_stamp);
  capture["timestamp_skew_seconds"] = timestamp_skew;
  capture["world_frame"] = world_frame_;
  capture["odometry_pose"] = poseToJson(latest_odometry_pose_);
  capture["map_pose_status"] = latest_map_pose_status_;
  capture["cameras"] = nlohmann::json::array();

  bool wrote_any_image = false;
  bool wrote_any_trainable_image = false;
  for (const auto & [camera_index, image] : selected_cameras) {
    const auto & stream = camera_streams_.at(camera_index);
    const std::string camera_token = sanitizeFileToken(stream.name);
    const std::filesystem::path image_path = bundle_directory / (camera_token + ".ppm");
    std::string image_format;
    std::string image_error;
    const bool wrote_image = writeImageFile(*image, image_path, image_format,
        image_error);

    nlohmann::json camera_record;
    camera_record["name"] = stream.name;
    camera_record["image_topic"] = stream.image_topic;
    camera_record["camera_info_topic"] = stream.camera_info_topic;
    camera_record["stamp"] = stampToString(image->header.stamp);
    camera_record["frame_id"] = image->header.frame_id;
    camera_record["encoding"] = image->encoding;
    camera_record["width"] = image->width;
    camera_record["height"] = image->height;
    camera_record["image_file"] = wrote_image ?
      (bundle_relative_path + "/" + image_path.filename().string()) : "";
    camera_record["image_format"] = image_format;
    camera_record["image_error"] = image_error;
    const bool has_camera_info = static_cast<bool>(stream.latest_camera_info);
    if (has_camera_info) {
      camera_record["camera_info"] = cameraInfoToJson(*stream.latest_camera_info);
    }

    bool has_world_from_camera = false;
    try {
      const auto camera_pose = tf_buffer_.lookupTransform(
        world_frame_,
        image->header.frame_id,
        image->header.stamp,
        tf2::durationFromSec(0.02));
      camera_record["world_from_camera"] = transformToJson(camera_pose);
      has_world_from_camera = true;
    } catch (const tf2::TransformException & exception) {
      camera_record["world_from_camera_error"] = exception.what();
    }

    wrote_any_image = wrote_any_image || wrote_image;
    wrote_any_trainable_image =
      wrote_any_trainable_image || (wrote_image && has_camera_info && has_world_from_camera);
    capture["cameras"].push_back(camera_record);
  }

  if (!wrote_any_image || !wrote_any_trainable_image) {
    ++rejected_bundles_;
    return;
  }

  if (enable_pointcloud_capture_ && latest_pointcloud_) {
    const rclcpp::Time pointcloud_stamp(latest_pointcloud_->header.stamp);
    if (secondsBetween(max_stamp, pointcloud_stamp) <= max_pointcloud_sample_age_seconds_) {
      const std::filesystem::path pointcloud_path = bundle_directory / "depth_color_points.bin";
      std::string pointcloud_error;
      if (writePointCloudFile(*latest_pointcloud_, pointcloud_path, pointcloud_error)) {
        ++pointcloud_capture_files_;
        capture["pointcloud"] = pointcloudMetadataToJson(
          *latest_pointcloud_,
          bundle_relative_path + "/" + pointcloud_path.filename().string());
      } else {
        capture["pointcloud_error"] = pointcloud_error;
      }
    }
  }

  capture_records_.push_back(capture);
  ++capture_bundles_;
  last_capture_pose_ = latest_odometry_pose_;
  last_capture_stamp_ = max_stamp;
  last_capture_pose_ready_ = true;
  saveManifest();
}

std::size_t GaussianSplatCaptureNode::expectedCameraCount() const
{
  std::size_t observed_camera_count = 0U;
  for (const auto & stream : camera_streams_) {
    if (stream.image_frames > 0U) {
      ++observed_camera_count;
    }
  }
  const std::size_t configured_minimum =
    min_camera_count_ > 0 ?
    std::min<std::size_t>(static_cast<std::size_t>(min_camera_count_), camera_streams_.size()) :
    camera_streams_.size();
  return std::max<std::size_t>(
    configured_minimum,
    observed_camera_count);
}

bool GaussianSplatCaptureNode::shouldCaptureForMotion() const
{
  if (!last_capture_pose_ready_) {
    return true;
  }

  const rclcpp::Time motion_stamp = latest_odometry_stamp_.nanoseconds() >
    0 ? latest_odometry_stamp_ : now();
  if (secondsBetween(motion_stamp, last_capture_stamp_) < min_capture_period_seconds_) {
    return false;
  }

  const double dx = latest_odometry_pose_.position.x - last_capture_pose_.position.x;
  const double dy = latest_odometry_pose_.position.y - last_capture_pose_.position.y;
  const double distance = std::hypot(dx, dy);
  if (capture_distance_meters_ > 0.0 && distance >= capture_distance_meters_) {
    return true;
  }

  const double current_yaw = latestYawRadians();
  tf2::Quaternion previous_quaternion(
    last_capture_pose_.orientation.x,
    last_capture_pose_.orientation.y,
    last_capture_pose_.orientation.z,
    last_capture_pose_.orientation.w);
  double previous_roll = 0.0;
  double previous_pitch = 0.0;
  double previous_yaw = 0.0;
  tf2::Matrix3x3(previous_quaternion).getRPY(previous_roll, previous_pitch, previous_yaw);
  const double yaw_delta = std::abs(std::atan2(
    std::sin(current_yaw - previous_yaw),
    std::cos(current_yaw - previous_yaw)));
  return capture_yaw_degrees_ > 0.0 &&
         yaw_delta >= (capture_yaw_degrees_ * M_PI / 180.0);
}

double GaussianSplatCaptureNode::latestYawRadians() const
{
  tf2::Quaternion quaternion(
    latest_odometry_pose_.orientation.x,
    latest_odometry_pose_.orientation.y,
    latest_odometry_pose_.orientation.z,
    latest_odometry_pose_.orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

std::string GaussianSplatCaptureNode::stampToString(const builtin_interfaces::msg::Time & stamp) const
{
  std::ostringstream stream;
  stream << stamp.sec << "." << std::setw(9) << std::setfill('0') << stamp.nanosec;
  return stream.str();
}

std::string GaussianSplatCaptureNode::relativeBundlePath(const std::uint64_t bundle_index) const
{
  return std::string("frames/") + bundleDirectoryName(bundle_index);
}

bool GaussianSplatCaptureNode::writeImageFile(
  const sensor_msgs::msg::Image & image,
  const std::filesystem::path & path,
  std::string & format,
  std::string & error) const
{
  error.clear();
  format.clear();
  try {
    std::filesystem::create_directories(path.parent_path());
    if (image.encoding == "rgb8" || image.encoding == "bgr8") {
      std::ofstream stream(path, std::ios::binary);
      if (!stream.is_open()) {
        error = "failed to open image file";
        return false;
      }
      stream << "P6\n" << image.width << " " << image.height << "\n255\n";
      for (std::uint32_t row = 0; row < image.height; ++row) {
        const auto * row_data = image.data.data() + (static_cast<std::size_t>(row) * image.step);
        for (std::uint32_t column = 0; column < image.width; ++column) {
          const auto * pixel = row_data + (static_cast<std::size_t>(column) * 3U);
          if (image.encoding == "rgb8") {
            stream.write(reinterpret_cast<const char *>(pixel), 3);
          } else {
            const std::array<std::uint8_t, 3> rgb{pixel[2], pixel[1], pixel[0]};
            stream.write(reinterpret_cast<const char *>(rgb.data()), 3);
          }
        }
      }
      format = "ppm";
      return true;
    }

    if (image.encoding == "mono8" || image.encoding == "8UC1") {
      std::ofstream stream(path, std::ios::binary);
      if (!stream.is_open()) {
        error = "failed to open image file";
        return false;
      }
      stream << "P6\n" << image.width << " " << image.height << "\n255\n";
      for (std::uint32_t row = 0; row < image.height; ++row) {
        const auto * row_data = image.data.data() + (static_cast<std::size_t>(row) * image.step);
        for (std::uint32_t column = 0; column < image.width; ++column) {
          const std::uint8_t value = row_data[column];
          const std::array<std::uint8_t, 3> rgb{value, value, value};
          stream.write(reinterpret_cast<const char *>(rgb.data()), 3);
        }
      }
      format = "ppm";
      return true;
    }

    std::ofstream stream(path, std::ios::binary);
    if (!stream.is_open()) {
      error = "failed to open raw image file";
      return false;
    }
    stream.write(
      reinterpret_cast<const char *>(image.data.data()),
      static_cast<std::streamsize>(image.data.size()));
    format = "raw";
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }
}

bool GaussianSplatCaptureNode::writePointCloudFile(
  const sensor_msgs::msg::PointCloud2 & pointcloud,
  const std::filesystem::path & path,
  std::string & error) const
{
  error.clear();
  try {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream stream(path, std::ios::binary);
    if (!stream.is_open()) {
      error = "failed to open pointcloud file";
      return false;
    }
    stream.write(
      reinterpret_cast<const char *>(pointcloud.data.data()),
      static_cast<std::streamsize>(pointcloud.data.size()));
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }
}

nlohmann::json GaussianSplatCaptureNode::cameraInfoToJson(const sensor_msgs::msg::CameraInfo & message) const
{
  nlohmann::json document;
  document["stamp"] = stampToString(message.header.stamp);
  document["frame_id"] = message.header.frame_id;
  document["width"] = message.width;
  document["height"] = message.height;
  document["distortion_model"] = message.distortion_model;
  document["d"] = message.d;
  document["k"] = message.k;
  document["r"] = message.r;
  document["p"] = message.p;
  return document;
}

nlohmann::json GaussianSplatCaptureNode::poseToJson(const geometry_msgs::msg::Pose & pose) const
{
  return {
    {"position", {{"x", pose.position.x}, {"y", pose.position.y}, {"z", pose.position.z}}},
    {"orientation", {
        {"x", pose.orientation.x},
        {"y", pose.orientation.y},
        {"z", pose.orientation.z},
        {"w", pose.orientation.w}}}};
}

nlohmann::json GaussianSplatCaptureNode::pointcloudMetadataToJson(
  const sensor_msgs::msg::PointCloud2 & message,
  const std::string & file_name) const
{
  nlohmann::json fields = nlohmann::json::array();
  for (const auto & field : message.fields) {
    fields.push_back({
        {"name", field.name},
        {"offset", field.offset},
        {"datatype", pointFieldDatatypeName(field.datatype)},
        {"datatype_id", field.datatype},
        {"count", field.count}});
  }
  return {
    {"topic", pointcloud_topic_},
    {"file", file_name},
    {"format", "pointcloud2_data_binary"},
    {"stamp", stampToString(message.header.stamp)},
    {"frame_id", message.header.frame_id},
    {"height", message.height},
    {"width", message.width},
    {"fields", fields},
    {"is_bigendian", message.is_bigendian},
    {"point_step", message.point_step},
    {"row_step", message.row_step},
    {"is_dense", message.is_dense},
    {"byte_count", message.data.size()}};
}

nlohmann::json GaussianSplatCaptureNode::transformToJson(
  const geometry_msgs::msg::TransformStamped & transform) const
{
  return {
    {"stamp", stampToString(transform.header.stamp)},
    {"parent_frame", transform.header.frame_id},
    {"child_frame", transform.child_frame_id},
    {"translation", {
        {"x", transform.transform.translation.x},
        {"y", transform.transform.translation.y},
        {"z", transform.transform.translation.z}}},
    {"rotation", {
        {"x", transform.transform.rotation.x},
        {"y", transform.transform.rotation.y},
        {"z", transform.transform.rotation.z},
        {"w", transform.transform.rotation.w}}}};
}

nlohmann::json GaussianSplatCaptureNode::buildManifest() const
{
  nlohmann::json camera_streams = nlohmann::json::array();
  for (const auto & stream : camera_streams_) {
    camera_streams.push_back({
        {"name", stream.name},
        {"image_topic", stream.image_topic},
        {"camera_info_topic", stream.camera_info_topic},
        {"image_frames_seen", stream.image_frames},
        {"camera_info_frames_seen", stream.camera_info_frames},
        {"available", stream.image_frames > 0U}});
  }

  return {
    {"representation", "synchronized_gaussian_capture_dataset"},
    {"representation_name", representation_name_},
    {"mission_id", mission_id_},
    {"output_directory", output_directory_},
    {"gaussian_manifest_file", gaussian_manifest_file_},
    {"source_mode", source_mode_},
    {"world_frame", world_frame_},
    {"odometry_topic", odom_topic_},
    {"map_pose_status_topic", map_pose_status_topic_},
    {"pointcloud_topic", pointcloud_topic_},
    {"enable_pointcloud_capture", enable_pointcloud_capture_},
    {"enable_voxel_preview", enable_voxel_preview_},
    {"capture_distance_meters", capture_distance_meters_},
    {"capture_yaw_degrees", capture_yaw_degrees_},
    {"min_capture_period_seconds", min_capture_period_seconds_},
    {"max_camera_timestamp_skew_seconds", max_camera_timestamp_skew_seconds_},
    {"max_camera_sample_age_seconds", max_camera_sample_age_seconds_},
    {"max_pointcloud_sample_age_seconds", max_pointcloud_sample_age_seconds_},
    {"camera_discovery_seconds", camera_discovery_seconds_},
    {"min_camera_count", min_camera_count_},
    {"expected_camera_count", expectedCameraCount()},
    {"camera_streams", camera_streams},
    {"capture_bundles", capture_bundles_},
    {"rejected_bundles", rejected_bundles_},
    {"pointcloud_frames_seen", pointcloud_frames_},
    {"pointcloud_capture_files", pointcloud_capture_files_},
    {"voxel_preview", {
        {"voxels", voxels_.size()},
        {"integrated_frames", integrated_frames_},
        {"integrated_points", integrated_points_}}},
    {"captures", capture_records_}};
}

void GaussianSplatCaptureNode::saveManifest()
{
  if (output_directory_.empty()) {
    return;
  }
  std::filesystem::create_directories(output_directory_);
  const auto path = gaussian_manifest_file_.empty() ?
    (std::filesystem::path(output_directory_) / "manifest.json") :
    std::filesystem::path(gaussian_manifest_file_);
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("Failed to write gaussian manifest");
  }
  stream << buildManifest().dump(2) << "\n";
}

void GaussianSplatCaptureNode::publishStatus()
{
  std_msgs::msg::String message;
  message.data =
    "gaussian_splat_capture_node ready; source_mode=" + source_mode_ +
    "; world_frame=" + world_frame_ +
    "; cameras=" + std::to_string(camera_streams_.size()) +
    "; capture_bundles=" + std::to_string(capture_bundles_) +
    "; rejected_bundles=" + std::to_string(rejected_bundles_) +
    "; pointcloud_frames=" + std::to_string(pointcloud_frames_) +
    "; pointcloud_files=" + std::to_string(pointcloud_capture_files_) +
    "; voxel_preview=" + std::string(enable_voxel_preview_ ? "true" : "false") +
    "; voxels=" + std::to_string(voxels_.size());
  status_publisher_->publish(message);
}

void GaussianSplatCaptureNode::publishDebugPointCloud()
{
  if (!enable_voxel_preview_ || voxels_.empty()) {
    return;
  }

  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp = now();
  message.header.frame_id = world_frame_;
  message.height = 1;
  message.is_dense = false;

  const std::size_t publish_count = std::min<std::size_t>(voxels_.size(), max_debug_points_);
  message.width = static_cast<std::uint32_t>(publish_count);

  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(publish_count);

  sensor_msgs::PointCloud2Iterator<float> iter_x(message, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(message, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(message, "z");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> iter_r(message, "r");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> iter_g(message, "g");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> iter_b(message, "b");

  std::size_t written = 0U;
  for (const auto & [key, voxel] : voxels_) {
    (void)key;
    if (written >= publish_count) {
      break;
    }

    const double inverse_count = 1.0 / static_cast<double>(voxel.count);
    *iter_x = static_cast<float>(voxel.sum_x * inverse_count);
    *iter_y = static_cast<float>(voxel.sum_y * inverse_count);
    *iter_z = static_cast<float>(voxel.sum_z * inverse_count);
    *iter_r = static_cast<std::uint8_t>(std::clamp(voxel.sum_r * inverse_count, 0.0, 255.0));
    *iter_g = static_cast<std::uint8_t>(std::clamp(voxel.sum_g * inverse_count, 0.0, 255.0));
    *iter_b = static_cast<std::uint8_t>(std::clamp(voxel.sum_b * inverse_count, 0.0, 255.0));

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
    ++written;
  }

  debug_pointcloud_publisher_->publish(message);
}

void GaussianSplatCaptureNode::saveArtifacts()
{
  saveManifest();
  if (
    !enable_voxel_preview_ ||
    !artifact_dirty_ ||
    voxels_.empty() ||
    output_directory_.empty())
  {
    return;
  }

  namespace fs = std::filesystem;
  fs::create_directories(output_directory_);

  const std::string base_path = artifactBasePath();

  if (save_json_) {
    nlohmann::json document;
    document["representation"] = "voxel_gaussians";
    document["world_frame"] = world_frame_;
    document["voxel_size_meters"] = voxel_size_meters_;
    document["integrated_frames"] = integrated_frames_;
    document["integrated_points"] = integrated_points_;
    document["gaussians"] = nlohmann::json::array();

    for (const auto & [key, voxel] : voxels_) {
      (void)key;
      const double inverse_count = 1.0 / static_cast<double>(voxel.count);
      const double mean_x = voxel.sum_x * inverse_count;
      const double mean_y = voxel.sum_y * inverse_count;
      const double mean_z = voxel.sum_z * inverse_count;
      const double sigma =
        std::max(voxel_size_meters_ * 0.5, std::sqrt(voxel.sum_squared_radius * inverse_count));

      document["gaussians"].push_back({
          {"position", {mean_x, mean_y, mean_z}},
          {"color", {
              static_cast<int>(std::lround(voxel.sum_r * inverse_count)),
              static_cast<int>(std::lround(voxel.sum_g * inverse_count)),
              static_cast<int>(std::lround(voxel.sum_b * inverse_count))}},
          {"sigma", sigma},
          {"weight", voxel.count}});
    }

    std::ofstream json_stream(base_path + ".json");
    if (!json_stream.is_open()) {
      throw std::runtime_error("Failed to write gaussian json artifact");
    }
    json_stream << document.dump(2) << "\n";
  }

  if (save_ply_) {
    std::ofstream ply_stream(base_path + ".ply");
    if (!ply_stream.is_open()) {
      throw std::runtime_error("Failed to write gaussian ply artifact");
    }

    ply_stream
      << "ply\n"
      << "format ascii 1.0\n"
      << "element vertex " << voxels_.size() << "\n"
      << "property float x\n"
      << "property float y\n"
      << "property float z\n"
      << "property uchar red\n"
      << "property uchar green\n"
      << "property uchar blue\n"
      << "property float sigma\n"
      << "property uint weight\n"
      << "end_header\n";

    for (const auto & [key, voxel] : voxels_) {
      (void)key;
      const double inverse_count = 1.0 / static_cast<double>(voxel.count);
      const double sigma =
        std::max(voxel_size_meters_ * 0.5, std::sqrt(voxel.sum_squared_radius * inverse_count));
      ply_stream
        << voxel.sum_x * inverse_count << " "
        << voxel.sum_y * inverse_count << " "
        << voxel.sum_z * inverse_count << " "
        << static_cast<int>(std::lround(voxel.sum_r * inverse_count)) << " "
        << static_cast<int>(std::lround(voxel.sum_g * inverse_count)) << " "
        << static_cast<int>(std::lround(voxel.sum_b * inverse_count)) << " "
        << sigma << " "
        << voxel.count << "\n";
    }
  }

  artifact_dirty_ = false;
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    10000,
    "Saved gaussian voxel preview artifacts to %s.[json|ply]",
    base_path.c_str());
}

void GaussianSplatCaptureNode::upsertVoxel(
  const double x,
  const double y,
  const double z,
  const std::uint8_t red,
  const std::uint8_t green,
  const std::uint8_t blue)
{
  const GaussianVoxelKey key = voxelKeyForPoint(x, y, z);
  const auto voxel_it = voxels_.find(key);
  if (voxel_it == voxels_.end() && static_cast<int>(voxels_.size()) >= max_voxels_) {
    return;
  }

  GaussianVoxel & voxel = voxels_[key];
  voxel.sum_x += x;
  voxel.sum_y += y;
  voxel.sum_z += z;
  voxel.sum_r += static_cast<double>(red);
  voxel.sum_g += static_cast<double>(green);
  voxel.sum_b += static_cast<double>(blue);

  const double center_x = (static_cast<double>(key.x) + 0.5) * voxel_size_meters_;
  const double center_y = (static_cast<double>(key.y) + 0.5) * voxel_size_meters_;
  const double center_z = (static_cast<double>(key.z) + 0.5) * voxel_size_meters_;
  const double offset_x = x - center_x;
  const double offset_y = y - center_y;
  const double offset_z = z - center_z;
  voxel.sum_squared_radius +=
    (offset_x * offset_x) + (offset_y * offset_y) + (offset_z * offset_z);
  ++voxel.count;
}

GaussianVoxelKey GaussianSplatCaptureNode::voxelKeyForPoint(
  const double x, const double y,
  const double z) const
{
  return {
    static_cast<int>(std::floor(x / voxel_size_meters_)),
    static_cast<int>(std::floor(y / voxel_size_meters_)),
    static_cast<int>(std::floor(z / voxel_size_meters_))};
}

std::string GaussianSplatCaptureNode::artifactBasePath() const
{
  namespace fs = std::filesystem;
  return (fs::path(output_directory_) / representation_name_).string();
}

}  // namespace amr_sweeper_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_mapping::GaussianSplatCaptureNode>());
  rclcpp::shutdown();
  return 0;
}
