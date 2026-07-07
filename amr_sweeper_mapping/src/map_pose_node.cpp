#include "map_pose_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace amr_sweeper_mapping
{

namespace
{

geometry_msgs::msg::TransformStamped stampedFromTransform(
  const tf2::Transform & transform,
  const rclcpp::Time & stamp,
  const std::string & parent_frame,
  const std::string & child_frame)
{
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = parent_frame;
  message.child_frame_id = child_frame;
  message.transform = tf2::toMsg(transform);
  return message;
}

double yawFromTransform(const tf2::Transform & transform)
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
  return yaw;
}

double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool worldToGrid(
  const nav_msgs::msg::OccupancyGrid & map,
  const double world_x,
  const double world_y,
  int & grid_x,
  int & grid_y)
{
  if (map.info.resolution <= 0.0F || map.info.width == 0U || map.info.height == 0U) {
    return false;
  }

  const double cell_x = (world_x - map.info.origin.position.x) / map.info.resolution;
  const double cell_y = (world_y - map.info.origin.position.y) / map.info.resolution;
  if (cell_x < 0.0 || cell_y < 0.0) {
    return false;
  }

  grid_x = static_cast<int>(std::floor(cell_x));
  grid_y = static_cast<int>(std::floor(cell_y));
  if (
    grid_x < 0 || grid_y < 0 ||
    grid_x >= static_cast<int>(map.info.width) ||
    grid_y >= static_cast<int>(map.info.height))
  {
    return false;
  }

  return true;
}

tf2::Transform transformFromXYYaw(
  const double x,
  const double y,
  const double z,
  const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  quaternion.normalize();
  return tf2::Transform(quaternion, tf2::Vector3(x, y, z));
}

tf2::Transform blendTransforms(
  const tf2::Transform & anchor,
  const tf2::Transform & candidate,
  const double alpha)
{
  const double clamped_alpha = std::clamp(alpha, 0.0, 1.0);
  const tf2::Vector3 anchor_origin = anchor.getOrigin();
  const tf2::Vector3 candidate_origin = candidate.getOrigin();
  const tf2::Vector3 blended_origin =
    anchor_origin + (candidate_origin - anchor_origin) * clamped_alpha;
  const double anchor_yaw = yawFromTransform(anchor);
  const double candidate_yaw = yawFromTransform(candidate);
  const double blended_yaw =
    normalizeAngle(anchor_yaw + normalizeAngle(candidate_yaw - anchor_yaw) * clamped_alpha);
  return transformFromXYYaw(
    blended_origin.x(),
    blended_origin.y(),
    blended_origin.z(),
    blended_yaw);
}

double geographicDistanceMeters(
  const geographic_msgs::msg::GeoPoint & first,
  const geographic_msgs::msg::GeoPoint & second)
{
  constexpr double earth_radius_m = 6378137.0;
  constexpr double degrees_to_radians = M_PI / 180.0;
  const double mean_latitude_rad =
    ((first.latitude + second.latitude) * 0.5) * degrees_to_radians;
  const double delta_latitude_m =
    (second.latitude - first.latitude) * degrees_to_radians * earth_radius_m;
  const double delta_longitude_m =
    (second.longitude - first.longitude) * degrees_to_radians * earth_radius_m *
    std::cos(mean_latitude_rad);
  return std::hypot(delta_latitude_m, delta_longitude_m);
}

const char * healthStateToString(const MapPoseHealthState state)
{
  switch (state) {
    case MapPoseHealthState::BOOTSTRAP:
      return "BOOTSTRAP";
    case MapPoseHealthState::DEGRADED:
      return "DEGRADED";
    case MapPoseHealthState::HEALTHY:
      return "HEALTHY";
    case MapPoseHealthState::WARNING:
      return "WARNING";
    case MapPoseHealthState::FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

std::string trim(const std::string & value)
{
  const auto begin = value.find_first_not_of(" \t");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t");
  return value.substr(begin, end - begin + 1U);
}

std::string stripQuotes(const std::string & value)
{
  if (value.size() >= 2U) {
    const char first = value.front();
    const char last = value.back();
    if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
      return value.substr(1U, value.size() - 2U);
    }
  }
  return value;
}

nav_msgs::msg::OccupancyGrid loadOccupancyGridFromYaml(
  const std::string & yaml_path,
  const std::string & frame_id)
{
  std::ifstream yaml_stream(yaml_path);
  if (!yaml_stream.is_open()) {
    throw std::runtime_error("Failed to open costmap YAML artifact: " + yaml_path);
  }

  std::string image_name;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::string mode = "trinary";
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
  bool negate = false;
  std::string line;
  while (std::getline(yaml_stream, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trim(line.substr(0, colon));
    const std::string value = trim(line.substr(colon + 1U));
    if (key == "image") {
      image_name = stripQuotes(value);
    } else if (key == "mode") {
      mode = stripQuotes(value);
    } else if (key == "resolution") {
      resolution = std::stod(value);
    } else if (key == "origin") {
      const auto open = value.find('[');
      const auto first_comma = value.find(',', open + 1U);
      const auto second_comma = value.find(',', first_comma + 1U);
      origin_x = std::stod(trim(value.substr(open + 1U, first_comma - open - 1U)));
      origin_y = std::stod(trim(value.substr(first_comma + 1U, second_comma - first_comma - 1U)));
    } else if (key == "occupied_thresh") {
      occupied_thresh = std::stod(value);
    } else if (key == "free_thresh") {
      free_thresh = std::stod(value);
    } else if (key == "negate") {
      negate = std::stoi(value) != 0;
    }
  }

  const std::filesystem::path image_path = std::filesystem::path(yaml_path).parent_path() / image_name;
  std::ifstream image_stream(image_path, std::ios::binary);
  if (!image_stream.is_open()) {
    throw std::runtime_error("Failed to open costmap image artifact: " + image_path.string());
  }

  std::string magic;
  image_stream >> magic;
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error("Unsupported costmap image format: " + magic);
  }

  unsigned int width = 0U;
  unsigned int height = 0U;
  int max_value = 0;
  image_stream >> width >> height >> max_value;

  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = frame_id;
  map.info.resolution = static_cast<float>(resolution);
  map.info.width = width;
  map.info.height = height;
  map.info.origin.position.x = origin_x;
  map.info.origin.position.y = origin_y;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(static_cast<std::size_t>(width) * height, -1);

  auto to_cost = [max_value, negate, mode, occupied_thresh, free_thresh](const int pixel_value) -> int8_t {
      const int bounded = std::clamp(pixel_value, 0, std::max(1, max_value));
      const double normalized = static_cast<double>(bounded) / static_cast<double>(std::max(1, max_value));
      const double occupied = negate ? normalized : (1.0 - normalized);
      if (mode == "scale" || mode == "raw") {
        return static_cast<int8_t>(std::lround(std::clamp(occupied, 0.0, 1.0) * 100.0));
      }
      if (occupied >= occupied_thresh) {
        return 100;
      }
      if (occupied <= free_thresh) {
        return 0;
      }
      return -1;
    };

  if (magic == "P5") {
    image_stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    for (int row = static_cast<int>(height) - 1; row >= 0; --row) {
      for (unsigned int col = 0; col < width; ++col) {
        unsigned char pixel = 0U;
        image_stream.read(reinterpret_cast<char *>(&pixel), 1);
        const std::size_t index = static_cast<std::size_t>(row) * width + col;
        map.data[index] = to_cost(static_cast<int>(pixel));
      }
    }
    return map;
  }

  for (int row = static_cast<int>(height) - 1; row >= 0; --row) {
    for (unsigned int col = 0; col < width; ++col) {
      int pixel_value = 0;
      image_stream >> pixel_value;
      const std::size_t index = static_cast<std::size_t>(row) * width + col;
      map.data[index] = to_cost(pixel_value);
    }
  }

  return map;
}

}  // namespace

MapPoseNode::MapPoseNode()
: Node("map_pose_node")
{
  declare_parameter("map_frame", std::string("map"));
  declare_parameter("odom_frame", std::string("odom"));
  declare_parameter("base_frame", std::string("base_footprint"));
  declare_parameter("odometry_topic", std::string("localization/odometry_fused"));
  declare_parameter("navsat_topic", std::string("gnss/navsat"));
  declare_parameter("heading_topic", std::string("imu/data_heading"));
  declare_parameter("scan_topic", std::string("depth_camera/scan"));
  declare_parameter("global_costmap_topic", std::string("mapping/global_costmap"));
  declare_parameter("status_topic", std::string("mapping/map_pose_status"));
  declare_parameter("fromll_service", std::string("/fromLL"));
  declare_parameter("costmap_yaml_path", std::string(""));
  declare_parameter("publish_period_seconds", 0.5);
  declare_parameter("publish_identity_when_pose_missing", true);
  declare_parameter("occupied_threshold", 65);
  declare_parameter("scan_subsample_step", 4);
  declare_parameter("min_valid_scan_points", 20);
  declare_parameter("endpoint_search_radius_cells", 1);
  declare_parameter("max_scan_match_points", 72);
  declare_parameter("max_match_compute_time_ms", 30);
  declare_parameter("max_scan_match_range_m", 2.0);
  declare_parameter("search_translation_window_m", 2.0);
  declare_parameter("search_translation_step_m", 0.25);
  declare_parameter("search_yaw_window_rad", 0.35);
  declare_parameter("search_yaw_step_rad", 0.0872664626);
  declare_parameter("min_search_translation_window_m", 0.2);
  declare_parameter("min_search_yaw_window_rad", 0.0872664626);
  declare_parameter("search_window_translation_covariance_scale", 0.5);
  declare_parameter("search_window_yaw_covariance_scale", 0.5);
  declare_parameter("translation_penalty_per_meter", 0.5);
  declare_parameter("yaw_penalty_per_rad", 0.25);
  declare_parameter("free_space_penalty", 0.35);
  declare_parameter("free_space_reward", 0.02);
  declare_parameter("occupied_reward", 1.0);
  declare_parameter("occupied_penalty", 0.3);
  declare_parameter("likelihood_field_max_distance_m", 0.75);
  declare_parameter("likelihood_field_sigma_m", 0.12);
  declare_parameter("unknown_space_score", 0.05);
  declare_parameter("out_of_bounds_score", -0.25);
  declare_parameter("prior_blend_weight", 0.7);
  declare_parameter("scan_timeout_seconds", 1.0);
  declare_parameter("costmap_timeout_seconds", 2.0);
  declare_parameter("odometry_timeout_seconds", 1.0);
  declare_parameter("scan_match_period_seconds", 0.5);
  declare_parameter("global_costmap_min_update_period_seconds", 0.5);
  declare_parameter("startup_ready_streak_required", 3);
  declare_parameter("degraded_streak_before_warning", 3);
  declare_parameter("degraded_streak_before_fault", 6);
  declare_parameter("max_translation_jump_m", 0.75);
  declare_parameter("max_yaw_jump_rad", 0.35);
  declare_parameter("transform_smoothing_alpha", 0.35);
  declare_parameter("minimum_match_score", 0.15);
  declare_parameter("high_confidence_match_score", 0.75);
  declare_parameter("minimum_confidence_for_filter_update", 0.45);
  declare_parameter("low_confidence_identity_pull_alpha", 0.15);
  declare_parameter("georef_consistency_max_error_m", 5.0);
  declare_parameter("georef_consistency_min_confidence", 0.2);
  declare_parameter("process_noise_diagonal", std::vector<double>{0.01, 0.01, 0.005});
  declare_parameter("measurement_noise_diagonal_min", std::vector<double>{0.05, 0.05, 0.02});
  declare_parameter("measurement_noise_diagonal_max", std::vector<double>{0.6, 0.6, 0.3});

  map_frame_id_ = get_parameter("map_frame").as_string();
  odom_frame_id_ = get_parameter("odom_frame").as_string();
  base_frame_id_ = get_parameter("base_frame").as_string();
  odometry_topic_ = get_parameter("odometry_topic").as_string();
  navsat_topic_ = get_parameter("navsat_topic").as_string();
  heading_topic_ = get_parameter("heading_topic").as_string();
  scan_topic_ = get_parameter("scan_topic").as_string();
  global_costmap_topic_ = get_parameter("global_costmap_topic").as_string();
  status_topic_ = get_parameter("status_topic").as_string();
  fromll_service_name_ = get_parameter("fromll_service").as_string();
  costmap_yaml_path_ = get_parameter("costmap_yaml_path").as_string();
  publish_identity_when_pose_missing_ =
    get_parameter("publish_identity_when_pose_missing").as_bool();
  occupied_threshold_ = static_cast<int>(get_parameter("occupied_threshold").as_int());
  scan_subsample_step_ = std::max(
    1,
    static_cast<int>(get_parameter("scan_subsample_step").as_int()));
  min_valid_scan_points_ = std::max(
    1,
    static_cast<int>(get_parameter("min_valid_scan_points").as_int()));
  endpoint_search_radius_cells_ = std::max(
    0,
    static_cast<int>(get_parameter("endpoint_search_radius_cells").as_int()));
  max_scan_match_points_ = std::max(
    min_valid_scan_points_,
    static_cast<int>(get_parameter("max_scan_match_points").as_int()));
  max_match_compute_time_ms_ = std::max(
    1,
    static_cast<int>(get_parameter("max_match_compute_time_ms").as_int()));
  max_scan_match_range_m_ = std::max(
    0.05,
    get_parameter("max_scan_match_range_m").as_double());
  search_translation_window_m_ = get_parameter("search_translation_window_m").as_double();
  search_translation_step_m_ = std::max(
    0.05,
    get_parameter("search_translation_step_m").as_double());
  search_yaw_window_rad_ = get_parameter("search_yaw_window_rad").as_double();
  search_yaw_step_rad_ = std::max(0.01, get_parameter("search_yaw_step_rad").as_double());
  min_search_translation_window_m_ = std::max(
    0.05,
    get_parameter("min_search_translation_window_m").as_double());
  min_search_yaw_window_rad_ = std::max(
    0.01,
    get_parameter("min_search_yaw_window_rad").as_double());
  search_window_translation_covariance_scale_ = std::max(
    0.0,
    get_parameter("search_window_translation_covariance_scale").as_double());
  search_window_yaw_covariance_scale_ = std::max(
    0.0,
    get_parameter("search_window_yaw_covariance_scale").as_double());
  translation_penalty_per_meter_ = get_parameter("translation_penalty_per_meter").as_double();
  yaw_penalty_per_rad_ = get_parameter("yaw_penalty_per_rad").as_double();
  free_space_penalty_ = get_parameter("free_space_penalty").as_double();
  free_space_reward_ = get_parameter("free_space_reward").as_double();
  occupied_reward_ = get_parameter("occupied_reward").as_double();
  occupied_penalty_ = get_parameter("occupied_penalty").as_double();
  likelihood_field_max_distance_m_ = std::max(
    0.05,
    get_parameter("likelihood_field_max_distance_m").as_double());
  likelihood_field_sigma_m_ = std::max(
    1.0e-3,
    get_parameter("likelihood_field_sigma_m").as_double());
  unknown_space_score_ = get_parameter("unknown_space_score").as_double();
  out_of_bounds_score_ = get_parameter("out_of_bounds_score").as_double();
  prior_blend_weight_ = std::clamp(get_parameter("prior_blend_weight").as_double(), 0.0, 1.0);
  scan_timeout_seconds_ = std::max(0.05, get_parameter("scan_timeout_seconds").as_double());
  costmap_timeout_seconds_ = std::max(0.05, get_parameter("costmap_timeout_seconds").as_double());
  odometry_timeout_seconds_ = std::max(
    0.05,
    get_parameter("odometry_timeout_seconds").as_double());
  scan_match_period_seconds_ = std::max(
    0.0,
    get_parameter("scan_match_period_seconds").as_double());
  global_costmap_min_update_period_seconds_ = std::max(
    0.0,
    get_parameter("global_costmap_min_update_period_seconds").as_double());
  startup_ready_streak_required_ = std::max(
    1,
    static_cast<int>(get_parameter("startup_ready_streak_required").as_int()));
  degraded_streak_before_warning_ = std::max(
    1,
    static_cast<int>(get_parameter("degraded_streak_before_warning").as_int()));
  degraded_streak_before_fault_ = std::max(
    degraded_streak_before_warning_,
    static_cast<int>(get_parameter("degraded_streak_before_fault").as_int()));
  max_translation_jump_m_ = std::max(0.0, get_parameter("max_translation_jump_m").as_double());
  max_yaw_jump_rad_ = std::max(0.0, get_parameter("max_yaw_jump_rad").as_double());
  transform_smoothing_alpha_ = std::clamp(
    get_parameter("transform_smoothing_alpha").as_double(),
    0.0,
    1.0);
  minimum_match_score_ = get_parameter("minimum_match_score").as_double();
  high_confidence_match_score_ = std::max(
    minimum_match_score_ + 1.0e-6,
    get_parameter("high_confidence_match_score").as_double());
  minimum_confidence_for_filter_update_ = std::clamp(
    get_parameter("minimum_confidence_for_filter_update").as_double(),
    0.0,
    1.0);
  low_confidence_identity_pull_alpha_ = std::clamp(
    get_parameter("low_confidence_identity_pull_alpha").as_double(),
    0.0,
    1.0);
  georef_consistency_max_error_m_ = std::max(
    0.1,
    get_parameter("georef_consistency_max_error_m").as_double());
  georef_consistency_min_confidence_ = std::clamp(
    get_parameter("georef_consistency_min_confidence").as_double(),
    0.0,
    1.0);
  const auto process_noise_values = get_parameter("process_noise_diagonal").as_double_array();
  const auto measurement_noise_min_values =
    get_parameter("measurement_noise_diagonal_min").as_double_array();
  const auto measurement_noise_max_values =
    get_parameter("measurement_noise_diagonal_max").as_double_array();
  if (process_noise_values.size() == 3U) {
    process_noise_diagonal_ = {
      process_noise_values[0],
      process_noise_values[1],
      process_noise_values[2]};
  }
  if (measurement_noise_min_values.size() == 3U) {
    measurement_noise_diagonal_min_ = {
      measurement_noise_min_values[0],
      measurement_noise_min_values[1],
      measurement_noise_min_values[2]};
  }
  if (measurement_noise_max_values.size() == 3U) {
    measurement_noise_diagonal_max_ = {
      measurement_noise_max_values[0],
      measurement_noise_max_values[1],
      measurement_noise_max_values[2]};
  }
  global_costmap_wait_started_ = now();
  loadCostmapGeoreference();
  loadReferenceCostmapFromYaml();

  subscription_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  publish_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = subscription_callback_group_;

  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_,
    50,
    std::bind(&MapPoseNode::handleOdometry, this, std::placeholders::_1),
    subscription_options);
  navsat_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    navsat_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MapPoseNode::handleNavSat, this, std::placeholders::_1),
    subscription_options);
  const auto scan_qos = rclcpp::SensorDataQoS().keep_last(5);

  heading_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
    heading_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&MapPoseNode::handleHeading, this, std::placeholders::_1),
    subscription_options);
  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    scan_qos,
    std::bind(&MapPoseNode::handleScan, this, std::placeholders::_1),
    subscription_options);
  global_costmap_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    global_costmap_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MapPoseNode::handleGlobalCostmap, this, std::placeholders::_1),
    subscription_options);
  status_publisher_ = create_publisher<std_msgs::msg::String>(
    status_topic_,
    rclcpp::QoS(1).reliable().transient_local());
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  fromll_client_ = create_client<fusioncore_ros::srv::FromLL>(fromll_service_name_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("publish_period_seconds").as_double()),
    std::bind(&MapPoseNode::publishMapToOdomTransform, this),
    publish_callback_group_);
  global_costmap_worker_ = std::thread(&MapPoseNode::globalCostmapWorkerLoop, this);
}

MapPoseNode::~MapPoseNode()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    shutdown_global_costmap_worker_ = true;
  }
  global_costmap_worker_cv_.notify_all();
  if (global_costmap_worker_.joinable()) {
    global_costmap_worker_.join();
  }
}

void MapPoseNode::initializeMapToOdomFilter()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  initializeMapToOdomFilterLocked();
}

void MapPoseNode::initializeMapToOdomFilterLocked()
{
  map_to_odom_filter_state_ = {0.0, 0.0, 0.0};
  map_to_odom_filter_covariance_ = {
    measurement_noise_diagonal_max_[0],
    measurement_noise_diagonal_max_[1],
    measurement_noise_diagonal_max_[2]};
  last_map_to_odom_ = transformFromXYYaw(0.0, 0.0, 0.0, 0.0);
  last_map_to_odom_ready_ = true;
  map_to_odom_filter_ready_ = true;
}

void MapPoseNode::predictMapToOdomFilter()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  predictMapToOdomFilterLocked();
}

void MapPoseNode::predictMapToOdomFilterLocked()
{
  if (!map_to_odom_filter_ready_) {
    initializeMapToOdomFilterLocked();
  }

  for (std::size_t index = 0U; index < map_to_odom_filter_covariance_.size(); ++index) {
    map_to_odom_filter_covariance_[index] += std::max(1.0e-6, process_noise_diagonal_[index]);
  }
}

void MapPoseNode::updateMapToOdomFilter(const tf2::Transform & measurement, const double confidence)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  updateMapToOdomFilterLocked(measurement, confidence);
}

void MapPoseNode::updateMapToOdomFilterLocked(
  const tf2::Transform & measurement,
  const double confidence)
{
  if (!map_to_odom_filter_ready_) {
    initializeMapToOdomFilterLocked();
  }

  const double clamped_confidence = std::clamp(confidence, 0.0, 1.0);
  const std::array<double, 3> measurement_state{
    measurement.getOrigin().x(),
    measurement.getOrigin().y(),
    yawFromTransform(measurement)};

  for (std::size_t index = 0U; index < 3U; ++index) {
    const double measurement_noise =
      measurement_noise_diagonal_max_[index] -
      (measurement_noise_diagonal_max_[index] - measurement_noise_diagonal_min_[index]) *
      clamped_confidence;
    const double kalman_gain =
      map_to_odom_filter_covariance_[index] /
      (map_to_odom_filter_covariance_[index] + std::max(1.0e-6, measurement_noise));

    double innovation = measurement_state[index] - map_to_odom_filter_state_[index];
    if (index == 2U) {
      innovation = normalizeAngle(innovation);
    }

    map_to_odom_filter_state_[index] += kalman_gain * innovation;
    if (index == 2U) {
      map_to_odom_filter_state_[index] = normalizeAngle(map_to_odom_filter_state_[index]);
    }
    map_to_odom_filter_covariance_[index] *= (1.0 - kalman_gain);
  }
}

void MapPoseNode::decayMapToOdomFilterTowardsIdentity()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  decayMapToOdomFilterTowardsIdentityLocked();
}

void MapPoseNode::decayMapToOdomFilterTowardsIdentityLocked()
{
  if (!map_to_odom_filter_ready_) {
    initializeMapToOdomFilterLocked();
  }

  const double keep_weight = 1.0 - low_confidence_identity_pull_alpha_;
  map_to_odom_filter_state_[0] *= keep_weight;
  map_to_odom_filter_state_[1] *= keep_weight;
  map_to_odom_filter_state_[2] = normalizeAngle(map_to_odom_filter_state_[2] * keep_weight);

  for (std::size_t index = 0U; index < 3U; ++index) {
    map_to_odom_filter_covariance_[index] = std::min(
      measurement_noise_diagonal_max_[index],
      map_to_odom_filter_covariance_[index] + process_noise_diagonal_[index]);
  }
}

tf2::Transform MapPoseNode::filteredMapToOdomTransform() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return filteredMapToOdomTransform(map_to_odom_filter_state_);
}

tf2::Transform MapPoseNode::filteredMapToOdomTransform(
  const std::array<double, 3> & filter_state) const
{
  return transformFromXYYaw(
    filter_state[0],
    filter_state[1],
    0.0,
    filter_state[2]);
}

MapPoseNode::StateSnapshot MapPoseNode::snapshotState() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  StateSnapshot snapshot;
  snapshot.latest_odometry_ready = latest_odometry_ready_;
  snapshot.latest_navsat_ready = latest_navsat_ready_;
  snapshot.latest_heading_ready = latest_heading_ready_;
  snapshot.latest_scan_ready = latest_scan_ready_;
  snapshot.latest_global_costmap_ready = latest_global_costmap_ready_;
  snapshot.last_map_to_odom_ready = last_map_to_odom_ready_;
  snapshot.map_to_odom_filter_ready = map_to_odom_filter_ready_;
  snapshot.correction_startup_ready = correction_startup_ready_;
  snapshot.correction_ready_streak = correction_ready_streak_;
  snapshot.latest_odometry_position = latest_odometry_position_;
  snapshot.latest_odometry_orientation = latest_odometry_orientation_;
  snapshot.latest_navsat = latest_navsat_;
  snapshot.latest_heading = latest_heading_;
  snapshot.latest_scan = latest_scan_;
  snapshot.latest_global_costmap = latest_global_costmap_;
  snapshot.latest_global_costmap_score_field = latest_global_costmap_score_field_;
  snapshot.latest_odometry_stamp = latest_odometry_stamp_;
  snapshot.latest_navsat_stamp = latest_navsat_stamp_;
  snapshot.latest_heading_stamp = latest_heading_stamp_;
  snapshot.latest_scan_stamp = latest_scan_stamp_;
  snapshot.latest_global_costmap_stamp = latest_global_costmap_stamp_;
  snapshot.latest_map_pose_stamp = latest_map_pose_stamp_;
  snapshot.map_to_odom_filter_state = map_to_odom_filter_state_;
  snapshot.map_to_odom_filter_covariance = map_to_odom_filter_covariance_;
  snapshot.last_map_to_odom = last_map_to_odom_;
  return snapshot;
}

bool MapPoseNode::odometryInputReady(const StateSnapshot & snapshot) const
{
  if (!snapshot.latest_odometry_ready) {
    return false;
  }

  if (snapshot.latest_odometry_stamp.nanoseconds() == 0) {
    return false;
  }

  return (now() - snapshot.latest_odometry_stamp).seconds() <= odometry_timeout_seconds_;
}

bool MapPoseNode::correctionInputsReady(const StateSnapshot & snapshot) const
{
  if (!odometryInputReady(snapshot) || !snapshot.latest_scan_ready || !snapshot.latest_global_costmap_ready) {
    return false;
  }

  if ((now() - snapshot.latest_scan_stamp).seconds() > scan_timeout_seconds_) {
    return false;
  }

  if ((now() - snapshot.latest_global_costmap_stamp).seconds() > costmap_timeout_seconds_) {
    return false;
  }

  if (snapshot.latest_scan.header.frame_id.empty()) {
    return false;
  }

  try {
    tf_buffer_->lookupTransform(
      base_frame_id_,
      snapshot.latest_scan.header.frame_id,
      tf2::TimePointZero,
      tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException &) {
    return false;
  }

  return true;
}

bool MapPoseNode::waitingForInitialGlobalCostmap(const StateSnapshot & snapshot) const
{
  return !snapshot.latest_global_costmap_ready && !initialGlobalCostmapWaitTimedOut(snapshot);
}

bool MapPoseNode::initialGlobalCostmapWaitTimedOut(const StateSnapshot & snapshot) const
{
  if (snapshot.latest_global_costmap_ready) {
    return false;
  }
  if (global_costmap_wait_started_.nanoseconds() == 0) {
    return false;
  }
  return (now() - global_costmap_wait_started_).seconds() > costmap_timeout_seconds_;
}

std::string MapPoseNode::composeHealthReason(const StateSnapshot & snapshot) const
{
  if (!snapshot.latest_odometry_ready) {
    if (!snapshot.correction_startup_ready) {
      return "odometry_waiting";
    }
    return "odometry_missing";
  }
  if (snapshot.latest_odometry_stamp.nanoseconds() == 0) {
    if (!snapshot.correction_startup_ready) {
      return "odometry_waiting";
    }
    return "odometry_stamp_missing";
  }
  if ((now() - snapshot.latest_odometry_stamp).seconds() > odometry_timeout_seconds_) {
    if (!snapshot.correction_startup_ready) {
      return "odometry_waiting";
    }
    return "odometry_stale";
  }
  if (!snapshot.latest_scan_ready) {
    return "scan_missing";
  }
  if ((now() - snapshot.latest_scan_stamp).seconds() > scan_timeout_seconds_) {
    return "scan_stale";
  }
  if (!snapshot.latest_global_costmap_ready) {
    if (initialGlobalCostmapWaitTimedOut(snapshot)) {
      return "global_costmap_missing";
    }
    return "global_costmap_waiting";
  }
  if ((now() - snapshot.latest_global_costmap_stamp).seconds() > costmap_timeout_seconds_) {
    return "global_costmap_stale";
  }
  if (snapshot.latest_scan.header.frame_id.empty()) {
    return "scan_frame_missing";
  }
  try {
    tf_buffer_->lookupTransform(
      base_frame_id_,
      snapshot.latest_scan.header.frame_id,
      tf2::TimePointZero,
      tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException &) {
    return "scan_to_base_tf_missing";
  }

  if (!snapshot.correction_startup_ready) {
    return "startup_holdoff";
  }

  return "corrections_healthy";
}

bool MapPoseNode::shouldHoldIdentityAtStartup(const StateSnapshot & snapshot)
{
  if (snapshot.correction_startup_ready) {
    return false;
  }

  const bool inputs_ready = correctionInputsReady(snapshot);
  int updated_streak = 0;
  bool startup_ready = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (correction_startup_ready_) {
      return false;
    }
    if (inputs_ready) {
      correction_ready_streak_ = std::min(
      correction_ready_streak_ + 1,
      startup_ready_streak_required_);
    } else {
      correction_ready_streak_ = 0;
    }

    if (correction_ready_streak_ >= startup_ready_streak_required_) {
      correction_startup_ready_ = true;
      startup_ready = true;
    }

    updated_streak = correction_ready_streak_;
  }

  if (startup_ready) {
    RCLCPP_INFO(
      get_logger(),
      "Map pose startup holdoff cleared after %d consecutive ready cycles; enabling runtime map -> odom corrections.",
      startup_ready_streak_required_);
    return false;
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    3000,
    "Holding map -> odom at identity until scan, TF, and global map inputs stay ready for %d consecutive publish cycles (%d/%d).",
    startup_ready_streak_required_,
    updated_streak,
    startup_ready_streak_required_);
  return true;
}

void MapPoseNode::publishMapPoseStatus(const StateSnapshot & snapshot)
{
  const bool odometry_ready = odometryInputReady(snapshot);
  const bool correction_ready = correctionInputsReady(snapshot);
  const bool startup_cleared = snapshot.correction_startup_ready;
  const bool waiting_for_initial_costmap = waitingForInitialGlobalCostmap(snapshot);
  const std::string reason = composeHealthReason(snapshot);

  MapPoseHealthState next_state = MapPoseHealthState::BOOTSTRAP;
  int degraded_streak = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!odometry_ready) {
      if (!startup_cleared) {
        degraded_input_streak_ = 0;
        next_state = MapPoseHealthState::BOOTSTRAP;
      } else {
        degraded_input_streak_ = std::max(degraded_input_streak_, degraded_streak_before_fault_);
        next_state = MapPoseHealthState::FAULT;
      }
    } else if (waiting_for_initial_costmap) {
      degraded_input_streak_ = 0;
      next_state = MapPoseHealthState::BOOTSTRAP;
    } else if (!startup_cleared && snapshot.latest_global_costmap_ready) {
      degraded_input_streak_ = 0;
      next_state = MapPoseHealthState::BOOTSTRAP;
    } else if (correction_ready) {
      degraded_input_streak_ = 0;
      next_state = MapPoseHealthState::HEALTHY;
    } else {
      degraded_input_streak_ = std::min(degraded_input_streak_ + 1, degraded_streak_before_fault_);
      if (degraded_input_streak_ >= degraded_streak_before_fault_) {
        next_state = MapPoseHealthState::FAULT;
      } else if (degraded_input_streak_ >= degraded_streak_before_warning_) {
        next_state = MapPoseHealthState::WARNING;
      } else {
        next_state = MapPoseHealthState::DEGRADED;
      }
    }
    degraded_streak = degraded_input_streak_;
    if (health_state_ != next_state) {
      RCLCPP_INFO(
        get_logger(),
        "map_pose_node state transition %s -> %s (%s).",
        healthStateToString(health_state_),
        healthStateToString(next_state),
        reason.c_str());
      health_state_ = next_state;
    }
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "state=" << healthStateToString(next_state)
         << "; reason=" << reason
         << "; startup_cleared=" << (startup_cleared ? "true" : "false")
         << "; correction_ready=" << (correction_ready ? "true" : "false")
         << "; correction_ready_streak=" << snapshot.correction_ready_streak
         << "/" << startup_ready_streak_required_
         << "; degraded_streak=" << degraded_streak
         << "/" << degraded_streak_before_fault_
         << "; odometry_ready=" << (odometry_ready ? "true" : "false")
         << "; scan_ready=" << (snapshot.latest_scan_ready ? "true" : "false")
         << "; global_costmap_ready=" << (snapshot.latest_global_costmap_ready ? "true" : "false");
  if (snapshot.latest_odometry_stamp.nanoseconds() > 0) {
    stream << "; odometry_age_s=" << (now() - snapshot.latest_odometry_stamp).seconds();
  }
  if (snapshot.latest_scan_stamp.nanoseconds() > 0) {
    stream << "; scan_age_s=" << (now() - snapshot.latest_scan_stamp).seconds();
  }
  if (snapshot.latest_global_costmap_stamp.nanoseconds() > 0) {
    stream << "; global_costmap_age_s=" << (now() - snapshot.latest_global_costmap_stamp).seconds();
  }

  std_msgs::msg::String status_message;
  status_message.data = stream.str();
  status_publisher_->publish(status_message);
}

void MapPoseNode::loadCostmapGeoreference()
{
  if (costmap_yaml_path_.empty()) {
    return;
  }

  std::ifstream yaml_stream(costmap_yaml_path_);
  if (!yaml_stream.is_open()) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to open costmap YAML %s for georeference metadata.",
      costmap_yaml_path_.c_str());
    return;
  }

  std::string line;
  while (std::getline(yaml_stream, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const auto key = line.substr(0, colon);
    const auto value = line.substr(colon + 1U);
    if (key == "georeference_longitude_coefficients") {
      const auto open = value.find('[');
      const auto first_comma = value.find(',', open + 1U);
      const auto second_comma = value.find(',', first_comma + 1U);
      const auto close = value.find(']', second_comma + 1U);
      artifact_longitude_coefficients_ = {
        std::stod(value.substr(open + 1U, first_comma - open - 1U)),
        std::stod(value.substr(first_comma + 1U, second_comma - first_comma - 1U)),
        std::stod(value.substr(second_comma + 1U, close - second_comma - 1U))};
    } else if (key == "georeference_latitude_coefficients") {
      const auto open = value.find('[');
      const auto first_comma = value.find(',', open + 1U);
      const auto second_comma = value.find(',', first_comma + 1U);
      const auto close = value.find(']', second_comma + 1U);
      artifact_latitude_coefficients_ = {
        std::stod(value.substr(open + 1U, first_comma - open - 1U)),
        std::stod(value.substr(first_comma + 1U, second_comma - first_comma - 1U)),
        std::stod(value.substr(second_comma + 1U, close - second_comma - 1U))};
      artifact_georeference_ready_ = true;
    }
  }
}

void MapPoseNode::loadReferenceCostmapFromYaml()
{
  if (costmap_yaml_path_.empty()) {
    return;
  }

  try {
    nav_msgs::msg::OccupancyGrid reference_map =
      loadOccupancyGridFromYaml(costmap_yaml_path_, map_frame_id_);
    if (reference_map.info.width == 0U || reference_map.info.height == 0U) {
      return;
    }

    auto reference_score_field = buildGlobalCostmapScoreField(reference_map);
    const rclcpp::Time stamp = now();

    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_global_costmap_ = std::move(reference_map);
    latest_global_costmap_score_field_ = std::move(reference_score_field);
    latest_global_costmap_ready_ = true;
    latest_global_costmap_stamp_ = stamp;
    latest_global_costmap_processed_stamp_ = stamp;
    use_reference_costmap_ = true;
    global_costmap_wait_started_ = stamp;
    RCLCPP_INFO(
      get_logger(),
      "Loaded stable reference costmap from %s for map_pose scan matching; live mapping/global_costmap updates will not replace it.",
      costmap_yaml_path_.c_str());
  } catch (const std::exception & exception) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to load reference costmap from %s: %s",
      costmap_yaml_path_.c_str(),
      exception.what());
  }
}

std::shared_ptr<std::vector<float>> MapPoseNode::buildGlobalCostmapScoreField(
  const nav_msgs::msg::OccupancyGrid & map) const
{
  if (
    map.info.width == 0U || map.info.height == 0U ||
    map.info.resolution <= 0.0F)
  {
    return nullptr;
  }

  const std::size_t width = map.info.width;
  const std::size_t height = map.info.height;
  const std::size_t cell_count = width * height;
  auto score_field = std::make_shared<std::vector<float>>(
    cell_count,
    static_cast<float>(out_of_bounds_score_));
  std::vector<float> distances(cell_count, std::numeric_limits<float>::infinity());

  using QueueEntry = std::tuple<float, int, int>;
  auto queue_compare = [](const QueueEntry & left, const QueueEntry & right) {
      return std::get<0>(left) > std::get<0>(right);
    };
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, decltype(queue_compare)> open_set(
    queue_compare);

  auto enqueue_if_better = [&](const int x, const int y, const float distance) {
      if (
        x < 0 || y < 0 || x >= static_cast<int>(width) || y >= static_cast<int>(height) ||
        distance > static_cast<float>(likelihood_field_max_distance_m_))
      {
        return;
      }

      const std::size_t index =
        static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x);
      if (distance >= distances[index]) {
        return;
      }

      distances[index] = distance;
      open_set.emplace(distance, x, y);
    };

  for (std::size_t index = 0U; index < cell_count; ++index) {
    if (map.data.at(index) >= occupied_threshold_) {
      const int x = static_cast<int>(index % width);
      const int y = static_cast<int>(index / width);
      enqueue_if_better(x, y, 0.0F);
    }
  }

  const float resolution = map.info.resolution;
  const float diagonal = resolution * static_cast<float>(std::sqrt(2.0));
  constexpr std::array<int, 8> delta_x{{-1, 0, 1, -1, 1, -1, 0, 1}};
  constexpr std::array<int, 8> delta_y{{-1, -1, -1, 0, 0, 1, 1, 1}};

  while (!open_set.empty()) {
    const auto [distance, x, y] = open_set.top();
    open_set.pop();

    const std::size_t index =
      static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x);
    if (distance > distances[index]) {
      continue;
    }

    for (std::size_t neighbor = 0U; neighbor < delta_x.size(); ++neighbor) {
      const float step_cost =
        delta_x[neighbor] == 0 || delta_y[neighbor] == 0 ? resolution : diagonal;
      enqueue_if_better(x + delta_x[neighbor], y + delta_y[neighbor], distance + step_cost);
    }
  }

  const double gaussian_denom = 2.0 * likelihood_field_sigma_m_ * likelihood_field_sigma_m_;
  for (std::size_t index = 0U; index < cell_count; ++index) {
    const int8_t value = map.data.at(index);
    if (value < 0) {
      score_field->at(index) = static_cast<float>(unknown_space_score_);
      continue;
    }

    const float distance = distances[index];
    if (!std::isfinite(distance)) {
      score_field->at(index) = static_cast<float>(out_of_bounds_score_);
      continue;
    }

    score_field->at(index) = static_cast<float>(
      occupied_reward_ *
      std::exp(-(static_cast<double>(distance) * static_cast<double>(distance)) / gaussian_denom));
  }

  return score_field;
}

void MapPoseNode::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_odometry_position_ = message->pose.pose.position;
  latest_odometry_orientation_ = message->pose.pose.orientation;
  latest_odometry_stamp_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() :
    rclcpp::Time(message->header.stamp);
  if (latest_odometry_stamp_.nanoseconds() > latest_map_pose_stamp_.nanoseconds()) {
    latest_map_pose_stamp_ = latest_odometry_stamp_;
  }
  latest_odometry_ready_ = true;
}

void MapPoseNode::handleNavSat(const sensor_msgs::msg::NavSatFix::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_navsat_ = *message;
  latest_navsat_stamp_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() :
    rclcpp::Time(message->header.stamp);
  latest_navsat_ready_ = true;
}

void MapPoseNode::handleHeading(const sensor_msgs::msg::Imu::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_heading_ = *message;
  latest_heading_stamp_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() :
    rclcpp::Time(message->header.stamp);
  latest_heading_ready_ = true;
}

void MapPoseNode::handleScan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_scan_ = *message;
  latest_scan_stamp_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() :
    rclcpp::Time(message->header.stamp);
  if (latest_scan_stamp_.nanoseconds() > latest_map_pose_stamp_.nanoseconds()) {
    latest_map_pose_stamp_ = latest_scan_stamp_;
  }
  latest_scan_ready_ = true;
}

void MapPoseNode::handleGlobalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  if (use_reference_costmap_) {
    return;
  }

  const rclcpp::Time receipt_stamp = now();
  const bool costmap_ready =
    message->info.width > 0U && message->info.height > 0U && message->info.resolution > 0.0F;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (
      pending_global_costmap_stamp_.nanoseconds() > 0 &&
      pending_global_costmap_stamp_ > receipt_stamp)
    {
      return;
    }
    pending_global_costmap_ = *message;
    pending_global_costmap_stamp_ = receipt_stamp;
    pending_global_costmap_ready_ = costmap_ready;
    pending_global_costmap_dirty_ = true;
  }
  global_costmap_worker_cv_.notify_one();
}

void MapPoseNode::globalCostmapWorkerLoop()
{
  while (true) {
    nav_msgs::msg::OccupancyGrid pending_map;
    rclcpp::Time pending_stamp{0, 0, get_clock()->get_clock_type()};
    bool pending_ready = false;

    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      global_costmap_worker_cv_.wait(lock, [this]() {
        return shutdown_global_costmap_worker_ || pending_global_costmap_dirty_;
      });
      if (shutdown_global_costmap_worker_) {
        return;
      }

      if (
        global_costmap_min_update_period_seconds_ > 0.0 &&
        latest_global_costmap_processed_stamp_.nanoseconds() > 0 &&
        (pending_global_costmap_stamp_ - latest_global_costmap_processed_stamp_).seconds() <
        global_costmap_min_update_period_seconds_)
      {
        pending_global_costmap_dirty_ = false;
        continue;
      }

      pending_map = pending_global_costmap_;
      pending_stamp = pending_global_costmap_stamp_;
      pending_ready = pending_global_costmap_ready_;
      pending_global_costmap_dirty_ = false;
    }

    if (!pending_ready) {
      continue;
    }

    std::shared_ptr<std::vector<float>> score_field = buildGlobalCostmapScoreField(pending_map);

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (shutdown_global_costmap_worker_) {
      return;
    }
    if (
      latest_global_costmap_processed_stamp_.nanoseconds() > 0 &&
      latest_global_costmap_processed_stamp_ > pending_stamp)
    {
      continue;
    }
    latest_global_costmap_ = std::move(pending_map);
    latest_global_costmap_stamp_ = pending_stamp;
    latest_global_costmap_ready_ = pending_ready;
    latest_global_costmap_processed_stamp_ = pending_stamp;
    latest_global_costmap_score_field_ = std::move(score_field);
  }
}

float MapPoseNode::scoreCostmapCell(
  const StateSnapshot & snapshot,
  const int grid_x,
  const int grid_y) const
{
  if (
    grid_x < 0 || grid_y < 0 ||
    grid_x >= static_cast<int>(snapshot.latest_global_costmap.info.width) ||
    grid_y >= static_cast<int>(snapshot.latest_global_costmap.info.height))
  {
    return static_cast<float>(out_of_bounds_score_);
  }

  const std::size_t index =
    static_cast<std::size_t>(grid_y) * snapshot.latest_global_costmap.info.width +
    static_cast<std::size_t>(grid_x);
  if (
    snapshot.latest_global_costmap_score_field &&
    index < snapshot.latest_global_costmap_score_field->size())
  {
    return snapshot.latest_global_costmap_score_field->at(index);
  }

  return static_cast<float>(out_of_bounds_score_);
}

std::optional<geometry_msgs::msg::Point> MapPoseNode::latestMapPositionFromNavSat() const
{
  if (!latest_navsat_ready_) {
    return std::nullopt;
  }

  if (const auto artifact_map_point = mapPositionFromArtifactGeoreference();
    artifact_map_point.has_value())
  {
    return artifact_map_point;
  }

  if (!fromll_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for %s before deriving the georeferenced map pose from %s.",
      fromll_service_name_.c_str(),
      navsat_topic_.c_str());
    return std::nullopt;
  }

  auto request = std::make_shared<fusioncore_ros::srv::FromLL::Request>();
  request->ll_point.latitude = latest_navsat_.latitude;
  request->ll_point.longitude = latest_navsat_.longitude;
  request->ll_point.altitude = latest_navsat_.altitude;

  auto future = fromll_client_->async_send_request(request);
  const auto future_status = future.wait_for(std::chrono::milliseconds(200));
  if (future_status != std::future_status::ready) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Timed out waiting for %s while deriving the georeferenced map pose.",
      fromll_service_name_.c_str());
    return std::nullopt;
  }

  const auto response = future.get();
  geometry_msgs::msg::Point map_point;
  map_point.x = response->map_point.x;
  map_point.y = response->map_point.y;
  map_point.z = response->map_point.z;
  return map_point;
}

std::optional<geometry_msgs::msg::Point> MapPoseNode::mapPositionFromArtifactGeoreference() const
{
  if (!artifact_georeference_ready_) {
    return std::nullopt;
  }

  const double a = artifact_longitude_coefficients_[0];
  const double b = artifact_longitude_coefficients_[1];
  const double c = artifact_longitude_coefficients_[2];
  const double d = artifact_latitude_coefficients_[0];
  const double e = artifact_latitude_coefficients_[1];
  const double f = artifact_latitude_coefficients_[2];
  const double determinant = (a * e) - (b * d);
  if (std::abs(determinant) < 1.0e-12) {
    return std::nullopt;
  }

  const double longitude_delta = latest_navsat_.longitude - c;
  const double latitude_delta = latest_navsat_.latitude - f;
  geometry_msgs::msg::Point map_point;
  map_point.x = ((e * longitude_delta) - (b * latitude_delta)) / determinant;
  map_point.y = ((-d * longitude_delta) + (a * latitude_delta)) / determinant;
  map_point.z = latest_navsat_.altitude;
  return map_point;
}

std::optional<geographic_msgs::msg::GeoPoint> MapPoseNode::artifactGeoPointFromMapPoint(
  const geometry_msgs::msg::Point & map_point) const
{
  if (!artifact_georeference_ready_) {
    return std::nullopt;
  }

  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.longitude =
    artifact_longitude_coefficients_[0] * map_point.x +
    artifact_longitude_coefficients_[1] * map_point.y +
    artifact_longitude_coefficients_[2];
  geo_point.latitude =
    artifact_latitude_coefficients_[0] * map_point.x +
    artifact_latitude_coefficients_[1] * map_point.y +
    artifact_latitude_coefficients_[2];
  geo_point.altitude = map_point.z;
  return geo_point;
}

double MapPoseNode::georeferenceConsistencyConfidence(
  const StateSnapshot & snapshot,
  const tf2::Transform & candidate_map_to_base) const
{
  if (!snapshot.latest_navsat_ready || !snapshot.latest_heading_ready || !artifact_georeference_ready_) {
    return 1.0;
  }

  if (
    snapshot.latest_navsat_stamp.nanoseconds() == 0 ||
    snapshot.latest_heading_stamp.nanoseconds() == 0)
  {
    return 1.0;
  }

  if (
    (now() - snapshot.latest_navsat_stamp).seconds() > costmap_timeout_seconds_ ||
    (now() - snapshot.latest_heading_stamp).seconds() > costmap_timeout_seconds_)
  {
    return 1.0;
  }

  geometry_msgs::msg::Point mapped_pose_point;
  mapped_pose_point.x = candidate_map_to_base.getOrigin().x();
  mapped_pose_point.y = candidate_map_to_base.getOrigin().y();
  mapped_pose_point.z = candidate_map_to_base.getOrigin().z();

  const auto derived_geo_point = artifactGeoPointFromMapPoint(mapped_pose_point);
  if (!derived_geo_point.has_value()) {
    return 1.0;
  }

  geographic_msgs::msg::GeoPoint gnss_geo_point;
  gnss_geo_point.latitude = snapshot.latest_navsat.latitude;
  gnss_geo_point.longitude = snapshot.latest_navsat.longitude;
  gnss_geo_point.altitude = snapshot.latest_navsat.altitude;

  const double position_confidence = std::clamp(
    1.0 - (geographicDistanceMeters(gnss_geo_point, *derived_geo_point) / georef_consistency_max_error_m_),
    georef_consistency_min_confidence_,
    1.0);

  tf2::Quaternion heading_quaternion;
  tf2::fromMsg(snapshot.latest_heading.orientation, heading_quaternion);
  heading_quaternion.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double heading_yaw = 0.0;
  tf2::Matrix3x3(heading_quaternion).getRPY(roll, pitch, heading_yaw);
  const double heading_error = std::abs(normalizeAngle(
    yawFromTransform(candidate_map_to_base) - heading_yaw));
  const double heading_confidence = std::clamp(
    1.0 - (heading_error / std::max(0.1, search_yaw_window_rad_)),
    georef_consistency_min_confidence_,
    1.0);

  return position_confidence * heading_confidence;
}

std::optional<MapPoseNode::MapMatchEstimate> MapPoseNode::estimateMapToBaseFromPrior(
  const StateSnapshot & snapshot,
  const tf2::Transform & map_to_base_prior) const
{
  if (!snapshot.latest_scan_ready || !snapshot.latest_global_costmap_ready) {
    return std::nullopt;
  }

  const auto & scan = snapshot.latest_scan;
  const auto & map = snapshot.latest_global_costmap;
  const auto match_start = std::chrono::steady_clock::now();
  if (scan.ranges.empty()) {
    return std::nullopt;
  }
  if ((now() - snapshot.latest_scan_stamp).seconds() > scan_timeout_seconds_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Skipping map pose scan matching because the latest scan is stale by %.3fs.",
      (now() - snapshot.latest_scan_stamp).seconds());
    return std::nullopt;
  }

  struct ScanEndpoint
  {
    double x;
    double y;
  };

  geometry_msgs::msg::TransformStamped base_from_scan_message;
  try {
    base_from_scan_message = tf_buffer_->lookupTransform(
      base_frame_id_,
      scan.header.frame_id,
      tf2::TimePointZero,
      tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Skipping map pose scan matching because %s -> %s is unavailable: %s",
      scan.header.frame_id.c_str(),
      base_frame_id_.c_str(),
      exception.what());
    return std::nullopt;
  }

  tf2::Transform base_from_scan;
  tf2::fromMsg(base_from_scan_message.transform, base_from_scan);
  std::vector<ScanEndpoint> endpoints;
  endpoints.reserve(scan.ranges.size() / static_cast<std::size_t>(scan_subsample_step_) + 1U);
  const double effective_max_range = std::min(
    scan.range_max > 0.0 ? scan.range_max : std::numeric_limits<double>::infinity(),
    max_scan_match_range_m_);
  for (std::size_t index = 0U; index < scan.ranges.size(); index += static_cast<std::size_t>(scan_subsample_step_)) {
    const double range = static_cast<double>(scan.ranges.at(index));
    if (!std::isfinite(range) || range < scan.range_min || range > effective_max_range) {
      continue;
    }

    const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
    const tf2::Vector3 endpoint_in_base = base_from_scan * tf2::Vector3(
      range * std::cos(angle),
      range * std::sin(angle),
      0.0);
    endpoints.push_back({
      endpoint_in_base.x(),
      endpoint_in_base.y()});
  }

  if (static_cast<int>(endpoints.size()) < min_valid_scan_points_) {
    return std::nullopt;
  }

  if (static_cast<int>(endpoints.size()) > max_scan_match_points_) {
    std::vector<ScanEndpoint> reduced_endpoints;
    reduced_endpoints.reserve(static_cast<std::size_t>(max_scan_match_points_));
    const double endpoint_stride =
      static_cast<double>(endpoints.size()) / static_cast<double>(max_scan_match_points_);
    for (int point_index = 0; point_index < max_scan_match_points_; ++point_index) {
      const std::size_t sample_index = std::min(
        endpoints.size() - 1U,
        static_cast<std::size_t>(std::floor(static_cast<double>(point_index) * endpoint_stride)));
      reduced_endpoints.push_back(endpoints[sample_index]);
    }
    endpoints = std::move(reduced_endpoints);
  }

  const auto sampleEndpoints = [&](const int requested_count) {
      if (requested_count <= 0 || static_cast<int>(endpoints.size()) <= requested_count) {
        return endpoints;
      }
      std::vector<ScanEndpoint> sampled_endpoints;
      sampled_endpoints.reserve(static_cast<std::size_t>(requested_count));
      const double endpoint_stride =
        static_cast<double>(endpoints.size()) / static_cast<double>(requested_count);
      for (int point_index = 0; point_index < requested_count; ++point_index) {
        const std::size_t sample_index = std::min(
          endpoints.size() - 1U,
          static_cast<std::size_t>(std::floor(static_cast<double>(point_index) * endpoint_stride)));
        sampled_endpoints.push_back(endpoints[sample_index]);
      }
      return sampled_endpoints;
    };

  const std::vector<ScanEndpoint> coarse_endpoints = sampleEndpoints(std::max(
      min_valid_scan_points_,
      max_scan_match_points_ / 2));

  double best_score = -std::numeric_limits<double>::infinity();
  tf2::Transform best_transform;
  bool best_transform_ready = false;
  bool search_timed_out = false;
  const tf2::Transform search_center = map_to_base_prior;
  const double search_center_yaw = yawFromTransform(search_center);

  auto scoreCandidate = [&](const std::vector<ScanEndpoint> & candidate_endpoints,
      const double candidate_x, const double candidate_y, const double candidate_yaw) {
      const double cos_yaw = std::cos(candidate_yaw);
      const double sin_yaw = std::sin(candidate_yaw);
      double endpoint_score_sum = 0.0;
      int valid_points = 0;
      const int total_points = static_cast<int>(candidate_endpoints.size());
      const double center_dx = candidate_x - search_center.getOrigin().x();
      const double center_dy = candidate_y - search_center.getOrigin().y();
      const double yaw_delta = normalizeAngle(candidate_yaw - search_center_yaw);
      const double candidate_penalty =
        (translation_penalty_per_meter_ * std::hypot(center_dx, center_dy)) +
        (yaw_penalty_per_rad_ * std::abs(yaw_delta));

      for (const auto & endpoint : candidate_endpoints) {
        const double world_x = candidate_x + endpoint.x * cos_yaw - endpoint.y * sin_yaw;
        const double world_y = candidate_y + endpoint.x * sin_yaw + endpoint.y * cos_yaw;
        int grid_x = 0;
        int grid_y = 0;
        if (!worldToGrid(map, world_x, world_y, grid_x, grid_y)) {
          endpoint_score_sum += out_of_bounds_score_;
          ++valid_points;
          continue;
        }

        endpoint_score_sum += static_cast<double>(scoreCostmapCell(snapshot, grid_x, grid_y));
        ++valid_points;

        const int remaining_points = total_points - valid_points;
        const double optimistic_score =
          ((endpoint_score_sum +
          static_cast<double>(remaining_points) * occupied_reward_) /
          static_cast<double>(total_points)) - candidate_penalty;
        if (optimistic_score <= best_score) {
          return -std::numeric_limits<double>::infinity();
        }
      }

      if (valid_points < min_valid_scan_points_) {
        return -std::numeric_limits<double>::infinity();
      }

      return
        (endpoint_score_sum / static_cast<double>(valid_points)) -
        candidate_penalty;
    };

  const double center_x = search_center.getOrigin().x();
  const double center_y = search_center.getOrigin().y();
  const double center_yaw = yawFromTransform(search_center);
  const double translation_sigma = std::sqrt(std::max(
      snapshot.map_to_odom_filter_covariance[0],
      snapshot.map_to_odom_filter_covariance[1]));
  const double yaw_sigma = std::sqrt(
    std::max(1.0e-6, snapshot.map_to_odom_filter_covariance[2]));
  const double translation_window = std::clamp(
    min_search_translation_window_m_ +
    (search_window_translation_covariance_scale_ * translation_sigma),
    min_search_translation_window_m_,
    search_translation_window_m_);
  const double yaw_window = std::clamp(
    min_search_yaw_window_rad_ +
    (search_window_yaw_covariance_scale_ * yaw_sigma),
    min_search_yaw_window_rad_,
    search_yaw_window_rad_);
  const auto runSearch = [&](const std::vector<ScanEndpoint> & candidate_endpoints,
      const double translation_window, const double translation_step,
      const double yaw_window, const double yaw_step) {
      for (double dx = -translation_window; dx <= translation_window + 1.0e-6; dx += translation_step) {
        for (double dy = -translation_window; dy <= translation_window + 1.0e-6; dy += translation_step) {
          for (double yaw_delta = -yaw_window; yaw_delta <= yaw_window + 1.0e-6; yaw_delta += yaw_step) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - match_start).count();
            if (elapsed_ms >= max_match_compute_time_ms_) {
              search_timed_out = true;
              return;
            }
            const double candidate_x = center_x + dx;
            const double candidate_y = center_y + dy;
            const double candidate_yaw = normalizeAngle(center_yaw + yaw_delta);
            const double candidate_score = scoreCandidate(
              candidate_endpoints,
              candidate_x,
              candidate_y,
              candidate_yaw);
            if (candidate_score <= best_score) {
              continue;
            }
            best_score = candidate_score;
            best_transform = transformFromXYYaw(
              candidate_x,
              candidate_y,
              map_to_base_prior.getOrigin().z(),
              candidate_yaw);
            best_transform_ready = true;
          }
        }
      }
    };

  runSearch(
    coarse_endpoints,
    translation_window,
    search_translation_step_m_,
    yaw_window,
    search_yaw_step_rad_);

  if (search_timed_out) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Skipping map pose scan matching because it exceeded the compute budget of %d ms.",
      max_match_compute_time_ms_);
    return std::nullopt;
  }

  const bool should_run_fine_search =
    best_transform_ready && best_score >= (minimum_match_score_ - 0.05);

  if (should_run_fine_search) {
    const double fine_center_x = best_transform.getOrigin().x();
    const double fine_center_y = best_transform.getOrigin().y();
    const double fine_center_yaw = yawFromTransform(best_transform);
    const double fine_translation_window = std::max(
      search_translation_step_m_,
      search_translation_step_m_ * 2.0);
    const double fine_translation_step = std::max(0.05, search_translation_step_m_ * 0.5);
    const double fine_yaw_window = std::max(search_yaw_step_rad_, search_yaw_step_rad_ * 2.0);
    const double fine_yaw_step = std::max(0.01, search_yaw_step_rad_ * 0.5);

    for (double dx = -fine_translation_window; dx <= fine_translation_window + 1.0e-6; dx += fine_translation_step) {
      for (double dy = -fine_translation_window; dy <= fine_translation_window + 1.0e-6; dy += fine_translation_step) {
        for (double yaw_delta = -fine_yaw_window; yaw_delta <= fine_yaw_window + 1.0e-6; yaw_delta += fine_yaw_step) {
          const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - match_start).count();
          if (elapsed_ms >= max_match_compute_time_ms_) {
            search_timed_out = true;
            break;
          }
          const double candidate_x = fine_center_x + dx;
          const double candidate_y = fine_center_y + dy;
          const double candidate_yaw = normalizeAngle(fine_center_yaw + yaw_delta);
          const double candidate_score = scoreCandidate(
            endpoints,
            candidate_x,
            candidate_y,
            candidate_yaw);
          if (candidate_score <= best_score) {
            continue;
          }
          best_score = candidate_score;
          best_transform = transformFromXYYaw(
            candidate_x,
            candidate_y,
            map_to_base_prior.getOrigin().z(),
            candidate_yaw);
        }
        if (search_timed_out) {
          break;
        }
      }
      if (search_timed_out) {
        break;
      }
    }
  }

  if (search_timed_out) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Skipping map pose scan matching because it exceeded the compute budget of %d ms.",
      max_match_compute_time_ms_);
    return std::nullopt;
  }

  if (!best_transform_ready) {
    return std::nullopt;
  }

  const double normalized_confidence = std::clamp(
    (best_score - minimum_match_score_) /
    std::max(1.0e-6, high_confidence_match_score_ - minimum_match_score_),
    0.0,
    1.0);
  const double georef_confidence = georeferenceConsistencyConfidence(snapshot, best_transform);
  return MapMatchEstimate{
    best_transform,
    best_score,
    normalized_confidence * georef_confidence};
}

void MapPoseNode::publishMapToOdomTransform()
{
  const StateSnapshot snapshot = snapshotState();
  if (map_frame_id_ == odom_frame_id_) {
    publishMapPoseStatus(snapshot);
    return;
  }
  if (!odometryInputReady(snapshot)) {
    publishMapPoseStatus(snapshot);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!map_to_odom_filter_ready_) {
      initializeMapToOdomFilterLocked();
    }
  }

  if (shouldHoldIdentityAtStartup(snapshot)) {
    const tf2::Transform identity = transformFromXYYaw(0.0, 0.0, 0.0, 0.0);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      initializeMapToOdomFilterLocked();
      last_map_to_odom_ = identity;
      last_map_to_odom_ready_ = true;
    }
    tf_broadcaster_->sendTransform(
      stampedFromTransform(
        identity,
        snapshot.latest_map_pose_stamp.nanoseconds() > 0 ? snapshot.latest_map_pose_stamp : now(),
        map_frame_id_,
        odom_frame_id_));
    publishMapPoseStatus(snapshotState());
    return;
  }

  tf2::Quaternion odom_base_quaternion;
  tf2::fromMsg(snapshot.latest_odometry_orientation, odom_base_quaternion);
  odom_base_quaternion.normalize();
  const tf2::Transform odom_to_base(
    odom_base_quaternion,
    tf2::Vector3(
      snapshot.latest_odometry_position.x,
      snapshot.latest_odometry_position.y,
      snapshot.latest_odometry_position.z));

  // Layer 3 starts with map and odom intentionally co-located so the correction state begins at zero.
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    predictMapToOdomFilterLocked();
  }
  StateSnapshot prediction_snapshot = snapshotState();
  tf2::Transform map_to_odom = filteredMapToOdomTransform(prediction_snapshot.map_to_odom_filter_state);
  tf2::Transform map_to_base_prior = map_to_odom * odom_to_base;

  bool attempted_scan_match = false;
  std::optional<MapMatchEstimate> map_match;
  const rclcpp::Time scan_match_now = now();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (
      last_scan_match_attempt_stamp_.nanoseconds() == 0 ||
      scan_match_period_seconds_ <= 0.0 ||
      (scan_match_now - last_scan_match_attempt_stamp_).seconds() >= scan_match_period_seconds_)
    {
      last_scan_match_attempt_stamp_ = scan_match_now;
      attempted_scan_match = true;
    }
  }

  if (attempted_scan_match) {
    map_match = estimateMapToBaseFromPrior(prediction_snapshot, map_to_base_prior);
  }

  if (
    map_match.has_value() &&
    map_match->confidence >= minimum_confidence_for_filter_update_ &&
    map_match->score >= minimum_match_score_)
  {
    tf2::Transform measured_map_to_odom = map_match->map_to_base * odom_to_base.inverse();

    if (prediction_snapshot.last_map_to_odom_ready) {
      const tf2::Vector3 previous_origin = prediction_snapshot.last_map_to_odom.getOrigin();
      const tf2::Vector3 candidate_origin = measured_map_to_odom.getOrigin();
      tf2::Vector3 delta = candidate_origin - previous_origin;
      const double delta_distance = delta.length();
      if (max_translation_jump_m_ > 0.0 && delta_distance > max_translation_jump_m_) {
        delta *= max_translation_jump_m_ / delta_distance;
      }

      const double previous_yaw = yawFromTransform(prediction_snapshot.last_map_to_odom);
      const double candidate_yaw = yawFromTransform(measured_map_to_odom);
      const double yaw_delta = normalizeAngle(candidate_yaw - previous_yaw);
      const double clamped_yaw_delta =
        max_yaw_jump_rad_ > 0.0 ?
        std::clamp(yaw_delta, -max_yaw_jump_rad_, max_yaw_jump_rad_) :
        yaw_delta;

      measured_map_to_odom = transformFromXYYaw(
        previous_origin.x() + delta.x(),
        previous_origin.y() + delta.y(),
        candidate_origin.z(),
        normalizeAngle(previous_yaw + clamped_yaw_delta));
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    updateMapToOdomFilterLocked(measured_map_to_odom, map_match->confidence);
  } else if (attempted_scan_match) {
    // Keep the last corrected map -> odom estimate when scan matching is stale
    // or low confidence instead of drifting the global alignment back to identity.
  }

  StateSnapshot final_snapshot = snapshotState();
  map_to_odom = blendTransforms(
    final_snapshot.last_map_to_odom_ready ?
    final_snapshot.last_map_to_odom :
    transformFromXYYaw(0.0, 0.0, 0.0, 0.0),
    filteredMapToOdomTransform(final_snapshot.map_to_odom_filter_state),
    transform_smoothing_alpha_);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_map_to_odom_ = map_to_odom;
    last_map_to_odom_ready_ = true;
  }

  tf_broadcaster_->sendTransform(
    stampedFromTransform(
      map_to_odom,
      final_snapshot.latest_map_pose_stamp.nanoseconds() > 0 ?
      final_snapshot.latest_map_pose_stamp :
      now(),
      map_frame_id_,
      odom_frame_id_));
  publishMapPoseStatus(final_snapshot);
}

}  // namespace amr_sweeper_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sweeper_mapping::MapPoseNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
