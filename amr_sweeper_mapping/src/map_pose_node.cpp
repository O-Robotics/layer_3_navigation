#include "map_pose_node.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
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

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion_message)
{
  tf2::Quaternion quaternion;
  tf2::fromMsg(quaternion_message, quaternion);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
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

double bestOccupancyScoreAroundCell(
  const nav_msgs::msg::OccupancyGrid & map,
  const int grid_x,
  const int grid_y,
  const int radius_cells,
  const int occupied_threshold)
{
  double best_score = -1.0;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int sample_x = grid_x + dx;
      const int sample_y = grid_y + dy;
      if (
        sample_x < 0 || sample_y < 0 ||
        sample_x >= static_cast<int>(map.info.width) ||
        sample_y >= static_cast<int>(map.info.height))
      {
        continue;
      }

      const std::size_t index =
        static_cast<std::size_t>(sample_y) * map.info.width + static_cast<std::size_t>(sample_x);
      const int8_t value = map.data.at(index);
      if (value < 0) {
        best_score = std::max(best_score, -0.25);
        continue;
      }

      if (value >= occupied_threshold) {
        return 1.0;
      }

      best_score = std::max(best_score, static_cast<double>(value) / 100.0);
    }
  }

  return best_score;
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
  declare_parameter("fromll_service", std::string("/fromLL"));
  declare_parameter("costmap_yaml_path", std::string(""));
  declare_parameter("publish_period_seconds", 0.5);
  declare_parameter("publish_identity_when_pose_missing", true);
  declare_parameter("occupied_threshold", 65);
  declare_parameter("scan_subsample_step", 4);
  declare_parameter("min_valid_scan_points", 20);
  declare_parameter("endpoint_search_radius_cells", 1);
  declare_parameter("search_translation_window_m", 2.0);
  declare_parameter("search_translation_step_m", 0.25);
  declare_parameter("search_yaw_window_rad", 0.35);
  declare_parameter("search_yaw_step_rad", 0.0872664626);
  declare_parameter("translation_penalty_per_meter", 0.5);
  declare_parameter("yaw_penalty_per_rad", 0.25);
  declare_parameter("free_space_penalty", 0.35);
  declare_parameter("free_space_reward", 0.02);
  declare_parameter("occupied_reward", 1.0);
  declare_parameter("occupied_penalty", 0.3);
  declare_parameter("prior_blend_weight", 0.7);
  declare_parameter("scan_timeout_seconds", 1.0);
  declare_parameter("costmap_timeout_seconds", 2.0);
  declare_parameter("max_translation_jump_m", 0.75);
  declare_parameter("max_yaw_jump_rad", 0.35);
  declare_parameter("transform_smoothing_alpha", 0.35);

  map_frame_id_ = get_parameter("map_frame").as_string();
  odom_frame_id_ = get_parameter("odom_frame").as_string();
  base_frame_id_ = get_parameter("base_frame").as_string();
  odometry_topic_ = get_parameter("odometry_topic").as_string();
  navsat_topic_ = get_parameter("navsat_topic").as_string();
  heading_topic_ = get_parameter("heading_topic").as_string();
  scan_topic_ = get_parameter("scan_topic").as_string();
  global_costmap_topic_ = get_parameter("global_costmap_topic").as_string();
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
  search_translation_window_m_ = get_parameter("search_translation_window_m").as_double();
  search_translation_step_m_ = std::max(
    0.05,
    get_parameter("search_translation_step_m").as_double());
  search_yaw_window_rad_ = get_parameter("search_yaw_window_rad").as_double();
  search_yaw_step_rad_ = std::max(0.01, get_parameter("search_yaw_step_rad").as_double());
  translation_penalty_per_meter_ = get_parameter("translation_penalty_per_meter").as_double();
  yaw_penalty_per_rad_ = get_parameter("yaw_penalty_per_rad").as_double();
  free_space_penalty_ = get_parameter("free_space_penalty").as_double();
  free_space_reward_ = get_parameter("free_space_reward").as_double();
  occupied_reward_ = get_parameter("occupied_reward").as_double();
  occupied_penalty_ = get_parameter("occupied_penalty").as_double();
  prior_blend_weight_ = std::clamp(get_parameter("prior_blend_weight").as_double(), 0.0, 1.0);
  scan_timeout_seconds_ = std::max(0.05, get_parameter("scan_timeout_seconds").as_double());
  costmap_timeout_seconds_ = std::max(0.05, get_parameter("costmap_timeout_seconds").as_double());
  max_translation_jump_m_ = std::max(0.0, get_parameter("max_translation_jump_m").as_double());
  max_yaw_jump_rad_ = std::max(0.0, get_parameter("max_yaw_jump_rad").as_double());
  transform_smoothing_alpha_ = std::clamp(
    get_parameter("transform_smoothing_alpha").as_double(),
    0.0,
    1.0);
  loadCostmapGeoreference();

  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_,
    50,
    std::bind(&MapPoseNode::handleOdometry, this, std::placeholders::_1));
  navsat_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    navsat_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MapPoseNode::handleNavSat, this, std::placeholders::_1));
  heading_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
    heading_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&MapPoseNode::handleHeading, this, std::placeholders::_1));
  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&MapPoseNode::handleScan, this, std::placeholders::_1));
  global_costmap_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    global_costmap_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MapPoseNode::handleGlobalCostmap, this, std::placeholders::_1));
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  fromll_client_ = create_client<fusioncore_ros::srv::FromLL>(fromll_service_name_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("publish_period_seconds").as_double()),
    std::bind(&MapPoseNode::publishMapToOdomTransform, this));
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

void MapPoseNode::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  latest_odometry_position_ = message->pose.pose.position;
  latest_odometry_orientation_ = message->pose.pose.orientation;
  latest_odometry_stamp_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() :
    rclcpp::Time(message->header.stamp);
  latest_odometry_ready_ = true;
}

void MapPoseNode::handleNavSat(const sensor_msgs::msg::NavSatFix::SharedPtr message)
{
  latest_navsat_ = *message;
  latest_map_pose_stamp_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() :
    rclcpp::Time(message->header.stamp);
  latest_navsat_ready_ = true;
}

void MapPoseNode::handleHeading(const sensor_msgs::msg::Imu::SharedPtr message)
{
  latest_heading_ = *message;
  latest_heading_stamp_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() :
    rclcpp::Time(message->header.stamp);
  latest_map_pose_stamp_ = latest_heading_stamp_;
  latest_heading_ready_ = true;
}

void MapPoseNode::handleScan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  latest_scan_ = *message;
  latest_scan_stamp_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() :
    rclcpp::Time(message->header.stamp);
  latest_map_pose_stamp_ = latest_scan_stamp_;
  latest_scan_ready_ = true;
}

void MapPoseNode::handleGlobalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  latest_global_costmap_ = *message;
  latest_global_costmap_stamp_ =
    message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() :
    rclcpp::Time(message->header.stamp);
  latest_global_costmap_ready_ =
    message->info.width > 0U && message->info.height > 0U && message->info.resolution > 0.0F;
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

std::optional<tf2::Transform> MapPoseNode::estimateMapToBaseFromPrior(
  const geometry_msgs::msg::Point & map_position_prior,
  const double heading_prior_yaw) const
{
  if (!latest_scan_ready_ || !latest_global_costmap_ready_) {
    return std::nullopt;
  }

  const auto & scan = latest_scan_;
  const auto & map = latest_global_costmap_;
  if (scan.ranges.empty()) {
    return std::nullopt;
  }
  if ((now() - latest_scan_stamp_).seconds() > scan_timeout_seconds_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Skipping map pose scan matching because the latest scan is stale by %.3fs.",
      (now() - latest_scan_stamp_).seconds());
    return std::nullopt;
  }
  if ((now() - latest_global_costmap_stamp_).seconds() > costmap_timeout_seconds_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Skipping map pose scan matching because the latest global costmap is stale by %.3fs.",
      (now() - latest_global_costmap_stamp_).seconds());
    return std::nullopt;
  }

  struct ScanEndpoint
  {
    double origin_x;
    double origin_y;
    double x;
    double y;
  };

  geometry_msgs::msg::TransformStamped base_from_scan_message;
  try {
    const rclcpp::Time transform_stamp = scan.header.stamp.sec == 0 && scan.header.stamp.nanosec == 0 ?
      now() : rclcpp::Time(scan.header.stamp);
    base_from_scan_message = tf_buffer_->lookupTransform(
      base_frame_id_,
      scan.header.frame_id,
      transform_stamp,
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
  const tf2::Vector3 sensor_origin_in_base = base_from_scan * tf2::Vector3(0.0, 0.0, 0.0);
  std::vector<ScanEndpoint> endpoints;
  endpoints.reserve(scan.ranges.size() / static_cast<std::size_t>(scan_subsample_step_) + 1U);
  const double effective_max_range = scan.range_max > 0.0 ?
    scan.range_max : std::numeric_limits<double>::infinity();
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
      sensor_origin_in_base.x(),
      sensor_origin_in_base.y(),
      endpoint_in_base.x(),
      endpoint_in_base.y()});
  }

  if (static_cast<int>(endpoints.size()) < min_valid_scan_points_) {
    return std::nullopt;
  }

  double best_score = -std::numeric_limits<double>::infinity();
  tf2::Transform best_transform;
  bool best_transform_ready = false;
  tf2::Transform search_center = transformFromXYYaw(
    map_position_prior.x,
    map_position_prior.y,
    map_position_prior.z,
    heading_prior_yaw);
  if (last_map_to_odom_ready_ && latest_odometry_ready_) {
    tf2::Quaternion odom_base_quaternion;
    tf2::fromMsg(latest_odometry_orientation_, odom_base_quaternion);
    odom_base_quaternion.normalize();
    const tf2::Transform odom_to_base(
      odom_base_quaternion,
      tf2::Vector3(
        latest_odometry_position_.x,
        latest_odometry_position_.y,
        latest_odometry_position_.z));
    const tf2::Transform previous_map_to_base = last_map_to_odom_ * odom_to_base;
    search_center = blendTransforms(search_center, previous_map_to_base, 1.0 - prior_blend_weight_);
  }

  auto scoreCandidate = [&](const double candidate_x, const double candidate_y, const double candidate_yaw) {
      const double cos_yaw = std::cos(candidate_yaw);
      const double sin_yaw = std::sin(candidate_yaw);
      double endpoint_score_sum = 0.0;
      double freespace_score_sum = 0.0;
      int valid_points = 0;

      for (const auto & endpoint : endpoints) {
        const double world_origin_x =
          candidate_x + endpoint.origin_x * cos_yaw - endpoint.origin_y * sin_yaw;
        const double world_origin_y =
          candidate_y + endpoint.origin_x * sin_yaw + endpoint.origin_y * cos_yaw;
        const double world_x = candidate_x + endpoint.x * cos_yaw - endpoint.y * sin_yaw;
        const double world_y = candidate_y + endpoint.x * sin_yaw + endpoint.y * cos_yaw;
        int grid_x = 0;
        int grid_y = 0;
        if (!worldToGrid(map, world_x, world_y, grid_x, grid_y)) {
          continue;
        }

        const double endpoint_score = bestOccupancyScoreAroundCell(
          map,
          grid_x,
          grid_y,
          endpoint_search_radius_cells_,
          occupied_threshold_);

        const double beam_dx = world_x - world_origin_x;
        const double beam_dy = world_y - world_origin_y;
        const double beam_length = std::hypot(beam_dx, beam_dy);
        const int ray_steps = std::max(
          1,
          static_cast<int>(beam_length / std::max(0.05, static_cast<double>(map.info.resolution))));
        double ray_score = 0.0;
        for (int step = 1; step < ray_steps; ++step) {
          const double ratio = static_cast<double>(step) / static_cast<double>(ray_steps);
          const double sample_x = world_origin_x + beam_dx * ratio;
          const double sample_y = world_origin_y + beam_dy * ratio;
          int sample_grid_x = 0;
          int sample_grid_y = 0;
          if (!worldToGrid(map, sample_x, sample_y, sample_grid_x, sample_grid_y)) {
            continue;
          }
          const std::size_t sample_index =
            static_cast<std::size_t>(sample_grid_y) * map.info.width +
            static_cast<std::size_t>(sample_grid_x);
          const int8_t sample_value = map.data.at(sample_index);
          if (sample_value >= occupied_threshold_) {
            ray_score -= free_space_penalty_;
          } else if (sample_value >= 0) {
            ray_score += free_space_reward_;
          }
        }

        endpoint_score_sum += endpoint_score >= 0.75 ? occupied_reward_ :
          (endpoint_score < 0.0 ? -occupied_penalty_ : endpoint_score);
        freespace_score_sum += ray_score;
        ++valid_points;
      }

      if (valid_points < min_valid_scan_points_) {
        return -std::numeric_limits<double>::infinity();
      }

      const double center_dx = candidate_x - search_center.getOrigin().x();
      const double center_dy = candidate_y - search_center.getOrigin().y();
      const double yaw_delta = normalizeAngle(candidate_yaw - yawFromTransform(search_center));
      return
        ((endpoint_score_sum + freespace_score_sum) / static_cast<double>(valid_points)) -
        (translation_penalty_per_meter_ * std::hypot(center_dx, center_dy)) -
        (yaw_penalty_per_rad_ * std::abs(yaw_delta));
    };

  const double center_x = search_center.getOrigin().x();
  const double center_y = search_center.getOrigin().y();
  const double center_yaw = yawFromTransform(search_center);
  const auto runSearch = [&](const double translation_window, const double translation_step,
      const double yaw_window, const double yaw_step) {
      for (double dx = -translation_window; dx <= translation_window + 1.0e-6; dx += translation_step) {
        for (double dy = -translation_window; dy <= translation_window + 1.0e-6; dy += translation_step) {
          for (double yaw_delta = -yaw_window; yaw_delta <= yaw_window + 1.0e-6; yaw_delta += yaw_step) {
            const double candidate_x = center_x + dx;
            const double candidate_y = center_y + dy;
            const double candidate_yaw = normalizeAngle(center_yaw + yaw_delta);
            const double candidate_score = scoreCandidate(candidate_x, candidate_y, candidate_yaw);
            if (candidate_score <= best_score) {
              continue;
            }
            best_score = candidate_score;
            best_transform = transformFromXYYaw(
              candidate_x,
              candidate_y,
              map_position_prior.z,
              candidate_yaw);
            best_transform_ready = true;
          }
        }
      }
    };

  runSearch(
    search_translation_window_m_,
    search_translation_step_m_,
    search_yaw_window_rad_,
    search_yaw_step_rad_);

  if (best_transform_ready) {
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
          const double candidate_x = fine_center_x + dx;
          const double candidate_y = fine_center_y + dy;
          const double candidate_yaw = normalizeAngle(fine_center_yaw + yaw_delta);
          const double candidate_score = scoreCandidate(candidate_x, candidate_y, candidate_yaw);
          if (candidate_score <= best_score) {
            continue;
          }
          best_score = candidate_score;
          best_transform = transformFromXYYaw(
            candidate_x,
            candidate_y,
            map_position_prior.z,
            candidate_yaw);
        }
      }
    }
  }

  if (!best_transform_ready) {
    return std::nullopt;
  }

  return best_transform;
}

void MapPoseNode::publishMapToOdomTransform()
{
  if (map_frame_id_ == odom_frame_id_) {
    return;
  }

  if (!latest_odometry_ready_) {
    return;
  }

  tf2::Transform map_to_odom;
  bool have_backcalculated_transform = false;
  const std::optional<geometry_msgs::msg::Point> map_position = latestMapPositionFromNavSat();

  if (map_position.has_value() && latest_heading_ready_) {
    const double heading_prior_yaw = yawFromQuaternion(latest_heading_.orientation);
    std::optional<tf2::Transform> map_to_base =
      estimateMapToBaseFromPrior(*map_position, heading_prior_yaw);
    if (!map_to_base.has_value()) {
      tf2::Quaternion map_base_quaternion;
      map_base_quaternion.setRPY(0.0, 0.0, heading_prior_yaw);
      map_base_quaternion.normalize();
      map_to_base = tf2::Transform(
        map_base_quaternion,
        tf2::Vector3(
          map_position->x,
          map_position->y,
          map_position->z));
    }

    tf2::Quaternion odom_base_quaternion;
    tf2::fromMsg(latest_odometry_orientation_, odom_base_quaternion);
    odom_base_quaternion.normalize();
    const tf2::Transform odom_to_base(
      odom_base_quaternion,
      tf2::Vector3(
        latest_odometry_position_.x,
        latest_odometry_position_.y,
        latest_odometry_position_.z));

    map_to_odom = *map_to_base * odom_to_base.inverse();
    have_backcalculated_transform = true;
  }

  if (!have_backcalculated_transform) {
    if (!publish_identity_when_pose_missing_) {
      return;
    }
    tf2::Quaternion identity_quaternion;
    identity_quaternion.setRPY(0.0, 0.0, 0.0);
    identity_quaternion.normalize();
    map_to_odom = tf2::Transform(identity_quaternion, tf2::Vector3(0.0, 0.0, 0.0));

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Falling back to identity %s -> %s because georeferenced map pose inputs are incomplete. navsat=%s heading=%s",
      map_frame_id_.c_str(),
      odom_frame_id_.c_str(),
      latest_navsat_ready_ ? "true" : "false",
      latest_heading_ready_ ? "true" : "false");
  }

  if (last_map_to_odom_ready_) {
    const tf2::Vector3 previous_origin = last_map_to_odom_.getOrigin();
    const tf2::Vector3 candidate_origin = map_to_odom.getOrigin();
    tf2::Vector3 delta = candidate_origin - previous_origin;
    const double delta_distance = delta.length();
    if (max_translation_jump_m_ > 0.0 && delta_distance > max_translation_jump_m_) {
      delta *= max_translation_jump_m_ / delta_distance;
    }

    const double previous_yaw = yawFromTransform(last_map_to_odom_);
    const double candidate_yaw = yawFromTransform(map_to_odom);
    const double yaw_delta = normalizeAngle(candidate_yaw - previous_yaw);
    const double clamped_yaw_delta =
      max_yaw_jump_rad_ > 0.0 ?
      std::clamp(yaw_delta, -max_yaw_jump_rad_, max_yaw_jump_rad_) :
      yaw_delta;

    const tf2::Transform limited_candidate = transformFromXYYaw(
      previous_origin.x() + delta.x(),
      previous_origin.y() + delta.y(),
      candidate_origin.z(),
      normalizeAngle(previous_yaw + clamped_yaw_delta));
    map_to_odom = blendTransforms(
      last_map_to_odom_,
      limited_candidate,
      transform_smoothing_alpha_);
  }

  last_map_to_odom_ = map_to_odom;
  last_map_to_odom_ready_ = true;

  tf_broadcaster_->sendTransform(
    stampedFromTransform(
      map_to_odom,
      latest_map_pose_stamp_.nanoseconds() > 0 ? latest_map_pose_stamp_ : now(),
      map_frame_id_,
      odom_frame_id_));
}

}  // namespace amr_sweeper_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sweeper_mapping::MapPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
