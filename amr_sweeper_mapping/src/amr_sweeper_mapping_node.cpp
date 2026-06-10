#include "amr_sweeper_mapping_node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <action_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nlohmann/json.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace amr_sweeper_mapping
{

namespace
{

constexpr char kEnabledParam[] = "enabled";
constexpr char kCostmapYamlPathParam[] = "costmap_yaml_path";
constexpr char kArtifactFrameIdParam[] = "artifact_frame_id";
constexpr char kDefaultCostmapYamlPath[] = "";
constexpr char kDefaultArtifactFrameId[] = "odom";
constexpr char kDefaultMissionRoutePath[] = "";
constexpr char kFollowWaypointsExecutionMode[] = "follow_waypoints";
constexpr char kManualMappingExecutionMode[] = "manual_mapping";

struct EcefPoint
{
  double x;
  double y;
  double z;
};

std::string trim(const std::string & value)
{
  const auto begin = value.find_first_not_of(" \t");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t");
  return value.substr(begin, end - begin + 1U);
}

std::string to_lower(std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  return value;
}

double yawForSegment(
  const geometry_msgs::msg::PoseStamped & current,
  const geometry_msgs::msg::PoseStamped & next)
{
  return std::atan2(
    next.pose.position.y - current.pose.position.y,
    next.pose.position.x - current.pose.position.x);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

std::vector<geometry_msgs::msg::PoseStamped> densifyRoute(
  const std::vector<geometry_msgs::msg::PoseStamped> & route,
  const double max_spacing_m)
{
  if (route.size() < 2U || max_spacing_m <= 0.0) {
    return route;
  }

  std::vector<geometry_msgs::msg::PoseStamped> densified;
  densified.reserve(route.size());
  densified.push_back(route.front());

  for (std::size_t index = 1U; index < route.size(); ++index) {
    const auto & previous = route.at(index - 1U);
    const auto & current = route.at(index);
    const double dx = current.pose.position.x - previous.pose.position.x;
    const double dy = current.pose.position.y - previous.pose.position.y;
    const double segment_length = std::hypot(dx, dy);
    const int subdivisions = std::max(
      1,
      static_cast<int>(std::ceil(segment_length / max_spacing_m)));

    for (int step = 1; step <= subdivisions; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(subdivisions);
      auto interpolated = current;
      interpolated.pose.position.x = previous.pose.position.x + dx * ratio;
      interpolated.pose.position.y = previous.pose.position.y + dy * ratio;
      densified.push_back(interpolated);
    }
  }

  return densified;
}

EcefPoint wgs84ToEcef(const double latitude_deg, const double longitude_deg, const double altitude_m)
{
  constexpr double kSemiMajorAxis = 6378137.0;
  constexpr double kFlattening = 1.0 / 298.257223563;
  constexpr double kFirstEccentricitySquared = kFlattening * (2.0 - kFlattening);

  const double latitude_rad = latitude_deg * M_PI / 180.0;
  const double longitude_rad = longitude_deg * M_PI / 180.0;
  const double sin_latitude = std::sin(latitude_rad);
  const double cos_latitude = std::cos(latitude_rad);
  const double sin_longitude = std::sin(longitude_rad);
  const double cos_longitude = std::cos(longitude_rad);
  const double radius_of_curvature =
    kSemiMajorAxis / std::sqrt(1.0 - kFirstEccentricitySquared * sin_latitude * sin_latitude);

  return {
    (radius_of_curvature + altitude_m) * cos_latitude * cos_longitude,
    (radius_of_curvature + altitude_m) * cos_latitude * sin_longitude,
    ((1.0 - kFirstEccentricitySquared) * radius_of_curvature + altitude_m) * sin_latitude};
}

tf2::Transform transformFromStamped(const geometry_msgs::msg::TransformStamped & message)
{
  tf2::Transform transform;
  tf2::fromMsg(message.transform, transform);
  return transform;
}

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

struct MissionRuntimeProfile
{
  std::string mission_type;
  std::string execution_mode{kFollowWaypointsExecutionMode};
};

MissionRuntimeProfile loadMissionRuntimeProfile(const std::string & mission_file_path)
{
  MissionRuntimeProfile profile;
  if (mission_file_path.empty()) {
    return profile;
  }

  const std::filesystem::path mission_path(mission_file_path);
  std::error_code filesystem_error;
  if (!std::filesystem::is_regular_file(mission_path, filesystem_error)) {
    return profile;
  }

  std::ifstream input_stream(mission_file_path);
  if (!input_stream.is_open()) {
    return profile;
  }

  nlohmann::json document;
  try {
    input_stream >> document;
  } catch (const std::exception &) {
    return profile;
  }

  if (document.contains("mission_type") && document.at("mission_type").is_string()) {
    profile.mission_type = document.at("mission_type").get<std::string>();
  }

  if (document.contains("execution_mode") && document.at("execution_mode").is_string()) {
    profile.execution_mode = to_lower(document.at("execution_mode").get<std::string>());
    return profile;
  }

  const std::string mission_type = to_lower(profile.mission_type);
  if (
    mission_type == "builtin_manual_mapping" ||
    mission_type == "record_map" ||
    mission_type == "manual_mapping")
  {
    profile.execution_mode = kManualMappingExecutionMode;
  }

  return profile;
}

nlohmann::json buildLocalPathGeoJson(
  const nlohmann::json & coordinates,
  const std::string & geographic_companion_file)
{
  nlohmann::json properties{
    {"name", "actual_path"},
    {"coordinate_frame", "odom"}};
  if (!geographic_companion_file.empty()) {
    properties["geographic_companion_file"] = geographic_companion_file;
  }

  return {
    {"type", "FeatureCollection"},
    {"features", nlohmann::json::array({
      {
        {"type", "Feature"},
        {"properties", properties},
        {"geometry", {{"type", "LineString"}, {"coordinates", coordinates}}}
      }
    })}
  };
}

}  // namespace

Vda5050CostmapLayer::Vda5050CostmapLayer() = default;

void Vda5050CostmapLayer::onInitialize()
{
  declareParameter(kEnabledParam, rclcpp::ParameterValue(true));
  declareParameter(kCostmapYamlPathParam, rclcpp::ParameterValue(std::string(kDefaultCostmapYamlPath)));
  declareParameter(kArtifactFrameIdParam, rclcpp::ParameterValue(std::string(kDefaultArtifactFrameId)));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node in Vda5050CostmapLayer");
  }

  node->get_parameter(getFullName(kEnabledParam), enabled_);
  node->get_parameter(getFullName(kArtifactFrameIdParam), artifact_frame_id_);
  global_frame_id_ = layered_costmap_->getGlobalFrameID();
  current_ = true;
  matchSize();
  loadArtifact();
}

void Vda5050CostmapLayer::updateBounds(
  double,
  double,
  double,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  if (!enabled_ || !artifact_loaded_) {
    return;
  }

  const std::array<std::pair<double, double>, 4> corners{{
    {artifact_.origin_x, artifact_.origin_y},
    {artifact_.origin_x + artifact_.resolution * artifact_.width_cells, artifact_.origin_y},
    {artifact_.origin_x, artifact_.origin_y + artifact_.resolution * artifact_.height_cells},
    {
      artifact_.origin_x + artifact_.resolution * artifact_.width_cells,
      artifact_.origin_y + artifact_.resolution * artifact_.height_cells,
    },
  }};

  for (const auto & [corner_x, corner_y] : corners) {
    const auto transformed = transformPoint(corner_x, corner_y, artifact_frame_id_, global_frame_id_);
    *min_x = std::min(*min_x, transformed.point.x);
    *min_y = std::min(*min_y, transformed.point.y);
    *max_x = std::max(*max_x, transformed.point.x);
    *max_y = std::max(*max_y, transformed.point.y);
  }
}

void Vda5050CostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_ || !artifact_loaded_) {
    return;
  }

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double world_x = 0.0;
      double world_y = 0.0;
      master_grid.mapToWorld(i, j, world_x, world_y);
      const unsigned char cost = sampleCostAtWorld(world_x, world_y);
      master_grid.setCost(i, j, std::max(master_grid.getCost(i, j), cost));
    }
  }
}

void Vda5050CostmapLayer::reset()
{
  loadArtifact();
}

bool Vda5050CostmapLayer::isClearable()
{
  return false;
}

void Vda5050CostmapLayer::matchSize()
{
  CostmapLayer::matchSize();
}

void Vda5050CostmapLayer::loadArtifact()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node while loading costmap artifact");
  }

  std::string yaml_path;
  node->get_parameter(getFullName(kCostmapYamlPathParam), yaml_path);
  if (yaml_path.empty()) {
    artifact_ = LoadedCostmapArtifact{};
    artifact_loaded_ = false;
    RCLCPP_INFO(
      node->get_logger(),
      "No mission costmap yaml was provided for %s. The geojson layer will stay inactive until a "
      "mission-specific costmap artifact is configured.",
      getName().c_str());
    return;
  }

  const std::string resolved_path = resolveArtifactPath(yaml_path);
  std::error_code filesystem_error;
  if (!std::filesystem::is_regular_file(resolved_path, filesystem_error)) {
    artifact_ = LoadedCostmapArtifact{};
    artifact_loaded_ = false;
    RCLCPP_WARN(
      node->get_logger(),
      "Mission costmap yaml for %s was configured as '%s' but no file was found there. "
      "The geojson layer will stay inactive.",
      getName().c_str(),
      resolved_path.c_str());
    return;
  }

  artifact_ = parseCostmapArtifact(resolved_path);
  artifact_loaded_ = true;
}

LoadedCostmapArtifact Vda5050CostmapLayer::parseCostmapArtifact(const std::string & yaml_path) const
{
  std::ifstream yaml_stream(yaml_path);
  if (!yaml_stream.is_open()) {
    throw std::runtime_error("Failed to open costmap YAML artifact: " + yaml_path);
  }

  std::string image_name;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::string line;
  while (std::getline(yaml_stream, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trim(line.substr(0, colon));
    const std::string value = trim(line.substr(colon + 1U));
    if (key == "image") {
      image_name = value;
    } else if (key == "resolution") {
      resolution = std::stod(value);
    } else if (key == "origin") {
      const auto open = value.find('[');
      const auto comma = value.find(',', open + 1U);
      const auto second_comma = value.find(',', comma + 1U);
      origin_x = std::stod(trim(value.substr(open + 1U, comma - open - 1U)));
      origin_y = std::stod(trim(value.substr(comma + 1U, second_comma - comma - 1U)));
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
  LoadedCostmapArtifact artifact;
  artifact.width_cells = width;
  artifact.height_cells = height;
  artifact.resolution = resolution;
  artifact.origin_x = origin_x;
  artifact.origin_y = origin_y;
  artifact.costs.resize(static_cast<std::size_t>(width) * height);

  if (magic == "P5") {
    image_stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    for (int row = static_cast<int>(height) - 1; row >= 0; --row) {
      for (unsigned int col = 0; col < width; ++col) {
        unsigned char pixel = 0U;
        image_stream.read(reinterpret_cast<char *>(&pixel), 1);
        const std::size_t index = static_cast<std::size_t>(row) * width + col;
        artifact.costs[index] = static_cast<unsigned char>(255U - pixel);
      }
    }
    return artifact;
  }

  for (int row = static_cast<int>(height) - 1; row >= 0; --row) {
    for (unsigned int col = 0; col < width; ++col) {
      int pixel_value = 0;
      image_stream >> pixel_value;
      const auto clamped_value = static_cast<unsigned char>(
        std::clamp(pixel_value, 0, max_value) * 255 / std::max(1, max_value));
      const std::size_t index = static_cast<std::size_t>(row) * width + col;
      artifact.costs[index] = static_cast<unsigned char>(255U - clamped_value);
    }
  }

  return artifact;
}

std::string Vda5050CostmapLayer::resolveArtifactPath(const std::string & configured_path) const
{
  namespace fs = std::filesystem;
  const fs::path configured(configured_path);
  if (configured.is_absolute()) {
    return configured.string();
  }

  const fs::path workspace_relative = fs::current_path() / configured;
  if (fs::exists(workspace_relative)) {
    return workspace_relative.string();
  }
  return configured.string();
}

geometry_msgs::msg::PointStamped Vda5050CostmapLayer::transformPoint(
  const double x,
  const double y,
  const std::string & source_frame,
  const std::string & target_frame) const
{
  geometry_msgs::msg::PointStamped point;
  point.header.frame_id = source_frame;
  point.point.x = x;
  point.point.y = y;
  point.point.z = 0.0;

  auto node = node_.lock();
  if (!node || source_frame.empty() || target_frame.empty() || source_frame == target_frame) {
    point.header.frame_id = target_frame.empty() ? source_frame : target_frame;
    return point;
  }

  try {
    return tf_->transform(point, target_frame, tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      5000,
      "Failed to transform mission artifact point from %s to %s: %s",
      source_frame.c_str(),
      target_frame.c_str(),
      exception.what());
    point.header.frame_id = target_frame;
    return point;
  }
}

unsigned char Vda5050CostmapLayer::sampleCostAtWorld(const double world_x, const double world_y) const
{
  const auto artifact_point = transformPoint(world_x, world_y, global_frame_id_, artifact_frame_id_);
  const double grid_x = (artifact_point.point.x - artifact_.origin_x) / artifact_.resolution;
  const double grid_y = (artifact_point.point.y - artifact_.origin_y) / artifact_.resolution;
  const int ix = static_cast<int>(std::floor(grid_x));
  const int iy = static_cast<int>(std::floor(grid_y));
  if (ix < 0 || iy < 0 ||
    ix >= static_cast<int>(artifact_.width_cells) ||
    iy >= static_cast<int>(artifact_.height_cells))
  {
    return 0U;
  }

  const std::size_t index =
    static_cast<std::size_t>(iy) * artifact_.width_cells + static_cast<std::size_t>(ix);
  return artifact_.costs.at(index);
}

MappingNode::MappingNode()
: Node("amr_sweeper_mapping_node")
{
  declare_parameter("mission_file", std::string(""));
  declare_parameter("mission_route_file", std::string(kDefaultMissionRoutePath));
  declare_parameter("mission_id", std::string(""));
  declare_parameter("mission_output_directory", std::string(""));
  declare_parameter("actual_path_output_file", std::string(""));
  declare_parameter("actual_path_navsat_output_file", std::string(""));
  declare_parameter("mission_costmap_yaml", std::string(""));
  declare_parameter("mission_window_start", std::string(""));
  declare_parameter("mission_window_end", std::string(""));
  declare_parameter("slam_backend", std::string("slam_toolbox"));
  declare_parameter("gaussian_mode", std::string("voxel_gaussians"));
  declare_parameter("frame_id", std::string("map"));
  declare_parameter("earth_frame", std::string("earth"));
  declare_parameter("map_frame", std::string("map"));
  declare_parameter("odom_frame", std::string("odom"));
  declare_parameter("fromll_service", std::string("/fromLL"));
  declare_parameter("datum_service", std::string("/get_datum"));
  declare_parameter("auto_start_mission", true);
  declare_parameter("repeat_mission", false);
  declare_parameter("publish_earth_to_map", true);
  declare_parameter("earth_to_map_planar_only", true);
  declare_parameter("earth_to_map_publish_period_seconds", 0.5);
  declare_parameter("max_segments_per_goal", 4);
  declare_parameter("max_waypoint_spacing_m", 0.5);
  declare_parameter("status_period_seconds", 2.0);
  declare_parameter("mission_tick_period_seconds", 1.0);
  declare_parameter("follow_waypoints_action", std::string("follow_waypoints"));
  declare_parameter("waypoint_follower_state_service", std::string("waypoint_follower/get_state"));
  declare_parameter("end_mission_service", std::string("end_mission"));

  mission_file_ = get_parameter("mission_file").as_string();
  const MissionRuntimeProfile mission_profile = loadMissionRuntimeProfile(resolveRuntimePath(mission_file_));
  mission_type_ = mission_profile.mission_type;
  execution_mode_ = mission_profile.execution_mode;
  mission_route_file_ = get_parameter("mission_route_file").as_string();
  mission_id_ = get_parameter("mission_id").as_string();
  mission_output_directory_ = get_parameter("mission_output_directory").as_string();
  actual_path_output_file_ = get_parameter("actual_path_output_file").as_string();
  actual_path_navsat_output_file_ = get_parameter("actual_path_navsat_output_file").as_string();
  mission_costmap_yaml_ = get_parameter("mission_costmap_yaml").as_string();
  mission_window_start_ = get_parameter("mission_window_start").as_string();
  mission_window_end_ = get_parameter("mission_window_end").as_string();
  slam_backend_ = get_parameter("slam_backend").as_string();
  gaussian_mode_ = get_parameter("gaussian_mode").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  earth_frame_id_ = get_parameter("earth_frame").as_string();
  map_frame_id_ = get_parameter("map_frame").as_string();
  odom_frame_id_ = get_parameter("odom_frame").as_string();
  fromll_service_name_ = get_parameter("fromll_service").as_string();
  datum_service_name_ = get_parameter("datum_service").as_string();
  end_mission_service_name_ = get_parameter("end_mission_service").as_string();
  waypoint_follower_state_service_name_ =
    get_parameter("waypoint_follower_state_service").as_string();
  auto_start_mission_ = get_parameter("auto_start_mission").as_bool();
  repeat_mission_ = get_parameter("repeat_mission").as_bool();
  publish_earth_to_map_ = get_parameter("publish_earth_to_map").as_bool();
  earth_to_map_planar_only_ = get_parameter("earth_to_map_planar_only").as_bool();
  max_segments_per_goal_ = static_cast<int>(get_parameter("max_segments_per_goal").as_int());
  max_waypoint_spacing_m_ = get_parameter("max_waypoint_spacing_m").as_double();
  manual_mapping_mode_ = execution_mode_ == kManualMappingExecutionMode;
  if (
    !manual_mapping_mode_ &&
    auto_start_mission_ &&
    mission_file_.empty() &&
    mission_route_file_.empty() &&
    mission_costmap_yaml_.empty())
  {
    manual_mapping_mode_ = true;
    mission_type_ = "builtin_manual_mapping";
    execution_mode_ = kManualMappingExecutionMode;
    if (mission_id_.empty()) {
      mission_id_ = "RecordMap";
    }
    RCLCPP_INFO(
      get_logger(),
      "No mission execution context was provided; falling back to manual mapping mode for mission_id=%s.",
      mission_id_.c_str());
  }
  mission_loaded_ = manual_mapping_mode_;
  mission_converted_ = manual_mapping_mode_;

  slam_status_subscription_ = create_subscription<std_msgs::msg::String>(
    "slam/status",
    10,
    std::bind(&MappingNode::handleSlamStatus, this, std::placeholders::_1));
  gaussian_status_subscription_ = create_subscription<std_msgs::msg::String>(
    "gaussian/status",
    10,
    std::bind(&MappingNode::handleGaussianStatus, this, std::placeholders::_1));
  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry/fused",
    50,
    std::bind(&MappingNode::handleOdometry, this, std::placeholders::_1));
  status_publisher_ = create_publisher<std_msgs::msg::String>("mapping/status", 10);
  route_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
    "mapping/route_marker",
    rclcpp::QoS(1).reliable().transient_local());

  mission_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  status_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  fromll_client_ = create_client<fusioncore_ros::srv::FromLL>(
    fromll_service_name_,
    rclcpp::ServicesQoS(),
    mission_callback_group_);
  datum_client_ = create_client<fusioncore_ros::srv::GetDatum>(
    datum_service_name_,
    rclcpp::ServicesQoS(),
    status_callback_group_);
  end_mission_client_ =
    create_client<amr_sweeper_mission_executor::srv::EndMission>(
    end_mission_service_name_,
    rclcpp::ServicesQoS(),
    mission_callback_group_);
  waypoint_follower_state_client_ =
    create_client<lifecycle_msgs::srv::GetState>(
    waypoint_follower_state_service_name_,
    rclcpp::ServicesQoS(),
    mission_callback_group_);
  follow_waypoints_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    this,
    get_parameter("follow_waypoints_action").as_string(),
    mission_callback_group_);

  status_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("status_period_seconds").as_double()),
    std::bind(&MappingNode::publishCoordinatorStatus, this),
    status_callback_group_);
  mission_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("mission_tick_period_seconds").as_double()),
    std::bind(&MappingNode::tickMissionExecution, this),
    mission_callback_group_);
  earth_to_map_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("earth_to_map_publish_period_seconds").as_double()),
    std::bind(&MappingNode::publishEarthToMapTransform, this),
    status_callback_group_);

  writeMissionSessionMetadata();

  RCLCPP_INFO(
    get_logger(),
    "Mapping coordinator ready; mission=%s route=%s costmap=%s output_dir=%s slam_backend=%s gaussian_mode=%s execution_mode=%s",
    mission_file_.c_str(),
    mission_route_file_.c_str(),
    mission_costmap_yaml_.c_str(),
    mission_output_directory_.c_str(),
    slam_backend_.c_str(),
    gaussian_mode_.c_str(),
    execution_mode_.c_str());
}

void MappingNode::handleSlamStatus(const std_msgs::msg::String::SharedPtr message)
{
  last_slam_status_ = message->data;
}

void MappingNode::handleGaussianStatus(const std_msgs::msg::String::SharedPtr message)
{
  last_gaussian_status_ = message->data;
}

void MappingNode::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  const auto & position = message->pose.pose.position;
  if (!traveled_path_points_.empty()) {
    const auto & last = traveled_path_points_.back();
    const double dx = position.x - last.x;
    const double dy = position.y - last.y;
    if ((dx * dx + dy * dy) < 0.01) {
      return;
    }
  }

  traveled_path_points_.push_back(position);
  writeActualPathArtifact();
}

void MappingNode::publishCoordinatorStatus()
{
  std_msgs::msg::String message;
  message.data =
    "amr_sweeper_mapping_node mission=" + mission_file_ +
    "; mission_type=" + mission_type_ +
    "; execution_mode=" + execution_mode_ +
    "; route=" + mission_route_file_ +
    "; mission_id=" + mission_id_ +
    "; mission_output_directory=" + mission_output_directory_ +
    "; mission_costmap_yaml=" + mission_costmap_yaml_ +
    "; mission_window_start=" + mission_window_start_ +
    "; mission_window_end=" + mission_window_end_ +
    "; slam_backend=" + slam_backend_ +
    "; gaussian_mode=" + gaussian_mode_ +
    "; earth_to_map=" + std::string(publish_earth_to_map_ ? "true" : "false") +
    "; earth_to_map_planar_only=" + std::string(earth_to_map_planar_only_ ? "true" : "false") +
    "; datum_ready=" + std::string(fusion_datum_ready_ ? "true" : "false") +
    "; mission_loaded=" + std::string(mission_loaded_ ? "true" : "false") +
    "; mission_converted=" + std::string(mission_converted_ ? "true" : "false") +
    "; mission_active=" + std::string(mission_active_ ? "true" : "false") +
    "; mission_completed=" + std::string(mission_completed_ ? "true" : "false") +
    "; active_chunk=" + std::to_string(active_chunk_index_) + "/" + std::to_string(mission_chunks_.size()) +
    "; slam_status=" + last_slam_status_ +
    "; gaussian_status=" + last_gaussian_status_;
  status_publisher_->publish(message);
}

void MappingNode::writeMissionSessionMetadata() const
{
  if (mission_output_directory_.empty()) {
    return;
  }

  std::filesystem::create_directories(mission_output_directory_);

  nlohmann::json document{
    {"mission_id", mission_id_},
    {"mission_file", mission_file_},
    {"mission_type", mission_type_},
    {"execution_mode", execution_mode_},
    {"mission_route_file", mission_route_file_},
    {"mission_costmap_yaml", mission_costmap_yaml_},
    {"mission_window_start", mission_window_start_},
    {"mission_window_end", mission_window_end_},
    {"slam_backend", slam_backend_},
    {"gaussian_mode", gaussian_mode_},
    {"frame_id", frame_id_},
    {"manual_drive_required", manual_mapping_mode_},
    {"actual_path_output_file", actual_path_output_file_},
    {"actual_path_navsat_output_file", actual_path_navsat_output_file_}};

  std::ofstream output_stream(std::filesystem::path(mission_output_directory_) / "mapping_session.json");
  if (!output_stream.is_open()) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to write mapping session metadata into %s",
      mission_output_directory_.c_str());
    return;
  }
  output_stream << document.dump(2) << "\n";
}

bool MappingNode::refreshFusionDatum()
{
  if (fusion_datum_ready_) {
    return true;
  }

  if (!datum_client_->wait_for_service(std::chrono::milliseconds(0))) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for FusionCore datum service %s",
      datum_service_name_.c_str());
    return false;
  }

  auto request = std::make_shared<fusioncore_ros::srv::GetDatum::Request>();
  auto future = datum_client_->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(250)) != std::future_status::ready) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for FusionCore datum response from %s",
      datum_service_name_.c_str());
    return false;
  }

  const auto response = future.get();
  if (!response) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "FusionCore datum request returned no response.");
    return false;
  }

  if (!response->available) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "FusionCore has not established a GNSS datum yet.");
    return false;
  }

  if (!response->local_frame_is_enu) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "FusionCore datum is available but the local frame is not ENU; earth->map publishing stays disabled.");
    return false;
  }

  fusion_datum_ = response->datum;
  fusion_datum_ready_ = true;
  RCLCPP_INFO(
    get_logger(),
    "Using FusionCore datum lat=%.8f lon=%.8f alt=%.3f for REP-105 earth->map publishing.",
    fusion_datum_.latitude,
    fusion_datum_.longitude,
    fusion_datum_.altitude);
  return true;
}

void MappingNode::publishEarthToMapTransform()
{
  if (!publish_earth_to_map_) {
    return;
  }

  if (!refreshFusionDatum()) {
    return;
  }

  geometry_msgs::msg::TransformStamped map_to_odom;
  try {
    map_to_odom = tf_buffer_->lookupTransform(odom_frame_id_, map_frame_id_, tf2::TimePointZero);
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for %s -> %s before publishing earth->map: %s",
      map_frame_id_.c_str(),
      odom_frame_id_.c_str(),
      exception.what());
    return;
  }

  const EcefPoint datum_ecef = wgs84ToEcef(
    fusion_datum_.latitude,
    fusion_datum_.longitude,
    fusion_datum_.altitude);
  tf2::Quaternion earth_to_odom_quaternion;
  if (earth_to_map_planar_only_) {
    earth_to_odom_quaternion.setRPY(0.0, 0.0, 0.0);
  } else {
    const double latitude_rad = fusion_datum_.latitude * M_PI / 180.0;
    const double longitude_rad = fusion_datum_.longitude * M_PI / 180.0;
    const double sin_latitude = std::sin(latitude_rad);
    const double cos_latitude = std::cos(latitude_rad);
    const double sin_longitude = std::sin(longitude_rad);
    const double cos_longitude = std::cos(longitude_rad);

    const tf2::Matrix3x3 earth_rotation(
      -sin_longitude, -sin_latitude * cos_longitude, cos_latitude * cos_longitude,
      cos_longitude, -sin_latitude * sin_longitude, cos_latitude * sin_longitude,
      0.0, cos_latitude, sin_latitude);
    earth_rotation.getRotation(earth_to_odom_quaternion);
  }
  earth_to_odom_quaternion.normalize();

  tf2::Transform earth_to_odom(earth_to_odom_quaternion, tf2::Vector3(
      datum_ecef.x,
      datum_ecef.y,
      datum_ecef.z));
  const tf2::Transform odom_to_map = transformFromStamped(map_to_odom);
  const tf2::Transform earth_to_map = earth_to_odom * odom_to_map;

  tf_broadcaster_->sendTransform(
    stampedFromTransform(earth_to_map, now(), earth_frame_id_, map_frame_id_));
}

void MappingNode::writeActualPathArtifact() const
{
  if (actual_path_output_file_.empty()) {
    return;
  }

  std::filesystem::create_directories(std::filesystem::path(actual_path_output_file_).parent_path());
  nlohmann::json coordinates = nlohmann::json::array();
  for (const auto & point : traveled_path_points_) {
    coordinates.push_back({point.x, point.y});
  }

  const std::string navsat_companion_file = actual_path_navsat_output_file_.empty() ?
    std::string{} :
    std::filesystem::path(actual_path_navsat_output_file_).filename().string();
  nlohmann::json document = buildLocalPathGeoJson(coordinates, navsat_companion_file);

  std::ofstream output_stream(actual_path_output_file_, std::ios::trunc);
  if (!output_stream.is_open()) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to write actual path artifact into %s",
      actual_path_output_file_.c_str());
    return;
  }
  output_stream << document.dump(2) << "\n";
}

void MappingNode::tickMissionExecution()
{
  if (!auto_start_mission_ || waiting_for_goal_result_) {
    return;
  }

  tryRequestMissionEnd();

  if (manual_mapping_mode_) {
    mission_loaded_ = true;
    mission_converted_ = true;
    return;
  }

  if (!mission_loaded_) {
    ensureMissionLoaded();
    return;
  }

  if (!mission_converted_) {
    convertMissionRoute();
    return;
  }

  if (mission_completed_ && repeat_mission_) {
    mission_completed_ = false;
    active_chunk_index_ = 0U;
  }

  if (!mission_active_ && !mission_completed_) {
    startNextMissionChunk();
  }
}

void MappingNode::ensureMissionLoaded()
{
  const std::string path = routeGeoJsonPath();
  if (path.empty()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for mission route artifact path to be configured.");
    return;
  }

  std::error_code filesystem_error;
  if (!std::filesystem::is_regular_file(path, filesystem_error)) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for mission route artifact file at %s",
      path.c_str());
    return;
  }

  if (!std::filesystem::exists(path)) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for mission route artifact at %s",
      path.c_str());
    return;
  }

  const auto coordinates = loadRouteCoordinates(path);
  (void)coordinates;
  mission_loaded_ = true;
  RCLCPP_INFO(get_logger(), "Mission route artifact detected at %s", path.c_str());
}

void MappingNode::convertMissionRoute()
{
  const std::vector<MissionCoordinate> coordinates = loadRouteCoordinates(routeGeoJsonPath());
  if (coordinates.empty()) {
    RCLCPP_WARN(get_logger(), "Mission route contained no coordinates.");
    return;
  }

  const bool use_local_frame = coordinates.front().use_local_frame;
  if (!use_local_frame && !fromll_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for FusionCore conversion service %s",
      fromll_service_name_.c_str());
    return;
  }

  mission_route_.clear();
  mission_route_.reserve(coordinates.size());

  for (const MissionCoordinate & coordinate : coordinates) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    if (coordinate.use_local_frame) {
      pose.pose.position.x = coordinate.x;
      pose.pose.position.y = coordinate.y;
      mission_route_.push_back(pose);
      continue;
    }

    auto request = std::make_shared<fusioncore_ros::srv::FromLL::Request>();
    request->ll_point.longitude = coordinate.x;
    request->ll_point.latitude = coordinate.y;
    request->ll_point.altitude = 0.0;
    auto future = fromll_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
      RCLCPP_WARN(get_logger(), "Timed out converting mission waypoint through %s", fromll_service_name_.c_str());
      return;
    }

    const auto response = future.get();
    pose.pose.position.x = response->map_point.x;
    pose.pose.position.y = response->map_point.y;
    mission_route_.push_back(pose);
  }

  mission_route_ = buildPoseSequence(coordinates);
  mission_route_ = densifyRoute(mission_route_, max_waypoint_spacing_m_);
  mission_chunks_ = chunkRoute(mission_route_);
  mission_converted_ = !mission_chunks_.empty();
  publishRouteMarker();
  RCLCPP_INFO(
    get_logger(),
    "Prepared %zu mission waypoint(s) in %s frame and %zu Nav2 chunk(s).",
    mission_route_.size(),
    use_local_frame ? "local" : "geographic",
    mission_chunks_.size());
}

void MappingNode::startNextMissionChunk()
{
  if (active_chunk_index_ >= mission_chunks_.size()) {
    mission_active_ = false;
    mission_completed_ = true;
    RCLCPP_INFO(get_logger(), "Mission waypoint execution completed.");
    if (!repeat_mission_) {
      markMissionTerminal("completed", "autonomous mission completed");
    }
    return;
  }

  if (!follow_waypoints_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for Nav2 follow_waypoints action server.");
    return;
  }

  if (!isWaypointFollowerActive()) {
    return;
  }

  nav2_msgs::action::FollowWaypoints::Goal goal;
  const auto stamp = now();
  for (auto pose : mission_chunks_.at(active_chunk_index_)) {
    pose.header.stamp = stamp;
    goal.poses.push_back(pose);
  }

  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const auto & goal_handle) {handleGoalResponse(goal_handle);};
  options.result_callback =
    [this](const auto & result) {handleGoalResult(result);};

  mission_active_ = true;
  waiting_for_goal_result_ = true;
  follow_waypoints_client_->async_send_goal(goal, options);
  RCLCPP_INFO(
    get_logger(),
    "Dispatching mission chunk %zu/%zu with %zu waypoint(s).",
    active_chunk_index_ + 1U,
    mission_chunks_.size(),
    goal.poses.size());
}

bool MappingNode::isWaypointFollowerActive()
{
  if (!waypoint_follower_state_client_->wait_for_service(std::chrono::milliseconds(0))) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for waypoint follower lifecycle service %s.",
      waypoint_follower_state_service_name_.c_str());
    return false;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future = waypoint_follower_state_client_->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(250)) != std::future_status::ready) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for waypoint follower lifecycle state from %s.",
      waypoint_follower_state_service_name_.c_str());
    return false;
  }

  const auto response = future.get();
  if (!response) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waypoint follower lifecycle state request returned no response.");
    return false;
  }

  if (response->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for waypoint follower to become active. Current state: %s (%u).",
      response->current_state.label.c_str(),
      static_cast<unsigned int>(response->current_state.id));
    return false;
  }

  return true;
}

void MappingNode::handleGoalResponse(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr goal_handle)
{
  if (!goal_handle) {
    waiting_for_goal_result_ = false;
    mission_active_ = false;
    if (!isWaypointFollowerActive()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Mission chunk was rejected before waypoint follower became active. Retrying.");
      return;
    }

    mission_completed_ = true;
    RCLCPP_ERROR(get_logger(), "Mission chunk was rejected by Nav2.");
    markMissionTerminal("aborted", "Nav2 rejected the mission chunk");
  }
}

void MappingNode::handleGoalResult(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult & result)
{
  waiting_for_goal_result_ = false;
  mission_active_ = false;

  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    mission_completed_ = true;
    RCLCPP_ERROR(get_logger(), "Mission chunk failed with result code %d", static_cast<int>(result.code));
    markMissionTerminal(
      "aborted",
      "Nav2 follow_waypoints ended with result code " + std::to_string(static_cast<int>(result.code)));
    return;
  }

  ++active_chunk_index_;
  if (active_chunk_index_ >= mission_chunks_.size()) {
    mission_completed_ = true;
    RCLCPP_INFO(get_logger(), "All mission chunks completed successfully.");
    if (!repeat_mission_) {
      markMissionTerminal("completed", "autonomous mission completed");
    }
  }
}

void MappingNode::tryRequestMissionEnd()
{
  if (!mission_end_pending_ || mission_end_requested_ || pending_end_reason_.empty()) {
    return;
  }

  if (!end_mission_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for mission executor end_mission service %s",
      end_mission_service_name_.c_str());
    return;
  }

  auto request = std::make_shared<amr_sweeper_mission_executor::srv::EndMission::Request>();
  request->mission_id = mission_id_;
  request->reason = pending_end_reason_;
  request->outcome = pending_end_outcome_;
  request->requester = "amr_sweeper_mapping_node";
  request->priority = 0U;
  request->force = false;
  request->request_idling = true;

  end_mission_client_->async_send_request(
    request,
    [this](rclcpp::Client<amr_sweeper_mission_executor::srv::EndMission>::SharedFuture future) {
      try {
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN(
            get_logger(),
            "Mission executor rejected mission end request: %s",
            response->message.c_str());
          mission_end_requested_ = false;
          return;
        }
        mission_end_pending_ = false;
        RCLCPP_INFO(get_logger(), "Mission executor finalized mission: %s", response->message.c_str());
      } catch (const std::exception & exception) {
        RCLCPP_WARN(get_logger(), "Mission end request failed: %s", exception.what());
        mission_end_requested_ = false;
      }
    });
  mission_end_requested_ = true;
}

void MappingNode::publishRouteMarker() const
{
  if (mission_route_.empty()) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = now();
  marker.ns = "mapping_route";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.08;
  marker.color.r = 0.2F;
  marker.color.g = 0.8F;
  marker.color.b = 0.3F;
  marker.color.a = 0.95F;
  for (const auto & pose : mission_route_) {
    marker.points.push_back(pose.pose.position);
  }
  route_marker_publisher_->publish(marker);
}

std::string MappingNode::routeGeoJsonPath() const
{
  return resolveRuntimePath(mission_route_file_);
}

std::string MappingNode::resolveRuntimePath(const std::string & configured_path) const
{
  namespace fs = std::filesystem;
  if (configured_path.empty()) {
    return "";
  }
  const fs::path configured(configured_path);
  if (configured.is_absolute()) {
    return configured.string();
  }

  const fs::path workspace_relative = fs::current_path() / configured;
  if (fs::exists(workspace_relative)) {
    return workspace_relative.string();
  }
  return configured.string();
}

std::vector<MissionCoordinate> MappingNode::loadRouteCoordinates(const std::string & path) const
{
  std::ifstream input_stream(path);
  if (!input_stream.is_open()) {
    throw std::runtime_error("Failed to open mission route GeoJSON: " + path);
  }

  nlohmann::json document;
  input_stream >> document;
  for (const auto & feature : document.at("features")) {
    const auto & geometry = feature.at("geometry");
    if (geometry.at("type") != "LineString") {
      continue;
    }

    bool use_local_frame = false;
    if (feature.contains("properties") && feature.at("properties").is_object()) {
      const auto & properties = feature.at("properties");
      if (properties.contains("coordinate_frame") && properties.at("coordinate_frame").is_string()) {
        const std::string coordinate_frame =
          to_lower(properties.at("coordinate_frame").get<std::string>());
        use_local_frame = coordinate_frame == "odom" || coordinate_frame == "local";
      }
    }

    std::vector<MissionCoordinate> coordinates;
    for (const auto & coordinate : geometry.at("coordinates")) {
      coordinates.push_back(MissionCoordinate{
        coordinate.at(0).get<double>(),
        coordinate.at(1).get<double>(),
        use_local_frame});
    }
    if (coordinates.size() >= 2U) {
      return coordinates;
    }
  }

  throw std::runtime_error("No usable LineString found in mission route GeoJSON");
}

std::vector<geometry_msgs::msg::PoseStamped> MappingNode::buildPoseSequence(
  const std::vector<MissionCoordinate> & coordinates) const
{
  std::vector<geometry_msgs::msg::PoseStamped> poses = mission_route_;
  for (std::size_t index = 0; index < poses.size(); ++index) {
    double yaw = 0.0;
    if (index + 1U < poses.size()) {
      yaw = yawForSegment(poses.at(index), poses.at(index + 1U));
    } else if (index > 0U) {
      yaw = yawForSegment(poses.at(index - 1U), poses.at(index));
    }
    poses.at(index).pose.orientation = quaternionFromYaw(yaw);
  }
  (void)coordinates;
  return poses;
}

std::vector<std::vector<geometry_msgs::msg::PoseStamped>> MappingNode::chunkRoute(
  const std::vector<geometry_msgs::msg::PoseStamped> & route) const
{
  std::vector<std::vector<geometry_msgs::msg::PoseStamped>> chunks;
  if (route.size() < 2U) {
    return chunks;
  }

  const std::size_t max_points_per_goal = static_cast<std::size_t>(std::max(1, max_segments_per_goal_) + 1);
  std::size_t start_index = 0U;
  while (start_index < route.size() - 1U) {
    const std::size_t end_index = std::min(start_index + max_points_per_goal, route.size());
    chunks.emplace_back(route.begin() + static_cast<long>(start_index), route.begin() + static_cast<long>(end_index));
    if (end_index >= route.size()) {
      break;
    }
    start_index = end_index - 1U;
  }

  return chunks;
}

void MappingNode::markMissionTerminal(const std::string & outcome, const std::string & reason)
{
  if (repeat_mission_) {
    return;
  }
  pending_end_outcome_ = outcome;
  pending_end_reason_ = reason;
  mission_end_pending_ = true;
}

}  // namespace amr_sweeper_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<amr_sweeper_mapping::MappingNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

PLUGINLIB_EXPORT_CLASS(amr_sweeper_mapping::Vda5050CostmapLayer, nav2_costmap_2d::Layer)
