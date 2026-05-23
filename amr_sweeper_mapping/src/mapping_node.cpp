#include "mapping_node.hpp"

#include <algorithm>
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
#include <nlohmann/json.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace amr_sweeper_mapping
{

namespace
{

constexpr char kEnabledParam[] = "enabled";
constexpr char kCostmapYamlPathParam[] = "costmap_yaml_path";
constexpr char kDefaultCostmapYamlPath[] = "src/missions/global_costmap.yaml";
constexpr char kDefaultMissionRoutePath[] = "src/missions/active_mission_path.geojson";

std::string trim(const std::string & value)
{
  const auto begin = value.find_first_not_of(" \t");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t");
  return value.substr(begin, end - begin + 1U);
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

}  // namespace

Vda5050CostmapLayer::Vda5050CostmapLayer() = default;

void Vda5050CostmapLayer::onInitialize()
{
  declareParameter(kEnabledParam, rclcpp::ParameterValue(true));
  declareParameter(kCostmapYamlPathParam, rclcpp::ParameterValue(std::string(kDefaultCostmapYamlPath)));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node in Vda5050CostmapLayer");
  }

  node->get_parameter(getFullName(kEnabledParam), enabled_);
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

  *min_x = std::min(*min_x, artifact_.origin_x);
  *min_y = std::min(*min_y, artifact_.origin_y);
  *max_x = std::max(*max_x, artifact_.origin_x + artifact_.resolution * artifact_.width_cells);
  *max_y = std::max(*max_y, artifact_.origin_y + artifact_.resolution * artifact_.height_cells);
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
  artifact_ = parseCostmapArtifact(resolveArtifactPath(yaml_path));
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
  if (magic != "P5") {
    throw std::runtime_error("Unsupported costmap image format: " + magic);
  }

  unsigned int width = 0U;
  unsigned int height = 0U;
  int max_value = 0;
  image_stream >> width >> height >> max_value;
  image_stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  LoadedCostmapArtifact artifact;
  artifact.width_cells = width;
  artifact.height_cells = height;
  artifact.resolution = resolution;
  artifact.origin_x = origin_x;
  artifact.origin_y = origin_y;
  artifact.costs.resize(static_cast<std::size_t>(width) * height);

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

unsigned char Vda5050CostmapLayer::sampleCostAtWorld(const double world_x, const double world_y) const
{
  const double grid_x = (world_x - artifact_.origin_x) / artifact_.resolution;
  const double grid_y = (world_y - artifact_.origin_y) / artifact_.resolution;
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
  declare_parameter("mission_output_directory", std::string("src/missions"));
  declare_parameter("mission_window_start", std::string(""));
  declare_parameter("mission_window_end", std::string(""));
  declare_parameter("slam_backend", std::string("slam_toolbox"));
  declare_parameter("gaussian_mode", std::string("voxel_gaussians"));
  declare_parameter("frame_id", std::string("map"));
  declare_parameter("fromll_service", std::string("/fromLL"));
  declare_parameter("auto_start_mission", true);
  declare_parameter("repeat_mission", false);
  declare_parameter("max_segments_per_goal", 4);
  declare_parameter("status_period_seconds", 2.0);
  declare_parameter("mission_tick_period_seconds", 1.0);
  declare_parameter("follow_waypoints_action", std::string("follow_waypoints"));

  mission_file_ = get_parameter("mission_file").as_string();
  mission_route_file_ = get_parameter("mission_route_file").as_string();
  mission_id_ = get_parameter("mission_id").as_string();
  mission_output_directory_ = get_parameter("mission_output_directory").as_string();
  mission_window_start_ = get_parameter("mission_window_start").as_string();
  mission_window_end_ = get_parameter("mission_window_end").as_string();
  slam_backend_ = get_parameter("slam_backend").as_string();
  gaussian_mode_ = get_parameter("gaussian_mode").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  fromll_service_name_ = get_parameter("fromll_service").as_string();
  auto_start_mission_ = get_parameter("auto_start_mission").as_bool();
  repeat_mission_ = get_parameter("repeat_mission").as_bool();
  max_segments_per_goal_ = static_cast<int>(get_parameter("max_segments_per_goal").as_int());

  slam_status_subscription_ = create_subscription<std_msgs::msg::String>(
    "slam/status",
    10,
    std::bind(&MappingNode::handleSlamStatus, this, std::placeholders::_1));
  gaussian_status_subscription_ = create_subscription<std_msgs::msg::String>(
    "gaussian/status",
    10,
    std::bind(&MappingNode::handleGaussianStatus, this, std::placeholders::_1));
  status_publisher_ = create_publisher<std_msgs::msg::String>("mapping/status", 10);
  route_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
    "mapping/route_marker",
    rclcpp::QoS(1).reliable().transient_local());

  fromll_client_ = create_client<fusioncore_ros::srv::FromLL>(fromll_service_name_);
  follow_waypoints_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    this,
    get_parameter("follow_waypoints_action").as_string());

  status_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("status_period_seconds").as_double()),
    std::bind(&MappingNode::publishCoordinatorStatus, this));
  mission_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("mission_tick_period_seconds").as_double()),
    std::bind(&MappingNode::tickMissionExecution, this));

  writeMissionSessionMetadata();

  RCLCPP_INFO(
    get_logger(),
    "Mapping coordinator ready; mission=%s route=%s output_dir=%s slam_backend=%s gaussian_mode=%s",
    mission_file_.c_str(),
    mission_route_file_.c_str(),
    mission_output_directory_.c_str(),
    slam_backend_.c_str(),
    gaussian_mode_.c_str());
}

void MappingNode::handleSlamStatus(const std_msgs::msg::String::SharedPtr message)
{
  last_slam_status_ = message->data;
}

void MappingNode::handleGaussianStatus(const std_msgs::msg::String::SharedPtr message)
{
  last_gaussian_status_ = message->data;
}

void MappingNode::publishCoordinatorStatus()
{
  std_msgs::msg::String message;
  message.data =
    "amr_sweeper_mapping_node mission=" + mission_file_ +
    "; route=" + mission_route_file_ +
    "; mission_id=" + mission_id_ +
    "; mission_output_directory=" + mission_output_directory_ +
    "; mission_window_start=" + mission_window_start_ +
    "; mission_window_end=" + mission_window_end_ +
    "; slam_backend=" + slam_backend_ +
    "; gaussian_mode=" + gaussian_mode_ +
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
    {"mission_route_file", mission_route_file_},
    {"mission_window_start", mission_window_start_},
    {"mission_window_end", mission_window_end_},
    {"slam_backend", slam_backend_},
    {"gaussian_mode", gaussian_mode_},
    {"frame_id", frame_id_}};

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

void MappingNode::tickMissionExecution()
{
  if (!auto_start_mission_ || waiting_for_goal_result_) {
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
  if (!fromll_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for FusionCore conversion service %s",
      fromll_service_name_.c_str());
    return;
  }

  const std::vector<MissionCoordinate> coordinates = loadRouteCoordinates(routeGeoJsonPath());
  mission_route_.clear();
  mission_route_.reserve(coordinates.size());

  for (const MissionCoordinate & coordinate : coordinates) {
    auto request = std::make_shared<fusioncore_ros::srv::FromLL::Request>();
    request->ll_point.longitude = coordinate.longitude;
    request->ll_point.latitude = coordinate.latitude;
    request->ll_point.altitude = 0.0;
    auto future = fromll_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Timed out converting mission waypoint through %s", fromll_service_name_.c_str());
      return;
    }

    const auto response = future.get();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = response->map_point.x;
    pose.pose.position.y = response->map_point.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    mission_route_.push_back(pose);
  }

  mission_route_ = buildPoseSequence(coordinates);
  mission_chunks_ = chunkRoute(mission_route_);
  mission_converted_ = !mission_chunks_.empty();
  publishRouteMarker();
  RCLCPP_INFO(
    get_logger(),
    "Converted %zu mission waypoint(s) through FusionCore and prepared %zu Nav2 chunk(s).",
    mission_route_.size(),
    mission_chunks_.size());
}

void MappingNode::startNextMissionChunk()
{
  if (active_chunk_index_ >= mission_chunks_.size()) {
    mission_active_ = false;
    mission_completed_ = true;
    RCLCPP_INFO(get_logger(), "Mission waypoint execution completed.");
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

void MappingNode::handleGoalResponse(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr goal_handle)
{
  if (!goal_handle) {
    waiting_for_goal_result_ = false;
    mission_active_ = false;
    RCLCPP_ERROR(get_logger(), "Mission chunk was rejected by Nav2.");
  }
}

void MappingNode::handleGoalResult(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult & result)
{
  waiting_for_goal_result_ = false;
  mission_active_ = false;

  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(get_logger(), "Mission chunk failed with result code %d", static_cast<int>(result.code));
    return;
  }

  ++active_chunk_index_;
  if (active_chunk_index_ >= mission_chunks_.size()) {
    mission_completed_ = true;
    RCLCPP_INFO(get_logger(), "All mission chunks completed successfully.");
  }
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
  namespace fs = std::filesystem;
  const fs::path configured(mission_route_file_);
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

    std::vector<MissionCoordinate> coordinates;
    for (const auto & coordinate : geometry.at("coordinates")) {
      coordinates.push_back(MissionCoordinate{
        coordinate.at(0).get<double>(),
        coordinate.at(1).get<double>()});
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

}  // namespace amr_sweeper_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_mapping::MappingNode>());
  rclcpp::shutdown();
  return 0;
}

PLUGINLIB_EXPORT_CLASS(amr_sweeper_mapping::Vda5050CostmapLayer, nav2_costmap_2d::Layer)
