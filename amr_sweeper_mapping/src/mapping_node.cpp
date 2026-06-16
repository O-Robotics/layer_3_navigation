#include "mapping_node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <action_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nlohmann/json.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
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
constexpr char kDefaultNavSatTopic[] = "gnss/navsat";
constexpr char kDefaultScanTopic[] = "depth_camera/scan";

struct LocalPoint
{
  double x;
  double y;
};

struct MapPoint
{
  double x{0.0};
  double y{0.0};
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

std::string formatTimestamp(const rclcpp::Time & stamp)
{
  std::ostringstream stream;
  const auto seconds = static_cast<long long>(stamp.nanoseconds() / 1000000000LL);
  const auto nanoseconds = static_cast<long long>(stamp.nanoseconds() % 1000000000LL);
  stream << seconds << "." << std::setw(9) << std::setfill('0') << std::llabs(nanoseconds);
  return stream.str();
}

bool solveLinear3x3(double matrix[3][4], double solution[3])
{
  for (int pivot = 0; pivot < 3; ++pivot) {
    int best_row = pivot;
    for (int row = pivot + 1; row < 3; ++row) {
      if (std::abs(matrix[row][pivot]) > std::abs(matrix[best_row][pivot])) {
        best_row = row;
      }
    }
    if (std::abs(matrix[best_row][pivot]) <= 1.0e-12) {
      return false;
    }
    if (best_row != pivot) {
      for (int column = pivot; column < 4; ++column) {
        std::swap(matrix[pivot][column], matrix[best_row][column]);
      }
    }
    const double pivot_value = matrix[pivot][pivot];
    for (int column = pivot; column < 4; ++column) {
      matrix[pivot][column] /= pivot_value;
    }
    for (int row = 0; row < 3; ++row) {
      if (row == pivot) {
        continue;
      }
      const double factor = matrix[row][pivot];
      for (int column = pivot; column < 4; ++column) {
        matrix[row][column] -= factor * matrix[pivot][column];
      }
    }
  }

  for (int row = 0; row < 3; ++row) {
    solution[row] = matrix[row][3];
  }
  return true;
}

bool fitAffineComponent(
  const std::vector<MapPoint> & local_points,
  const std::vector<double> & targets,
  double coefficients[3])
{
  if (local_points.size() != targets.size() || local_points.size() < 3U) {
    return false;
  }

  double ata[3][3] = {};
  double atb[3] = {};
  for (std::size_t index = 0U; index < local_points.size(); ++index) {
    const double row[3] = {local_points.at(index).x, local_points.at(index).y, 1.0};
    for (int i = 0; i < 3; ++i) {
      atb[i] += row[i] * targets.at(index);
      for (int j = 0; j < 3; ++j) {
        ata[i][j] += row[i] * row[j];
      }
    }
  }

  double augmented[3][4] = {
    {ata[0][0], ata[0][1], ata[0][2], atb[0]},
    {ata[1][0], ata[1][1], ata[1][2], atb[1]},
    {ata[2][0], ata[2][1], ata[2][2], atb[2]},
  };
  return solveLinear3x3(augmented, coefficients);
}

std::optional<nlohmann::json> buildGeoReferenceMetadata(
  const std::vector<MapPoint> & local_trace,
  const std::vector<GeoPoint> & geo_trace,
  const std::string & companion_file)
{
  if (local_trace.size() < 3U || geo_trace.size() < 3U || local_trace.size() != geo_trace.size()) {
    return std::nullopt;
  }

  const std::size_t sample_count = std::min<std::size_t>(12U, local_trace.size());
  std::vector<MapPoint> sampled_local_points;
  std::vector<double> sampled_longitudes;
  std::vector<double> sampled_latitudes;
  sampled_local_points.reserve(sample_count);
  sampled_longitudes.reserve(sample_count);
  sampled_latitudes.reserve(sample_count);

  for (std::size_t index = 0U; index < sample_count; ++index) {
    const std::size_t sample_index =
      sample_count == 1U ? 0U : ((index * (local_trace.size() - 1U)) / (sample_count - 1U));
    sampled_local_points.push_back(local_trace.at(sample_index));
    sampled_longitudes.push_back(geo_trace.at(sample_index).longitude);
    sampled_latitudes.push_back(geo_trace.at(sample_index).latitude);
  }

  GeoTransform transform;
  if (!fitAffineComponent(sampled_local_points, sampled_longitudes, transform.longitude_coefficients) ||
    !fitAffineComponent(sampled_local_points, sampled_latitudes, transform.latitude_coefficients))
  {
    return std::nullopt;
  }

  transform.valid = true;
  nlohmann::json georeference{
    {"type", "affine_xy_to_wgs84"},
    {"sample_count", sample_count},
    {"longitude_coefficients", {
       transform.longitude_coefficients[0],
       transform.longitude_coefficients[1],
       transform.longitude_coefficients[2]}},
    {"latitude_coefficients", {
       transform.latitude_coefficients[0],
       transform.latitude_coefficients[1],
       transform.latitude_coefficients[2]}}};
  if (!companion_file.empty()) {
    georeference["companion_file"] = companion_file;
  }
  return georeference;
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

LocalPoint transformBaseFootprintPointToOdom(
  const double x,
  const double y,
  const geometry_msgs::msg::Point & origin,
  const geometry_msgs::msg::Quaternion & orientation)
{
  tf2::Quaternion quaternion;
  tf2::fromMsg(orientation, quaternion);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  return {
    origin.x + (x * cos_yaw) - (y * sin_yaw),
    origin.y + (x * sin_yaw) + (y * cos_yaw)};
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

std::vector<geometry_msgs::msg::PoseStamped> orientRoute(
  std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  for (std::size_t index = 0; index < poses.size(); ++index) {
    double yaw = 0.0;
    if (index + 1U < poses.size()) {
      yaw = yawForSegment(poses.at(index), poses.at(index + 1U));
    } else if (index > 0U) {
      yaw = yawForSegment(poses.at(index - 1U), poses.at(index));
    }
    poses.at(index).pose.orientation = quaternionFromYaw(yaw);
  }
  return poses;
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

nlohmann::json buildSynchronizedLocalPathGeoJson(
  const std::vector<SynchronizedPathSample> & samples,
  const std::string & geographic_companion_file)
{
  nlohmann::json coordinates = nlohmann::json::array();
  nlohmann::json sample_timestamps = nlohmann::json::array();
  nlohmann::json yaw_values = nlohmann::json::array();
  std::vector<MapPoint> local_trace;
  std::vector<GeoPoint> geo_trace;
  local_trace.reserve(samples.size());
  geo_trace.reserve(samples.size());

  for (const auto & sample : samples) {
    coordinates.push_back({sample.odom_position.x, sample.odom_position.y});
    sample_timestamps.push_back(formatTimestamp(sample.odom_stamp));
    yaw_values.push_back(sample.yaw);
    local_trace.push_back({sample.odom_position.x, sample.odom_position.y});
    geo_trace.push_back({sample.raw_navsat.latitude, sample.raw_navsat.longitude});
  }

  nlohmann::json properties{
    {"name", "actual_path"},
    {"coordinate_frame", "odom"},
    {"sample_count", samples.size()},
    {"sample_timestamps", sample_timestamps},
    {"point_yaws_rad", yaw_values},
    {"position_source", "localization/odometry_fused"},
    {"orientation_source", "localization/odometry_fused"}};
  if (!geographic_companion_file.empty()) {
    properties["geographic_companion_file"] = geographic_companion_file;
  }

  const auto georeference = buildGeoReferenceMetadata(local_trace, geo_trace, geographic_companion_file);
  if (georeference.has_value()) {
    properties["georeference"] = *georeference;
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

nlohmann::json buildSynchronizedNavSatGeoJson(
  const std::vector<SynchronizedPathSample> & samples,
  const std::string & local_companion_file)
{
  nlohmann::json coordinates = nlohmann::json::array();
  nlohmann::json paired_sample_timestamps = nlohmann::json::array();
  nlohmann::json raw_navsat_timestamps = nlohmann::json::array();

  for (const auto & sample : samples) {
    coordinates.push_back({sample.raw_navsat.longitude, sample.raw_navsat.latitude});
    paired_sample_timestamps.push_back(formatTimestamp(sample.odom_stamp));
    raw_navsat_timestamps.push_back(formatTimestamp(sample.raw_navsat.stamp));
  }

  nlohmann::json properties{
    {"name", "actual_path_navsat"},
    {"coordinate_frame", "wgs84"},
    {"sample_count", samples.size()},
    {"sample_timestamps", paired_sample_timestamps},
    {"raw_navsat_timestamps", raw_navsat_timestamps},
    {"position_source", "gnss/navsat"}};
  if (!local_companion_file.empty()) {
    properties["local_companion_file"] = local_companion_file;
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

void writeJsonDocumentAtomic(
  const std::filesystem::path & path,
  const nlohmann::json & document)
{
  std::filesystem::create_directories(path.parent_path());
  const std::filesystem::path temp_path = path.string() + ".tmp";

  {
    std::ofstream output_stream(temp_path, std::ios::trunc);
    if (!output_stream.is_open()) {
      throw std::runtime_error("Failed to open JSON artifact for write: " + temp_path.string());
    }
    output_stream << document.dump(2) << "\n";
    output_stream.flush();
    if (!output_stream.good()) {
      throw std::runtime_error("Failed to flush JSON artifact: " + temp_path.string());
    }
  }

  std::filesystem::rename(temp_path, path);
}

std::string resolveArtifactPathString(const std::string & configured_path)
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

LoadedCostmapArtifact loadCostmapArtifactFromYaml(const std::string & yaml_path)
{
  std::ifstream yaml_stream(yaml_path);
  if (!yaml_stream.is_open()) {
    throw std::runtime_error("Failed to open costmap YAML artifact: " + yaml_path);
  }

  std::string image_name;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
  bool georeference_valid = false;
  std::string georeference_type;
  std::string georeference_source_crs = "EPSG:4326";
  std::string georeference_companion_file;
  std::size_t georeference_sample_count = 0U;
  std::array<double, 3> longitude_coefficients{0.0, 0.0, 0.0};
  std::array<double, 3> latitude_coefficients{0.0, 0.0, 0.0};
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
    } else if (key == "resolution") {
      resolution = std::stod(value);
    } else if (key == "origin") {
      const auto open = value.find('[');
      const auto comma = value.find(',', open + 1U);
      const auto second_comma = value.find(',', comma + 1U);
      origin_x = std::stod(trim(value.substr(open + 1U, comma - open - 1U)));
      origin_y = std::stod(trim(value.substr(comma + 1U, second_comma - comma - 1U)));
    } else if (key == "occupied_thresh") {
      occupied_thresh = std::stod(value);
    } else if (key == "free_thresh") {
      free_thresh = std::stod(value);
    } else if (key == "georeference_type") {
      georeference_type = stripQuotes(value);
      georeference_valid = !georeference_type.empty();
    } else if (key == "georeference_source_crs") {
      georeference_source_crs = stripQuotes(value);
    } else if (key == "georeference_companion_file") {
      georeference_companion_file = stripQuotes(value);
    } else if (key == "georeference_sample_count") {
      georeference_sample_count = static_cast<std::size_t>(std::stoul(value));
    } else if (key == "georeference_longitude_coefficients") {
      const auto open = value.find('[');
      const auto first_comma = value.find(',', open + 1U);
      const auto second_comma = value.find(',', first_comma + 1U);
      const auto close = value.find(']', second_comma + 1U);
      longitude_coefficients = {
        std::stod(trim(value.substr(open + 1U, first_comma - open - 1U))),
        std::stod(trim(value.substr(first_comma + 1U, second_comma - first_comma - 1U))),
        std::stod(trim(value.substr(second_comma + 1U, close - second_comma - 1U)))};
      georeference_valid = true;
    } else if (key == "georeference_latitude_coefficients") {
      const auto open = value.find('[');
      const auto first_comma = value.find(',', open + 1U);
      const auto second_comma = value.find(',', first_comma + 1U);
      const auto close = value.find(']', second_comma + 1U);
      latitude_coefficients = {
        std::stod(trim(value.substr(open + 1U, first_comma - open - 1U))),
        std::stod(trim(value.substr(first_comma + 1U, second_comma - first_comma - 1U))),
        std::stod(trim(value.substr(second_comma + 1U, close - second_comma - 1U)))};
      georeference_valid = true;
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

  LoadedCostmapArtifact artifact;
  artifact.width_cells = width;
  artifact.height_cells = height;
  artifact.resolution = resolution;
  artifact.origin_x = origin_x;
  artifact.origin_y = origin_y;
  artifact.occupied_thresh = occupied_thresh;
  artifact.free_thresh = free_thresh;
  artifact.georeference_valid = georeference_valid;
  artifact.georeference_type = georeference_type;
  artifact.georeference_source_crs = georeference_source_crs;
  artifact.georeference_companion_file = georeference_companion_file;
  artifact.georeference_sample_count = georeference_sample_count;
  artifact.longitude_coefficients = longitude_coefficients;
  artifact.latitude_coefficients = latitude_coefficients;
  artifact.costs.resize(static_cast<std::size_t>(width) * height);

  auto to_occupancy_byte = [max_value, negate](const int pixel_value) -> unsigned char {
    const int bounded = std::clamp(pixel_value, 0, std::max(1, max_value));
    const double normalized = static_cast<double>(bounded) / static_cast<double>(std::max(1, max_value));
    const double occupied = negate ? normalized : (1.0 - normalized);
    return static_cast<unsigned char>(std::lround(std::clamp(occupied, 0.0, 1.0) * 255.0));
  };

  if (magic == "P5") {
    image_stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    for (int row = static_cast<int>(height) - 1; row >= 0; --row) {
      for (unsigned int col = 0; col < width; ++col) {
        unsigned char pixel = 0U;
        image_stream.read(reinterpret_cast<char *>(&pixel), 1);
        const std::size_t index = static_cast<std::size_t>(row) * width + col;
        artifact.costs[index] = to_occupancy_byte(static_cast<int>(pixel));
      }
    }
    return artifact;
  }

  for (int row = static_cast<int>(height) - 1; row >= 0; --row) {
    for (unsigned int col = 0; col < width; ++col) {
      int pixel_value = 0;
      image_stream >> pixel_value;
      const std::size_t index = static_cast<std::size_t>(row) * width + col;
      artifact.costs[index] = to_occupancy_byte(pixel_value);
    }
  }

  return artifact;
}

void saveOccupancyGridArtifact(
  const nav_msgs::msg::OccupancyGrid & map,
  const std::filesystem::path & yaml_path,
  const std::optional<nlohmann::json> & georeference = std::nullopt)
{
  if (map.info.width == 0U || map.info.height == 0U || map.data.empty()) {
    throw std::runtime_error("Cannot save an empty occupancy grid artifact");
  }

  namespace fs = std::filesystem;
  const fs::path image_path = yaml_path.parent_path() / (yaml_path.stem().string() + ".pgm");
  fs::create_directories(yaml_path.parent_path());

  std::ofstream image_stream(image_path, std::ios::binary);
  if (!image_stream.is_open()) {
    throw std::runtime_error("Failed to write costmap image artifact: " + image_path.string());
  }
  image_stream << "P5\n" << map.info.width << " " << map.info.height << "\n255\n";
  for (int row = static_cast<int>(map.info.height) - 1; row >= 0; --row) {
    for (uint32_t col = 0U; col < map.info.width; ++col) {
      const std::size_t index = static_cast<std::size_t>(row) * map.info.width + col;
      const int8_t cell = map.data.at(index);
      const double occupied_ratio = cell < 0 ? 0.5 : std::clamp(static_cast<double>(cell) / 100.0, 0.0, 1.0);
      const unsigned char pixel = static_cast<unsigned char>(
        std::lround((1.0 - occupied_ratio) * 255.0));
      image_stream.write(reinterpret_cast<const char *>(&pixel), 1);
    }
  }

  std::ofstream yaml_stream(yaml_path, std::ios::trunc);
  if (!yaml_stream.is_open()) {
    throw std::runtime_error("Failed to write costmap yaml artifact: " + yaml_path.string());
  }
  yaml_stream
    << "image: " << image_path.filename().string() << "\n"
    << "mode: trinary\n"
    << "resolution: " << map.info.resolution << "\n"
    << "origin: [" << map.info.origin.position.x << ", " << map.info.origin.position.y << ", 0.0]\n"
    << "negate: 0\n"
    << "occupied_thresh: 0.65\n"
    << "free_thresh: 0.196\n";
  if (georeference.has_value()) {
    const auto & metadata = *georeference;
    yaml_stream << "georeference_type: " <<
      metadata.value("type", std::string("affine_xy_to_wgs84")) << "\n";
    yaml_stream << "georeference_source_crs: " <<
      metadata.value("source_crs", std::string("EPSG:4326")) << "\n";
    if (metadata.contains("companion_file")) {
      yaml_stream << "georeference_companion_file: " <<
        metadata.at("companion_file").get<std::string>() << "\n";
    }
    if (metadata.contains("sample_count")) {
      yaml_stream << "georeference_sample_count: " <<
        metadata.at("sample_count").get<std::size_t>() << "\n";
    }
    if (metadata.contains("longitude_coefficients")) {
      const auto & coefficients = metadata.at("longitude_coefficients");
      yaml_stream << "georeference_longitude_coefficients: [" <<
        coefficients.at(0).get<double>() << ", " <<
        coefficients.at(1).get<double>() << ", " <<
        coefficients.at(2).get<double>() << "]\n";
    }
    if (metadata.contains("latitude_coefficients")) {
      const auto & coefficients = metadata.at("latitude_coefficients");
      yaml_stream << "georeference_latitude_coefficients: [" <<
        coefficients.at(0).get<double>() << ", " <<
        coefficients.at(1).get<double>() << ", " <<
        coefficients.at(2).get<double>() << "]\n";
    }
  }
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

  const std::string resolved_path = resolveArtifactPathString(yaml_path);
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

  artifact_ = loadCostmapArtifactFromYaml(resolved_path);
  artifact_loaded_ = true;
}

LoadedCostmapArtifact Vda5050CostmapLayer::parseCostmapArtifact(const std::string & yaml_path) const
{
  return loadCostmapArtifactFromYaml(yaml_path);
}

std::string Vda5050CostmapLayer::resolveArtifactPath(const std::string & configured_path) const
{
  return resolveArtifactPathString(configured_path);
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
: Node("mapping_node")
{
  declare_parameter("mission_file", std::string(""));
  declare_parameter("mission_route_file", std::string(kDefaultMissionRoutePath));
  declare_parameter("mission_id", std::string(""));
  declare_parameter("mission_output_directory", std::string(""));
  declare_parameter("actual_path_output_file", std::string(""));
  declare_parameter("actual_path_navsat_output_file", std::string(""));
  declare_parameter("saved_costmap_yaml", std::string(""));
  declare_parameter("mission_costmap_yaml", std::string(""));
  declare_parameter("mission_window_start", std::string(""));
  declare_parameter("mission_window_end", std::string(""));
  declare_parameter("slam_backend", std::string("map_builder"));
  declare_parameter("gaussian_mode", std::string("voxel_gaussians"));
  declare_parameter("frame_id", std::string("map"));
  declare_parameter("map_frame", std::string("map"));
  declare_parameter("odom_frame", std::string("odom"));
  declare_parameter("base_frame", std::string("base_footprint"));
  declare_parameter("navsat_topic", std::string(kDefaultNavSatTopic));
  declare_parameter("scan_topic", std::string(kDefaultScanTopic));
  declare_parameter("seeded_map_frame", std::string("map"));
  declare_parameter("fromll_service", std::string("/fromLL"));
  declare_parameter("auto_start_mission", true);
  declare_parameter("repeat_mission", false);
  declare_parameter("publish_seeded_map_to_odom", false);
  declare_parameter("map_alignment_publish_period_seconds", 0.5);
  declare_parameter("global_map_resolution_m", 0.05);
  declare_parameter("local_map_size_m", 10.0);
  declare_parameter("runtime_costmap_save_period_seconds", 10.0);
  declare_parameter("static_obstacle_min_observations", 6);
  declare_parameter("static_obstacle_min_occupied_fraction", 0.75);
  declare_parameter("max_segments_per_goal", 4);
  declare_parameter("max_waypoint_spacing_m", 0.5);
  declare_parameter("pad_live_map_to_minimum_size", true);
  declare_parameter("min_global_map_size_m", 10.0);
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
  saved_costmap_yaml_ = get_parameter("saved_costmap_yaml").as_string();
  mission_costmap_yaml_ = get_parameter("mission_costmap_yaml").as_string();
  mission_window_start_ = get_parameter("mission_window_start").as_string();
  mission_window_end_ = get_parameter("mission_window_end").as_string();
  slam_backend_ = get_parameter("slam_backend").as_string();
  gaussian_mode_ = get_parameter("gaussian_mode").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  map_frame_id_ = get_parameter("map_frame").as_string();
  odom_frame_id_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  navsat_topic_ = get_parameter("navsat_topic").as_string();
  scan_topic_ = get_parameter("scan_topic").as_string();
  seeded_map_frame_id_ = get_parameter("seeded_map_frame").as_string();
  fromll_service_name_ = get_parameter("fromll_service").as_string();
  end_mission_service_name_ = get_parameter("end_mission_service").as_string();
  waypoint_follower_state_service_name_ =
    get_parameter("waypoint_follower_state_service").as_string();
  auto_start_mission_ = get_parameter("auto_start_mission").as_bool();
  repeat_mission_ = get_parameter("repeat_mission").as_bool();
  publish_seeded_map_to_odom_ = get_parameter("publish_seeded_map_to_odom").as_bool();
  global_map_resolution_m_ = get_parameter("global_map_resolution_m").as_double();
  local_map_size_m_ = get_parameter("local_map_size_m").as_double();
  runtime_costmap_save_period_seconds_ = get_parameter("runtime_costmap_save_period_seconds").as_double();
  static_obstacle_min_observations_ =
    static_cast<int>(get_parameter("static_obstacle_min_observations").as_int());
  static_obstacle_min_occupied_fraction_ =
    get_parameter("static_obstacle_min_occupied_fraction").as_double();
  max_segments_per_goal_ = static_cast<int>(get_parameter("max_segments_per_goal").as_int());
  max_waypoint_spacing_m_ = get_parameter("max_waypoint_spacing_m").as_double();
  pad_live_map_to_minimum_size_ = get_parameter("pad_live_map_to_minimum_size").as_bool();
  min_global_map_size_m_ = get_parameter("min_global_map_size_m").as_double();
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

  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&MappingNode::handleScan, this, std::placeholders::_1));
  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "localization/odometry_fused",
    50,
    std::bind(&MappingNode::handleOdometry, this, std::placeholders::_1));
  navsat_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    navsat_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MappingNode::handleNavSat, this, std::placeholders::_1));
  status_publisher_ = create_publisher<std_msgs::msg::String>("mapping/status", 10);
  live_map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "mapping/occupancy_grid",
    rclcpp::QoS(1).reliable().transient_local());
  global_costmap_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "mapping/global_costmap",
    rclcpp::QoS(1).reliable().transient_local());
  live_map_metadata_publisher_ = create_publisher<nav_msgs::msg::MapMetaData>(
    "mapping/occupancy_grid_metadata",
    rclcpp::QoS(1).reliable().transient_local());
  waypoint_path_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
    "mapping/waypoint_path",
    rclcpp::QoS(1).reliable().transient_local());

  mission_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  status_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  fromll_client_ = create_client<fusioncore_ros::srv::FromLL>(
    fromll_service_name_,
    rclcpp::ServicesQoS(),
    mission_callback_group_);
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
  if (runtime_costmap_save_period_seconds_ > 0.0) {
    runtime_costmap_save_timer_ = create_wall_timer(
      std::chrono::duration<double>(runtime_costmap_save_period_seconds_),
      std::bind(&MappingNode::persistRuntimeCostmapArtifact, this),
      status_callback_group_);
  }

  loadSavedCostmapIfConfigured();
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

void MappingNode::handleScan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  last_scan_frame_id_ = message->header.frame_id;
  last_scan_time_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() :
    rclcpp::Time(message->header.stamp);
  have_scan_ = true;
  if (latest_odometry_pose_ready_) {
    integrateScanIntoGlobalMap(*message);
  }
}

void MappingNode::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  latest_odometry_position_ = message->pose.pose.position;
  latest_odometry_orientation_ = message->pose.pose.orientation;
  latest_odometry_pose_ready_ = true;

  const auto & position = message->pose.pose.position;
  bool should_record_path_sample = true;
  if (!traveled_path_points_.empty()) {
    const auto & last = traveled_path_points_.back();
    const double dx = position.x - last.x;
    const double dy = position.y - last.y;
    if ((dx * dx + dy * dy) < 0.01) {
      should_record_path_sample = false;
    }
  }

  if (should_record_path_sample) {
    traveled_path_points_.push_back(position);
  }
  if (should_record_path_sample && !manual_mapping_mode_) {
    std::lock_guard<std::mutex> lock(synchronized_path_mutex_);
    if (latest_raw_navsat_sample_.has_value()) {
      tf2::Quaternion quaternion;
      tf2::fromMsg(message->pose.pose.orientation, quaternion);
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
      SynchronizedPathSample synchronized_sample;
      synchronized_sample.odom_position = position;
      synchronized_sample.yaw = yaw;
      synchronized_sample.odom_stamp = message->header.stamp.sec == 0 &&
        message->header.stamp.nanosec == 0 ? now() : rclcpp::Time(message->header.stamp);
      synchronized_sample.raw_navsat = *latest_raw_navsat_sample_;
      synchronized_path_samples_.push_back(synchronized_sample);
    }
  }
  if (should_record_path_sample) {
    writeActualPathArtifact();
    writeActualPathNavSatArtifact();
  }
}

void MappingNode::handleNavSat(const sensor_msgs::msg::NavSatFix::SharedPtr message)
{
  if (!message) {
    return;
  }

  RawNavSatSample sample;
  sample.longitude = message->longitude;
  sample.latitude = message->latitude;
  sample.altitude = message->altitude;
  sample.stamp = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() : rclcpp::Time(message->header.stamp);

  std::lock_guard<std::mutex> lock(synchronized_path_mutex_);
  latest_raw_navsat_sample_ = sample;
}

void MappingNode::loadSavedCostmapIfConfigured()
{
  if (saved_costmap_yaml_.empty()) {
    return;
  }

  const std::string resolved_path = resolveRuntimePath(saved_costmap_yaml_);
  std::error_code filesystem_error;
  if (!std::filesystem::is_regular_file(resolved_path, filesystem_error)) {
    RCLCPP_WARN(
      get_logger(),
      "Saved costmap yaml was configured as '%s' but no file was found there.",
      resolved_path.c_str());
    return;
  }

  try {
    initializeMapFromArtifact(loadCostmapArtifactFromYaml(resolved_path));
    publishGlobalMaps();
    RCLCPP_INFO(
      get_logger(),
      "Loaded saved costmap artifact from %s into %s.",
      resolved_path.c_str(),
      map_frame_id_.c_str());
  } catch (const std::exception & exception) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to load saved costmap artifact from %s: %s",
      resolved_path.c_str(),
      exception.what());
  }
}

void MappingNode::initializeMapFromArtifact(const LoadedCostmapArtifact & artifact)
{
  if (
    artifact.width_cells == 0U || artifact.height_cells == 0U ||
    artifact.resolution <= 0.0 || artifact.costs.empty())
  {
    return;
  }

  latest_live_map_.header.stamp = now();
  latest_live_map_.header.frame_id = map_frame_id_;
  latest_live_map_.info.map_load_time = latest_live_map_.header.stamp;
  latest_live_map_.info.resolution = static_cast<float>(artifact.resolution);
  latest_live_map_.info.width = artifact.width_cells;
  latest_live_map_.info.height = artifact.height_cells;
  latest_live_map_.info.origin.position.x = artifact.origin_x;
  latest_live_map_.info.origin.position.y = artifact.origin_y;
  latest_live_map_.info.origin.position.z = 0.0;
  latest_live_map_.info.origin.orientation.w = 1.0;
  latest_live_map_.data.assign(artifact.costs.size(), -1);

  global_map_resolution_m_ = artifact.resolution;
  global_map_scores_.assign(artifact.costs.size(), 0);
  global_map_observations_.assign(artifact.costs.size(), 0U);
  global_map_occupied_observations_.assign(artifact.costs.size(), 0U);
  global_map_free_observations_.assign(artifact.costs.size(), 0U);

  for (std::size_t index = 0U; index < artifact.costs.size(); ++index) {
    const double occupied_ratio = static_cast<double>(artifact.costs.at(index)) / 255.0;
    if (occupied_ratio >= artifact.occupied_thresh) {
      latest_live_map_.data.at(index) = 100;
      global_map_scores_.at(index) = 20;
      global_map_observations_.at(index) = 1U;
      global_map_occupied_observations_.at(index) = 1U;
    } else if (occupied_ratio <= artifact.free_thresh) {
      latest_live_map_.data.at(index) = 0;
      global_map_scores_.at(index) = -20;
      global_map_observations_.at(index) = 1U;
      global_map_free_observations_.at(index) = 1U;
    }
  }

  seeded_runtime_map_ = latest_live_map_;
  seeded_runtime_map_ready_ = true;
  have_scan_ = false;
  live_map_ready_ = true;
}

nav_msgs::msg::OccupancyGrid MappingNode::buildStaticRuntimeCostmapArtifact() const
{
  nav_msgs::msg::OccupancyGrid artifact_map;
  if (latest_live_map_.info.width > 0U && latest_live_map_.info.height > 0U) {
    artifact_map = latest_live_map_;
  } else if (latest_padded_live_map_.info.width > 0U && latest_padded_live_map_.info.height > 0U) {
    artifact_map = latest_padded_live_map_;
  } else {
    return artifact_map;
  }

  artifact_map.header.stamp = now();
  artifact_map.info.map_load_time = artifact_map.header.stamp;
  artifact_map.data.assign(
    static_cast<std::size_t>(artifact_map.info.width) * artifact_map.info.height,
    -1);

  if (seeded_runtime_map_ready_) {
    for (uint32_t row = 0U; row < seeded_runtime_map_.info.height; ++row) {
      for (uint32_t col = 0U; col < seeded_runtime_map_.info.width; ++col) {
        const std::size_t source_index =
          static_cast<std::size_t>(row) * seeded_runtime_map_.info.width + col;
        const int8_t seeded_value = seeded_runtime_map_.data.at(source_index);
        if (seeded_value < 0) {
          continue;
        }

        const double world_x = seeded_runtime_map_.info.origin.position.x +
          (static_cast<double>(col) + 0.5) * seeded_runtime_map_.info.resolution;
        const double world_y = seeded_runtime_map_.info.origin.position.y +
          (static_cast<double>(row) + 0.5) * seeded_runtime_map_.info.resolution;
        int destination_x = 0;
        int destination_y = 0;
        if (!worldToGrid(artifact_map, world_x, world_y, destination_x, destination_y)) {
          continue;
        }

        const std::size_t destination_index =
          static_cast<std::size_t>(destination_y) * artifact_map.info.width +
          static_cast<std::size_t>(destination_x);
        artifact_map.data.at(destination_index) = seeded_value;
      }
    }
  }

  if (
    artifact_map.data.size() != latest_live_map_.data.size() ||
    latest_live_map_.data.size() != global_map_observations_.size() ||
    latest_live_map_.data.size() != global_map_occupied_observations_.size())
  {
    return artifact_map;
  }

  for (std::size_t index = 0U; index < latest_live_map_.data.size(); ++index) {
    const uint16_t total_observations = global_map_observations_.at(index);
    if (total_observations < static_cast<uint16_t>(std::max(1, static_obstacle_min_observations_))) {
      continue;
    }

    const uint16_t occupied_observations = global_map_occupied_observations_.at(index);
    const double occupied_fraction =
      static_cast<double>(occupied_observations) / static_cast<double>(total_observations);
    if (
      occupied_fraction < static_obstacle_min_occupied_fraction_ ||
      latest_live_map_.data.at(index) < 65)
    {
      continue;
    }

    artifact_map.data.at(index) = 100;
  }

  return artifact_map;
}

void MappingNode::persistRuntimeCostmapArtifact()
{
  if (saved_costmap_yaml_.empty()) {
    return;
  }

  const nav_msgs::msg::OccupancyGrid map_to_save = buildStaticRuntimeCostmapArtifact();
  if (map_to_save.info.width == 0U || map_to_save.info.height == 0U) {
    return;
  }

  const std::filesystem::path yaml_path(resolveRuntimePath(saved_costmap_yaml_));
  try {
    std::optional<nlohmann::json> georeference_metadata;
    if (!manual_mapping_mode_ && !synchronized_path_samples_.empty()) {
      std::vector<MapPoint> local_trace;
      std::vector<GeoPoint> geo_trace;
      local_trace.reserve(synchronized_path_samples_.size());
      geo_trace.reserve(synchronized_path_samples_.size());
      for (const auto & sample : synchronized_path_samples_) {
        local_trace.push_back({sample.odom_position.x, sample.odom_position.y});
        geo_trace.push_back({sample.raw_navsat.latitude, sample.raw_navsat.longitude});
      }
      const std::string companion_file = actual_path_navsat_output_file_.empty() ? std::string{} :
        std::filesystem::path(actual_path_navsat_output_file_).filename().string();
      georeference_metadata = buildGeoReferenceMetadata(local_trace, geo_trace, companion_file);
      if (georeference_metadata.has_value()) {
        (*georeference_metadata)["source_crs"] = "EPSG:4326";
      }
    }
    saveOccupancyGridArtifact(map_to_save, yaml_path, georeference_metadata);
  } catch (const std::exception & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Failed to persist runtime costmap artifact to %s: %s",
      yaml_path.string().c_str(),
      exception.what());
  }
}

std::string MappingNode::composeMapBuilderStatus() const
{
  const bool ready = have_scan_ && latest_odometry_pose_ready_;
  const bool have_global_map = latest_live_map_.info.width > 0U && latest_live_map_.info.height > 0U;
  bool have_scan_to_base_tf = false;
  bool have_map_to_odom_tf = false;
  bool have_base_frame = false;
  bool have_scan_frame = false;

  if (tf_buffer_->_frameExists(base_frame_)) {
    have_base_frame = true;
  }

  if (have_scan_ && !last_scan_frame_id_.empty()) {
    have_scan_frame = tf_buffer_->_frameExists(last_scan_frame_id_);
  }

  if (have_scan_ && have_base_frame && have_scan_frame && !last_scan_frame_id_.empty()) {
    have_scan_to_base_tf = tf_buffer_->canTransform(
      base_frame_,
      last_scan_frame_id_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.05));
  }

  if (tf_buffer_->_frameExists(map_frame_id_) && tf_buffer_->_frameExists(odom_frame_id_)) {
    have_map_to_odom_tf = tf_buffer_->canTransform(
      map_frame_id_,
      odom_frame_id_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.05));
  }

  std::ostringstream stream;
  stream
    << "ready=" << (ready ? "true" : "false")
    << "; tracking=" << (have_global_map ? "true" : "false")
    << "; scan_tf=" << (have_scan_to_base_tf ? "true" : "false")
    << "; base_frame=" << (have_base_frame ? "true" : "false")
    << "; scan_frame_exists=" << (have_scan_frame ? "true" : "false")
    << "; map_tf=" << (have_map_to_odom_tf ? "true" : "false")
    << "; global_map=" << (have_global_map ? "true" : "false")
    << "; scan_frame=" << (last_scan_frame_id_.empty() ? "<none>" : last_scan_frame_id_)
    << "; pose_frame=" << (latest_odometry_pose_ready_ ? odom_frame_id_ : "<none>");
  return stream.str();
}

void MappingNode::publishCoordinatorStatus()
{
  last_map_builder_status_ = composeMapBuilderStatus();
  std_msgs::msg::String message;
  message.data =
    "mapping_node mission=" + mission_file_ +
    "; mission_type=" + mission_type_ +
    "; execution_mode=" + execution_mode_ +
    "; route=" + mission_route_file_ +
    "; mission_id=" + mission_id_ +
    "; mission_output_directory=" + mission_output_directory_ +
    "; saved_costmap_yaml=" + saved_costmap_yaml_ +
    "; mission_costmap_yaml=" + mission_costmap_yaml_ +
    "; mission_window_start=" + mission_window_start_ +
    "; mission_window_end=" + mission_window_end_ +
    "; slam_backend=" + slam_backend_ +
    "; gaussian_mode=" + gaussian_mode_ +
    "; seeded_map_to_odom=" + std::string(publish_seeded_map_to_odom_ ? "true" : "false") +
    "; global_resolution=" + std::to_string(global_map_resolution_m_) +
    "; global_map_ready=" + std::string(latest_padded_live_map_ready_ ? "true" : "false") +
    "; mission_loaded=" + std::string(mission_loaded_ ? "true" : "false") +
    "; mission_converted=" + std::string(mission_converted_ ? "true" : "false") +
    "; mission_active=" + std::string(mission_active_ ? "true" : "false") +
    "; mission_completed=" + std::string(mission_completed_ ? "true" : "false") +
    "; active_chunk=" + std::to_string(active_chunk_index_) + "/" + std::to_string(mission_chunks_.size()) +
    "; map_builder_status=" + last_map_builder_status_;
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
    {"saved_costmap_yaml", saved_costmap_yaml_},
    {"mission_costmap_yaml", mission_costmap_yaml_},
    {"mission_window_start", mission_window_start_},
    {"mission_window_end", mission_window_end_},
    {"slam_backend", slam_backend_},
    {"gaussian_mode", gaussian_mode_},
    {"frame_id", frame_id_},
    {"manual_drive_required", manual_mapping_mode_},
    {"actual_path_output_file", actual_path_output_file_},
    {"actual_path_navsat_output_file", actual_path_navsat_output_file_}};

  try {
    writeJsonDocumentAtomic(
      std::filesystem::path(mission_output_directory_) / "mapping_session.json",
      document);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to write mapping session metadata into %s",
      mission_output_directory_.c_str());
    return;
  }
}

void MappingNode::publishMapAlignmentTransform()
{
  // map -> odom is published by map_pose_node so mapping_node only builds and publishes maps.
}

void MappingNode::integrateScanIntoGlobalMap(const sensor_msgs::msg::LaserScan & message)
{
  if (message.ranges.empty() || global_map_resolution_m_ <= 0.0) {
    return;
  }

  geometry_msgs::msg::TransformStamped map_from_scan;
  try {
    const rclcpp::Time transform_stamp = message.header.stamp.sec == 0 &&
      message.header.stamp.nanosec == 0 ? now() : rclcpp::Time(message.header.stamp);
    map_from_scan = tf_buffer_->lookupTransform(
      map_frame_id_,
      message.header.frame_id,
      transform_stamp,
      tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Skipping scan integration because %s -> %s is unavailable: %s",
      message.header.frame_id.c_str(),
      map_frame_id_.c_str(),
      exception.what());
    return;
  }

  tf2::Transform scan_to_map;
  tf2::fromMsg(map_from_scan.transform, scan_to_map);
  const tf2::Vector3 sensor_origin = scan_to_map * tf2::Vector3(0.0, 0.0, 0.0);

  struct Endpoint
  {
    double x;
    double y;
  };
  std::vector<Endpoint> hit_points;
  hit_points.reserve(message.ranges.size());

  double min_x = sensor_origin.x();
  double min_y = sensor_origin.y();
  double max_x = sensor_origin.x();
  double max_y = sensor_origin.y();
  double angle = message.angle_min;
  const double effective_max_range = message.range_max > 0.0 ?
    message.range_max : std::numeric_limits<double>::infinity();
  for (const float range : message.ranges) {
    const double range_value = static_cast<double>(range);
    if (!std::isfinite(range_value) || range_value < message.range_min || range_value > effective_max_range) {
      angle += message.angle_increment;
      continue;
    }

    const tf2::Vector3 point_in_map = scan_to_map * tf2::Vector3(
      range_value * std::cos(angle),
      range_value * std::sin(angle),
      0.0);
    hit_points.push_back({point_in_map.x(), point_in_map.y()});
    min_x = std::min(min_x, point_in_map.x());
    min_y = std::min(min_y, point_in_map.y());
    max_x = std::max(max_x, point_in_map.x());
    max_y = std::max(max_y, point_in_map.y());
    angle += message.angle_increment;
  }

  geometry_msgs::msg::Point center;
  center.x = sensor_origin.x();
  center.y = sensor_origin.y();
  center.z = 0.0;
  if (latest_live_map_.info.width == 0U || latest_live_map_.info.height == 0U) {
    initializeGlobalMap(center);
  }
  expandGlobalMapToFit(min_x, min_y, max_x, max_y);

  int start_x = 0;
  int start_y = 0;
  if (!worldToGrid(latest_live_map_, sensor_origin.x(), sensor_origin.y(), start_x, start_y)) {
    return;
  }

  for (const auto & hit_point : hit_points) {
    int end_x = 0;
    int end_y = 0;
    if (!worldToGrid(latest_live_map_, hit_point.x, hit_point.y, end_x, end_y)) {
      continue;
    }

    int current_x = start_x;
    int current_y = start_y;
    const int delta_x = std::abs(end_x - start_x);
    const int delta_y = std::abs(end_y - start_y);
    const int step_x = start_x < end_x ? 1 : -1;
    const int step_y = start_y < end_y ? 1 : -1;
    int error = delta_x - delta_y;

    while (current_x != end_x || current_y != end_y) {
      markCellFree(current_x, current_y);
      const int doubled_error = 2 * error;
      if (doubled_error > -delta_y) {
        error -= delta_y;
        current_x += step_x;
      }
      if (doubled_error < delta_x) {
        error += delta_x;
        current_y += step_y;
      }
    }
    markCellOccupied(end_x, end_y);
  }

  publishGlobalMaps();
}

void MappingNode::writeActualPathArtifact() const
{
  if (actual_path_output_file_.empty()) {
    return;
  }

  std::filesystem::create_directories(std::filesystem::path(actual_path_output_file_).parent_path());
  std::vector<SynchronizedPathSample> synchronized_samples;
  {
    std::lock_guard<std::mutex> lock(synchronized_path_mutex_);
    synchronized_samples = synchronized_path_samples_;
  }

  const std::string navsat_companion_file = actual_path_navsat_output_file_.empty() ? std::string{} :
    std::filesystem::path(actual_path_navsat_output_file_).filename().string();
  nlohmann::json document;
  if (!synchronized_samples.empty()) {
    document = buildSynchronizedLocalPathGeoJson(synchronized_samples, navsat_companion_file);
  } else {
    nlohmann::json coordinates = nlohmann::json::array();
    for (const auto & point : traveled_path_points_) {
      coordinates.push_back({point.x, point.y});
    }
    document = buildLocalPathGeoJson(coordinates, navsat_companion_file);
  }

  try {
    writeJsonDocumentAtomic(actual_path_output_file_, document);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to write actual path artifact into %s",
      actual_path_output_file_.c_str());
    return;
  }
}

void MappingNode::writeActualPathNavSatArtifact() const
{
  if (manual_mapping_mode_ || actual_path_navsat_output_file_.empty()) {
    return;
  }

  std::vector<SynchronizedPathSample> synchronized_samples;
  {
    std::lock_guard<std::mutex> lock(synchronized_path_mutex_);
    synchronized_samples = synchronized_path_samples_;
  }
  if (synchronized_samples.empty()) {
    return;
  }

  const std::string local_companion_file = actual_path_output_file_.empty() ? std::string{} :
    std::filesystem::path(actual_path_output_file_).filename().string();
  const nlohmann::json document =
    buildSynchronizedNavSatGeoJson(synchronized_samples, local_companion_file);

  try {
    writeJsonDocumentAtomic(actual_path_navsat_output_file_, document);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to write actual navsat path artifact into %s",
      actual_path_navsat_output_file_.c_str());
  }
}

void MappingNode::writeAnchoredMissionRouteArtifact(const std::string & source_coordinate_frame) const
{
  const std::string route_path = routeGeoJsonPath();
  if (route_path.empty() || mission_route_.empty()) {
    return;
  }

  nlohmann::json coordinates = nlohmann::json::array();
  for (const auto & pose : mission_route_) {
    coordinates.push_back({pose.pose.position.x, pose.pose.position.y});
  }

  nlohmann::json properties{
    {"name", mission_id_.empty() ? "mission_route" : mission_id_ + "_path"},
    {"coordinate_frame", odom_frame_id_},
    {"anchored_at_mission_start", true},
    {"anchored_from_coordinate_frame", source_coordinate_frame}};

  const nlohmann::json document{
    {"type", "FeatureCollection"},
    {"features", nlohmann::json::array({
      {
        {"type", "Feature"},
        {"properties", properties},
        {"geometry", {{"type", "LineString"}, {"coordinates", coordinates}}}
      }
    })}};

  try {
    writeJsonDocumentAtomic(route_path, document);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to write anchored mission route artifact into %s",
      route_path.c_str());
  }
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

  const std::string coordinate_frame = coordinates.front().frame_id;
  const bool uses_base_footprint_anchor =
    coordinate_frame == "base_footprint" || coordinate_frame == "local";
  const bool uses_geographic_frame = !coordinates.front().use_local_frame;
  if (uses_base_footprint_anchor && !latest_odometry_pose_ready_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for odometry pose before anchoring mission route from %s into %s.",
      coordinate_frame.c_str(),
      frame_id_.c_str());
    return;
  }

  if (uses_geographic_frame && !fromll_client_->wait_for_service(std::chrono::seconds(1))) {
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

  geometry_msgs::msg::Point mission_anchor_position;
  geometry_msgs::msg::Quaternion mission_anchor_orientation;
  if (uses_base_footprint_anchor) {
    if (!mission_anchor_pose_ready_) {
      mission_anchor_position_ = latest_odometry_position_;
      mission_anchor_orientation_ = latest_odometry_orientation_;
      mission_anchor_pose_ready_ = true;
    }
    mission_anchor_position = mission_anchor_position_;
    mission_anchor_orientation = mission_anchor_orientation_;
  }

  for (const MissionCoordinate & coordinate : coordinates) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    if (coordinate.frame_id == "base_footprint" || coordinate.frame_id == "local") {
      const auto odom_point = transformBaseFootprintPointToOdom(
        coordinate.x,
        coordinate.y,
        mission_anchor_position,
        mission_anchor_orientation);
      pose.pose.position.x = odom_point.x;
      pose.pose.position.y = odom_point.y;
      mission_route_.push_back(pose);
      continue;
    }

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
  const bool is_builtin_mission = mission_type_.rfind("builtin_", 0) == 0;
  if (!is_builtin_mission) {
    mission_route_ = densifyRoute(mission_route_, max_waypoint_spacing_m_);
  }
  mission_route_ = orientRoute(std::move(mission_route_));
  if (uses_base_footprint_anchor) {
    writeAnchoredMissionRouteArtifact(coordinate_frame);
  }
  mission_chunks_ = chunkRoute(mission_route_);
  mission_converted_ = !mission_chunks_.empty();
  publishRouteMarker();
  if (uses_base_footprint_anchor) {
    tf2::Quaternion anchor_quaternion;
    tf2::fromMsg(mission_anchor_orientation, anchor_quaternion);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(anchor_quaternion).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(
      get_logger(),
      "Anchored mission route once from %s at odom x=%.3f y=%.3f yaw=%.3f rad (%.1f deg).",
      coordinate_frame.c_str(),
      mission_anchor_position.x,
      mission_anchor_position.y,
      yaw,
      yaw * 180.0 / M_PI);
  }
  RCLCPP_INFO(
    get_logger(),
    "Prepared %zu mission waypoint(s) from %s into %s and %zu Nav2 chunk(s)%s.",
    mission_route_.size(),
    coordinate_frame.c_str(),
    frame_id_.c_str(),
    mission_chunks_.size(),
    is_builtin_mission ? " without densification for builtin mission" : "");
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

  const bool odom_only_navigation = map_frame_id_ == odom_frame_id_;

  if (!odom_only_navigation && !live_map_ready_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for the first live padded map before dispatching Nav2 waypoints.");
    return;
  }

  if (!odom_only_navigation && latest_odometry_pose_ready_ && latest_padded_live_map_ready_) {
    try {
      const auto map_to_odom = tf_buffer_->lookupTransform(
        latest_padded_live_map_.header.frame_id,
        odom_frame_id_,
        tf2::TimePointZero);
      geometry_msgs::msg::PointStamped odom_point;
      odom_point.header.frame_id = odom_frame_id_;
      odom_point.point =
        mission_anchor_pose_ready_ ? mission_anchor_position_ : latest_odometry_position_;
      geometry_msgs::msg::PointStamped map_point;
      tf2::doTransform(odom_point, map_point, map_to_odom);

      const double resolution = static_cast<double>(latest_padded_live_map_.info.resolution);
      const double min_x = latest_padded_live_map_.info.origin.position.x;
      const double min_y = latest_padded_live_map_.info.origin.position.y;
      const double max_x =
        min_x + static_cast<double>(latest_padded_live_map_.info.width) * resolution;
      const double max_y =
        min_y + static_cast<double>(latest_padded_live_map_.info.height) * resolution;

      if (map_point.point.x < min_x || map_point.point.x >= max_x ||
        map_point.point.y < min_y || map_point.point.y >= max_y)
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Waiting for Nav2 startup map to contain robot start pose. Pose in %s is x=%.3f y=%.3f, map bounds are [%.3f, %.3f] x [%.3f, %.3f].",
          latest_padded_live_map_.header.frame_id.c_str(),
          map_point.point.x,
          map_point.point.y,
          min_x,
          max_x,
          min_y,
          max_y);
        return;
      }
    } catch (const tf2::TransformException & exception) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting to verify robot start pose against the padded startup map: %s",
        exception.what());
      return;
    }
  }

  nav2_msgs::action::FollowWaypoints::Goal goal;
  const rclcpp::Time latest_transform_stamp(0, 0, get_clock()->get_clock_type());
  try {
    if (odom_only_navigation) {
      for (auto pose : mission_chunks_.at(active_chunk_index_)) {
        pose.header.stamp = latest_transform_stamp;
        pose.header.frame_id = odom_frame_id_;
        goal.poses.push_back(pose);
      }
    } else {
      const auto map_to_odom = tf_buffer_->lookupTransform(
        map_frame_id_,
        odom_frame_id_,
        tf2::TimePointZero);
      for (auto pose : mission_chunks_.at(active_chunk_index_)) {
        pose.header.stamp = latest_transform_stamp;
        geometry_msgs::msg::PoseStamped transformed_pose;
        tf2::doTransform(pose, transformed_pose, map_to_odom);
        transformed_pose.header.frame_id = map_frame_id_;
        transformed_pose.header.stamp = latest_transform_stamp;
        goal.poses.push_back(transformed_pose);
      }
    }
  } catch (const tf2::TransformException & exception) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Waiting to transform mission chunk from %s into %s before dispatching Nav2 waypoints: %s",
      odom_frame_id_.c_str(),
      map_frame_id_.c_str(),
      exception.what());
    return;
  }

  for (std::size_t index = 0U; index < goal.poses.size(); ++index) {
    const auto & pose = goal.poses.at(index);
    const tf2::Quaternion orientation(
      pose.pose.orientation.x,
      pose.pose.orientation.y,
      pose.pose.orientation.z,
      pose.pose.orientation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(
      get_logger(),
      "Chunk %zu waypoint %zu/%zu: x=%.3f y=%.3f yaw=%.3f rad (%.1f deg)",
      active_chunk_index_ + 1U,
      index + 1U,
      goal.poses.size(),
      pose.pose.position.x,
      pose.pose.position.y,
      yaw,
      yaw * 180.0 / M_PI);
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
  request->requester = "mapping_node";
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
  marker.ns = "mapping_waypoint_path";
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
  waypoint_path_publisher_->publish(marker);
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
    std::string coordinate_frame = "odom";
    if (feature.contains("properties") && feature.at("properties").is_object()) {
      const auto & properties = feature.at("properties");
      if (properties.contains("coordinate_frame") && properties.at("coordinate_frame").is_string()) {
        coordinate_frame = to_lower(properties.at("coordinate_frame").get<std::string>());
        use_local_frame =
          coordinate_frame == "odom" ||
          coordinate_frame == "local" ||
          coordinate_frame == "base_footprint";
      }
    }

    std::vector<MissionCoordinate> coordinates;
    for (const auto & coordinate : geometry.at("coordinates")) {
      coordinates.push_back(MissionCoordinate{
        coordinate.at(0).get<double>(),
        coordinate.at(1).get<double>(),
        use_local_frame,
        coordinate_frame});
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
  poses = orientRoute(std::move(poses));
  (void)coordinates;
  return poses;
}

nav_msgs::msg::OccupancyGrid MappingNode::padLiveMap(
  const nav_msgs::msg::OccupancyGrid & message)
{
  nav_msgs::msg::OccupancyGrid padded = message;
  if (!pad_live_map_to_minimum_size_ || min_global_map_size_m_ <= 0.0) {
    return padded;
  }

  const double resolution = static_cast<double>(message.info.resolution);
  if (resolution <= 0.0) {
    return padded;
  }

  double anchor_x = 0.0;
  double anchor_y = 0.0;
  bool anchor_available = false;
  if (latest_odometry_pose_ready_) {
    geometry_msgs::msg::Point odom_anchor =
      mission_anchor_pose_ready_ ? mission_anchor_position_ : latest_odometry_position_;
    try {
      const auto map_to_odom = tf_buffer_->lookupTransform(
        message.header.frame_id,
        odom_frame_id_,
        tf2::TimePointZero);
      geometry_msgs::msg::PointStamped odom_point;
      odom_point.header.frame_id = odom_frame_id_;
      odom_point.point = odom_anchor;
      geometry_msgs::msg::PointStamped map_point;
      tf2::doTransform(odom_point, map_point, map_to_odom);
      anchor_x = map_point.point.x;
      anchor_y = map_point.point.y;
      anchor_available = true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Unable to center padded live map around robot pose yet: %s",
        exception.what());
    }
  }

  const double source_min_x = message.info.origin.position.x;
  const double source_min_y = message.info.origin.position.y;
  const double source_max_x =
    source_min_x + static_cast<double>(message.info.width) * resolution;
  const double source_max_y =
    source_min_y + static_cast<double>(message.info.height) * resolution;

  if (!padded_live_map_bounds_ready_) {
    if (!latest_odometry_pose_ready_ || !anchor_available) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting to initialize the mission startup map until the robot reference pose can be centered in %s.",
        message.header.frame_id.c_str());
      return nav_msgs::msg::OccupancyGrid{};
    }

    padded_live_map_min_x_ = anchor_x - min_global_map_size_m_ * 0.5;
    padded_live_map_max_x_ = anchor_x + min_global_map_size_m_ * 0.5;
    padded_live_map_min_y_ = anchor_y - min_global_map_size_m_ * 0.5;
    padded_live_map_max_y_ = anchor_y + min_global_map_size_m_ * 0.5;
    padded_live_map_bounds_ready_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Initialized persistent padded live-map bounds around the mission start pose in %s: [%.3f, %.3f] x [%.3f, %.3f].",
      message.header.frame_id.c_str(),
      padded_live_map_min_x_,
      padded_live_map_max_x_,
      padded_live_map_min_y_,
      padded_live_map_max_y_);
  }

  bool expanded_bounds = false;
  if (source_min_x < padded_live_map_min_x_) {
    padded_live_map_min_x_ = source_min_x;
    expanded_bounds = true;
  }
  if (source_min_y < padded_live_map_min_y_) {
    padded_live_map_min_y_ = source_min_y;
    expanded_bounds = true;
  }
  if (source_max_x > padded_live_map_max_x_) {
    padded_live_map_max_x_ = source_max_x;
    expanded_bounds = true;
  }
  if (source_max_y > padded_live_map_max_y_) {
    padded_live_map_max_y_ = source_max_y;
    expanded_bounds = true;
  }

  if (expanded_bounds) {
    RCLCPP_INFO(
      get_logger(),
      "Expanded persistent padded live-map bounds in %s to [%.3f, %.3f] x [%.3f, %.3f] to contain the latest SLAM map.",
      message.header.frame_id.c_str(),
      padded_live_map_min_x_,
      padded_live_map_max_x_,
      padded_live_map_min_y_,
      padded_live_map_max_y_);
  }

  double padded_min_x = padded_live_map_min_x_;
  double padded_min_y = padded_live_map_min_y_;
  double padded_max_x = padded_live_map_max_x_;
  double padded_max_y = padded_live_map_max_y_;
  padded_min_x = std::min(padded_min_x, source_min_x);
  padded_min_y = std::min(padded_min_y, source_min_y);
  padded_max_x = std::max(padded_max_x, source_max_x);
  padded_max_y = std::max(padded_max_y, source_max_y);

  const uint32_t padded_width = static_cast<uint32_t>(
    std::max(1.0, std::ceil((padded_max_x - padded_min_x) / resolution)));
  const uint32_t padded_height = static_cast<uint32_t>(
    std::max(1.0, std::ceil((padded_max_y - padded_min_y) / resolution)));
  const uint32_t left_pad = static_cast<uint32_t>(
    std::round((source_min_x - padded_min_x) / resolution));
  const uint32_t bottom_pad = static_cast<uint32_t>(
    std::round((source_min_y - padded_min_y) / resolution));

  if (padded_width == message.info.width &&
    padded_height == message.info.height &&
    left_pad == 0U &&
    bottom_pad == 0U)
  {
    return message;
  }

  padded.info.width = padded_width;
  padded.info.height = padded_height;
  padded.info.origin.position.x = padded_min_x;
  padded.info.origin.position.y = padded_min_y;
  padded.data.assign(static_cast<std::size_t>(padded_width) * padded_height, -1);

  for (uint32_t row = 0U; row < message.info.height; ++row) {
    for (uint32_t col = 0U; col < message.info.width; ++col) {
      const std::size_t source_index = static_cast<std::size_t>(row) * message.info.width + col;
      const std::size_t destination_index =
        static_cast<std::size_t>(row + bottom_pad) * padded_width + (col + left_pad);
      padded.data.at(destination_index) = message.data.at(source_index);
    }
  }

  return padded;
}

void MappingNode::initializeGlobalMap(const geometry_msgs::msg::Point & center)
{
  const double resolution = global_map_resolution_m_;
  const double size_m = std::max(min_global_map_size_m_, local_map_size_m_);
  const uint32_t width = static_cast<uint32_t>(std::max(1.0, std::ceil(size_m / resolution)));
  const uint32_t height = static_cast<uint32_t>(std::max(1.0, std::ceil(size_m / resolution)));

  latest_live_map_.header.frame_id = map_frame_id_;
  latest_live_map_.header.stamp = now();
  latest_live_map_.info.map_load_time = latest_live_map_.header.stamp;
  latest_live_map_.info.resolution = resolution;
  latest_live_map_.info.width = width;
  latest_live_map_.info.height = height;
  latest_live_map_.info.origin.position.x = center.x - (static_cast<double>(width) * resolution * 0.5);
  latest_live_map_.info.origin.position.y = center.y - (static_cast<double>(height) * resolution * 0.5);
  latest_live_map_.info.origin.position.z = 0.0;
  latest_live_map_.info.origin.orientation.w = 1.0;
  latest_live_map_.data.assign(static_cast<std::size_t>(width) * height, -1);
  global_map_scores_.assign(latest_live_map_.data.size(), 0);
  global_map_observations_.assign(latest_live_map_.data.size(), 0);
  global_map_occupied_observations_.assign(latest_live_map_.data.size(), 0U);
  global_map_free_observations_.assign(latest_live_map_.data.size(), 0U);
}

void MappingNode::expandGlobalMapToFit(
  const double min_x,
  const double min_y,
  const double max_x,
  const double max_y)
{
  if (latest_live_map_.info.width == 0U || latest_live_map_.info.height == 0U) {
    return;
  }

  const double resolution = latest_live_map_.info.resolution;
  const double current_min_x = latest_live_map_.info.origin.position.x;
  const double current_min_y = latest_live_map_.info.origin.position.y;
  const double current_max_x =
    current_min_x + static_cast<double>(latest_live_map_.info.width) * resolution;
  const double current_max_y =
    current_min_y + static_cast<double>(latest_live_map_.info.height) * resolution;

  const double expanded_min_x = std::min(current_min_x, min_x);
  const double expanded_min_y = std::min(current_min_y, min_y);
  const double expanded_max_x = std::max(current_max_x, max_x);
  const double expanded_max_y = std::max(current_max_y, max_y);
  if (
    expanded_min_x == current_min_x &&
    expanded_min_y == current_min_y &&
    expanded_max_x == current_max_x &&
    expanded_max_y == current_max_y)
  {
    return;
  }

  const uint32_t new_width = static_cast<uint32_t>(
    std::max(1.0, std::ceil((expanded_max_x - expanded_min_x) / resolution)));
  const uint32_t new_height = static_cast<uint32_t>(
    std::max(1.0, std::ceil((expanded_max_y - expanded_min_y) / resolution)));
  const uint32_t x_offset = static_cast<uint32_t>(
    std::round((current_min_x - expanded_min_x) / resolution));
  const uint32_t y_offset = static_cast<uint32_t>(
    std::round((current_min_y - expanded_min_y) / resolution));

  std::vector<int8_t> expanded_data(static_cast<std::size_t>(new_width) * new_height, -1);
  std::vector<int16_t> expanded_scores(expanded_data.size(), 0);
  std::vector<uint16_t> expanded_observations(expanded_data.size(), 0);
  std::vector<uint16_t> expanded_occupied_observations(expanded_data.size(), 0);
  std::vector<uint16_t> expanded_free_observations(expanded_data.size(), 0);

  for (uint32_t row = 0U; row < latest_live_map_.info.height; ++row) {
    for (uint32_t col = 0U; col < latest_live_map_.info.width; ++col) {
      const std::size_t source_index =
        static_cast<std::size_t>(row) * latest_live_map_.info.width + col;
      const std::size_t destination_index =
        static_cast<std::size_t>(row + y_offset) * new_width + (col + x_offset);
      expanded_data.at(destination_index) = latest_live_map_.data.at(source_index);
      expanded_scores.at(destination_index) = global_map_scores_.at(source_index);
      expanded_observations.at(destination_index) = global_map_observations_.at(source_index);
      expanded_occupied_observations.at(destination_index) =
        global_map_occupied_observations_.at(source_index);
      expanded_free_observations.at(destination_index) =
        global_map_free_observations_.at(source_index);
    }
  }

  latest_live_map_.info.width = new_width;
  latest_live_map_.info.height = new_height;
  latest_live_map_.info.origin.position.x = expanded_min_x;
  latest_live_map_.info.origin.position.y = expanded_min_y;
  latest_live_map_.data = std::move(expanded_data);
  global_map_scores_ = std::move(expanded_scores);
  global_map_observations_ = std::move(expanded_observations);
  global_map_occupied_observations_ = std::move(expanded_occupied_observations);
  global_map_free_observations_ = std::move(expanded_free_observations);
}

bool MappingNode::worldToGrid(
  const nav_msgs::msg::OccupancyGrid & map,
  const double world_x,
  const double world_y,
  int & grid_x,
  int & grid_y) const
{
  const double resolution = map.info.resolution;
  if (resolution <= 0.0) {
    return false;
  }
  grid_x = static_cast<int>(std::floor((world_x - map.info.origin.position.x) / resolution));
  grid_y = static_cast<int>(std::floor((world_y - map.info.origin.position.y) / resolution));
  return grid_x >= 0 && grid_y >= 0 &&
    grid_x < static_cast<int>(map.info.width) && grid_y < static_cast<int>(map.info.height);
}

void MappingNode::markCellFree(const int grid_x, const int grid_y)
{
  if (
    grid_x < 0 || grid_y < 0 ||
    grid_x >= static_cast<int>(latest_live_map_.info.width) ||
    grid_y >= static_cast<int>(latest_live_map_.info.height))
  {
    return;
  }

  const std::size_t index =
    static_cast<std::size_t>(grid_y) * latest_live_map_.info.width + static_cast<std::size_t>(grid_x);
  ++global_map_observations_.at(index);
  ++global_map_free_observations_.at(index);
  global_map_scores_.at(index) = static_cast<int16_t>(std::max<int>(-20, global_map_scores_.at(index) - 1));
}

void MappingNode::markCellOccupied(const int grid_x, const int grid_y)
{
  if (
    grid_x < 0 || grid_y < 0 ||
    grid_x >= static_cast<int>(latest_live_map_.info.width) ||
    grid_y >= static_cast<int>(latest_live_map_.info.height))
  {
    return;
  }

  const std::size_t index =
    static_cast<std::size_t>(grid_y) * latest_live_map_.info.width + static_cast<std::size_t>(grid_x);
  ++global_map_observations_.at(index);
  ++global_map_occupied_observations_.at(index);
  global_map_scores_.at(index) = static_cast<int16_t>(std::min<int>(20, global_map_scores_.at(index) + 3));
}

void MappingNode::publishGlobalMaps()
{
  if (latest_live_map_.data.size() != global_map_scores_.size() ||
    latest_live_map_.data.size() != global_map_observations_.size())
  {
    return;
  }

  for (std::size_t index = 0U; index < latest_live_map_.data.size(); ++index) {
    if (global_map_observations_.at(index) == 0U) {
      latest_live_map_.data.at(index) = -1;
      continue;
    }
    const int score = std::clamp<int>(global_map_scores_.at(index), -20, 20);
    const int occupancy = score >= 0 ?
      std::min(100, 50 + score * 2) :
      std::max(0, 50 + score * 2);
    latest_live_map_.data.at(index) = static_cast<int8_t>(occupancy);
  }

  latest_live_map_.header.stamp = now();
  latest_live_map_.info.map_load_time = latest_live_map_.header.stamp;

  latest_padded_live_map_ = padLiveMap(latest_live_map_);
  if (latest_padded_live_map_.info.width == 0U || latest_padded_live_map_.info.height == 0U) {
    return;
  }
  latest_padded_live_map_ready_ = true;
  live_map_publisher_->publish(latest_padded_live_map_);
  global_costmap_publisher_->publish(latest_padded_live_map_);
  live_map_metadata_publisher_->publish(latest_padded_live_map_.info);
  live_map_ready_ = true;
  persistRuntimeCostmapArtifact();
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
