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
#include <sensor_msgs/msg/imu.hpp>
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

constexpr char kDefaultMissionRoutePath[] = "";
constexpr char kNavigateThroughPosesExecutionMode[] = "navigate_through_poses";
constexpr char kManualMappingExecutionMode[] = "manual_mapping";
constexpr char kScheduledMissionType[] = "vda5050_scheduled_mission";
constexpr char kLocalScheduledMissionType[] = "vda5050_scheduled_mission_local";
constexpr char kDefaultNavSatTopic[] = "gnss/navsat";
constexpr char kDefaultScanTopic[] = "depth_camera/scan";
constexpr double kEarthRadiusMeters = 6378137.0;
constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kTrinaryOccupiedThreshold = 0.65;
constexpr double kTrinaryFreeThreshold = 0.196;
constexpr unsigned char kUnknownTrinaryPixel = 205U;
constexpr unsigned char kUnknownTrinaryOccupancyByte = 50U;
constexpr unsigned char kLegacyUnknownScalePixelLow = 127U;
constexpr unsigned char kLegacyUnknownScalePixelHigh = 128U;

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

struct ArtifactSeedProjection
{
  RawNavSatSample anchor_navsat;
  double cos_map_yaw{1.0};
  double sin_map_yaw{0.0};
  double alignment_dx{0.0};
  double alignment_dy{0.0};
};

struct AffineMapTransform
{
  double xx{1.0};
  double xy{0.0};
  double yx{0.0};
  double yy{1.0};
  double tx{0.0};
  double ty{0.0};
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

std::string parseMapPoseStatusState(const std::string & message)
{
  constexpr char key[] = "state=";
  const auto state_position = message.find(key);
  if (state_position == std::string::npos) {
    return trim(message);
  }

  const auto value_start = state_position + (sizeof(key) - 1U);
  const auto value_end = message.find(';', value_start);
  return trim(message.substr(value_start, value_end == std::string::npos ? std::string::npos : value_end - value_start));
}

bool mapPoseStatusAllowsScanIntegration(const std::string & state)
{
  return
    state == "BOOTSTRAP" ||
    state == "DEGRADED" ||
    state == "HEALTHY" ||
    state == "WARNING";
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

std::optional<nlohmann::json> georeferenceMetadataFromArtifact(
  const LoadedCostmapArtifact & artifact)
{
  if (!artifact.georeference_valid) {
    return std::nullopt;
  }

  nlohmann::json georeference{
    {"type", artifact.georeference_type.empty() ? std::string("affine_xy_to_wgs84") : artifact.georeference_type},
    {"source_crs", artifact.georeference_source_crs.empty() ? std::string("EPSG:4326") : artifact.georeference_source_crs},
    {"sample_count", artifact.georeference_sample_count},
    {"longitude_coefficients", {
       artifact.longitude_coefficients[0],
       artifact.longitude_coefficients[1],
       artifact.longitude_coefficients[2]}},
    {"latitude_coefficients", {
       artifact.latitude_coefficients[0],
       artifact.latitude_coefficients[1],
       artifact.latitude_coefficients[2]}}};
  if (!artifact.georeference_companion_file.empty()) {
    georeference["companion_file"] = artifact.georeference_companion_file;
  }
  return georeference;
}

bool artifactUsesScaleEncoding(const LoadedCostmapArtifact & artifact)
{
  return artifact.mode == "scale" || artifact.mode == "raw";
}

bool rawCostRepresentsLegacyUnknown(
  const LoadedCostmapArtifact & artifact,
  const unsigned char raw_cost)
{
  if (!artifactUsesScaleEncoding(artifact)) {
    return raw_cost == kUnknownTrinaryOccupancyByte;
  }

  return
    artifact.occupied_thresh >= 0.99 &&
    artifact.free_thresh <= 0.01 &&
    (raw_cost == kLegacyUnknownScalePixelLow || raw_cost == kLegacyUnknownScalePixelHigh);
}

unsigned char unknownFillValueForArtifact(const LoadedCostmapArtifact & artifact)
{
  if (artifactUsesScaleEncoding(artifact)) {
    return kLegacyUnknownScalePixelHigh;
  }
  return kUnknownTrinaryOccupancyByte;
}

std::optional<unsigned char> artifactCellRawCost(
  const LoadedCostmapArtifact & artifact,
  const double world_x,
  const double world_y)
{
  if (
    artifact.width_cells == 0U || artifact.height_cells == 0U ||
    artifact.resolution <= 0.0 || artifact.costs.empty())
  {
    return std::nullopt;
  }

  const int cell_x = static_cast<int>(std::floor((world_x - artifact.origin_x) / artifact.resolution));
  const int cell_y = static_cast<int>(std::floor((world_y - artifact.origin_y) / artifact.resolution));
  if (
    cell_x < 0 || cell_y < 0 ||
    cell_x >= static_cast<int>(artifact.width_cells) ||
    cell_y >= static_cast<int>(artifact.height_cells))
  {
    return std::nullopt;
  }

  const std::size_t index =
    static_cast<std::size_t>(cell_y) * artifact.width_cells + static_cast<std::size_t>(cell_x);
  return artifact.costs.at(index);
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

struct MissionRuntimeProfile
{
  std::string mission_type;
  std::string execution_mode{kNavigateThroughPosesExecutionMode};
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

std::filesystem::path resolveExecutionContextPath(
  const std::filesystem::path & mission_output_directory)
{
  if (mission_output_directory.empty()) {
    return {};
  }

  const std::filesystem::path legacy_path = mission_output_directory / "execution_context.json";
  if (std::filesystem::exists(legacy_path)) {
    return legacy_path;
  }

  std::error_code error;
  std::vector<std::filesystem::path> candidates;
  for (std::filesystem::directory_iterator iterator(
         mission_output_directory,
         std::filesystem::directory_options::skip_permission_denied,
         error);
       iterator != std::filesystem::directory_iterator();
       iterator.increment(error))
  {
    if (error) {
      error.clear();
      continue;
    }
    if (!iterator->is_regular_file(error)) {
      error.clear();
      continue;
    }
    const std::string filename = iterator->path().filename().string();
    if (filename.size() > std::string("_context.json").size() &&
      filename.compare(filename.size() - 13U, 13U, "_context.json") == 0)
    {
      candidates.push_back(iterator->path());
    }
  }

  if (candidates.empty()) {
    return {};
  }
  std::sort(candidates.begin(), candidates.end());
  return candidates.front();
}

nlohmann::json loadJsonDocument(const std::filesystem::path & path)
{
  std::ifstream input_stream(path);
  if (!input_stream.is_open()) {
    throw std::runtime_error("Failed to open JSON file: " + path.string());
  }
  nlohmann::json document;
  input_stream >> document;
  return document;
}

double yawFromQuaternionMessage(const geometry_msgs::msg::Quaternion & orientation)
{
  tf2::Quaternion quaternion;
  tf2::fromMsg(orientation, quaternion);
  quaternion.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Point mapPointFromArtifactGeoreference(
  const LoadedCostmapArtifact & artifact,
  const RawNavSatSample & navsat_sample)
{
  const double a = artifact.longitude_coefficients[0];
  const double b = artifact.longitude_coefficients[1];
  const double c = artifact.longitude_coefficients[2];
  const double d = artifact.latitude_coefficients[0];
  const double e = artifact.latitude_coefficients[1];
  const double f = artifact.latitude_coefficients[2];
  const double determinant = (a * e) - (b * d);
  if (std::abs(determinant) < 1.0e-12) {
    throw std::runtime_error("Artifact georeference coefficients are singular");
  }

  const double longitude_delta = navsat_sample.longitude - c;
  const double latitude_delta = navsat_sample.latitude - f;
  geometry_msgs::msg::Point map_point;
  map_point.x = ((e * longitude_delta) - (b * latitude_delta)) / determinant;
  map_point.y = ((-d * longitude_delta) + (a * latitude_delta)) / determinant;
  map_point.z = navsat_sample.altitude;
  return map_point;
}

double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double navSatDistanceMeters(const RawNavSatSample & lhs, const RawNavSatSample & rhs)
{
  const double base_latitude_rad = 0.5 * (lhs.latitude + rhs.latitude) * kDegreesToRadians;
  const double east_m =
    (lhs.longitude - rhs.longitude) * kDegreesToRadians * kEarthRadiusMeters *
    std::cos(base_latitude_rad);
  const double north_m =
    (lhs.latitude - rhs.latitude) * kDegreesToRadians * kEarthRadiusMeters;
  return std::hypot(east_m, north_m);
}

ArtifactSeedProjection buildArtifactSeedProjection(
  const LoadedCostmapArtifact & artifact,
  const RawNavSatSample & anchor_navsat,
  const double anchor_heading_yaw)
{
  ArtifactSeedProjection projection;
  projection.anchor_navsat = anchor_navsat;
  projection.cos_map_yaw = std::cos(anchor_heading_yaw);
  projection.sin_map_yaw = std::sin(anchor_heading_yaw);

  const geometry_msgs::msg::Point robot_artifact_point =
    mapPointFromArtifactGeoreference(artifact, anchor_navsat);

  const auto transform_point = [&](const double artifact_x, const double artifact_y) {
      const double longitude =
        artifact.longitude_coefficients[0] * artifact_x +
        artifact.longitude_coefficients[1] * artifact_y +
        artifact.longitude_coefficients[2];
      const double latitude =
        artifact.latitude_coefficients[0] * artifact_x +
        artifact.latitude_coefficients[1] * artifact_y +
        artifact.latitude_coefficients[2];
      const double base_latitude_rad = anchor_navsat.latitude * kDegreesToRadians;
      const double east_m =
        (longitude - anchor_navsat.longitude) * kDegreesToRadians * kEarthRadiusMeters *
        std::cos(base_latitude_rad);
      const double north_m =
        (latitude - anchor_navsat.latitude) * kDegreesToRadians * kEarthRadiusMeters;
      geometry_msgs::msg::Point point;
      point.x = (projection.cos_map_yaw * east_m) + (projection.sin_map_yaw * north_m);
      point.y = (-projection.sin_map_yaw * east_m) + (projection.cos_map_yaw * north_m);
      point.z = 0.0;
      return point;
    };

  const geometry_msgs::msg::Point transformed_robot_point =
    transform_point(robot_artifact_point.x, robot_artifact_point.y);
  projection.alignment_dx = -transformed_robot_point.x;
  projection.alignment_dy = -transformed_robot_point.y;
  return projection;
}

geometry_msgs::msg::Point projectArtifactPointIntoCurrentMap(
  const LoadedCostmapArtifact & artifact,
  const ArtifactSeedProjection & projection,
  const double artifact_x,
  const double artifact_y)
{
  const double longitude =
    artifact.longitude_coefficients[0] * artifact_x +
    artifact.longitude_coefficients[1] * artifact_y +
    artifact.longitude_coefficients[2];
  const double latitude =
    artifact.latitude_coefficients[0] * artifact_x +
    artifact.latitude_coefficients[1] * artifact_y +
    artifact.latitude_coefficients[2];
  const double base_latitude_rad = projection.anchor_navsat.latitude * kDegreesToRadians;
  const double east_m =
    (longitude - projection.anchor_navsat.longitude) * kDegreesToRadians *
    kEarthRadiusMeters * std::cos(base_latitude_rad);
  const double north_m =
    (latitude - projection.anchor_navsat.latitude) * kDegreesToRadians * kEarthRadiusMeters;
  geometry_msgs::msg::Point point;
  point.x =
    (projection.cos_map_yaw * east_m) + (projection.sin_map_yaw * north_m) +
    projection.alignment_dx;
  point.y =
    (-projection.sin_map_yaw * east_m) + (projection.cos_map_yaw * north_m) +
    projection.alignment_dy;
  point.z = 0.0;
  return point;
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
  std::string mode = "trinary";
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
    } else if (key == "mode") {
      mode = stripQuotes(value);
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
  artifact.mode = mode;
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

std::optional<int8_t> artifactCellOccupancy(
  const LoadedCostmapArtifact & artifact,
  const double world_x,
  const double world_y)
{
  const auto raw_cost = artifactCellRawCost(artifact, world_x, world_y);
  if (!raw_cost.has_value() || rawCostRepresentsLegacyUnknown(artifact, *raw_cost)) {
    return std::nullopt;
  }

  const double occupied_ratio = static_cast<double>(*raw_cost) / 255.0;
  if (artifactUsesScaleEncoding(artifact)) {
    return static_cast<int8_t>(
      std::lround(std::clamp(occupied_ratio, 0.0, 1.0) * 100.0));
  }
  if (occupied_ratio >= artifact.occupied_thresh) {
    return static_cast<int8_t>(100);
  }
  if (occupied_ratio <= artifact.free_thresh) {
    return static_cast<int8_t>(0);
  }
  return std::nullopt;
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
      unsigned char pixel = kUnknownTrinaryPixel;
      if (cell >= 0) {
        if (cell >= static_cast<int8_t>(std::lround(kTrinaryOccupiedThreshold * 100.0))) {
          pixel = 0U;
        } else if (cell <= static_cast<int8_t>(std::floor(kTrinaryFreeThreshold * 100.0))) {
          pixel = 255U;
        }
      }
      image_stream.write(reinterpret_cast<const char *>(&pixel), 1);
    }
  }

  std::ofstream yaml_stream(yaml_path, std::ios::trunc);
  if (!yaml_stream.is_open()) {
    throw std::runtime_error("Failed to write costmap yaml artifact: " + yaml_path.string());
  }
  yaml_stream << std::setprecision(std::numeric_limits<double>::max_digits10);
  yaml_stream
    << "image: " << image_path.filename().string() << "\n"
    << "mode: trinary\n"
    << "resolution: " << map.info.resolution << "\n"
    << "origin: [" << map.info.origin.position.x << ", " << map.info.origin.position.y << ", 0.0]\n"
    << "negate: 0\n"
    << "occupied_thresh: " << kTrinaryOccupiedThreshold << "\n"
    << "free_thresh: " << kTrinaryFreeThreshold << "\n";
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

MappingNode::MappingNode()
: Node("mapping_node")
{
  declare_parameter("mission_file", std::string(""));
  declare_parameter("mission_type", std::string(""));
  declare_parameter("execution_mode", std::string(""));
  declare_parameter("mission_route_file", std::string(kDefaultMissionRoutePath));
  declare_parameter("mission_id", std::string(""));
  declare_parameter("mission_output_directory", std::string(""));
  declare_parameter("actual_path_output_file", std::string(""));
  declare_parameter("actual_path_navsat_output_file", std::string(""));
  declare_parameter("startup_saved_costmap_yaml", std::string(""));
  declare_parameter("saved_costmap_yaml", std::string(""));
  declare_parameter("persistent_mission_costmap_yaml", std::string(""));
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
  declare_parameter("heading_topic", std::string("imu/data_heading"));
  declare_parameter("scan_topic", std::string(kDefaultScanTopic));
  declare_parameter("seeded_map_frame", std::string("map"));
  declare_parameter("fromll_service", std::string("/fromLL"));
  declare_parameter("auto_start_mission", true);
  declare_parameter("repeat_mission", false);
  declare_parameter("publish_seeded_map_to_odom", false);
  declare_parameter("map_alignment_publish_period_seconds", 0.5);
  declare_parameter("global_map_resolution_m", 0.05);
  declare_parameter("runtime_costmap_save_period_seconds", 10.0);
  declare_parameter("static_obstacle_min_observations", 6);
  declare_parameter("static_obstacle_min_occupied_fraction", 0.75);
  declare_parameter("static_obstacle_min_free_fraction", 0.75);
  declare_parameter("georef_lock_window_seconds", 3.0);
  declare_parameter("georef_lock_min_samples", 10);
  declare_parameter("georef_lock_max_navsat_spread_m", 2.0);
  declare_parameter("georef_lock_max_heading_deviation_deg", 12.0);
  declare_parameter("max_segments_per_goal", 1);
  declare_parameter("max_waypoint_spacing_m", 0.5);
  declare_parameter("advance_past_blocked_waypoints", true);
  declare_parameter("max_blocked_waypoint_advances", 20);
  declare_parameter("pad_live_map_to_minimum_size", true);
  declare_parameter("min_global_map_size_m", 10.0);
  declare_parameter("status_period_seconds", 2.0);
  declare_parameter("mission_tick_period_seconds", 1.0);
  declare_parameter("navigate_through_poses_action", std::string("navigate_through_poses"));
  declare_parameter("navigator_state_service", std::string("bt_navigator/get_state"));
  declare_parameter("nav2_local_costmap_topic", std::string("local_costmap/costmap_raw"));
  declare_parameter(
    "nav2_local_costmap_updates_topic",
    std::string("local_costmap/costmap_updates"));
  declare_parameter("nav2_global_costmap_topic", std::string("global_costmap/costmap_raw"));
  declare_parameter(
    "nav2_global_costmap_updates_topic",
    std::string("global_costmap/costmap_updates"));
  declare_parameter("map_pose_status_topic", std::string("mapping/map_pose_status"));
  declare_parameter("nav2_costmap_ready_timeout_seconds", 2.5);
  declare_parameter("startup_tf_lookup_timeout_seconds", 0.05);
  declare_parameter("startup_tf_ready_streak_required", 3);
  declare_parameter("bootstrap_empty_global_costmap", false);
  declare_parameter("end_mission_service", std::string("end_mission"));

  mission_file_ = get_parameter("mission_file").as_string();
  const MissionRuntimeProfile mission_profile = loadMissionRuntimeProfile(resolveRuntimePath(mission_file_));
  const std::string configured_mission_type = get_parameter("mission_type").as_string();
  const std::string configured_execution_mode = get_parameter("execution_mode").as_string();
  mission_type_ = configured_mission_type.empty() ? mission_profile.mission_type : configured_mission_type;
  execution_mode_ = configured_execution_mode.empty() ?
    mission_profile.execution_mode : to_lower(configured_execution_mode);
  mission_route_file_ = get_parameter("mission_route_file").as_string();
  mission_id_ = get_parameter("mission_id").as_string();
  mission_output_directory_ = get_parameter("mission_output_directory").as_string();
  actual_path_output_file_ = get_parameter("actual_path_output_file").as_string();
  actual_path_navsat_output_file_ = get_parameter("actual_path_navsat_output_file").as_string();
  startup_saved_costmap_yaml_ = get_parameter("startup_saved_costmap_yaml").as_string();
  saved_costmap_yaml_ = get_parameter("saved_costmap_yaml").as_string();
  persistent_mission_costmap_yaml_ = get_parameter("persistent_mission_costmap_yaml").as_string();
  mission_costmap_yaml_ = get_parameter("mission_costmap_yaml").as_string();
  mission_window_start_ = get_parameter("mission_window_start").as_string();
  mission_window_end_ = get_parameter("mission_window_end").as_string();
  slam_backend_ = get_parameter("slam_backend").as_string();
  gaussian_mode_ = get_parameter("gaussian_mode").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  map_frame_id_ = get_parameter("map_frame").as_string();
  odom_frame_id_ = get_parameter("odom_frame").as_string();
  mission_route_frame_id_ = frame_id_;
  base_frame_ = get_parameter("base_frame").as_string();
  navsat_topic_ = get_parameter("navsat_topic").as_string();
  heading_topic_ = get_parameter("heading_topic").as_string();
  scan_topic_ = get_parameter("scan_topic").as_string();
  seeded_map_frame_id_ = get_parameter("seeded_map_frame").as_string();
  fromll_service_name_ = get_parameter("fromll_service").as_string();
  end_mission_service_name_ = get_parameter("end_mission_service").as_string();
  navigator_state_service_name_ =
    get_parameter("navigator_state_service").as_string();
  nav2_local_costmap_topic_ = get_parameter("nav2_local_costmap_topic").as_string();
  nav2_local_costmap_updates_topic_ =
    get_parameter("nav2_local_costmap_updates_topic").as_string();
  nav2_global_costmap_topic_ = get_parameter("nav2_global_costmap_topic").as_string();
  nav2_global_costmap_updates_topic_ =
    get_parameter("nav2_global_costmap_updates_topic").as_string();
  map_pose_status_topic_ = get_parameter("map_pose_status_topic").as_string();
  auto_start_mission_ = get_parameter("auto_start_mission").as_bool();
  repeat_mission_ = get_parameter("repeat_mission").as_bool();
  bootstrap_empty_global_costmap_ = get_parameter("bootstrap_empty_global_costmap").as_bool();
  publish_seeded_map_to_odom_ = get_parameter("publish_seeded_map_to_odom").as_bool();
  global_map_resolution_m_ = get_parameter("global_map_resolution_m").as_double();
  runtime_costmap_save_period_seconds_ = get_parameter("runtime_costmap_save_period_seconds").as_double();
  startup_tf_lookup_timeout_seconds_ =
    std::max(0.01, get_parameter("startup_tf_lookup_timeout_seconds").as_double());
  startup_tf_ready_streak_required_ =
    std::max(1, static_cast<int>(get_parameter("startup_tf_ready_streak_required").as_int()));
  static_obstacle_min_observations_ =
    static_cast<int>(get_parameter("static_obstacle_min_observations").as_int());
  static_obstacle_min_occupied_fraction_ =
    get_parameter("static_obstacle_min_occupied_fraction").as_double();
  static_obstacle_min_free_fraction_ =
    get_parameter("static_obstacle_min_free_fraction").as_double();
  georef_lock_window_seconds_ = std::max(0.1, get_parameter("georef_lock_window_seconds").as_double());
  georef_lock_max_navsat_spread_m_ = std::max(0.0, get_parameter("georef_lock_max_navsat_spread_m").as_double());
  georef_lock_max_heading_deviation_deg_ = std::max(0.0, get_parameter("georef_lock_max_heading_deviation_deg").as_double());
  georef_lock_min_samples_ = std::max(1, static_cast<int>(get_parameter("georef_lock_min_samples").as_int()));
  max_segments_per_goal_ = static_cast<int>(get_parameter("max_segments_per_goal").as_int());
  max_waypoint_spacing_m_ = get_parameter("max_waypoint_spacing_m").as_double();
  advance_past_blocked_waypoints_ = get_parameter("advance_past_blocked_waypoints").as_bool();
  max_blocked_waypoint_advances_ =
    std::max(0, static_cast<int>(get_parameter("max_blocked_waypoint_advances").as_int()));
  pad_live_map_to_minimum_size_ = get_parameter("pad_live_map_to_minimum_size").as_bool();
  min_global_map_size_m_ = get_parameter("min_global_map_size_m").as_double();
  nav2_costmap_ready_timeout_seconds_ = std::max(
    0.1,
    get_parameter("nav2_costmap_ready_timeout_seconds").as_double());
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
  heading_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
    heading_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&MappingNode::handleHeading, this, std::placeholders::_1));
  map_pose_status_subscription_ = create_subscription<std_msgs::msg::String>(
    map_pose_status_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&MappingNode::handleMapPoseStatus, this, std::placeholders::_1));
  nav2_local_costmap_subscription_ = create_subscription<nav2_msgs::msg::Costmap>(
    nav2_local_costmap_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MappingNode::handleNav2LocalCostmap, this, std::placeholders::_1));
  nav2_local_costmap_updates_subscription_ =
    create_subscription<map_msgs::msg::OccupancyGridUpdate>(
    nav2_local_costmap_updates_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MappingNode::handleNav2LocalCostmapUpdate, this, std::placeholders::_1));
  nav2_global_costmap_subscription_ = create_subscription<nav2_msgs::msg::Costmap>(
    nav2_global_costmap_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MappingNode::handleNav2GlobalCostmap, this, std::placeholders::_1));
  nav2_global_costmap_updates_subscription_ =
    create_subscription<map_msgs::msg::OccupancyGridUpdate>(
    nav2_global_costmap_updates_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MappingNode::handleNav2GlobalCostmapUpdate, this, std::placeholders::_1));
  status_publisher_ = create_publisher<std_msgs::msg::String>("mapping/status", 10);
  global_costmap_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "mapping/static_costmap",
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
  navigator_state_client_ =
    create_client<lifecycle_msgs::srv::GetState>(
    navigator_state_service_name_,
    rclcpp::ServicesQoS(),
    mission_callback_group_);
  navigate_through_poses_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
    this,
    get_parameter("navigate_through_poses_action").as_string(),
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
  latest_scan_ = *message;
  last_scan_frame_id_ = message->header.frame_id;
  last_scan_time_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() :
    rclcpp::Time(message->header.stamp);
  have_scan_ = true;
  if (!mapPoseStatusAllowsScanIntegration(map_pose_status_state_)) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Delaying scan integration until map_pose_node reports a non-fault status on %s. current_state=%s",
      map_pose_status_topic_.c_str(),
      map_pose_status_state_.c_str());
    return;
  }
  if (!updateStartupTfReadiness(message->header.frame_id)) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Waiting for local TF buffer warmup before scan integration. frame=%s streak=%d/%d detail=%s",
      message->header.frame_id.c_str(),
      startup_tf_ready_streak_,
      startup_tf_ready_streak_required_,
      startup_tf_status_detail_.c_str());
    return;
  }
  if (latest_odometry_pose_ready_) {
    integrateScanIntoGlobalMap(*message);
  }
}

void MappingNode::handleNav2LocalCostmap(const nav2_msgs::msg::Costmap::SharedPtr message)
{
  latest_nav2_local_costmap_stamp_ = now();
  if (!nav2_local_costmap_ready_) {
    nav2_local_costmap_ready_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Nav2 local costmap is now publishing on %s with frame=%s size=%ux%u res=%.3f.",
      nav2_local_costmap_topic_.c_str(),
      message->header.frame_id.c_str(),
      message->metadata.size_x,
      message->metadata.size_y,
      static_cast<double>(message->metadata.resolution));
  }
}

void MappingNode::handleNav2LocalCostmapUpdate(
  const map_msgs::msg::OccupancyGridUpdate::SharedPtr message)
{
  latest_nav2_local_costmap_stamp_ = now();
  if (!nav2_local_costmap_ready_) {
    nav2_local_costmap_ready_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Nav2 local costmap updates are now publishing on %s with size=%ux%u.",
      nav2_local_costmap_updates_topic_.c_str(),
      message->width,
      message->height);
  }
}

void MappingNode::handleNav2GlobalCostmap(const nav2_msgs::msg::Costmap::SharedPtr message)
{
  latest_nav2_global_costmap_stamp_ = now();
  if (!nav2_global_costmap_ready_) {
    nav2_global_costmap_ready_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Nav2 global costmap is now publishing on %s with frame=%s size=%ux%u res=%.3f origin=(%.3f, %.3f).",
      nav2_global_costmap_topic_.c_str(),
      message->header.frame_id.c_str(),
      message->metadata.size_x,
      message->metadata.size_y,
      static_cast<double>(message->metadata.resolution),
      message->metadata.origin.position.x,
      message->metadata.origin.position.y);
  }
}

void MappingNode::handleNav2GlobalCostmapUpdate(
  const map_msgs::msg::OccupancyGridUpdate::SharedPtr message)
{
  latest_nav2_global_costmap_stamp_ = now();
  if (!nav2_global_costmap_ready_) {
    nav2_global_costmap_ready_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Nav2 global costmap updates are now publishing on %s with size=%ux%u.",
      nav2_global_costmap_updates_topic_.c_str(),
      message->width,
      message->height);
  }
}

void MappingNode::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  latest_odometry_position_ = message->pose.pose.position;
  latest_odometry_orientation_ = message->pose.pose.orientation;
  latest_odometry_pose_ready_ = true;
  tryPublishBootstrapGlobalCostmap();

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

  tryInitializeSavedCostmapFromSensors();
}

void MappingNode::tryPublishBootstrapGlobalCostmap()
{
  if (
    !bootstrap_empty_global_costmap_ ||
    bootstrap_global_costmap_published_ ||
    latest_padded_live_map_ready_ ||
    latest_live_map_.info.width > 0U ||
    !latest_odometry_pose_ready_)
  {
    return;
  }

  geometry_msgs::msg::PointStamped odom_point;
  odom_point.header.frame_id = odom_frame_id_;
  odom_point.point = mission_anchor_pose_ready_ ? mission_anchor_position_ : latest_odometry_position_;

  geometry_msgs::msg::PointStamped map_point;
  try {
    const auto map_to_odom = tf_buffer_->lookupTransform(
      map_frame_id_,
      odom_frame_id_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.05));
    tf2::doTransform(odom_point, map_point, map_to_odom);
  } catch (const tf2::TransformException & exception) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Waiting for the first %s -> %s TF before publishing the bootstrap global map: %s",
      map_frame_id_.c_str(),
      odom_frame_id_.c_str(),
      exception.what());
    return;
  }

  initializeGlobalMap(map_point.point);
  publishGlobalMaps();
  bootstrap_global_costmap_published_ = latest_padded_live_map_ready_;

  if (bootstrap_global_costmap_published_) {
    RCLCPP_INFO(
      get_logger(),
      "Published a bootstrap in-memory empty global costmap after the first %s -> %s TF became available, so Nav2 and map_pose_node can initialize without a saved costmap artifact.",
      map_frame_id_.c_str(),
      odom_frame_id_.c_str());
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

  {
    std::lock_guard<std::mutex> lock(synchronized_path_mutex_);
    latest_raw_navsat_sample_ = sample;
    navsat_history_.push_back(sample);
    pruneGeoreferenceSamples();
  }
  tryInitializeSavedCostmapFromSensors();
}

void MappingNode::handleHeading(const sensor_msgs::msg::Imu::SharedPtr message)
{
  if (!message) {
    return;
  }

  latest_heading_ = *message;
  latest_heading_ready_ = true;
  HeadingSample sample;
  sample.yaw = yawFromQuaternionMessage(message->orientation);
  sample.stamp = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() : rclcpp::Time(message->header.stamp);
  {
    std::lock_guard<std::mutex> lock(synchronized_path_mutex_);
    heading_history_.push_back(sample);
    pruneGeoreferenceSamples();
  }
  tryInitializeSavedCostmapFromSensors();
}

void MappingNode::handleMapPoseStatus(const std_msgs::msg::String::SharedPtr message)
{
  if (!message) {
    return;
  }

  const std::string previous_state = map_pose_status_state_;
  latest_map_pose_status_message_ = message->data;
  map_pose_status_state_ = parseMapPoseStatusState(message->data);
  if (map_pose_status_state_ != previous_state) {
    RCLCPP_INFO(
      get_logger(),
      "map_pose_node status transition %s -> %s.",
      previous_state.c_str(),
      map_pose_status_state_.c_str());
  }
  if (map_pose_status_state_ == "HEALTHY" && previous_state != "HEALTHY") {
    RCLCPP_INFO(
      get_logger(),
      "map_pose_node is HEALTHY; runtime scan integration is fully enabled.");
  } else if (map_pose_status_state_ == "FAULT" && previous_state != "FAULT") {
    RCLCPP_WARN(
      get_logger(),
      "map_pose_node entered FAULT; runtime scan integration will pause until the status recovers.");
  }
}

bool MappingNode::updateStartupTfReadiness(const std::string & scan_frame_id)
{
  if (startup_tf_ready_) {
    return true;
  }

  if (scan_frame_id.empty()) {
    startup_tf_ready_streak_ = 0;
    startup_tf_status_detail_ = "missing_scan_frame";
    return false;
  }

  std::string missing_reason;
  const bool have_transform = tf_buffer_->canTransform(
    map_frame_id_,
    scan_frame_id,
    tf2::TimePointZero,
    tf2::durationFromSec(startup_tf_lookup_timeout_seconds_),
    &missing_reason);

  if (!have_transform) {
    startup_tf_ready_streak_ = 0;
    startup_tf_status_detail_ = missing_reason.empty() ? "map_to_scan_unavailable" : missing_reason;
    return false;
  }

  startup_tf_ready_streak_ = std::min(
    startup_tf_ready_streak_ + 1,
    startup_tf_ready_streak_required_);
  startup_tf_status_detail_ = "map_to_scan_available";
  if (startup_tf_ready_streak_ >= startup_tf_ready_streak_required_) {
    startup_tf_ready_ = true;
    startup_tf_status_detail_ = "ready";
    RCLCPP_INFO(
      get_logger(),
      "Startup TF warmup complete for %s -> %s after %d consecutive checks; scan integration is now enabled.",
      scan_frame_id.c_str(),
      map_frame_id_.c_str(),
      startup_tf_ready_streak_required_);
  }
  return startup_tf_ready_;
}

void MappingNode::pruneGeoreferenceSamples()
{
  const rclcpp::Time cutoff = now() - rclcpp::Duration::from_seconds(georef_lock_window_seconds_);
  auto navsat_keep_from = std::find_if(
    navsat_history_.begin(),
    navsat_history_.end(),
    [&cutoff](const RawNavSatSample & sample) {return sample.stamp >= cutoff;});
  navsat_history_.erase(navsat_history_.begin(), navsat_keep_from);

  auto heading_keep_from = std::find_if(
    heading_history_.begin(),
    heading_history_.end(),
    [&cutoff](const HeadingSample & sample) {return sample.stamp >= cutoff;});
  heading_history_.erase(heading_history_.begin(), heading_keep_from);
}

std::optional<RawNavSatSample> MappingNode::stabilizedNavSatSample() const
{
  std::lock_guard<std::mutex> lock(synchronized_path_mutex_);
  if (static_cast<int>(navsat_history_.size()) < georef_lock_min_samples_) {
    return std::nullopt;
  }

  RawNavSatSample stabilized;
  for (const auto & sample : navsat_history_) {
    stabilized.longitude += sample.longitude;
    stabilized.latitude += sample.latitude;
    stabilized.altitude += sample.altitude;
    stabilized.stamp = std::max(stabilized.stamp, sample.stamp);
  }

  const double sample_count = static_cast<double>(navsat_history_.size());
  stabilized.longitude /= sample_count;
  stabilized.latitude /= sample_count;
  stabilized.altitude /= sample_count;
  return stabilized;
}

std::optional<double> MappingNode::stabilizedHeadingYaw() const
{
  std::lock_guard<std::mutex> lock(synchronized_path_mutex_);
  if (static_cast<int>(heading_history_.size()) < georef_lock_min_samples_) {
    return std::nullopt;
  }

  double sum_sin = 0.0;
  double sum_cos = 0.0;
  for (const auto & sample : heading_history_) {
    sum_sin += std::sin(sample.yaw);
    sum_cos += std::cos(sample.yaw);
  }

  if (std::abs(sum_sin) < 1.0e-9 && std::abs(sum_cos) < 1.0e-9) {
    return std::nullopt;
  }

  return std::atan2(sum_sin, sum_cos);
}

std::optional<double> MappingNode::navSatHistoryMaxSpreadMeters() const
{
  std::lock_guard<std::mutex> lock(synchronized_path_mutex_);
  if (static_cast<int>(navsat_history_.size()) < georef_lock_min_samples_) {
    return std::nullopt;
  }

  RawNavSatSample centroid;
  for (const auto & sample : navsat_history_) {
    centroid.longitude += sample.longitude;
    centroid.latitude += sample.latitude;
  }
  const double sample_count = static_cast<double>(navsat_history_.size());
  centroid.longitude /= sample_count;
  centroid.latitude /= sample_count;

  double max_distance_m = 0.0;
  for (const auto & sample : navsat_history_) {
    max_distance_m = std::max(max_distance_m, navSatDistanceMeters(sample, centroid));
  }
  return max_distance_m;
}

std::optional<double> MappingNode::headingHistoryMaxDeviationRadians(const double reference_yaw) const
{
  std::lock_guard<std::mutex> lock(synchronized_path_mutex_);
  if (static_cast<int>(heading_history_.size()) < georef_lock_min_samples_) {
    return std::nullopt;
  }

  double max_deviation_rad = 0.0;
  for (const auto & sample : heading_history_) {
    max_deviation_rad = std::max(
      max_deviation_rad,
      std::abs(normalizeAngle(sample.yaw - reference_yaw)));
  }
  return max_deviation_rad;
}

void MappingNode::loadSavedCostmapIfConfigured()
{
  const std::string startup_costmap_yaml = startup_saved_costmap_yaml_.empty() ?
    saved_costmap_yaml_ : startup_saved_costmap_yaml_;
  if (startup_costmap_yaml.empty()) {
    return;
  }

  const std::string resolved_path = resolveRuntimePath(startup_costmap_yaml);
  std::error_code filesystem_error;
  if (!std::filesystem::is_regular_file(resolved_path, filesystem_error)) {
    RCLCPP_WARN(
      get_logger(),
      "Startup saved costmap yaml was configured as '%s' but no file was found there.",
      resolved_path.c_str());
    return;
  }

  try {
    LoadedCostmapArtifact artifact = loadCostmapArtifactFromYaml(resolved_path);
    RCLCPP_INFO(
      get_logger(),
      "Startup costmap candidate %s parsed with georeference_valid=%s type='%s' resolution=%.3f origin=(%.3f, %.3f) size=%ux%u samples=%zu.",
      resolved_path.c_str(),
      artifact.georeference_valid ? "true" : "false",
      artifact.georeference_type.c_str(),
      artifact.resolution,
      artifact.origin_x,
      artifact.origin_y,
      artifact.width_cells,
      artifact.height_cells,
      artifact.georeference_sample_count);
    if (artifact.georeference_valid) {
      authoritative_saved_costmap_artifact_ = artifact;
      pending_saved_costmap_artifact_ = artifact;
      locked_georef_navsat_sample_.reset();
      locked_georef_heading_yaw_.reset();
      tryInitializeSavedCostmapFromSensors();
      if (pending_saved_costmap_artifact_.has_value()) {
        RCLCPP_INFO(
          get_logger(),
          "Loaded georeferenced saved costmap artifact from %s and waiting for a stable startup seeding window before projecting it into %s.",
          resolved_path.c_str(),
          map_frame_id_.c_str());
      }
      return;
    }

    if (!manual_mapping_mode_ && map_frame_id_ != odom_frame_id_) {
      RCLCPP_WARN(
        get_logger(),
        "Skipping startup load of non-georeferenced costmap artifact from %s because this mission navigates in %s and an unprojected local-frame map can place the robot outside the costmap before motion starts.",
        resolved_path.c_str(),
        map_frame_id_.c_str());
      return;
    }

    initializeMapFromArtifact(artifact);
    publishGlobalMaps();
    saved_costmap_initialized_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Loaded saved costmap artifact from %s into %s without georeferenced reprojection.",
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

bool MappingNode::useAuthoritativeMissionGeoreference() const
{
  return mission_type_ == kScheduledMissionType || mission_type_ == kLocalScheduledMissionType;
}

std::optional<LoadedCostmapArtifact> MappingNode::projectArtifactIntoCurrentMap(
  const LoadedCostmapArtifact & artifact,
  const RawNavSatSample & anchor_navsat,
  const double anchor_heading_yaw) const
{
  if (!artifact.georeference_valid) {
    return std::nullopt;
  }

  const ArtifactSeedProjection projection =
    buildArtifactSeedProjection(artifact, anchor_navsat, anchor_heading_yaw);

  const std::array<geometry_msgs::msg::Point, 4> corners{{
    projectArtifactPointIntoCurrentMap(artifact, projection, artifact.origin_x, artifact.origin_y),
    projectArtifactPointIntoCurrentMap(
      artifact,
      projection,
      artifact.origin_x + artifact.resolution * static_cast<double>(artifact.width_cells),
      artifact.origin_y),
    projectArtifactPointIntoCurrentMap(
      artifact,
      projection,
      artifact.origin_x,
      artifact.origin_y + artifact.resolution * static_cast<double>(artifact.height_cells)),
    projectArtifactPointIntoCurrentMap(
      artifact,
      projection,
      artifact.origin_x + artifact.resolution * static_cast<double>(artifact.width_cells),
      artifact.origin_y + artifact.resolution * static_cast<double>(artifact.height_cells))
  }};

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto & corner : corners) {
    min_x = std::min(min_x, corner.x);
    min_y = std::min(min_y, corner.y);
    max_x = std::max(max_x, corner.x);
    max_y = std::max(max_y, corner.y);
  }

  const geometry_msgs::msg::Point projected_origin =
    projectArtifactPointIntoCurrentMap(artifact, projection, artifact.origin_x, artifact.origin_y);
  const geometry_msgs::msg::Point projected_unit_x =
    projectArtifactPointIntoCurrentMap(artifact, projection, artifact.origin_x + 1.0, artifact.origin_y);
  const geometry_msgs::msg::Point projected_unit_y =
    projectArtifactPointIntoCurrentMap(artifact, projection, artifact.origin_x, artifact.origin_y + 1.0);

  const AffineMapTransform transform{
    projected_unit_x.x - projected_origin.x,
    projected_unit_y.x - projected_origin.x,
    projected_unit_x.y - projected_origin.y,
    projected_unit_y.y - projected_origin.y,
    projected_origin.x,
    projected_origin.y};
  const double determinant = transform.xx * transform.yy - transform.xy * transform.yx;
  if (std::abs(determinant) <= 1.0e-9) {
    return std::nullopt;
  }

  LoadedCostmapArtifact projected = artifact;
  projected.origin_x = min_x;
  projected.origin_y = min_y;
  projected.width_cells = static_cast<unsigned int>(
    std::max(1.0, std::ceil((max_x - min_x) / artifact.resolution)));
  projected.height_cells = static_cast<unsigned int>(
    std::max(1.0, std::ceil((max_y - min_y) / artifact.resolution)));
  projected.costs.assign(
    static_cast<std::size_t>(projected.width_cells) * projected.height_cells,
    unknownFillValueForArtifact(artifact));

  for (unsigned int row = 0U; row < projected.height_cells; ++row) {
    for (unsigned int col = 0U; col < projected.width_cells; ++col) {
      const double map_x =
        projected.origin_x + (static_cast<double>(col) + 0.5) * projected.resolution;
      const double map_y =
        projected.origin_y + (static_cast<double>(row) + 0.5) * projected.resolution;
      const double centered_x = map_x - transform.tx;
      const double centered_y = map_y - transform.ty;
      const double artifact_x =
        (transform.yy * centered_x - transform.xy * centered_y) / determinant;
      const double artifact_y =
        (-transform.yx * centered_x + transform.xx * centered_y) / determinant;
      const auto source_cost = artifactCellRawCost(artifact, artifact_x, artifact_y);
      if (!source_cost.has_value()) {
        continue;
      }

      const std::size_t target_index =
        static_cast<std::size_t>(row) * projected.width_cells + static_cast<std::size_t>(col);
      projected.costs[target_index] = *source_cost;
    }
  }

  return projected;
}

void MappingNode::tryInitializeSavedCostmapFromSensors()
{
  if (saved_costmap_initialized_ || !pending_saved_costmap_artifact_.has_value()) {
    return;
  }

  const auto stabilized_navsat = stabilizedNavSatSample();
  const auto stabilized_heading_yaw = stabilizedHeadingYaw();
  if (!stabilized_navsat.has_value() || !stabilized_heading_yaw.has_value()) {
    return;
  }

  const auto navsat_spread_m = navSatHistoryMaxSpreadMeters();
  if (
    georef_lock_max_navsat_spread_m_ > 0.0 &&
    navsat_spread_m.has_value() &&
    *navsat_spread_m > georef_lock_max_navsat_spread_m_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Rejecting startup georeference seed because the GNSS spread over the %.1fs window is %.3fm, which exceeds the configured %.3fm limit.",
      georef_lock_window_seconds_,
      *navsat_spread_m,
      georef_lock_max_navsat_spread_m_);
    return;
  }

  const auto heading_deviation_rad = headingHistoryMaxDeviationRadians(*stabilized_heading_yaw);
  if (
    georef_lock_max_heading_deviation_deg_ > 0.0 &&
    heading_deviation_rad.has_value() &&
    (*heading_deviation_rad * 180.0 / M_PI) > georef_lock_max_heading_deviation_deg_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Rejecting startup georeference seed because the heading deviation over the %.1fs window is %.2f deg, which exceeds the configured %.2f deg limit.",
      georef_lock_window_seconds_,
      *heading_deviation_rad * 180.0 / M_PI,
      georef_lock_max_heading_deviation_deg_);
    return;
  }

  const auto projected_artifact = projectArtifactIntoCurrentMap(
    *pending_saved_costmap_artifact_,
    *stabilized_navsat,
    *stabilized_heading_yaw);
  if (!projected_artifact.has_value()) {
    return;
  }

  locked_georef_navsat_sample_ = *stabilized_navsat;
  locked_georef_heading_yaw_ = *stabilized_heading_yaw;
  initializeMapFromArtifact(*projected_artifact);
  publishGlobalMaps();
  saved_costmap_initialized_ = true;
  georeferenced_costmap_locked_ = true;
  pending_saved_costmap_artifact_.reset();
  RCLCPP_INFO(
    get_logger(),
    "Locked the saved georeferenced costmap into %s after a %.1fs startup seeding window with GNSS spread %.3fm and heading deviation %.2f deg, while keeping the startup %s -> %s transform at identity.",
    map_frame_id_.c_str(),
    georef_lock_window_seconds_,
    navsat_spread_m.value_or(0.0),
    heading_deviation_rad.has_value() ? (*heading_deviation_rad * 180.0 / M_PI) : 0.0,
    map_frame_id_.c_str(),
    odom_frame_id_.c_str());
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

  const bool preserve_scaled_costs = artifactUsesScaleEncoding(artifact);
  for (std::size_t index = 0U; index < artifact.costs.size(); ++index) {
    const unsigned char raw_cost = artifact.costs.at(index);
    if (rawCostRepresentsLegacyUnknown(artifact, raw_cost)) {
      continue;
    }

    const double occupied_ratio = static_cast<double>(raw_cost) / 255.0;
    if (preserve_scaled_costs) {
      latest_live_map_.data.at(index) = static_cast<int8_t>(
        std::lround(std::clamp(occupied_ratio, 0.0, 1.0) * 100.0));
    } else if (occupied_ratio >= artifact.occupied_thresh) {
      latest_live_map_.data.at(index) = 100;
    } else if (occupied_ratio <= artifact.free_thresh) {
      latest_live_map_.data.at(index) = 0;
    }

    if (occupied_ratio >= artifact.occupied_thresh) {
      global_map_scores_.at(index) = 20;
      global_map_observations_.at(index) = 1U;
      global_map_occupied_observations_.at(index) = 1U;
    } else if (occupied_ratio <= artifact.free_thresh) {
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
    latest_live_map_.data.size() != global_map_occupied_observations_.size() ||
    latest_live_map_.data.size() != global_map_free_observations_.size())
  {
    return artifact_map;
  }

  for (std::size_t index = 0U; index < latest_live_map_.data.size(); ++index) {
    const uint16_t total_observations = global_map_observations_.at(index);
    if (total_observations < static_cast<uint16_t>(std::max(1, static_obstacle_min_observations_))) {
      continue;
    }

    const uint16_t occupied_observations = global_map_occupied_observations_.at(index);
    const uint16_t free_observations = global_map_free_observations_.at(index);
    const double occupied_fraction =
      static_cast<double>(occupied_observations) / static_cast<double>(total_observations);
    const double free_fraction =
      static_cast<double>(free_observations) / static_cast<double>(total_observations);

    if (
      occupied_fraction >= static_obstacle_min_occupied_fraction_ &&
      latest_live_map_.data.at(index) >= 65)
    {
      artifact_map.data.at(index) = 100;
      continue;
    }

    if (
      free_fraction >= static_obstacle_min_free_fraction_ &&
      latest_live_map_.data.at(index) >= 0 &&
      latest_live_map_.data.at(index) <= 35)
    {
      artifact_map.data.at(index) = 0;
    }
  }

  return artifact_map;
}

void MappingNode::mergeCompletedRuntimeCostmapIntoMissionCostmap()
{
  if (persistent_mission_costmap_merged_) {
    return;
  }
  if (persistent_mission_costmap_yaml_.empty() || saved_costmap_yaml_.empty()) {
    return;
  }

  const std::filesystem::path persistent_yaml_path(resolveRuntimePath(persistent_mission_costmap_yaml_));
  const std::filesystem::path runtime_yaml_path(resolveRuntimePath(saved_costmap_yaml_));
  if (persistent_yaml_path.empty() || runtime_yaml_path.empty() || persistent_yaml_path == runtime_yaml_path) {
    persistent_mission_costmap_merged_ = true;
    return;
  }

  try {
    const LoadedCostmapArtifact persistent_artifact =
      loadCostmapArtifactFromYaml(persistent_yaml_path.string());
    const LoadedCostmapArtifact runtime_artifact =
      loadCostmapArtifactFromYaml(runtime_yaml_path.string());

    if (persistent_artifact.resolution <= 0.0 || runtime_artifact.resolution <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Skipping persistent mission costmap merge because one of the artifacts has an invalid resolution. persistent=%s runtime=%s",
        persistent_yaml_path.string().c_str(),
        runtime_yaml_path.string().c_str());
      return;
    }

    if (std::abs(persistent_artifact.resolution - runtime_artifact.resolution) > 1.0e-6) {
      RCLCPP_WARN(
        get_logger(),
        "Skipping persistent mission costmap merge because the artifact resolutions differ. persistent=%.6f runtime=%.6f",
        persistent_artifact.resolution,
        runtime_artifact.resolution);
      return;
    }

    const double resolution = persistent_artifact.resolution;
    const double persistent_max_x =
      persistent_artifact.origin_x + resolution * static_cast<double>(persistent_artifact.width_cells);
    const double persistent_max_y =
      persistent_artifact.origin_y + resolution * static_cast<double>(persistent_artifact.height_cells);
    const double runtime_max_x =
      runtime_artifact.origin_x + resolution * static_cast<double>(runtime_artifact.width_cells);
    const double runtime_max_y =
      runtime_artifact.origin_y + resolution * static_cast<double>(runtime_artifact.height_cells);

    const double merged_min_x = std::min(persistent_artifact.origin_x, runtime_artifact.origin_x);
    const double merged_min_y = std::min(persistent_artifact.origin_y, runtime_artifact.origin_y);
    const double merged_max_x = std::max(persistent_max_x, runtime_max_x);
    const double merged_max_y = std::max(persistent_max_y, runtime_max_y);

    nav_msgs::msg::OccupancyGrid merged_map;
    merged_map.header.frame_id = map_frame_id_;
    merged_map.header.stamp = now();
    merged_map.info.map_load_time = merged_map.header.stamp;
    merged_map.info.resolution = static_cast<float>(resolution);
    merged_map.info.width = static_cast<uint32_t>(
      std::max(1.0, std::ceil((merged_max_x - merged_min_x) / resolution)));
    merged_map.info.height = static_cast<uint32_t>(
      std::max(1.0, std::ceil((merged_max_y - merged_min_y) / resolution)));
    merged_map.info.origin.position.x = merged_min_x;
    merged_map.info.origin.position.y = merged_min_y;
    merged_map.info.origin.position.z = 0.0;
    merged_map.info.origin.orientation.w = 1.0;
    merged_map.data.assign(
      static_cast<std::size_t>(merged_map.info.width) * merged_map.info.height,
      static_cast<int8_t>(-1));

    for (uint32_t row = 0U; row < merged_map.info.height; ++row) {
      for (uint32_t col = 0U; col < merged_map.info.width; ++col) {
        const double world_x = merged_min_x + (static_cast<double>(col) + 0.5) * resolution;
        const double world_y = merged_min_y + (static_cast<double>(row) + 0.5) * resolution;
        const auto persistent_cell = artifactCellOccupancy(persistent_artifact, world_x, world_y);
        const auto runtime_cell = artifactCellOccupancy(runtime_artifact, world_x, world_y);

        int8_t merged_cell = -1;
        if (runtime_cell.has_value()) {
          merged_cell = *runtime_cell;
        } else if (persistent_cell.has_value()) {
          merged_cell = *persistent_cell;
        }

        const std::size_t index =
          static_cast<std::size_t>(row) * merged_map.info.width + static_cast<std::size_t>(col);
        merged_map.data.at(index) = merged_cell;
      }
    }

    auto georeference_metadata = georeferenceMetadataFromArtifact(persistent_artifact);
    if (!georeference_metadata.has_value()) {
      georeference_metadata = georeferenceMetadataFromArtifact(runtime_artifact);
    }
    saveOccupancyGridArtifact(merged_map, persistent_yaml_path, georeference_metadata);
    persistent_mission_costmap_merged_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Merged completed runtime costmap %s back into persistent mission costmap %s while preferring runtime cell occupancies in overlapping regions.",
      runtime_yaml_path.string().c_str(),
      persistent_yaml_path.string().c_str());
  } catch (const std::exception & exception) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to merge completed runtime costmap into persistent mission costmap: %s",
      exception.what());
  }
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
    std::vector<SynchronizedPathSample> synchronized_samples;
    {
      std::lock_guard<std::mutex> lock(synchronized_path_mutex_);
      synchronized_samples = synchronized_path_samples_;
    }
    if (!manual_mapping_mode_ && !synchronized_samples.empty()) {
      std::vector<MapPoint> local_trace;
      std::vector<GeoPoint> geo_trace;
      local_trace.reserve(synchronized_samples.size());
      geo_trace.reserve(synchronized_samples.size());
      for (const auto & sample : synchronized_samples) {
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
    << "; map_pose_status=" << map_pose_status_state_
    << "; startup_tf_ready=" << (startup_tf_ready_ ? "true" : "false")
    << "; startup_tf_streak=" << startup_tf_ready_streak_ << "/" << startup_tf_ready_streak_required_
    << "; tracking=" << (have_global_map ? "true" : "false")
    << "; scan_tf=" << (have_scan_to_base_tf ? "true" : "false")
    << "; base_frame=" << (have_base_frame ? "true" : "false")
    << "; scan_frame_exists=" << (have_scan_frame ? "true" : "false")
    << "; map_tf=" << (have_map_to_odom_tf ? "true" : "false")
    << "; global_map=" << (have_global_map ? "true" : "false")
    << "; scan_frame=" << (last_scan_frame_id_.empty() ? "<none>" : last_scan_frame_id_)
    << "; pose_frame=" << (latest_odometry_pose_ready_ ? odom_frame_id_ : "<none>");
  if (!latest_map_pose_status_message_.empty()) {
    stream << "; map_pose_status_detail=" << latest_map_pose_status_message_;
  }
  if (!startup_tf_status_detail_.empty()) {
    stream << "; startup_tf_detail=" << startup_tf_status_detail_;
  }
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

  const auto mission_output_directory = std::filesystem::path(mission_output_directory_);
  const auto context_path = resolveExecutionContextPath(mission_output_directory);
  if (context_path.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to locate execution context for mapping metadata in %s",
      mission_output_directory_.c_str());
    return;
  }

  try {
    auto context_document = loadJsonDocument(context_path);
    context_document["mapping"] = document;
    writeJsonDocumentAtomic(context_path, context_document);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to write mapping metadata into %s",
      context_path.string().c_str());
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
    map_from_scan = tf_buffer_->lookupTransform(
      map_frame_id_,
      message.header.frame_id,
      tf2::TimePointZero,
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
    consecutive_blocked_waypoint_advances_ = 0;
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
  const bool should_use_seeded_georeference =
    uses_geographic_frame &&
    useAuthoritativeMissionGeoreference() &&
    authoritative_saved_costmap_artifact_.has_value() &&
    authoritative_saved_costmap_artifact_->georeference_valid;
  const bool can_use_authoritative_georeference =
    should_use_seeded_georeference &&
    georeferenced_costmap_locked_ &&
    locked_georef_navsat_sample_.has_value() &&
    locked_georef_heading_yaw_.has_value();
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

  if (should_use_seeded_georeference && !can_use_authoritative_georeference) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Waiting for the startup georeference seed to lock before projecting the scheduled mission route into %s.",
      map_frame_id_.c_str());
    return;
  }

  if (uses_geographic_frame && !should_use_seeded_georeference &&
    !fromll_client_->wait_for_service(std::chrono::seconds(1)))
  {
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
  mission_route_frame_id_ = frame_id_;
  if (can_use_authoritative_georeference) {
    mission_route_frame_id_ = map_frame_id_;
  }

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

  ArtifactSeedProjection seeded_projection;
  if (can_use_authoritative_georeference) {
    seeded_projection = buildArtifactSeedProjection(
      *authoritative_saved_costmap_artifact_,
      *locked_georef_navsat_sample_,
      *locked_georef_heading_yaw_);
  }

  for (const MissionCoordinate & coordinate : coordinates) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = mission_route_frame_id_;
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
    if (can_use_authoritative_georeference) {
      RawNavSatSample waypoint_navsat;
      waypoint_navsat.longitude = coordinate.x;
      waypoint_navsat.latitude = coordinate.y;
      waypoint_navsat.altitude = 0.0;
      const geometry_msgs::msg::Point artifact_point = mapPointFromArtifactGeoreference(
        *authoritative_saved_costmap_artifact_,
        waypoint_navsat);
      const geometry_msgs::msg::Point map_point = projectArtifactPointIntoCurrentMap(
        *authoritative_saved_costmap_artifact_,
        seeded_projection,
        artifact_point.x,
        artifact_point.y);
      pose.pose.position.x = map_point.x;
      pose.pose.position.y = map_point.y;
    } else {
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
    }
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
  mission_chunk_start_indices_ = chunkRouteStartIndices(mission_route_);
  active_chunk_index_ = 0U;
  consecutive_blocked_waypoint_advances_ = 0;
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
    mission_route_frame_id_.c_str(),
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

  if (!navigate_through_poses_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for Nav2 navigate_through_poses action server.");
    return;
  }

  if (!isNavigatorActive()) {
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

  if (!areMissionCostmapsReadyForMissionStart()) {
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

  nav2_msgs::action::NavigateThroughPoses::Goal goal;
  const rclcpp::Time latest_transform_stamp(0, 0, get_clock()->get_clock_type());
  try {
    if (odom_only_navigation) {
      for (auto pose : mission_chunks_.at(active_chunk_index_)) {
        pose.header.stamp = latest_transform_stamp;
        pose.header.frame_id = odom_frame_id_;
        goal.poses.push_back(pose);
      }
    } else {
      for (auto pose : mission_chunks_.at(active_chunk_index_)) {
        pose.header.stamp = latest_transform_stamp;
        const std::string source_frame =
          pose.header.frame_id.empty() ? mission_route_frame_id_ : pose.header.frame_id;
        if (source_frame == map_frame_id_) {
          pose.header.frame_id = map_frame_id_;
          pose.header.stamp = latest_transform_stamp;
          goal.poses.push_back(pose);
          continue;
        }

        const auto map_from_source = tf_buffer_->lookupTransform(
          map_frame_id_,
          source_frame,
          tf2::TimePointZero);
        geometry_msgs::msg::PoseStamped transformed_pose;
        tf2::doTransform(pose, transformed_pose, map_from_source);
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
      "Waiting to transform mission chunk from %s into %s before dispatching Nav2 poses: %s",
      odom_frame_id_.c_str(),
      map_frame_id_.c_str(),
      exception.what());
    return;
  }

  if (goal.poses.size() > 1U) {
    const auto dropped_start_pose = goal.poses.front();
    goal.poses.erase(goal.poses.begin());
    RCLCPP_INFO(
      get_logger(),
      "Dropping chunk start pose x=%.3f y=%.3f before dispatch; Nav2 starts from the current robot pose.",
      dropped_start_pose.pose.position.x,
      dropped_start_pose.pose.position.y);
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

  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const auto & goal_handle) {handleGoalResponse(goal_handle);};
  options.result_callback =
    [this](const auto & result) {handleGoalResult(result);};

  mission_active_ = true;
  waiting_for_goal_result_ = true;
  navigate_through_poses_client_->async_send_goal(goal, options);
  RCLCPP_INFO(
    get_logger(),
    "Dispatching mission chunk %zu/%zu with %zu pose(s).",
    active_chunk_index_ + 1U,
    mission_chunks_.size(),
    goal.poses.size());
}

bool MappingNode::areMissionCostmapsReadyForMissionStart() const
{
  if (!nav2_local_costmap_ready_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Waiting for mission costmaps before dispatching the first mission chunk. "
      "local=%s (%s) global=%s (%s).",
      nav2_local_costmap_ready_ ? "ready" : "missing",
      nav2_local_costmap_topic_.c_str(),
      latest_padded_live_map_ready_ ? "ready" : "missing",
      "mapping/static_costmap");
    return false;
  }

  if (!latest_padded_live_map_ready_ || latest_global_costmap_publish_stamp_.nanoseconds() == 0) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Waiting for mapping/static_costmap before dispatching the first mission chunk. "
      "Nav2 consumes mapping/static_costmap via StaticLayer, so the upstream mapping publisher "
      "must be ready before mission start.");
    return false;
  }

  const rclcpp::Time now_time = now();
  const double local_age = (now_time - latest_nav2_local_costmap_stamp_).seconds();
  const double global_age = (now_time - latest_nav2_global_costmap_stamp_).seconds();
  if (local_age > nav2_costmap_ready_timeout_seconds_ ||
    global_age > nav2_costmap_ready_timeout_seconds_)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Waiting for fresh mission costmap updates before dispatching the first mission chunk. "
      "local age=%.3fs global age=%.3fs timeout=%.3fs.",
      local_age,
      global_age,
      nav2_costmap_ready_timeout_seconds_);
    return false;
  }

  return true;
}

bool MappingNode::isNavigatorActive()
{
  if (!navigator_state_client_->wait_for_service(std::chrono::milliseconds(0))) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for navigator lifecycle service %s.",
      navigator_state_service_name_.c_str());
    return false;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future = navigator_state_client_->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(250)) != std::future_status::ready) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for navigator lifecycle state from %s.",
      navigator_state_service_name_.c_str());
    return false;
  }

  const auto response = future.get();
  if (!response) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Navigator lifecycle state request returned no response.");
    return false;
  }

  if (response->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Waiting for navigator to become active. Current state: %s (%u).",
      response->current_state.label.c_str(),
      static_cast<unsigned int>(response->current_state.id));
    return false;
  }

  return true;
}

void MappingNode::handleGoalResponse(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle)
{
  if (!goal_handle) {
    waiting_for_goal_result_ = false;
    mission_active_ = false;
    if (!isNavigatorActive()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Mission chunk was rejected before the navigator became active. Retrying.");
      return;
    }

    mission_completed_ = true;
    RCLCPP_ERROR(get_logger(), "Mission chunk was rejected by Nav2.");
    markMissionTerminal("aborted", "Nav2 rejected the mission chunk");
  }
}

void MappingNode::handleGoalResult(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result)
{
  waiting_for_goal_result_ = false;
  mission_active_ = false;

  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    const std::string failure_reason =
      "Nav2 navigate_through_poses ended with result code " +
      std::to_string(static_cast<int>(result.code));
    if (advancePastBlockedMissionWaypoint(failure_reason)) {
      RCLCPP_WARN(
        get_logger(),
        "Mission chunk failed with result code %d, but the blocked waypoint was skipped and the mission will continue.",
        static_cast<int>(result.code));
      return;
    }

    RCLCPP_ERROR(get_logger(), "Mission chunk failed with result code %d", static_cast<int>(result.code));
    mission_completed_ = true;
    markMissionTerminal("aborted", failure_reason);
    return;
  }

  consecutive_blocked_waypoint_advances_ = 0;
  ++active_chunk_index_;
  if (active_chunk_index_ >= mission_chunks_.size()) {
    mission_completed_ = true;
    RCLCPP_INFO(get_logger(), "All mission chunks completed successfully.");
    if (!repeat_mission_) {
      markMissionTerminal("completed", "autonomous mission completed");
    }
  }
}


bool MappingNode::advancePastBlockedMissionWaypoint(const std::string & failure_reason)
{
  if (!advance_past_blocked_waypoints_) {
    return false;
  }

  if (active_chunk_index_ >= mission_chunks_.size() || mission_route_.size() < 2U) {
    return false;
  }

  if (max_blocked_waypoint_advances_ >= 0 &&
    consecutive_blocked_waypoint_advances_ >= max_blocked_waypoint_advances_)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Blocked waypoint advance limit reached after %d skipped waypoint(s); aborting mission. Last failure: %s",
      consecutive_blocked_waypoint_advances_,
      failure_reason.c_str());
    return false;
  }

  const auto & failed_chunk = mission_chunks_.at(active_chunk_index_);
  if (failed_chunk.empty()) {
    return false;
  }

  const std::size_t failed_chunk_start_route_index =
    active_chunk_index_ < mission_chunk_start_indices_.size() ?
    mission_chunk_start_indices_.at(active_chunk_index_) : active_chunk_index_;
  const std::size_t blocked_route_index = std::min(
    failed_chunk_start_route_index + failed_chunk.size() - 1U,
    mission_route_.size() - 1U);
  const std::size_t resume_route_index = blocked_route_index + 1U;

  if (resume_route_index >= mission_route_.size()) {
    RCLCPP_ERROR(
      get_logger(),
      "Mission chunk failed at the final waypoint, so there is no later waypoint to advance to. Last failure: %s",
      failure_reason.c_str());
    return false;
  }

  std::vector<geometry_msgs::msg::PoseStamped> remaining_route;
  remaining_route.reserve(mission_route_.size() - resume_route_index + 1U);
  if (latest_odometry_pose_ready_) {
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header.frame_id = odom_frame_id_;
    current_pose.pose.position = latest_odometry_position_;
    current_pose.pose.orientation = latest_odometry_orientation_;
    remaining_route.push_back(current_pose);
  } else {
    remaining_route.push_back(mission_route_.at(failed_chunk_start_route_index));
  }
  remaining_route.insert(
    remaining_route.end(),
    mission_route_.begin() + static_cast<long>(resume_route_index),
    mission_route_.end());
  remaining_route = orientRoute(std::move(remaining_route));

  const std::size_t original_route_size = mission_route_.size();
  const auto skipped_pose = mission_route_.at(blocked_route_index);
  const auto resume_pose = mission_route_.at(resume_route_index);
  mission_route_ = std::move(remaining_route);
  mission_chunks_ = chunkRoute(mission_route_);
  mission_chunk_start_indices_ = chunkRouteStartIndices(mission_route_);
  active_chunk_index_ = 0U;
  ++consecutive_blocked_waypoint_advances_;

  if (mission_chunks_.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Skipping blocked waypoint %zu left no executable mission chunks. Last failure: %s",
      blocked_route_index + 1U,
      failure_reason.c_str());
    return false;
  }

  publishRouteMarker();
  RCLCPP_WARN(
    get_logger(),
    "Advancing past blocked mission waypoint %zu/%zu at x=%.3f y=%.3f after Nav2 failure (%s). Resuming toward waypoint %zu/%zu at x=%.3f y=%.3f; skipped waypoint count in this streak: %d/%d.",
    blocked_route_index + 1U,
    original_route_size,
    skipped_pose.pose.position.x,
    skipped_pose.pose.position.y,
    failure_reason.c_str(),
    resume_route_index + 1U,
    original_route_size,
    resume_pose.pose.position.x,
    resume_pose.pose.position.y,
    consecutive_blocked_waypoint_advances_,
    max_blocked_waypoint_advances_);
  return true;
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

  const std::string request_outcome = request->outcome;
  end_mission_client_->async_send_request(
    request,
    [this, request_outcome](rclcpp::Client<amr_sweeper_mission_executor::srv::EndMission>::SharedFuture future) {
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
        if (request_outcome == "completed") {
          mergeCompletedRuntimeCostmapIntoMissionCostmap();
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
  const bool has_seeded_static_map = saved_costmap_initialized_ || seeded_runtime_map_ready_;

  if (!padded_live_map_bounds_ready_) {
    if (has_seeded_static_map) {
      padded_live_map_min_x_ = source_min_x;
      padded_live_map_max_x_ = source_max_x;
      padded_live_map_min_y_ = source_min_y;
      padded_live_map_max_y_ = source_max_y;
      padded_live_map_bounds_ready_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Initialized persistent padded live-map bounds from the seeded static map extents in %s: [%.3f, %.3f] x [%.3f, %.3f].",
        message.header.frame_id.c_str(),
        padded_live_map_min_x_,
        padded_live_map_max_x_,
        padded_live_map_min_y_,
        padded_live_map_max_y_);
    } else if (!latest_odometry_pose_ready_ || !anchor_available) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting to initialize the mission startup map until the robot reference pose can be centered in %s.",
        message.header.frame_id.c_str());
      return nav_msgs::msg::OccupancyGrid{};
    } else {
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
  const double size_m = std::max(1.0, min_global_map_size_m_);
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
    const bool seed_index_available =
      seeded_runtime_map_ready_ &&
      seeded_runtime_map_.info.width == latest_live_map_.info.width &&
      seeded_runtime_map_.info.height == latest_live_map_.info.height &&
      seeded_runtime_map_.data.size() == latest_live_map_.data.size();
    const int8_t seeded_value = seed_index_available ? seeded_runtime_map_.data.at(index) : -1;
    const bool only_seeded_threshold_observation =
      seeded_value >= 0 &&
      global_map_observations_.at(index) == 1U &&
      ((global_map_scores_.at(index) == 20 &&
      global_map_occupied_observations_.at(index) == 1U) ||
      (global_map_scores_.at(index) == -20 &&
      global_map_free_observations_.at(index) == 1U));
    if (seeded_value >= 0 &&
      (global_map_observations_.at(index) == 0U || only_seeded_threshold_observation))
    {
      latest_live_map_.data.at(index) = seeded_value;
      continue;
    }
    if (global_map_observations_.at(index) == 0U) {
      latest_live_map_.data.at(index) = -1;
      continue;
    }
    const int score = std::clamp<int>(global_map_scores_.at(index), -20, 20);
    int occupancy = score >= 0 ?
      std::min(100, 50 + score * 2) :
      std::max(0, 50 + score * 2);
    if (score >= 20) {
      occupancy = 100;
    } else if (score <= -20) {
      occupancy = 0;
    }
    latest_live_map_.data.at(index) = static_cast<int8_t>(occupancy);
  }

  latest_live_map_.header.stamp = now();
  latest_live_map_.info.map_load_time = latest_live_map_.header.stamp;

  latest_padded_live_map_ = padLiveMap(latest_live_map_);
  if (latest_padded_live_map_.info.width == 0U || latest_padded_live_map_.info.height == 0U) {
    return;
  }
  latest_padded_live_map_ready_ = true;
  global_costmap_publisher_->publish(latest_padded_live_map_);
  latest_global_costmap_publish_stamp_ = latest_padded_live_map_.header.stamp;
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

  // Make the route start its own goal so Nav2 must explicitly drive to the
  // first waypoint before the remaining mission path is executed.
  chunks.push_back({route.front()});

  const std::size_t max_points_per_goal = static_cast<std::size_t>(std::max(1, max_segments_per_goal_) + 1);
  std::size_t start_index = 1U;
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


std::vector<std::size_t> MappingNode::chunkRouteStartIndices(
  const std::vector<geometry_msgs::msg::PoseStamped> & route) const
{
  std::vector<std::size_t> start_indices;
  if (route.size() < 2U) {
    return start_indices;
  }

  start_indices.push_back(0U);

  const std::size_t max_points_per_goal = static_cast<std::size_t>(std::max(1, max_segments_per_goal_) + 1);
  std::size_t start_index = 1U;
  while (start_index < route.size() - 1U) {
    const std::size_t end_index = std::min(start_index + max_points_per_goal, route.size());
    start_indices.push_back(start_index);
    if (end_index >= route.size()) {
      break;
    }
    start_index = end_index - 1U;
  }

  return start_indices;
}

void MappingNode::markMissionTerminal(const std::string & outcome, const std::string & reason)
{
  if (repeat_mission_) {
    return;
  }
  if (outcome == "completed") {
    persistRuntimeCostmapArtifact();
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
