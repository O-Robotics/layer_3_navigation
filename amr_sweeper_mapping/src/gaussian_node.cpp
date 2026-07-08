#include "gaussian_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace amr_sweeper_mapping
{

namespace
{

constexpr char kDefaultPointcloudTopic[] = "/amr_sweeper/depth_camera/depth/color/points";
constexpr char kDefaultWorldFrame[] = "map";
constexpr char kDefaultRepresentationName[] = "global_gaussian_map";
constexpr char kDefaultSurfaceTextureTopic[] = "/amr_sweeper/usb_cameras/tools_camera/image_raw";

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

}  // namespace

std::size_t GaussianVoxelKeyHash::operator()(const GaussianVoxelKey & key) const
{
  const std::size_t hx = std::hash<int>{}(key.x);
  const std::size_t hy = std::hash<int>{}(key.y);
  const std::size_t hz = std::hash<int>{}(key.z);
  return hx ^ (hy << 1U) ^ (hz << 2U);
}

GaussianNode::GaussianNode()
: Node("gaussian_node"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  declare_parameter("output_directory", std::string(""));
  declare_parameter("source_mode", std::string("realsense_pointcloud"));
  declare_parameter("representation_name", std::string(kDefaultRepresentationName));
  declare_parameter("world_frame", std::string(kDefaultWorldFrame));
  declare_parameter("pointcloud_topic", std::string(kDefaultPointcloudTopic));
  declare_parameter("enable_surface_texture_camera", true);
  declare_parameter("surface_texture_topic", std::string(kDefaultSurfaceTextureTopic));
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
  source_mode_ = get_parameter("source_mode").as_string();
  representation_name_ = get_parameter("representation_name").as_string();
  world_frame_ = get_parameter("world_frame").as_string();
  pointcloud_topic_ = get_parameter("pointcloud_topic").as_string();
  enable_surface_texture_camera_ = get_parameter("enable_surface_texture_camera").as_bool();
  surface_texture_topic_ = get_parameter("surface_texture_topic").as_string();
  voxel_size_meters_ = get_parameter("voxel_size_meters").as_double();
  min_range_meters_ = get_parameter("min_range_meters").as_double();
  max_range_meters_ = get_parameter("max_range_meters").as_double();
  process_every_nth_point_ =
    std::max<int>(1, static_cast<int>(get_parameter("process_every_nth_point").as_int()));
  max_voxels_ =
    std::max<int>(1, static_cast<int>(get_parameter("max_voxels").as_int()));
  max_debug_points_ =
    std::max<int>(1, static_cast<int>(get_parameter("max_debug_points").as_int()));
  save_ply_ = get_parameter("save_ply").as_bool();
  save_json_ = get_parameter("save_json").as_bool();

  pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&GaussianNode::processPointCloud, this, std::placeholders::_1));
  if (enable_surface_texture_camera_) {
    surface_texture_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      surface_texture_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&GaussianNode::handleSurfaceTextureImage, this, std::placeholders::_1));
  }
  debug_pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("mapping/gaussian/world_points", 1);
  status_publisher_ = create_publisher<std_msgs::msg::String>("mapping/gaussian/status", 10);

  status_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("status_period_seconds").as_double()),
    std::bind(&GaussianNode::publishStatus, this));
  debug_publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("debug_publish_period_seconds").as_double()),
    std::bind(&GaussianNode::publishDebugPointCloud, this));
  artifact_save_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("artifact_save_period_seconds").as_double()),
    std::bind(&GaussianNode::saveArtifacts, this));

  RCLCPP_INFO(
    get_logger(),
    "Gaussian world node listening on %s into frame %s with voxel size %.2f m.",
    pointcloud_topic_.c_str(),
    world_frame_.c_str(),
    voxel_size_meters_);
}

GaussianNode::~GaussianNode()
{
  try {
    saveArtifacts();
  } catch (const std::exception & exception) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to save gaussian artifacts during shutdown: %s",
      exception.what());
  }
}

void GaussianNode::handleSurfaceTextureImage(const sensor_msgs::msg::Image::SharedPtr message)
{
  (void)message;
  ++surface_texture_frames_;
}

void GaussianNode::processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
{
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
      "Incoming point cloud is missing rgb field; gaussian world needs colored points.");
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
      "Skipping point cloud due to missing transform %s -> %s: %s",
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

  for (std::size_t index = 0; index < point_count; ++index, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
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

void GaussianNode::publishStatus()
{
  std_msgs::msg::String message;
  message.data =
    "gaussian_node ready; source_mode=" + source_mode_ +
    "; world_frame=" + world_frame_ +
    "; voxels=" + std::to_string(voxels_.size()) +
    "; integrated_frames=" + std::to_string(integrated_frames_) +
    "; integrated_points=" + std::to_string(integrated_points_) +
    "; surface_texture_frames=" + std::to_string(surface_texture_frames_) +
    "; surface_texture_topic=" + surface_texture_topic_;
  status_publisher_->publish(message);
}

void GaussianNode::publishDebugPointCloud()
{
  if (voxels_.empty()) {
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

void GaussianNode::saveArtifacts()
{
  if (!artifact_dirty_ || voxels_.empty() || output_directory_.empty()) {
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
        {"weight", voxel.count}
      });
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
    "Saved gaussian world artifacts to %s.[json|ply]",
    base_path.c_str());
}

void GaussianNode::upsertVoxel(
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

GaussianVoxelKey GaussianNode::voxelKeyForPoint(const double x, const double y, const double z) const
{
  return {
    static_cast<int>(std::floor(x / voxel_size_meters_)),
    static_cast<int>(std::floor(y / voxel_size_meters_)),
    static_cast<int>(std::floor(z / voxel_size_meters_))};
}

std::string GaussianNode::artifactBasePath() const
{
  namespace fs = std::filesystem;
  return (fs::path(output_directory_) / representation_name_).string();
}

}  // namespace amr_sweeper_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_mapping::GaussianNode>());
  rclcpp::shutdown();
  return 0;
}
