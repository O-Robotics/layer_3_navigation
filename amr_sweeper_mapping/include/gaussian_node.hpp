#ifndef AMR_SWEEPER_MAPPING__GAUSSIAN_NODE_HPP_
#define AMR_SWEEPER_MAPPING__GAUSSIAN_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace amr_sweeper_mapping
{

struct GaussianVoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const GaussianVoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GaussianVoxelKeyHash
{
  std::size_t operator()(const GaussianVoxelKey & key) const;
};

struct GaussianVoxel
{
  double sum_x{0.0};
  double sum_y{0.0};
  double sum_z{0.0};
  double sum_r{0.0};
  double sum_g{0.0};
  double sum_b{0.0};
  double sum_squared_radius{0.0};
  std::uint32_t count{0U};
};

class GaussianNode : public rclcpp::Node
{
public:
  GaussianNode();
  ~GaussianNode() override;

private:
  void handleSurfaceTextureImage(const sensor_msgs::msg::Image::SharedPtr message);
  void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message);
  void publishStatus();
  void publishDebugPointCloud();
  void saveArtifacts();
  void upsertVoxel(
    double x,
    double y,
    double z,
    std::uint8_t red,
    std::uint8_t green,
    std::uint8_t blue);
  [[nodiscard]] GaussianVoxelKey voxelKeyForPoint(double x, double y, double z) const;
  [[nodiscard]] std::string artifactBasePath() const;

  std::string output_directory_;
  std::string source_mode_;
  std::string representation_name_;
  std::string world_frame_;
  std::string pointcloud_topic_;
  std::string surface_texture_topic_;
  double voxel_size_meters_;
  double min_range_meters_;
  double max_range_meters_;
  int process_every_nth_point_;
  int max_voxels_;
  int max_debug_points_;
  bool save_ply_;
  bool save_json_;
  bool enable_surface_texture_camera_{true};
  bool artifact_dirty_{false};
  std::uint64_t integrated_frames_{0U};
  std::uint64_t integrated_points_{0U};
  std::uint64_t surface_texture_frames_{0U};
  std::unordered_map<GaussianVoxelKey, GaussianVoxel, GaussianVoxelKeyHash> voxels_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr surface_texture_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pointcloud_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr debug_publish_timer_;
  rclcpp::TimerBase::SharedPtr artifact_save_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace amr_sweeper_mapping

#endif  // AMR_SWEEPER_MAPPING__GAUSSIAN_NODE_HPP_
