#ifndef GAUSSIAN_NODE_HPP_
#define GAUSSIAN_NODE_HPP_

#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nlohmann/json.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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
  struct CameraStream
  {
    std::string name;
    std::string image_topic;
    std::string camera_info_topic;
    sensor_msgs::msg::Image::SharedPtr latest_image;
    sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info;
    std::deque<sensor_msgs::msg::Image::SharedPtr> image_history;
    std::uint64_t image_frames{0U};
    std::uint64_t camera_info_frames{0U};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription;
  };

  void handleCameraImage(std::size_t camera_index, sensor_msgs::msg::Image::SharedPtr message);
  void handleCameraInfo(std::size_t camera_index, sensor_msgs::msg::CameraInfo::SharedPtr message);
  void handleOdometry(nav_msgs::msg::Odometry::SharedPtr message);
  void handleMapPoseStatus(std_msgs::msg::String::SharedPtr message);
  void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message);
  void tryCaptureBundle();
  std::size_t expectedCameraCount() const;
  void publishStatus();
  void publishDebugPointCloud();
  void saveArtifacts();
  void saveManifest();
  [[nodiscard]] nlohmann::json buildManifest() const;
  [[nodiscard]] nlohmann::json cameraInfoToJson(
    const sensor_msgs::msg::CameraInfo & message) const;
  [[nodiscard]] nlohmann::json poseToJson(const geometry_msgs::msg::Pose & pose) const;
  [[nodiscard]] nlohmann::json pointcloudMetadataToJson(
    const sensor_msgs::msg::PointCloud2 & message,
    const std::string & file_name) const;
  [[nodiscard]] nlohmann::json transformToJson(
    const geometry_msgs::msg::TransformStamped & transform) const;
  [[nodiscard]] bool shouldCaptureForMotion() const;
  [[nodiscard]] double latestYawRadians() const;
  [[nodiscard]] std::string stampToString(const builtin_interfaces::msg::Time & stamp) const;
  [[nodiscard]] std::string relativeBundlePath(std::uint64_t bundle_index) const;
  [[nodiscard]] bool writeImageFile(
    const sensor_msgs::msg::Image & image,
    const std::filesystem::path & path,
    std::string & format,
    std::string & error) const;
  [[nodiscard]] bool writePointCloudFile(
    const sensor_msgs::msg::PointCloud2 & pointcloud,
    const std::filesystem::path & path,
    std::string & error) const;
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
  std::string mission_id_;
  std::string gaussian_manifest_file_;
  std::string source_mode_;
  std::string representation_name_;
  std::string world_frame_;
  std::string odom_topic_;
  std::string map_pose_status_topic_;
  std::string pointcloud_topic_;
  std::vector<CameraStream> camera_streams_;
  double voxel_size_meters_;
  double min_range_meters_;
  double max_range_meters_;
  double capture_distance_meters_;
  double capture_yaw_degrees_;
  double min_capture_period_seconds_;
  double max_camera_timestamp_skew_seconds_;
  double max_camera_sample_age_seconds_;
  double max_pointcloud_sample_age_seconds_;
  double camera_discovery_seconds_;
  int process_every_nth_point_;
  int max_voxels_;
  int max_debug_points_;
  int min_camera_count_;
  bool save_ply_;
  bool save_json_;
  bool enable_voxel_preview_{false};
  bool enable_pointcloud_capture_{true};
  bool artifact_dirty_{false};
  bool latest_odometry_ready_{false};
  bool last_capture_pose_ready_{false};
  std::uint64_t integrated_frames_{0U};
  std::uint64_t integrated_points_{0U};
  std::uint64_t capture_bundles_{0U};
  std::uint64_t rejected_bundles_{0U};
  std::uint64_t pointcloud_frames_{0U};
  std::uint64_t pointcloud_capture_files_{0U};
  geometry_msgs::msg::Pose latest_odometry_pose_;
  rclcpp::Time latest_odometry_stamp_;
  geometry_msgs::msg::Pose last_capture_pose_;
  rclcpp::Time last_capture_stamp_;
  std::string latest_map_pose_status_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
  std::vector<nlohmann::json> capture_records_;
  std::unordered_map<GaussianVoxelKey, GaussianVoxel, GaussianVoxelKeyHash> voxels_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_pose_status_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pointcloud_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::TimerBase::SharedPtr debug_publish_timer_;
  rclcpp::TimerBase::SharedPtr artifact_save_timer_;
  rclcpp::Time capture_start_time_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace amr_sweeper_mapping

#endif  // GAUSSIAN_NODE_HPP_
