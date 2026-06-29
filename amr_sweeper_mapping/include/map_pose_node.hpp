#ifndef AMR_SWEEPER_MAPPING__MAP_POSE_NODE_HPP_
#define AMR_SWEEPER_MAPPING__MAP_POSE_NODE_HPP_

#include <chrono>
#include <functional>
#include <mutex>
#include <memory>
#include <optional>
#include <array>
#include <condition_variable>
#include <string>
#include <thread>
#include <vector>

#include <fusioncore_ros/srv/from_ll.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace amr_sweeper_mapping
{

enum class MapPoseHealthState
{
  BOOTSTRAP,
  DEGRADED,
  HEALTHY,
  WARNING,
  FAULT
};

class MapPoseNode : public rclcpp::Node
{
public:
  MapPoseNode();
  ~MapPoseNode() override;

private:
  struct MapMatchEstimate
  {
    tf2::Transform map_to_base;
    double score{0.0};
    double confidence{0.0};
  };

  struct StateSnapshot
  {
    bool latest_odometry_ready{false};
    bool latest_navsat_ready{false};
    bool latest_heading_ready{false};
    bool latest_scan_ready{false};
    bool latest_global_costmap_ready{false};
    bool last_map_to_odom_ready{false};
    bool map_to_odom_filter_ready{false};
    bool correction_startup_ready{false};
    int correction_ready_streak{0};
    geometry_msgs::msg::Point latest_odometry_position;
    geometry_msgs::msg::Quaternion latest_odometry_orientation;
    sensor_msgs::msg::NavSatFix latest_navsat;
    sensor_msgs::msg::Imu latest_heading;
    sensor_msgs::msg::LaserScan latest_scan;
    nav_msgs::msg::OccupancyGrid latest_global_costmap;
    std::shared_ptr<const std::vector<float>> latest_global_costmap_score_field;
    rclcpp::Time latest_odometry_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time latest_scan_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time latest_global_costmap_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time latest_map_pose_stamp{0, 0, RCL_ROS_TIME};
    std::array<double, 3> map_to_odom_filter_state{0.0, 0.0, 0.0};
    std::array<double, 3> map_to_odom_filter_covariance{1.0, 1.0, 0.5};
    tf2::Transform last_map_to_odom;
  };

  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handleNavSat(const sensor_msgs::msg::NavSatFix::SharedPtr message);
  void handleHeading(const sensor_msgs::msg::Imu::SharedPtr message);
  void handleScan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void handleGlobalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  void globalCostmapWorkerLoop();
  void publishMapToOdomTransform();
  void loadCostmapGeoreference();
  [[nodiscard]] std::shared_ptr<std::vector<float>> buildGlobalCostmapScoreField(
    const nav_msgs::msg::OccupancyGrid & map) const;
  void initializeMapToOdomFilter();
  void initializeMapToOdomFilterLocked();
  void predictMapToOdomFilter();
  void predictMapToOdomFilterLocked();
  void updateMapToOdomFilter(const tf2::Transform & measurement, double confidence);
  void updateMapToOdomFilterLocked(const tf2::Transform & measurement, double confidence);
  void decayMapToOdomFilterTowardsIdentity();
  void decayMapToOdomFilterTowardsIdentityLocked();
  [[nodiscard]] tf2::Transform filteredMapToOdomTransform() const;
  [[nodiscard]] tf2::Transform filteredMapToOdomTransform(
    const std::array<double, 3> & filter_state) const;
  [[nodiscard]] float scoreCostmapCell(
    const StateSnapshot & snapshot,
    int grid_x,
    int grid_y) const;
  [[nodiscard]] StateSnapshot snapshotState() const;
  void publishMapPoseStatus(const StateSnapshot & snapshot);
  [[nodiscard]] bool odometryInputReady(const StateSnapshot & snapshot) const;
  [[nodiscard]] bool correctionInputsReady(const StateSnapshot & snapshot) const;
  [[nodiscard]] std::string composeHealthReason(const StateSnapshot & snapshot) const;
  [[nodiscard]] bool shouldHoldIdentityAtStartup(const StateSnapshot & snapshot);
  [[nodiscard]] std::optional<geometry_msgs::msg::Point> latestMapPositionFromNavSat() const;
  [[nodiscard]] std::optional<geometry_msgs::msg::Point> mapPositionFromArtifactGeoreference() const;
  [[nodiscard]] std::optional<geographic_msgs::msg::GeoPoint> artifactGeoPointFromMapPoint(
    const geometry_msgs::msg::Point & map_point) const;
  [[nodiscard]] double georeferenceConsistencyConfidence(
    const StateSnapshot & snapshot,
    const tf2::Transform & candidate_map_to_base) const;
  [[nodiscard]] std::optional<MapMatchEstimate> estimateMapToBaseFromPrior(
    const StateSnapshot & snapshot,
    const tf2::Transform & map_to_base_prior) const;

  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string odometry_topic_;
  std::string navsat_topic_;
  std::string heading_topic_;
  std::string scan_topic_;
  std::string global_costmap_topic_;
  std::string status_topic_;
  std::string fromll_service_name_;
  std::string costmap_yaml_path_;
  bool publish_identity_when_pose_missing_{true};
  bool latest_odometry_ready_{false};
  bool latest_navsat_ready_{false};
  bool latest_heading_ready_{false};
  bool latest_scan_ready_{false};
  bool latest_global_costmap_ready_{false};
  bool last_map_to_odom_ready_{false};
  bool map_to_odom_filter_ready_{false};
  bool artifact_georeference_ready_{false};
  bool correction_startup_ready_{false};
  int correction_ready_streak_{0};
  int occupied_threshold_{65};
  int scan_subsample_step_{4};
  int min_valid_scan_points_{20};
  int endpoint_search_radius_cells_{1};
  int max_scan_match_points_{72};
  int max_match_compute_time_ms_{30};
  double search_translation_window_m_{2.0};
  double search_translation_step_m_{0.25};
  double search_yaw_window_rad_{0.35};
  double search_yaw_step_rad_{0.0872664626};
  double min_search_translation_window_m_{0.2};
  double min_search_yaw_window_rad_{0.0872664626};
  double search_window_translation_covariance_scale_{0.5};
  double search_window_yaw_covariance_scale_{0.5};
  double max_scan_match_range_m_{2.0};
  double translation_penalty_per_meter_{0.5};
  double yaw_penalty_per_rad_{0.25};
  double free_space_penalty_{0.35};
  double free_space_reward_{0.02};
  double occupied_reward_{1.0};
  double occupied_penalty_{0.3};
  double likelihood_field_max_distance_m_{0.75};
  double likelihood_field_sigma_m_{0.12};
  double unknown_space_score_{0.05};
  double out_of_bounds_score_{-0.25};
  double prior_blend_weight_{0.7};
  double scan_timeout_seconds_{1.0};
  double costmap_timeout_seconds_{2.0};
  double odometry_timeout_seconds_{1.0};
  double scan_match_period_seconds_{0.5};
  double global_costmap_min_update_period_seconds_{0.5};
  int startup_ready_streak_required_{3};
  int degraded_streak_before_warning_{3};
  int degraded_streak_before_fault_{6};
  double max_translation_jump_m_{0.75};
  double max_yaw_jump_rad_{0.35};
  double transform_smoothing_alpha_{0.35};
  double minimum_match_score_{0.15};
  double high_confidence_match_score_{0.75};
  double minimum_confidence_for_filter_update_{0.45};
  double low_confidence_identity_pull_alpha_{0.15};
  double georef_consistency_max_error_m_{5.0};
  double georef_consistency_min_confidence_{0.2};
  std::array<double, 3> process_noise_diagonal_{0.01, 0.01, 0.005};
  std::array<double, 3> measurement_noise_diagonal_min_{0.05, 0.05, 0.02};
  std::array<double, 3> measurement_noise_diagonal_max_{0.6, 0.6, 0.3};
  geometry_msgs::msg::Point latest_odometry_position_;
  geometry_msgs::msg::Quaternion latest_odometry_orientation_;
  sensor_msgs::msg::NavSatFix latest_navsat_;
  sensor_msgs::msg::Imu latest_heading_;
  sensor_msgs::msg::LaserScan latest_scan_;
  nav_msgs::msg::OccupancyGrid latest_global_costmap_;
  nav_msgs::msg::OccupancyGrid pending_global_costmap_;
  std::shared_ptr<std::vector<float>> latest_global_costmap_score_field_;
  rclcpp::Time latest_odometry_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_heading_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_global_costmap_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_global_costmap_processed_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pending_global_costmap_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_match_attempt_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_map_pose_stamp_{0, 0, RCL_ROS_TIME};
  std::array<double, 3> artifact_longitude_coefficients_{0.0, 0.0, 0.0};
  std::array<double, 3> artifact_latitude_coefficients_{0.0, 0.0, 0.0};
  std::array<double, 3> map_to_odom_filter_state_{0.0, 0.0, 0.0};
  std::array<double, 3> map_to_odom_filter_covariance_{1.0, 1.0, 0.5};
  tf2::Transform last_map_to_odom_;
  mutable std::mutex state_mutex_;
  std::condition_variable global_costmap_worker_cv_;
  bool pending_global_costmap_ready_{false};
  bool pending_global_costmap_dirty_{false};
  bool shutdown_global_costmap_worker_{false};
  MapPoseHealthState health_state_{MapPoseHealthState::BOOTSTRAP};
  int degraded_input_streak_{0};
  std::thread global_costmap_worker_;
  rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
  rclcpp::CallbackGroup::SharedPtr publish_callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr heading_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedPtr fromll_client_;
};

}  // namespace amr_sweeper_mapping

#endif  // AMR_SWEEPER_MAPPING__MAP_POSE_NODE_HPP_
