#ifndef AMR_SWEEPER_MAPPING__AMR_SWEEPER_MAPPING_NODE_HPP_
#define AMR_SWEEPER_MAPPING__AMR_SWEEPER_MAPPING_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <array>
#include <string>
#include <vector>

#include <amr_sweeper_mission_executor/srv/end_mission.hpp>
#include <fusioncore_ros/srv/from_ll.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

namespace amr_sweeper_mapping
{

struct MissionCoordinate
{
  double x;
  double y;
  bool use_local_frame{false};
  std::string frame_id{"odom"};
};

struct GeoPoint
{
  double latitude{0.0};
  double longitude{0.0};
};

struct GeoTransform
{
  bool valid{false};
  double longitude_coefficients[3]{0.0, 0.0, 0.0};
  double latitude_coefficients[3]{0.0, 0.0, 0.0};
};

struct RawNavSatSample
{
  double longitude{0.0};
  double latitude{0.0};
  double altitude{0.0};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct HeadingSample
{
  double yaw{0.0};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct SynchronizedPathSample
{
  geometry_msgs::msg::Point odom_position;
  double yaw{0.0};
  rclcpp::Time odom_stamp{0, 0, RCL_ROS_TIME};
  RawNavSatSample raw_navsat;
};

struct LoadedCostmapArtifact
{
  std::vector<unsigned char> costs;
  unsigned int width_cells{0U};
  unsigned int height_cells{0U};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::string mode{"trinary"};
  double occupied_thresh{0.65};
  double free_thresh{0.196};
  bool georeference_valid{false};
  std::string georeference_type;
  std::string georeference_source_crs{"EPSG:4326"};
  std::string georeference_companion_file;
  std::size_t georeference_sample_count{0U};
  std::array<double, 3> longitude_coefficients{0.0, 0.0, 0.0};
  std::array<double, 3> latitude_coefficients{0.0, 0.0, 0.0};
};

class MappingNode : public rclcpp::Node
{
public:
  MappingNode();

private:
  void handleScan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handleNavSat(const sensor_msgs::msg::NavSatFix::SharedPtr message);
  void handleHeading(const sensor_msgs::msg::Imu::SharedPtr message);
  void handleMapPoseStatus(const std_msgs::msg::String::SharedPtr message);
  void handleNav2LocalCostmap(const nav2_msgs::msg::Costmap::SharedPtr message);
  void handleNav2LocalCostmapUpdate(const map_msgs::msg::OccupancyGridUpdate::SharedPtr message);
  void handleNav2GlobalCostmap(const nav2_msgs::msg::Costmap::SharedPtr message);
  void handleNav2GlobalCostmapUpdate(const map_msgs::msg::OccupancyGridUpdate::SharedPtr message);
  void publishCoordinatorStatus();
  [[nodiscard]] std::string composeMapBuilderStatus() const;
  [[nodiscard]] bool updateStartupTfReadiness(const std::string & scan_frame_id);
  void persistRuntimeCostmapArtifact();
  void mergeCompletedRuntimeCostmapIntoMissionStaticCostmap();
  void tickMissionExecution();
  void tryRequestMissionEnd();
  void publishMapAlignmentTransform();
  void integrateScanIntoGlobalMap(const sensor_msgs::msg::LaserScan & message);
  void ensureMissionLoaded();
  void convertMissionRoute();
  void startNextMissionChunk();
  [[nodiscard]] bool isNavigatorActive();
  void handleGoalResponse(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle);
  void handleGoalResult(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result);
  [[nodiscard]] bool advancePastBlockedMissionWaypoint(const std::string & failure_reason);
  void publishRouteMarker() const;
  void writeMissionSessionMetadata() const;
  void writeActualPathArtifact() const;
  void writeActualPathNavSatArtifact() const;
  void writeAnchoredMissionRouteArtifact(const std::string & source_coordinate_frame) const;
  [[nodiscard]] std::string routeGeoJsonPath() const;
  [[nodiscard]] std::string resolveRuntimePath(const std::string & configured_path) const;
  [[nodiscard]] std::vector<MissionCoordinate> loadRouteCoordinates(const std::string & path) const;
  [[nodiscard]] std::vector<geometry_msgs::msg::PoseStamped> buildPoseSequence(
    const std::vector<MissionCoordinate> & coordinates) const;
  [[nodiscard]] nav_msgs::msg::OccupancyGrid padLiveMap(const nav_msgs::msg::OccupancyGrid & message);
  void loadSavedCostmapIfConfigured();
  void initializeMapFromArtifact(const LoadedCostmapArtifact & artifact);
  [[nodiscard]] std::optional<LoadedCostmapArtifact> projectArtifactIntoCurrentMap(
    const LoadedCostmapArtifact & artifact,
    const RawNavSatSample & anchor_navsat,
    double anchor_heading_yaw) const;
  [[nodiscard]] bool useAuthoritativeMissionGeoreference() const;
  void tryInitializeSavedCostmapFromSensors();
  void pruneGeoreferenceSamples();
  [[nodiscard]] std::optional<RawNavSatSample> stabilizedNavSatSample() const;
  [[nodiscard]] std::optional<double> stabilizedHeadingYaw() const;
  [[nodiscard]] std::optional<double> navSatHistoryMaxSpreadMeters() const;
  [[nodiscard]] std::optional<double> headingHistoryMaxDeviationRadians(double reference_yaw) const;
  [[nodiscard]] nav_msgs::msg::OccupancyGrid buildStaticRuntimeCostmapArtifact() const;
  void tryPublishBootstrapGlobalCostmap();
  void initializeGlobalMap(const geometry_msgs::msg::Point & center);
  void expandGlobalMapToFit(
    double min_x,
    double min_y,
    double max_x,
    double max_y);
  [[nodiscard]] bool worldToGrid(
    const nav_msgs::msg::OccupancyGrid & map,
    double world_x,
    double world_y,
    int & grid_x,
    int & grid_y) const;
  void markCellFree(int grid_x, int grid_y);
  void markCellOccupied(int grid_x, int grid_y);
  void publishGlobalMaps();
  [[nodiscard]] bool areMissionStaticCostmapsReadyForMissionStart() const;
  [[nodiscard]] std::vector<std::vector<geometry_msgs::msg::PoseStamped>> chunkRoute(
    const std::vector<geometry_msgs::msg::PoseStamped> & route) const;
  [[nodiscard]] std::vector<std::size_t> chunkRouteStartIndices(
    const std::vector<geometry_msgs::msg::PoseStamped> & route) const;
  void markMissionTerminal(const std::string & outcome, const std::string & reason);

  std::string mission_file_;
  std::string mission_type_;
  std::string execution_mode_{"navigate_through_poses"};
  std::string mission_static_costmap_yaml_;
  std::string slam_backend_;
  std::string gaussian_mode_;
  std::string mission_route_file_;
  std::string mission_id_;
  std::string mission_output_directory_;
  std::string actual_path_output_file_;
  std::string actual_path_navsat_output_file_;
  std::string startup_saved_costmap_yaml_;
  std::string saved_costmap_yaml_;
  std::string persistent_mission_static_costmap_yaml_;
  std::string mission_window_start_;
  std::string mission_window_end_;
  std::string frame_id_;
  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string mission_route_frame_id_;
  std::string base_frame_;
  std::string navsat_topic_;
  std::string heading_topic_;
  std::string scan_topic_;
  std::string seeded_map_frame_id_;
  std::string fromll_service_name_;
  std::string end_mission_service_name_;
  std::string navigator_state_service_name_;
  std::string nav2_local_costmap_topic_;
  std::string nav2_local_costmap_updates_topic_;
  std::string nav2_global_costmap_topic_;
  std::string nav2_global_costmap_updates_topic_;
  std::string map_pose_status_topic_;
  double runtime_costmap_save_period_seconds_{10.0};
  double nav2_costmap_ready_timeout_seconds_{2.5};
  double startup_tf_lookup_timeout_seconds_{0.05};
  int static_obstacle_min_observations_{6};
  int startup_tf_ready_streak_required_{3};
  double static_obstacle_min_occupied_fraction_{0.75};
  double static_obstacle_min_free_fraction_{0.75};
  bool bootstrap_empty_global_costmap_{false};
  bool bootstrap_global_costmap_published_{false};
  bool auto_start_mission_{true};
  bool repeat_mission_{false};
  bool publish_seeded_map_to_odom_{false};
  bool manual_mapping_mode_{false};
  bool mission_loaded_{false};
  bool mission_converted_{false};
  bool mission_active_{false};
  bool waiting_for_goal_result_{false};
  bool mission_completed_{false};
  bool mission_end_requested_{false};
  bool mission_end_pending_{false};
  bool persistent_mission_static_costmap_merged_{false};
  bool advance_past_blocked_waypoints_{true};
  std::size_t active_chunk_index_{0U};
  int max_segments_per_goal_{1};
  int max_blocked_waypoint_advances_{20};
  int consecutive_blocked_waypoint_advances_{0};
  double global_map_resolution_m_{0.05};
  double max_waypoint_spacing_m_{0.5};
  double georef_lock_window_seconds_{3.0};
  double georef_lock_fallback_timeout_seconds_{6.0};
  double georef_lock_max_navsat_spread_m_{2.0};
  double georef_lock_max_heading_deviation_deg_{12.0};
  int georef_lock_min_samples_{10};
  std::string pending_end_outcome_{"completed"};
  std::string pending_end_reason_;
  std::vector<geometry_msgs::msg::PoseStamped> mission_route_;
  std::vector<geometry_msgs::msg::Point> traveled_path_points_;
  std::vector<std::vector<geometry_msgs::msg::PoseStamped>> mission_chunks_;
  std::vector<std::size_t> mission_chunk_start_indices_;
  std::string last_map_builder_status_{"unavailable"};
  bool pad_live_map_to_minimum_size_{true};
  double min_global_map_size_m_{10.0};
  bool live_map_ready_{false};
  bool latest_padded_live_map_ready_{false};
  std::string map_pose_status_state_{"UNKNOWN"};
  std::string latest_map_pose_status_message_;
  std::string startup_tf_status_detail_;
  bool nav2_local_costmap_ready_{false};
  bool nav2_global_costmap_ready_{false};
  bool latest_odometry_pose_ready_{false};
  bool latest_heading_ready_{false};
  bool georeferenced_costmap_locked_{false};
  bool mission_anchor_pose_ready_{false};
  bool padded_live_map_bounds_ready_{false};
  bool startup_tf_ready_{false};
  bool have_scan_{false};
  sensor_msgs::msg::LaserScan latest_scan_;
  geometry_msgs::msg::Point latest_odometry_position_;
  geometry_msgs::msg::Quaternion latest_odometry_orientation_;
  sensor_msgs::msg::Imu latest_heading_;
  rclcpp::Time latest_nav2_local_costmap_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_nav2_global_costmap_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_global_costmap_publish_stamp_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Point mission_anchor_position_;
  geometry_msgs::msg::Quaternion mission_anchor_orientation_;
  std::string last_scan_frame_id_;
  int startup_tf_ready_streak_{0};
  double padded_live_map_min_x_{0.0};
  double padded_live_map_min_y_{0.0};
  double padded_live_map_max_x_{0.0};
  double padded_live_map_max_y_{0.0};
  std::vector<int16_t> global_map_scores_;
  std::vector<uint16_t> global_map_observations_;
  std::vector<uint16_t> global_map_occupied_observations_;
  std::vector<uint16_t> global_map_free_observations_;
  nav_msgs::msg::OccupancyGrid latest_live_map_;
  nav_msgs::msg::OccupancyGrid latest_padded_live_map_;
  nav_msgs::msg::OccupancyGrid seeded_runtime_map_;
  bool seeded_runtime_map_ready_{false};
  bool saved_costmap_initialized_{false};
  bool pending_seed_deadline_armed_{false};
  bool pending_seed_timeout_logged_{false};
  // Wall/steady clock, deliberately NOT rclcpp::Time: under use_sim_time the ROS
  // clock reads as time-zero until the first /clock message arrives, which would
  // make a now()-based deadline computed at construction time already appear
  // expired the moment real timestamps start flowing.
  std::chrono::steady_clock::time_point pending_seed_deadline_{};
  std::optional<LoadedCostmapArtifact> authoritative_saved_costmap_artifact_;
  std::optional<LoadedCostmapArtifact> pending_saved_costmap_artifact_;
  std::optional<RawNavSatSample> locked_georef_navsat_sample_;
  std::optional<double> locked_georef_heading_yaw_;
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  mutable std::mutex synchronized_path_mutex_;
  std::optional<RawNavSatSample> latest_raw_navsat_sample_;
  std::vector<RawNavSatSample> navsat_history_;
  std::vector<HeadingSample> heading_history_;
  std::vector<SynchronizedPathSample> synchronized_path_samples_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr heading_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_pose_status_subscription_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr nav2_local_costmap_subscription_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
    nav2_local_costmap_updates_subscription_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr nav2_global_costmap_subscription_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
    nav2_global_costmap_updates_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_path_publisher_;
  rclcpp::CallbackGroup::SharedPtr mission_callback_group_;
  rclcpp::CallbackGroup::SharedPtr status_callback_group_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
  rclcpp::TimerBase::SharedPtr runtime_costmap_save_timer_;
  rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedPtr fromll_client_;
  rclcpp::Client<amr_sweeper_mission_executor::srv::EndMission>::SharedPtr end_mission_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr navigator_state_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigate_through_poses_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace amr_sweeper_mapping

#endif  // AMR_SWEEPER_MAPPING__AMR_SWEEPER_MAPPING_NODE_HPP_
