#ifndef AMR_SWEEPER_MAPPING__AMR_SWEEPER_MAPPING_NODE_HPP_
#define AMR_SWEEPER_MAPPING__AMR_SWEEPER_MAPPING_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <amr_sweeper_mission_executor/srv/end_mission.hpp>
#include <fusioncore_ros/srv/from_ll.hpp>
#include <fusioncore_ros/srv/get_datum.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
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

struct LoadedCostmapArtifact
{
  std::vector<unsigned char> costs;
  unsigned int width_cells{0U};
  unsigned int height_cells{0U};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
};

class Vda5050CostmapLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  Vda5050CostmapLayer();

  void onInitialize() override;
  void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j) override;
  void reset() override;
  bool isClearable() override;
  void matchSize() override;

private:
  void loadArtifact();
  [[nodiscard]] LoadedCostmapArtifact parseCostmapArtifact(const std::string & yaml_path) const;
  [[nodiscard]] std::string resolveArtifactPath(const std::string & configured_path) const;
  [[nodiscard]] unsigned char sampleCostAtWorld(double world_x, double world_y) const;
  [[nodiscard]] geometry_msgs::msg::PointStamped transformPoint(
    double x,
    double y,
    const std::string & source_frame,
    const std::string & target_frame) const;

  LoadedCostmapArtifact artifact_;
  bool artifact_loaded_{false};
  std::string artifact_frame_id_{"odom"};
  std::string global_frame_id_;
};

class MappingNode : public rclcpp::Node
{
public:
  MappingNode();

private:
  void handleSlamStatus(const std_msgs::msg::String::SharedPtr message);
  void handleGaussianStatus(const std_msgs::msg::String::SharedPtr message);
  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handleLiveMap(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  void publishCoordinatorStatus();
  void tickMissionExecution();
  void tryRequestMissionEnd();
  void publishEarthToMapTransform();
  void ensureMissionLoaded();
  void convertMissionRoute();
  void startNextMissionChunk();
  [[nodiscard]] bool isWaypointFollowerActive();
  [[nodiscard]] bool refreshFusionDatum();
  void handleGoalResponse(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr goal_handle);
  void handleGoalResult(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult & result);
  void publishRouteMarker() const;
  void writeMissionSessionMetadata() const;
  void writeActualPathArtifact() const;
  [[nodiscard]] std::string routeGeoJsonPath() const;
  [[nodiscard]] std::string resolveRuntimePath(const std::string & configured_path) const;
  [[nodiscard]] std::vector<MissionCoordinate> loadRouteCoordinates(const std::string & path) const;
  [[nodiscard]] std::vector<geometry_msgs::msg::PoseStamped> buildPoseSequence(
    const std::vector<MissionCoordinate> & coordinates) const;
  [[nodiscard]] nav_msgs::msg::OccupancyGrid padLiveMap(
    const nav_msgs::msg::OccupancyGrid & message) const;
  [[nodiscard]] std::vector<std::vector<geometry_msgs::msg::PoseStamped>> chunkRoute(
    const std::vector<geometry_msgs::msg::PoseStamped> & route) const;
  void markMissionTerminal(const std::string & outcome, const std::string & reason);

  std::string mission_file_;
  std::string mission_type_;
  std::string execution_mode_{"follow_waypoints"};
  std::string mission_costmap_yaml_;
  std::string slam_backend_;
  std::string gaussian_mode_;
  std::string mission_route_file_;
  std::string mission_id_;
  std::string mission_output_directory_;
  std::string actual_path_output_file_;
  std::string actual_path_navsat_output_file_;
  std::string mission_window_start_;
  std::string mission_window_end_;
  std::string frame_id_;
  std::string earth_frame_id_;
  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string fromll_service_name_;
  std::string datum_service_name_;
  std::string end_mission_service_name_;
  std::string waypoint_follower_state_service_name_;
  bool auto_start_mission_{true};
  bool repeat_mission_{false};
  bool publish_earth_to_map_{true};
  bool earth_to_map_planar_only_{true};
  bool manual_mapping_mode_{false};
  bool mission_loaded_{false};
  bool mission_converted_{false};
  bool mission_active_{false};
  bool waiting_for_goal_result_{false};
  bool mission_completed_{false};
  bool mission_end_requested_{false};
  bool mission_end_pending_{false};
  std::size_t active_chunk_index_{0U};
  int max_segments_per_goal_{4};
  double max_waypoint_spacing_m_{0.5};
  std::string pending_end_outcome_{"completed"};
  std::string pending_end_reason_;
  std::vector<geometry_msgs::msg::PoseStamped> mission_route_;
  std::vector<geometry_msgs::msg::Point> traveled_path_points_;
  std::vector<std::vector<geometry_msgs::msg::PoseStamped>> mission_chunks_;
  std::string last_slam_status_{"unavailable"};
  std::string last_gaussian_status_{"unavailable"};
  geographic_msgs::msg::GeoPoint fusion_datum_;
  bool fusion_datum_ready_{false};
  bool pad_live_map_to_minimum_size_{true};
  double min_global_map_size_m_{10.0};
  bool live_map_ready_{false};
  bool latest_padded_live_map_ready_{false};
  bool latest_odometry_pose_ready_{false};
  bool mission_anchor_pose_ready_{false};
  geometry_msgs::msg::Point latest_odometry_position_;
  geometry_msgs::msg::Quaternion latest_odometry_orientation_;
  geometry_msgs::msg::Point mission_anchor_position_;
  geometry_msgs::msg::Quaternion mission_anchor_orientation_;
  nav_msgs::msg::OccupancyGrid latest_padded_live_map_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr slam_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gaussian_status_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr live_map_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr route_marker_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr padded_live_map_publisher_;
  rclcpp::CallbackGroup::SharedPtr mission_callback_group_;
  rclcpp::CallbackGroup::SharedPtr status_callback_group_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
  rclcpp::TimerBase::SharedPtr earth_to_map_timer_;
  rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedPtr fromll_client_;
  rclcpp::Client<fusioncore_ros::srv::GetDatum>::SharedPtr datum_client_;
  rclcpp::Client<amr_sweeper_mission_executor::srv::EndMission>::SharedPtr end_mission_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr waypoint_follower_state_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace amr_sweeper_mapping

#endif  // AMR_SWEEPER_MAPPING__AMR_SWEEPER_MAPPING_NODE_HPP_
