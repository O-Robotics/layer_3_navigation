#ifndef AMR_SWEEPER_MAPPING__AMR_SWEEPER_MAPPING_NODE_HPP_
#define AMR_SWEEPER_MAPPING__AMR_SWEEPER_MAPPING_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <fusioncore_ros/srv/from_ll.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace amr_sweeper_mapping
{

struct MissionCoordinate
{
  double x;
  double y;
  bool use_local_frame{false};
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

  LoadedCostmapArtifact artifact_;
  bool artifact_loaded_{false};
};

class MappingNode : public rclcpp::Node
{
public:
  MappingNode();

private:
  void handleSlamStatus(const std_msgs::msg::String::SharedPtr message);
  void handleGaussianStatus(const std_msgs::msg::String::SharedPtr message);
  void publishCoordinatorStatus();
  void tickMissionExecution();
  void ensureMissionLoaded();
  void convertMissionRoute();
  void startNextMissionChunk();
  void handleGoalResponse(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr goal_handle);
  void handleGoalResult(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult & result);
  void publishRouteMarker() const;
  void writeMissionSessionMetadata() const;
  [[nodiscard]] std::string routeGeoJsonPath() const;
  [[nodiscard]] std::vector<MissionCoordinate> loadRouteCoordinates(const std::string & path) const;
  [[nodiscard]] std::vector<geometry_msgs::msg::PoseStamped> buildPoseSequence(
    const std::vector<MissionCoordinate> & coordinates) const;
  [[nodiscard]] std::vector<std::vector<geometry_msgs::msg::PoseStamped>> chunkRoute(
    const std::vector<geometry_msgs::msg::PoseStamped> & route) const;

  std::string mission_file_;
  std::string slam_backend_;
  std::string gaussian_mode_;
  std::string mission_route_file_;
  std::string mission_id_;
  std::string mission_output_directory_;
  std::string mission_window_start_;
  std::string mission_window_end_;
  std::string frame_id_;
  std::string fromll_service_name_;
  bool auto_start_mission_{true};
  bool repeat_mission_{false};
  bool mission_loaded_{false};
  bool mission_converted_{false};
  bool mission_active_{false};
  bool waiting_for_goal_result_{false};
  bool mission_completed_{false};
  std::size_t active_chunk_index_{0U};
  int max_segments_per_goal_{4};
  std::vector<geometry_msgs::msg::PoseStamped> mission_route_;
  std::vector<std::vector<geometry_msgs::msg::PoseStamped>> mission_chunks_;
  std::string last_slam_status_{"unavailable"};
  std::string last_gaussian_status_{"unavailable"};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr slam_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gaussian_status_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr route_marker_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
  rclcpp::Client<fusioncore_ros::srv::FromLL>::SharedPtr fromll_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_client_;
};

}  // namespace amr_sweeper_mapping

#endif  // AMR_SWEEPER_MAPPING__AMR_SWEEPER_MAPPING_NODE_HPP_
