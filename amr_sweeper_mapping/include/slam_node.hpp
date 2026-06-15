#ifndef AMR_SWEEPER_MAPPING__SLAM_NODE_HPP_
#define AMR_SWEEPER_MAPPING__SLAM_NODE_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace amr_sweeper_mapping
{

class SlamNode : public rclcpp::Node
{
public:
  SlamNode();

private:
  void handleFusionPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);
  void handleScan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void handleSlamPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);
  void publishStatus();
  void maybeSeedInitialPose();
  [[nodiscard]] bool buildSeedPose(geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose);

  std::string backend_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string fusion_pose_topic_;
  std::string slam_pose_topic_;
  std::string scan_topic_;
  bool seed_slam_from_fusion_pose_{true};
  int max_seed_publications_{5};
  int seed_publications_{0};
  bool warned_about_seed_projection_{false};
  bool seed_pose_ready_{false};
  bool have_fusion_pose_{false};
  bool have_scan_{false};
  bool have_slam_pose_{false};
  bool warned_about_seed_frame_mismatch_{false};
  geometry_msgs::msg::PoseWithCovarianceStamped latest_fusion_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped frozen_seed_pose_;
  std::string latest_fusion_pose_frame_id_;
  std::string last_scan_frame_id_;
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_slam_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fusion_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr seed_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace amr_sweeper_mapping

#endif  // AMR_SWEEPER_MAPPING__SLAM_NODE_HPP_
