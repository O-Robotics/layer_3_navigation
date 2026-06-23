#ifndef AMR_SWEEPER_LOCALIZATION_ODOMETRY_PROJECTION_NODE_HPP_
#define AMR_SWEEPER_LOCALIZATION_ODOMETRY_PROJECTION_NODE_HPP_

#include <array>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace amr_sweeper_localization
{

class OdometryProjectionNode : public rclcpp::Node
{
public:
  explicit OdometryProjectionNode(const rclcpp::NodeOptions & options);

private:
  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  static void copyPlanarCovariance(
    const std::array<double, 36> & input,
    std::array<double, 36> * output);
  static double yawToQuaternionZ(double yaw);
  static double yawToQuaternionW(double yaw);

  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string odom_frame_;
  std::string body_frame_;
  std::string projected_frame_;
  bool publish_tf_{true};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace amr_sweeper_localization

#endif  // AMR_SWEEPER_LOCALIZATION_ODOMETRY_PROJECTION_NODE_HPP_
