// Copyright 2026 O-Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "odometry_projection_node.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace amr_sweeper_localization
{

OdometryProjectionNode::OdometryProjectionNode(const rclcpp::NodeOptions & options)
: Node("odometry_projection_node", options)
{
  input_odom_topic_ = declare_parameter("input_odom_topic",
      std::string("localization/odometry_body"));
  output_odom_topic_ = declare_parameter(
    "output_odom_topic", std::string("localization/odometry_fused"));
  odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
  body_frame_ = declare_parameter("body_frame", std::string("base_link"));
  projected_frame_ = declare_parameter("projected_frame", std::string("base_footprint"));
  publish_tf_ = declare_parameter("publish_tf", true);

  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, rclcpp::QoS(20));
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    input_odom_topic_,
    rclcpp::QoS(20),
    std::bind(&OdometryProjectionNode::handleOdometry, this, std::placeholders::_1));

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  RCLCPP_INFO(
    get_logger(),
    "Projecting body odometry '%s' (%s) to planar odometry '%s' (%s).",
    input_odom_topic_.c_str(),
    body_frame_.c_str(),
    output_odom_topic_.c_str(),
    projected_frame_.c_str());
}

double OdometryProjectionNode::yawToQuaternionZ(const double yaw)
{
  return std::sin(yaw * 0.5);
}

double OdometryProjectionNode::yawToQuaternionW(const double yaw)
{
  return std::cos(yaw * 0.5);
}

void OdometryProjectionNode::copyPlanarCovariance(
  const std::array<double, 36> & input,
  std::array<double, 36> * output)
{
  output->fill(0.0);

  constexpr std::array<int, 3> kept_axes{0, 1, 5};  // x, y, yaw
  for (const int row : kept_axes) {
    for (const int column : kept_axes) {
      (*output)[row * 6 + column] = input[row * 6 + column];
    }
  }

  constexpr double kProjectedAxisVariance = 1.0e6;
  (*output)[2 * 6 + 2] = kProjectedAxisVariance;
  (*output)[3 * 6 + 3] = kProjectedAxisVariance;
  (*output)[4 * 6 + 4] = kProjectedAxisVariance;
}

void OdometryProjectionNode::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  if (!message) {
    return;
  }

  if (!message->child_frame_id.empty() && message->child_frame_id != body_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Received odometry child_frame_id '%s', expected '%s'. Projecting anyway.",
      message->child_frame_id.c_str(),
      body_frame_.c_str());
  }

  tf2::Quaternion body_quaternion(
    message->pose.pose.orientation.x,
    message->pose.pose.orientation.y,
    message->pose.pose.orientation.z,
    message->pose.pose.orientation.w);

  if (body_quaternion.length2() < 1.0e-12) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Received odometry with invalid quaternion. Skipping projection.");
    return;
  }
  body_quaternion.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(body_quaternion).getRPY(roll, pitch, yaw);

  nav_msgs::msg::Odometry projected = *message;
  projected.header.frame_id = odom_frame_;
  projected.child_frame_id = projected_frame_;
  projected.pose.pose.position.z = 0.0;
  projected.pose.pose.orientation.x = 0.0;
  projected.pose.pose.orientation.y = 0.0;
  projected.pose.pose.orientation.z = yawToQuaternionZ(yaw);
  projected.pose.pose.orientation.w = yawToQuaternionW(yaw);
  projected.twist.twist.linear.z = 0.0;
  projected.twist.twist.angular.x = 0.0;
  projected.twist.twist.angular.y = 0.0;
  copyPlanarCovariance(message->pose.covariance, &projected.pose.covariance);
  copyPlanarCovariance(message->twist.covariance, &projected.twist.covariance);

  odom_publisher_->publish(projected);

  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = projected.header;
    transform.child_frame_id = projected_frame_;
    transform.transform.translation.x = projected.pose.pose.position.x;
    transform.transform.translation.y = projected.pose.pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = projected.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }
}

}  // namespace amr_sweeper_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sweeper_localization::OdometryProjectionNode>(
    rclcpp::NodeOptions{});
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
