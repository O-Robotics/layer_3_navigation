#include "slam_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <utility>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace amr_sweeper_mapping
{

namespace
{

constexpr char kDefaultFusionPoseTopic[] = "localization/pose";
constexpr char kDefaultSlamPoseTopic[] = "mapping/slam_toolbox/pose";
constexpr char kDefaultScanTopic[] = "depth_camera/scan";

}  // namespace

SlamNode::SlamNode()
: Node("amr_sweeper_slam_node"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  declare_parameter("slam_backend", std::string("slam_toolbox"));
  declare_parameter("map_frame", std::string("map"));
  declare_parameter("odom_frame", std::string("odom"));
  declare_parameter("base_frame", std::string("base_footprint"));
  declare_parameter("fusion_pose_topic", std::string(kDefaultFusionPoseTopic));
  declare_parameter("slam_pose_topic", std::string(kDefaultSlamPoseTopic));
  declare_parameter("scan_topic", std::string(kDefaultScanTopic));
  declare_parameter("seed_slam_from_fusion_pose", true);
  declare_parameter("seed_period_seconds", 2.0);
  declare_parameter("status_period_seconds", 2.0);
  declare_parameter("max_seed_publications", 5);

  backend_ = get_parameter("slam_backend").as_string();
  map_frame_ = get_parameter("map_frame").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  fusion_pose_topic_ = get_parameter("fusion_pose_topic").as_string();
  slam_pose_topic_ = get_parameter("slam_pose_topic").as_string();
  scan_topic_ = get_parameter("scan_topic").as_string();
  seed_slam_from_fusion_pose_ = get_parameter("seed_slam_from_fusion_pose").as_bool();
  max_seed_publications_ = static_cast<int>(get_parameter("max_seed_publications").as_int());

  fusion_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    fusion_pose_topic_,
    rclcpp::QoS(10),
    std::bind(&SlamNode::handleFusionPose, this, std::placeholders::_1));
  slam_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    slam_pose_topic_,
    rclcpp::QoS(10),
    std::bind(&SlamNode::handleSlamPose, this, std::placeholders::_1));
  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&SlamNode::handleScan, this, std::placeholders::_1));
  initial_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "navigation/slam/initialpose",
    rclcpp::QoS(10).reliable());
  status_publisher_ = create_publisher<std_msgs::msg::String>("mapping/slam_toolbox/status", 10);

  seed_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("seed_period_seconds").as_double()),
    std::bind(&SlamNode::maybeSeedInitialPose, this));
  status_timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("status_period_seconds").as_double()),
    std::bind(&SlamNode::publishStatus, this));

  RCLCPP_INFO(
    get_logger(),
    "SLAM supervision node configured for backend '%s' using scan topic '%s'.",
    backend_.c_str(),
    scan_topic_.c_str());
}

void SlamNode::handleFusionPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
  latest_fusion_pose_ = *message;
  latest_fusion_pose_frame_id_ = message->header.frame_id;
  have_fusion_pose_ = true;
}

void SlamNode::handleScan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  last_scan_frame_id_ = message->header.frame_id;
  last_scan_time_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() :
    rclcpp::Time(message->header.stamp);
  have_scan_ = true;
}

void SlamNode::handleSlamPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
  last_slam_pose_time_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
    now() :
    rclcpp::Time(message->header.stamp);
  have_slam_pose_ = true;
}

void SlamNode::maybeSeedInitialPose()
{
  if (!seed_slam_from_fusion_pose_ || backend_ != "slam_toolbox") {
    return;
  }

  const bool have_map_to_odom_tf =
    tf_buffer_._frameExists(map_frame_) &&
    tf_buffer_._frameExists(odom_frame_) &&
    tf_buffer_.canTransform(
    map_frame_,
    odom_frame_,
    tf2::TimePointZero,
    tf2::durationFromSec(0.05));
  if (have_map_to_odom_tf && have_slam_pose_) {
    return;
  }

  if (seed_publications_ >= max_seed_publications_ || !have_fusion_pose_) {
    return;
  }

  if (!seed_pose_ready_) {
    if (!buildSeedPose(frozen_seed_pose_)) {
      return;
    }
    seed_pose_ready_ = true;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose = frozen_seed_pose_;
  initial_pose.header.stamp = now();
  initial_pose_publisher_->publish(initial_pose);
  ++seed_publications_;

  RCLCPP_INFO(
    get_logger(),
    "Published frozen SLAM seed pose %d/%d from FusionCore onto navigation/slam/initialpose in frame '%s'.",
    seed_publications_,
    max_seed_publications_,
    initial_pose.header.frame_id.c_str());
}

bool SlamNode::buildSeedPose(geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose)
{
  if (latest_fusion_pose_frame_id_.empty()) {
    if (!warned_about_seed_frame_mismatch_) {
      warned_about_seed_frame_mismatch_ = true;
      RCLCPP_WARN(
        get_logger(),
        "Skipping SLAM seed pose because FusionCore pose arrived without a frame_id.");
    }
    return false;
  }

  initial_pose = latest_fusion_pose_;
  initial_pose.header.stamp = now();
  initial_pose.header.frame_id = map_frame_;

  if (latest_fusion_pose_frame_id_ == map_frame_) {
    return true;
  }

  geometry_msgs::msg::PoseStamped source_pose;
  source_pose.header = latest_fusion_pose_.header;
  source_pose.pose = latest_fusion_pose_.pose.pose;
  geometry_msgs::msg::PoseStamped projected_pose;

  try {
    const auto transform = tf_buffer_.lookupTransform(
      map_frame_,
      latest_fusion_pose_frame_id_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.05));
    tf2::doTransform(source_pose, projected_pose, transform);
    initial_pose.pose.pose = projected_pose.pose;
    return true;
  } catch (const tf2::TransformException &) {
    if (latest_fusion_pose_frame_id_ != odom_frame_) {
      if (!warned_about_seed_frame_mismatch_) {
        warned_about_seed_frame_mismatch_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Skipping SLAM seed pose because FusionCore pose is in frame '%s' and no transform into '%s' is available yet.",
          latest_fusion_pose_frame_id_.c_str(),
          map_frame_.c_str());
      }
      return false;
    }
  }

  if (!warned_about_seed_projection_) {
    warned_about_seed_projection_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Seeding SLAM from FusionCore odom pose before map -> odom exists; assuming startup map and odom are initially aligned.");
  }

  return true;
}

void SlamNode::publishStatus()
{
  std_msgs::msg::String message;
  const bool ready = have_scan_ && have_fusion_pose_;
  const bool tracking = have_slam_pose_;
  bool have_scan_to_base_tf = false;
  bool have_map_to_odom_tf = false;
  bool have_base_frame = false;
  bool have_scan_frame = false;

  if (tf_buffer_._frameExists(base_frame_)) {
    have_base_frame = true;
  }

  if (have_scan_ && !last_scan_frame_id_.empty()) {
    have_scan_frame = tf_buffer_._frameExists(last_scan_frame_id_);
  }

  if (have_scan_ && have_base_frame && have_scan_frame && !last_scan_frame_id_.empty()) {
    have_scan_to_base_tf = tf_buffer_.canTransform(
      base_frame_,
      last_scan_frame_id_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.05));
  }

  if (tf_buffer_._frameExists(map_frame_) && tf_buffer_._frameExists(odom_frame_)) {
    have_map_to_odom_tf = tf_buffer_.canTransform(
      map_frame_,
      odom_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.05));
  }

  std::ostringstream stream;
  stream
    << "ready=" << (ready ? "true" : "false")
    << "; tracking=" << (tracking ? "true" : "false")
    << "; scan_tf=" << (have_scan_to_base_tf ? "true" : "false")
    << "; base_frame=" << (have_base_frame ? "true" : "false")
    << "; scan_frame_exists=" << (have_scan_frame ? "true" : "false")
    << "; map_tf=" << (have_map_to_odom_tf ? "true" : "false")
    << "; scan_frame=" << (last_scan_frame_id_.empty() ? "<none>" : last_scan_frame_id_)
    << "; pose_frame=" <<
    (latest_fusion_pose_frame_id_.empty() ? "<none>" : latest_fusion_pose_frame_id_)
    << "; seeds=" << seed_publications_;
  message.data = stream.str();
  status_publisher_->publish(message);
}

}  // namespace amr_sweeper_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_mapping::SlamNode>());
  rclcpp::shutdown();
  return 0;
}
