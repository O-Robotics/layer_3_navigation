#!/usr/bin/env python3

import sys
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class Layer3SystemCheck(Node):
    def __init__(self) -> None:
        super().__init__("layer_3_system_check")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("action_name", "follow_waypoints")
        self.declare_parameter("passed_topic", "layer_3/system_check_passed")
        self.declare_parameter("action_wait_timeout_sec", 45.0)
        self.declare_parameter("tf_wait_timeout_sec", 45.0)
        self.declare_parameter("goal_result_timeout_sec", 30.0)
        self.declare_parameter("success_publish_period_sec", 1.0)

        self.global_frame = self.get_parameter("global_frame").get_parameter_value().string_value
        self.robot_frame = self.get_parameter("robot_frame").get_parameter_value().string_value
        self.action_name = self.get_parameter("action_name").get_parameter_value().string_value
        self.passed_topic = self.get_parameter("passed_topic").get_parameter_value().string_value
        self.action_wait_timeout = Duration(
            seconds=self.get_parameter("action_wait_timeout_sec").get_parameter_value().double_value
        )
        self.tf_wait_timeout = Duration(
            seconds=self.get_parameter("tf_wait_timeout_sec").get_parameter_value().double_value
        )
        self.goal_result_timeout = Duration(
            seconds=self.get_parameter("goal_result_timeout_sec").get_parameter_value().double_value
        )
        self.success_publish_period = max(
            0.1,
            self.get_parameter("success_publish_period_sec").get_parameter_value().double_value,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, FollowWaypoints, self.action_name)

        self.start_time = self.get_clock().now()
        self.goal_send_time: Optional[rclpy.time.Time] = None
        self.goal_result_future = None
        self.goal_response_future = None
        self.success_publisher = None
        self.success_timer = None
        self.failed = False
        self.completed = False
        self.goal_inflight = False

        self.tick_timer = self.create_timer(0.5, self._tick)

    def _tick(self) -> None:
        if self.failed or self.completed:
            return

        now = self.get_clock().now()

        if not self.goal_inflight:
            if now - self.start_time > self.action_wait_timeout:
                self._fail(
                    f"Timed out waiting for layer 3 navigation action server '{self.action_name}' to become available."
                )
                return

            if not self.action_client.wait_for_server(timeout_sec=0.0):
                return

            pose = self._lookup_current_pose()
            if pose is None:
                if now - self.start_time > self.tf_wait_timeout:
                    self._fail(
                        f"Timed out looking up current pose in frame '{self.global_frame}' from '{self.robot_frame}'."
                    )
                return

            goal = FollowWaypoints.Goal()
            goal.poses.append(pose)

            self.goal_send_time = now
            self.goal_inflight = True
            self.goal_response_future = self.action_client.send_goal_async(goal)
            self.goal_response_future.add_done_callback(self._handle_goal_response)
            self.get_logger().info(
                "Dispatching layer 3 system-check goal using the robot's current pose."
            )
            return

        if self.goal_send_time is not None and now - self.goal_send_time > self.goal_result_timeout:
            self._fail("Timed out waiting for layer 3 system-check goal result.")

    def _lookup_current_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = self.global_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _handle_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._fail(f"Layer 3 system-check goal request failed: {exc}")
            return

        if not goal_handle.accepted:
            self._fail("Layer 3 system-check goal was rejected.")
            return

        self.goal_result_future = goal_handle.get_result_async()
        self.goal_result_future.add_done_callback(self._handle_goal_result)

    def _handle_goal_result(self, future) -> None:
        try:
            wrapped_result = future.result()
        except Exception as exc:  # noqa: BLE001
            self._fail(f"Layer 3 system-check result retrieval failed: {exc}")
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(
                "Layer 3 system-check goal did not succeed "
                f"(status={wrapped_result.status})."
            )
            return

        self.completed = True
        self.success_publisher = self.create_publisher(
            Bool,
            self.passed_topic,
            1,
        )
        self.success_timer = self.create_timer(
            self.success_publish_period,
            self._publish_success,
        )
        self._publish_success()
        self.get_logger().info("Layer 3 system check passed.")

    def _publish_success(self) -> None:
        if self.success_publisher is None:
            return
        message = Bool()
        message.data = True
        self.success_publisher.publish(message)

    def _fail(self, message: str) -> None:
        if self.failed or self.completed:
            return
        self.failed = True
        self.get_logger().error(message)
        rclpy.shutdown()


def main() -> int:
    rclpy.init()
    node = Layer3SystemCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        exit_code = 1 if node.failed else 0
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
