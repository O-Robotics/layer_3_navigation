#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from sensor_msgs.msg import Imu


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z))
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class ImuFusionCoreBridge(Node):
    def __init__(self) -> None:
        super().__init__("imu_fusioncore_bridge")

        input_topic = self.declare_parameter("input_topic", "imu/data_raw").value
        imu_output_topic = self.declare_parameter(
            "imu_output_topic", "imu/data_fusioncore"
        ).value
        heading_output_topic = self.declare_parameter(
            "heading_output_topic", "imu/heading"
        ).value
        default_yaw_variance = float(
            self.declare_parameter("default_yaw_variance", 0.01).value
        )

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._default_yaw_variance = max(default_yaw_variance, 1.0e-6)
        self._imu_publisher = self.create_publisher(Imu, imu_output_topic, output_qos)
        self._heading_publisher = self.create_publisher(Imu, heading_output_topic, output_qos)
        self._subscription = self.create_subscription(Imu, input_topic, self._forward, input_qos)

        self.get_logger().info(
            f"Bridging IMU from '{input_topic}' to '{imu_output_topic}' "
            f"(gyro/accel only) and '{heading_output_topic}' (yaw-only heading)"
        )

    def _forward(self, msg: Imu) -> None:
        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        imu_msg.angular_velocity = msg.angular_velocity
        imu_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        imu_msg.linear_acceleration = msg.linear_acceleration
        imu_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._imu_publisher.publish(imu_msg)

        if msg.orientation_covariance[0] < 0.0:
            return

        yaw = yaw_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        heading_msg = Imu()
        heading_msg.header = msg.header
        heading_msg.orientation.x = qx
        heading_msg.orientation.y = qy
        heading_msg.orientation.z = qz
        heading_msg.orientation.w = qw
        yaw_variance = (
            msg.orientation_covariance[8]
            if msg.orientation_covariance[8] > 0.0
            else self._default_yaw_variance
        )
        heading_msg.orientation_covariance = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            yaw_variance,
        ]
        heading_msg.angular_velocity = msg.angular_velocity
        heading_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        heading_msg.linear_acceleration = msg.linear_acceleration
        heading_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._heading_publisher.publish(heading_msg)


def main() -> None:
    rclpy.init()
    node = ImuFusionCoreBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
