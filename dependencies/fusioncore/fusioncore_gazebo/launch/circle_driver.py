#!/usr/bin/env python3
# Publishes constant cmd_vel to drive the robot in a circle.
# Waits start_delay_s before publishing so FusionCore can initialize first.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircleDriver(Node):
    def __init__(self):
        super().__init__("circle_driver")
        self.declare_parameter("linear_speed",  0.8)   # m/s
        self.declare_parameter("radius",        12.0)  # m
        self.declare_parameter("start_delay_s", 18.0)  # s: wait for FC to initialize

        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._start_ns = self.get_clock().now().nanoseconds
        self._moving = False

        # 20 Hz tick: checks delay, then publishes
        self._timer = self.create_timer(0.05, self._tick)
        self.get_logger().info("circle_driver ready, waiting for start delay...")

    def _tick(self):
        delay = self.get_parameter("start_delay_s").get_parameter_value().double_value
        elapsed = (self.get_clock().now().nanoseconds - self._start_ns) * 1e-9

        if elapsed < delay:
            return

        if not self._moving:
            self._moving = True
            self.get_logger().info("circle_driver: starting circular motion")

        v = self.get_parameter("linear_speed").get_parameter_value().double_value
        r = self.get_parameter("radius").get_parameter_value().double_value

        msg = Twist()
        msg.linear.x  = v
        msg.angular.z = v / r
        self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(CircleDriver())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
