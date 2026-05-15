#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix


class NavSatQosBridge(Node):
    def __init__(self) -> None:
        super().__init__("navsat_qos_bridge")

        self.declare_parameter("input_topic", "gnss/navsat")
        self.declare_parameter("output_topic", "gnss/navsat_reliable")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._publisher = self.create_publisher(NavSatFix, output_topic, pub_qos)
        self._subscription = self.create_subscription(
            NavSatFix,
            input_topic,
            self._forward,
            sub_qos,
        )

        self.get_logger().info(
            f"Bridging NavSatFix from '{input_topic}' (BEST_EFFORT) to "
            f"'{output_topic}' (RELIABLE)"
        )

    def _forward(self, msg: NavSatFix) -> None:
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavSatQosBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
