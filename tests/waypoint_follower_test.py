#!/usr/bin/env python3

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import List, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from fusioncore_ros.srv import FromLL
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def resolve_geojson_path(path_arg: str | None) -> Path:
    if path_arg:
        return Path(path_arg).expanduser().resolve()
    raise ValueError('A GeoJSON file path is required. Pass it with --geojson.')


def load_linestring_points(path: Path) -> List[Tuple[float, float]]:
    document = json.loads(path.read_text(encoding='utf-8'))
    features = document.get('features', [])
    for feature in features:
        geometry = feature.get('geometry', {})
        if geometry.get('type') == 'LineString':
            coordinates = geometry.get('coordinates', [])
            if len(coordinates) < 2:
                raise ValueError('GeoJSON LineString must contain at least two coordinates.')
            return [(float(lon), float(lat)) for lon, lat in coordinates]
    raise ValueError('No LineString geometry found in the GeoJSON file.')


def convert_geojson_points_with_fusioncore(
    node: Node,
    coordinates: Sequence[Tuple[float, float]],
    service_name: str,
    timeout_sec: float,
) -> List[Tuple[float, float]]:
    client = node.create_client(FromLL, service_name)
    node.get_logger().info(f'Waiting for FusionCore conversion service {service_name}...')
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f'Timed out waiting for FusionCore service {service_name}')

    converted_points: List[Tuple[float, float]] = []
    for index, (longitude, latitude) in enumerate(coordinates, start=1):
        request = FromLL.Request()
        request.ll_point.latitude = latitude
        request.ll_point.longitude = longitude
        request.ll_point.altitude = 0.0
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        response = future.result()
        if response is None:
            raise RuntimeError(f'FusionCore conversion failed for waypoint {index}')
        converted_points.append((float(response.map_point.x), float(response.map_point.y)))

    node.get_logger().info(
        f'Converted {len(converted_points)} GeoJSON point(s) through FusionCore service {service_name}.'
    )
    return converted_points


def build_pose_sequence(
    node: Node,
    xy_points: Sequence[Tuple[float, float]],
    frame_id: str,
) -> List[PoseStamped]:
    poses: List[PoseStamped] = []
    for index, (x, y) in enumerate(xy_points):
        if index < len(xy_points) - 1:
            next_x, next_y = xy_points[index + 1]
            yaw = math.atan2(next_y - y, next_x - x)
        else:
            prev_x, prev_y = xy_points[index - 1]
            yaw = math.atan2(y - prev_y, x - prev_x)

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        poses.append(pose)

    first_pose = poses[0].pose.position
    last_pose = poses[-1].pose.position
    node.get_logger().info(
        f'Route endpoints in frame "{frame_id}": '
        f'start=({first_pose.x:.2f}, {first_pose.y:.2f}) '
        f'end=({last_pose.x:.2f}, {last_pose.y:.2f})'
    )

    return poses


def trim_duplicate_endpoint(poses: Sequence[PoseStamped]) -> List[PoseStamped]:
    if len(poses) <= 1:
        return list(poses)
    return list(poses[1:])


def make_color(r: float, g: float, b: float, a: float) -> ColorRGBA:
    color = ColorRGBA()
    color.r = r
    color.g = g
    color.b = b
    color.a = a
    return color


def namespace_join(namespace: str, topic_or_action: str) -> str:
    ns = namespace.strip('/')
    leaf = topic_or_action.lstrip('/')
    if not ns:
        return f'/{leaf}'
    return f'/{ns}/{leaf}'


class WaypointFollowerTester(Node):
    def __init__(
        self,
        args: argparse.Namespace,
        geojson_path: Path,
        forward_route: Sequence[PoseStamped],
        reverse_route: Sequence[PoseStamped],
    ):
        super().__init__('waypoint_follower_test')
        self.args = args
        self.geojson_path = geojson_path
        self.forward_route = list(forward_route)
        self.reverse_route = list(reverse_route)
        self.brush_command = Twist()
        self.brush_command.linear.x = args.brush_linear
        self.brush_command.angular.z = args.brush_angular
        self.route_frame_id = self.forward_route[0].header.frame_id if self.forward_route else args.frame_id
        self.follow_waypoints_action = namespace_join(args.namespace, 'follow_waypoints')
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.route_marker_publisher = self.create_publisher(
            Marker,
            namespace_join(args.namespace, 'waypoint_test/route_marker'),
            path_qos,
        )
        self.next_waypoint_publisher = self.create_publisher(
            Marker,
            namespace_join(args.namespace, 'waypoint_test/next_waypoint'),
            path_qos,
        )
        self.brush_publisher = self.create_publisher(
            Twist,
            namespace_join(args.namespace, 'cmd_vel_joy_brushes'),
            10,
        )
        self.brush_timer = self.create_timer(1.0 / args.brush_publish_hz, self.publish_brush_command)
        self.brush_enabled = False
        self.action_client = ActionClient(
            self,
            FollowWaypoints,
            self.follow_waypoints_action,
        )
        self.active_goal_handle = None
        self.publish_debug_paths()

    def publish_brush_command(self) -> None:
        if self.brush_enabled:
            self.brush_publisher.publish(self.brush_command)

    def publish_debug_paths(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.route_marker_publisher.publish(self.build_route_marker(stamp))

    def build_route_marker(self, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.route_frame_id
        marker.header.stamp = stamp
        marker.ns = 'waypoint_test'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.color = make_color(0.3, 0.7, 1.0, 0.95)
        marker.points = [pose.pose.position for pose in self.forward_route]
        return marker

    def publish_next_waypoint_marker(self, route_name: str, poses: Sequence[PoseStamped]) -> None:
        stamp = self.get_clock().now().to_msg()
        marker = Marker()
        marker.header.frame_id = self.route_frame_id
        marker.header.stamp = stamp
        marker.ns = 'waypoint_test'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color = make_color(1.0, 0.2, 0.2, 0.95)
        if poses:
            marker.pose = poses[0].pose
            self.get_logger().info(
                f'Publishing next waypoint marker for {route_name} at '
                f'({marker.pose.position.x:.2f}, {marker.pose.position.y:.2f}) in {self.route_frame_id}.'
            )
        else:
            marker.action = Marker.DELETE
        self.next_waypoint_publisher.publish(marker)

    def set_brushes_enabled(self, enabled: bool) -> None:
        self.brush_enabled = enabled
        if not enabled:
            self.stop_brushes()

    def stop_brushes(self) -> None:
        stop_msg = Twist()
        for _ in range(3):
            self.brush_publisher.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def wait_for_nav_server(self) -> None:
        self.get_logger().info(f'Waiting for action server {self.follow_waypoints_action}...')
        while rclpy.ok() and not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waypoint follower action server not available yet.')
        if not rclpy.ok():
            raise RuntimeError('ROS shutdown requested before the waypoint follower action server became available.')

    def execute_route(self, route_name: str, poses: Sequence[PoseStamped]) -> None:
        if not poses:
            self.get_logger().warn(f'Skipping empty route: {route_name}')
            return

        self.publish_next_waypoint_marker(route_name, poses)
        goal = FollowWaypoints.Goal()
        now = self.get_clock().now().to_msg()
        for pose in poses:
            pose.header.stamp = now
            goal.poses.append(pose)

        self.get_logger().info(f'Sending {route_name} route with {len(goal.poses)} waypoint(s).')
        send_goal_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.active_goal_handle = send_goal_future.result()

        if self.active_goal_handle is None or not self.active_goal_handle.accepted:
            raise RuntimeError(f'{route_name} route was rejected by Nav2.')

        result_future = self.active_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            raise RuntimeError(f'{route_name} route did not return a result.')

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'{route_name} route failed with goal status {result.status}.')

        missed = list(result.result.missed_waypoints)
        if missed:
            raise RuntimeError(f'{route_name} route missed waypoint indices: {missed}')

        self.get_logger().info(f'{route_name} route completed successfully.')
        self.active_goal_handle = None
        self.publish_next_waypoint_marker(route_name, [])

    def cancel_active_goal(self) -> None:
        if self.active_goal_handle is None:
            return
        cancel_future = self.active_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
        self.active_goal_handle = None

    def run(self) -> None:
        self.wait_for_nav_server()
        self.publish_debug_paths()
        self.set_brushes_enabled(True)

        forward_initial = self.forward_route
        forward_repeat = trim_duplicate_endpoint(self.forward_route)
        reverse_repeat = trim_duplicate_endpoint(self.reverse_route)

        if self.args.round_trips == 0:
            trip_index = 1
            self.execute_route('forward trip 1', forward_initial)
            self.execute_route('reverse trip 1', reverse_repeat)
            trip_index += 1
            while rclpy.ok():
                self.execute_route(f'forward trip {trip_index}', forward_repeat)
                self.execute_route(f'reverse trip {trip_index}', reverse_repeat)
                trip_index += 1
            return

        self.execute_route('forward trip 1', forward_initial)
        self.execute_route('reverse trip 1', reverse_repeat)

        for trip_index in range(2, self.args.round_trips + 1):
            self.execute_route(f'forward trip {trip_index}', forward_repeat)
            self.execute_route(f'reverse trip {trip_index}', reverse_repeat)


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run the AMR Sweeper back and forth along a GeoJSON line while keeping the brushes on.',
    )
    parser.add_argument(
        '--geojson',
        required=True,
        help='Path to the GeoJSON line to follow.',
    )
    parser.add_argument(
        '--namespace',
        default='amr_sweeper',
        help='ROS namespace that hosts Nav2 and the brush command topic.',
    )
    parser.add_argument(
        '--frame-id',
        default='odom',
        help='Target frame for the generated waypoint poses. Use the same local frame FusionCore publishes to Nav2.',
    )
    parser.add_argument(
        '--fromll-service',
        default='/fromLL',
        help='FusionCore service used to convert WGS84 GeoJSON coordinates into the local navigation frame.',
    )
    parser.add_argument(
        '--fromll-timeout',
        type=float,
        default=10.0,
        help='Seconds to wait for the FusionCore /fromLL service and each waypoint conversion response.',
    )
    parser.add_argument(
        '--round-trips',
        type=int,
        default=1,
        help='Number of there-and-back cycles to run. Use 0 to keep looping until interrupted.',
    )
    parser.add_argument(
        '--brush-linear',
        type=float,
        default=0.1,
        help='Brush command linear.x value. The layer 2 tool controller maps this into both brush motors.',
    )
    parser.add_argument(
        '--brush-angular',
        type=float,
        default=0.0,
        help='Brush command angular.z value. Keep 0.0 to drive both brushes equally.',
    )
    parser.add_argument(
        '--brush-publish-hz',
        type=float,
        default=10.0,
        help='How often to republish the brush command while the route is running.',
    )

    args = parser.parse_args(argv)
    if args.round_trips < 0:
        parser.error('--round-trips must be >= 0')
    if args.brush_publish_hz <= 0.0:
        parser.error('--brush-publish-hz must be > 0')
    if args.fromll_timeout <= 0.0:
        parser.error('--fromll-timeout must be > 0')
    return args


def main() -> int:
    ros_argv = remove_ros_args(args=sys.argv)
    args = parse_args(ros_argv[1:])
    geojson_path = resolve_geojson_path(args.geojson)
    coordinates = load_linestring_points(geojson_path)

    rclpy.init(args=sys.argv)
    route_loader = Node('waypoint_follower_route_loader')
    try:
        converted_points = convert_geojson_points_with_fusioncore(
            route_loader,
            coordinates,
            args.fromll_service,
            args.fromll_timeout,
        )
        forward_route = build_pose_sequence(
            route_loader,
            converted_points,
            args.frame_id,
        )
        reverse_route = build_pose_sequence(
            route_loader,
            list(reversed(converted_points)),
            args.frame_id,
        )
    finally:
        route_loader.destroy_node()

    node = None
    try:
        node = WaypointFollowerTester(args, geojson_path, forward_route, reverse_route)
        node.get_logger().info(f'Using GeoJSON route: {node.geojson_path}')
        node.get_logger().info(
            'Publishing route debug topics on '
            f'{namespace_join(args.namespace, "waypoint_test/route_marker")} and '
            f'{namespace_join(args.namespace, "waypoint_test/next_waypoint")}.'
        )
        node.run()
        return 0
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('Interrupted by user. Cancelling any active route and stopping brushes.')
            node.cancel_active_goal()
        return 130
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
            node.cancel_active_goal()
        else:
            print(str(exc), file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.set_brushes_enabled(False)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
