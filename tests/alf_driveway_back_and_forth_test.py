#!/usr/bin/env python3

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import NavSatFix


def latlon_to_utm(latitude: float, longitude: float) -> Tuple[float, float, int, str]:
    a = 6378137.0
    f = 1 / 298.257223563
    k0 = 0.9996
    e2 = f * (2 - f)
    ep2 = e2 / (1 - e2)

    zone = int((longitude + 180.0) / 6.0) + 1
    hemisphere = 'N' if latitude >= 0.0 else 'S'

    lat_rad = math.radians(latitude)
    lon_rad = math.radians(longitude)
    lon0_rad = math.radians((zone - 1) * 6 - 180 + 3)

    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    tan_lat = math.tan(lat_rad)

    n = a / math.sqrt(1 - e2 * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = ep2 * cos_lat * cos_lat
    a_term = cos_lat * (lon_rad - lon0_rad)

    m = a * (
        (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * lat_rad
        - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * math.sin(2 * lat_rad)
        + (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * math.sin(4 * lat_rad)
        - (35 * e2 * e2 * e2 / 3072) * math.sin(6 * lat_rad)
    )

    easting = k0 * n * (
        a_term
        + (1 - t + c) * a_term**3 / 6
        + (5 - 18 * t + t * t + 72 * c - 58 * ep2) * a_term**5 / 120
    ) + 500000.0

    northing = k0 * (
        m
        + n
        * tan_lat
        * (
            a_term * a_term / 2
            + (5 - t + 9 * c + 4 * c * c) * a_term**4 / 24
            + (61 - 58 * t + t * t + 600 * c - 330 * ep2) * a_term**6 / 720
        )
    )

    if latitude < 0.0:
        northing += 10000000.0

    return easting, northing, zone, hemisphere


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def resolve_geojson_path(path_arg: str | None) -> Path:
    if path_arg:
        return Path(path_arg).expanduser().resolve()
    return Path(__file__).with_name('Alf_Driveway.geojson').resolve()


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


def build_pose_sequence(
    node: Node,
    coordinates: Sequence[Tuple[float, float]],
    frame_id: str,
    map_origin_utm: Tuple[float, float] | None = None,
) -> List[PoseStamped]:
    poses: List[PoseStamped] = []
    utm_points: List[Tuple[float, float]] = []
    zones = set()

    for longitude, latitude in coordinates:
        easting, northing, zone, hemisphere = latlon_to_utm(latitude, longitude)
        utm_points.append((easting, northing))
        zones.add((zone, hemisphere))

    if len(zones) != 1:
        raise ValueError(f'GeoJSON spans multiple UTM zones: {sorted(zones)}')

    zone, hemisphere = next(iter(zones))
    node.get_logger().info(f'Loaded {len(utm_points)} route points in UTM zone {zone}{hemisphere}.')
    first_easting, first_northing = utm_points[0]
    last_easting, last_northing = utm_points[-1]
    node.get_logger().info(
        'Raw UTM route endpoints: '
        f'start=({first_easting:.2f}, {first_northing:.2f}) '
        f'end=({last_easting:.2f}, {last_northing:.2f})'
    )

    if map_origin_utm is not None:
        origin_easting, origin_northing = map_origin_utm
        node.get_logger().info(
            'Converting UTM route into local map coordinates using '
            f'origin ({origin_easting:.2f}, {origin_northing:.2f}).'
        )

    for index, (x, y) in enumerate(utm_points):
        if map_origin_utm is not None:
            x -= origin_easting
            y -= origin_northing

        if index < len(utm_points) - 1:
            next_x, next_y = utm_points[index + 1]
            if map_origin_utm is not None:
                next_x -= origin_easting
                next_y -= origin_northing
            yaw = math.atan2(next_y - y, next_x - x)
        else:
            prev_x, prev_y = utm_points[index - 1]
            if map_origin_utm is not None:
                prev_x -= origin_easting
                prev_y -= origin_northing
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


def namespace_join(namespace: str, topic_or_action: str) -> str:
    ns = namespace.strip('/')
    leaf = topic_or_action.lstrip('/')
    if not ns:
        return f'/{leaf}'
    return f'/{ns}/{leaf}'


def wait_for_navsat_origin(node: Node, topic: str, timeout_sec: float) -> Tuple[float, float]:
    navsat_msg: NavSatFix | None = None

    def on_navsat(msg: NavSatFix) -> None:
        nonlocal navsat_msg
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            navsat_msg = msg

    subscription = node.create_subscription(NavSatFix, topic, on_navsat, 10)
    try:
        node.get_logger().info(f'Waiting for NavSatFix on {topic} to derive local map origin...')
        deadline = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and navsat_msg is None and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)

        if navsat_msg is None:
            raise RuntimeError(f'Timed out waiting for NavSatFix on {topic}')

        easting, northing, zone, hemisphere = latlon_to_utm(navsat_msg.latitude, navsat_msg.longitude)
        node.get_logger().info(
            'Derived local map origin from NavSatFix: '
            f'lat={navsat_msg.latitude:.9f} lon={navsat_msg.longitude:.9f} '
            f'-> UTM ({easting:.2f}, {northing:.2f}) zone {zone}{hemisphere}'
        )
        return easting, northing
    finally:
        node.destroy_subscription(subscription)


class DrivewayBackAndForthTester(Node):
    def __init__(
        self,
        args: argparse.Namespace,
        geojson_path: Path,
        forward_route: Sequence[PoseStamped],
        reverse_route: Sequence[PoseStamped],
    ):
        super().__init__('alf_driveway_back_and_forth_test')
        self.args = args
        self.geojson_path = geojson_path
        self.forward_route = list(forward_route)
        self.reverse_route = list(reverse_route)
        self.brush_command = Twist()
        self.brush_command.linear.x = args.brush_linear
        self.brush_command.angular.z = args.brush_angular
        self.follow_waypoints_action = namespace_join(args.namespace, 'follow_waypoints')
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

    def publish_brush_command(self) -> None:
        if self.brush_enabled:
            self.brush_publisher.publish(self.brush_command)

    def set_brushes_enabled(self, enabled: bool) -> None:
        self.brush_enabled = enabled
        if not enabled:
            stop_msg = Twist()
            self.brush_publisher.publish(stop_msg)

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

    def cancel_active_goal(self) -> None:
        if self.active_goal_handle is None:
            return
        cancel_future = self.active_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
        self.active_goal_handle = None

    def run(self) -> None:
        self.wait_for_nav_server()
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
        description='Run the AMR Sweeper back and forth along the Alf_Driveway GeoJSON line while keeping the brushes on.',
    )
    parser.add_argument(
        '--geojson',
        default=None,
        help='Path to the GeoJSON line to follow. Defaults to Alf_Driveway.geojson next to this script.',
    )
    parser.add_argument(
        '--namespace',
        default='amr_sweeper',
        help='ROS namespace that hosts Nav2 and the brush command topic.',
    )
    parser.add_argument(
        '--frame-id',
        default='map',
        help='Target frame for the generated waypoint poses.',
    )
    parser.add_argument(
        '--map-origin-easting',
        type=float,
        default=None,
        help='Optional UTM easting for the local map frame origin. If omitted with --frame-id map, the script derives it from NavSatFix.',
    )
    parser.add_argument(
        '--map-origin-northing',
        type=float,
        default=None,
        help='Optional UTM northing for the local map frame origin. If omitted with --frame-id map, the script derives it from NavSatFix.',
    )
    parser.add_argument(
        '--navsat-topic',
        default='navsat',
        help='NavSatFix topic used to derive the local map origin when no explicit UTM origin is provided.',
    )
    parser.add_argument(
        '--navsat-timeout',
        type=float,
        default=10.0,
        help='Seconds to wait for a NavSatFix sample when deriving the local map origin.',
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
        default=0.5,
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
    if (args.map_origin_easting is None) != (args.map_origin_northing is None):
        parser.error('--map-origin-easting and --map-origin-northing must be provided together')
    if args.navsat_timeout <= 0.0:
        parser.error('--navsat-timeout must be > 0')
    return args


def main() -> int:
    ros_argv = remove_ros_args(args=sys.argv)
    args = parse_args(ros_argv[1:])
    geojson_path = resolve_geojson_path(args.geojson)
    coordinates = load_linestring_points(geojson_path)

    rclpy.init(args=sys.argv)
    route_loader = Node('alf_driveway_route_loader')
    try:
        map_origin_utm = None
        if args.frame_id == 'map':
            if args.map_origin_easting is not None and args.map_origin_northing is not None:
                map_origin_utm = (args.map_origin_easting, args.map_origin_northing)
            else:
                map_origin_utm = wait_for_navsat_origin(
                    route_loader,
                    namespace_join(args.namespace, args.navsat_topic),
                    args.navsat_timeout,
                )
        forward_route = build_pose_sequence(route_loader, coordinates, args.frame_id, map_origin_utm)
        reverse_route = build_pose_sequence(route_loader, list(reversed(coordinates)), args.frame_id, map_origin_utm)
    finally:
        route_loader.destroy_node()

    node = None
    try:
        node = DrivewayBackAndForthTester(args, geojson_path, forward_route, reverse_route)
        node.get_logger().info(f'Using GeoJSON route: {node.geojson_path}')
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
