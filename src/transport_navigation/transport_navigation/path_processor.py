"""Convert the planner's Path into a validated route preview, never motor commands."""

import copy
import math
from uuid import uuid4

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from transport_interfaces.msg import ProcessedRoute, RouteSegment
from visualization_msgs.msg import Marker, MarkerArray

from transport_navigation.path_processing import InvalidPath, process_path_message


def point_message(xy):
    return Point(x=float(xy[0]), y=float(xy[1]), z=0.0)


def pose_message(xy, yaw):
    pose = Pose()
    pose.position = point_message(xy)
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class PathProcessor(Node):
    def __init__(self):
        super().__init__('path_processor')
        self.declare_parameter('max_input_points', 100000)
        self.declare_parameter('max_segments', 2000)
        self.max_input_points = self.get_parameter('max_input_points').value
        self.max_segments = self.get_parameter('max_segments').value
        if self.max_input_points < 1 or self.max_segments < 1:
            raise ValueError('max_input_points and max_segments must be positive')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.route_pub = self.create_publisher(ProcessedRoute, '/navigation/processed_route', qos)
        self.path_pub = self.create_publisher(Path, '/navigation/keypoint_path', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/navigation/keypoint_markers', qos)
        self.path_sub = self.create_subscription(Path, '/navigation/path', self.on_path, qos)
        self.publish_result(None, None, 'waiting for /navigation/path')
        self.get_logger().info('Path processor ready; geometry preview only, no motor output.')

    def on_path(self, msg):
        try:
            route = process_path_message(msg, max_points=self.max_input_points,
                                         max_segments=self.max_segments)
        except InvalidPath as exc:
            self.publish_result(msg, None, str(exc))
            self.get_logger().warning(f'Route cleared: {exc}')
            return
        self.publish_result(msg, route, 'geometry valid; execution and collision checks not performed')
        self.get_logger().info(
            f'{len(msg.poses)} input poses -> {len(route.points)} keypoints, '
            f'{len(route.segments)} segments, {route.total_length:.3f} m; '
            f'goal yaw {math.degrees(route.goal_yaw):.1f} deg')

    def publish_result(self, source, route, reason):
        result = ProcessedRoute()
        result.header.frame_id = 'map'
        result.header.stamp = self.get_clock().now().to_msg()
        if source is not None:
            result.source_stamp = copy.deepcopy(source.header.stamp)
        result.route_id = str(uuid4())
        result.valid = route is not None
        result.reason = reason
        preview = Path()
        preview.header = copy.deepcopy(result.header)

        if route is not None:
            result.total_length = route.total_length
            result.goal_yaw = route.goal_yaw
            for index, xy in enumerate(route.points):
                yaw = route.segments[index].heading if index < len(route.segments) else route.goal_yaw
                pose = pose_message(xy, yaw)
                result.waypoints.append(pose)
                stamped = PoseStamped()
                stamped.header = copy.deepcopy(result.header)
                stamped.pose = copy.deepcopy(pose)
                preview.poses.append(stamped)
            for index, item in enumerate(route.segments):
                segment = RouteSegment()
                segment.index = index
                segment.start = point_message(item.start)
                segment.end = point_message(item.end)
                segment.axis = item.axis
                segment.heading = item.heading
                segment.length = item.length
                segment.has_previous_turn = item.has_previous_turn
                segment.turn_from_previous = item.turn_from_previous
                result.segments.append(segment)

        # DELETEALL and new markers in ONE latched message: late RViz subscribers
        # get the complete scene, and invalid/shorter routes leave no stale labels.
        markers = MarkerArray()
        clear = Marker()
        clear.header = copy.deepcopy(result.header)
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        if route is not None:
            dots = Marker()
            dots.header = copy.deepcopy(result.header)
            dots.ns = 'keypoints'
            dots.id = 0
            dots.type = Marker.SPHERE_LIST
            dots.action = Marker.ADD
            dots.pose.orientation.w = 1.0
            dots.scale.x = dots.scale.y = dots.scale.z = 0.045
            dots.color.r, dots.color.g, dots.color.b, dots.color.a = 1.0, 0.45, 0.0, 1.0
            dots.points = [point_message(xy) for xy in route.points]
            markers.markers.append(dots)
            for index, xy in enumerate(route.points):
                label = Marker()
                label.header = copy.deepcopy(result.header)
                label.ns = 'keypoint_labels'
                label.id = index
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose = pose_message(xy, 0.0)
                label.pose.position.z = 0.10
                label.scale.z = 0.08
                label.color.r = label.color.g = label.color.b = label.color.a = 1.0
                label.text = f'P{index}'
                markers.markers.append(label)
        # Atomic executable data lives in result; other topics are only previews.
        self.route_pub.publish(result)
        self.path_pub.publish(preview)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PathProcessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
