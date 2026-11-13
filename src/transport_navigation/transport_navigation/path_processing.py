"""Pure geometry processing, independent of ROS. Units: metres and radians."""

from dataclasses import dataclass
import math


class InvalidPath(ValueError):
    """The input cannot be represented by an orthogonal planar route."""


@dataclass(frozen=True)
class Segment:
    start: tuple
    end: tuple
    axis: int
    heading: float
    length: float
    has_previous_turn: bool
    turn_from_previous: float


@dataclass(frozen=True)
class Route:
    points: tuple
    segments: tuple
    total_length: float
    goal_yaw: float


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def direction(a, b):
    """Return +X/+Y/-X/-Y as 0/1/2/3; never snap diagonal input."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    if not math.isfinite(dx) or not math.isfinite(dy):
        raise InvalidPath('coordinate difference overflow')
    if dx == 0.0 and dy == 0.0:
        raise InvalidPath('zero-length edge')
    if dx != 0.0 and dy != 0.0:
        raise InvalidPath('diagonal edge: both x and y change')
    return (0 if dx > 0.0 else 2) if dy == 0.0 else (1 if dy > 0.0 else 3)


def process_points(points, goal_yaw, max_segments=2000):
    """Remove exact duplicates and same-direction collinear vertices only.

    Strict equality is intentional: our planner copies identical axis coordinates
    into adjacent points. Approximate snapping would change the collision-tested
    route. Very short edges and direction reversals are preserved.
    """
    if not math.isfinite(goal_yaw):
        raise InvalidPath('non-finite goal yaw')
    if max_segments < 1:
        raise InvalidPath('max_segments must be positive')
    corners = []
    for index, point in enumerate(points):
        if len(point) != 2 or not all(math.isfinite(v) for v in point):
            raise InvalidPath(f'point {index}: expected finite x/y')
        point = (float(point[0]), float(point[1]))
        if corners and point == corners[-1]:
            continue
        if corners:
            new_axis = direction(corners[-1], point)
            if len(corners) >= 2 and direction(corners[-2], corners[-1]) == new_axis:
                corners[-1] = point
                continue
        corners.append(point)
        if len(corners) - 1 > max_segments:
            raise InvalidPath(f'more than {max_segments} segments')
    if not corners:
        raise InvalidPath('empty path: previous route cleared')

    segments = []
    headings = (0.0, math.pi / 2, math.pi, -math.pi / 2)
    for a, b in zip(corners, corners[1:]):
        axis = direction(a, b)
        heading = headings[axis]
        length = abs(b[0] - a[0]) + abs(b[1] - a[1])
        previous_turn = wrap_angle(heading - segments[-1].heading) if segments else 0.0
        segments.append(Segment(a, b, axis, heading, length, bool(segments), previous_turn))
    try:
        total = math.fsum(segment.length for segment in segments)
    except OverflowError as exc:
        raise InvalidPath('total length overflow') from exc
    if not math.isfinite(total):
        raise InvalidPath('non-finite total length')
    return Route(tuple(corners), tuple(segments), total, wrap_angle(goal_yaw))


def process_path_message(path, frame='map', max_points=100000, max_segments=2000):
    """Validate a nav_msgs/Path-like object without importing ROS types."""
    if path.header.frame_id != frame:
        raise InvalidPath(f'expected frame {frame!r}, got {path.header.frame_id!r}')
    if not path.poses:
        raise InvalidPath('empty path: previous route cleared')
    if len(path.poses) > max_points:
        raise InvalidPath(f'more than {max_points} input points')
    points = []
    goal_yaw = 0.0
    for index, stamped in enumerate(path.poses):
        if stamped.header.frame_id not in ('', frame):
            raise InvalidPath(f'pose {index}: wrong coordinate frame')
        p, q = stamped.pose.position, stamped.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            raise InvalidPath(f'pose {index}: non-finite position/quaternion')
        if abs(p.z) > 1e-6:
            raise InvalidPath(f'pose {index}: non-planar z')
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if norm < 1e-12 or not math.isfinite(norm):
            raise InvalidPath(f'pose {index}: invalid quaternion norm')
        if abs(q.x / norm) > 1e-6 or abs(q.y / norm) > 1e-6:
            raise InvalidPath(f'pose {index}: non-planar orientation')
        points.append((p.x, p.y))
        # Read every orientation, so the final duplicate still retains goal yaw.
        goal_yaw = wrap_angle(2.0 * math.atan2(q.z / norm, q.w / norm))
    return process_points(points, goal_yaw, max_segments)
