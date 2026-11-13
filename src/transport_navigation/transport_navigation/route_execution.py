"""ROS-independent route validation, coordinate conversion and motion sequencing."""
from dataclasses import dataclass
import hashlib
import math


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('non-finite quaternion')
    norm = math.hypot(*values)
    if norm < 1e-9 or abs(q.x / norm) > 1e-6 or abs(q.y / norm) > 1e-6:
        raise ValueError('expected nonzero planar quaternion')
    return wrap(2 * math.atan2(q.z / norm, q.w / norm))


def point_of(p):
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or abs(p.z) > 1e-6:
        raise ValueError('expected finite planar position')
    return float(p.x), float(p.y)


def to_map(pose, transform):
    x, y, yaw = pose
    tx, ty, angle = transform
    c, s = math.cos(angle), math.sin(angle)
    return tx + c*x - s*y, ty + s*x + c*y, wrap(angle + yaw)


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def line_error(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx*dx + dy*dy
    t = max(0.0, min(1.0, ((p[0]-a[0])*dx + (p[1]-a[1])*dy) / length2)) if length2 else 0.0
    return distance(p, (a[0] + t*dx, a[1] + t*dy))


@dataclass(frozen=True)
class Route:
    route_id: str
    points: tuple
    headings: tuple
    lengths: tuple
    goal_yaw: float


def validate_route(msg, max_segments=2000, max_length=10.0):
    if not msg.valid or not msg.route_id or msg.header.frame_id != 'map':
        raise ValueError('invalid route or non-map frame')
    if len(msg.segments) > max_segments or len(msg.waypoints) != len(msg.segments) + 1:
        raise ValueError('waypoint/segment count mismatch')
    points = tuple(point_of(p.position) for p in msg.waypoints)
    for p in msg.waypoints:
        yaw_of(p.orientation)
    if not math.isfinite(msg.goal_yaw) or not math.isfinite(msg.total_length):
        raise ValueError('non-finite route metadata')
    headings, lengths = [], []
    for i, segment in enumerate(msg.segments):
        a, b = points[i:i+2]
        dx, dy = b[0]-a[0], b[1]-a[1]
        if (dx == 0) == (dy == 0):
            raise ValueError('zero-length or diagonal segment')
        if distance(point_of(segment.start), a) > 1e-8 or distance(point_of(segment.end), b) > 1e-8:
            raise ValueError('segment endpoints do not match waypoints')
        axis = (0 if dx > 0 else 2) if dy == 0 else (1 if dy > 0 else 3)
        heading = (0.0, math.pi/2, math.pi, -math.pi/2)[axis]
        length = math.hypot(dx, dy)
        if length > max_length or length < 1e-5:
            raise ValueError('segment length outside executable range [0.00001, max_length]')
        if (segment.index != i or segment.axis != axis
                or not math.isfinite(segment.heading) or not math.isfinite(segment.length)
                or abs(wrap(segment.heading-heading)) > 1e-6
                or abs(segment.length-length) > 1e-6):
            raise ValueError('inconsistent segment metadata')
        headings.append(heading)
        lengths.append(length)
    if abs(math.fsum(lengths)-msg.total_length) > 1e-6:
        raise ValueError('total length mismatch')
    if abs(wrap(yaw_of(msg.waypoints[-1].orientation)-msg.goal_yaw)) > 1e-6:
        raise ValueError('final orientation does not match goal_yaw')
    return Route(msg.route_id, points, tuple(headings), tuple(lengths), wrap(msg.goal_yaw))


def validate_map(grid):
    info = grid.info
    ox, oy = point_of(info.origin.position)
    if grid.header.frame_id != 'map' or abs(yaw_of(info.origin.orientation)) > 1e-8:
        raise ValueError('map must be axis-aligned in map frame')
    if (info.width <= 0 or info.height <= 0 or not math.isfinite(info.resolution)
            or info.resolution <= 0 or len(grid.data) != info.width*info.height
            or len(grid.data) > 4000000 or any(v < -1 or v > 100 for v in grid.data)):
        raise ValueError('invalid map geometry/data')
    return ox, oy, float(info.resolution), int(info.width), int(info.height)


def map_digest(grid):
    geometry = validate_map(grid)
    return hashlib.sha256(repr(geometry).encode() + bytes(v + 1 for v in grid.data)).hexdigest()


def collision_check(route, grid, radius, max_checks=2000000):
    """Conservative circle sweep; unknown and occupancy >=65 are blocked.

    Includes cell area, map bounds, endpoints and in-place rotations. A tracking
    allowance should be included in radius. No diagonal shortcuts or snapping.
    """
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('positive robot envelope required')
    ox, oy, res, width, height = validate_map(grid)
    edges = list(zip(route.points, route.points[1:])) or [(route.points[0], route.points[0])]
    checks = 0
    for a, b in edges:
        xmin, xmax = min(a[0], b[0]), max(a[0], b[0])
        ymin, ymax = min(a[1], b[1]), max(a[1], b[1])
        if xmin-radius <= ox or ymin-radius <= oy or xmax+radius >= ox+width*res or ymax+radius >= oy+height*res:
            raise ValueError('robot envelope intersects map boundary')
        # Include the cell on the lower side of an exact grid-line tangency.
        c0, c1 = max(0, math.floor((xmin-radius-ox)/res)-1), math.floor((xmax+radius-ox)/res)
        r0, r1 = max(0, math.floor((ymin-radius-oy)/res)-1), math.floor((ymax+radius-oy)/res)
        checks += (c1-c0+1)*(r1-r0+1)
        if checks > max_checks:
            raise ValueError('collision check budget exceeded; simplify map/route')
        for row in range(r0, r1+1):
            for col in range(c0, c1+1):
                value = grid.data[row*width+col]
                if value != -1 and value < 65:
                    continue
                left, bottom = ox+col*res, oy+row*res
                dx = max(left-xmax, xmin-(left+res), 0.0)
                dy = max(bottom-ymax, ymin-(bottom+res), 0.0)
                if math.hypot(dx, dy) <= radius:
                    raise ValueError(f'robot envelope intersects blocked cell ({col},{row})')


@dataclass(frozen=True)
class Step:
    kind: int
    segment: int
    phase: str
    yaw: float
    length: float
    start: tuple
    end: tuple


def build_steps(route, map_to_odom_yaw):
    steps = []
    for i, (heading, length) in enumerate(zip(route.headings, route.lengths)):
        yaw = wrap(heading-map_to_odom_yaw)
        a, b = route.points[i:i+2]
        steps.append(Step(1, i, 'ROTATING', yaw, 0.0, a, a))
        steps.append(Step(2, i, 'MOVING', yaw, length, a, b))
    steps.append(Step(1, len(route.lengths), 'FINAL_ROTATION',
                      wrap(route.goal_yaw-map_to_odom_yaw), 0.0, route.points[-1], route.points[-1]))
    return steps
