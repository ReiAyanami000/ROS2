import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool


def draw_circle(cells, info, x, y, radius):
    """在原点无旋转的地图中，保守绘制圆形占据区域。"""
    resolution = info.resolution
    width = info.width
    height = info.height

    local_x = x - info.origin.position.x
    local_y = y - info.origin.position.y

    # 整个圆必须位于地图范围内，不把越界坐标强行夹到边缘。
    if (local_x - radius < 0 or local_y - radius < 0
            or local_x + radius > width * resolution
            or local_y + radius > height * resolution):
        raise ValueError(f'圆柱 ({x:.3f}, {y:.3f}) 超出地图范围')

    min_col = max(
        0, math.floor((local_x - radius) / resolution) - 1
    )
    max_col = min(
        width - 1, math.floor((local_x + radius) / resolution) + 1
    )
    min_row = max(
        0, math.floor((local_y - radius) / resolution) - 1
    )
    max_row = min(
        height - 1, math.floor((local_y + radius) / resolution) + 1
    )

    for row in range(min_row, max_row + 1):
        for col in range(min_col, max_col + 1):
            x0 = col * resolution
            y0 = row * resolution

            nearest_x = min(max(local_x, x0), x0 + resolution)
            nearest_y = min(max(local_y, y0), y0 + resolution)

            dx = local_x - nearest_x
            dy = local_y - nearest_y

            if dx * dx + dy * dy <= radius * radius + 1e-12:
                cells[row * width + col] = 100


class MapComposer(Node):
    def __init__(self):
        super().__init__('map_composer')

        self.declare_parameter('obstacle_diameter', 0.02)
        diameter = float(
            self.get_parameter('obstacle_diameter').value
        )

        if not math.isfinite(diameter) or diameter <= 0:
            raise ValueError('obstacle_diameter 必须是正数')

        # 本版参数在启动时读取，修改直径后应重启节点。
        self.radius = diameter / 2.0

        self.base_map = None
        # None：尚未收到有效扫描；[]：有效扫描，没有障碍。
        self.obstacles = None

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 障碍输入要求节点启动后收到新结果，不自动复用历史扫描。
        obstacle_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid, '/navigation/mission_map', map_qos
        )
        self.ready_pub = self.create_publisher(
            Bool, '/navigation/map_ready', map_qos
        )

        self.base_sub = self.create_subscription(
            OccupancyGrid,
            '/field/base_map',
            self.on_base_map,
            map_qos,
        )
        self.obstacle_sub = self.create_subscription(
            PoseArray,
            '/radar/obstacle_centers',
            self.on_obstacles,
            obstacle_qos,
        )

        self.set_ready(False)
        self.get_logger().info('等待底图和本轮完整障碍列表')

    def set_ready(self, ready):
        msg = Bool()
        msg.data = ready
        self.ready_pub.publish(msg)

    def on_base_map(self, msg):
        self.set_ready(False)
        self.base_map = None

        info = msg.info
        q = info.origin.orientation

        if msg.header.frame_id != 'map':
            self.get_logger().error('底图 frame_id 必须为 map')
            return

        if (not math.isfinite(info.resolution)
                or info.resolution <= 0
                or info.width == 0 or info.height == 0
                or len(msg.data) != info.width * info.height):
            self.get_logger().error('底图分辨率、尺寸或数据长度不合法')
            return

        values = (
            info.origin.position.x, info.origin.position.y,
            q.x, q.y, q.z, q.w,
        )
        if not all(math.isfinite(v) for v in values):
            self.get_logger().error('底图原点存在非有限数值')
            return

        # 对应当前地图 origin: [-0.1, -0.1, 0.0]。
        if (abs(q.x) > 1e-6 or abs(q.y) > 1e-6
                or abs(q.z) > 1e-6
                or abs(abs(q.w) - 1.0) > 1e-6):
            self.get_logger().error('本版只支持原点旋转角为 0 的底图')
            return

        if any(value < -1 or value > 100 for value in msg.data):
            self.get_logger().error('底图占据值必须位于 -1～100')
            return

        self.base_map = copy.deepcopy(msg)
        self.try_compose()

    def on_obstacles(self, msg):
        self.set_ready(False)
        self.obstacles = None

        if msg.header.frame_id != 'map':
            self.get_logger().error(
                '障碍必须先转换到 map 坐标系，本节点不执行 TF 转换'
            )
            return

        centers = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y

            if not math.isfinite(x) or not math.isfinite(y):
                self.get_logger().error('障碍坐标存在非有限数值')
                return

            centers.append((x, y))

        # 空列表也是有效结果，不能写成 if not centers: return。
        self.obstacles = centers
        self.try_compose()

    def try_compose(self):
        if self.base_map is None or self.obstacles is None:
            return

        # 每次从底图重新构造，而不是修改上一轮合成地图。
        output = copy.deepcopy(self.base_map)
        cells = list(self.base_map.data)

        try:
            for x, y in self.obstacles:
                draw_circle(cells, output.info, x, y, self.radius)
        except ValueError as error:
            self.obstacles = None
            self.set_ready(False)
            self.get_logger().error(str(error))
            return

        output.data = cells
        output.header.stamp = self.get_clock().now().to_msg()

        self.map_pub.publish(output)
        self.set_ready(True)

        self.get_logger().info(
            f'合成完成：{len(self.obstacles)} 个圆柱，'
            f'直径 {self.radius * 2:.3f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapComposer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()