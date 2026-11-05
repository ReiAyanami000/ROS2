import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid


class MapComposerDemo(Node):
    def __init__(self):
        super().__init__('map_composer_demo')

        # 启动时设置的模拟障碍参数，单位为米。
        self.declare_parameter('obstacle_x', 1.20)
        self.declare_parameter('obstacle_y', 0.80)
        self.declare_parameter('obstacle_diameter', 0.02)
        self.declare_parameter('obstacle_enabled', True)

        self.cx = float(self.get_parameter('obstacle_x').value)
        self.cy = float(self.get_parameter('obstacle_y').value)
        self.radius = (
            float(self.get_parameter('obstacle_diameter').value) / 2.0
        )
        self.enabled = self.get_parameter('obstacle_enabled').value

        if not all(math.isfinite(v) for v in
                   (self.cx, self.cy, self.radius)) or self.radius <= 0:
            raise ValueError('障碍坐标必须有限，直径必须大于 0')

        # 保留最新地图，支持晚启动的订阅者。
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/navigation/mission_map',
            map_qos,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/field/base_map',
            self.on_base_map,
            map_qos,
        )

        self.get_logger().info('等待 /field/base_map ...')

    def on_base_map(self, base_map):
        info = base_map.info
        resolution = info.resolution
        width = info.width
        height = info.height

        if (resolution <= 0 or width == 0 or height == 0
                or len(base_map.data) != width * height):
            self.get_logger().error('底图尺寸或数据长度不合法')
            return

        if base_map.header.frame_id != 'map':
            self.get_logger().error('本示例要求底图坐标系为 map')
            return

        # 当前底图 origin 的旋转角为 0；旋转地图不能套用下面的公式。
        q = info.origin.orientation
        if (abs(q.x) > 1e-6 or abs(q.y) > 1e-6
                or abs(q.z) > 1e-6 or abs(abs(q.w) - 1.0) > 1e-6):
            self.get_logger().error('本示例仅支持原点旋转角为 0 的地图')
            return

        # 每次都从原始底图复制，避免累积旧障碍。
        output = copy.deepcopy(base_map)
        cells = list(base_map.data)
        added_cells = 0

        if self.enabled:
            origin_x = info.origin.position.x
            origin_y = info.origin.position.y

            # 障碍中心转换为相对于栅格原点的米制坐标。
            local_x = self.cx - origin_x
            local_y = self.cy - origin_y
            radius = self.radius

            if (local_x - radius < 0 or local_y - radius < 0
                    or local_x + radius > width * resolution
                    or local_y + radius > height * resolution):
                self.get_logger().error('模拟障碍超出地图范围，未发布')
                return

            # 多检查一格，保守覆盖与圆周接触的栅格。
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
                    # 当前栅格矩形与圆心之间的最近距离。
                    x0 = col * resolution
                    y0 = row * resolution
                    x1 = x0 + resolution
                    y1 = y0 + resolution

                    nearest_x = min(max(local_x, x0), x1)
                    nearest_y = min(max(local_y, y0), y1)

                    dx = local_x - nearest_x
                    dy = local_y - nearest_y

                    # 圆与栅格相交或接触，就标为占据。
                    if dx * dx + dy * dy <= radius * radius + 1e-12:
                        index = row * width + col
                        if cells[index] != 100:
                            added_cells += 1
                        cells[index] = 100

        output.data = cells
        output.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(output)

        self.get_logger().info(
            f'已发布合成地图：{width}×{height}，'
            f'新增占据栅格 {added_cells} 个，'
            f'模拟障碍启用={self.enabled}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapComposerDemo()
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
