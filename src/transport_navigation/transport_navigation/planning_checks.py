"""独立于 ROS 运行时的地图检查，便于在主机上验证。"""

import hashlib
import math


def geometry(grid):
    """地图几何信息；本工程只接收无旋转的 map 地图。"""
    info = grid.info
    p = info.origin.position
    q = info.origin.orientation
    return (info.width, info.height, info.resolution,
            p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def valid_map(grid):
    """防止空地图、非法坐标和不支持的地图原点进入规划流程。"""
    g = geometry(grid)
    q = grid.info.origin.orientation
    return (
        grid.header.frame_id == 'map'
        and all(math.isfinite(v) for v in g)
        and grid.info.width > 0 and grid.info.height > 0
        and grid.info.resolution > 0
        and len(grid.data) == grid.info.width * grid.info.height
        and all(-1 <= v <= 100 for v in grid.data)
        and abs(q.x) < 1e-6 and abs(q.y) < 1e-6 and abs(q.z) < 1e-6
        and abs(abs(q.w) - 1.0) < 1e-6
    )


def map_digest(grid):
    """忽略发布时间，识别本轮地图内容变化。"""
    digest = hashlib.sha256(repr(geometry(grid)).encode())
    digest.update(bytes(v + 1 for v in grid.data))
    return digest.digest()


def costmap_contains_map(source, costmap):
    """首次加载的保守检查，并非通用的地图版本确认协议。"""
    if not valid_map(costmap):
        return False
    if not all(math.isclose(a, b, rel_tol=0, abs_tol=1e-6)
               for a, b in zip(geometry(source), geometry(costmap))):
        return False
    return all(actual == 100 for expected, actual in zip(source.data, costmap.data)
               if expected == 100)


def point_issue(grid, x, y):
    """仅做占据地图粗筛；车体和膨胀区碰撞最终由 Nav2 判断。"""
    if not math.isfinite(x) or not math.isfinite(y):
        return '位置含非有限数值'
    info = grid.info
    col = math.floor((x - info.origin.position.x) / info.resolution)
    row = math.floor((y - info.origin.position.y) / info.resolution)
    if not (0 <= col < info.width and 0 <= row < info.height):
        return '位置超出地图'
    value = grid.data[row * info.width + col]
    if value < 0:
        return '位置在未知区域'
    if value >= 100:
        return '位置在障碍内'
    return ''
