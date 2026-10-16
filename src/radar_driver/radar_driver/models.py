"""雷达业务数据模型，不依赖 ROS、串口或协议解码器。

保留协议单位（毫米、角度×64），在 ROS 边界转换为米和弧度。
client 层负责校验载荷、关联扫描结果及判定事务是否成功。
集合字段使用 tuple；构造时也应传入 tuple，避免可变列表共享。
"""

from dataclasses import dataclass
from math import radians


GRID_OUTSIDE = 0xFF
MAX_OBSTACLES = 25
SCAN_FRAME_COUNT = 14


@dataclass(frozen=True)
class RadarPose:
    """雷达中心的场地位姿，不代表小车移动中的实时定位。"""

    x_mm: int
    y_mm: int
    heading_q6: int
    zone: int

    @property
    def x_m(self) -> float:
        return self.x_mm / 1000.0

    @property
    def y_m(self) -> float:
        return self.y_mm / 1000.0

    @property
    def heading_deg(self) -> float:
        return self.heading_q6 / 64.0

    @property
    def heading_rad(self) -> float:
        """仅转换单位，不改变设备的角度零点和正方向。"""
        return radians(self.heading_deg)


@dataclass(frozen=True)
class HelloInfo:
    """EVT_HELLO：设备信息；pose.zone=0表示尚未确认区域。"""

    firmware_major: int
    firmware_minor: int
    protocol: int
    pose: RadarPose
    running: bool
    max_scan_points: int


@dataclass(frozen=True)
class Ack:
    """EVT_ACK：仅表示受理结果，不包含请求序号。"""

    command: int
    result: int

    @property
    def accepted(self) -> bool:
        return self.result == 0


@dataclass(frozen=True)
class RadarStatus:
    """EVT_STATUS：雷达运行状态，不是控制端链路状态。

    status和mode保留原始整数，避免收到保留值时枚举转换失败。
    """

    status: int
    mode: int
    running: bool
    frame_count: int
    overflow: bool


@dataclass(frozen=True)
class Obstacle:
    """障碍中心坐标；grid_id由同扫描序号的方格事件关联得到。

    不包含障碍物尺寸。GRID_OUTSIDE表示方格区域外，坐标仍有效。
    """

    x_mm: int
    y_mm: int
    grid_id: int
    frame_hits: int
    confidence_percent: int
    point_count: int

    @property
    def x_m(self) -> float:
        return self.x_mm / 1000.0

    @property
    def y_m(self) -> float:
        return self.y_mm / 1000.0


@dataclass(frozen=True)
class ScanResult:
    """已关联方格的障碍结果；事务成功仍需客户端确认。

    obstacles=()可以是有效空结果；不能用空结果代替超时或失败。
    """

    scan_sequence: int
    frame_count: int
    locked: bool
    overflow: bool
    obstacles: tuple[Obstacle, ...]

    @property
    def obstacle_count(self) -> int:
        return len(self.obstacles)


@dataclass(frozen=True)
class GridResult:
    """EVT_OBSTACLE_GRID：关联完整障碍结果时的中间数据。"""

    scan_sequence: int
    grid_ids: tuple[int, ...]

    @property
    def count(self) -> int:
        return len(self.grid_ids)


@dataclass(frozen=True)
class PersistedState:
    """EVT_PERSISTED_STATE：source_zone不是当前工作区域。

    grid_result_valid=True且grid_ids为空表示已保存有效空扫描。
    非公开保留字段由解析器跳过，不在本模型中暴露。
    """

    storage_version: int
    record_sequence: int
    source_zone: int
    grid_result_valid: bool
    zone1_pose: RadarPose
    zone2_pose: RadarPose
    grid_ids: tuple[int, ...]

    @property
    def grid_count(self) -> int:
        return len(self.grid_ids)


@dataclass(frozen=True)
class CalibrationResult:
    """EVT_CALIBRATION：还需结合ACK和保存状态确认校准事务。"""

    pose: RadarPose
    x_offset_mm: int
    y_offset_mm: int
    heading_trim_q6: int
    residual_mm: int
    pair_distance_error_mm: int


@dataclass(frozen=True)
class RadarError:
    """EVT_ERROR的原始设备错误；本地串口异常不伪装成该事件。"""

    code: int
    detail: int