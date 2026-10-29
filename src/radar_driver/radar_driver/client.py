"""雷达 v6 非阻塞业务客户端，不依赖 ROS 和实际串口。

单线程调用顺序：feed(收到的字节) -> poll() -> take_tx() -> take_events()。
take_tx()交出完整帧；串口层必须按顺序处理部分写入，不能重复发送或交叉帧。
计时从命令入队开始，发送层应及时取走数据；传输失败调用transport_lost()。
初始化和重连不自动扫描。扫描/校准成功仅通过相应完成事件发布。
"""

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
import struct
import time
from typing import Callable

from .models import (
    Ack, CalibrationResult, GridResult, HelloInfo, Obstacle, PersistedState,
    RadarError, RadarPose, RadarStatus, ScanResult, GRID_OUTSIDE,
    MAX_OBSTACLES, SCAN_FRAME_COUNT,
)
from .protocol import Command, Event, Frame, FrameDecoder, ProtocolError, encode_frame


class LinkState(str, Enum):
    DISCONNECTED = "disconnected"
    WAIT_HELLO = "wait_hello"
    READING_STATE = "reading_state"
    NEED_ZONE = "need_zone"
    SELECTING_ZONE = "selecting_zone"
    READY = "ready"
    STOPPING = "stopping"
    FAULT = "fault"


class RequestRejected(RuntimeError):
    """本地拒绝：未就绪、忙或参数错误以外的运行条件不满足。"""


@dataclass(frozen=True)
class ClientFault:
    kind: str
    message: str
    command: int | None = None


@dataclass(frozen=True)
class ClientEvent:
    # kind: link/hello/status/persisted/scan/calibration/stopped/cancelled/device_error/fault
    kind: str
    data: object


@dataclass
class _Transaction:
    command: Command
    started: float
    deadline: float
    ack_ok: bool = False
    values: dict = field(default_factory=dict)
    last_progress: float = 0.0


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ProtocolError(message)


def _pose(x: int, y: int, heading: int, zone: int) -> RadarPose:
    _require(zone in (0, 1, 2), "非法位姿区域")
    _require(0 <= heading < 360 * 64, "非法朝向")
    return RadarPose(x, y, heading, zone)


def _parse(frame: Frame):
    """只解析本阶段使用的业务事件；其他公开或未知事件忽略。"""
    p = frame.payload
    kind = frame.message_type
    if kind == Event.HELLO:
        _require(len(p) == 13, "HELLO长度必须为13")
        major, minor, version, zone, x, y, heading, running, capacity = struct.unpack("<BBBBhhHBH", p)
        _require(version == 6 and running in (0, 1), "HELLO版本或运行标志非法")
        return HelloInfo(major, minor, version, _pose(x, y, heading, zone), bool(running), capacity)
    if kind == Event.ACK:
        _require(len(p) == 2, "ACK长度必须为2")
        return Ack(p[0], p[1])
    if kind == Event.STATUS:
        _require(len(p) == 5, "STATUS长度必须为5")
        status, mode, running, count, overflow = p
        _require(running in (0, 1) and overflow in (0, 1), "STATUS布尔字段非法")
        _require(count <= SCAN_FRAME_COUNT, "扫描圈数超出14")
        return RadarStatus(status, mode, bool(running), count, bool(overflow))
    if kind == Event.OBSTACLE_GRID:
        _require(len(p) >= 5, "GRID载荷过短")
        seq, count = struct.unpack_from("<IB", p)
        _require(count <= MAX_OBSTACLES and len(p) == 5 + count, "GRID数量或长度错误")
        ids = tuple(p[5:])
        _require(all(i < 25 or i == GRID_OUTSIDE for i in ids), "GRID编号非法")
        return GridResult(seq, ids)
    if kind == Event.RESULT:
        _require(len(p) >= 8, "RESULT载荷过短")
        seq, count, locked, overflow, n = struct.unpack_from("<IBBBB", p)
        _require(n <= MAX_OBSTACLES and len(p) == 8 + n * 8, "RESULT数量或长度错误")
        _require(count <= SCAN_FRAME_COUNT and locked in (0, 1) and overflow in (0, 1), "RESULT标志非法")
        items = tuple(struct.iter_unpack("<hhBBH", p[8:]))
        _require(all(hits <= count and confidence <= 100 for _, _, hits, confidence, _ in items), "障碍命中数或置信度非法")
        # 尚无grid_id，暂存原始字段，不构造虚假的Obstacle。
        return seq, count, bool(locked), bool(overflow), items
    if kind == Event.PERSISTED_STATE:
        _require(len(p) >= 33, "PERSISTED载荷过短")
        version, seq, source = struct.unpack_from("<HIB", p)
        valid, count = p[9], p[10]
        _require(source in (0, 1, 2) and valid in (0, 1), "保存状态标志非法")
        _require(count <= 25 and len(p) == 33 + count, "保存方格数量或长度错误")
        ids = tuple(p[33:])
        _require(all(i < 25 for i in ids) and len(set(ids)) == count, "保存方格编号非法或重复")
        x1, y1, heading1, tail1 = struct.unpack_from("<hhHB", p, 11)
        x2, y2, heading2, tail2 = struct.unpack_from("<hhHB", p, 18)
        # 区域由记录所在位置确定；末尾字节的业务含义尚未公开。
        p1 = _pose(x1, y1, heading1, 1)
        p2 = _pose(x2, y2, heading2, 2)
        return PersistedState(
            storage_version=version,
            record_sequence=seq,
            source_zone=source,
            grid_result_valid=bool(valid),
            zone1_pose=p1,
            zone2_pose=p2,
            grid_ids=ids,
            zone1_pose_tail_raw=tail1,
            zone2_pose_tail_raw=tail2,
        )
    if kind == Event.CALIBRATION:
        _require(len(p) == 15, "CALIBRATION长度必须为15")
        zone, x, y, heading, dx, dy, trim, residual, pair_error = struct.unpack("<BhhHhhhBB", p)
        _require(zone in (1, 2), "校准区域非法")
        return CalibrationResult(_pose(x, y, heading, zone), dx, dy, trim, residual, pair_error)
    if kind == Event.ERROR:
        _require(len(p) == 3, "ERROR长度必须为3")
        return RadarError(*struct.unpack("<BH", p))
    return None


class RadarClient:
    """调用方周期调用poll，持续交付输入和取走输出；不从多个线程并发调用。

    node_id：1～254。zone：0=未指定，1/2=明确请求的区域。
    所有request方法仅入队；返回的序号不表示成功，等待完成事件。
    HELLO和ACK没有请求序号回显，无法从协议上完全辨别任意迟到回复。
    因此不并发同类事务；取消/失败后先STOP，再允许新扫描。
    """

    ACK_TIMEOUT = 1.5
    OP_TIMEOUT = 8.0
    SILENCE_TIMEOUT = 3.5
    STOP_TIMEOUT = 0.4
    HEARTBEAT_PERIOD = 0.5
    HELLO_PERIOD = 2.0
    LINK_TIMEOUT = 5.0
    TX_LIMIT = 32
    EVENT_LIMIT = 128

    def __init__(self, node_id: int = 1, zone: int = 0,
                 clock: Callable[[], float] = time.monotonic):
        if not 1 <= node_id <= 254 or zone not in (0, 1, 2):
            raise ValueError("node_id须为1～254，zone须为0/1/2")
        self.node_id = node_id
        self.target_zone = zone
        self._clock = clock
        self._decoder = FrameDecoder()
        self._tx = deque()
        self._stop_wire = None
        self._events = deque()
        self._sequence = 0
        self._pending = None
        self._probe = None
        self._last_rx = clock()
        self._next_heartbeat = clock()
        self._next_hello = clock()
        self._selected_zone = 0
        self._last_scan_sequence = None
        self._needs_stop = False
        self._running = False
        self._queue_fault = False
        self._bootstrap = False
        self.state = LinkState.DISCONNECTED
        self.hello = None
        self.persisted = None
        self.status = None
        self._restart()

    @property
    def ready(self) -> bool:
        return self.state == LinkState.READY

    @property
    def busy(self) -> bool:
        return self._pending is not None

    @property
    def selected_zone(self) -> int:
        return self._selected_zone

    def take_events(self) -> list[ClientEvent]:
        result = list(self._events)
        self._events.clear()
        return result

    def take_tx(self) -> bytes | None:
        """一次取一条完整帧。STOP优先；None表示当前无数据。

        已交出的帧不可撤销。串口层不得大量预取，应完成当前帧后再取下一帧。
        """
        if self._stop_wire is not None:
            wire, self._stop_wire = self._stop_wire, None
            return wire
        return self._tx.popleft() if self._tx else None

    def _emit(self, kind: str, data: object) -> None:
        if self._queue_fault:
            return
        if len(self._events) >= self.EVENT_LIMIT:
            # 旧结果全部作废，保留一个明确故障；STOP仍可发送。
            self._events.clear()
            self._events.append(ClientEvent("fault", ClientFault("event_queue", "事件未及时消费，结果作废；请重建客户端")))
            self._queue_fault = True
            self._pending = None
            self._probe = None
            self._tx.clear()
            self._stop_wire = self._wire(Command.STOP)[1]
            self.state = LinkState.FAULT
            return
        self._events.append(ClientEvent(kind, data))

    def _set_state(self, state: LinkState) -> None:
        if self._queue_fault:
            return
        if self.state != state:
            self.state = state
            self._emit("link", state)

    def _wire(self, command: Command, payload: bytes = b""):
        seq = self._sequence
        self._sequence = (seq + 1) & 0xFFFF
        return seq, encode_frame(self.node_id, command, seq, payload)

    def _send(self, command: Command, payload: bytes = b"") -> int:
        if self._queue_fault:
            raise RequestRejected("事件队列溢出，请重建客户端")
        if len(self._tx) >= self.TX_LIMIT:
            raise RequestRejected("发送队列已满；调用方没有及时发送")
        seq, wire = self._wire(command, payload)
        self._tx.append(wire)
        return seq

    def _begin(self, command: Command, payload: bytes = b"", **values) -> int:
        seq = self._send(command, payload)
        now = self._clock()
        timeout = self.OP_TIMEOUT if command in (Command.SCAN_LOCKED, Command.CALIBRATE) else self.ACK_TIMEOUT
        self._pending = _Transaction(command, now, now + timeout, values=values, last_progress=now)
        return seq

    def _restart(self) -> None:
        if self._queue_fault:
            return  # 保留事件溢出时预留的STOP帧
        self._pending = None
        self._probe = None
        self._tx.clear()
        self._stop_wire = None
        self._decoder.reset_partial()
        self.hello = None
        self.persisted = None
        self.status = None
        self._running = False
        self._selected_zone = 0
        self._last_scan_sequence = None
        self._bootstrap = True
        now = self._clock()
        self._last_rx = now
        self._next_hello = now
        self._next_heartbeat = now
        self._set_state(LinkState.WAIT_HELLO)

    def transport_lost(self, reason: str = "串口连接中断") -> None:
        """串口异常后调用；串口层同时清除其未发送帧和接收残留。"""
        self._emit("fault", ClientFault("transport", reason))
        if not self._queue_fault:
            self._restart()

    def _check_request(self) -> None:
        if not self.ready or self.busy or self._queue_fault:
            raise RequestRejected("客户端未READY或已有业务正在执行")

    def request_select_zone(self, zone: int) -> int | None:
        if zone not in (1, 2):
            raise ValueError("工作区域必须为1或2")
        if self._queue_fault or self.busy or self.state in (LinkState.STOPPING, LinkState.FAULT):
            raise RequestRejected("忙或故障状态不能选区")
        self.target_zone = zone
        if self.state == LinkState.WAIT_HELLO:
            return None  # 握手完成后执行
        self._selected_zone = 0
        self._set_state(LinkState.SELECTING_ZONE)
        return self._begin(Command.SELECT_ZONE, bytes([zone]), zone=zone)

    def request_read_persisted_state(self) -> int:
        self._check_request()
        return self._begin(Command.GET_PERSISTED_STATE)

    def _check_idle(self) -> None:
        self._check_request()
        if self._running:
            raise RequestRejected("雷达正在运行；请先停止当前操作")
        if self.status is not None:
            # running只代表采集是否运行，不代表设备业务是否空闲。
            # 0/3/4分别为就绪、已停止、结果锁定；其他状态保守拒绝。
            if self.status.status not in (0, 3, 4):
                raise RequestRejected("雷达处于忙或未知状态，暂不接受新业务")
            # 5～7为内部保留模式；未来新增的未知模式也不能自动放行。
            if self.status.mode not in (0, 1, 2, 3, 4):
                raise RequestRejected("雷达处于内部或未知模式，暂不接受新业务")

    def request_scan_once(self) -> int:
        self._check_idle()
        seq = self._begin(Command.SCAN_LOCKED, baseline=self._last_scan_sequence)
        self._needs_stop = True
        return seq

    def request_calibrate(self, zone: int | None = None) -> int:
        self._check_idle()
        if zone is None:
            zone = self.selected_zone
        if zone != self.selected_zone:
            raise ValueError("只能校准已确认的当前区域")
        seq = self._begin(Command.CALIBRATE, bytes([zone]), zone=zone)
        self._needs_stop = True
        return seq

    def request_stop(self) -> int:
        if self._queue_fault:
            # 队列溢出后只允许恢复停止通道，旧业务和事件全部作废。
            # 保留接收解码器，以便处理已收到一部分的回复。
            self._events.clear()
            self._queue_fault = False
            self._pending = None
        if self._pending is not None and self._pending.command == Command.STOP:
            raise RequestRejected("STOP尚未完成")
        if self._pending is not None:
            self._emit("cancelled", self._pending.command)
        return self._begin_stop()

    def _begin_stop(self) -> int:
        self._tx.clear()
        self._probe = None
        seq, self._stop_wire = self._wire(Command.STOP)
        now = self._clock()
        self._pending = _Transaction(Command.STOP, now, now + self.STOP_TIMEOUT)
        self._needs_stop = True
        self._set_state(LinkState.STOPPING)
        return seq

    def _fail(self, kind: str, message: str) -> None:
        command = self._pending.command if self._pending else None
        self._emit("fault", ClientFault(kind, message, command))
        if self._queue_fault:
            return
        if command in (Command.SCAN_LOCKED, Command.CALIBRATE):
            self._begin_stop()
        elif command == Command.STOP:
            self._pending = None
            self._set_state(LinkState.FAULT)  # 不宣称已停止；可显式重试STOP
        else:
            self._restart()

    def _expire(self) -> None:
        """接收前也检查期限，防止迟到结果在下一次poll之前被判为成功。"""
        now = self._clock()
        if self.state != LinkState.WAIT_HELLO and now - self._last_rx >= self.LINK_TIMEOUT:
            self._emit("fault", ClientFault("link_lost", "连续5秒无当前节点有效回复"))
            self._restart()
        if self._queue_fault:
            return
        pending = self._pending
        if pending is not None:
            ack_expired = not pending.ack_ok and now - pending.started >= self.ACK_TIMEOUT
            silent = pending.command in (Command.SCAN_LOCKED, Command.CALIBRATE) and now - pending.last_progress >= self.SILENCE_TIMEOUT
            if now >= pending.deadline or ack_expired or silent:
                self._fail("timeout", f"{pending.command.name}等待确认或结果超时")
        if self._probe is not None and now >= self._probe.deadline:
            self._probe = None
            self._next_hello = now

    def poll(self) -> None:
        if self._queue_fault:
            return
        self._expire()
        if self._queue_fault:
            return
        now = self._clock()
        try:
            if self.hello is not None and now >= self._next_heartbeat:
                self._send(Command.HEARTBEAT)
                self._next_heartbeat = now + self.HEARTBEAT_PERIOD
            if (self.state != LinkState.STOPPING
                    and now >= self._next_hello
                    and (self._probe is None or self.state == LinkState.WAIT_HELLO)):
                self._send(Command.HELLO)
                # 握手重试属于同一个有界探测窗口，不为每次重发建立并发事务。
                if self._probe is None:
                    self._probe = _Transaction(Command.HELLO, now, now + self.ACK_TIMEOUT)
                period = 0.2 if self.state == LinkState.WAIT_HELLO else self.HELLO_PERIOD
                self._next_hello = now + period
        except RequestRejected as error:
            self._fail("tx_queue", str(error))

    def feed(self, data: bytes) -> None:
        if self._queue_fault:
            return
        self._expire()
        if self._queue_fault:
            return
        previous_bad = self._decoder.bad_frames
        frames = self._decoder.feed(data)
        if self._decoder.bad_frames != previous_bad:
            self._emit("fault", ClientFault("frame", self._decoder.last_error))
        for frame in frames:
            if self._queue_fault:
                break
            if frame.node_id != self.node_id:
                continue
            try:
                value = _parse(frame)
                if value is None:
                    continue
                self._last_rx = self._clock()
                self._handle(frame.message_type, value)
            except (ProtocolError, RequestRejected) as error:
                self._fail("protocol", str(error))

    def _handle(self, kind: int, value) -> None:
        pending = self._pending
        if kind == Event.HELLO:
            self.hello = value
            self._running = value.running
            self._emit("hello", value)
            if self._probe is not None:
                self._probe.values["hello"] = value
            if pending is not None and pending.command == Command.SELECT_ZONE:
                if value.pose.zone == pending.values["zone"]:
                    pending.values["hello"] = value
            elif self.selected_zone and value.pose.zone != self.selected_zone:
                self._fail("zone_changed", "设备区域变化，重新确认连接")
                return
        elif kind == Event.ACK:
            if value.command == Command.HELLO and self._probe is not None:
                if value.accepted:
                    self._probe.ack_ok = True
                else:
                    self._probe = None
                    self._emit("fault", ClientFault("device_ack", f"HELLO被拒绝，代码{value.result}", value.command))
            elif pending is not None and value.command == pending.command:
                if not value.accepted:
                    self._fail("device_ack", f"命令被拒绝，代码{value.result}")
                    return
                pending.ack_ok = True
        elif kind == Event.ERROR:
            self._emit("device_error", value)
            if pending is not None:
                self._fail("device", f"雷达错误code={value.code}, detail={value.detail}")
                return
        elif kind == Event.STATUS:
            self.status = value
            self._running = value.running
            self._emit("status", value)
            if pending is not None:
                if pending.command == Command.STOP and value.status == 3 and not value.running:
                    pending.values["stopped"] = True
                elif pending.ack_ok and pending.command in (Command.SCAN_LOCKED, Command.CALIBRATE):
                    expected_mode = 1 if pending.command == Command.SCAN_LOCKED else 3
                    if value.mode == expected_mode:
                        _require(not value.overflow, "本次采集容量溢出")
                        previous = pending.values.get("progress", -1)
                        _require(value.frame_count >= previous, "扫描进度倒退")
                        if value.frame_count > previous:
                            pending.values["progress"] = value.frame_count
                            pending.last_progress = self._clock()
                        if value.status == 4 and value.frame_count == 14 and not value.running:
                            pending.values["done"] = True
        elif kind == Event.PERSISTED_STATE:
            if pending is not None and pending.command in (Command.GET_PERSISTED_STATE, Command.CALIBRATE):
                if pending.command != Command.CALIBRATE or (pending.ack_ok and value.source_zone == pending.values["zone"]):
                    pending.values["persisted"] = value
        elif kind == Event.CALIBRATION:
            if pending is not None and pending.command == Command.CALIBRATE and pending.ack_ok:
                _require(value.pose.zone == pending.values["zone"], "校准结果区域不匹配")
                pending.values["calibration"] = value
        elif kind in (Event.OBSTACLE_GRID, Event.RESULT):
            sequence = value.scan_sequence if kind == Event.OBSTACLE_GRID else value[0]
            if self._last_scan_sequence is None or self._newer(sequence, self._last_scan_sequence):
                self._last_scan_sequence = sequence
            if pending is not None and pending.command == Command.SCAN_LOCKED and pending.ack_ok:
                baseline = pending.values["baseline"]
                if baseline is not None and not self._newer(sequence, baseline):
                    return
                # ACK后的进度标记用于排除旧事务的迟到结果。
                if "progress" not in pending.values:
                    return
                old_sequence = pending.values.get("scan_sequence")
                _require(old_sequence is None or old_sequence == sequence, "同一事务扫描序号不一致")
                pending.values["scan_sequence"] = sequence
                key = "grid" if kind == Event.OBSTACLE_GRID else "result"
                _require(key not in pending.values or pending.values[key] == value, "重复结果内容冲突")
                pending.values[key] = value
        self._advance()

    @staticmethod
    def _newer(value: int, previous: int) -> bool:
        return 0 < ((value - previous) & 0xFFFFFFFF) < 0x80000000

    def _advance(self) -> None:
        if self._queue_fault:
            return
        probe = self._probe
        if probe is not None and probe.ack_ok and "hello" in probe.values:
            self._probe = None
            self._next_hello = self._clock() + self.HELLO_PERIOD
            if self.state == LinkState.WAIT_HELLO:
                if self._needs_stop:
                    self._begin_stop()
                else:
                    self._start_state_read()
        pending = self._pending
        if pending is None:
            return
        values = pending.values
        command = pending.command
        if command == Command.STOP:
            if pending.ack_ok or values.get("stopped"):
                self._pending = None
                self._needs_stop = False
                self.status = RadarStatus(3, 4, False, 0, False)
                self._emit("stopped", None)
                # 重新读取设备信息和保存状态，不自动恢复被取消业务。
                self._restart()
            return
        if not pending.ack_ok:
            return
        if command == Command.GET_PERSISTED_STATE and "persisted" in values:
            self.persisted = values["persisted"]
            self._pending = None
            self._emit("persisted", self.persisted)
            if self._bootstrap:
                self._bootstrap = False
                current_zone = self.hello.pose.zone
                if self.target_zone and current_zone != self.target_zone:
                    self.request_select_zone(self.target_zone)
                else:
                    self._selected_zone = current_zone
                    self._set_state(LinkState.READY if current_zone else LinkState.NEED_ZONE)
        elif command == Command.SELECT_ZONE and "hello" in values:
            self._selected_zone = values["zone"]
            self._pending = None
            self._bootstrap = True
            self._start_state_read()
        elif command == Command.SCAN_LOCKED and "result" in values:
            seq, count, locked, overflow, items = values["result"]
            _require(not overflow and locked and count == 14, "完整扫描结果未锁定、溢出或圈数错误")
            if "grid" not in values or not values.get("done"):
                return
            grid = values["grid"]
            _require(grid.scan_sequence == seq and grid.count == len(items), "坐标与方格不匹配")
            obstacles = tuple(Obstacle(x, y, grid_id, hits, confidence, points)
                              for (x, y, hits, confidence, points), grid_id in zip(items, grid.grid_ids))
            result = ScanResult(seq, count, locked, overflow, obstacles)
            self._pending = None
            self._needs_stop = False
            self._emit("scan", result)
        elif command == Command.CALIBRATE and "calibration" in values and "persisted" in values:
            result = values["calibration"]
            persisted = values["persisted"]
            saved_pose = persisted.zone1_pose if result.pose.zone == 1 else persisted.zone2_pose
            _require(saved_pose == result.pose, "校准位姿与保存位姿不一致")
            self.persisted = persisted
            self._pending = None
            self._needs_stop = False
            self._emit("persisted", persisted)
            self._emit("calibration", result)

    def _start_state_read(self) -> None:
        self._set_state(LinkState.READING_STATE)
        self._begin(Command.GET_PERSISTED_STATE)
