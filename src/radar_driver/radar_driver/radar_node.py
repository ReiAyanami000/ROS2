"""ROS 2 Jazzy 雷达连接节点。

实现：
1. 打开串口。
2. HELLO握手、读取保存状态、确认区域。
3. 打印设备信息和状态变化。
4. 由RadarClient维护心跳、健康检查和协议重连。
5. 串口异常后定时重新打开。

本阶段不提供扫描、校准或运动控制接口。
所有通信操作在单线程执行器中串行执行。
"""

import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .client import LinkState, RadarClient
from .serial_transport import SerialTransport, TransportError


class RadarNode(Node):
    """管理串口连接并运行雷达协议客户端。"""

    def __init__(self) -> None:
        super().__init__("radar_node")

        # 参数在启动时设置；修改后需要重启节点。
        self._declare_read_only(
            "port",
            "/dev/ttyUSB0",
            "雷达串口路径，建议使用/dev/serial/by-id/下的稳定路径",
        )
        self._declare_read_only(
            "node_id",
            1,
            "雷达节点ID，范围1～254",
        )
        self._declare_read_only(
            "zone",
            0,
            "工作区域：0表示不主动指定，1或2表示明确选择实际区域",
        )
        self._declare_read_only(
            "poll_period_s",
            0.005,
            "通信轮询周期，单位秒",
        )
        self._declare_read_only(
            "reconnect_interval_s",
            2.0,
            "串口打开失败后的重试间隔，单位秒",
        )

        self._port = self.get_parameter("port").value
        self._node_id = self.get_parameter("node_id").value
        self._zone = self.get_parameter("zone").value
        self._poll_period = self.get_parameter("poll_period_s").value
        self._reconnect_interval = self.get_parameter(
            "reconnect_interval_s"
        ).value

        self._validate_parameters()

        self._transport = SerialTransport(
            port=self._port,
            baudrate=115200,
            read_limit=4096,
            tx_timeout=0.2,
        )

        # 串口成功打开后才创建客户端，避免离线期间累计发送任务。
        self._client = None

        self._next_open_time = 0.0
        self._open_attempts = 0
        self._closing = False

        # 避免每次HELLO都重复打印相同信息。
        self._last_hello = None
        self._last_status = None

        # 对重复故障日志进行简单限频。
        self._last_fault_key = None
        self._last_fault_time = 0.0

        # 使用单调时钟驱动通信，不受use_sim_time和系统时间调整影响。
        self._communication_clock = Clock(
            clock_type=ClockType.STEADY_TIME
        )
        self._timer = self.create_timer(
            self._poll_period,
            self._tick,
            clock=self._communication_clock,
        )

        self.get_logger().info(
            f"雷达节点启动：port={self._port}, "
            f"baudrate=115200, node_id={self._node_id}, "
            f"zone={self._zone}"
        )

        if self._zone == 0:
            self.get_logger().info(
                "未主动指定工作区域：若雷达已有有效区域则沿用；"
                "若返回zone=0，将保持等待选区"
            )

    def _declare_read_only(self, name, default, description) -> None:
        """声明启动参数，禁止运行中只改参数而未重配串口。"""
        self.declare_parameter(
            name,
            default,
            ParameterDescriptor(
                description=description,
                read_only=True,
            ),
        )

    def _validate_parameters(self) -> None:
        """检查参数，配置错误应直接报告。"""
        if not isinstance(self._port, str) or not self._port.strip():
            raise ValueError("port必须是非空串口路径")

        if not 1 <= self._node_id <= 254:
            raise ValueError("node_id必须为1～254")

        if self._zone not in (0, 1, 2):
            raise ValueError("zone必须为0、1或2")

        if (
            not math.isfinite(self._poll_period)
            or not 0.001 <= self._poll_period <= 0.01
        ):
            raise ValueError(
                "poll_period_s必须在0.001～0.01秒之间"
            )

        if (
            not math.isfinite(self._reconnect_interval)
            or self._reconnect_interval < 0.1
        ):
            raise ValueError(
                "reconnect_interval_s必须为不小于0.1的有限数值"
            )

    def _try_open(self) -> None:
        """到达重试时间时尝试打开一次，不循环等待。"""
        now = time.monotonic()

        if now < self._next_open_time:
            return

        self._open_attempts += 1

        try:
            self._transport.open()

        except TransportError as exc:
            self._next_open_time = (
                time.monotonic() + self._reconnect_interval
            )
            self.get_logger().warning(
                f"第{self._open_attempts}次打开串口失败：{exc}；"
                f"{self._reconnect_interval:.1f}秒后重试"
            )
            return

        # 当前节点不发起业务，因此每次重新打开串口时可以建立新客户端。
        # 以后添加扫描/校准Action时，还需要通知旧任务失败或取消。
        self._client = RadarClient(
            node_id=self._node_id,
            zone=self._zone,
        )

        self._open_attempts = 0
        self._last_hello = None
        self._last_status = None
        self._last_fault_key = None

        self.get_logger().info(
            f"串口已打开：{self._port}，开始雷达握手"
        )

    def _tick(self) -> None:
        """执行一次有界通信轮询。"""
        if self._closing:
            return

        if not self._transport.is_open:
            self._try_open()
            return

        client = self._client
        if client is None:
            return

        try:
            # 1. 读取当前可用数据，不等待完整协议帧。
            data = self._transport.read_available()
            if data:
                client.feed(data)

            # 2. 推进握手、心跳、健康检查和超时处理。
            client.poll()

            # 3. 当前帧写完后才取下一帧。
            #    客户端会优先交出STOP，不能提前批量取帧。
            if not self._transport.tx_pending:
                frame = client.take_tx()
                if frame is not None:
                    self._transport.enqueue(frame)

            # 4. 非阻塞地尝试写入当前帧剩余数据。
            self._transport.flush_tx()

        except TransportError as exc:
            self._handle_transport_failure(str(exc))
            return

        # 5. 每轮及时消费所有客户端事件。
        for event in client.take_events():
            self._handle_event(event)

        # 客户端明确要求重建时，通过关闭再打开清理连接。
        # 普通协议超时不走这里，由RadarClient自行重新握手。
        if self._client is client and client.state == LinkState.FAULT:
            self._handle_transport_failure(
                "客户端进入FAULT，关闭串口后重新建立连接"
            )

    def _handle_transport_failure(self, reason: str) -> None:
        """清理故障连接并安排重试，不在此处阻塞等待。"""
        self._transport.close()

        if self._client is not None:
            self._client.transport_lost(reason)

            # 本阶段没有业务任务，旧会话事件直接废弃。
            self._client.take_events()
            self._client = None

        self._next_open_time = (
            time.monotonic() + self._reconnect_interval
        )
        self._last_hello = None
        self._last_status = None

        self.get_logger().error(
            f"雷达串口连接中断：{reason}；"
            f"{self._reconnect_interval:.1f}秒后尝试重新打开"
        )

    def _handle_event(self, event) -> None:
        """处理客户端事件，目前仅打印日志。"""
        kind = event.kind
        data = event.data

        if kind == "link":
            self._handle_link_state(data)

        elif kind == "hello":
            # dataclass支持按字段比较，信息不变时不重复打印。
            if data != self._last_hello:
                self._last_hello = data
                pose = data.pose

                self.get_logger().info(
                    f"设备信息：固件={data.firmware_major}."
                    f"{data.firmware_minor}, "
                    f"协议={data.protocol}, "
                    f"区域={pose.zone}, "
                    f"X={pose.x_mm}mm, Y={pose.y_mm}mm, "
                    f"朝向={pose.heading_q6 / 64.0:.2f}deg, "
                    f"采集运行={data.running}, "
                    f"单圈点数上限={data.max_scan_points}"
                )

        elif kind == "persisted":
            self.get_logger().info(
                f"保存状态：版本={data.storage_version}, "
                f"记录序号={data.record_sequence}, "
                f"最近修改区域={data.source_zone}, "
                f"方格记录有效={data.grid_result_valid}, "
                f"保存方格数量={data.grid_count}"
            )

        elif kind == "status":
            if data != self._last_status:
                self._last_status = data

                self.get_logger().info(
                    f"设备状态：status={data.status}, "
                    f"mode={data.mode}, "
                    f"running={data.running}, "
                    f"圈数={data.frame_count}, "
                    f"overflow={data.overflow}"
                )

        elif kind == "fault":
            self._log_fault(data)

        elif kind == "device_error":
            self.get_logger().error(
                f"雷达设备错误：code={data.code}, "
                f"detail=0x{data.detail:04X}"
            )

        elif kind == "stopped":
            self.get_logger().info("雷达已确认停止当前操作")

        elif kind == "cancelled":
            self.get_logger().warning(
                f"客户端事务已取消：{data}"
            )

        else:
            # 本节点不主动发起扫描或校准。
            self.get_logger().debug(
                f"收到客户端事件：{kind}"
            )

    def _handle_link_state(self, state: LinkState) -> None:
        """打印链路状态变化。"""
        if state == LinkState.READY:
            zone = self._client.selected_zone
            self.get_logger().info(
                f"雷达连接就绪，已确认工作区域={zone}；"
                "当前仅保持连接，不启动扫描"
            )

        elif state == LinkState.NEED_ZONE:
            self.get_logger().warning(
                "雷达握手及保存状态读取完成，但尚未选择工作区域。"
                "本版本请按实际位置，以-p zone:=1或-p zone:=2"
                "重新启动节点"
            )

        elif state == LinkState.WAIT_HELLO:
            self._last_hello = None
            self._last_status = None
            self.get_logger().info(
                "等待雷达HELLO和ACK；串口保持打开，客户端后台重试"
            )

        elif state == LinkState.READING_STATE:
            self.get_logger().info("正在读取雷达保存状态")

        elif state == LinkState.SELECTING_ZONE:
            self.get_logger().info(
                f"正在确认工作区域：{self._zone}"
            )

        elif state == LinkState.STOPPING:
            self.get_logger().warning("正在等待雷达停止确认")

        elif state == LinkState.FAULT:
            self.get_logger().error("雷达客户端进入故障状态")

        else:
            self.get_logger().info(
                f"雷达链路状态：{state.value}"
            )

    def _log_fault(self, fault) -> None:
        """相同故障最多每两秒打印一次，避免串口噪声刷屏。"""
        key = (fault.kind, fault.message, fault.command)
        now = time.monotonic()

        if (
            key == self._last_fault_key
            and now - self._last_fault_time < 2.0
        ):
            return

        self._last_fault_key = key
        self._last_fault_time = now

        self.get_logger().warning(
            f"客户端故障：kind={fault.kind}, "
            f"command={fault.command}, "
            f"message={fault.message}"
        )

    def close(self) -> None:
        """停止轮询并关闭串口，可重复调用。"""
        if self._closing:
            return

        self._closing = True
        self._timer.cancel()
        self._transport.close()
        self._client = None


def main(args=None) -> None:
    """ROS 2入口。"""
    rclpy.init(args=args)
    node = None

    try:
        node = RadarNode()

        # 使用默认单线程执行器，避免并发操作客户端和串口。
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            node.close()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()