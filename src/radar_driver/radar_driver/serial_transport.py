"""雷达串口传输层，不依赖 ROS。

配置：115200、8N1、无流控，非阻塞读写。

使用要求：
1. 所有方法由同一个线程调用。
2. 每次只接收一条待发送帧，禁止提前取走客户端的大量帧。
3. 通信异常时关闭串口并清除本地发送缓存。
4. 调用方捕获 TransportError 后，调用 client.transport_lost()。
5. 自动重连、心跳和业务事务由上层负责。
"""

import math
import os
import time
from typing import Callable

import serial


class TransportError(RuntimeError):
    """串口打开、读取或发送失败。"""


class SerialTransport:
    """有界、非阻塞的雷达串口传输层。

    tx_timeout：
        单帧交给操作系统的最长允许时间。
        不代表设备响应超时，也不代表数据已被雷达接收。

    read_limit：
        单次读取的最大字节数，避免一次轮询占用过久。
    """

    # 协议最大原始帧235字节，COBS编码后加分隔符最多237字节。
    MAX_WIRE_SIZE = 237

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        read_limit: int = 4096,
        tx_timeout: float = 0.2,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not isinstance(port, str) or not port:
            raise ValueError("port 必须是非空串口路径")

        if baudrate != 115200:
            raise ValueError("当前雷达协议要求波特率为115200")

        if read_limit <= 0:
            raise ValueError("read_limit 必须大于0")

        if not math.isfinite(tx_timeout) or tx_timeout <= 0:
            raise ValueError("tx_timeout 必须是有限正数")

        self.port = port
        self.baudrate = baudrate
        self.read_limit = read_limit
        self.tx_timeout = tx_timeout
        self._clock = clock

        self._serial: serial.Serial | None = None

        self._tx_data = b""
        self._tx_offset = 0
        self._tx_started = 0.0

        # 累计统计，仅表示本程序读写的字节数。
        self.rx_bytes = 0
        self.tx_bytes = 0

    @property
    def is_open(self) -> bool:
        """串口是否已打开，不代表雷达握手成功。"""
        return self._serial is not None and self._serial.is_open

    @property
    def tx_pending(self) -> bool:
        """是否仍有尚未交给操作系统的发送数据。"""
        return self._tx_offset < len(self._tx_data)

    def open(self) -> None:
        """打开串口，清理旧缓冲，并安排发送帧分隔符。"""
        if self.is_open:
            return

        self.close()

        try:
            # port=None：先创建对象，再显式打开。
            self._serial = serial.Serial(
                port=None,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                write_timeout=0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )

            if os.name == "posix":
                self._serial.exclusive = True

            self._serial.port = self.port
            self._serial.open()

            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()

        except (serial.SerialException, OSError, ValueError) as exc:
            self.close()
            raise TransportError(
                f"打开串口 {self.port} 失败：{exc}"
            ) from exc

        # 断线前可能只发送了一部分帧。
        # 首先发送00，结束设备端可能残留的半帧。
        # 空分隔符本身不是雷达业务命令。
        self._tx_data = b"\x00"
        self._tx_offset = 0
        self._tx_started = self._clock()

    def close(self) -> None:
        """关闭串口并丢弃本地待发送数据。

        关闭串口不等于向雷达发送了STOP。
        正常结束业务应由上层先处理STOP事务。
        """
        port = self._serial
        self._serial = None
        self._clear_tx()

        if port is not None:
            try:
                port.close()
            except (serial.SerialException, OSError):
                # 设备已拔出时，关闭也可能失败。
                # 本地引用和待发送数据仍然必须清除。
                pass

    def read_available(self) -> bytes:
        """读取当前已有数据，不等待完整帧。

        返回b''表示本次没有读取到数据，不表示设备断线。
        """
        port = self._require_open()

        try:
            count = min(port.in_waiting, self.read_limit)

            if count <= 0:
                return b""

            data = port.read(count)

        except (serial.SerialException, OSError) as exc:
            self._abort("读取", exc)

        self.rx_bytes += len(data)
        return data

    def enqueue(self, frame: bytes) -> None:
        """接收一条完整的COBS编码帧，包含结尾00。

        这里只保存数据，不执行实际发送。
        当前帧没有发送完时，不允许加入下一帧。
        """
        self._require_open()

        if self.tx_pending:
            raise RuntimeError("当前帧尚未发送完成，不能加入下一帧")

        if not isinstance(frame, (bytes, bytearray, memoryview)):
            raise TypeError("frame 必须是字节数据")

        frame = bytes(frame)

        if not 2 <= len(frame) <= self.MAX_WIRE_SIZE:
            raise ValueError("发送帧长度不合法")

        if frame[-1] != 0 or 0 in frame[:-1]:
            raise ValueError("发送帧必须为COBS数据加结尾00")

        self._tx_data = frame
        self._tx_offset = 0
        self._tx_started = self._clock()

    def flush_tx(self) -> int:
        """尝试发送当前帧的剩余部分，返回本次写入字节数。

        每次最多调用一次write()，不忙等待。
        部分写入后，下次从剩余位置继续发送。

        此方法不调用pySerial.flush()，不等待物理发送完成。
        """
        port = self._require_open()

        if not self.tx_pending:
            return 0

        if self._clock() - self._tx_started >= self.tx_timeout:
            self.close()
            raise TransportError(
                "单帧发送超时，已关闭串口并清除发送缓存"
            )

        remaining = self._tx_data[self._tx_offset:]

        try:
            written = port.write(remaining)

        except (serial.SerialException, OSError) as exc:
            # 写入异常时，无法可靠知道本次实际发送了多少字节。
            # 关闭连接，不能从头重发而产生重复或混合帧。
            self._abort("发送", exc)

        if (
            not isinstance(written, int)
            or not 0 <= written <= len(remaining)
        ):
            self.close()
            raise TransportError("串口返回了非法写入长度")

        self._tx_offset += written
        self.tx_bytes += written

        if not self.tx_pending:
            self._clear_tx()

        return written

    def _require_open(self) -> serial.Serial:
        """取得已打开的串口对象。"""
        if not self.is_open:
            raise TransportError("串口未打开")

        return self._serial

    def _clear_tx(self) -> None:
        """清除本地发送状态。"""
        self._tx_data = b""
        self._tx_offset = 0
        self._tx_started = 0.0

    def _abort(self, operation: str, error: Exception) -> None:
        """清理连接后，向上层报告传输错误。"""
        self.close()
        raise TransportError(
            f"串口{operation}失败：{error}"
        ) from error