"""雷达应用协议 v6。

线上格式：
    COBS(帧头 + 载荷 + CRC16) + 00

本模块负责：
1. CRC16 校验。
2. COBS 编码和解码。
3. 发送帧组装。
4. 接收帧解析。
5. 拆包、粘包及坏帧后的恢复。

节点过滤、命令白名单和业务事务管理由 client 层负责。
"""

from dataclasses import dataclass
from enum import IntEnum
import struct


# ==================== 协议常量 ====================

MAGIC = b"LR"
VERSION = 6
MAX_PAYLOAD = 224

# 小端格式：
# 2s：帧标识
# B ：协议版本
# B ：节点 ID
# B ：消息类型
# H ：发送序号
# H ：载荷长度
HEADER = struct.Struct("<2sBBBHH")

# CRC 按小端 uint16 存储
CRC_FIELD = struct.Struct("<H")

# 最大原始帧长度：9 + 224 + 2 = 235 字节
MAX_RAW_FRAME = HEADER.size + MAX_PAYLOAD + CRC_FIELD.size

# 最大 COBS 编码长度，不包含最后的 00 分隔符
MAX_ENCODED_FRAME = (
    MAX_RAW_FRAME + MAX_RAW_FRAME // 254 + 1
)


# ==================== 命令和事件 ====================

class Command(IntEnum):
    """控制端发送给雷达的命令。"""

    HELLO = 0x01
    HEARTBEAT = 0x02
    START = 0x03
    STOP = 0x04
    SELECT_ZONE = 0x05
    SET_POSE = 0x06
    SCAN_LOCKED = 0x07
    SINGLE_MEASURE = 0x08
    CALIBRATE = 0x09
    CLEAR = 0x0A
    RESET_CALIBRATION = 0x0B
    SAVE_POSE = 0x11
    GET_PERSISTED_STATE = 0x13
    PLAN_PATH = 0x14
    SET_NODE_ID = 0x17
    SET_POINT_CLOUD_OUTPUT = 0x19


class Event(IntEnum):
    """雷达返回给控制端的事件。"""

    HELLO = 0x81
    ACK = 0x82
    STATUS = 0x83
    SCAN_BEGIN = 0x84
    SCAN_POINTS = 0x85
    SCAN_END = 0x86
    RESULT = 0x87
    CALIBRATION = 0x88
    STATS = 0x89
    ERROR = 0x8A
    OBSTACLE_GRID = 0x8D
    PERSISTED_STATE = 0x8E
    PATH_RESULT = 0x8F


# ==================== 数据结构和异常 ====================

class ProtocolError(ValueError):
    """编码、帧头、长度或者 CRC 不合法。"""


@dataclass(frozen=True)
class Frame:
    """一条通过协议检查的消息。"""

    node_id: int
    message_type: int
    sequence: int
    payload: bytes


# ==================== CRC16 ====================

def crc16_ccitt(data: bytes) -> int:
    """计算 CRC16-CCITT。

    初值：0xFFFF
    多项式：0x1021
    不反射，无输出异或。
    """
    crc = 0xFFFF

    for value in data:
        crc ^= value << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


# ==================== COBS 编解码 ====================

def cobs_encode(data: bytes) -> bytes:
    """将原始数据编码成不含零字节的数据段。

    此函数不追加帧末尾的 00。
    """
    output = bytearray([0])

    code_index = 0
    code = 1

    for value in data:
        if value == 0:
            # 当前非零数据块结束，填写块长度标记
            output[code_index] = code

            # 为下一个数据块预留长度标记
            code_index = len(output)
            output.append(0)
            code = 1

        else:
            output.append(value)
            code += 1

            # 一个块最多容纳 254 个非零字节
            if code == 0xFF:
                output[code_index] = code

                code_index = len(output)
                output.append(0)
                code = 1

    output[code_index] = code

    return bytes(output)


def cobs_decode(encoded: bytes) -> bytes:
    """解码一个 COBS 数据段。

    输入不得包含帧末尾的 00。
    """
    if not encoded or 0 in encoded:
        raise ProtocolError(
            "COBS 编码段为空或包含零字节"
        )

    output = bytearray()
    offset = 0

    while offset < len(encoded):
        code = encoded[offset]
        offset += 1

        end = offset + code - 1

        if end > len(encoded):
            raise ProtocolError("COBS 块长度越界")

        output.extend(encoded[offset:end])
        offset = end

        # 非 0xFF 块之间恢复被编码掉的零字节
        if code != 0xFF and offset < len(encoded):
            output.append(0)

    return bytes(output)


# ==================== 帧编码和解码 ====================

def _validate_address(
    node_id: int,
    message_type: int,
    payload: bytes,
) -> None:
    """检查节点地址及广播限制。"""
    if not 0 <= node_id <= 254:
        raise ProtocolError("节点 ID 必须为 0～254")

    if node_id == 0:
        if message_type != Command.STOP or payload:
            raise ProtocolError(
                "节点0仅允许空载荷广播 STOP"
            )


def encode_frame(
    node_id: int,
    message_type: int,
    sequence: int,
    payload: bytes = b"",
) -> bytes:
    """构造可直接交给串口发送的完整帧。

    sequence 由客户端管理，本函数不自动递增。
    """
    if not isinstance(
        payload,
        (bytes, bytearray, memoryview),
    ):
        raise TypeError("payload 必须是字节数据")

    payload = bytes(payload)

    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError("载荷超过224字节")

    if not 0 <= message_type <= 255:
        raise ProtocolError("消息类型超出范围")

    if not 0 <= sequence <= 65535:
        raise ProtocolError("序号超出范围")

    _validate_address(node_id, message_type, payload)

    header = HEADER.pack(
        MAGIC,
        VERSION,
        node_id,
        message_type,
        sequence,
        len(payload),
    )

    body = header + payload

    crc = crc16_ccitt(body)
    raw = body + CRC_FIELD.pack(crc)

    return cobs_encode(raw) + b"\x00"


def decode_frame(encoded: bytes) -> Frame:
    """解析一个不含末尾 00 的完整编码段。"""
    if len(encoded) > MAX_ENCODED_FRAME:
        raise ProtocolError("编码帧过长")

    raw = cobs_decode(encoded)

    minimum_length = HEADER.size + CRC_FIELD.size

    if not minimum_length <= len(raw) <= MAX_RAW_FRAME:
        raise ProtocolError("原始帧长度非法")

    (
        magic,
        version,
        node_id,
        message_type,
        sequence,
        payload_length,
    ) = HEADER.unpack_from(raw)

    if magic != MAGIC:
        raise ProtocolError("帧标识错误")

    if version != VERSION:
        raise ProtocolError("协议版本错误")

    expected_length = (
        HEADER.size + payload_length + CRC_FIELD.size
    )

    if (
        payload_length > MAX_PAYLOAD
        or len(raw) != expected_length
    ):
        raise ProtocolError("载荷长度不匹配")

    received_crc = CRC_FIELD.unpack_from(
        raw,
        len(raw) - CRC_FIELD.size,
    )[0]

    calculated_crc = crc16_ccitt(
        raw[:-CRC_FIELD.size]
    )

    if calculated_crc != received_crc:
        raise ProtocolError("CRC 校验失败")

    payload = raw[HEADER.size:-CRC_FIELD.size]

    _validate_address(node_id, message_type, payload)

    return Frame(
        node_id=node_id,
        message_type=message_type,
        sequence=sequence,
        payload=payload,
    )


# ==================== 增量接收器 ====================

class FrameDecoder:
    """处理串口任意分块输入。

    每条串口使用一个长期存在的实例。

    feed() 每次可以返回零条、一条或多条消息。
    坏帧计入 bad_frames，并记录 last_error。
    超长编码段丢弃到下一个 00，避免缓存无限增长。
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._discarding = False

        self.good_frames = 0
        self.bad_frames = 0
        self.last_error = ""

    def reset_partial(self) -> None:
        """清除半帧状态，不清空诊断计数。"""
        self._buffer.clear()
        self._discarding = False

    def feed(self, data: bytes) -> list[Frame]:
        """输入新收到的字节，返回本次解析出的合法帧。"""
        frames = []

        for value in data:
            if value == 0:
                # 超长段结束，恢复正常接收
                if self._discarding:
                    self.reset_partial()
                    continue

                # 忽略连续出现的空分隔符
                if not self._buffer:
                    continue

                try:
                    frame = decode_frame(
                        bytes(self._buffer)
                    )

                except ProtocolError as error:
                    self.bad_frames += 1
                    self.last_error = str(error)

                else:
                    self.good_frames += 1
                    frames.append(frame)

                finally:
                    self._buffer.clear()

            elif not self._discarding:
                if len(self._buffer) >= MAX_ENCODED_FRAME:
                    self._buffer.clear()
                    self._discarding = True

                    self.bad_frames += 1
                    self.last_error = (
                        "编码帧过长，丢弃到下一分隔符"
                    )

                else:
                    self._buffer.append(value)

        return frames