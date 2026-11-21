"""V0.1 wire protocol. No ROS/serial dependency; all wire scalars little endian."""
from dataclasses import dataclass
from enum import IntEnum
import math
import struct


class Kind(IntEnum):
    ROTATE_TO = 1
    MOVE_DISTANCE = 2
    STOP = 3
    QUERY_STATUS = 4
    HEARTBEAT = 5
    ACK = 0x81
    MOTION_STATE = 0x82
    ODOMETRY = 0x83
    BASE_STATUS = 0x84


class State(IntEnum):
    ACCEPTED = 0
    RUNNING = 1
    SUCCEEDED = 2
    CANCELED = 3
    FAILED = 4


class Reason(IntEnum):
    OK = 0
    BUSY = 1
    INVALID_PARAMETER = 2
    SESSION_MISMATCH = 3
    BOOT_MISMATCH = 4
    MOTOR_FAULT = 5
    IMU_INVALID = 6
    ENCODER_INVALID = 7
    STALL = 8
    MOTION_TIMEOUT = 9
    CANCELED_BY_STOP = 10
    HEARTBEAT_TIMEOUT = 11
    UNKNOWN_COMMAND = 12
    COMMAND_ID_CONFLICT = 13
    OLD_COMMAND = 14
    BASE_FAULT = 15
    UNSUPPORTED = 16
    NOT_ALIGNED = 17


HEADER = struct.Struct('<BBHIIQI')
FORMATS = {
    Kind.ROTATE_TO: struct.Struct('<ffffI'),
    Kind.MOVE_DISTANCE: struct.Struct('<ffffffI'),
    Kind.STOP: struct.Struct('<HH'),
    Kind.QUERY_STATUS: struct.Struct('<'),
    Kind.HEARTBEAT: struct.Struct('<Q'),
    Kind.ACK: struct.Struct('<BBH'),
    Kind.MOTION_STATE: struct.Struct('<BBHHHI8f'),
    Kind.ODOMETRY: struct.Struct('<QHH5f'),
    Kind.BASE_STATUS: struct.Struct('<QBBHIIfHH'),
}


def crc16(data):
    value = 0xffff
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ 0x1021) & 0xffff if value & 0x8000 else (value << 1) & 0xffff
    return value


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class Frame:
    kind: int
    session: int
    seq: int
    boot: int
    command: int
    payload: bytes

    def values(self):
        return FORMATS[self.kind].unpack(self.payload)

    def encode(self):
        if len(self.payload) != FORMATS[self.kind].size:
            raise ValueError('incorrect payload size')
        body = HEADER.pack(1, self.kind, len(self.payload), self.session,
                           self.seq, self.boot, self.command) + self.payload
        return b'\xaa\x55' + body + struct.pack('<H', crc16(body))


def payload(kind, *values):
    return FORMATS[kind].pack(*values)


@dataclass(frozen=True)
class Motion:
    kind: int
    yaw: float
    distance: float = 0.0
    speed: float = 0.2
    acceleration: float = 0.2
    angular_speed: float = 0.5
    angular_acceleration: float = 0.5
    distance_tolerance: float = 0.01
    yaw_tolerance: float = 0.03
    timeout_ms: int = 30000

    def validate(self, limits=None):
        # 与当前 STM32 固件一致的硬上限。
        hard_caps = {
            'distance': 5.0,
            'speed': 0.20,
            'acceleration': 0.20,
            'angular_speed': 1.5707964,
            'angular_acceleration': 1.3962634,
            'timeout_ms': 120000,
        }

        caps = hard_caps.copy()

        # YAML 可以进一步收紧限制，但不能放宽固件上限。
        for name, value in (limits or {}).items():
            if name in caps:
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f'invalid limit: {name}')
                caps[name] = min(value, hard_caps[name])

        if self.kind not in (Kind.ROTATE_TO, Kind.MOVE_DISTANCE):
            raise ValueError('unsupported motion type')

        for name in (
            'yaw', 'distance', 'speed', 'acceleration',
            'angular_speed', 'angular_acceleration',
            'distance_tolerance', 'yaw_tolerance'
        ):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f'{name} must be finite')

        if (
            not isinstance(self.timeout_ms, int)
            or not 100 <= self.timeout_ms <= caps['timeout_ms']
        ):
            raise ValueError('timeout outside firmware limits')

        # 按实际发送的 float32 数值检查，避免 Python 双精度边界差异。
        def f32(value):
            return struct.unpack('<f', struct.pack('<f', value))[0]

        def check(name, value, low, high):
            if not f32(low) <= value <= f32(high):
                raise ValueError(f'{name} outside firmware limits')

        values = FORMATS[self.kind].unpack(self.encode())

        if not all(math.isfinite(value) for value in values):
            raise ValueError('non-finite wire value')

        if self.kind == Kind.ROTATE_TO:
            yaw, speed, acceleration, yaw_tol, timeout = values

            check('angular_speed', speed, 0.08726646,
                caps['angular_speed'])
            check('angular_acceleration', acceleration, 0.08726646,
                caps['angular_acceleration'])
            check('yaw_tolerance', yaw_tol, 0.008726646, 0.08726647)

        else:
            distance, yaw, speed, acceleration, distance_tol, yaw_tol, timeout = values

            check('distance', distance, 0.02, caps['distance'])
            check('speed', speed, 0.03, caps['speed'])
            check('acceleration', acceleration, 0.03,
                caps['acceleration'])
            check('distance_tolerance', distance_tol, 0.005, 0.05)
            check('yaw_tolerance', yaw_tol, 0.008726646, 0.08726647)

            if distance_tol >= distance:
                raise ValueError('distance tolerance must be smaller than distance')

        if not -f32(math.pi) <= yaw < f32(math.pi):
            raise ValueError('wire yaw outside [-pi, pi)')

    def encode(self):
        if self.kind == Kind.ROTATE_TO:
            return payload(self.kind, wrap(self.yaw), self.angular_speed,
                           self.angular_acceleration, self.yaw_tolerance, self.timeout_ms)
        return payload(self.kind, self.distance, wrap(self.yaw), self.speed,
                       self.acceleration, self.distance_tolerance,
                       self.yaw_tolerance, self.timeout_ms)

    @classmethod
    def decode(cls, frame):
        v = frame.values()
        if frame.kind == Kind.ROTATE_TO:
            return cls(frame.kind, v[0], angular_speed=v[1], angular_acceleration=v[2],
                       yaw_tolerance=v[3], timeout_ms=v[4])
        return cls(frame.kind, v[1], distance=v[0], speed=v[2], acceleration=v[3],
                   distance_tolerance=v[4], yaw_tolerance=v[5], timeout_ms=v[6])
