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
        caps = {'distance': 10.0, 'speed': 0.5, 'acceleration': 1.0,
                'angular_speed': 1.5, 'angular_acceleration': 3.0,
                'timeout_ms': 120000}
        if limits:
            caps.update(limits)
        if self.kind not in (Kind.ROTATE_TO, Kind.MOVE_DISTANCE):
            raise ValueError('unsupported motion type')
        for name in ('yaw', 'distance', 'speed', 'acceleration', 'angular_speed',
                     'angular_acceleration', 'distance_tolerance', 'yaw_tolerance'):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f'{name} must be finite')
        if not 0 < self.timeout_ms <= caps['timeout_ms']:
            raise ValueError('timeout outside configured limits')
        if not 0 < self.yaw_tolerance <= 0.2:
            raise ValueError('yaw tolerance must be in (0, 0.2] rad')
        names = ('angular_speed', 'angular_acceleration') if self.kind == 1 else (
            'distance', 'speed', 'acceleration')
        for name in names:
            if not 0 < getattr(self, name) <= caps[name]:
                raise ValueError(f'{name} outside configured limits')
        if self.kind == 2 and not 0 < self.distance_tolerance <= min(0.05, self.distance):
            raise ValueError('distance tolerance outside allowed range')
        # Verify float32 conversion, including nonzero tiny tolerances/speeds.
        data = self.encode()
        values = FORMATS[self.kind].unpack(data)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('float32 overflow')
        positive_indices = (1, 2, 3) if self.kind == 1 else (0, 2, 3, 4, 5)
        if any(values[i] <= 0 for i in positive_indices):
            raise ValueError('float32 underflow')

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
