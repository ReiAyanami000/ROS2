"""Deterministic, ROS-independent bridge state machine. Call from ONE event loop."""
from dataclasses import dataclass
import math

from .protocol import Frame, Kind, Motion, State, payload, wrap
from .session_manager import Session

STOPPED_ANGULAR_SPEED = math.radians(2.0)
STOPPED_LINEAR_SPEED = 0.015

@dataclass
class Pending:
    command: int
    kind: int
    data: bytes
    started: float
    sent: float
    deadline: float
    retries: int = 0
    acknowledged: bool = False
    feedback: tuple = None
    terminal: tuple = None
    terminal_time: float = -1e9


class MotionClient:
    def __init__(self, sender, *, heartbeat_period=0.1, status_timeout=0.6,
                 odom_timeout=0.3, ack_timeout=0.25, max_retries=2,
                 query_period=0.4, stop_timeout=4.0, limits=None):
        self.sender = sender
        self.heartbeat_period = heartbeat_period
        self.status_timeout, self.odom_timeout = status_timeout, odom_timeout
        self.ack_timeout, self.max_retries = ack_timeout, max_retries
        self.query_period, self.stop_timeout = query_period, stop_timeout
        self.limits = limits or {}
        self.session = Session()
        self.port_open = False
        self.seq = self.next_command = 0
        self.heartbeat_sent = self.query_sent = -1e9
        self.motion = self.stop = None
        self.odom = self.status = None
        self.odom_version = self.status_version = 0
        self.outcomes = []
        self.locked = False
        self.message = 'waiting for transport'
        self.stop_state = 'NONE'
        self.cancel_requested = False
        self.failure = None

    def open(self, now):
        self.port_open = True
        self.session = Session()
        self.status = self.odom = None
        self.next_command = 0
        self.heartbeat_sent = -1e9
        self.message = 'discovering STM32'

    def disconnect(self, reason, now):
        self.port_open = False
        self.session.bound = False
        if self.motion:
            self.locked = True
            self._finish('failed', 1, 0, False, 'connection lost: ' + reason)
        if self.stop:
            self.locked = True
            self.stop_state = 'UNCONFIRMED'
            self.stop = None
        self.status = self.odom = None
        self.message = reason

    def _send(self, kind, command, data, priority=False):
        if not self.port_open:
            raise ConnectionError('port closed')
        self.seq = (self.seq + 1) & 0xffffffff
        self.sender(Frame(kind, self.session.session, self.seq, self.session.boot,
                          command, data), priority)

    def _allocate(self):
        if self.next_command >= 0xfffffffe:
            self.locked = True
            raise ValueError('command IDs exhausted; restart while stopped')
        self.next_command += 1
        return self.next_command

    def fresh(self, now):
        return (self.port_open and self.session.bound
                and now - self.session.status_time <= self.status_timeout
                and now - self.session.odom_time <= self.odom_timeout
                and self.status is not None and self.odom is not None)

    def idle(self, now, after=-1e9):
        return (self.fresh(now) and self.session.status_time >= after
                and self.session.odom_time >= after
                and self.status[1] == 0 and self.status[4] == 0
                and self.status[5] == 0 and (self.status[3] & 0x1e) == 0x1c
                and abs(self.odom[6]) <= STOPPED_LINEAR_SPEED and abs(self.odom[7]) <= STOPPED_ANGULAR_SPEED)

    def ready(self, now):
        return not self.locked and self.motion is None and self.stop is None and self.idle(now)

    def begin(self, motion, now):
        motion.validate(self.limits)
        if not self.ready(now):
            raise ValueError('not ready: connection, stale data, fault, lock or busy')
        if motion.kind == Kind.MOVE_DISTANCE and abs(wrap(motion.yaw - self.odom[5])) > motion.yaw_tolerance:
            raise ValueError('NOT_ALIGNED: rotate to target yaw first')
        command = self._allocate()
        data = motion.encode()
        self.motion = Pending(command, motion.kind, data, now, now,
                              now + motion.timeout_ms / 1000.0 + 0.5)
        self.cancel_requested = False
        self.failure = None
        self._send(motion.kind, command, data)
        self.message = 'waiting for STM32 ACK'
        return command

    def request_stop(self, now, reason=1, cancel=False):
        if reason not in (1, 2, 3):
            raise ValueError('stop reason must be 1, 2 or 3')
        if not self.port_open:
            raise ConnectionError('port closed: stop not submitted')
        self.cancel_requested |= cancel
        if self.stop:
            return self.stop.command
        command = self._allocate()
        data = payload(Kind.STOP, reason, 0)
        self.stop = Pending(command, Kind.STOP, data, now, now, now + self.stop_timeout)
        self.stop_state = 'PENDING'
        self._send(Kind.STOP, command, data, True)
        return command

    def _fault(self, code, message, now, stm32=0):
        self.locked = True
        if self.failure is None:
            self.failure = (code, stm32, message)
        if self.port_open:
            self.request_stop(now, 2)
        elif self.motion:
            self._finish('failed', code, stm32, False, message)

    def _finish(self, outcome, error, stm32_error, stopped, message):
        if self.motion:
            self.outcomes.append(dict(command_id=self.motion.command, outcome=outcome,
                                      bridge_error=error, stm32_error=stm32_error,
                                      stopped_confirmed=stopped, message=message))
        self.motion = None
        self.failure = None
        self.cancel_requested = False
        self.message = message

    def receive(self, frame, now):
        if frame.kind not in (Kind.ACK, Kind.MOTION_STATE, Kind.ODOMETRY, Kind.BASE_STATUS):
            return
        if frame.kind == Kind.BASE_STATUS and frame.boot and frame.session in (0, self.session.session):
            if self.session.boot and frame.boot != self.session.boot:
                self.locked = True
                if self.motion:
                    self._finish('failed', 1, 0, False, 'STM32 reboot: initial alignment invalid')
                self.stop = None
                self.stop_state = 'UNCONFIRMED'
                self.session = Session()
                self.status = self.odom = None
                self.heartbeat_sent = -1e9
                self.message = 'STM32 reboot; restart bridge after checking position'
                return
            if not self.session.boot:
                # Only discovery reply to our fresh session can establish identity.
                if frame.session != self.session.session:
                    return
                self.session.boot = frame.boot
        if not self.session.accepts(frame) or not self.session.forward_sequence(frame.seq):
            return
        v = frame.values()
        if frame.kind == Kind.BASE_STATUS:
            if v[1] > 4 or v[2] not in (0, 1, 2, 3) or v[8] != 0 or not math.isfinite(v[6]):
                return
            was_bound = self.session.bound
            self.session.bound = bool(v[3] & 64)
            self.status = v
            self.session.status_time = now
            self.status_version += 1
            if was_bound and not self.session.bound and self.motion:
                self._fault(1, 'session binding lost', now)
            if self.motion and (v[1] == 4 or v[5] or (v[3] & 0x1e) != 0x1c):
                self._fault(5, 'base fault or sensors invalid', now)
        elif frame.kind == Kind.ODOMETRY:
            if not self.session.bound:
                return
            if v[1] != 15 or v[2] != 0 or not all(math.isfinite(x) for x in v[3:]):
                self.session.odom_time = -1e9
                self.odom = None
                if self.motion:
                    self._fault(5, 'invalid odometry', now)
                return
            if v[0] <= self.session.sample_time:
                self.session.odom_time = -1e9
                if self.motion:
                    self._fault(5, 'odometry time did not increase', now)
                return
            self.session.sample_time = v[0]
            self.session.odom_time = now
            self.odom = v
            self.odom_version += 1
        else:
            pending = next((p for p in (self.motion, self.stop)
                            if p is not None and p.command == frame.command), None)
            if pending is None:
                return
            if frame.kind == Kind.ACK:
                if v[0] not in (pending.kind, Kind.QUERY_STATUS) or v[1] not in (0, 1):
                    return
                if v[1] == 1:
                    pending.acknowledged = True
                elif pending is self.stop:
                    self.locked = True
                    self.stop_state = 'REJECTED'
                    if self.motion:
                        self._finish('failed', 4, v[2], False, 'STOP rejected')
                    self.stop = None
                elif not pending.acknowledged and pending.terminal is None and v[0] == pending.kind:
                    self._finish('failed', 7, v[2], self.idle(now), 'STM32 rejected motion')
                else:
                    self._fault(3, 'motion result cannot be verified', now, v[2])
            else:
                if (v[0] != pending.kind or v[1] not in range(5) or v[4] != 0
                        or not all(math.isfinite(x) for x in v[6:])):
                    self._fault(3, 'invalid motion state', now)
                    return
                pending.acknowledged = True
                if pending.terminal is not None:
                    return  # terminal is immutable, duplicates cannot restart a goal
                pending.feedback = v
                if v[1] >= State.SUCCEEDED:
                    pending.terminal = v
                    pending.terminal_time = now
                    if v[1] == State.FAILED and pending is self.motion:
                        self._fault(0, 'STM32 motion failed', now, v[2])

    def _terminal_valid(self, p):
        v = p.terminal
        if v is None or v[1] != State.SUCCEEDED or v[2] != 0:
            return False
        mask = 4 if p.kind == Kind.STOP else (6 if p.kind == 1 else 7)
        if (v[3] & mask) != mask or abs(v[12]) > STOPPED_LINEAR_SPEED or abs(v[13]) > STOPPED_ANGULAR_SPEED:
            return False
        if p.kind == Kind.STOP:
            return True
        request = Motion.decode(Frame(p.kind, 0, 0, 0, 0, p.data))
        if abs(wrap(v[9] - request.yaw)) > 1e-5:
            return False
        if abs(wrap(request.yaw - v[10])) > request.yaw_tolerance + 1e-6:
            return False
        if p.kind == 2:
            return (abs(v[6] - request.distance) <= 1e-5
                    and abs(request.distance - v[7]) <= request.distance_tolerance + 1e-6)
        return True

    def tick(self, now):
        if not self.port_open:
            return
        watchdog_fallback = self.locked and self.stop is None and self.stop_state in ('UNCONFIRMED', 'REJECTED')
        if not watchdog_fallback and now - self.heartbeat_sent >= self.heartbeat_period:
            self._send(Kind.HEARTBEAT, 0, payload(Kind.HEARTBEAT, max(0, int(now * 1000))))
            self.heartbeat_sent = now
        if self.motion and self.stop is None:
            if not self.fresh(now):
                self._fault(5, 'status/odometry stale; stop requested', now)
            elif now >= self.motion.deadline:
                self._fault(2, 'motion deadline exceeded', now)

        for p in (self.motion, self.stop):
            if p is None:
                continue
            # After STOP, never retransmit the movement it superseded.
            if p is self.motion and self.stop is not None:
                continue
            if not p.acknowledged and now - p.sent >= self.ack_timeout:
                if p.retries < self.max_retries:
                    self._send(p.kind, p.command, p.data, p.kind == Kind.STOP)
                    p.retries += 1
                    p.sent = now
                elif p is self.motion:
                    self._fault(2, 'ACK timeout; stop requested', now)
        if now - self.query_sent >= self.query_period:
            target = self.stop or self.motion
            if self.stop and self.stop.terminal and self.motion and self.motion.terminal is None:
                target = self.motion
            if target:
                self._send(Kind.QUERY_STATUS, target.command, b'')
                self.query_sent = now

        if self.stop:
            stop = self.stop
            if stop.terminal and self._terminal_valid(stop) and self.idle(now, stop.terminal_time):
                self.stop_state = 'SUCCEEDED'
                self.stop = None
                if self.motion:
                    if self.failure:
                        code, device, msg = self.failure
                        self._finish('failed', code, device, True, msg)
                    elif self.motion.terminal and self.motion.terminal[1] == State.SUCCEEDED:
                        if self._terminal_valid(self.motion):
                            self._finish('succeeded', 0, 0, True, 'completed before stop')
                        else:
                            self.locked = True
                            self._finish('failed', 3, 0, True, 'invalid success telemetry')
                    elif self.motion.terminal and self.motion.terminal[1] == State.CANCELED:
                        canceled = self.cancel_requested
                        self._finish('canceled' if canceled else 'failed', 0 if canceled else 6,
                                     self.motion.terminal[2], True, 'motion stopped')
                    else:
                        # Original terminal may have been lost; query until deadline.
                        self.stop = stop
            if self.stop and now >= self.stop.deadline:
                self.locked = True
                self.stop_state = 'UNCONFIRMED'
                if self.motion:
                    self._finish('failed', 4, 0, False, 'stop/original result unconfirmed')
                self.stop = None
        elif self.motion and self.motion.terminal:
            p = self.motion
            if p.terminal[1] == State.SUCCEEDED:
                if not self._terminal_valid(p):
                    self._fault(3, 'invalid success telemetry', now)
                elif self.idle(now, p.terminal_time):
                    self._finish('succeeded', 0, 0, True, 'motion completed and stopped')
            elif p.terminal[1] == State.CANCELED and self.idle(now, p.terminal_time):
                self._finish('failed', 6, p.terminal[2], True, 'external stop')
