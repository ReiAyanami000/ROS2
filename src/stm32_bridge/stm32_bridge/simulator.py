"""Idealised STM32 protocol simulator, not a motor-control/physics implementation."""
from collections import OrderedDict
import math
import secrets

from .protocol import Frame, Kind, Motion, Reason, State, payload, wrap


class FakeSTM32:
    def __init__(self, now=0.0, watchdog=0.5, drop_first_ack=False,
                 drop_first_terminal=False, stall=False):
        self.watchdog = watchdog
        self.drop_ack = drop_first_ack
        self.drop_terminal = drop_first_terminal
        self.stall = stall
        self.execution_count = 0
        self.reboot(now)

    def reboot(self, now):
        self.boot = secrets.randbits(64) or 1
        self.session = self.seq = self.highest = 0
        self.started = self.last_tick = now
        self.last_heartbeat = -1e9
        self.last_odom = self.last_status = self.last_motion = -1e9
        self.records = OrderedDict()
        self.active = self.stopping = None
        self.stop_due = None
        self.x = self.y = self.yaw = self.v = self.w = 0.0
        self.fault = 0
        self.out = []

    def _emit(self, kind, command, data, session=None):
        self.seq = (self.seq + 1) & 0xffffffff
        if kind == Kind.ACK and self.drop_ack:
            self.drop_ack = False
            return
        if kind == Kind.MOTION_STATE and data[1] >= State.SUCCEEDED and self.drop_terminal:
            self.drop_terminal = False
            return
        self.out.append(Frame(kind, self.session if session is None else session,
                              self.seq, self.boot, command, data))

    def drain(self):
        result, self.out = self.out, []
        return result

    def _ack(self, f, reason=0):
        self._emit(Kind.ACK, f.command, payload(Kind.ACK, f.kind, int(reason == 0), reason), f.session)

    def status(self, now, session=None):
        state = 4 if self.fault else (3 if self.stopping else (
            1 if self.active and self.active['kind'] == 1 else 2 if self.active else 0))
        replying_to_bound_session = self.session and (session is None or session == self.session)
        flags = 0x1d | (64 if replying_to_bound_session else 0) | (32 if self.fault & 64 else 0)
        self._emit(Kind.BASE_STATUS, 0, payload(Kind.BASE_STATUS,
                   max(0, int((now - self.started) * 1000)), state,
                   self.active['kind'] if self.active else 0, flags,
                   self.active['id'] if self.active else 0, self.fault, 12.0, 1, 0), session)

    def _state(self, record, now, state=None, reason=0):
        if state is not None:
            record['state'] = int(state)
            record['reason'] = int(reason)
        m = record.get('motion')
        distance = m.distance if m and record['kind'] == 2 else 0.0
        target_yaw = wrap(m.yaw) if m else 0.0
        flags = 7 if record['kind'] == 2 else 6 if record['kind'] == 1 else 4
        # Cache the terminal payload unchanged, including measured final values.
        if record.get('terminal') is None:
            data = payload(Kind.MOTION_STATE, record['kind'], record['state'], record['reason'],
                           flags, 0, max(0, int((now - record['started']) * 1000)),
                           distance, record.get('traveled', 0.0),
                           distance - record.get('traveled', 0.0), target_yaw,
                           self.yaw if m else 0.0, wrap(target_yaw - self.yaw) if m else 0.0,
                           self.v, self.w)
            if record['state'] >= 2:
                record['terminal'] = data
        else:
            data = record['terminal']
        self._emit(Kind.MOTION_STATE, record['id'], data, record['session'])

    def _remember(self, f, reason, now, motion=None):
        record = dict(id=f.command, kind=f.kind, data=f.payload, reason=int(reason),
                      session=f.session, started=now, state=State.ACCEPTED, motion=motion,
                      traveled=0.0, terminal=None)
        self.records[(f.session, f.command)] = record
        self.highest = max(self.highest, f.command) if f.session == self.session else self.highest
        # Keep active + latest 64 records. Never evict active/stopping commands.
        for key in list(self.records):
            if len(self.records) <= 65:
                break
            if self.records[key] is not self.active and self.records[key] is not self.stopping:
                del self.records[key]
        return record

    def receive(self, f, now):
        if f.kind == Kind.HEARTBEAT:
            if f.session == 0:
                return
            if f.boot == 0:
                self.status(now, f.session)
                return
            if f.boot != self.boot:
                return
            if self.session == 0 and not self.active and not self.stopping:
                self.session = f.session
                self.records.clear()
                self.highest = 0
                self.fault &= ~64
            if f.session == self.session:
                self.last_heartbeat = now
            self.status(now, f.session)
            return
        if f.kind not in (Kind.ROTATE_TO, Kind.MOVE_DISTANCE, Kind.STOP, Kind.QUERY_STATUS):
            return
        if f.boot != self.boot and not (f.kind == Kind.STOP and f.boot == 0):
            self._ack(f, Reason.BOOT_MISMATCH)
            return
        if (not self.session or f.session != self.session) and f.kind != Kind.STOP:
            self._ack(f, Reason.SESSION_MISMATCH)
            return
        key = (f.session, f.command)
        if f.kind == Kind.QUERY_STATUS:
            if f.command == 0:
                self.status(now, f.session)
                return
            record = self.records.get(key)
            if not record:
                self._ack(f, Reason.OLD_COMMAND if f.command <= self.highest else Reason.UNKNOWN_COMMAND)
            elif record['reason'] and record['state'] == State.ACCEPTED:
                self._ack(f, record['reason'])
            else:
                self._ack(f)
                self._state(record, now)
            return
        if f.command == 0:
            self._ack(f, Reason.INVALID_PARAMETER)
            return
        if key in self.records:
            record = self.records[key]
            if record['kind'] != f.kind or record['data'] != f.payload:
                self._ack(f, Reason.COMMAND_ID_CONFLICT)
            else:
                # A final FAILED/CANCELED reason is not an original ACK rejection.
                rejected = record.get('rejected', False)
                self._ack(f, record['reason'] if rejected else 0)
                if not rejected:
                    self._state(record, now)
            return
        if f.session == self.session and f.command <= self.highest:
            self._ack(f, Reason.OLD_COMMAND)
            return
        if f.kind == Kind.STOP:
            reason, reserved = f.values()
            if reason not in (1, 2, 3) or reserved:
                self._ack(f, Reason.INVALID_PARAMETER)
                return
            record = self._remember(f, 0, now)
            self._ack(f)
            if self.stopping:
                # A second STOP never supersedes a pending STOP or original action.
                record['state'] = State.FAILED
                record['reason'] = Reason.BUSY
                self._state(record, now)
                return
            self.stopping = record
            self.stop_due = now + 0.12
            self.v = self.w = 0.0
            self._state(record, now, State.RUNNING)
            self.status(now)
            return
        reason = Reason.OK
        motion = None
        try:
            motion = Motion.decode(f)
            motion.validate()
        except (ValueError, OverflowError):
            reason = Reason.INVALID_PARAMETER
        if self.active or self.stopping:
            reason = Reason.BUSY
        elif self.fault:
            reason = Reason.BASE_FAULT
        elif motion and f.kind == Kind.MOVE_DISTANCE and abs(wrap(motion.yaw - self.yaw)) > motion.yaw_tolerance:
            reason = Reason.NOT_ALIGNED
        record = self._remember(f, reason, now, motion)
        record['rejected'] = bool(reason)
        self._ack(f, reason)
        if reason:
            return
        self.execution_count += 1
        self.active = record
        self._state(record, now, State.ACCEPTED)
        self._state(record, now, State.RUNNING)
        self.status(now)

    def tick(self, now):
        dt = min(0.05, max(0.0, now - self.last_tick))
        self.last_tick = now
        if self.session and now - self.last_heartbeat > self.watchdog:
            self.v = self.w = 0.0
            self.fault |= 64
            if self.active:
                self._state(self.active, now, State.FAILED, Reason.HEARTBEAT_TIMEOUT)
                self.active = None
            if self.stopping:
                self._state(self.stopping, now, State.FAILED, Reason.HEARTBEAT_TIMEOUT)
                self.stopping = None
            self.status(now)
            self.session = 0
        if self.stopping and now >= self.stop_due:
            self.v = self.w = 0.0
            if self.active:
                self._state(self.active, now, State.CANCELED, Reason.CANCELED_BY_STOP)
                self.active = None
            self._state(self.stopping, now, State.SUCCEEDED)
            self.stopping = None
            self.status(now)
        if self.active and not self.stopping:
            record = self.active
            m = record['motion']
            if now - record['started'] >= m.timeout_ms / 1000:
                self.v = self.w = 0.0
                self._state(record, now, State.FAILED, Reason.MOTION_TIMEOUT)
                self.active = None
                self.status(now)
            elif not self.stall:
                if record['kind'] == Kind.ROTATE_TO:
                    remaining = wrap(m.yaw - self.yaw)
                    desired = min(m.angular_speed, math.sqrt(2 * m.angular_acceleration * abs(remaining)))
                    speed = min(abs(self.w) + m.angular_acceleration * dt, desired)
                    step = math.copysign(min(abs(remaining), speed * dt), remaining)
                    self.yaw = wrap(self.yaw + step)
                    self.w = math.copysign(speed, remaining)
                    reached = abs(remaining) <= m.yaw_tolerance / 2
                else:
                    remaining = max(0.0, m.distance - record['traveled'])
                    desired = min(m.speed, math.sqrt(2 * m.acceleration * remaining))
                    self.v = min(self.v + m.acceleration * dt, desired)
                    step = min(remaining, self.v * dt)
                    record['traveled'] += step
                    self.x += step * math.cos(self.yaw)
                    self.y += step * math.sin(self.yaw)
                    reached = remaining <= m.distance_tolerance / 2
                if reached:
                    self.v = self.w = 0.0
                    if 'settle' not in record:
                        record['settle'] = now
                    if now - record['settle'] >= 0.15:
                        self._state(record, now, State.SUCCEEDED)
                        self.active = None
                        self.status(now)
        if now - self.last_odom >= 0.02:
            self._emit(Kind.ODOMETRY, 0, payload(Kind.ODOMETRY,
                       max(0, int((now - self.started) * 1e6)), 15, 0,
                       self.x, self.y, self.yaw, self.v, self.w))
            self.last_odom = now
        if now - self.last_status >= 0.2:
            self.status(now)
            self.last_status = now
        if self.active and now - self.last_motion >= 0.1:
            self._state(self.active, now)
            self.last_motion = now
        elif not self.active and now - self.last_motion >= 1.0:
            terminals = [r for r in self.records.values() if r.get('terminal')]
            if terminals:
                self._state(terminals[-1], now)
            self.last_motion = now
