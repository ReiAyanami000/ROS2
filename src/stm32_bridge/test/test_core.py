"""Protocol and end-to-end state machine tests; no ROS or hardware required."""
import math
from pathlib import Path
import random
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stm32_bridge.protocol import Frame, Kind, Motion, Reason, State, crc16, payload
from stm32_bridge.stream_parser import StreamParser
from stm32_bridge.motion_client import MotionClient
from stm32_bridge.simulator import FakeSTM32


class Harness:
    def __init__(self, **fake_options):
        self.now = 0.0
        self.sent = []
        self.history = []
        self.engine = MotionClient(self.send)
        self.fake = FakeSTM32(**fake_options)
        self.engine.open(self.now)
        self.host_parser = StreamParser()
        self.device_parser = StreamParser()
        self.filter = lambda f: True
        self.pump(0.3)

    def send(self, frame, priority=False):
        self.history.append(frame)
        self.sent.append(frame)

    def pump(self, duration):
        for _ in range(int(duration / 0.01) + 1):
            self.now += 0.01
            self.engine.tick(self.now)
            outgoing, self.sent = self.sent, []
            for frame in outgoing:
                for decoded in self.device_parser.feed(frame.encode(), self.now):
                    self.fake.receive(decoded, self.now)
            self.fake.tick(self.now)
            for frame in self.fake.drain():
                if self.filter(frame):
                    for decoded in self.host_parser.feed(frame.encode(), self.now):
                        self.engine.receive(decoded, self.now)

    def move(self, distance=0.1, timeout_ms=10000):
        return self.engine.begin(Motion(2, 0.0, distance=distance, timeout_ms=timeout_ms), self.now)


class ProtocolTests(unittest.TestCase):
    def test_crc_golden(self):
        self.assertEqual(crc16(b'123456789'), 0x29b1)

    def test_documented_heartbeat(self):
        expected = bytes.fromhex('AA 55 01 05 08 00 04 03 02 01 01 00 00 00 '
                                 '00 00 00 00 00 00 00 00 00 00 00 00 '
                                 'E8 03 00 00 00 00 00 00 F2 57')
        self.assertEqual(Frame(5, 0x01020304, 1, 0, 0, payload(5, 1000)).encode(), expected)

    def test_documented_ack(self):
        expected = bytes.fromhex('AA 55 01 81 04 00 04 03 02 01 02 00 00 00 '
                                 '08 07 06 05 04 03 02 01 01 00 00 00 '
                                 '01 01 00 00 BB 6C')
        f = Frame(0x81, 0x01020304, 2, 0x0102030405060708, 1, payload(0x81, 1, 1, 0))
        self.assertEqual(f.encode(), expected)

    def test_random_fragmentation_noise_and_crc(self):
        frames = [Frame(5, 7, n, 8, 0, payload(5, n)) for n in range(100)]
        bad = bytearray(frames[0].encode())
        bad[-1] ^= 1
        data = b'noise' + bytes(bad) + b''.join(f.encode() for f in frames)
        rng, parser, output = random.Random(42), StreamParser(), []
        while data:
            count = rng.randrange(1, 60)
            output.extend(parser.feed(data[:count], 0.0))
            data = data[count:]
        self.assertEqual(output, frames)
        self.assertGreater(parser.errors, 0)

    def test_truncated_frame_timeout(self):
        full = Frame(5, 1, 1, 1, 0, payload(5, 9)).encode()
        parser = StreamParser()
        parser.feed(full[:10], 0.0)
        parser.feed(b'', 0.3)
        self.assertEqual(parser.feed(full, 0.31)[0].values(), (9,))

    def test_bad_length_and_bounded_buffer(self):
        good = Frame(4, 1, 1, 1, 0, b'').encode()
        bad = bytearray(good)
        bad[4:6] = b'\xff\xff'
        p = StreamParser()
        self.assertEqual(len(p.feed(bytes(bad) + good)), 1)
        p.feed(b'x' * 100000)
        self.assertLessEqual(len(p.buffer), 8192)

    def test_parameter_limits(self):
        for motion in (Motion(2, 0, distance=-1), Motion(1, float('nan')),
                       Motion(2, 0, distance=1, speed=100),
                       Motion(1, 0, yaw_tolerance=1e-50)):
            with self.subTest(motion=motion), self.assertRaises(ValueError):
                motion.validate()


class LifecycleTests(unittest.TestCase):
    def test_handshake_and_move(self):
        h = Harness()
        self.assertTrue(h.engine.ready(h.now))
        h.move()
        h.pump(3)
        self.assertEqual(h.engine.outcomes[-1]['outcome'], 'succeeded')
        self.assertTrue(h.engine.outcomes[-1]['stopped_confirmed'])
        self.assertAlmostEqual(h.fake.x, 0.1, delta=0.01)

    def test_another_live_session_cannot_claim_binding(self):
        h = Harness()
        original_session = h.fake.session
        stranger = (original_session % 0xfffffffe) + 1
        h.fake.receive(Frame(Kind.HEARTBEAT, stranger, 1, h.fake.boot, 0,
                             payload(Kind.HEARTBEAT, 100)), h.now)
        status = h.fake.drain()[-1]
        self.assertEqual(h.fake.session, original_session)
        self.assertEqual(status.session, stranger)
        self.assertFalse(status.values()[3] & 64)

    def test_rotate(self):
        h = Harness()
        h.engine.begin(Motion(1, math.pi / 2), h.now)
        h.pump(6)
        self.assertEqual(h.engine.outcomes[-1]['outcome'], 'succeeded')
        self.assertAlmostEqual(h.fake.yaw, math.pi / 2, delta=0.03)

    def test_missing_ack_and_terminal(self):
        h = Harness(drop_first_ack=True, drop_first_terminal=True)
        h.move()
        h.pump(4)
        self.assertEqual(h.fake.execution_count, 1)
        self.assertEqual(h.engine.outcomes[-1]['outcome'], 'succeeded')

    def test_retry_same_id_when_all_initial_replies_lost(self):
        h = Harness()
        threshold = h.now + 0.35
        h.filter = lambda f: not (h.now < threshold and f.kind in (Kind.ACK, Kind.MOTION_STATE))
        cmd = h.move()
        h.pump(4)
        movements = [f for f in h.history if f.kind == Kind.MOVE_DISTANCE]
        self.assertGreaterEqual(len(movements), 2)
        self.assertEqual({f.command for f in movements}, {cmd})
        self.assertEqual(h.fake.execution_count, 1)

    def test_cancel_waits_for_stop(self):
        h = Harness()
        h.move(1)
        h.pump(0.3)
        h.engine.request_stop(h.now, cancel=True)
        self.assertEqual(h.engine.outcomes, [])
        h.pump(1)
        self.assertEqual(h.engine.outcomes[-1]['outcome'], 'canceled')
        self.assertTrue(h.engine.outcomes[-1]['stopped_confirmed'])
        self.assertEqual(h.fake.execution_count, 1)

    def test_external_stop_is_abort(self):
        h = Harness()
        h.move(1)
        h.pump(0.3)
        h.engine.request_stop(h.now)
        h.pump(1)
        self.assertEqual(h.engine.outcomes[-1]['bridge_error'], 6)

    def test_unknown_stop_never_claims_canceled(self):
        h = Harness()
        h.move(1)
        h.pump(0.3)
        h.filter = lambda f: f.kind != Kind.MOTION_STATE
        h.engine.request_stop(h.now, cancel=True)
        h.pump(3)
        self.assertEqual(h.engine.outcomes[-1]['outcome'], 'failed')
        self.assertFalse(h.engine.outcomes[-1]['stopped_confirmed'])
        self.assertTrue(h.engine.locked)

    def test_stale_odom_requests_stop(self):
        h = Harness()
        h.move(1)
        h.pump(0.3)
        h.filter = lambda f: f.kind != Kind.ODOMETRY
        h.pump(3)
        self.assertTrue(any(f.kind == Kind.STOP for f in h.history))
        self.assertEqual(h.engine.outcomes[-1]['outcome'], 'failed')

    def test_reboot_invalidates_task(self):
        h = Harness()
        h.move(1)
        h.pump(0.3)
        h.fake.reboot(h.now)
        h.pump(1)
        self.assertEqual(h.engine.outcomes[-1]['outcome'], 'failed')
        self.assertTrue(h.engine.locked)
        self.assertFalse(h.engine.ready(h.now))

    def test_disconnect_cannot_resume(self):
        h = Harness()
        h.move(1)
        h.engine.disconnect('test unplug', h.now)
        h.sent.clear()  # SerialTransport drops unsent bytes on disconnect.
        h.engine.open(h.now)
        h.pump(1)
        self.assertTrue(h.engine.locked)
        self.assertEqual(h.fake.execution_count, 0)  # old queued frame has old session

    def test_lost_original_cancel_result_is_queried(self):
        h = Harness()
        h.move(1)
        h.pump(0.3)
        dropped = []

        def filtering(f):
            if f.kind == Kind.MOTION_STATE and f.values()[1] == State.CANCELED and not dropped:
                dropped.append(f)
                return False
            return True

        h.filter = filtering
        h.engine.request_stop(h.now, cancel=True)
        h.pump(1.5)
        self.assertTrue(dropped)
        self.assertEqual(h.engine.outcomes[-1]['outcome'], 'canceled')

    def test_wrong_session_ignored(self):
        h = Harness()
        cmd = h.move(1)
        f = Frame(Kind.ACK, 123, 900, h.fake.boot, cmd, payload(Kind.ACK, 2, 1, 0))
        h.engine.receive(f, h.now)
        self.assertFalse(h.engine.motion.acknowledged)

    def test_no_concurrent_motion(self):
        h = Harness()
        h.move(1)
        with self.assertRaises(ValueError):
            h.move(1)

    def test_simulator_duplicate_and_conflict(self):
        h = Harness()
        cmd = h.move(1)
        h.pump(0.2)
        original = next(f for f in h.history if f.kind == 2)
        h.fake.receive(original, h.now)
        self.assertEqual(h.fake.execution_count, 1)
        bad = Frame(2, original.session, 100, original.boot, cmd,
                    Motion(2, 0, distance=2).encode())
        h.fake.receive(bad, h.now)
        self.assertEqual(h.fake.drain()[-1].values()[2], Reason.COMMAND_ID_CONFLICT)

    def test_stall_times_out(self):
        h = Harness(stall=True)
        h.move(1, timeout_ms=1000)
        h.pump(3)
        self.assertEqual(h.engine.outcomes[-1]['outcome'], 'failed')

    def test_device_watchdog(self):
        h = Harness()
        h.move(1)
        h.pump(0.2)
        h.fake.tick(h.now + 0.6)
        self.assertIsNone(h.fake.active)
        self.assertEqual(h.fake.session, 0)
        self.assertTrue(h.fake.fault & 64)

    def test_success_requires_matching_target_and_stop(self):
        h = Harness()
        cmd = h.move(1)
        h.pump(0.1)
        data = payload(Kind.MOTION_STATE, 2, State.SUCCEEDED, 0, 7, 0, 100,
                       1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        h.fake._emit(Kind.MOTION_STATE, cmd, data)
        h.pump(1)
        self.assertEqual(h.engine.outcomes[-1]['outcome'], 'failed')
        self.assertEqual(h.engine.outcomes[-1]['bridge_error'], 3)
        self.assertTrue(h.engine.locked)

    def test_old_or_invalid_odometry_not_republished(self):
        h = Harness()
        version = h.engine.odom_version
        old = h.engine.odom
        h.fake._emit(Kind.ODOMETRY, 0, payload(Kind.ODOMETRY, *old))
        for f in h.fake.drain():
            h.engine.receive(f, h.now)
        self.assertEqual(h.engine.odom_version, version)
        self.assertFalse(h.engine.ready(h.now))

    def test_stop_timeout_withdraws_heartbeat(self):
        h = Harness()
        h.move(1)
        h.pump(0.1)
        h.filter = lambda f: f.kind != Kind.MOTION_STATE
        h.engine.request_stop(h.now, cancel=True)
        h.pump(2.2)
        count = len([f for f in h.history if f.kind == Kind.HEARTBEAT])
        h.pump(0.3)
        self.assertEqual(count, len([f for f in h.history if f.kind == Kind.HEARTBEAT]))

    def test_terminal_waits_for_fresh_stopped_status(self):
        h = Harness()
        h.move(0.1)
        h.filter = lambda f: not (f.kind == Kind.BASE_STATUS and f.values()[1] == 0)
        h.pump(1.8)
        self.assertFalse(any(x['outcome'] == 'succeeded' for x in h.engine.outcomes))


if __name__ == '__main__':
    unittest.main()
