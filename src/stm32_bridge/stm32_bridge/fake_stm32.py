"""Run via ros2 run stm32_bridge fake_stm32, or python -m stm32_bridge.fake_stm32."""
import argparse
import errno
import os
import select
import socket
import sys
import time

from .protocol import Frame, Kind, payload
from .simulator import FakeSTM32
from .stream_parser import StreamParser


def main():
    parser = argparse.ArgumentParser(description='STM32 V0.1 simulator: TCP or Linux PTY')
    parser.add_argument('--transport', choices=['tcp', 'pty'], default='tcp')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--fragment-size', type=int, default=0)
    parser.add_argument('--drop-first-ack', action='store_true')
    parser.add_argument('--drop-first-terminal', action='store_true')
    parser.add_argument('--inject-bad-crc', action='store_true')
    parser.add_argument('--stall', action='store_true')
    parser.add_argument('--reboot-after', type=float, default=0)
    parser.add_argument('--silent-after', type=float, default=0)
    parser.add_argument('--invalid-odom-after', type=float, default=0)
    argv = sys.argv[1:]
    # launch_ros adds ROS arguments; this simulator intentionally does not use rclpy.
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = parser.parse_args(argv)
    device = FakeSTM32(time.monotonic(), drop_first_ack=args.drop_first_ack,
                       drop_first_terminal=args.drop_first_terminal, stall=args.stall)
    stream = StreamParser()
    listener = channel = None
    master = slave = None
    pending = bytearray()
    try:
        if args.transport == 'tcp':
            listener = socket.socket()
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(('127.0.0.1', args.port))
            listener.listen(1)
            listener.setblocking(False)
            print(f'SIMULATOR PORT: socket://127.0.0.1:{args.port}', flush=True)
        else:
            import pty
            import tty
            master, slave = pty.openpty()
            tty.setraw(slave)
            os.set_blocking(master, False)
            print(f'SIMULATOR PORT: {os.ttyname(slave)}', flush=True)
        connected_at = None
        rebooted = False
        injected = False
        while True:
            now = time.monotonic()
            if listener and channel is None:
                try:
                    channel, _ = listener.accept()
                    channel.setblocking(False)
                    pending.clear()
                    stream = StreamParser()
                    connected_at = now
                    print('Client connected', flush=True)
                except BlockingIOError:
                    pass
            endpoint = channel if listener else master
            if endpoint is not None:
                readable, writable, _ = select.select([endpoint], [endpoint] if pending else [], [], 0.005)
                if readable:
                    try:
                        data = channel.recv(4096) if channel else os.read(master, 4096)
                    except OSError as exc:
                        if channel and isinstance(exc, (ConnectionResetError, ConnectionAbortedError)):
                            channel.close()
                            channel = None
                            pending.clear()
                            data = None
                        elif exc.errno not in (errno.EAGAIN, errno.EIO):
                            raise
                        else:
                            data = None
                    if channel and data == b'':
                        channel.close()
                        channel = None
                        pending.clear()
                    if data:
                        if connected_at is None:
                            connected_at = now
                        for frame in stream.feed(data, now):
                            device.receive(frame, now)
                if writable and pending:
                    try:
                        budget = 4096
                        while pending and budget > 0:
                            count = min(args.fragment_size if args.fragment_size > 0 else len(pending), budget)
                            sent = channel.send(pending[:count]) if channel else os.write(master, pending[:count])
                            del pending[:sent]
                            budget -= sent
                            if sent == 0:
                                break
                    except (BlockingIOError, BrokenPipeError, ConnectionResetError):
                        if channel:
                            channel.close()
                            channel = None
                            pending.clear()
            else:
                time.sleep(0.005)
            elapsed = now - connected_at if connected_at is not None else 0.0
            if args.reboot_after and elapsed >= args.reboot_after and not rebooted:
                device.reboot(now)
                rebooted = True
                print('Injected STM32 reboot', flush=True)
            device.tick(now)
            outgoing = device.drain()
            silent = args.silent_after and elapsed >= args.silent_after
            for frame in outgoing:
                if frame.kind == Kind.ODOMETRY and args.invalid_odom_after and elapsed >= args.invalid_odom_after:
                    v = list(frame.values())
                    v[1] = 0
                    frame = Frame(frame.kind, frame.session, frame.seq, frame.boot,
                                  frame.command, payload(frame.kind, *v))
                if silent or (listener and channel is None):
                    continue
                raw = frame.encode()
                if args.inject_bad_crc and not injected and device.session != 0:
                    bad = bytearray(raw)
                    bad[-1] ^= 0xff
                    pending.extend(bad)
                    injected = True
                pending.extend(raw)
            if len(pending) > 65536:
                raise RuntimeError('simulator output stalled: increase fragment-size')
    except KeyboardInterrupt:
        pass
    finally:
        for obj in (channel, listener):
            if obj:
                obj.close()
        for fd in (master, slave):
            if fd is not None:
                os.close(fd)


if __name__ == '__main__':
    main()
