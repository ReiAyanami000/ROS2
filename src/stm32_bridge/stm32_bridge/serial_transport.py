"""Bounded background I/O. socket:// is a simulator option; UART uses pyserial."""
from collections import deque
import queue
import socket
import threading
import time
from urllib.parse import urlparse

from .protocol import Kind


class SerialTransport:
    def __init__(self, port, baudrate=115200):
        self.port, self.baudrate = port, baudrate
        self.events = queue.Queue(maxsize=512)
        self.tx = deque()
        self.lock = threading.Lock()
        self.quit = threading.Event()
        self.reconnect = threading.Event()
        self.opened = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def send(self, frame, priority=False):
        data = frame.encode()
        with self.lock:
            if not self.opened:
                raise ConnectionError('transport is not open')
            if priority:
                # Never transmit a queued movement after a newer STOP.
                self.tx = deque(item for item in self.tx if item[0] not in (1, 2, 4))
            if len(self.tx) >= 64:
                raise ConnectionError('transmit queue full')
            if priority:
                self.tx.appendleft((frame.kind, data))
            else:
                self.tx.append((frame.kind, data))

    def _event(self, kind, data):
        try:
            self.events.put_nowait((kind, data, time.monotonic()))
        except queue.Full:
            # Overflow is a connection failure, never silently lose motor events.
            while not self.events.empty():
                try:
                    self.events.get_nowait()
                except queue.Empty:
                    break
            self.events.put_nowait(('error', 'receive queue overflow', time.monotonic()))
            raise ConnectionError('receive queue overflow')

    def _run(self):
        while not self.quit.is_set():
            channel = None
            try:
                tcp = self.port.startswith('socket://')
                if tcp:
                    url = urlparse(self.port)
                    if url.hostname not in ('127.0.0.1', 'localhost', '::1'):
                        raise ValueError('simulator TCP must use localhost')
                    channel = socket.create_connection((url.hostname, url.port), timeout=0.2)
                    channel.settimeout(0.02)
                else:
                    import serial
                    channel = serial.Serial(self.port, self.baudrate, timeout=0.02,
                                            write_timeout=0.1)
                with self.lock:
                    self.tx.clear()
                    self.opened = True
                self.reconnect.clear()
                self._event('opened', self.port)
                while not self.quit.is_set():
                    if self.reconnect.is_set():
                        raise ConnectionError('reconnect requested')
                    with self.lock:
                        item = self.tx.popleft() if self.tx else None
                    if item:
                        if tcp:
                            channel.sendall(item[1])
                        elif channel.write(item[1]) != len(item[1]):
                            raise ConnectionError('partial serial write')
                    try:
                        data = channel.recv(4096) if tcp else channel.read(4096)
                    except socket.timeout:
                        data = None
                    if tcp and data == b'':
                        raise ConnectionError('simulator connection closed')
                    if data:
                        self._event('data', data)
            except Exception as exc:
                with self.lock:
                    self.opened = False
                    self.tx.clear()
                try:
                    self._event('error', str(exc))
                except ConnectionError:
                    pass
            finally:
                with self.lock:
                    self.opened = False
                    self.tx.clear()
                if channel is not None:
                    channel.close()
            self.quit.wait(0.5)

    def close(self):
        self.quit.set()
        self.thread.join(timeout=1.0)

    def reset(self):
        self.reconnect.set()
