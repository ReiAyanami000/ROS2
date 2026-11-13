"""Bounded byte stream parser with stale partial-frame recovery."""
import struct
import time
from .protocol import FORMATS, HEADER, Frame, crc16


class StreamParser:
    def __init__(self, max_buffer=8192, partial_timeout=0.25):
        self.buffer = bytearray()
        self.max_buffer = max_buffer
        self.partial_timeout = partial_timeout
        self.partial_since = None
        self.errors = 0

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        self.buffer.extend(data)
        if len(self.buffer) > self.max_buffer:
            del self.buffer[:-self.max_buffer]
            self.errors += 1
            self.partial_since = None
        frames = []
        while self.buffer:
            index = self.buffer.find(b'\xaa\x55')
            if index < 0:
                self.buffer[:] = b'\xaa' if self.buffer[-1:] == b'\xaa' else b''
                self.partial_since = None
                break
            if index:
                del self.buffer[:index]
                self.partial_since = None
            if self.partial_since is None:
                self.partial_since = now
            total = None
            if len(self.buffer) >= 26:
                version, kind, length, session, seq, boot, command = HEADER.unpack(self.buffer[2:26])
                if version != 1 or kind not in FORMATS or length != FORMATS[kind].size:
                    del self.buffer[0]
                    self.errors += 1
                    self.partial_since = None
                    continue
                total = 28 + length
            if total is None or len(self.buffer) < total:
                if now - self.partial_since > self.partial_timeout:
                    del self.buffer[0]
                    self.errors += 1
                    self.partial_since = None
                    continue
                break
            raw = self.buffer[:total]
            if crc16(raw[2:-2]) != struct.unpack('<H', raw[-2:])[0]:
                del self.buffer[0]
                self.errors += 1
                self.partial_since = None
                continue
            frames.append(Frame(kind, session, seq, boot, command, bytes(raw[26:-2])))
            del self.buffer[:total]
            self.partial_since = None
        return frames
