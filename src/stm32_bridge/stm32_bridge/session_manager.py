"""Connection identity and freshness, owned by the bridge event loop."""
import secrets


class Session:
    def __init__(self):
        self.session = secrets.randbits(32) or 1
        self.boot = 0
        self.bound = False
        self.status_time = -1e9
        self.odom_time = -1e9
        self.sample_time = -1
        self.last_seq = None

    def accepts(self, frame):
        return frame.session == self.session and frame.boot == self.boot and self.boot != 0

    def forward_sequence(self, seq):
        if self.last_seq is None:
            self.last_seq = seq
            return True
        delta = (seq - self.last_seq) & 0xffffffff
        if 0 < delta < 0x80000000:
            self.last_seq = seq
            return True
        return False
