"""Read the real UI heartbeat independently of slow operator graph calls.

Only the socket and lease live on this thread. ROS, operator decisions and
STOP verification remain on the original operator thread.
"""
import threading
import time


class OperatorChannelReader:
    def __init__(self, channel, *, clock=time.monotonic):
        self.channel, self.clock = channel, clock
        self.lease = channel.lease
        self.lock = threading.Lock()
        self.finished = threading.Event()
        self.key = None
        self.error = None
        self.last_poll = clock()
        self.thread = threading.Thread(target=self._read, name='operator-ui-reader', daemon=True)
        self.thread.start()

    def _read(self):
        try:
            while not self.finished.is_set():
                with self.lock:
                    key = self.channel.poll(self.clock())
                    self.last_poll = self.clock()
                    if key == 'Q' or (key == 'X' and self.key is None):
                        self.key = key
                self.finished.wait(.01)
        except BaseException as exc:
            with self.lock:
                self.error = type(exc).__name__

    def poll(self, now):
        with self.lock:
            if self.error is not None or not self.thread.is_alive():
                raise RuntimeError('OPERATOR_UI_READER_FAILED')
            # A stalled reader is not allowed to extend the UI lease.
            if now - self.last_poll > self.lease.lease_s:
                self.lease.request_stop('FRONTEND_READER_STALLED')
                self.key = 'Q'
            key, self.key = self.key, None
            if key == 'Q':
                print('APP_OPERATOR_UI_STOP reason=' + str(self.lease.reason), flush=True)
            return key

    def close(self):
        self.finished.set()
        self.thread.join(timeout=1.)
        if self.thread.is_alive():
            raise RuntimeError('OPERATOR_UI_READER_DID_NOT_STOP')
        self.channel.close()
