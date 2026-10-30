"""Exercise the independent reader with real sockets and real monotonic time."""
import json
import os
from pathlib import Path
import socket
import tempfile
import time
import unittest

from services.frontend_lease import FrontendLease
from services.operator_channel import OperatorChannel
from services.operator_channel_reader import OperatorChannelReader


class ReaderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.lease = FrontendLease('s', 'g', 'n', os.getuid(), time.monotonic())
        self.reader = OperatorChannelReader(OperatorChannel(Path(self.tmp.name) / 'ui.sock', self.lease, clock=time.monotonic))
        self.client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.client.connect(str(Path(self.tmp.name) / 'ui.sock'))
        self.seq = 0

    def tearDown(self):
        self.client.close()
        self.reader.close()
        self.tmp.cleanup()

    def send(self, event):
        self.seq += 1
        self.client.sendall(json.dumps(dict(version=1, session='s', generation='g', nonce='n',
            seq=self.seq, sent_mono=time.monotonic(), event=event)).encode() + b'\n')

    def test_slow_operator_does_not_misclassify_a_live_ui(self):
        # Main/operator thread is busy >350ms while actual UI keeps sending.
        for _ in range(7):
            self.send('HEARTBEAT')
            time.sleep(.08)
        self.assertIsNone(self.reader.poll(time.monotonic()))
        self.assertEqual(self.lease.reason, 'FRONTEND_ACTIVE')

    def test_frozen_ui_expires_and_cannot_revive(self):
        self.send('HEARTBEAT')
        time.sleep(.45)
        self.send('HEARTBEAT')
        time.sleep(.04)
        self.assertEqual(self.reader.poll(time.monotonic()), 'Q')
        self.assertEqual(self.lease.reason, 'FRONTEND_LEASE_EXPIRED')

    def test_q_dominates_pending_x_while_operator_is_busy(self):
        self.send('HEARTBEAT')
        time.sleep(.03)
        self.send('X')
        time.sleep(.03)
        self.send('Q')
        time.sleep(.03)
        self.assertEqual(self.reader.poll(time.monotonic()), 'Q')

    def test_disconnect_requests_q(self):
        self.send('HEARTBEAT')
        time.sleep(.03)
        self.client.close()
        time.sleep(.05)
        self.assertEqual(self.reader.poll(time.monotonic()), 'Q')


if __name__ == '__main__':
    unittest.main()
