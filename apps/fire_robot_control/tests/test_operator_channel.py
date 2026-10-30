import json
import os
from pathlib import Path
import socket
import tempfile
import unittest

from services.frontend_lease import FrontendLease
from services.operator_channel import OperatorChannel


class OperatorChannelTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "control.sock"
        self.lease = FrontendLease("s", "g", "n", os.getuid(), 10.0)

    def tearDown(self):
        self.tmp.cleanup()

    @staticmethod
    def message(event, seq, when):
        return json.dumps(dict(version=1, session="s", generation="g", nonce="n",
                               seq=seq, sent_mono=when, event=event)).encode() + b"\n"

    def test_fragmented_messages_and_frontend_loss(self):
        with OperatorChannel(self.path, self.lease) as channel:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.connect(str(self.path))
                data = self.message("HEARTBEAT", 1, 10.1)
                client.sendall(data[:12])
                self.assertIsNone(channel.poll(10.1))
                client.sendall(data[12:])
                self.assertIsNone(channel.poll(10.1))
                client.sendall(self.message("X", 2, 10.2))
                self.assertEqual(channel.poll(10.2), "X")
            self.assertEqual(channel.poll(10.3), "Q")
            self.assertEqual(self.lease.reason, "FRONTEND_DISCONNECTED")
        self.assertFalse(self.path.exists())

    def test_existing_file_and_nonprivate_directory_are_never_removed(self):
        self.path.write_text("user data")
        with self.assertRaises(OSError):
            OperatorChannel(self.path, self.lease)
        self.assertEqual(self.path.read_text(), "user data")
        self.path.unlink()
        os.chmod(self.tmp.name, 0o755)
        with self.assertRaises(ValueError):
            OperatorChannel(self.path, self.lease)

    def test_q_dominates_x_in_same_batch(self):
        with OperatorChannel(self.path, self.lease) as channel:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.connect(str(self.path))
                client.sendall(self.message("HEARTBEAT", 1, 10.1) +
                               self.message("X", 2, 10.1) +
                               self.message("Q", 3, 10.1))
                self.assertEqual(channel.poll(10.1), "Q")
                self.assertIsNone(channel.poll(10.2))

    def test_q_is_not_delayed_by_a_batch_of_heartbeats(self):
        with OperatorChannel(self.path, self.lease) as channel:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.connect(str(self.path))
                payload = b"".join(self.message("HEARTBEAT", n, 10.1) for n in range(1, 22))
                client.sendall(payload + self.message("X", 22, 10.1) + self.message("Q", 23, 10.1))
                self.assertEqual(channel.poll(10.1), "Q")

    def test_message_overflow_requests_stop(self):
        with OperatorChannel(self.path, self.lease) as channel:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.connect(str(self.path))
                client.sendall(self.message("HEARTBEAT", 1, 10.1))
                self.assertIsNone(channel.poll(10.1))
                client.sendall(b"x" * 2049)
                self.assertEqual(channel.poll(10.2), "Q")
                self.assertEqual(self.lease.reason, "FRONTEND_MESSAGE_OVERFLOW")

    def test_clock_resampled_after_receive_does_not_reject_valid_heartbeat(self):
        with OperatorChannel(self.path, self.lease, clock=lambda: 10.02) as channel:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.connect(str(self.path))
                # Caller sampled 10.0; peer sent between sample and recv.
                client.sendall(self.message("HEARTBEAT", 1, 10.01))
                self.assertIsNone(channel.poll(10.0))
                self.assertTrue(self.lease.active)


if __name__ == "__main__":
    unittest.main()
