"""Drive the sealed operator's pure core through the new local channel.

No ROS node, hardware, authority publisher or real operator is started here.
This is narrower than integration with the real operator process.
"""
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import unittest

from services.frontend_lease import FrontendLease
from services.operator_adapter import OperatorAdapter
from services.operator_channel import OperatorChannel


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "releases/r5_20261029/ros/ros2_ws/src/fire_robot_mission"))
from fire_robot_mission.operational_session_core import OperationalSessionCore


def message(event, seq, now):
    return json.dumps(dict(version=1, session="session", generation="generation",
        nonce="nonce", seq=seq, sent_mono=now, event=event)).encode() + b"\n"


class OperatorCoreIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.lease = FrontendLease("session", "generation", "nonce", os.getuid(), 10.)
        self.channel = OperatorChannel(Path(self.tmp.name) / "control.sock", self.lease)
        self.client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.client.connect(str(self.channel.path))
        self.core = OperationalSessionCore()
        core = self.core
        class CoreHarness:
            def tick(self, key):
                return core.step(self.now, () if key is None else (key,))
            def request_shutdown(self, reason):
                core.step(self.now, ("Q",))
        self.node = CoreHarness()
        self.adapter = OperatorAdapter(self.node, self.channel)

    def tearDown(self):
        self.client.close()
        self.channel.close()
        self.tmp.cleanup()

    def tick(self, now):
        self.node.now = now
        self.adapter.tick(now)
        return self.core.feedback.get("key") if self.core.feedback.get("at") == now else None

    def test_simultaneous_x_q_is_delivered_as_q_to_real_core(self):
        self.client.sendall(message("HEARTBEAT", 1, 10.1) + message("X", 2, 10.1) + message("Q", 3, 10.1))
        self.assertEqual(self.tick(10.1), "Q")
        self.assertTrue(self.core.closed)
        self.assertFalse(self.core.armed)
        self.assertEqual(self.core.feedback["reason"], "STOP_ACCEPTED")
        self.assertIsNone(self.tick(10.2))

    def test_ui_freeze_sends_q_even_with_backend_ticks(self):
        self.client.sendall(message("HEARTBEAT", 1, 10.1))
        self.assertIsNone(self.tick(10.1))
        self.assertFalse(self.core.closed)
        self.assertEqual(self.tick(10.46), "Q")
        self.assertEqual(self.lease.reason, "FRONTEND_LEASE_EXPIRED")
        self.assertTrue(self.core.closed)

    def test_x_without_active_task_is_not_misreported_as_cancel(self):
        self.client.sendall(message("HEARTBEAT", 1, 10.1) + message("X", 2, 10.1))
        self.assertEqual(self.tick(10.1), "X")
        self.assertFalse(self.core.feedback["accepted"])
        self.assertFalse(self.core.closed)
        self.assertIsNone(self.tick(10.2))


if __name__ == "__main__":
    unittest.main()
