"""Exercise Qt UI-loop heartbeat over the real Python Unix socket server."""
import os
from contextlib import ExitStack
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

from services.frontend_lease import FrontendLease
from services.operator_adapter import OperatorAdapter
from services.operator_bootstrap import create_bootstrap_file
from services.operator_channel import OperatorChannel
from services.operator_channel_reader import OperatorChannelReader
from services.operator_runner import OperatorBootstrap


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "releases/r5_20261029/ros/ros2_ws/src/fire_robot_mission"))
from fire_robot_mission.operational_session_core import OperationalSessionCore

FIXTURE = Path(os.environ.get("FIRE_ROBOT_QT_TEST_CLIENT",
    f"/tmp/fire_robot_control_preview_{os.getuid()}/fire_robot_frontend_fixture"))


@unittest.skipUnless(FIXTURE.is_file() and os.access(FIXTURE, os.X_OK),
                     "Build the Qt fixture first with launchers/build_preview.sh")
class QtPythonChannelTests(unittest.TestCase):
    def run_case(self, mode, *, expect_active=True, threaded=False):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp) / "phiên điều khiển"
            directory.mkdir(mode=0o700)
            path = directory / "operator.sock"
            boot = OperatorBootstrap("8518a120-a012-4db3-bbf3-f73f67aed61e", "b" * 32,
                                     "c" * 32, "a" * 64, path)
            lease = FrontendLease(boot.session, boot.generation, boot.nonce, os.getuid(), time.monotonic(),
                                  connect_grace_s=0.4 if not expect_active else 3.0)
            with ExitStack() as stack:
                raw_channel = OperatorChannel(path, lease, clock=time.monotonic)
                if threaded:
                    channel = OperatorChannelReader(raw_channel)
                    stack.callback(channel.close)
                else:
                    channel = stack.enter_context(raw_channel)
                core = OperationalSessionCore()
                class CoreNode:
                    def tick(self, key):
                        return core.step(self.now, () if key is None else (key,))
                    def request_shutdown(self, reason):
                        core.step(self.now, ("Q",))
                node = CoreNode()
                adapter = OperatorAdapter(node, channel)
                with create_bootstrap_file(boot, temp) as bootstrap_file:
                    fd = bootstrap_file.fileno()
                    child = subprocess.Popen([str(FIXTURE), str(path), mode, str(fd)],
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, pass_fds=(fd,))
                keys = []
                try:
                    deadline = time.monotonic() + 3
                    while time.monotonic() < deadline and "Q" not in keys:
                        before = core.event_index
                        node.now = time.monotonic()
                        adapter.tick(node.now)
                        if core.event_index > before:
                            keys.append(core.feedback["key"])
                        time.sleep(.01)
                    self.assertIn("Q", keys)
                    self.assertEqual(lease.active, expect_active)
                    self.assertTrue(core.closed)
                    self.assertFalse(core.armed)
                    if mode != "disconnect":
                        self.assertIsNone(child.poll())
                    return keys, lease.reason
                finally:
                    if child.poll() is None:
                        child.terminate()  # synthetic fixture, never a robot runner
                    child.communicate(timeout=2)

    def test_qt_cancel_and_stop_reach_python_server(self):
        keys, reason = self.run_case("actions")
        self.assertEqual(keys, ["X", "Q"])
        self.assertEqual(reason, "OPERATOR_Q")

    def test_qt_event_loop_freeze_expires_real_lease(self):
        keys, reason = self.run_case("freeze")
        self.assertEqual(keys, ["Q"])
        self.assertEqual(reason, "FRONTEND_LEASE_EXPIRED")

    def test_wrong_frontend_identity_cannot_start_and_times_out(self):
        keys, reason = self.run_case("wrong-nonce", expect_active=False)
        self.assertEqual(keys, ["Q"])
        self.assertEqual(reason, "FRONTEND_LEASE_EXPIRED")

    def test_frontend_disconnect_requests_stop(self):
        keys, reason = self.run_case("disconnect")
        self.assertEqual(keys, ["Q"])
        self.assertEqual(reason, "FRONTEND_DISCONNECTED")

    def test_threaded_reader_preserves_qt_ui_liveness_and_stop(self):
        for mode, reason, active in [('actions', 'OPERATOR_Q', True),
                                     ('freeze', 'FRONTEND_LEASE_EXPIRED', True),
                                     ('disconnect', 'FRONTEND_DISCONNECTED', True),
                                     ('wrong-nonce', 'FRONTEND_LEASE_EXPIRED', False)]:
            with self.subTest(mode=mode):
                keys, actual = self.run_case(mode, expect_active=active, threaded=True)
                self.assertEqual(actual, reason)
                self.assertEqual(keys, ['X', 'Q'] if mode == 'actions' else ['Q'])


if __name__ == "__main__":
    unittest.main()
