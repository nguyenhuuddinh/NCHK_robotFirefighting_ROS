import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from services.operator_owned_backend import OperatorOwnedBackend
from services.operator_runner import OperatorBootstrap


ROOT = Path(__file__).resolve().parents[3]
APP = ROOT / "apps/fire_robot_control"
sys.path.insert(0, str(ROOT / "releases/r5_20261029/ros/scripts"))
from operational_owned_process import Backend, OwnedProcess
from operational_entry import LocalProcesses


class OperatorOwnedBackendTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.candidate = self.root / "candidate"
        self.session, self.runtime = self.root / "session", self.root / "runtime"
        self.session.mkdir(mode=0o700)
        self.runtime.mkdir(mode=0o700)
        config = self.candidate / "ros2_ws/src/fire_robot_mission/config/operational_gate2.yaml"
        config.parent.mkdir(parents=True)
        config.write_text("operator: {}\n")
        self.bootstrap = OperatorBootstrap("8518a120-a012-4db3-bbf3-f73f67aed61e", "b" * 32,
            "c" * 32, "a" * 64, self.runtime / "operator.sock")
    def tearDown(self): self.temp.cleanup()

    def backend(self, popen):
        return OperatorOwnedBackend(self.bootstrap, delegate=Backend(), app_dir=APP,
            candidate_root=self.candidate, session_dir=self.session, runtime_dir=self.runtime,
            env=dict(os.environ), popen=popen)

    def test_unexpected_command_never_spawns_and_duplicate_is_rejected(self):
        calls = []
        def fake(*args, **kwargs): calls.append(args); return object()
        backend = self.backend(fake)
        with self.assertRaises(ValueError): backend.start(["ros2", "run"], None)
        self.assertEqual(calls, [])
        backend.start(backend.command, None)
        with self.assertRaises(ValueError): backend.start(backend.command, None)
        self.assertEqual(len(calls), 1)

    def test_real_owned_process_tracks_and_stops_local_fake_child(self):
        def fake(argv, **kwargs):
            fd = kwargs["pass_fds"][0]
            code = ("import signal,sys,time; from services.operator_bootstrap import read_bootstrap_fd; "
                    "read_bootstrap_fd(int(sys.argv[1])); "
                    "signal.signal(signal.SIGINT,lambda *_:sys.exit(0)); "
                    "print('FIXTURE_READY',flush=True); signal.pause()")
            return subprocess.Popen([sys.executable, "-u", "-c", code, str(fd)], **kwargs)
        backend = self.backend(fake)
        owned = OwnedProcess("operator", backend=backend)
        with tempfile.TemporaryFile(mode="w+b") as output:
            try:
                owned.start(backend.command, output)
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline and b"FIXTURE_READY" not in os.pread(output.fileno(), 1024, 0):
                    time.sleep(.01)
                self.assertIn(b"FIXTURE_READY", os.pread(output.fileno(), 1024, 0))
                self.assertNotIn(self.bootstrap.nonce.encode(), os.pread(output.fileno(), 1024, 0))
                owned.stop(time.monotonic())
                deadline = time.monotonic() + 3
                while not owned.done and time.monotonic() < deadline:
                    time.sleep(.01)
                    owned.tick(time.monotonic())
                report = owned.report()
                self.assertTrue(report["group_exit_confirmed"], report)
                self.assertEqual(report["leader_rc"], 0)
                self.assertFalse(report["forced"])
                self.assertEqual(report["robot_stop_proof"], "NOT_EVALUATED")
            finally:
                # Synthetic local fixture only; never used for a robot process.
                if owned.process is not None and not owned.done:
                    os.killpg(owned.pid, signal.SIGKILL)
                    owned.process.wait(timeout=2)

    def test_r5_registry_accepts_only_owned_complete_stop_receipt(self):
        for receipt in (True, False):
            with self.subTest(receipt=receipt):
                output_dir = self.root / ("with-receipt" if receipt else "without-receipt")
                output_dir.mkdir()
                def fake(argv, **kwargs):
                    fd = kwargs["pass_fds"][0]
                    code = """
import json, signal, sys
from services.operator_bootstrap import read_bootstrap_fd
read_bootstrap_fd(int(sys.argv[1]))
def stop(*_):
    if sys.argv[2] == 'yes':
        proof = dict(scope='R5_STOP_TELEMETRY_VERIFICATION', requested=True,
            done=True, confirmed=True, reason='FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED',
            blockers=[], fault=None, motion_authorized=False, pulse_replayed=False)
        print('[INFO] [1.0] [fire_robot_operator]: OPERATOR_STOP ' + json.dumps(proof), flush=True)
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
print('FIXTURE_READY', flush=True)
signal.pause()
"""
                    return subprocess.Popen([sys.executable, "-u", "-c", code, str(fd),
                                             "yes" if receipt else "no"], **kwargs)
                backend = self.backend(fake)
                local = LocalProcesses(output_dir, self.bootstrap.session,
                    backend_factory=lambda role: backend if role == "operator" else None)
                process = None
                try:
                    process = local.start("operator", backend.command)
                    deadline = time.monotonic() + 3
                    while time.monotonic() < deadline and "FIXTURE_READY" not in (output_dir / "operator.log").read_text():
                        time.sleep(.01)
                    self.assertIn("FIXTURE_READY", (output_dir / "operator.log").read_text())
                    report = local.drain()
                    self.assertEqual(report["stop_confirmed"], receipt)
                    self.assertEqual(report["clean"], receipt)
                    if not receipt: self.assertIn("OPERATOR_STOP_UNCONFIRMED", report["errors"])
                finally:
                    if process is not None and not process.done:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.process.wait(timeout=2)
                    local.close_logs()


if __name__ == "__main__":
    unittest.main()
