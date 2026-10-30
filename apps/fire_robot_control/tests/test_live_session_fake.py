"""Real session worker with synthetic Pi/runner endpoints; no network or ROS."""
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
import uuid
from unittest.mock import patch

from services.live_session import LiveSession, atomic
from proof_fixture import clean_local, clean_remote

PACKET = "a" * 64
PI_ROOT = "/home/pi/qa_logs/r5_automatic_20261029_120000"


class FakeChild:
    def __init__(self, output=b""):
        self.output, self.code, self.writes = output, None, []
        self.process = type("Process", (), {"pid": 12345})()
    def read(self):
        result, self.output = self.output, b""
        return result
    def poll(self): return self.code
    def write(self, data):
        self.writes.append(data)
        if data == b"\x03": self.code = 0
        return len(data)
    def close_after_exit(self):
        if self.code is None: raise RuntimeError("still active")


class FakeTransfer:
    def __init__(self, announcement, packet, *, runtime_dir, assert_owned_pi, **kwargs):
        assert_owned_pi()
        self.owner = assert_owned_pi.__self__
        self.child = FakeChild()
        self.child.code = 0
        self.path = Path(runtime_dir) / "fake_pairing.json"
        self.binding = dict(session=str(uuid.uuid4()), secret="1" * 32, packet=packet,
            boot_id=str(uuid.uuid4()), runner_pid=12, runner_start_ticks=345, root=announcement.log_dir)
        self.path.write_text(json.dumps(self.binding))
    def finish(self):
        if self.owner.inject_duplicate:
            self.owner.pi.output = ("PI_PAIRING=" + PI_ROOT + "/pairing.json\n").encode()
        return self.path
    def close_after_exit(self): self.path.unlink()


class FakeOwnedRunner:
    def __init__(self, owner, *, valid=True):
        self.owner, self.valid = owner, valid
        self.process, self.done, self.rc, self.forced = object(), False, None, False
        folder = owner.out / "laptop"
        folder.mkdir()
        atomic(folder / "startup.json", dict(session=owner.session, stage="READY_GATE2"))
    def tick(self, now): pass
    def stop(self, now):
        if self.done: return
        assert self.owner.pi.poll() is None, "Pi must remain alive through laptop drain"
        self.owner.order.append("laptop-drain")
        binding = {key: value for key, value in self.owner.remote_binding.items() if key != "secret"}
        atomic(self.owner.out / "laptop/receipt.json", dict(session=self.owner.session,
            packet=self.owner.packet if self.valid else "b" * 64, rc=0, profile="R5_AUTOMATIC",
            local=clean_local(),
            remote=clean_remote(binding)))
        self.done, self.rc = True, 0
    def report(self): return dict(group_exit_confirmed=self.done)


class FakeLiveSession(LiveSession):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.order = []
        self.valid_receipt = True
        self.inject_duplicate = False
    def _owned_pi(self):
        if self.pi.poll() is not None: raise RuntimeError("PI_RUNNER_EXITED")
    def _launch_pi(self):
        self.pi = FakeChild(("LOG_PI=" + PI_ROOT + "\nPI_PAIRING=" + PI_ROOT + "/pairing.json\n").encode())
        old_write = self.pi.write
        def write(data):
            if data == b"\x03": self.order.append("pi-stop")
            return old_write(data)
        self.pi.write = write
        return "/fake/staging/ros"
    def _spawn_laptop(self, remote, pairing_path):
        self.runner_log = open(os.devnull, "wb")
        self.local = FakeOwnedRunner(self, valid=self.valid_receipt)


class LiveSessionTests(unittest.TestCase):
    def test_missing_pi_receipt_stops_before_ssh_and_does_not_claim_physical_stop(self):
        with tempfile.TemporaryDirectory(prefix="fire-test-") as tmp:
            base = Path(tmp)
            for name in ("candidate", "runtime", "out", "maps"):
                (base / name).mkdir(mode=0o700)
            (base / "candidate/desktop").mkdir()
            events = []
            runner = FakeLiveSession(candidate=base / "candidate", packet=PACKET, runtime=base / "runtime",
                out=base / "out", map_root=base / "maps", map_roots=[base / "maps"],
                kind="fire", selected_map=None, emit=events.append, pi_receipt=base / "missing")
            preflight = type("Result", (), {"returncode": 0, "stderr": ""})()
            with patch("services.live_session.subprocess.run", return_value=preflight):
                runner.run()
            self.assertIsNone(runner.pi)
            final = [event for event in events if event.get("event") == "FINISHED"][-1]
            self.assertIsNone(final["robot_stop_confirmed"])
            self.assertFalse(final["session_was_started"])
            self.assertIn("receipt", final["error"])

    def test_stop_preempts_a_ready_pairing_predicate(self):
        with tempfile.TemporaryDirectory(prefix="fire-test-") as tmp:
            base = Path(tmp)
            runner = FakeLiveSession(candidate=base, packet=PACKET, runtime=base, out=base,
                map_root=base, map_roots=[base], kind="fire", selected_map=None, emit=lambda _: None)
            runner.pi = FakeChild()
            runner.send("STOP")
            called = []
            with self.assertRaises(InterruptedError):
                runner._wait(lambda: called.append(True) or True, 1)
            self.assertEqual(called, [])

    def run_case(self, valid, duplicate=False):
        with tempfile.TemporaryDirectory(prefix="fire-test-") as tmp:
            base = Path(tmp)
            for name in ("candidate", "runtime", "out", "maps"):
                (base / name).mkdir(mode=0o700)
            (base / "candidate/desktop").mkdir()
            events = []
            runner = FakeLiveSession(candidate=base / "candidate", packet=PACKET, runtime=base / "runtime",
                out=base / "out", map_root=base / "maps", map_roots=[base / "maps"],
                kind="fire", selected_map=None, emit=events.append, pi_receipt=base / "fake-receipt")
            runner.valid_receipt = valid
            runner.inject_duplicate = duplicate
            preflight = type("Result", (), {"returncode": 0, "stderr": ""})()
            with patch("services.live_session.PairingTransfer", FakeTransfer), \
                 patch("services.live_session.subprocess.run", return_value=preflight), \
                 patch("services.live_session.validate_pi_receipt", return_value={}):
                runner.start()
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline and runner.stage != "READY_GATE2" and runner.thread.is_alive():
                    runner.heartbeat(time.monotonic())
                    time.sleep(.01)
                if not duplicate:
                    self.assertEqual(runner.stage, "READY_GATE2", events)
                    runner.send("STOP")
                while runner.thread.is_alive() and time.monotonic() < deadline:
                    runner.heartbeat(time.monotonic())
                    time.sleep(.01)
                runner.thread.join(timeout=.2)
                self.assertFalse(runner.thread.is_alive(), events)
            self.assertEqual(runner.order, ["laptop-drain", "pi-stop"])
            final = [event for event in events if event.get("event") == "FINISHED"][-1]
            return final

    def test_verified_receipt_after_laptop_drain_and_pi_stop(self):
        result = self.run_case(True)
        self.assertTrue(result["robot_stop_confirmed"])
        self.assertTrue(result["cleanup_clean"])

    def test_foreign_packet_receipt_blocks_success_even_if_processes_exit(self):
        result = self.run_case(False)
        self.assertFalse(result["robot_stop_confirmed"])
        self.assertFalse(result["cleanup_clean"])

    def test_duplicate_pairing_after_transfer_requests_stop_and_drains_before_pi(self):
        result = self.run_case(True, duplicate=True)
        self.assertIn("PairingError", result["error"])

    def test_pi_exits_after_laptop_started_drain_before_receipt_arrives(self):
        for valid in (True, False):
            with self.subTest(valid=valid), tempfile.TemporaryDirectory() as tmp:
                base=Path(tmp)
                for name in ('candidate','runtime','out','maps'):(base/name).mkdir()
                events=[]
                runner=FakeLiveSession(candidate=base/'candidate',packet=PACKET,runtime=base/'runtime',
                    out=base/'out',map_root=base/'maps',map_roots=[base/'maps'],kind='fire',
                    selected_map=None,emit=events.append,pi_receipt=base/'fake')
                original_spawn=runner._spawn_laptop
                def spawn(*args):
                    original_spawn(*args)
                    count=[0]
                    def tick(now):
                        count[0]+=1
                        if count[0]==1:
                            atomic(runner.out/'laptop/startup.json',dict(session=runner.session,stage='STOPPING'))
                            runner.pi.code=0
                        elif count[0]==2:
                            binding={k:v for k,v in runner.remote_binding.items() if k!='secret'}
                            atomic(runner.out/'laptop/receipt.json',dict(session=runner.session,packet=PACKET,
                                rc=0,local=clean_local() if valid else {},remote=clean_remote(binding)))
                            runner.local.done=True;runner.local.rc=0
                    runner.local.tick=tick
                with patch.object(runner,'_spawn_laptop',side_effect=spawn), \
                        patch('services.live_session.PairingTransfer',FakeTransfer), \
                        patch('services.live_session.subprocess.run',return_value=type('R',(),{'returncode':0})()), \
                        patch('services.live_session.validate_pi_receipt',return_value={}):
                    runner.run()
                result=[x for x in events if x['event']=='FINISHED'][-1]
                self.assertEqual(result['robot_stop_confirmed'],valid,result)
                self.assertEqual(result['cleanup_clean'],valid,result)
                self.assertIsNone(result['error'],result)


if __name__ == "__main__": unittest.main()
