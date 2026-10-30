"""Cross-module scenarios with local fake children and fake STOP proof only."""
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import time
import unittest

from services.frontend_lease import FrontendLease
from services.map_catalog import still_matches
from services.map_save import save_map_pair
from services.operator_channel import OperatorChannel
from services.pairing import PairingAnnouncement
from services.pairing_transfer import PairingTransfer
from services.pty_child import PtyChild
from services.workflow import Phase, Workflow, WorkflowError


PACKET = "a" * 64
ROOT = "/home/pi/qa_logs/r5_automatic_20261029_123456"


def fake_binding():
    return json.dumps(dict(session="8518a120-a012-4db3-bbf3-f73f67aed61e",
        secret="1" * 32, packet=PACKET, boot_id="a41182f4-ce1a-4a41-aed2-e93f5f9b6389",
        runner_pid=42, runner_start_ticks=7, root=ROOT)).encode()


def make_map(prefix):
    prefix.with_suffix(".pgm").write_bytes(b"P5\n2 2\n255\n\x00\x00\xff\xff")
    prefix.with_suffix(".yaml").write_text(
        f"image: {prefix.name}.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


def wire(event, seq, when):
    return json.dumps(dict(version=1, session="session", generation="generation", nonce="nonce",
        seq=seq, sent_mono=when, event=event)).encode() + b"\n"


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.children = []

    def tearDown(self):
        for child in self.children:
            if child.poll() is None:
                child.process.kill()  # local test fixture only
                child.process.wait(timeout=2)
            child.close_after_exit()
        self.tmp.cleanup()

    def fake_child(self, script):
        child = PtyChild([sys.executable, "-u", "-c", script])
        self.children.append(child)
        return child

    def saved_map(self, flow):
        scan = flow.scan(ownership_clear=True)
        flow.mapping_ready(scan.token, owned_and_ready=True)
        save = flow.save("room")
        record = save_map_pair(self.root, "room", assert_owned_mapping=lambda: None, generate=make_map)
        flow.save_finished(scan.token, operation=save.operation, record=record)
        flow.stop()
        flow.stop_finished(scan.token, robot_stop_confirmed=True, cleanup_clean=True)
        return record

    def pi_until_pairing(self, flow, record):
        start = flow.start_fire(ownership_clear=True, map_matches=still_matches(record, [self.root]))
        pi = self.fake_child("print('LOG_PI=" + ROOT + "',flush=True); "
            "print('PI_PAIRING=" + ROOT + "/pairing.json',flush=True); input()")
        effect = None
        deadline = time.monotonic() + 3
        while effect is None and time.monotonic() < deadline:
            data = pi.read(32)
            if data:
                effect = flow.pi_output(start.token, data, owned_pi_alive=pi.poll() is None)
            time.sleep(.005)
        self.assertIsNotNone(effect)
        self.assertEqual(effect.kind, "READ_PAIRING")
        return start.token, pi

    def test_scan_save_pair_start_cancel_and_stop_keep_pi_alive_through_drain(self):
        flow = Workflow(PACKET)
        record = self.saved_map(flow)
        token, pi = self.pi_until_pairing(flow, record)
        start = flow.pairing_ready(token, fake_binding(), owned_pi_alive=pi.poll() is None,
                                   map_matches=still_matches(record, [self.root]))
        self.assertEqual(start.kind, "START_LAPTOP")
        laptop = self.fake_child("print('R5_AUTOMATIC_READY',flush=True); input()")
        deadline = time.monotonic() + 3
        output = b""
        while b"R5_AUTOMATIC_READY" not in output and time.monotonic() < deadline:
            output += laptop.read()
            time.sleep(.005)
        self.assertIn(b"R5_AUTOMATIC_READY", output)
        self.assertEqual(flow.phase, Phase.STARTING_FIRE)  # banner isn't ready proof
        flow.fire_ready(token, verified=True)  # explicitly synthetic fixture proof

        lease = FrontendLease("session", "generation", "nonce", os.getuid(), 10.)
        with OperatorChannel(self.root / "channel.sock", lease) as channel:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.connect(str(self.root / "channel.sock"))
                client.sendall(wire("HEARTBEAT", 1, 10.1) + wire("X", 2, 10.1))
                self.assertEqual(channel.poll(10.1), "X")
                self.assertEqual(flow.cancel().kind, "CANCEL_CYCLE")
                client.sendall(wire("Q", 3, 10.2))
                self.assertEqual(channel.poll(10.2), "Q")
                self.assertEqual(flow.stop().kind, "REQUEST_STOP")
                self.assertIsNone(flow.stop())
                self.assertIsNone(pi.poll())
                laptop.write(b"done\n")
                laptop.process.wait(timeout=2)
                self.assertIsNone(pi.poll())  # Pi outlives local child drain
                pi.write(b"done\n")
                pi.process.wait(timeout=2)
                flow.stop_finished(token, robot_stop_confirmed=True, cleanup_clean=True)
        self.assertEqual(flow.phase, Phase.IDLE)

    def test_changed_map_between_pairing_and_laptop_blocks_start(self):
        flow = Workflow(PACKET)
        record = self.saved_map(flow)
        token, pi = self.pi_until_pairing(flow, record)
        Path(record.image_path).write_bytes(b"P5\n2 2\n255\n\x00\x00\x00\x00")
        effect = flow.pairing_ready(token, fake_binding(), owned_pi_alive=pi.poll() is None,
                                    map_matches=still_matches(record, [self.root]))
        self.assertEqual(effect.kind, "REQUEST_STOP")
        self.assertEqual(flow.phase, Phase.STOPPING)
        with self.assertRaises(WorkflowError):
            flow.start_fire(ownership_clear=True, map_matches=True)
        pi.write(b"done\n")
        pi.process.wait(timeout=2)

    def test_private_pairing_transfer_with_local_fake_pty_child(self):
        announcement = PairingAnnouncement(ROOT, ROOT + "/pairing.json")
        def fake_scp(argv):
            self.assertEqual(argv[-2], "pi@10.0.0.1:" + announcement.pairing_path)
            # Actual local PTY, with input supplied by the fixture instead of SSH.
            child = PtyChild([sys.executable, "-u", "-c", "import sys; "
                "data=sys.stdin.buffer.readline(); open(sys.argv[1],'wb').write(data)", argv[-1]])
            self.children.append(child)
            return child
        transfer = PairingTransfer(announcement, PACKET, host="10.0.0.1", runtime_dir=self.root,
                                   assert_owned_pi=lambda: None, child_factory=fake_scp)
        try:
            payload = fake_binding() + b"\n"
            self.assertEqual(transfer.child.write(payload), len(payload))
            transfer.child.process.wait(timeout=3)
            self.assertEqual(transfer.finish().read_bytes().strip(), fake_binding())
            self.assertNotIn(payload.strip(), transfer.child.read())
        finally:
            transfer.close_after_exit()


if __name__ == "__main__":
    unittest.main()
