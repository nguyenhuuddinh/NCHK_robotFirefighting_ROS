import json
import os
from pathlib import Path
import stat
import tempfile
import unittest

from services.pairing import PairingAnnouncement
from services.pairing_transfer import PairingTransfer, PairingTransferError


PACKET = "a" * 64
ROOT = "/home/pi/qa_logs/r5_automatic_20261029_123456"
ANNOUNCEMENT = PairingAnnouncement(ROOT, ROOT + "/pairing.json")


def binding(**overrides):
    fields = dict(session="8518a120-a012-4db3-bbf3-f73f67aed61e", secret="1" * 32,
                  packet=PACKET, boot_id="a41182f4-ce1a-4a41-aed2-e93f5f9b6389",
                  runner_pid=42, runner_start_ticks=7, root=ROOT)
    fields.update(overrides)
    return json.dumps(fields).encode()


class FakeChild:
    def __init__(self, argv): self.argv, self.rc, self.closed = argv, None, False
    def poll(self): return self.rc
    def close_after_exit(self): self.closed = True


class PairingTransferTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.created = []
    def tearDown(self):
        for transfer in self.created:
            if transfer.child.poll() is None:
                transfer.child.rc = 1
            transfer.close_after_exit()
        self.tmp.cleanup()
    def transfer(self, **kwargs):
        child_args = dict(host="10.0.0.1", runtime_dir=self.root,
                          assert_owned_pi=lambda: None, child_factory=FakeChild)
        child_args.update(kwargs)
        item = PairingTransfer(ANNOUNCEMENT, PACKET, **child_args)
        self.created.append(item)
        return item

    def test_private_transfer_checks_binding_and_never_exposes_secret_in_argv(self):
        transfer = self.transfer()
        self.assertEqual(stat.S_IMODE(transfer.directory.stat().st_mode), 0o700)
        self.assertEqual(stat.S_IMODE(transfer.path.stat().st_mode), 0o600)
        self.assertEqual(transfer.child.argv[-2], "pi@10.0.0.1:" + ANNOUNCEMENT.pairing_path)
        self.assertNotIn("1" * 32, repr(transfer.child.argv))
        with self.assertRaises(PairingTransferError): transfer.finish()
        with self.assertRaises(PairingTransferError): transfer.close_after_exit()
        transfer.path.write_bytes(binding())
        transfer.child.rc = 0
        self.assertEqual(transfer.finish(), transfer.path)
        with self.assertRaises(PairingTransferError): transfer.finish()

    def test_bad_packet_missing_data_and_failed_transfer_never_returns_file(self):
        for raw, rc in ((binding(packet="b" * 64), 0), (b"", 0), (binding(), 1)):
            with self.subTest(rc=rc, raw=raw[:10]):
                item = self.transfer()
                item.path.write_bytes(raw)
                item.child.rc = rc
                with self.assertRaises(PairingTransferError): item.finish()

    def test_replaced_file_or_symlink_is_rejected_and_user_file_preserved(self):
        item = self.transfer()
        other = self.root / "user-file"
        other.write_bytes(binding())
        item.path.unlink()
        item.path.symlink_to(other)
        item.child.rc = 0
        with self.assertRaises(PairingTransferError): item.finish()
        self.assertEqual(other.read_bytes(), binding())

    def test_pi_ownership_is_checked_before_during_and_after_read(self):
        calls = []
        def owned():
            calls.append(True)
            if len(calls) == 3: raise RuntimeError("Pi exited")
        item = self.transfer(assert_owned_pi=owned)
        item.path.write_bytes(binding())
        item.child.rc = 0
        with self.assertRaisesRegex(RuntimeError, "Pi exited"): item.finish()
        self.assertEqual(len(calls), 3)

    def test_unknown_host_wrong_path_or_public_directory_is_rejected_before_child(self):
        with self.assertRaises(PairingTransferError): self.transfer(host="example.org")
        invalid = PairingAnnouncement(ROOT, ROOT + "/../foreign.json")
        with self.assertRaises(PairingTransferError):
            PairingTransfer(invalid, PACKET, host="10.0.0.1", runtime_dir=self.root,
                            assert_owned_pi=lambda: None, child_factory=FakeChild)
        os.chmod(self.root, 0o755)
        with self.assertRaises(PairingTransferError): self.transfer()
        os.chmod(self.root, 0o700)


if __name__ == "__main__":
    unittest.main()
