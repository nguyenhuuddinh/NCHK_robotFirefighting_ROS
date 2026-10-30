import json
import unittest

from services.operator_status import describe
from services.pairing import PairingAnnouncement, PairingError, PairingOutputParser, validate_binding


class PairingTests(unittest.TestCase):
    def test_chunked_current_session_is_paired_once(self):
        p = PairingOutputParser()
        directory = b"/home/pi/qa_logs/r5_automatic_20261029_140102"
        self.assertEqual(p.feed(b"LOG_PI=" + directory + b"\r\nPI_PAIR"), [])
        found = p.feed(b"ING=" + directory + b"/pairing.json\r\n")
        self.assertEqual(found[0].pairing_path, directory.decode() + "/pairing.json")
        with self.assertRaises(PairingError):
            p.feed(b"PI_PAIRING=" + directory + b"/pairing.json\n")

    def test_old_foreign_or_late_pairing_is_rejected(self):
        p = PairingOutputParser()
        p.feed(b"LOG_PI=/home/pi/qa_logs/r5_automatic_20261029_140102\n")
        with self.assertRaises(PairingError):
            p.feed(b"PI_PAIRING=/home/pi/qa_logs/r5_automatic_20261029_130102/pairing.json\n")
        p.close()
        with self.assertRaises(PairingError):
            p.feed(b"anything\n")

    def test_unbounded_line_cannot_accumulate(self):
        with self.assertRaises(PairingError):
            PairingOutputParser().feed(b"x" * 8193)

    def test_binding_checks_session_root_and_packet_without_log_secret(self):
        path = "/home/pi/qa_logs/r5_automatic_20261029_140102"
        announcement = PairingAnnouncement(path, path + "/pairing.json")
        packet = "a" * 64
        data = {"session": "f29c419c-f4af-414f-8054-4fe43d770ffe", "secret": "b" * 32,
                "packet": packet, "boot_id": "d71e43b6-d5d4-40f3-a219-02401df68e90",
                "runner_pid": 123, "runner_start_ticks": 456, "root": path}
        self.assertEqual(validate_binding(json.dumps(data).encode(), announcement, packet), data)
        for changed in ({**data, "packet": "c" * 64}, {**data, "root": path + "/old"},
                        {**data, "runner_pid": True}, {**data, "session": "old"}):
            with self.subTest(binding=changed), self.assertRaises(PairingError):
                validate_binding(json.dumps(changed).encode(), announcement, packet)


class OperatorStatusTests(unittest.TestCase):
    def report(self, **overrides):
        base = {"schema_version": 1, "scope": "R5_SESSION_INTENT_ONLY",
                "task": {"stage": "NAVIGATION", "reason": "GOAL"}}
        base.update(overrides)
        return json.dumps(base).encode()

    def test_missing_invalid_and_stale_do_not_become_ready(self):
        self.assertEqual(describe(None, None, 1)["quality"], "UNKNOWN")
        self.assertEqual(describe(b"{bad", 1, 1)["quality"], "INVALID")
        self.assertEqual(describe(self.report(), 1, 1.6)["quality"], "STALE")
        self.assertEqual(describe(self.report(), 2, 1)["quality"], "INVALID")

    def test_stop_request_is_not_stop_confirmation(self):
        pending = describe(self.report(shutdown={"requested": True, "done": False}), 1, 1.1)
        self.assertEqual(pending["label"], "Đang dừng phiên")
        unconfirmed = describe(self.report(shutdown={"requested": True, "done": True,
                                                      "confirmed": False}), 1, 1.1)
        self.assertEqual(unconfirmed["label"], "Chưa xác nhận dừng")

    def test_unknown_fire_is_not_no_fire_and_nonfinite_json_fails(self):
        fresh = describe(self.report(), 1, 1.1)
        self.assertIsNone(fresh["fire_confirmed"])
        bad = b'{"schema_version":1,"scope":"R5_SESSION_INTENT_ONLY","x":NaN}'
        self.assertEqual(describe(bad, 1, 1.1)["quality"], "INVALID")

    def test_wrong_types_and_duplicate_keys_do_not_crash_or_fake_ready(self):
        unusual = self.report(task={"stage": []}, runtime_rebind=["held"])
        self.assertEqual(describe(unusual, 1, 1.1)["quality"], "INVALID")
        duplicate = b'{"schema_version":1,"schema_version":1,"scope":"R5_SESSION_INTENT_ONLY"}'
        self.assertEqual(describe(duplicate, 1, 1.1)["quality"], "INVALID")


if __name__ == "__main__":
    unittest.main()
