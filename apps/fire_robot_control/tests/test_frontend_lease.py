import json
import unittest

from services.frontend_lease import FrontendLease


class FrontendLeaseTests(unittest.TestCase):
    def setUp(self):
        self.lease = FrontendLease("session", "gen1", "secret", 1000, 10.0)

    def send(self, event, seq, now, *, uid=1000, **changes):
        msg = dict(version=1, session="session", generation="gen1", nonce="secret",
                   seq=seq, sent_mono=now, event=event)
        msg.update(changes)
        return self.lease.ingest(json.dumps(msg).encode(), now, uid)

    def test_x_q_order_and_q_is_idempotent(self):
        self.assertTrue(self.send("HEARTBEAT", 1, 10.1))
        self.assertTrue(self.send("X", 2, 10.2))
        self.assertTrue(self.send("Q", 3, 10.21))
        self.assertEqual(self.lease.next_key(10.22), "Q")
        self.assertIsNone(self.lease.next_key(10.23))
        self.assertFalse(self.send("X", 4, 10.24))

    def test_frontend_freeze_stops_even_if_backend_alive(self):
        self.assertTrue(self.send("HEARTBEAT", 1, 10.1))
        self.assertIsNone(self.lease.next_key(10.4))
        self.assertEqual(self.lease.next_key(10.46), "Q")
        self.assertEqual(self.lease.reason, "FRONTEND_LEASE_EXPIRED")
        self.assertFalse(self.send("HEARTBEAT", 2, 10.47))

    def test_late_heartbeat_cannot_revive_expired_lease(self):
        self.assertTrue(self.send("HEARTBEAT", 1, 10.1))
        self.assertFalse(self.send("HEARTBEAT", 2, 10.46))
        self.assertEqual(self.lease.next_key(10.47), "Q")

    def test_wrong_identity_replay_and_old_event_rejected(self):
        self.assertFalse(self.send("X", 1, 10.1))
        self.assertFalse(self.send("HEARTBEAT", 1, 10.1, uid=1001))
        self.assertFalse(self.send("HEARTBEAT", 1, 10.1, generation="old"))
        self.assertFalse(self.send("HEARTBEAT", 1, 10.1, nonce="🔑"))
        self.assertFalse(self.send("HEARTBEAT", 1, 10.1, nonce="\ud800"))
        self.assertTrue(self.send("HEARTBEAT", 1, 10.1))
        self.assertFalse(self.send("X", 1, 10.2))
        self.assertFalse(self.send("X", 2, 10.3, sent_mono=9.0))
        self.assertIsNone(self.lease.next_key(10.31))

    def test_missing_frontend_or_clock_regression_requests_q(self):
        self.assertEqual(self.lease.next_key(13.01), "Q")
        other = FrontendLease("s", "g", "n", 1, 10.0)
        self.assertEqual(other.next_key(9.0), "Q")
        self.assertEqual(other.reason, "FRONTEND_CLOCK_INVALID")

    def test_unsupported_command_duplicate_keys_and_bad_json_rejected(self):
        self.assertTrue(self.send("HEARTBEAT", 1, 10.1))
        self.assertFalse(self.send("P", 2, 10.2))
        self.assertFalse(self.lease.ingest(b'{"version":1,"version":1}', 10.21, 1000))
        self.assertFalse(self.lease.ingest(b"x" * 2049, 10.22, 1000))


if __name__ == "__main__":
    unittest.main()
