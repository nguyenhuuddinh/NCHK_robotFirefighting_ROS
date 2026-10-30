import json
import unittest

from services.session_model import SessionError, SessionModel


PACKET = "a" * 64
ROOT = "/home/pi/qa_logs/r5_automatic_20261029_123456"


def binding(**overrides):
    value = dict(session="8518a120-a012-4db3-bbf3-f73f67aed61e",
                 secret="1" * 32, packet=PACKET,
                 boot_id="a41182f4-ce1a-4a41-aed2-e93f5f9b6389",
                 runner_pid=42, runner_start_ticks=7, root=ROOT)
    value.update(overrides)
    return json.dumps(value).encode()


class SessionModelTests(unittest.TestCase):
    def make_waiting(self):
        model = SessionModel(PACKET)
        self.assertEqual(model.begin("map", map_matches=True, no_other_owned_session=True), "START_PI")
        return model

    def announce(self, model):
        self.assertIsNone(model.pi_output(("LOG_PI=" + ROOT + "\nPI_PAIR").encode(), owned_pi_alive=True))
        self.assertEqual(model.pi_output(("ING=" + ROOT + "/pairing.json\n").encode(), owned_pi_alive=True), "READ_PAIRING")

    def test_startup_requires_matching_pairing_and_verified_ready(self):
        model = self.make_waiting()
        self.announce(model)
        self.assertEqual(model.pairing_copied(binding(), owned_pi_alive=True, map_matches=True), "START_LAPTOP")
        with self.assertRaises(SessionError):
            model.backend_ready(verified=False)
        model.backend_ready(verified=True)
        self.assertEqual(model.state, "RUNNING")

    def test_wrong_packet_or_changed_map_blocks_laptop(self):
        for raw, match in ((binding(packet="b" * 64), True), (binding(), False)):
            model = self.make_waiting()
            self.announce(model)
            with self.assertRaises(SessionError):
                model.pairing_copied(raw, owned_pi_alive=True, map_matches=match)
            self.assertEqual(model.state, "STOP_REQUIRED")

    def test_lost_pi_during_pairing_requires_stop(self):
        model = self.make_waiting()
        with self.assertRaises(SessionError):
            model.pi_output(b"anything\n", owned_pi_alive=False)
        self.assertEqual(model.state, "STOP_REQUIRED")

    def test_stop_during_pairing_blocks_late_data_and_is_idempotent(self):
        model = self.make_waiting()
        self.assertEqual(model.request_stop(), "REQUEST_STOP")
        self.assertIsNone(model.request_stop())
        with self.assertRaises(SessionError):
            model.pi_output(("LOG_PI=" + ROOT + "\n").encode(), owned_pi_alive=True)
        model.finish_stop(robot_stop_confirmed=False, cleanup_clean=True)
        self.assertEqual(model.stop_status, "UNCONFIRMED")
        self.assertEqual(model.cleanup_status, "CLEAN")
        with self.assertRaises(SessionError):
            model.begin("map", map_matches=True, no_other_owned_session=True)

    def test_cleanup_failure_is_not_hidden_by_confirmed_stop(self):
        model = self.make_waiting()
        model.request_stop()
        model.finish_stop(robot_stop_confirmed=True, cleanup_clean=False)
        self.assertEqual((model.stop_status, model.cleanup_status), ("CONFIRMED", "DEGRADED"))

