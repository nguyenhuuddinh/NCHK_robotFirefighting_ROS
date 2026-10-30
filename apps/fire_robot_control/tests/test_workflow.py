"""User action sequences against the offline policy, never hardware."""
import json
import random
import unittest

from services.workflow import Phase, Workflow, WorkflowError


PACKET = "a" * 64
ROOT = "/home/pi/qa_logs/r5_automatic_20261029_123456"


def pairing(**overrides):
    fields = dict(session="8518a120-a012-4db3-bbf3-f73f67aed61e",
                  secret="1" * 32, packet=PACKET,
                  boot_id="a41182f4-ce1a-4a41-aed2-e93f5f9b6389",
                  runner_pid=42, runner_start_ticks=7, root=ROOT)
    fields.update(overrides)
    return json.dumps(fields).encode()


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.flow = Workflow(PACKET)

    def mapping(self):
        effect = self.flow.scan(ownership_clear=True)
        self.flow.mapping_ready(effect.token, owned_and_ready=True)
        return effect.token

    def start_fire(self):
        self.flow.select_map("selected-map-record")
        return self.flow.start_fire(ownership_clear=True, map_matches=True).token

    def read_pairing(self, token):
        self.assertIsNone(self.flow.pi_output(token, ("LOG_PI=" + ROOT + "\nPI_PAIR").encode(), owned_pi_alive=True))
        effect = self.flow.pi_output(token, ("ING=" + ROOT + "/pairing.json\n").encode(), owned_pi_alive=True)
        self.assertEqual(effect.kind, "READ_PAIRING")

    def running(self):
        token = self.start_fire()
        self.read_pairing(token)
        effect = self.flow.pairing_ready(token, pairing(), owned_pi_alive=True, map_matches=True)
        self.assertEqual(effect.kind, "START_LAPTOP")
        self.assertNotIn("1" * 32, repr(effect))
        self.flow.fire_ready(token, verified=True)
        return token

    def test_scan_double_click_save_double_click_and_saved_map_selection(self):
        token = self.mapping()
        with self.assertRaises(WorkflowError):
            self.flow.scan(ownership_clear=True)
        request = self.flow.save("new_map")
        with self.assertRaises(WorkflowError):
            self.flow.save("again")
        with self.assertRaises(WorkflowError):
            self.flow.start_fire(ownership_clear=True, map_matches=True)
        self.assertTrue(self.flow.save_finished(token, operation=request.operation, record="new-map"))
        self.assertEqual(self.flow.selected_map, "new-map")
        self.assertEqual(self.flow.phase, Phase.MAPPING)

    def test_previous_save_callback_cannot_finish_new_save(self):
        token = self.mapping()
        first = self.flow.save("one")
        self.assertTrue(self.flow.save_finished(token, operation=first.operation, record="one"))
        second = self.flow.save("two")
        self.assertFalse(self.flow.save_finished(token, operation=first.operation, record="stale"))
        self.assertEqual(self.flow.phase, Phase.SAVING)
        self.assertTrue(self.flow.save_finished(token, operation=second.operation, record="two"))
        self.assertEqual(self.flow.selected_map, "two")

    def test_save_failure_preserves_previous_selection_and_mapping(self):
        token = self.mapping()
        self.flow.select_map("existing")
        request = self.flow.save("new")
        self.assertFalse(self.flow.save_finished(token, operation=request.operation, error="disk full"))
        self.assertEqual((self.flow.phase, self.flow.selected_map), (Phase.MAPPING, "existing"))

    def test_stop_during_save_ignores_late_result_and_blocks_restart(self):
        token = self.mapping()
        request = self.flow.save("new")
        self.assertEqual(self.flow.stop().kind, "STOP_MAPPING")
        self.assertIsNone(self.flow.stop())
        self.assertFalse(self.flow.save_finished(token, operation=request.operation, record="late"))
        self.assertIsNone(self.flow.selected_map)
        self.flow.stop_finished(token, robot_stop_confirmed=False, cleanup_clean=True)
        with self.assertRaises(WorkflowError):
            self.flow.scan(ownership_clear=True)

    def test_clean_mapping_stop_permits_fire_with_saved_map(self):
        token = self.mapping()
        request = self.flow.save("room")
        self.flow.save_finished(token, operation=request.operation, record="room")
        self.flow.stop()
        self.flow.stop_finished(token, robot_stop_confirmed=True, cleanup_clean=True)
        fire = self.flow.start_fire(ownership_clear=True, map_matches=True)
        self.assertEqual(fire.kind, "START_PI")
        self.assertNotEqual(fire.token, token)
        self.assertIsNone(self.flow.mapping_ready(token, owned_and_ready=True))

    def test_start_requires_map_revalidation_and_no_other_session(self):
        with self.assertRaises(WorkflowError):
            self.flow.start_fire(ownership_clear=True, map_matches=True)
        self.flow.select_map("map")
        for clear, match in ((False, True), (True, False)):
            with self.subTest(clear=clear, match=match), self.assertRaises(WorkflowError):
                self.flow.start_fire(ownership_clear=clear, map_matches=match)
        self.assertEqual(self.flow.phase, Phase.IDLE)

    def test_pairing_wrong_packet_or_map_changed_requests_stop(self):
        for raw, match in ((pairing(packet="b" * 64), True), (pairing(), False)):
            with self.subTest(match=match):
                self.flow = Workflow(PACKET)
                token = self.start_fire()
                self.read_pairing(token)
                result = self.flow.pairing_ready(token, raw, owned_pi_alive=True, map_matches=match)
                self.assertEqual(result.kind, "REQUEST_STOP")
                self.assertEqual(self.flow.phase, Phase.STOPPING)

    def test_stop_during_pairing_ignores_late_pairing_and_ready(self):
        token = self.start_fire()
        self.read_pairing(token)
        self.flow.stop()
        self.assertIsNone(self.flow.pairing_ready(token, pairing(), owned_pi_alive=True, map_matches=True))
        self.assertIsNone(self.flow.fire_ready(token, verified=True))
        self.assertEqual(self.flow.phase, Phase.STOPPING)

    def test_x_repeated_until_verified_and_q_takes_precedence(self):
        token = self.running()
        first = self.flow.cancel()
        self.assertEqual(first.kind, "CANCEL_CYCLE")
        self.assertIsNone(self.flow.cancel())
        self.flow.cancel_finished(token, operation=first.operation, verified=False)
        self.assertIsNone(self.flow.cancel())
        self.flow.cancel_finished(token, operation=first.operation, verified=True)
        second = self.flow.cancel()
        self.flow.cancel_finished(token, operation=first.operation, verified=True)
        self.assertIsNone(self.flow.cancel())
        self.assertEqual(self.flow.stop().kind, "REQUEST_STOP")
        self.flow.cancel_finished(token, operation=second.operation, verified=True)
        self.assertIsNone(self.flow.cancel())
        self.assertEqual(self.flow.phase, Phase.STOPPING)

    def test_unconfirmed_stop_or_degraded_cleanup_never_permits_restart(self):
        for stopped, clean in ((False, True), (True, False), (False, False)):
            with self.subTest(stopped=stopped, clean=clean):
                self.flow = Workflow(PACKET)
                token = self.running()
                self.flow.stop()
                self.flow.stop_finished(token, robot_stop_confirmed=stopped, cleanup_clean=clean)
                self.assertEqual(self.flow.phase, Phase.BLOCKED)
                with self.assertRaises(WorkflowError):
                    self.flow.start_fire(ownership_clear=True, map_matches=True)

    def test_startup_failure_and_pi_loss_request_stop_once(self):
        token = self.start_fire()
        self.assertEqual(self.flow.pi_output(token, b"", owned_pi_alive=False).kind, "REQUEST_STOP")
        self.assertIsNone(self.flow.stop())
        self.assertIsNone(self.flow.pi_output(token, b"", owned_pi_alive=True))

    def test_delayed_duplicate_pairing_requests_stop(self):
        token = self.start_fire()
        self.read_pairing(token)
        result = self.flow.pi_output(token, ("PI_PAIRING=" + ROOT + "/pairing.json\n").encode(), owned_pi_alive=True)
        self.assertEqual(result.kind, "REQUEST_STOP")
        self.assertIsNone(self.flow.pairing_ready(token, pairing(), owned_pi_alive=True, map_matches=True))

    def test_source_loss_or_timeout_stops_each_live_phase(self):
        setup = (lambda: self.flow.scan(ownership_clear=True).token,
                 lambda: self.mapping(), lambda: self.start_fire(), lambda: self.running())
        for prepare in setup:
            self.flow = Workflow(PACKET)
            token = prepare()
            self.assertIsNone(self.flow.source_lost("stale"))
            self.assertIn(self.flow.source_lost(token).kind, ("STOP_MAPPING", "REQUEST_STOP"))
            self.assertIsNone(self.flow.source_lost(token))
            self.assertEqual(self.flow.phase, Phase.STOPPING)

    def test_old_session_stop_receipt_cannot_stop_new_session(self):
        old = self.mapping()
        self.flow.stop()
        self.flow.stop_finished(old, robot_stop_confirmed=True, cleanup_clean=True)
        new = self.mapping()
        self.flow.stop()
        self.assertFalse(self.flow.stop_finished(old, robot_stop_confirmed=True, cleanup_clean=True))
        self.assertEqual(self.flow.phase, Phase.STOPPING)
        self.assertTrue(self.flow.stop_finished(new, robot_stop_confirmed=True, cleanup_clean=True))

    def test_many_interleaved_button_and_delayed_callback_sequences(self):
        for seed in range(100):
            rng = random.Random(seed)
            flow = Workflow(PACKET)
            last_save = None
            old_tokens = ["foreign"]
            for _ in range(100):
                before = flow.phase
                action = rng.randrange(11)
                token = flow.token or "foreign"
                effect = None
                try:
                    if action == 0:
                        effect = flow.scan(ownership_clear=rng.choice((True, False)))
                    elif action == 1:
                        effect = flow.mapping_ready(token, owned_and_ready=rng.choice((True, False)))
                    elif action == 2:
                        effect = flow.save("room")
                        last_save = effect
                    elif action == 3 and last_save is not None:
                        flow.save_finished(token, operation=last_save.operation, record="room")
                    elif action == 4:
                        flow.select_map("room")
                    elif action == 5:
                        effect = flow.start_fire(ownership_clear=True, map_matches=True)
                    elif action == 6:
                        effect = flow.stop()
                    elif action == 7:
                        flow.stop_finished(rng.choice((token, rng.choice(old_tokens))),
                            robot_stop_confirmed=rng.choice((True, False)),
                            cleanup_clean=rng.choice((True, False)))
                    elif action == 8:
                        effect = flow.cancel()
                    elif action == 9:
                        effect = flow.source_lost(rng.choice((token, rng.choice(old_tokens))))
                    else:
                        flow.fire_ready(token, verified=rng.choice((True, False)))
                except WorkflowError:
                    pass
                if effect is not None:
                    if effect.kind in ("START_PI", "START_MAPPING"):
                        self.assertEqual(before, Phase.IDLE, (seed, action))
                        old_tokens.append(effect.token)
                    if effect.kind == "SAVE_MAP":
                        self.assertEqual(before, Phase.MAPPING, (seed, action))
                    if effect.kind == "CANCEL_CYCLE":
                        self.assertEqual(before, Phase.RUNNING_FIRE, (seed, action))
                if before is Phase.BLOCKED:
                    self.assertEqual(flow.phase, Phase.BLOCKED, (seed, action))
                if before is Phase.STOPPING and flow.phase is Phase.IDLE:
                    self.assertEqual((flow.stop_status, flow.cleanup_status), ("CONFIRMED", "CLEAN"))


if __name__ == "__main__":
    unittest.main()
