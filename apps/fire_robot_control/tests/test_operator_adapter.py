import unittest

from services.operator_adapter import OperatorAdapter


class FakeNode:
    def __init__(self):
        self.keys = []
        self.shutdown_reasons = []

    def tick(self, key):
        self.keys.append(key)
        return {"shutdown": {"done": False}, "key": key}

    def request_shutdown(self, reason):
        self.shutdown_reasons.append(reason)


class FakeChannel:
    def __init__(self, result, active=True):
        self.result = result
        self.polls = 0
        self.lease = type("FakeLease", (), {"active": active})()

    def poll(self, now):
        self.polls += 1
        if isinstance(self.result, Exception):
            raise self.result
        return self.result


class OperatorAdapterTests(unittest.TestCase):
    def test_x_and_q_are_passed_to_existing_operator_tick(self):
        for key in (None, "X", "Q"):
            with self.subTest(key=key):
                node = FakeNode()
                report = OperatorAdapter(node, FakeChannel(key)).tick(10.)
                self.assertEqual(node.keys, [key])
                self.assertFalse(report["shutdown"]["done"])
                self.assertEqual(node.shutdown_reasons, ["Q"] if key == "Q" else [])

    def test_channel_error_or_unexpected_command_requests_shutdown_once(self):
        for result in (OSError("failed"), "A", "P", "ESC"):
            with self.subTest(result=result):
                node = FakeNode()
                channel = FakeChannel(result)
                adapter = OperatorAdapter(node, channel)
                adapter.tick(10.)
                adapter.tick(10.1)
                self.assertEqual(node.shutdown_reasons, ["FRONTEND_CHANNEL_ERROR"])
                self.assertEqual(node.keys, [None, None])
                self.assertEqual(channel.polls, 1)

    def test_operator_tick_exception_is_not_silenced(self):
        class BrokenNode(FakeNode):
            def tick(self, key):
                raise RuntimeError("existing operator failure")
        with self.assertRaisesRegex(RuntimeError, "existing operator failure"):
            OperatorAdapter(BrokenNode(), FakeChannel("Q")).tick(10.)

    def test_no_operator_tick_before_first_frontend_heartbeat(self):
        node = FakeNode()
        channel = FakeChannel(None, active=False)
        adapter = OperatorAdapter(node, channel)
        self.assertIsNone(adapter.tick(10.))
        self.assertEqual(node.keys, [])
        channel.lease.active = True
        adapter.tick(10.1)
        self.assertEqual(node.keys, [None])

    def test_missing_frontend_q_can_stop_before_first_heartbeat(self):
        node = FakeNode()
        OperatorAdapter(node, FakeChannel("Q", active=False)).tick(10.)
        self.assertEqual(node.keys, ["Q"])

    def test_missing_frontend_keeps_ticking_stop_after_q(self):
        node = FakeNode()
        channel = FakeChannel("Q", active=False)
        adapter = OperatorAdapter(node, channel)
        adapter.tick(10.)
        channel.result = None
        adapter.tick(10.1)
        adapter.tick(10.2)
        self.assertEqual(node.keys, ["Q", None, None])

    def test_no_later_action_is_forwarded_after_stop(self):
        node = FakeNode()
        channel = FakeChannel("Q", active=True)
        adapter = OperatorAdapter(node, channel)
        adapter.tick(10.)
        channel.result = "X"
        adapter.tick(10.1)
        self.assertEqual(node.keys, ["Q", None])
        self.assertEqual(channel.polls, 1)


if __name__ == "__main__":
    unittest.main()
