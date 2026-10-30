import unittest

from services.operator_loop import operator_loop


class Clock:
    def __init__(self):
        self.now = 0.0
    def __call__(self):
        return self.now


class FakeRos:
    def __init__(self, clock, until=2.0):
        self.clock, self.until, self.calls = clock, until, 0
    def ok(self):
        return self.clock.now < self.until
    def spin_once(self, node, timeout_sec):
        self.calls += 1
        self.clock.now += max(timeout_sec, .001)


class FakeNode:
    def __init__(self, clock):
        self.clock, self.calls, self.reasons = clock, [], []
        self.stopping = False
    def request_shutdown(self, reason):
        self.reasons.append(reason)
        self.stopping = True
    def tick(self, key):
        self.calls.append((self.clock.now, key))
        if key == "Q":
            self.stopping = True
        return {"shutdown": {"done": self.stopping and len(self.calls) >= 3}}


class Channel:
    def __init__(self, clock, activate_at=.2, q_at=.5):
        self.clock, self.activate_at, self.q_at = clock, activate_at, q_at
        self.lease = type("Lease", (), {"active": False})()
        self.delivered = False
    def poll(self, now):
        self.lease.active = now >= self.activate_at
        if now >= self.q_at and not self.delivered:
            self.delivered = True
            return "Q"
        return None


class OperatorLoopTests(unittest.TestCase):
    def test_ros_callbacks_drain_before_ui_but_operator_waits_for_frontend(self):
        clock = Clock()
        ros = FakeRos(clock)
        node = FakeNode(clock)
        result = operator_loop(node, Channel(clock), ros=ros, clock=clock, stopping=lambda: False)
        self.assertTrue(result["shutdown"]["done"])
        self.assertGreater(ros.calls, len(node.calls))
        self.assertGreaterEqual(node.calls[0][0], .2)
        self.assertEqual(sum(key == "Q" for _, key in node.calls), 1)

    def test_signal_before_ui_connect_still_runs_existing_stop_tick(self):
        clock = Clock()
        node = FakeNode(clock)
        ros = FakeRos(clock)
        result = operator_loop(node, Channel(clock, activate_at=10), ros=ros,
                               clock=clock, stopping=lambda: clock.now >= .1)
        self.assertTrue(result["shutdown"]["done"])
        self.assertEqual(node.reasons[0], "SIGNAL")
        self.assertTrue(all(key is None for _, key in node.calls))

    def test_ros_context_loss_does_not_claim_shutdown(self):
        clock = Clock()
        node = FakeNode(clock)
        ros = FakeRos(clock, until=.1)
        result = operator_loop(node, Channel(clock, activate_at=10), ros=ros,
                               clock=clock, stopping=lambda: False)
        self.assertIsNone(result)
        self.assertEqual(node.calls, [])

    def test_tick_failure_propagates_to_caller_finally(self):
        class BrokenNode(FakeNode):
            def tick(self, key):
                raise RuntimeError("operator failed")
        clock = Clock()
        with self.assertRaisesRegex(RuntimeError, "operator failed"):
            operator_loop(BrokenNode(clock), Channel(clock, activate_at=0), ros=FakeRos(clock),
                          clock=clock, stopping=lambda: False)

    def test_callback_drain_does_not_itself_exceed_authority_interval(self):
        class SlowNode(FakeNode):
            def tick(self, key):
                result = super().tick(key)
                self.clock.now += .2
                return result
        clock = Clock()
        node = SlowNode(clock)
        ros = FakeRos(clock, until=3.)
        result = operator_loop(node, Channel(clock, activate_at=0, q_at=1.), ros=ros,
                               clock=clock, stopping=lambda: False)
        self.assertTrue(result['shutdown']['done'])
        intervals = [b[0]-a[0] for a,b in zip(node.calls, node.calls[1:])]
        self.assertTrue(intervals)
        self.assertTrue(all(.2 < interval < .35 for interval in intervals))


if __name__ == "__main__":
    unittest.main()
