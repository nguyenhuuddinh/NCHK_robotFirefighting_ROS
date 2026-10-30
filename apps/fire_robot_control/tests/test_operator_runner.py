import os
import io
from contextlib import redirect_stderr
from pathlib import Path
import signal
from types import SimpleNamespace
import unittest

from services.operator_runner import OperatorBootstrap, run_integrated


SESSION = "8518a120-a012-4db3-bbf3-f73f67aed61e"
PACKET = "a" * 64


class Clock:
    def __init__(self): self.now = 0.
    def __call__(self): return self.now


class FakeRos:
    def __init__(self, clock, until=10.):
        self.clock, self.active, self.until = clock, False, until
        self.shutdown_called = False
    def init(self, **_): self.active = True
    def ok(self): return self.active and self.clock.now < self.until
    def spin_once(self, node, timeout_sec): self.clock.now += max(.001, timeout_sec)
    def shutdown(self): self.active = False; self.shutdown_called = True


class Verification:
    def __init__(self): self.done = False; self.confirmed = False; self.aborted = None
    def abort(self, reason): self.aborted = reason
    def report(self, now): return {"done": self.done, "confirmed": self.confirmed}


class FakeNode:
    def __init__(self, packet=PACKET, view=None, tick_error=False):
        self.view, self.core = view, SimpleNamespace(fault=None)
        self.packet, self.tick_error = packet, tick_error
        self.stop_verification = Verification()
        self.reasons, self.keys, self.logs = [], [], []
        self.destroyed, self.stopping = False, False
        self.logger = SimpleNamespace(error=self.logs.append, info=self.logs.append)
    def get_parameter(self, name):
        return SimpleNamespace(value={"automatic_mode": True, "automatic_runner_session": SESSION,
                                      "automatic_packet": self.packet}[name])
    def get_logger(self): return self.logger
    def request_shutdown(self, reason): self.reasons.append(reason); self.stopping = True
    def tick(self, key):
        self.keys.append(key)
        if self.tick_error: raise RuntimeError("secret must not appear in log")
        if key == "Q": self.stopping = True
        if self.stopping and len(self.keys) >= 2:
            self.stop_verification.done = self.stop_verification.confirmed = True
        return {"shutdown": {"done": self.stop_verification.done}}
    def stop_tick(self): self.stop_verification.done = self.stop_verification.confirmed = True
    def destroy_node(self): self.destroyed = True


class FakeChannel:
    def __init__(self, path, lease, *, error_on_close=False):
        self.lease, self.closed, self.calls, self.error_on_close = lease, False, 0, error_on_close
    def poll(self, now):
        self.calls += 1
        if self.calls == 1:
            self.lease.active = True
            return None
        return "Q" if self.calls == 2 else None
    def close(self):
        self.closed = True
        if self.error_on_close: raise OSError("close failed")


class OperatorRunnerTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.ros = FakeRos(self.clock)
        self.bootstrap = OperatorBootstrap(SESSION, "b" * 32, "c" * 32, PACKET, Path("/tmp/private/control.sock"))
        self.channel = None
    def channel_factory(self, path, lease):
        self.channel = FakeChannel(path, lease)
        return self.channel
    def run_node(self, node):
        return run_integrated(self.bootstrap, ros=self.ros, node_factory=lambda: node,
                              channel_factory=self.channel_factory, clock=self.clock)

    def test_q_runs_existing_stop_and_cleans_up_after_proof(self):
        node = FakeNode()
        self.assertEqual(self.run_node(node), 0)
        self.assertIn("Q", node.keys)
        self.assertTrue(node.stop_verification.confirmed)
        self.assertEqual(node.reasons[-1], "FINALLY")
        self.assertTrue(node.destroyed and self.channel.closed and self.ros.shutdown_called)

    def test_wrong_packet_never_opens_channel_but_drains_node(self):
        node = FakeNode(packet="d" * 64)
        self.assertEqual(self.run_node(node), 2)
        self.assertIsNone(self.channel)
        self.assertEqual(node.keys, [])
        self.assertTrue(node.stop_verification.confirmed and node.destroyed and self.ros.shutdown_called)

    def test_operator_exception_is_not_logged_with_sensitive_text_and_drains(self):
        node = FakeNode(tick_error=True)
        self.assertEqual(self.run_node(node), 2)
        self.assertTrue(node.stop_verification.confirmed and self.channel.closed and self.ros.shutdown_called)
        self.assertNotIn("secret", " ".join(node.logs))

    def test_frontend_close_error_is_failure_but_ros_still_shuts_down(self):
        node = FakeNode()
        def factory(path, lease):
            self.channel = FakeChannel(path, lease, error_on_close=True)
            return self.channel
        self.assertEqual(run_integrated(self.bootstrap, ros=self.ros, node_factory=lambda: node,
                                       channel_factory=factory, clock=self.clock), 2)
        self.assertTrue(node.stop_verification.confirmed and node.destroyed and self.ros.shutdown_called)

    def test_existing_context_and_invalid_bootstrap_rejected_before_node(self):
        self.assertNotIn("c" * 32, repr(self.bootstrap))
        self.ros.active = True
        with self.assertRaises(RuntimeError):
            self.run_node(FakeNode())
        self.ros.active = False
        invalid = OperatorBootstrap(SESSION, "b" * 32, "short", PACKET, Path("relative"))
        with self.assertRaises(ValueError):
            run_integrated(invalid, ros=self.ros, node_factory=FakeNode, clock=self.clock)
        self.assertFalse(self.ros.active)

    def test_context_loss_is_unconfirmed_and_not_a_success(self):
        self.ros.until = .04
        node = FakeNode()
        self.assertEqual(self.run_node(node), 2)
        self.assertEqual(node.stop_verification.aborted, "ROS_CONTEXT_LOST")
        self.assertFalse(node.stop_verification.confirmed)

    def test_signal_during_startup_drains_and_restores_handler(self):
        class SignalRos(FakeRos):
            fired = False
            def spin_once(self, node, timeout_sec):
                super().spin_once(node, timeout_sec)
                if not self.fired:
                    self.fired = True
                    signal.raise_signal(signal.SIGINT)
        self.ros = SignalRos(self.clock)
        node = FakeNode()
        previous = signal.getsignal(signal.SIGINT)
        self.assertEqual(self.run_node(node), 0)
        self.assertIn("SIGNAL", node.reasons)
        self.assertTrue(node.stop_verification.confirmed and node.destroyed)
        self.assertEqual(signal.getsignal(signal.SIGINT), previous)

    def test_node_construction_failure_is_reported_without_sensitive_details(self):
        def broken():
            raise RuntimeError("nonce-secret")
        captured = io.StringIO()
        with redirect_stderr(captured):
            self.assertEqual(run_integrated(self.bootstrap, ros=self.ros, node_factory=broken,
                                           channel_factory=self.channel_factory, clock=self.clock), 2)
        self.assertIn("RuntimeError", captured.getvalue())
        self.assertNotIn("nonce-secret", captured.getvalue())
        self.assertTrue(self.ros.shutdown_called)


if __name__ == "__main__":
    unittest.main()
