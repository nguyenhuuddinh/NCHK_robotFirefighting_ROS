"""Paced operator loop for a future integrated R5 entrypoint.

The caller owns node construction, signal handling and the existing R5
finally/STOP verification and cleanup. This helper by itself does not start an
operator or provide STOP proof. ROS callbacks and node.tick remain on one
thread, as in the original operator's main loop.
"""
from .operator_adapter import OperatorAdapter


TICK_PERIOD_S = 0.05
CALLBACK_MIN_WINDOW_S = 0.01


def operator_loop(node, channel, *, ros, clock, stopping):
    adapter = OperatorAdapter(node, channel)
    next_tick = clock()
    while ros.ok():
        remaining = max(0.0, next_tick - clock())
        spin_at = clock()
        ros.spin_once(node, timeout_sec=min(0.01, remaining))
        from .operator_timing import observe_query
        observe_query(node, 'spin_once', clock()-spin_at, clock())
        signal_stop = stopping()
        if signal_stop:
            node.request_shutdown("SIGNAL")
        tick_at = clock()
        if tick_at < next_tick and not signal_stop:
            continue
        # The signal path must still tick shutdown even without a frontend.
        report = node.tick(None) if signal_stop else adapter.tick(tick_at)
        if report is not None and (report.get("shutdown") or {}).get("done"):
            return report
        finished = clock()
        # After a slow graph evaluation, let queued telemetry callbacks drain
        # before paying for another evaluation. Never replay missed ticks.
        cost = finished-tick_at
        # Reserve room within the existing 350ms authority interval. A slow
        # 200ms query must not be followed by another 200ms of deliberate wait.
        callback_window = (.1 if cost > .35 else
                           min(.05, max(CALLBACK_MIN_WINDOW_S, .30-cost)))
        next_tick = max(tick_at + TICK_PERIOD_S, finished + callback_window)
    return None


def drain_operator_stop(node, *, ros, clock):
    """Pace the unchanged STOP verifier while ROS callbacks keep draining."""
    next_check = clock()
    while ros.ok() and not node.stop_verification.done:
        remaining = max(0., next_check-clock())
        ros.spin_once(node, timeout_sec=min(.01, remaining))
        if clock() < next_check:
            continue
        node.stop_tick()
        next_check = clock()+TICK_PERIOD_S
