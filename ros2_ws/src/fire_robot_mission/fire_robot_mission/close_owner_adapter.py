"""Small opt-in glue for existing owner callbacks; no node, timer or authority."""

import time

from fire_robot_mission.close_owner_evidence import OwnerCloseReporter
from fire_robot_mission.session_context import SessionContextGuard


def configure_owner_observation(node, owner, *, live_enabled, context, timeout_s):
    """Keep report-only context separate from the motion admission context."""
    node.declare_parameter('enable_close_owner_observation', False)
    enabled = node.get_parameter('enable_close_owner_observation').value
    if type(enabled) is not bool:
        raise ValueError('enable_close_owner_observation must be a boolean')
    if enabled and live_enabled and context is None:
        raise ValueError('live close-owner observation requires session-bound migration')
    node._close_owner_context = (
        context if context is not None else SessionContextGuard(timeout_s=min(timeout_s, 0.35))
    ) if enabled else None
    node._close_owner_reporter = OwnerCloseReporter(owner) if enabled else None
    node._close_owner_live_enabled = live_enabled if enabled else None


def start_owner_observation(node):
    """Read the source clock before graph/evaluation/effects, only when selected."""
    if getattr(node, '_close_owner_reporter', None) is None:
        return None
    return node.get_clock().now().nanoseconds


def append_owner_observation(node, payload, stamp_ns, *, publisher_owned):
    """Caller invokes after effects; failures must propagate before reaching here."""
    reporter = getattr(node, '_close_owner_reporter', None)
    if reporter is not None:
        payload['close_owner_observation'] = reporter.report(
            node._close_owner_context, publisher_owned=publisher_owned,
            stamp_ns=stamp_ns, now=time.monotonic())
        payload['close_owner_observation']['live_enabled'] = getattr(node, '_close_owner_live_enabled', None)
