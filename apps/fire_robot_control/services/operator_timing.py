"""Bounded timing diagnostics; never participate in safety admission."""
import json


def observe_query(node, stage, elapsed, now):
    if elapsed <= .35:
        return
    previous = getattr(node, '_app_query_warning_at', None)
    if previous is not None and now-previous < 1.:
        return
    node._app_query_warning_at = now
    node.get_logger().warning('APP_OPERATOR_QUERY_DELAY ' + json.dumps(
        dict(stage=stage, elapsed_ms=round(elapsed*1000, 3), sample_mono=now), allow_nan=False))
