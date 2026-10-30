"""Request actual AMCL evidence after a late pose; never synthesize readiness.

Humble's Empty service sets force_update_ for a subsequent laser update. The
response is NOT a pose/STOP proof. Only the existing pose callback can admit it.
"""
import math


class PoseRefresh:
    RETRY_S = 5.
    REQUEST_TIMEOUT_S = 2.

    def __init__(self, client, request):
        self.client, self.request = client, request
        self.future = None
        self.sent = None
        self.attempts = 0
        self.state = 'IDLE'

    def _forget(self):
        self.future.cancel()
        self.client.remove_pending_request(self.future)
        self.future = None

    def step(self, row, detail, pose_at, now):
        if self.future is not None:
            if self.future.done():
                try:
                    self.future.result()
                    self.state = 'WAIT_ACTUAL_POSE'
                except Exception:
                    self.state = 'REQUEST_FAILED'
                self.future = None
            elif now - self.sent >= self.REQUEST_TIMEOUT_S:
                self._forget()
                self.state = 'REQUEST_TIMEOUT'
        sample = row.get('sample_mono')
        needed = ('pose_source', 'initial_pose_seen', 'pose_after_initial',
                  'map_tf', 'action_ready', 'lifecycle', 'identity_present')
        eligible = (isinstance(sample, (int, float)) and math.isfinite(sample)
            and 0 <= now - sample <= .75
            and row.get('checks', {}).get('io') is True
            and row.get('stable_s', {}).get('io', 0.) >= 1.
            and detail.get('pose_valid') is False
            and all(detail.get(k) is True for k in needed)
            and isinstance(pose_at, (int, float)) and math.isfinite(pose_at)
            and now - pose_at >= 1.)
        if detail.get('pose_valid') is True:
            self.state = 'VALID_POSE_OBSERVED'
        if (not eligible or self.future is not None
                or self.sent is not None and now - self.sent < self.RETRY_S):
            return
        self.sent = now
        if not self.client.service_is_ready():
            self.state = 'SERVICE_UNAVAILABLE'
            return
        self.attempts += 1
        try:
            self.future = self.client.call_async(self.request())
            self.state = 'REQUESTED_ACTUAL_AMCL_UPDATE'
        except Exception:
            self.state = 'REQUEST_FAILED'

    def report(self):
        return dict(state=self.state, attempts=self.attempts,
                    response_is_pose_proof=False, motion_authorized=False)
