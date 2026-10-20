"""Event-driven fire-session identity and cleanup; no actuation authority.

Callers must admit fresh confirmed targets before begin(), and establish
target absence/review policy before requesting closure. This core does not
replace perception, Nav2 cancel proof, arming, E-stop or pump quotas.
"""

from dataclasses import dataclass
import math
from uuid import uuid4


CLOSE_REASONS = frozenset({
    'TARGET_ABSENT_CONFIRMED', 'OPERATOR_CANCEL', 'NAVIGATION_ABORTED',
    'POST_SPRAY_CLEAR', 'SAFETY_ABORT',
})


def _nav_uuid(value):
    return (type(value) is str and len(value) == 32 and
            value != '0' * 32 and
            all(c in '0123456789abcdef' for c in value))


@dataclass(frozen=True)
class FireSessionIdentity:
    """A fire session has its own ID and an optional real Nav2 goal binding."""

    session_id: str
    nav_goal_uuid: object


@dataclass(frozen=True)
class SessionCloseRequest:
    """One immutable local close event, distinct from a heartbeat snapshot.

    requested_at uses the supervisor's monotonic clock. It must not be
    compared with a different host/process clock by a future ROS adapter.
    """

    session_id: str
    nav_goal_uuid: object
    generation: int
    reason: str
    requested_at: float


@dataclass(frozen=True)
class SessionCleanupEvidence:
    """Correlated receipt from verified adapters, not a ROS wire protocol.

    observed_at must cover fresh underlying evidence. close_generation must
    come from the bound close event, never be added to a legacy receipt.
    Setting these fields alone does not verify publishers or physical devices.
    """

    session_id: str
    nav_goal_uuid: object
    observed_at: float
    nav_inactive: bool = False
    motion_owner_released: bool = False
    pump_owner_released: bool = False
    pump_off: bool = False
    servo_idle: bool = False
    stationary: bool = False
    cmd_zero: bool = False
    nav_terminal_status: object = None
    post_spray_clear: bool = False
    close_generation: object = None


class FireSessionCore:
    """Retain one current and one closed identity without background work."""

    def __init__(self, *, proof_timeout_s):
        if (type(proof_timeout_s) not in (int, float) or
                not math.isfinite(proof_timeout_s) or proof_timeout_s <= 0):
            raise ValueError('cleanup proof timeout must be finite and positive')
        self.proof_timeout_s = float(proof_timeout_s)
        self._namespace = uuid4().hex
        self._sequence = 0
        self._close_sequence = 0
        self.identity = None
        self.phase = 'IDLE'
        self.close_reason = None
        self.close_requested_at = None
        self.close_request = None
        self.last_closed = None
        self.last_closed_request = None
        self.last_closed_evidence = None
        self.last_superseded_request = None
        self.blocker = None

    @property
    def source_instance_id(self):
        """Stable namespace of this core, including while no session is active."""
        return self._namespace

    def begin(self, nav_goal_uuid=None):
        """Accept a caller-admitted fire event; never create a fake Nav2 goal."""
        self.blocker = None
        if nav_goal_uuid is not None and not _nav_uuid(nav_goal_uuid):
            self.blocker = 'INVALID_NAV_GOAL_UUID'
            return None
        if self.phase == 'STOPPING':
            self.blocker = 'SESSION_CLEANUP_PENDING'
            return None
        if self.identity is not None:
            if nav_goal_uuid != self.identity.nav_goal_uuid:
                self.blocker = 'SESSION_GOAL_BINDING_CHANGED'
                return None
            return self.identity  # Duplicate acquisition does not start twice.
        self._sequence += 1
        self.identity = FireSessionIdentity(
            f'fire:{self._namespace}:{self._sequence}', nav_goal_uuid)
        self.phase = 'ACTIVE'
        self.close_reason = None
        self.close_requested_at = None
        self.close_request = None
        return self.identity

    def request_close(self, session_id, reason, now):
        """Latch closure of this session; no new session until cleanup is proven."""
        self.blocker = None
        if self.identity is None or session_id != self.identity.session_id:
            self.blocker = 'SESSION_ID_MISMATCH'
            return False
        if type(reason) is not str or reason not in CLOSE_REASONS:
            self.blocker = 'INVALID_CLOSE_REASON'
            return False
        if type(now) not in (int, float) or not math.isfinite(now):
            self.blocker = 'INVALID_TIME'
            return False
        if self.phase != 'STOPPING':
            self._close_sequence += 1
            self.close_request = SessionCloseRequest(
                self.identity.session_id, self.identity.nav_goal_uuid,
                self._close_sequence, reason, float(now))
            self.phase = 'STOPPING'
            self.close_reason = reason
            self.close_requested_at = float(now)
        return True  # Preserve the first reason/time on duplicate events.

    def renew_close_for_recovery(self, now, *, operator_confirmed=False):
        """An explicit new recovery event, never a heartbeat/duplicate close."""
        if (operator_confirmed is not True or self.phase != 'STOPPING' or
                self.close_request is None or type(now) not in (int, float) or
                not math.isfinite(now) or now < self.close_request.requested_at):
            return False
        self.last_superseded_request = self.close_request
        self._close_sequence += 1
        self.close_request = SessionCloseRequest(
            self.identity.session_id, self.identity.nav_goal_uuid,
            self._close_sequence, 'SAFETY_ABORT', float(now))
        self.close_reason, self.close_requested_at = 'SAFETY_ABORT', float(now)
        return True

    def finish_close(self, evidence, now):
        """Close only after fresh, exact-session stop/release evidence."""
        self.blocker = self._cleanup_blocker(evidence, now)
        if self.blocker is not None:
            return False
        self.last_closed = (self.identity, self.close_reason)
        self.last_closed_request = self.close_request
        self.last_closed_evidence = evidence
        self.identity = None
        self.phase = 'IDLE'
        self.close_reason = None
        self.close_requested_at = None
        self.close_request = None
        return True

    def can_retire_collectors(self, request, evidence, now):
        """Only the exact evidence just accepted by finish_close may retire a round."""
        return (self.phase == 'IDLE' and self.identity is None and self.close_request is None and
                request is not None and request == self.last_closed_request and
                evidence is self.last_closed_evidence and isinstance(evidence, SessionCleanupEvidence) and
                type(now) in (int, float) and math.isfinite(now) and
                0 <= now - evidence.observed_at <= self.proof_timeout_s)

    def _cleanup_blocker(self, evidence, now):
        if self.phase != 'STOPPING' or self.identity is None:
            return 'SESSION_NOT_STOPPING'
        if not isinstance(evidence, SessionCleanupEvidence):
            return 'CLEANUP_EVIDENCE_INVALID'
        if (evidence.session_id != self.identity.session_id or
                evidence.nav_goal_uuid != self.identity.nav_goal_uuid):
            return 'SESSION_ID_MISMATCH'
        if (self.close_request is None or
                type(evidence.close_generation) is not int or
                evidence.close_generation != self.close_request.generation):
            return 'CLOSE_GENERATION_MISMATCH'
        if (type(now) not in (int, float) or not math.isfinite(now) or
                type(evidence.observed_at) not in (int, float) or
                not math.isfinite(evidence.observed_at) or
                evidence.observed_at < self.close_requested_at or
                not 0 <= now - evidence.observed_at <= self.proof_timeout_s):
            return 'CLEANUP_EVIDENCE_STALE_OR_EARLY'
        if evidence.nav_inactive is not True:
            return 'NAVIGATION_STILL_ACTIVE'
        if self.identity.nav_goal_uuid is not None:
            # Session closure can accept a real terminal goal. This is NOT
            # the CANCELED-only admission proof for alignment or pumping.
            if (type(evidence.nav_terminal_status) is not int or
                    evidence.nav_terminal_status not in (4, 5, 6)):
                return 'NAV_GOAL_TERMINAL_PROOF_MISSING'
        elif evidence.nav_terminal_status is not None:
            return 'UNEXPECTED_NAV_GOAL_PROOF'
        for field, blocker in (
                ('motion_owner_released', 'MOTION_OWNER_PRESENT'),
                ('pump_owner_released', 'PUMP_OWNER_PRESENT'),
                ('pump_off', 'PUMP_NOT_OFF'), ('servo_idle', 'SERVO_NOT_IDLE'),
                ('stationary', 'ROBOT_NOT_STATIONARY'),
                ('cmd_zero', 'CMD_VEL_NOT_ZERO')):
            if getattr(evidence, field) is not True:
                return blocker
        if (self.close_reason == 'POST_SPRAY_CLEAR' and
                evidence.post_spray_clear is not True):
            return 'POST_SPRAY_REVIEW_MISSING'
        return None
