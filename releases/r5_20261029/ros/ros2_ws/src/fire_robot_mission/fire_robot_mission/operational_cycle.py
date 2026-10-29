"""Fire-cycle identity and original-I budget, independent of a Nav2 goal.

Pure bookkeeping: none of these objects is a motion/STOP/cancel/clear proof.
The operational adapter must validate graph, sensor age, BOOT, ownership and
stage-specific release evidence before using this context. No ROS/device I/O.
"""
from dataclasses import dataclass
import math
import uuid


ACTIVE_STATUSES = frozenset((1, 2, 3))
TERMINAL_STATUSES = frozenset((4, 5, 6))


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def zero(values, linear=.001, angular=.001):
    return (isinstance(values, (tuple, list)) and len(values) == 6
            and all(finite(v) for v in values)
            and all(abs(v) <= linear for v in values[:3])
            and all(abs(v) <= angular for v in values[3:]))


def valid_uuid(value):
    return (isinstance(value, str) and len(value) == 32 and value != '0' * 32
            and all(c in '0123456789abcdef' for c in value))


@dataclass(frozen=True)
class NavigationSnapshot:
    """Immutable validated action status data, not an authorization to move."""
    rows: tuple
    received_at: float

    @classmethod
    def parse(cls, rows, received_at):
        if not finite(received_at) or not isinstance(rows, (tuple, list)):
            raise ValueError('NAV_STATUS_INVALID')
        result = []
        for row in rows:
            if (not isinstance(row, (tuple, list)) or len(row) != 2
                    or not valid_uuid(row[0]) or type(row[1]) is not int
                    or row[1] not in ACTIVE_STATUSES | TERMINAL_STATUSES):
                raise ValueError('NAV_STATUS_INVALID')
            result.append(tuple(row))
        if len({r[0] for r in result}) != len(result):
            raise ValueError('NAV_STATUS_DUPLICATE_UUID')
        if sum(s in ACTIVE_STATUSES for _, s in result) > 1:
            raise ValueError('NAV_STATUS_MULTIPLE_ACTIVE_GOALS')
        return cls(tuple(result), float(received_at))

    def fresh(self, now):
        return finite(now) and 0 <= now - self.received_at <= .35

    @property
    def active_uuid(self):
        return next((u for u, s in self.rows if s in ACTIVE_STATUSES), None)


@dataclass(frozen=True)
class CycleIdentity:
    cycle_id: str
    nav_goal_uuid: str | None
    created_at: float


class ApproachBudget:
    """R4 limits: one I, 0.30 m/15 s including waits and recentering.

    Progress comes from the live IR core's cumulative odometry accounting.
    No helper here integrates commanded speed as a substitute for odometry.
    """
    DISTANCE_M = .30
    TIME_S = 15.

    def __init__(self, identity, *, unbounded=False):
        self.identity = identity
        self.unbounded = unbounded is True
        self.requested_at = None
        self.distance_m = 0.
        self.last_at = identity.created_at
        self.fault = None

    def _time(self, now):
        if self.fault:
            return False
        if not finite(now) or now < self.last_at:
            self.fault = 'CLOCK_DISCONTINUITY'
            return False
        self.last_at = now
        return True

    def request(self, now, *, admission_ready):
        if not self._time(now) or self.requested_at is not None or admission_ready is not True:
            return False
        self.requested_at = now
        return True

    def observe(self, cycle_id, cumulative_m, now):
        if not self._time(now):
            return False
        if cycle_id != self.identity.cycle_id:
            self.fault = 'IR_CYCLE_MISMATCH'
        elif self.requested_at is None:
            self.fault = 'IR_PROGRESS_BEFORE_I'
        elif not finite(cumulative_m) or cumulative_m < self.distance_m:
            self.fault = 'IR_DISTANCE_INVALID_OR_RESET'
        else:
            self.distance_m = cumulative_m
        return self.fault is None

    def report(self, now):
        self._time(now)
        elapsed = (max(0., now-self.requested_at) if finite(now)
                   and self.requested_at is not None else 0.)
        exhausted = (not self.unbounded and self.requested_at is not None and
                     (elapsed >= self.TIME_S or self.distance_m >= self.DISTANCE_M))
        return dict(requested_at=self.requested_at, distance_m=self.distance_m,
                    unbounded=self.unbounded, remaining_m=None if self.unbounded else max(0., self.DISTANCE_M-self.distance_m),
                    remaining_s=None if self.unbounded else max(0., self.TIME_S-elapsed),
                    exhausted=exhausted, fault=self.fault,
                    forward_budget_available=(self.requested_at is not None
                        and not exhausted and self.fault is None),
                    motion_authorized=False)


class CycleRegistry:
    """A target reacquisition retains the cycle, optional goal and I budget.

    Only explicit retirement after an external release handshake may clear a
    cycle. No target flag, Nav2 terminal status, or new goal implicitly does so.
    """
    def __init__(self, id_factory=None, *, unbounded_approach=False):
        self._id_factory = id_factory or (lambda: uuid.uuid4().hex)
        self.unbounded_approach = unbounded_approach is True
        self.current = None
        self.budget = None
        self.retired = set()
        self.last_at = None

    def _clock(self, now):
        if not finite(now) or (self.last_at is not None and now < self.last_at):
            raise ValueError('CLOCK_DISCONTINUITY')
        self.last_at = now

    def begin(self, rows, received_at, now, *, armed, fire_confirmed,
              retained_source_current=False):
        self._clock(now)
        snap = NavigationSnapshot.parse(rows, received_at)
        # Nav2 status is an event stream. The operational observer may retain
        # its last status while the exact, pinned publisher is still present.
        # This exception only creates context; it is never no-active STOP proof.
        if not snap.fresh(now) and not (retained_source_current is True
                                        and received_at <= now):
            raise ValueError('NAV_STATUS_STALE')
        if armed is not True or fire_confirmed is not True:
            raise ValueError('EXPLICIT_FIRE_PERMISSION_REQUIRED')
        if self.current is not None:
            # A new goal never overwrites a latched fire cycle.
            if snap.active_uuid not in (None, self.current.nav_goal_uuid):
                raise ValueError('NEW_GOAL_DURING_FIRE_CYCLE')
            return self.current
        cycle_id = self._id_factory()
        if (not valid_uuid(cycle_id) or cycle_id in self.retired
                or cycle_id in {u for u, _ in snap.rows}):
            raise ValueError('CYCLE_ID_INVALID_OR_REUSED')
        self.current = CycleIdentity(cycle_id, snap.active_uuid, now)
        self.budget = ApproachBudget(self.current, unbounded=self.unbounded_approach)
        return self.current

    def retire(self, receipt, now, *, expected_context):
        """Consume the coordinator's typed, current, exact all-peer receipt."""
        from .cycle_release import ReleaseReceipt, ReleaseContext, ROLES
        self._clock(now)
        if (not isinstance(receipt, ReleaseReceipt) or not isinstance(receipt.context, ReleaseContext)
                or receipt.context.as_dict()!=expected_context or receipt.roles!=ROLES
                or not valid_uuid(receipt.request_id) or not finite(receipt.completed_at)
                or not 0<=now-receipt.completed_at<=.35 or self.current is None
                or receipt.context.cycle_id!=self.current.cycle_id
                or receipt.context.nav_goal_uuid!=self.current.nav_goal_uuid):
            raise ValueError('CURRENT_EXACT_RELEASE_RECEIPT_REQUIRED')
        self.retired.add(self.current.cycle_id)
        self.current=None;self.budget=None

    def retirement_request(self):
        """Identity to bind future STOP/OFF/clear/ACK; this does not retire it."""
        if self.current is None:
            return None
        return dict(cycle_id=self.current.cycle_id,
                    nav_goal_uuid=self.current.nav_goal_uuid)

    def report(self, now):
        self._clock(now)
        return dict(scope='CYCLE_CONTEXT_ONLY_NOT_MOTION_AUTHORITY',
                    cycle=self.retirement_request(),
                    budget=self.budget.report(now) if self.budget else None,
                    requires_stop_release_handshake=self.current is not None,
                    motion_authorized=False)
