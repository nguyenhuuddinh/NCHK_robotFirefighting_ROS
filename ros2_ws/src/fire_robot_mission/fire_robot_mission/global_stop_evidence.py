"""One coherent post-close stop snapshot; no ROS effects or rearm authority."""

from dataclasses import dataclass
import math

from fire_robot_mission.actuator_freshness import ActuatorStampGuard
from fire_robot_mission.close_owner_evidence import CLOSE_OWNERS, CloseOwnersReadiness
from fire_robot_mission.fire_session_core import (
    CLOSE_REASONS, SessionCleanupEvidence, SessionCloseRequest,
)
from fire_robot_mission.session_context import _session_id_matches, _uuid
from fire_robot_mission.runtime_manifest import runtime_manifest


STOP_SOURCES = ('odom', 'cmd', 'actuator', 'nav_status') + CLOSE_OWNERS


def _time(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


def _gid(value):
    return type(value) is bytes and len(value) in (16, 24) and any(value)


def _axes(values):
    return (type(values) in (tuple, list) and len(values) == 6 and
            all(type(v) in (int, float) and math.isfinite(v) for v in values))


@dataclass(frozen=True)
class StopSample:
    value: object
    stamp_ns: int
    received_at: float
    effective_at: float
    source_age_s: float
    source_id: bytes


@dataclass(frozen=True)
class GlobalStopReadiness:
    ready: bool
    blockers: tuple
    evidence: object


class GlobalStopEvidence:
    """Four latest inputs, eight source pins and a fixed close barrier.

    A ready result is telemetry/graph evidence, not physical verification.
    It never closes a session, changes arm/watchdog, or resets any budget.
    The caller must supply exact endpoint checks and source timestamps (DDS
    metadata for unstamped Twist/status). Missing timestamps are not receipt time.
    """

    timeout_s = 0.35
    future_ns = 20_000_000
    zero_hold_s = 0.5

    def __init__(self, *, linear_limit=0.02, angular_limit=0.05,
                 timeout_s=0.35, future_skew_ms=20.0, manifest_id='all_four_external_v1',
                 require_recovery_on_change=False, require_through_status=False):
        if (not _time(linear_limit) or not 0 < linear_limit <= 0.02 or
                not _time(angular_limit) or not 0 < angular_limit <= 0.05):
            raise ValueError('global stationary limits may not weaken existing limits')
        if (not _time(timeout_s) or not 0 < timeout_s <= 0.35 or
                not _time(future_skew_ms) or future_skew_ms > 20.0):
            raise ValueError('global stop must retain freshness/skew limits')
        self.timeout_s, self.future_ns = timeout_s, round(future_skew_ms * 1e6)
        if type(require_recovery_on_change) is not bool:
            raise ValueError('recovery policy must be a bool')
        self.require_recovery_on_change = require_recovery_on_change
        if type(require_through_status) is not bool:
            raise ValueError('through-status policy must be a bool')
        self.require_through_status = require_through_status
        self.telemetry_roles = STOP_SOURCES[:4] + (('nav_through_status',) if require_through_status else ())
        self.manifest = runtime_manifest(manifest_id)
        self.source_roles = self.telemetry_roles + self.manifest.required
        self.linear_limit, self.angular_limit = linear_limit, angular_limit
        self.request = self.barrier_ros_ns = self.barrier_at = None
        self.fault = None
        self.samples = dict.fromkeys(self.telemetry_roles)
        self.reasons = dict.fromkeys(self.telemetry_roles, 'NOT_RECEIVED')
        self._watermarks = dict.fromkeys(self.telemetry_roles)
        self._pins = dict.fromkeys((*STOP_SOURCES, *self.telemetry_roles))
        self._last_ros = self._last_mono = self._clock_anchor = None
        self.clock_fault_evidence = None
        self.clock_sampling = {'calls': 0, 'retries': 0, 'uncertain': 0,
                               'max_span_ns': 0, 'last': None}
        self._hold_since = self._hold_last = None
        self.graph = None
        self.graph_reason = 'GLOBAL_GRAPH_NOT_RECEIVED'
        self.actuator_guard = ActuatorStampGuard(
            max_age_ms=timeout_s * 1000.0, max_future_skew_ms=future_skew_ms)
        self._actuator_identity = None

    def observe_clock_sample(self, sample):
        """Bounded diagnostics; an unusable local read cannot authorize cleanup."""
        info = self.clock_sampling
        info['calls'] += 1
        info['retries'] += sample.attempts - 1
        info['uncertain'] += int(sample.reason is not None)
        # Invalid values must remain a fault without making the diagnostic
        # heartbeat contain NaN, infinity or a non-JSON object.
        clean = lambda value: value if _time(value) else None
        info['last'] = {'ros_ns': clean(sample.ros_ns), 'before': clean(sample.before),
                        'after': clean(sample.after), 'span_ns': sample.span_ns,
                        'attempts': sample.attempts, 'reason': sample.reason,
                        'widest_span_ns': sample.widest_span_ns}
        info['max_span_ns'] = max(info['max_span_ns'], sample.widest_span_ns)
        if sample.reason is not None:
            if self.fault is None:
                self.clock_fault_evidence = {
                    'reason': sample.reason, 'sample': dict(info['last']),
                    'anchor': self._clock_anchor,
                    'previous': (self._last_ros, self._last_mono)}
            self.fault = self.fault or sample.reason
            self._hold_since = self._hold_last = None

    def _clock(self, ros_ns, now):
        previous_fault = self.fault
        previous = self._last_ros, self._last_mono
        if type(ros_ns) is not int or ros_ns <= 0 or not _time(now):
            self.fault = self.fault or 'GLOBAL_CLOCK_INVALID'
        elif ((self._last_ros is not None and ros_ns < self._last_ros) or
              (self._last_mono is not None and now < self._last_mono)):
            self.fault = self.fault or 'GLOBAL_CLOCK_BACKWARD'
        else:
            if self._clock_anchor is not None:
                old_ros, old_mono = self._clock_anchor
                if abs((ros_ns - old_ros) - round((now - old_mono) * 1e9)) > self.future_ns:
                    self.fault = self.fault or 'GLOBAL_CLOCK_DISCONTINUITY'
            else:
                self._clock_anchor = ros_ns, now
            self._last_ros, self._last_mono = ros_ns, now
        if self.fault is not None:
            if previous_fault is None:
                self.clock_fault_evidence = {
                    'reason': self.fault, 'ros_ns': ros_ns, 'mono': now,
                    'anchor': self._clock_anchor, 'previous': previous,
                    'threshold_ns': self.future_ns,
                    'sampling': self.clock_sampling['last']}
            self._hold_since = self._hold_last = None
            return False
        return True

    def begin_close(self, request, *, announced_ros_ns, announced_at):
        if self.request is not None:
            if request != self.request:
                self.fault = self.fault or 'GLOBAL_CLOSE_ROUND_CHANGED'
            return self.fault is None
        session = request.session_id if isinstance(request, SessionCloseRequest) else None
        parts = session.split(':') if type(session) is str else []
        namespace = parts[1] if len(parts) == 3 else None
        if (not isinstance(request, SessionCloseRequest) or not _uuid(namespace) or
                not _session_id_matches(session, namespace) or
                (request.nav_goal_uuid is not None and not _uuid(request.nav_goal_uuid)) or
                type(request.generation) is not int or request.generation <= 0 or
                type(request.reason) is not str or request.reason not in CLOSE_REASONS or
                not _time(request.requested_at) or not _time(announced_at) or
                announced_at < request.requested_at or not self._clock(announced_ros_ns, announced_at)):
            return False
        self.request = request
        self.barrier_ros_ns, self.barrier_at = announced_ros_ns, announced_at
        self._hold_since = self._hold_last = None
        self._actuator_identity = self.actuator_guard.last_identity
        return True

    def _pin(self, role, source_id):
        if not _gid(source_id):
            return False
        previous = self._pins[role]
        if (self.request is not None or self.require_recovery_on_change) and previous not in (None, source_id):
            self.fault = self.fault or 'GLOBAL_SOURCE_CHANGED:' + role
            return False
        self._pins[role] = source_id
        return True

    def _admit(self, role, value, stamp_ns, source_id, ros_ns, now, *, age_s=0.0):
        if not self._clock(ros_ns, now):
            self.reasons[role] = self.fault
            return
        reason = None
        if not self._pin(role, source_id):
            reason = self.fault or 'SOURCE_INVALID'
        elif type(stamp_ns) is not int or stamp_ns <= 0 or not _time(age_s):
            reason = 'STAMP_OR_AGE_INVALID'
        else:
            measured = ros_ns - stamp_ns
            previous = self._watermarks[role]
            if measured < -(0 if role == 'odom' else self.future_ns):
                reason = 'SOURCE_IN_FUTURE'
            elif max(measured / 1e9, age_s) > self.timeout_s:
                reason = 'SOURCE_STALE'
            elif previous is not None and stamp_ns <= previous:
                reason = 'STAMP_NON_INCREASING'
            else:
                age = max(0.0, age_s, measured / 1e9)
                conservative_stamp = min(stamp_ns, ros_ns - math.ceil(age * 1e9))
                self.samples[role] = StopSample(
                    value, conservative_stamp, now, now - age, age, source_id)
                self._watermarks[role] = stamp_ns
        self.reasons[role] = reason

    def observe_motion(self, role, values, *, stamp_ns, source_id, ros_ns, now):
        if role not in ('odom', 'cmd'):
            raise ValueError('unsupported motion input')
        old = self.samples[role]
        self.samples[role] = None
        if old is not None and _time(now) and now - old.effective_at > self.timeout_s:
            self._hold_since = self._hold_last = None
        if not _axes(values):
            self.reasons[role] = 'AXES_INVALID'
        else:
            self._admit(role, tuple(values), stamp_ns, source_id, ros_ns, now)
        self._update_hold(ros_ns, now, may_start=True)

    def observe_status(self, statuses, *, stamp_ns, source_id, ros_ns, now, role='nav_status'):
        if role not in ('nav_status', 'nav_through_status') or role not in self.samples:
            raise ValueError('status role not selected')
        self.samples[role] = None
        # Bound normalization; duplicates/invalid records cannot prove inactivity.
        if (type(statuses) not in (list, tuple) or len(statuses) > 256 or
                any(type(pair) not in (list, tuple) or len(pair) != 2 or
                    not _uuid(pair[0]) or type(pair[1]) is not int or not 0 <= pair[1] <= 6
                    for pair in statuses) or
                len({pair[0] for pair in statuses}) != len(statuses)):
            self.reasons[role] = 'STATUS_INVALID'
            self._clock(ros_ns, now)
            return
        self._admit(role, tuple(tuple(pair) for pair in statuses), stamp_ns,
                    source_id, ros_ns, now)

    def observe_actuator(self, payload, *, source_id, ros_ns, now):
        self.samples['actuator'] = None
        if not self._clock(ros_ns, now):
            return
        if not self._pin('actuator', source_id):
            self.reasons['actuator'] = self.fault or 'SOURCE_INVALID'
            return
        age_ms = self.actuator_guard.admit(payload, ros_ns)
        if age_ms is None:
            self.reasons['actuator'] = self.actuator_guard.reason
            return
        identity = self.actuator_guard.last_identity
        if ((self.request is not None or self.require_recovery_on_change) and
                self._actuator_identity not in (None, identity)):
            self.fault = self.fault or 'GLOBAL_ACTUATOR_CONTEXT_CHANGED'
            return
        self._actuator_identity = identity
        value = (payload['pump_pwm_commanded_on'], payload['servo_pwm_commanded_on'],
                 payload['servo_phase'])
        self._admit('actuator', value, self.actuator_guard.last_stamp_ns,
                    source_id, ros_ns, now, age_s=age_ms / 1000.0)

    def observe_graph(self, *, raw_count, pump_count, sources, started_ros_ns,
                      started_at, ros_ns, now, omitted_present=None):
        self.graph = None
        if not self._clock(ros_ns, now):
            return
        if (type(raw_count) is not int or raw_count < 0 or type(pump_count) is not int or
                pump_count < 0 or type(sources) is not dict or set(sources) != set(self.source_roles) or
                type(started_ros_ns) is not int or started_ros_ns <= 0 or
                not _time(started_at) or not 0 <= now - started_at <= self.timeout_s or
                not 0 <= ros_ns - started_ros_ns <= round(self.timeout_s * 1e9)):
            self.graph_reason = 'GLOBAL_GRAPH_INVALID_OR_STALE'
        elif self.manifest.omitted and omitted_present is not False:
            self.graph_reason = 'GLOBAL_OMITTED_OWNER_ABSENCE_UNPROVEN'
            if omitted_present is True and self.require_recovery_on_change:
                self.fault = self.fault or 'GLOBAL_OMITTED_OWNER_PRESENT'
        elif any(not self._pin(role, sources[role]) for role in self.source_roles):
            self.graph_reason = self.fault or 'GLOBAL_GRAPH_SOURCE_INVALID'
        elif self.request is None or started_at <= self.barrier_at or started_ros_ns <= self.barrier_ros_ns:
            self.graph_reason = 'GLOBAL_GRAPH_NOT_AFTER_CLOSE'
        elif raw_count != 0 or pump_count != 0:
            self.graph_reason = 'GLOBAL_COMMAND_OWNER_PRESENT'
        else:
            self.graph_reason = None
            self.graph = (started_ros_ns, started_at, tuple(sources.items()))
        if self.graph is None:
            self._hold_since = self._hold_last = None

    def _fresh(self, role, ros_ns, now):
        sample = self.samples[role]
        if sample is None:
            return self.reasons[role] or 'NOT_RECEIVED'
        if (not 0 <= now - sample.received_at or now - sample.effective_at > self.timeout_s or
                not -self.future_ns <= ros_ns - sample.stamp_ns <= round(self.timeout_s * 1e9)):
            return 'SOURCE_STALE_OR_FUTURE'
        if (self.request is None or sample.effective_at <= self.barrier_at or
                sample.stamp_ns <= self.barrier_ros_ns + self.future_ns):
            return 'NOT_AFTER_CLOSE'
        return None

    def _update_hold(self, ros_ns, now, *, may_start=False):
        if (not self._clock(ros_ns, now) or self.request is None or
                any(self._fresh(role, ros_ns, now) for role in ('odom', 'cmd'))):
            self._hold_since = self._hold_last = None
            return
        odom, cmd = self.samples['odom'].value, self.samples['cmd'].value
        stationary = all(abs(v) <= self.linear_limit for v in odom[:3]) and all(
            abs(v) <= self.angular_limit for v in odom[3:])
        if not stationary or any(v != 0 for v in cmd):
            self._hold_since = self._hold_last = None
        elif may_start:
            if (self._hold_since is None or self._hold_last is None or
                    now - self._hold_last > self.timeout_s):
                self._hold_since = now
            self._hold_last = now

    def assess(self, owners, *, ros_ns, now):
        self._update_hold(ros_ns, now)
        blockers = []
        request = self.request
        if self.fault is not None:
            return GlobalStopReadiness(False, (self.fault,), None)
        if request is None:
            blockers.append('GLOBAL_CLOSE_ROUND_NOT_STARTED')
        else:
            if (not isinstance(owners, CloseOwnersReadiness) or owners.ready is not True or
                    owners.manifest_id != self.manifest.name or
                    owners.blockers != () or type(owners.close_generation) is not int or
                    (owners.session_id, owners.nav_goal_uuid, owners.close_generation) !=
                    (request.session_id, request.nav_goal_uuid, request.generation) or
                    not _time(owners.observed_at) or not self.barrier_at < owners.observed_at <= now or
                    now - owners.observed_at > self.timeout_s):
                blockers.append('GLOBAL_OWNER_ACKS_NOT_CURRENT')
            if self.graph is None:
                blockers.append(self.graph_reason or 'GLOBAL_GRAPH_MISSING')
            elif (not 0 <= now - self.graph[1] <= self.timeout_s or
                  not 0 <= ros_ns - self.graph[0] <= round(self.timeout_s * 1e9)):
                blockers.append('GLOBAL_GRAPH_STALE')
            for role in self.samples:
                reason = self._fresh(role, ros_ns, now)
                if reason is not None:
                    blockers.append(role + ':' + reason)
            if (self._hold_since is None or not _time(now) or
                    now - self._hold_since < self.zero_hold_s):
                blockers.append('GLOBAL_STATIONARY_ZERO_HOLD_PENDING')
            act = self.samples['actuator']
            if act is not None and act.value != (False, False, 'IDLE'):
                blockers.append('GLOBAL_ACTUATOR_NOT_OFF_IDLE')
            nav = self.samples['nav_status']
            status = None
            if nav is not None:
                if any(state in (0, 1, 2, 3) for _, state in nav.value):
                    blockers.append('GLOBAL_NAVIGATION_NOT_INACTIVE')
                if request.nav_goal_uuid is not None:
                    status = next((state for goal, state in nav.value if goal == request.nav_goal_uuid), None)
                    if status not in (4, 5, 6):
                        blockers.append('GLOBAL_EXACT_NAV_TERMINAL_MISSING')
            through = self.samples.get('nav_through_status')
            if through is not None and any(state in (0, 1, 2, 3) for _, state in through.value):
                blockers.append('GLOBAL_THROUGH_NAVIGATION_NOT_INACTIVE')
            if self.graph is not None:
                sources = dict(self.graph[2])
                if any(sample is not None and sources[role] != sample.source_id
                       for role, sample in self.samples.items()):
                    blockers.append('GLOBAL_GRAPH_SAMPLE_SOURCE_MISMATCH')
        if blockers:
            return GlobalStopReadiness(False, tuple(blockers), None)
        observed = min(owners.observed_at, self.graph[1],
                       *(sample.effective_at for sample in self.samples.values()))
        evidence = SessionCleanupEvidence(
            request.session_id, request.nav_goal_uuid, observed,
            nav_inactive=True, motion_owner_released=True, pump_owner_released=True,
            pump_off=True, servo_idle=True, stationary=True, cmd_zero=True,
            nav_terminal_status=status, close_generation=request.generation)
        return GlobalStopReadiness(True, (), evidence)

    def through_navigation_blocker(self, *, ros_ns, now):
        """Admission check separate from post-close barriers; never retime cache."""
        if not self.require_through_status:
            return None
        if not self._clock(ros_ns, now):
            return self.fault
        sample = self.samples['nav_through_status']
        if (sample is None or now < sample.received_at or
                now - sample.effective_at > self.timeout_s or
                not -self.future_ns <= ros_ns - sample.stamp_ns <= round(self.timeout_s * 1e9)):
            return 'THROUGH_NAV_STATUS_NOT_CURRENT'
        if any(status in (0, 1, 2, 3) for _, status in sample.value):
            return 'THROUGH_NAVIGATION_ACTIVE_OR_UNKNOWN'
        return None

    def retire_after_close(self, core, evidence, now):
        if self.fault is not None or not core.can_retire_collectors(self.request, evidence, now):
            raise ValueError('global round retirement is not verified')
        self.request = self.barrier_ros_ns = self.barrier_at = None
        self.graph = None
        self.graph_reason = 'GLOBAL_GRAPH_NOT_RECEIVED'
        self.samples = dict.fromkeys(self.telemetry_roles)
        self.reasons = dict.fromkeys(self.telemetry_roles, 'NOT_RECEIVED')
        self._hold_since = self._hold_last = None
        # Keep GID, ACT epoch, sequence/stamp and clock history across sessions.
