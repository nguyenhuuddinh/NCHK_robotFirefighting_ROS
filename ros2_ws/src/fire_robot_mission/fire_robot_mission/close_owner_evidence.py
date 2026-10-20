"""Bounded close-owner observations; no ROS, release API or cleanup authority.

A current NO_OWNER_AT_CLOSE observation is distinct from a historical release
receipt. Callers still have to verify global graph ownership, fresh stationary
odom/cmd, actuator OFF/idle and exact Nav2 terminal state before session cleanup.
"""

from copy import deepcopy
from dataclasses import dataclass
import math
from uuid import uuid4

from fire_robot_mission.fire_session_core import CLOSE_REASONS, SessionCloseRequest
from fire_robot_mission.session_context import (
    SessionContextGuard, _session_id_matches, _uuid,
)
from fire_robot_mission.runtime_manifest import runtime_manifest


CLOSE_OWNERS = (
    'live_nav_stop_gateway', 'live_alignment_takeover',
    'ir_approach_live', 'pump_demo_controller',
)


def _time(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


def _source(value):
    """Return immutable instance/sequence/ROS-stamp fields, or None."""
    stamp = value.get('stamp') if isinstance(value, dict) else None
    if not isinstance(stamp, dict):
        return None
    instance, sequence = value.get('instance_id'), value.get('sequence')
    sec, ns = stamp.get('sec'), stamp.get('nanosec')
    if (not _uuid(instance) or type(sequence) is not int or sequence <= 0 or
            type(sec) is not int or sec < 0 or type(ns) is not int or
            not 0 <= ns < 1_000_000_000 or sec * 1_000_000_000 + ns <= 0):
        return None
    return instance, sequence, sec * 1_000_000_000 + ns


@dataclass(frozen=True)
class CloseOwnerAck:
    session_id: str
    nav_goal_uuid: object
    generation: int
    reason: str
    supervisor_instance: str
    supervisor_sequence: int
    supervisor_stamp_ns: int


def _ack(value):
    if not isinstance(value, dict) or value.get('kind') != 'NO_OWNER_AT_CLOSE':
        return None
    source = _source(value.get('supervisor_source'))
    goal, generation = value.get('nav_goal_uuid'), value.get('close_generation')
    reason, session = value.get('close_reason'), value.get('session_id')
    if (source is None or not _session_id_matches(session, source[0]) or
            (goal is not None and not _uuid(goal)) or
            type(generation) is not int or generation <= 0 or
            type(reason) is not str or reason not in CLOSE_REASONS):
        return None
    return CloseOwnerAck(session, goal, generation, reason, *source)


@dataclass(frozen=True)
class OwnerObservation:
    instance_id: str
    sequence: int
    stamp_ns: int
    publisher_owned: bool
    close_ack: object
    received_at: float
    source_age_s: float
    supervisor_age_s: object


@dataclass(frozen=True)
class CloseOwnersReadiness:
    """Only four owner observations, NOT SessionCleanupEvidence or permission."""
    session_id: object
    nav_goal_uuid: object
    close_generation: object
    ready: bool
    observed_at: object
    blockers: tuple
    manifest_id: str = 'all_four_external_v1'


class OwnerCloseReporter:
    """One process instance/sequence; caller invokes only in selected migration mode."""

    def __init__(self, owner):
        if owner not in CLOSE_OWNERS:
            raise ValueError('unknown close owner')
        self.owner = owner
        self.instance_id = uuid4().hex
        self.sequence = 0
        self.last_stamp_ns = None

    def report(self, context, *, publisher_owned, stamp_ns, now):
        """Call AFTER effects with actual ownership and a stamp taken BEFORE work.

        No publisher references are changed here. A producer may only pass
        False after actual destruction is confirmed, or when no publisher was
        acquired. The already-admitted context guard is reused, never fabricated.
        """
        if (type(publisher_owned) is not bool or
                (context is not None and not isinstance(context, SessionContextGuard))):
            raise ValueError('close observation requires actual ownership and admitted context')
        result = {
            'schema_version': 1, 'mode': 'CLOSE_OWNER_OBSERVATION',
            'owner': self.owner, 'actuation_authorized': False,
            'publisher_owned': publisher_owned, 'source': None,
            'source_blocker': None, 'close_ack': None,
        }
        if type(stamp_ns) is not int or stamp_ns <= 0 or not _time(now):
            result['source_blocker'] = 'OWNER_REPORT_CLOCK_INVALID'
            return result
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            result['source_blocker'] = 'OWNER_REPORT_STAMP_NON_INCREASING'
            return result
        self.last_stamp_ns = stamp_ns
        self.sequence += 1
        sec, ns = divmod(stamp_ns, 1_000_000_000)
        result['source'] = {
            'instance_id': self.instance_id, 'sequence': self.sequence,
            'stamp': {'sec': sec, 'nanosec': ns},
        }
        observer = None if context is None else context.current(now)
        if not publisher_owned and observer is not None and observer['phase'] == 'STOPPING':
            result['close_ack'] = {
                'kind': 'NO_OWNER_AT_CLOSE', 'session_id': observer['session_id'],
                'nav_goal_uuid': observer['nav_goal_uuid'],
                'close_generation': observer['close_generation'],
                'close_reason': observer['close_reason'],
                'supervisor_source': deepcopy(observer['source']),
            }
        return result


class CloseOwnerEvidenceCollector:
    """Four latest samples/watermarks and one immutable pending close round.

    Exact endpoint checks belong to the caller. No-owner acknowledgments are
    not enough to call finish_close: global stop/OFF/graph proofs remain separate.
    Process/clock change during a round is latched; a fresh heartbeat cannot
    silently rebaseline an interrupted round or reset any encounter budget.
    """

    def __init__(self, *, timeout_s=0.35, future_skew_ms=20.0, manifest_id='all_four_external_v1'):
        if (type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or
                not 0 < timeout_s <= 0.35 or
                type(future_skew_ms) not in (int, float) or not math.isfinite(future_skew_ms) or
                not 0 <= future_skew_ms <= 20.0):
            raise ValueError('owner evidence must retain freshness and clock skew limits')
        self.timeout_s = timeout_s
        self.manifest = runtime_manifest(manifest_id)
        self.future_ns = round(future_skew_ms * 1e6)
        self.request = None
        self.announced_ros_ns = self.announced_at = None
        self._samples = dict.fromkeys(CLOSE_OWNERS)
        self._watermarks = dict.fromkeys(CLOSE_OWNERS)
        self._reasons = dict.fromkeys(CLOSE_OWNERS, 'OWNER_REPORT_NOT_RECEIVED')
        self._pins = dict.fromkeys(CLOSE_OWNERS)
        self._last_ros_ns = self._last_mono = None
        self.fault = None

    def _clock(self, ros_ns, now):
        if (type(ros_ns) is not int or ros_ns <= 0 or not _time(now)):
            self.fault = self.fault or 'OWNER_COLLECTOR_CLOCK_INVALID'
        elif ((self._last_ros_ns is not None and ros_ns < self._last_ros_ns) or
              (self._last_mono is not None and now < self._last_mono)):
            self.fault = self.fault or 'OWNER_COLLECTOR_CLOCK_BACKWARD'
        else:
            self._last_ros_ns, self._last_mono = ros_ns, now
        if self.fault is not None:
            self._samples = dict.fromkeys(CLOSE_OWNERS)
            return False
        return True

    def begin_close(self, request, *, announced_ros_ns, announced_at):
        """First local announcement is the barrier; later heartbeats do not move it."""
        if self.request is not None:
            if request == self.request:
                return self.fault is None
            self.fault = self.fault or 'OWNER_CLOSE_ROUND_CHANGED'
            return False
        session = request.session_id if isinstance(request, SessionCloseRequest) else None
        parts = session.split(':') if type(session) is str else []
        namespace = parts[1] if len(parts) == 3 else None
        if (not isinstance(request, SessionCloseRequest) or not _uuid(namespace) or
                not _session_id_matches(session, namespace) or
                (request.nav_goal_uuid is not None and not _uuid(request.nav_goal_uuid)) or
                type(request.generation) is not int or request.generation <= 0 or
                type(request.reason) is not str or request.reason not in CLOSE_REASONS or
                not _time(request.requested_at) or not _time(announced_at) or
                announced_at < request.requested_at):
            return False
        if not self._clock(announced_ros_ns, announced_at):
            return False
        self.request = request
        self.announced_ros_ns, self.announced_at = announced_ros_ns, announced_at
        self._pins = {owner: None if watermark is None else watermark[0]
                      for owner, watermark in self._watermarks.items()}
        return True

    def observe(self, owner, payload, *, ros_now_ns, received_at, source_ok):
        """Admit one exact-source report without renewing its source-age budget."""
        if owner not in CLOSE_OWNERS:
            raise ValueError('unknown close owner')
        if owner not in self.manifest.required:
            self.fault = self.fault or 'OMITTED_OWNER_REPORTED:' + owner
            return False
        self._samples[owner] = None
        if not self._clock(ros_now_ns, received_at):
            return False
        reason = None
        if source_ok is not True:
            reason = 'OWNER_PUBLISHER_INVALID'
        elif (not isinstance(payload, dict) or type(payload.get('schema_version')) is not int or
              payload['schema_version'] != 1 or payload.get('mode') != 'CLOSE_OWNER_OBSERVATION' or
              payload.get('owner') != owner or payload.get('actuation_authorized') is not False or
              type(payload.get('publisher_owned')) is not bool or payload.get('source_blocker') is not None):
            reason = 'OWNER_REPORT_INVALID'
        source = _source(payload.get('source')) if isinstance(payload, dict) else None
        ack = _ack(payload.get('close_ack')) if isinstance(payload, dict) else None
        if reason is None and source is None:
            reason = 'OWNER_SOURCE_INVALID'
        if reason is None and payload.get('close_ack') is not None and (
                ack is None or payload['publisher_owned']):
            reason = 'OWNER_CLOSE_ACK_INVALID'
        if reason is None and ack is not None and not (
                -self.future_ns <= ros_now_ns - ack.supervisor_stamp_ns <= round(self.timeout_s * 1e9)):
            reason = 'OWNER_SUPERVISOR_SOURCE_STALE_OR_FUTURE'
        if reason is None:
            instance, sequence, stamp_ns = source
            age_ns = ros_now_ns - stamp_ns
            previous = self._watermarks[owner]
            if age_ns < -self.future_ns:
                reason = 'OWNER_SOURCE_IN_FUTURE'
            elif age_ns > round(self.timeout_s * 1e9):
                reason = 'OWNER_SOURCE_STALE'
            elif previous is not None and stamp_ns <= previous[2]:
                reason = 'OWNER_STAMP_NON_INCREASING'
            elif previous is not None and instance == previous[0] and sequence <= previous[1]:
                reason = 'OWNER_SEQUENCE_NON_INCREASING'
            elif self.request is not None and self._pins[owner] not in (None, instance):
                self.fault = 'OWNER_PROCESS_CHANGED:' + owner
                reason = self.fault
            else:
                self._watermarks[owner] = source
                if self.request is not None:
                    self._pins[owner] = instance
                self._samples[owner] = OwnerObservation(
                    instance, sequence, stamp_ns, payload['publisher_owned'], ack,
                    received_at, max(0, age_ns) / 1e9,
                    None if ack is None else max(0, ros_now_ns - ack.supervisor_stamp_ns) / 1e9)
        self._reasons[owner] = reason
        return reason is None

    def assess(self, *, ros_now_ns, now, absence_observed_at=None):
        """Return owner-only readiness; never synthesize global cleanup booleans."""
        clock_ok = self._clock(ros_now_ns, now)
        request = self.request
        blockers, observed = [], []
        if not clock_ok:
            blockers.append(self.fault)
        elif request is None:
            blockers.append('OWNER_CLOSE_ROUND_NOT_STARTED')
        else:
            namespace = request.session_id.split(':')[1]
            for owner in self.manifest.required:
                sample = self._samples[owner]
                reason = self._reasons[owner]
                if sample is not None:
                    age = sample.source_age_s + now - sample.received_at
                    ack = sample.close_ack
                    if (not 0 <= now - sample.received_at or age > self.timeout_s or
                            ros_now_ns - sample.stamp_ns > round(self.timeout_s * 1e9)):
                        reason = 'OWNER_SOURCE_STALE'
                    elif (sample.stamp_ns <= self.announced_ros_ns + self.future_ns or
                          sample.received_at - sample.source_age_s <= self.announced_at):
                        reason = 'OWNER_OBSERVATION_NOT_AFTER_CLOSE'
                    elif sample.publisher_owned:
                        reason = 'OWNER_STILL_OWNED'
                    elif ack is None:
                        reason = 'OWNER_CLOSE_ACK_MISSING'
                    elif ((ack.session_id, ack.nav_goal_uuid, ack.generation, ack.reason,
                           ack.supervisor_instance) !=
                          (request.session_id, request.nav_goal_uuid, request.generation,
                           request.reason, namespace)):
                        reason = 'OWNER_CLOSE_CONTEXT_MISMATCH'
                    elif (sample.supervisor_age_s + now - sample.received_at > self.timeout_s or
                          not -self.future_ns <= ros_now_ns - ack.supervisor_stamp_ns <= round(self.timeout_s * 1e9)):
                        reason = 'OWNER_SUPERVISOR_SOURCE_STALE_OR_FUTURE'
                    else:
                        observed.append(sample.received_at - sample.source_age_s)
                if reason is not None:
                    blockers.append(owner + ':' + reason)
        if request is not None and not self.manifest.required and not blockers:
            if (not _time(absence_observed_at) or not self.announced_at < absence_observed_at <= now or
                    now - absence_observed_at > self.timeout_s):
                blockers.append('MANIFEST_ABSENCE_GRAPH_NOT_CURRENT')
            else:
                observed.append(absence_observed_at)
        ready = not blockers and request is not None and len(observed) == (len(self.manifest.required) or 1)
        return CloseOwnersReadiness(
            None if request is None else request.session_id,
            None if request is None else request.nav_goal_uuid,
            None if request is None else request.generation,
            ready, min(observed) if ready else None, tuple(blockers), self.manifest.name)

    def retire_after_close(self, core, evidence, now):
        if self.fault is not None or not core.can_retire_collectors(self.request, evidence, now):
            raise ValueError('owner round retirement is not verified')
        self.request = self.announced_ros_ns = self.announced_at = None
        self._samples = dict.fromkeys(CLOSE_OWNERS)
        self._reasons = dict.fromkeys(CLOSE_OWNERS, 'OWNER_REPORT_NOT_RECEIVED')
        # Preserve source/sequence/clock watermarks; no silent replay rebaseline.
