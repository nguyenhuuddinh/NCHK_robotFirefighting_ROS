"""Bounded admission of supervisor session snapshots; no ROS or authority."""

from copy import deepcopy
from dataclasses import dataclass
import math

from fire_robot_mission.fire_session_core import CLOSE_REASONS


def _uuid(value):
    return (type(value) is str and len(value) == 32 and value != '0' * 32 and
            all(c in '0123456789abcdef' for c in value))


def _session_id_matches(value, instance):
    parts = value.split(':') if type(value) is str else []
    return (len(parts) == 3 and parts[0] == 'fire' and parts[1] == instance and
            parts[2].isascii() and parts[2].isdigit() and parts[2][0] != '0')


@dataclass(frozen=True)
class SessionBinding:
    session_id: str
    nav_goal_uuid: str
    source_instance_id: str
    source_sequence: int
    source_stamp_ns: int


@dataclass(frozen=True)
class NavigationBinding:
    """Real Nav2 goal/source before fire; deliberately has no fire-session ID."""

    nav_goal_uuid: str
    source_instance_id: str
    source_sequence: int
    source_stamp_ns: int

    def __post_init__(self):
        if (not _uuid(self.nav_goal_uuid) or not _uuid(self.source_instance_id) or
                type(self.source_sequence) is not int or self.source_sequence <= 0 or
                type(self.source_stamp_ns) is not int or self.source_stamp_ns <= 0):
            raise ValueError('navigation binding needs a real goal and admitted source identity')


class SessionContextGuard:
    """One source watermark and one snapshot; endpoint checks belong to caller."""

    def __init__(self, *, timeout_s=0.35):
        if (type(timeout_s) not in (int, float) or not math.isfinite(timeout_s)
                or not 0 < timeout_s <= 0.35):
            raise ValueError('session timeout must be positive and <=0.35s')
        self.timeout_s = timeout_s
        self.sample = None
        self.last_ros_ns = None
        self.last_stamp_ns = None
        self.last_instance = None
        self.last_sequence = None
        self.reason = 'SESSION_CONTEXT_NOT_RECEIVED'

    def update(self, payload, ros_now_ns, received_at, *, source_ok):
        self.sample = None
        if (type(received_at) not in (int, float) or
                not math.isfinite(received_at) or received_at < 0 or
                type(ros_now_ns) is not int or ros_now_ns <= 0):
            self.reason = 'SESSION_CLOCK_INVALID'
            return
        previous_ros = self.last_ros_ns
        self.last_ros_ns = ros_now_ns
        if previous_ros is not None and ros_now_ns < previous_ros:
            self.reason = 'SESSION_CLOCK_BACKWARD'
            return
        if source_ok is not True:
            self.reason = 'SESSION_PUBLISHER_INVALID'
            return
        lifecycle = payload.get('lifecycle') if isinstance(payload, dict) else None
        observer = lifecycle.get('fire_session_observer') if isinstance(lifecycle, dict) else None
        if (not isinstance(payload, dict) or
                type(payload.get('schema_version')) is not int or payload['schema_version'] != 1 or
                payload.get('observe_only') is not True or not isinstance(observer, dict) or
                observer.get('mode') != 'OBSERVE_ONLY' or
                observer.get('actuation_authorized') is not False or
                observer.get('source_blocker') is not None):
            self.reason = 'SESSION_REPORT_INVALID'
            return
        source = observer.get('source')
        stamp = source.get('stamp') if isinstance(source, dict) else None
        if not isinstance(stamp, dict):
            self.reason = 'SESSION_SOURCE_INVALID'
            return
        sec, ns = stamp.get('sec'), stamp.get('nanosec')
        instance, seq = source.get('instance_id'), source.get('sequence')
        if (type(sec) is not int or sec < 0 or type(ns) is not int or
                not 0 <= ns < 1_000_000_000 or not _uuid(instance) or
                type(seq) is not int or seq <= 0):
            self.reason = 'SESSION_SOURCE_INVALID'
            return
        stamp_ns = sec * 1_000_000_000 + ns
        session_id, goal = observer.get('session_id'), observer.get('nav_goal_uuid')
        phase, generation = observer.get('phase'), observer.get('close_generation')
        reason = observer.get('close_reason')
        valid_session = _session_id_matches(session_id, instance)
        if ((goal is not None and not _uuid(goal)) or
                phase not in ('IDLE', 'ACTIVE', 'STOPPING') or
                (phase == 'IDLE' and (session_id is not None or goal is not None)) or
                (phase != 'IDLE' and not valid_session) or
                (phase != 'STOPPING' and (generation is not None or reason is not None)) or
                (phase == 'STOPPING' and (type(generation) is not int or generation <= 0 or
                                         type(reason) is not str or reason not in CLOSE_REASONS))):
            self.reason = 'SESSION_CONTEXT_INVALID'
            return
        age_ns = ros_now_ns - stamp_ns
        if stamp_ns <= 0:
            self.reason = 'SESSION_SOURCE_INVALID'
        elif age_ns < -20_000_000:
            self.reason = 'SESSION_SOURCE_IN_FUTURE'
        elif age_ns > round(self.timeout_s * 1e9):
            self.reason = 'SESSION_SOURCE_STALE'
        elif self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            self.reason = 'SESSION_STAMP_NON_INCREASING'
        elif instance == self.last_instance and seq <= self.last_sequence:
            self.reason = 'SESSION_SEQUENCE_NON_INCREASING'
        else:
            self.last_stamp_ns, self.last_instance, self.last_sequence = stamp_ns, instance, seq
            self.sample = (deepcopy(observer), received_at, max(age_ns, 0) / 1e9)
            self.reason = None

    def current(self, now):
        if self.sample is None:
            return None
        observer, received, source_age = self.sample
        if (type(now) not in (int, float) or not math.isfinite(now) or
                not 0 <= now - received or source_age + now - received > self.timeout_s):
            self.sample = None
            self.reason = 'SESSION_SOURCE_STALE'
            return None
        return observer

    def bind(self, goal_uuid, now):
        observer = self.current(now)
        if observer is None:
            return None
        if (observer['phase'] != 'ACTIVE' or not _uuid(goal_uuid) or
                observer['nav_goal_uuid'] != goal_uuid):
            self.reason = 'SESSION_NOT_ACTIVE_FOR_GOAL'
            return None
        source = observer['source']
        stamp = source['stamp']
        self.reason = None
        return SessionBinding(observer['session_id'], goal_uuid, source['instance_id'],
                              source['sequence'], stamp['sec'] * 1_000_000_000 + stamp['nanosec'])

    def bind_navigation(self, goal_uuid, now):
        """Identity only: caller must prove actual Nav2 goal, arm and ownership."""
        observer = self.current(now)
        if observer is None:
            return None
        if observer['phase'] != 'IDLE' or not _uuid(goal_uuid):
            self.reason = 'NAVIGATION_CONTEXT_NOT_IDLE_OR_GOAL_INVALID'
            return None
        source = observer['source']
        stamp = source['stamp']
        self.reason = None
        return NavigationBinding(goal_uuid, source['instance_id'], source['sequence'],
                                 stamp['sec'] * 1_000_000_000 + stamp['nanosec'])

    def bind_fire_for_navigation(self, navigation, now):
        """Correlate fresh fire/close evidence; never grants navigation or motion."""
        observer = self.current(now)
        if observer is None:
            return None
        if (not isinstance(navigation, NavigationBinding) or
                observer['phase'] not in ('ACTIVE', 'STOPPING') or
                observer['nav_goal_uuid'] != navigation.nav_goal_uuid or
                observer['source']['instance_id'] != navigation.source_instance_id):
            self.reason = 'NAVIGATION_SESSION_MISMATCH'
            return None
        source = observer['source']
        stamp = source['stamp']
        self.reason = None
        return SessionBinding(observer['session_id'], navigation.nav_goal_uuid,
                              source['instance_id'], source['sequence'],
                              stamp['sec'] * 1_000_000_000 + stamp['nanosec'])

    def blocker_for(self, binding, now):
        observer = self.current(now)
        if observer is None:
            return self.reason
        if (not isinstance(binding, SessionBinding) or
                observer['session_id'] != binding.session_id or
                observer['nav_goal_uuid'] != binding.nav_goal_uuid or
                observer['source']['instance_id'] != binding.source_instance_id):
            return 'SESSION_BINDING_CHANGED'
        if observer['phase'] == 'STOPPING':
            return 'SESSION_CLOSE_REQUESTED'
        if observer['phase'] != 'ACTIVE':
            return 'SESSION_CONTEXT_NOT_ACTIVE'
        return None
