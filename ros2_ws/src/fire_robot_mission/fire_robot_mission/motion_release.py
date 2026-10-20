"""One motion-owner release event; callers retain all motion/stop guards."""

from copy import deepcopy
from dataclasses import asdict
import math
from uuid import uuid4

from fire_robot_mission.session_context import (
    NavigationBinding, SessionBinding, _session_id_matches,
)


def _time(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


class MotionReleaseProof:
    """Bounded identity and zero-publish markers, never command authority."""

    def __init__(self, *, timeout_s=0.35, zero_hold_s=0.5,
                 owner='live_alignment_takeover'):
        if (not _time(timeout_s) or not 0 < timeout_s <= 0.35 or
                not _time(zero_hold_s) or zero_hold_s < 0.5):
            raise ValueError('motion release must retain freshness and zero hold limits')
        if owner not in ('live_alignment_takeover', 'ir_approach_live', 'live_nav_stop_gateway'):
            raise ValueError('unsupported motion owner')
        self.owner = owner
        self.timeout_s, self.zero_hold_s = timeout_s, zero_hold_s
        self.binding = None
        self.fire_binding = None
        self.fire_associated_at = None
        self.owner_token = None
        self.created_at = None
        self.close_generation = None
        self.close_source = None
        self.close_received_at = None
        self.zero_published_at = None
        self.receipt = None
        self.stop_latched = False

    def created(self, binding, now):
        binding_type = (NavigationBinding if self.owner == 'live_nav_stop_gateway'
                        else SessionBinding)
        if not isinstance(binding, binding_type) or not _time(now) or self.owner_token is not None:
            raise ValueError('motion owner needs a fresh admitted binding and a new lease')
        self.binding = binding
        self.owner_token = uuid4().hex
        self.created_at = now

    def associate_fire(self, binding, now):
        """Freeze a caller-admitted fire/close context on an existing navigation lease.

        This correlation requests stop; it grants no command authority. The
        original navigation binding/token is never replaced or re-created.
        """
        if (not isinstance(self.binding, NavigationBinding) or
                not isinstance(binding, SessionBinding) or self.owner_token is None or
                self.receipt is not None or not _time(now) or now < self.created_at or
                type(binding.source_sequence) is not int or
                type(binding.source_stamp_ns) is not int or
                not _session_id_matches(binding.session_id, binding.source_instance_id) or
                binding.nav_goal_uuid != self.binding.nav_goal_uuid or
                binding.source_instance_id != self.binding.source_instance_id or
                binding.source_sequence <= self.binding.source_sequence or
                binding.source_stamp_ns <= self.binding.source_stamp_ns):
            return False
        if self.fire_binding is not None:
            return (binding.session_id == self.fire_binding.session_id and
                    binding.nav_goal_uuid == self.fire_binding.nav_goal_uuid and
                    binding.source_instance_id == self.fire_binding.source_instance_id)
        self.fire_binding = binding
        self.fire_associated_at = now
        self.zero_published_at = None
        self.stop_latched = True
        return True

    def _session_binding(self):
        return self.binding if isinstance(self.binding, SessionBinding) else self.fire_binding

    def accept_close(self, observer, now):
        """Only a caller-admitted fresh STOPPING snapshot may enter here."""
        source = observer.get('source') if isinstance(observer, dict) else None
        generation = observer.get('close_generation') if isinstance(observer, dict) else None
        binding = self._session_binding()
        if (binding is None or self.receipt is not None or not _time(now) or
                now < self.created_at or not isinstance(source, dict) or
                (self.fire_associated_at is not None and now < self.fire_associated_at) or
                observer.get('phase') != 'STOPPING' or
                observer.get('session_id') != binding.session_id or
                observer.get('nav_goal_uuid') != binding.nav_goal_uuid or
                source.get('instance_id') != binding.source_instance_id or
                type(generation) is not int or generation <= 0):
            return False
        if self.close_generation is not None:
            return generation == self.close_generation
        self.close_generation = generation
        self.close_source = deepcopy(source)
        self.close_received_at = now
        self.zero_published_at = None
        self.stop_latched = True
        return True

    def command_published(self, angular, now):
        """Record the sole varying Twist axis after publish; all other axes must be zero.

        The argument is angular.z for alignment or linear.x for IR approach.
        """
        if self.owner == 'live_nav_stop_gateway':
            self.zero_published_at = None
            raise RuntimeError('NAVIGATION_REQUIRES_FULL_TWIST_EVIDENCE')
        if type(angular) not in (int, float) or not math.isfinite(angular):
            raise RuntimeError('MOTION_PUBLISH_EVIDENCE_INVALID')
        self._command_state_published(angular == 0.0, now)

    def twist_published(self, values, now):
        """Record all six axes of an actual Twist after publish returns."""
        if (not isinstance(values, (tuple, list)) or len(values) != 6 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
            self.zero_published_at = None
            raise RuntimeError('MOTION_PUBLISH_EVIDENCE_INVALID')
        try:
            self._command_state_published(all(v == 0.0 for v in values), now)
        except RuntimeError:
            self.zero_published_at = None
            raise

    def _command_state_published(self, zero, now):
        if (self.binding is None or self.receipt is not None or not _time(now) or
                now < self.created_at or
                (self.fire_associated_at is not None and now < self.fire_associated_at) or
                (self.close_received_at is not None and now < self.close_received_at)):
            raise RuntimeError('MOTION_PUBLISH_EVIDENCE_INVALID')
        if not zero:
            self.zero_published_at = None
        elif self.zero_published_at is None:
            self.zero_published_at = now

    def can_release(self, now, *, odom_at, cmd_at):
        zero = self.zero_published_at
        return (self.binding is not None and self.receipt is None and _time(now) and
                zero is not None and 0 <= now - zero and now - zero >= self.zero_hold_s and
                (self.fire_associated_at is None or zero >= self.fire_associated_at) and
                (self.close_received_at is None or zero >= self.close_received_at) and
                all(_time(at) and zero < at <= now and now - at <= self.timeout_s
                    for at in (odom_at, cmd_at)))

    def released(self, *, checked_at, released_at, odom_at, cmd_at, stationary_since):
        """Caller must confirm destroy_publisher(...) is True before calling."""
        if (self.binding is None or self.created_at is None or
                not _time(released_at) or not _time(checked_at) or released_at < checked_at or
                not _time(stationary_since) or stationary_since < self.created_at or
                checked_at - stationary_since < self.zero_hold_s or
                (self.fire_associated_at is not None and stationary_since < self.fire_associated_at) or
                (self.close_received_at is not None and stationary_since < self.close_received_at) or
                not self.can_release(checked_at, odom_at=odom_at, cmd_at=cmd_at)):
            raise RuntimeError('MOTION_RELEASE_EVIDENCE_INVALID')
        session = self._session_binding()
        self.receipt = {
            'owner': self.owner, 'owner_token': self.owner_token,
            'session_id': None if session is None else session.session_id,
            'nav_goal_uuid': self.binding.nav_goal_uuid,
            'session_binding': None if session is None else asdict(session),
            'close_generation': self.close_generation,
            'close_request_source': deepcopy(self.close_source),
            'close_received_mono': self.close_received_at,
            'zero_published_mono': self.zero_published_at,
            'stationary_hold_started_mono': stationary_since,
            'checked_at_mono': checked_at, 'released_at_mono': released_at,
            'odom_evidence_mono': odom_at, 'cmd_received_mono': cmd_at,
            'physical_motion_verified': False,
        }
        if isinstance(self.binding, NavigationBinding):
            self.receipt['navigation_binding'] = asdict(self.binding)
            self.receipt['fire_associated_mono'] = self.fire_associated_at

    def snapshot(self, *, owned, blocker):
        session = self._session_binding()
        payload = {
            'mode': 'SESSION_OWNER_MIGRATION', 'publisher_owned': owned,
            'session_binding': None if session is None else asdict(session),
            'stop_latched': self.stop_latched, 'blocker': blocker,
            'receipt': deepcopy(self.receipt),
        }
        if self.owner == 'live_nav_stop_gateway':
            payload['navigation_binding'] = (
                asdict(self.binding) if isinstance(self.binding, NavigationBinding) else None)
            payload['fire_associated_mono'] = self.fire_associated_at
        return payload
