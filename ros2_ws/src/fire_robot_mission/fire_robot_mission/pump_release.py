"""Fresh OFF evidence for one pump publisher lease; no ROS or actuation."""

from copy import deepcopy
from dataclasses import asdict
import math
from uuid import uuid4

from fire_robot_mission.actuator_freshness import ActuatorStampGuard
from fire_robot_mission.session_context import SessionBinding


class PumpReleaseProof:
    """Keep one owner token, one ACT sample and one historical release event."""

    def __init__(self, *, timeout_s=0.35, future_skew_ms=20.0):
        if (type(timeout_s) not in (int, float) or not math.isfinite(timeout_s)
                or not 0 < timeout_s <= 0.35):
            raise ValueError('pump release timeout must be positive and <=0.35s')
        self.guard = ActuatorStampGuard(
            max_age_ms=timeout_s * 1000.0, max_future_skew_ms=future_skew_ms)
        self.timeout_s = timeout_s
        self.sample = None
        self.owner_token = None
        self.session_binding = None
        self.close_generation = None
        self.close_request_source = None
        self.goal_uuid = None
        self.pending = False
        self.request_ros_ns = None
        self.request_mono = None
        self.request_identity = None
        self.receipt = None
        self.blocker = 'ACTUATOR_NOT_RECEIVED'

    def owner_created(self, *, session_binding=None):
        if session_binding is not None and not isinstance(session_binding, SessionBinding):
            raise ValueError('pump session binding must be admitted before acquisition')
        self.owner_token = uuid4().hex
        self.session_binding = session_binding
        self.close_generation = None
        self.close_request_source = None
        self.goal_uuid = None if session_binding is None else session_binding.nav_goal_uuid
        self.pending = False
        self.request_ros_ns = None
        self.request_mono = None
        self.request_identity = None
        self.receipt = None
        self.blocker = None

    def accept_close_request(self, *, session_id, nav_goal_uuid, generation, source):
        """Caller must admit a fresh STOPPING snapshot and its exact publisher."""
        binding = self.session_binding
        stamp = source.get('stamp') if isinstance(source, dict) else None
        if (binding is None or self.owner_token is None or self.receipt is not None or
                session_id != binding.session_id or nav_goal_uuid != binding.nav_goal_uuid or
                type(generation) is not int or generation <= 0 or
                not isinstance(stamp, dict) or
                source.get('instance_id') != binding.source_instance_id or
                type(source.get('sequence')) is not int or source['sequence'] <= 0 or
                type(stamp.get('sec')) is not int or stamp['sec'] < 0 or
                type(stamp.get('nanosec')) is not int or not 0 <= stamp['nanosec'] < 1_000_000_000 or
                stamp['sec'] * 1_000_000_000 + stamp['nanosec'] <= 0):
            self.blocker = 'PUMP_CLOSE_CONTEXT_INVALID'
            return False
        if self.close_generation is not None:
            if generation != self.close_generation:
                self.blocker = 'PUMP_CLOSE_REQUEST_CHANGED'
                return False
            return True  # Do not re-date the event or reset its OFF barrier.
        self.close_generation = generation
        self.close_request_source = deepcopy(source)
        # Any OFF proof from before this close request is insufficient.
        self.sample = None
        self.request_ros_ns = None
        self.request_mono = None
        self.request_identity = None
        return True

    def update(self, payload, ros_now_ns, received_at, *, source_ok):
        self.sample = None
        if (type(received_at) not in (int, float) or
                not math.isfinite(received_at) or received_at < 0):
            self.blocker = 'ACTUATOR_RECEIPT_INVALID'
            return
        if source_ok is not True:
            self.blocker = 'ACTUATOR_PUBLISHER_INVALID'
            return
        age_ms = self.guard.admit(payload, ros_now_ns)
        if age_ms is None:
            self.blocker = self.guard.reason
            if self.guard.reason in ('ACTUATOR_CLOCK_BACKWARD', 'ACTUATOR_CLOCK_INVALID'):
                self.request_ros_ns = None
                self.request_mono = None
            return
        if self.pending and (self.guard.context_changed or
                             self.request_identity != self.guard.last_identity):
            # A new source requires another actual OFF publish and barrier.
            self.request_ros_ns = None
            self.request_mono = None
            self.request_identity = self.guard.last_identity
        self.sample = (deepcopy(payload), received_at, age_ms, ros_now_ns)
        self.blocker = None

    def fresh_off(self, now):
        if self.sample is None:
            return False
        payload, received, age_ms, _ = self.sample
        if (not math.isfinite(now) or not math.isfinite(received) or
                not 0 <= now - received or
                age_ms + (now - received) * 1000.0 > self.timeout_s * 1000.0):
            self.blocker = 'ACTUATOR_SOURCE_STALE'
            return False
        if (payload.get('pump_pwm_commanded_on') is not False or
                payload.get('servo_pwm_commanded_on') is not False or
                payload.get('servo_phase') != 'IDLE'):
            self.blocker = 'ACTUATOR_NOT_OFF_AND_IDLE'
            return False
        return True

    def request_after_off_publish(self, goal_uuid, ros_now_ns, published_at):
        """Caller must have published OFF successfully before calling."""
        if self.session_binding is not None and goal_uuid != self.session_binding.nav_goal_uuid:
            self.blocker = 'PUMP_RELEASE_GOAL_CHANGED'
            return False
        if not self.pending:
            self.pending = True
            self.goal_uuid = goal_uuid
        if goal_uuid != self.goal_uuid:
            self.sample = None
            self.request_ros_ns = None
            self.request_mono = None
            self.blocker = 'PUMP_RELEASE_GOAL_CHANGED'
            return False
        if (type(goal_uuid) is not str or len(goal_uuid) != 32 or
                goal_uuid == '0' * 32 or
                any(c not in '0123456789abcdef' for c in goal_uuid)):
            self.blocker = 'PUMP_RELEASE_GOAL_MISSING'
            return False
        if (type(ros_now_ns) is not int or ros_now_ns <= 0 or
                type(published_at) not in (int, float) or
                not math.isfinite(published_at) or published_at < 0):
            self.blocker = 'ACTUATOR_CLOCK_INVALID'
            return False
        if (self.guard.last_ros_ns is not None and
                ros_now_ns < self.guard.last_ros_ns):
            self.sample = None
            self.request_ros_ns = None
            self.request_mono = None
            self.blocker = 'ACTUATOR_CLOCK_BACKWARD'
            return False
        if self.request_ros_ns is not None and (
                published_at < self.request_mono or
                abs((ros_now_ns - self.request_ros_ns) -
                    (published_at - self.request_mono) * 1e9) >
                round(self.guard.max_future_skew_ms * 1e6)):
            self.sample = None
            self.request_ros_ns = None
            self.request_mono = None
            self.blocker = 'ACTUATOR_CLOCK_DISCONTINUITY'
            return False
        if self.request_ros_ns is None:
            if self.guard.last_identity is None:
                self.blocker = 'ACTUATOR_NOT_RECEIVED'
                return False
            self.request_ros_ns = ros_now_ns
            self.request_mono = published_at
            self.request_identity = self.guard.last_identity
        return True

    def ready(self, now):
        if not self.pending or self.request_ros_ns is None or self.owner_token is None:
            return False
        if not self.fresh_off(now):
            return False
        payload, _, age_ms, received_ros_ns = self.sample
        source = payload['source']
        identity = (source['bridge_instance_id'], source['serial_epoch'])
        stamp = source['stamp']
        stamp_ns = stamp['sec'] * 1_000_000_000 + stamp['nanosec']
        conservative_stamp_ns = min(stamp_ns, received_ros_ns - math.ceil(age_ms * 1e6))
        # Account for the admitted inter-host future skew: a slightly ahead
        # Pi stamp from before OFF is not evidence of a later ACT receipt.
        if (identity != self.request_identity or conservative_stamp_ns <= self.request_ros_ns +
                round(self.guard.max_future_skew_ms * 1e6)):
            self.blocker = 'ACTUATOR_SAMPLE_NOT_AFTER_OFF'
            return False
        self.blocker = None
        return True

    def destroyed(self, *, ros_now_ns, checked_at):
        """Caller must have checked destroy_publisher(...) is True first."""
        if (type(ros_now_ns) is not int or ros_now_ns <= 0 or
                self.request_ros_ns is None or ros_now_ns < self.request_ros_ns or
                not self.ready(checked_at)):
            raise RuntimeError('PUMP_RELEASE_EVIDENCE_INVALID')
        payload, received_at, age_ms, received_ros_ns = self.sample
        self.receipt = {
            'owner_token': self.owner_token,
            'goal_uuid': self.goal_uuid,
            'session_id': (None if self.session_binding is None else
                           self.session_binding.session_id),
            'off_request_ros_ns': self.request_ros_ns,
            'off_request_mono': self.request_mono,
            'released_ros_ns': ros_now_ns,
            'checked_at_mono': checked_at,
            'actuator_received_mono': received_at,
            'actuator_age_at_receipt_ms': age_ms,
            'actuator_receipt_ros_ns': received_ros_ns,
            'actuator_source': deepcopy(payload['source']),
            'actuator_esp_ms': payload['esp_ms'],
            'physical_pump_or_nozzle_verified': False,
        }
        if self.session_binding is not None:
            self.receipt['session_binding'] = asdict(self.session_binding)
            self.receipt['close_generation'] = self.close_generation
            self.receipt['close_request_source'] = deepcopy(self.close_request_source)
        self.pending = False
        self.blocker = None

    def snapshot(self, *, owned):
        result = {
            'mode': 'TERMINAL_RELEASE_MIGRATION',
            'publisher_owned': owned,
            'pending': self.pending,
            'blocker': self.blocker,
            'receipt': deepcopy(self.receipt),
        }
        if self.session_binding is not None:
            result['session_binding'] = asdict(self.session_binding)
        return result
