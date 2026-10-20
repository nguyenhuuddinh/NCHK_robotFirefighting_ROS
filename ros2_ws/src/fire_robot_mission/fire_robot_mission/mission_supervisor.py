"""Observe Nav2 and fire perception without commanding the robot."""

import json
import math
import time

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from fire_robot_safety.odom_freshness import OdomStampGuard, admit_odom
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from fire_robot_mission.detection_freshness import DetectionStampGuard
from fire_robot_mission.fire_session_core import FireSessionCore
from fire_robot_mission.actuator_freshness import ActuatorStampGuard
from fire_robot_mission.close_owner_evidence import CloseOwnerEvidenceCollector
from fire_robot_mission.global_stop_evidence import GlobalStopEvidence
from fire_robot_mission.stop_clock_sampling import sample_stop_clock
from fire_robot_mission.runtime_manifest import runtime_manifest


# Explicit, conservative manifest for the optional observation-only collector.
# This does not start owners or infer that an unlaunched owner has released.
CLOSE_OWNER_MANIFEST = 'all_four_external_v1'
CLOSE_OWNER_TOPICS = {
    'live_nav_stop_gateway': '/fire_mission/live_nav_stop_state',
    'live_alignment_takeover': '/fire_mission/live_alignment_state',
    'ir_approach_live': '/fire_mission/ir_approach_live_state',
    'pump_demo_controller': '/fire_mission/pump_demo_state',
}
STOP_INPUTS = {
    'odom': ('/odom', 'serial_bridge_node', 'nav_msgs/msg/Odometry', False),
    'cmd': ('/cmd_vel', 'safety_watchdog', 'geometry_msgs/msg/Twist', False),
    'actuator': ('/fire_sensors/actuator_state', 'serial_bridge_node', 'std_msgs/msg/String', False),
    'nav_status': ('/navigate_to_pose/_action/status', 'bt_navigator',
                   'action_msgs/msg/GoalStatusArray', True),
}
THROUGH_STOP_INPUT = ('/navigate_through_poses/_action/status', 'bt_navigator',
                      'action_msgs/msg/GoalStatusArray', True)


def _stop_clock_pair(node):
    """Bracket selected stop reads; preserve the legacy mode's clock path."""
    stop = getattr(node, '_global_stop', None)
    if stop is None:
        return node.get_clock().now().nanoseconds, node._now()
    sample = sample_stop_clock(lambda: node.get_clock().now().nanoseconds, node._now,
                               previous_ros_ns=stop._last_ros, previous_mono=stop._last_mono)
    stop.observe_clock_sample(sample)
    return sample.ros_ns, sample.after


def _full_twist(twist):
    return (twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z)


ACTIVE_GOAL_STATUSES = {1, 2, 3}
GOAL_STATUS_NAMES = {
    0: 'UNKNOWN',
    1: 'ACCEPTED',
    2: 'EXECUTING',
    3: 'CANCELING',
    4: 'SUCCEEDED',
    5: 'CANCELED',
    6: 'ABORTED',
}
FORBIDDEN_OUTPUT_TOPICS = {
    '/cmd_vel',
    '/cmd_vel_raw',
    '/cmd_vel_nav',
    '/pump_cmd',
    '/fire_target',
}


def sensor_qos():
    """Return a no-backlog profile for high-rate robot telemetry."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def action_status_qos():
    """Match the standard ROS 2 action status topic profile."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _is_number(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _age_seconds(received_at, now):
    if received_at is None:
        return None
    return max(0.0, float(now) - float(received_at))


def _valid_goal_uuid(value):
    return (isinstance(value, str) and len(value) == 32 and
            value != '0' * 32 and
            all(char in '0123456789abcdef' for char in value.lower()))


class MissionSupervisorCore:
    """Pure state machine used by the ROS node and unit tests."""

    def __init__(
            self, detection_timeout_s=0.5, alignment_timeout_s=0.5,
            odom_timeout_s=0.5, command_timeout_s=0.5,
            fire_acquire_hold_s=0.5, fire_reacquire_hold_s=None,
            centered_hold_s=0.5,
            max_source_age_ms=300.0, max_future_skew_ms=20.0,
            stationary_linear_speed_m_s=0.02,
            stationary_angular_speed_rad_s=0.05,
            turn_comparison_epsilon_rad_s=0.03,
            review_timeout_s=0.6, actuator_timeout_s=0.35,
            pre_spray_clear_hold_s=1.5,
            enable_fire_session_observer=False):
        if type(enable_fire_session_observer) is not bool:
            raise ValueError('enable_fire_session_observer must be a bool')
        positive = {
            'detection_timeout_s': detection_timeout_s,
            'alignment_timeout_s': alignment_timeout_s,
            'odom_timeout_s': odom_timeout_s,
            'command_timeout_s': command_timeout_s,
            'fire_acquire_hold_s': fire_acquire_hold_s,
            'centered_hold_s': centered_hold_s,
            'max_source_age_ms': max_source_age_ms,
            'max_future_skew_ms': max_future_skew_ms,
            'stationary_linear_speed_m_s': stationary_linear_speed_m_s,
            'stationary_angular_speed_rad_s': (
                stationary_angular_speed_rad_s),
            'turn_comparison_epsilon_rad_s': (
                turn_comparison_epsilon_rad_s),
            'review_timeout_s': review_timeout_s,
            'actuator_timeout_s': actuator_timeout_s,
            'pre_spray_clear_hold_s': pre_spray_clear_hold_s,
        }
        invalid = [name for name, value in positive.items() if value <= 0.0]
        if invalid:
            raise ValueError(
                'mission supervisor parameters must be positive: '
                + ', '.join(invalid)
            )
        if fire_reacquire_hold_s is None:
            fire_reacquire_hold_s = min(0.2, fire_acquire_hold_s)
        if fire_reacquire_hold_s <= 0.0:
            raise ValueError('fire_reacquire_hold_s must be positive')
        self.detection_timeout = float(detection_timeout_s)
        self.alignment_timeout = float(alignment_timeout_s)
        self.odom_timeout = float(odom_timeout_s)
        self.command_timeout = float(command_timeout_s)
        self.fire_acquire_hold = float(fire_acquire_hold_s)
        self.fire_reacquire_hold = float(fire_reacquire_hold_s)
        if self.fire_reacquire_hold > self.fire_acquire_hold:
            raise ValueError('fire_reacquire_hold_s must not exceed '
                             'fire_acquire_hold_s')
        self.centered_hold = float(centered_hold_s)
        self.max_source_age = float(max_source_age_ms)
        self.max_future_skew = float(max_future_skew_ms)
        self.stationary_linear = float(stationary_linear_speed_m_s)
        self.stationary_angular = float(stationary_angular_speed_rad_s)
        self.turn_epsilon = float(turn_comparison_epsilon_rad_s)
        self.review_timeout = float(review_timeout_s)
        self.actuator_timeout = float(actuator_timeout_s)
        self.pre_spray_clear_hold = float(pre_spray_clear_hold_s)
        # Optional migration telemetry only. OFF creates no session core/UUID.
        self._fire_session = (FireSessionCore(
            proof_timeout_s=self.actuator_timeout)
            if enable_fire_session_observer else None)
        self._session_runtime = None

        self._detection = None
        self._detection_received = None
        self._detection_error = ''
        self._detection_error_blocker = 'DETECTION_INVALID_JSON'
        self._alignment = None
        self._alignment_received = None
        self._alignment_error = ''
        self._odom = None
        self._odom_received = None
        self._nav_command = None
        self._nav_command_received = None
        self._raw_command = None
        self._raw_command_received = None
        self._goal_statuses = []
        self._goal_status_records = []
        self._goal_status_received = None
        self._review = None
        self._review_received = None
        self._actuator = None
        self._actuator_received = None
        self._actuator_source_age_ms = 0.0
        self._actuator_source_blocker = 'ACTUATOR_NOT_RECEIVED'
        self._ir = None
        self._ir_received = None
        self._pump_started_uuid = None
        self._pump_owner_count = 0
        self._pre_spray_clear_since = None
        self._pre_spray_clear_uuid = None
        self._candidate_since = None
        self._candidate_reacquire = False
        self._candidate_confirmed = False
        self._candidate_last_detection_at = None
        self._candidate_detection_count = 0
        self._centered_since = None
        self._ever_acquired = False
        self._acquired_goal_uuid = None
        self._acquired_goal_canceled = False
        self._last_state = None
        self._state_since = None

    def update_detection(self, payload, now):
        """Store one decoded YOLO observation."""
        self._detection = payload
        self._detection_received = float(now)
        self._detection_error = ''
        self._detection_error_blocker = 'DETECTION_INVALID_JSON'

    def mark_detection_error(self, error, now, *,
                             blocker='DETECTION_INVALID_JSON'):
        """Record malformed detection telemetry as a fail-safe input."""
        self._detection = None
        self._detection_received = float(now)
        self._detection_error = str(error)
        self._detection_error_blocker = blocker
        self._reset_candidate()
        self._centered_since = None

    def update_alignment(self, payload, now):
        """Store one decoded alignment preview."""
        self._alignment = payload
        self._alignment_received = float(now)
        self._alignment_error = ''

    def mark_alignment_error(self, error, now):
        """Record malformed alignment telemetry as a fail-safe input."""
        self._alignment = None
        self._alignment_received = float(now)
        self._alignment_error = str(error)

    def update_odom(self, linear_x, angular_z, now):
        """Store measured robot velocity for readiness checks."""
        self._odom = (float(linear_x), float(angular_z))
        self._odom_received = float(now)

    def update_nav_command(self, linear_x, angular_z, now):
        """Store the latest controller-server command for comparison."""
        self._nav_command = (float(linear_x), float(angular_z))
        self._nav_command_received = float(now)

    def update_raw_command(self, linear_x, angular_z, now):
        """Store the command entering the Pi safety gate."""
        self._raw_command = (float(linear_x), float(angular_z))
        self._raw_command_received = float(now)

    def update_goal_statuses(self, statuses, now):
        """Store action statuses and their UUIDs when supplied by Nav2."""
        records = []
        for item in statuses:
            if isinstance(item, (tuple, list)) and len(item) == 2:
                uuid, status = item
            else:
                uuid, status = None, item
            records.append((str(uuid).lower() if uuid is not None else None,
                            int(status)))
        self._goal_status_records = records
        self._goal_statuses = [status for _, status in records]
        self._goal_status_received = float(now)
        if _valid_goal_uuid(self._acquired_goal_uuid):
            if any(uuid != self._acquired_goal_uuid and
                   status in ACTIVE_GOAL_STATUSES and
                   _valid_goal_uuid(uuid) for uuid, status in records):
                self._acquired_goal_canceled = False
                self._reset_candidate()
            elif any(uuid == self._acquired_goal_uuid and status == 5
                     for uuid, status in records):
                self._acquired_goal_canceled = True
            elif any(uuid == self._acquired_goal_uuid and status != 5
                     for uuid, status in records):
                self._acquired_goal_canceled = False

    def update_review(self, payload, now):
        """Store subscriber-only post-spray evidence for this goal."""
        self._review = payload
        self._review_received = float(now)

    def update_actuator(self, payload, now, *, source_age_ms=0.0, blocker=None):
        """Store receipt-aged telemetry; pure callers must admit source first."""
        if (blocker is not None or not isinstance(payload, dict) or
                type(source_age_ms) not in (int, float) or
                not math.isfinite(source_age_ms) or source_age_ms < 0):
            payload = None
            source_age_ms = 0.0
            blocker = blocker or 'ACTUATOR_REPORT_INVALID'
        self._actuator_source_blocker = blocker
        self._actuator = payload
        self._actuator_received = float(now)
        self._actuator_source_age_ms = float(source_age_ms)
        if (payload is None or payload.get('pump_pwm_commanded_on') is not False or
                payload.get('servo_pwm_commanded_on') is not False or
                payload.get('servo_phase') != 'IDLE'):
            # Even if a valid OFF arrives before the next timer, an intervening
            # invalid/ON observation must break the old absence/review proof.
            self._pre_spray_clear_since = None
            self._review = None
            self._review_received = None

    def _actuator_age(self, now):
        if (self._actuator_received is None or not math.isfinite(now) or
                not math.isfinite(self._actuator_received) or
                now < self._actuator_received):
            return None
        return now - self._actuator_received + self._actuator_source_age_ms / 1000.0

    def update_ir(self, payload, now):
        self._ir = payload
        self._ir_received = float(now)

    def update_pump(self, payload, now):
        if (isinstance(payload, dict) and
                payload.get('state') == 'FIRING' and
                payload.get('pump_on') is True and
                payload.get('goal_uuid') == self._acquired_goal_uuid):
            self._pump_started_uuid = self._acquired_goal_uuid

    def update_pump_owner_count(self, count):
        self._pump_owner_count = int(count)
        if self._pump_owner_count > 0 and _valid_goal_uuid(
                self._acquired_goal_uuid):
            self._pump_started_uuid = self._acquired_goal_uuid

    def _pre_spray_clear(self, now, nav_active):
        """Require sustained camera and IR absence after cancellation."""
        fire = ((self._detection or {}).get('fire') or {}) if \
            isinstance(self._detection, dict) else {}
        ready = (fire.get('detected_now') is False and
                 fire.get('confirmed') is False and
                 self._ever_acquired and
                 _valid_goal_uuid(self._acquired_goal_uuid) and
                 self._acquired_goal_canceled and not nav_active and
                 self._pump_owner_count == 0 and
                 self._pump_started_uuid != self._acquired_goal_uuid and
                 not any(status in ACTIVE_GOAL_STATUSES
                         for _, status in self._goal_status_records))
        ir = self._ir if isinstance(self._ir, dict) else {}
        actuator = self._actuator if isinstance(self._actuator, dict) else {}
        age = _age_seconds(self._ir_received, now)
        actuator_age = self._actuator_age(now)
        odom_fresh, stationary, _ = self._odom_status(now)
        actuator_fresh = (actuator_age is not None and
                          actuator_age <= self.actuator_timeout)
        actuator_off = (actuator.get('schema_version') == 1 and
                        actuator.get('mode') == 'COMMAND_TELEMETRY_ONLY' and
                        actuator.get('pump_pwm_commanded_on') is False and
                        actuator.get('servo_pwm_commanded_on') is False and
                        actuator.get('servo_phase') == 'IDLE')
        ready = (ready and age is not None and age <= 0.6 and
                 ir.get('schema_version') == 1 and
                 ir.get('mode') == 'OBSERVE_ONLY' and
                 ir.get('observe_only') is True and
                 ir.get('ir_band') == 'NO_FLAME_SIGNATURE' and
                 ir.get('zone') == 'UNKNOWN' and
                 type(ir.get('stable_samples')) is int and
                 ir['stable_samples'] >= 3 and
                 isinstance(ir.get('ir_receive_age_ms'), (int, float)) and
                 0 <= ir['ir_receive_age_ms'] <= 300 and
                 isinstance(ir.get('blockers'), list) and
                 'NO_IR_FLAME_SIGNATURE' in ir['blockers'] and
                 set(ir['blockers']) <= {
                     'NO_IR_FLAME_SIGNATURE', 'FIRE_NOT_CONFIRMED',
                     'FIRE_NOT_CENTERED'} and
                 odom_fresh and stationary and
                 self._motion_owner(False, now) !=
                 'UNKNOWN_RAW_COMMAND_SOURCE')
        # Keep the no-fire/stationary hold across gaps between OFF reports.
        # A fresh ON or invalid report revokes it immediately; the first and
        # final samples must both be fresh, valid OFF telemetry.
        if not ready or (actuator_fresh and not actuator_off):
            self._pre_spray_clear_since = None
            return False
        if actuator_fresh and actuator_off and \
                self._pre_spray_clear_since is None:
            self._pre_spray_clear_since = now
        return (self._pre_spray_clear_since is not None and
                actuator_fresh and actuator_off and
                now - self._pre_spray_clear_since >=
                self.pre_spray_clear_hold)

    def _active_goal_uuid(self):
        active = {uuid for uuid, status in self._goal_status_records
                  if status in ACTIVE_GOAL_STATUSES and
                  _valid_goal_uuid(uuid)}
        return next(iter(active)) if len(active) == 1 else None

    def _same_goal_reacquisition(self, nav_active):
        """Only shorten confirmation for the already acquired Nav2 goal."""
        if not self._ever_acquired or not _valid_goal_uuid(
                self._acquired_goal_uuid):
            return False
        if nav_active:
            return self._active_goal_uuid() == self._acquired_goal_uuid
        return self._acquired_goal_canceled

    def _reset_candidate(self):
        self._candidate_since = None
        self._candidate_reacquire = False
        self._candidate_confirmed = False
        self._candidate_last_detection_at = None
        self._candidate_detection_count = 0

    def _reviewed_clear(self, now, nav_active):
        """Require fresh exact-goal review and stopped actuator to clear."""
        if not self._ever_acquired or not _valid_goal_uuid(
                self._acquired_goal_uuid) or nav_active:
            return False
        if _age_seconds(self._review_received, now) is None or \
                _age_seconds(self._review_received, now) > \
                self.review_timeout:
            return False
        if self._actuator_age(now) is None or \
                self._actuator_age(now) > \
                self.actuator_timeout:
            return False
        review = self._review if isinstance(self._review, dict) else {}
        actuator = self._actuator if isinstance(self._actuator, dict) else {}
        safety = review.get('safety') or {}
        odom_fresh, stationary, _ = self._odom_status(now)
        return (review.get('schema_version') == 1 and
                review.get('mode') == 'SHADOW_ONLY' and
                review.get('state') == 'NO_FIRE_EVIDENCE' and
                review.get('goal_uuid') == self._acquired_goal_uuid and
                type(review.get('pulse_index')) is int and
                review['pulse_index'] >= 1 and
                review.get('blockers') == [] and
                safety.get('motion_command_publisher') is False and
                safety.get('pump_command_publisher') is False and
                safety.get('nav2_action_client') is False and
                safety.get('actuation_authorized') is False and
                actuator.get('schema_version') == 1 and
                actuator.get('mode') == 'COMMAND_TELEMETRY_ONLY' and
                actuator.get('pump_pwm_commanded_on') is False and
                actuator.get('servo_phase') == 'IDLE' and
                actuator.get('servo_pwm_commanded_on') is False and
                odom_fresh and stationary and
                self._motion_owner(False, now) !=
                'UNKNOWN_RAW_COMMAND_SOURCE' and
                self._acquired_goal_canceled and
                not any(status in ACTIVE_GOAL_STATUSES
                        for _, status in self._goal_status_records) and
                not any(uuid == self._acquired_goal_uuid and status != 5
                        for uuid, status in self._goal_status_records))

    def _nav2_goal(self):
        active = [
            status for status in self._goal_statuses
            if status in ACTIVE_GOAL_STATUSES
        ]
        if active:
            if 2 in active:
                status = 2
            elif 3 in active:
                status = 3
            else:
                status = 1
            return True, GOAL_STATUS_NAMES[status]
        if self._acquired_goal_canceled:
            return False, 'CANCELED'
        if self._goal_statuses:
            status = self._goal_statuses[-1]
            return False, GOAL_STATUS_NAMES.get(status, f'INVALID_{status}')
        return False, 'IDLE_OR_UNKNOWN'

    def _detection_health(self, now):
        blockers = []
        warnings = []
        age = _age_seconds(self._detection_received, now)
        if self._detection_received is None:
            blockers.append('DETECTION_NOT_RECEIVED')
            return blockers, warnings, None
        if age > self.detection_timeout:
            blockers.append('DETECTION_TIMEOUT')
        if self._detection_error:
            blockers.append(self._detection_error_blocker)
            return blockers, warnings, None
        payload = self._detection
        if not isinstance(payload, dict):
            blockers.append('DETECTION_INVALID_PAYLOAD')
            return blockers, warnings, None
        if payload.get('observe_only') is not True:
            blockers.append('DETECTION_NOT_OBSERVE_ONLY')
        state = str(payload.get('state', 'missing'))
        if state != 'ok':
            blockers.append(f'DETECTION_STATE_{state.upper()}')
        source_age = (payload.get('source') or {}).get('age_ms')
        if not _is_number(source_age):
            blockers.append('SOURCE_AGE_MISSING')
        else:
            reported_source_age = float(source_age)
            # age_ms belongs to the YOLO observation, not this evaluation.
            # Charge monotonic time spent cached here on every timer tick.
            # The ROS adapter first accounts for transit using the stamp.
            # Direct core callers must likewise supply age at receipt.
            source_age = max(0.0, reported_source_age) + age * 1000.0
            if reported_source_age < -self.max_future_skew:
                # Waiting must not rehabilitate an invalid future timestamp.
                blockers.append('SOURCE_TIMESTAMP_IN_FUTURE')
            elif source_age > self.max_source_age:
                blockers.append('SOURCE_TIMESTAMP_STALE')
            elif reported_source_age < 0.0:
                warnings.append('SMALL_NEGATIVE_SOURCE_AGE')
        return blockers, warnings, source_age

    def _alignment_health(self, now, target_present):
        blockers = []
        if not target_present:
            return blockers
        age = _age_seconds(self._alignment_received, now)
        if self._alignment_received is None:
            return ['ALIGNMENT_NOT_RECEIVED']
        if age > self.alignment_timeout:
            blockers.append('ALIGNMENT_TIMEOUT')
        if self._alignment_error:
            blockers.append('ALIGNMENT_INVALID_JSON')
            return blockers
        payload = self._alignment
        if not isinstance(payload, dict):
            return blockers + ['ALIGNMENT_INVALID_PAYLOAD']
        if payload.get('observe_only') is not True:
            blockers.append('ALIGNMENT_NOT_OBSERVE_ONLY')
        safety = payload.get('safety') or {}
        if safety.get('motion_command_publisher') is not False or \
                safety.get('actuator_command_publisher') is not False:
            blockers.append('ALIGNMENT_SAFETY_CONTRACT')
        return blockers

    def _alignment_values(self):
        payload = self._alignment or {}
        preview = payload.get('preview') or {}
        state = str(payload.get('state', 'NO_TARGET'))
        direction = str(preview.get('direction', 'STOP'))
        angular = preview.get('target_angular_z_rad_s')
        if not _is_number(angular):
            angular = 0.0
        return state, direction, float(angular)

    def _command_comparison(self, fire_angular, now, target_present):
        age = _age_seconds(self._nav_command_received, now)
        if not target_present:
            return 'NO_FIRE_TARGET'
        if self._nav_command is None or age > self.command_timeout:
            return 'NAV_COMMAND_UNAVAILABLE'
        nav_angular = self._nav_command[1]
        if abs(fire_angular) <= self.turn_epsilon:
            return 'FIRE_CENTERED'
        if abs(nav_angular) <= self.turn_epsilon:
            return 'NAV_NEUTRAL'
        if fire_angular * nav_angular > 0.0:
            return 'AGREE'
        return 'CONFLICT'

    def _odom_status(self, now):
        age = _age_seconds(self._odom_received, now)
        fresh = self._odom is not None and age <= self.odom_timeout
        stationary = False
        if fresh:
            stationary = (
                abs(self._odom[0]) <= self.stationary_linear
                and abs(self._odom[1]) <= self.stationary_angular
            )
        return fresh, stationary, age

    def _motion_owner(self, nav_active, now):
        raw_age = _age_seconds(self._raw_command_received, now)
        raw_active = self._raw_command is not None \
            and raw_age <= self.command_timeout \
            and (
                abs(self._raw_command[0]) > 1e-6
                or abs(self._raw_command[1]) > 1e-6
            )
        if nav_active:
            return 'NAV2'
        if raw_active:
            return 'UNKNOWN_RAW_COMMAND_SOURCE'
        return 'NONE'

    def _set_state(self, state, now):
        if state != self._last_state:
            self._last_state = state
            self._state_since = float(now)
        return round((float(now) - self._state_since) * 1000.0, 1)

    def _observe_fire_session(self, now, hard_blockers, target_present,
                              pre_spray_clear, reviewed_clear, clear_goal):
        """Mirror admission/close requests without granting command authority.

        Cleanup is separate and runs only after a successful STOPPING publish.
        """
        session = self._fire_session
        runtime = self._session_runtime
        blocker = None
        absence = False
        if runtime is not None:
            fire = (self._detection or {}).get('fire') or {}
            source_age_ms = ((self._detection or {}).get('source') or {}).get('age_ms')
            effective_at = (self._detection_received - max(0.0, source_age_ms) / 1000.0
                            if self._detection_received is not None and _is_number(source_age_ms) else None)
            absence = runtime.observe_absence(
                absent=(not hard_blockers and fire.get('detected_now') is False and
                        fire.get('confirmed') is False),
                event_at=effective_at, now=now)
        current_status = (self._goal_status_received is not None and
                          0 <= now - self._goal_status_received <=
                          (self.command_timeout if runtime is None else
                           min(self.command_timeout, runtime.timeout_s)))
        active = [uuid for uuid, status in self._goal_status_records
                  if status in ACTIVE_GOAL_STATUSES]
        goal = self._active_goal_uuid()
        ambiguous = bool(active) and (
            goal is None or any(not _valid_goal_uuid(uuid) for uuid in active))
        identity = session.identity
        if identity is not None:
            if session.phase == 'ACTIVE':
                reason = None
                if hard_blockers or (runtime is not None and runtime.owner_blocker(now)):
                    reason = 'SAFETY_ABORT'
                elif (current_status and active and
                      (ambiguous or goal != identity.nav_goal_uuid)):
                    reason = 'SAFETY_ABORT'
                elif (current_status and identity.nav_goal_uuid is not None and
                      (identity.nav_goal_uuid, 6) in self._goal_status_records):
                    reason = 'NAVIGATION_ABORTED'
                elif clear_goal == identity.nav_goal_uuid and clear_goal is not None:
                    if reviewed_clear:
                        reason = 'POST_SPRAY_CLEAR'
                    elif pre_spray_clear:
                        reason = 'TARGET_ABSENT_CONFIRMED'
                if reason is None and runtime is not None and absence:
                    reason = 'TARGET_ABSENT_CONFIRMED'
                if reason is not None:
                    session.request_close(identity.session_id, reason, now)
            if session.phase == 'STOPPING':
                blocker = ('VERIFIED_SESSION_CLEANUP_NOT_CONNECTED' if runtime is None else
                           runtime.blocker or 'GLOBAL_STOP_PROOF_REQUIRED')
            elif not current_status:
                blocker = 'NAV2_STATUS_NOT_CURRENT'
            elif not target_present:
                blocker = 'TARGET_ABSENCE_PROOF_NOT_CONNECTED'
        elif not hard_blockers and target_present and self._candidate_confirmed:
            if not current_status:
                blocker = 'NAV2_STATUS_NOT_CURRENT'
            elif ambiguous:
                blocker = 'NAV2_GOAL_BINDING_AMBIGUOUS'
            else:
                # Empty current status permits an unbound observation. It is
                # not a verified "Nav2 inactive" proof for live admission.
                if runtime is None:
                    session.begin(goal if active else None)
                else:
                    runtime.try_begin(session, goal if active else None, now)
                    blocker = runtime.blocker
        identity = session.identity
        return {
            'mode': 'OBSERVE_ONLY',
            'actuation_authorized': False,
            'phase': session.phase,
            'session_id': None if identity is None else identity.session_id,
            'nav_goal_uuid': None if identity is None else identity.nav_goal_uuid,
            'close_reason': session.close_reason,
            'close_generation': (None if session.close_request is None else
                                 session.close_request.generation),
            'blocker': blocker,
        }

    def finish_verified_session(self, evidence, now):
        """Retire legacy session inputs only after the local typed proof succeeds.

        Source guards/clock watermarks belong to the node and are preserved.
        No owner arm, live limit, watchdog or durable budget is reset here.
        """
        if self._session_runtime is None or not self._fire_session.finish_close(evidence, now):
            return False
        for name in ('_detection', '_detection_received', '_alignment', '_alignment_received',
                     '_odom', '_odom_received', '_nav_command', '_nav_command_received',
                     '_raw_command', '_raw_command_received', '_review', '_review_received',
                     '_actuator', '_actuator_received', '_ir', '_ir_received',
                     '_goal_status_received', '_pump_started_uuid', '_pre_spray_clear_since',
                     '_pre_spray_clear_uuid', '_centered_since', '_acquired_goal_uuid',
                     '_last_state', '_state_since'):
            setattr(self, name, None)
        self._goal_statuses, self._goal_status_records = [], []
        self._detection_error = self._alignment_error = ''
        self._actuator_source_blocker = 'ACTUATOR_NOT_RECEIVED'
        self._actuator_source_age_ms = 0.0
        self._ever_acquired = self._acquired_goal_canceled = False
        self._reset_candidate()
        return True

    def evaluate(self, now):
        """Return one JSON-safe observe-only mission snapshot."""
        now = float(now)
        nav_active, nav_state = self._nav2_goal()
        detection_blockers, warnings, source_age = (
            self._detection_health(now))
        fire = {}
        if isinstance(self._detection, dict):
            fire = self._detection.get('fire') or {}
        detected_now = fire.get('detected_now') is True
        confirmed = fire.get('confirmed') is True
        target_present = detected_now and confirmed
        alignment_blockers = self._alignment_health(now, target_present)
        hard_blockers = detection_blockers + alignment_blockers
        if target_present or hard_blockers:
            self._pre_spray_clear_since = None

        pre_spray_clear = (not target_present and not hard_blockers and
                           self._pre_spray_clear(now, nav_active))
        reviewed_clear = (not target_present and not hard_blockers and
                          self._reviewed_clear(now, nav_active))
        clear_goal = self._acquired_goal_uuid
        if reviewed_clear or pre_spray_clear:
            if pre_spray_clear:
                self._pre_spray_clear_uuid = self._acquired_goal_uuid
            self._ever_acquired = False
            self._acquired_goal_uuid = None
            self._acquired_goal_canceled = False
            self._pre_spray_clear_since = None

        alignment_state, direction, fire_angular = (
            self._alignment_values())
        intent = 'HOLD_STOP'
        if self._detection_received is None:
            state = 'WAITING_FOR_INPUT'
        elif hard_blockers:
            state = 'FAULT'
            intent = 'REQUEST_SAFE_STOP'
            self._reset_candidate()
            self._centered_since = None
        elif target_present:
            goal_uuid = self._active_goal_uuid() if nav_active else None
            if (self._candidate_confirmed and goal_uuid is not None and
                    self._acquired_goal_uuid is not None and
                    goal_uuid != self._acquired_goal_uuid):
                self._reset_candidate()
            reacquire = self._same_goal_reacquisition(nav_active)
            if self._candidate_since is None or \
                    (not self._candidate_confirmed and
                     self._candidate_reacquire != reacquire):
                self._reset_candidate()
                self._candidate_since = now
                self._candidate_reacquire = reacquire
            if self._candidate_last_detection_at != \
                    self._detection_received:
                self._candidate_last_detection_at = self._detection_received
                self._candidate_detection_count += 1
            acquired_for = now - self._candidate_since
            hold = (self.fire_reacquire_hold if reacquire else
                    self.fire_acquire_hold)
            enough_time = acquired_for >= hold
            enough_samples = self._candidate_detection_count >= 2
            if not self._candidate_confirmed and not (
                    enough_time and enough_samples):
                state = 'FIRE_CANDIDATE'
                intent = 'KEEP_NAV2' if nav_active else 'HOLD_STOP'
                self._centered_since = None
            else:
                self._candidate_confirmed = True
                if goal_uuid is not None:
                    self._ever_acquired = True
                    self._acquired_goal_uuid = goal_uuid
                    self._acquired_goal_canceled = False
                if alignment_state == 'CENTERED':
                    if self._centered_since is None:
                        self._centered_since = now
                    centered_for = now - self._centered_since
                    if centered_for >= self.centered_hold:
                        state = 'ALIGNMENT_READY'
                    else:
                        state = 'CENTERED_HOLD'
                    intent = 'HOLD_CENTER_PREVIEW'
                elif alignment_state in (
                        'ALIGNING_LEFT', 'ALIGNING_RIGHT'):
                    state = 'FIRE_TRACKING'
                    intent = f'{direction}_PREVIEW'
                    self._centered_since = None
                else:
                    state = 'WAITING_FOR_ALIGNMENT'
                    intent = 'REQUEST_SAFE_STOP'
                    self._centered_since = None
        else:
            self._reset_candidate()
            self._centered_since = None
            if self._ever_acquired:
                state = 'TARGET_LOST'
                intent = 'REQUEST_SAFE_STOP'
            elif nav_active:
                state = 'NAV2_NAVIGATING'
                intent = 'KEEP_NAV2'
            else:
                state = 'IDLE'

        state_age_ms = self._set_state(state, now)
        odom_fresh, stationary, odom_age = self._odom_status(now)
        motion_owner = self._motion_owner(nav_active, now)
        readiness_blockers = list(hard_blockers)
        if state not in {
                'FIRE_TRACKING', 'CENTERED_HOLD', 'ALIGNMENT_READY'}:
            readiness_blockers.append('FIRE_NOT_ACQUIRED')
        if not odom_fresh:
            readiness_blockers.append('ODOM_MISSING_OR_STALE')
        elif not stationary:
            readiness_blockers.append('ROBOT_NOT_STATIONARY')
        if nav_active:
            readiness_blockers.append('NAV2_GOAL_ACTIVE')
        if motion_owner == 'UNKNOWN_RAW_COMMAND_SOURCE':
            readiness_blockers.append('RAW_COMMAND_SOURCE_ACTIVE')
        eligible = not readiness_blockers

        candidate = fire.get('candidate') or {}
        confidence = candidate.get('confidence')
        nav_command_age = _age_seconds(self._nav_command_received, now)
        raw_command_age = _age_seconds(self._raw_command_received, now)
        detection_age = _age_seconds(self._detection_received, now)
        alignment_age = _age_seconds(self._alignment_received, now)
        actuator_age = self._actuator_age(now)
        actuator_blocker = self._actuator_source_blocker
        if actuator_blocker is None and (
                actuator_age is None or actuator_age > self.actuator_timeout):
            actuator_blocker = 'ACTUATOR_SOURCE_STALE'
        lifecycle = {
            'pre_spray_clear_goal_uuid': self._pre_spray_clear_uuid,
            'actuator_source_blocker': actuator_blocker,
        }
        if self._fire_session is not None:
            lifecycle['fire_session_observer'] = self._observe_fire_session(
                now, hard_blockers, target_present, pre_spray_clear,
                reviewed_clear, clear_goal)
        return {
            'schema_version': 1,
            'observe_only': True,
            'state': state,
            'lifecycle': lifecycle,
            'state_age_ms': state_age_ms,
            'perception': {
                'detected_now': detected_now,
                'confirmed': confirmed,
                'confidence': (
                    round(float(confidence), 4)
                    if _is_number(confidence) else None
                ),
                'source_age_ms': source_age,
                'detection_receive_age_ms': (
                    None if detection_age is None
                    else round(detection_age * 1000.0, 1)
                ),
                'alignment_receive_age_ms': (
                    None if alignment_age is None
                    else round(alignment_age * 1000.0, 1)
                ),
                'alignment_state': alignment_state,
                'alignment_direction': direction,
                'alignment_target_angular_z_rad_s': round(
                    fire_angular, 4),
            },
            'navigation': {
                'goal_active': nav_active,
                'goal_state': nav_state,
                'observed_motion_owner': motion_owner,
                'nav_command': self._command_dict(
                    self._nav_command, nav_command_age),
                'raw_command': self._command_dict(
                    self._raw_command, raw_command_age),
                'fire_nav_turn_comparison': self._command_comparison(
                    fire_angular, now, target_present),
            },
            'robot': {
                'odom_fresh': odom_fresh,
                'odom_age_ms': (
                    None if odom_age is None
                    else round(odom_age * 1000.0, 1)
                ),
                'stationary': stationary,
            },
            'decision': {
                'intent': intent,
                'eligible_for_future_alignment_handoff': eligible,
                'readiness_blockers': sorted(set(readiness_blockers)),
                'command_published': False,
            },
            'health': {
                'status': 'BLOCKED' if hard_blockers else 'OK',
                'blockers': sorted(set(hard_blockers)),
                'warnings': sorted(set(warnings)),
            },
            'safety': {
                'motion_command_publisher': False,
                'actuator_command_publisher': False,
                'nav2_action_client': False,
                'collision_check_bypassed': False,
            },
        }

    @staticmethod
    def _command_dict(command, age):
        if command is None:
            return None
        return {
            'linear_x_m_s': round(command[0], 4),
            'angular_z_rad_s': round(command[1], 4),
            'receive_age_ms': round(age * 1000.0, 1),
        }


class FireMissionSupervisor(Node):
    """Publish an action-free mission state from existing telemetry."""

    def __init__(self):
        super().__init__('fire_mission_supervisor')
        self._odom_stamp_guard = OdomStampGuard()
        self._declare_parameters()
        self._read_parameters()
        self._core = MissionSupervisorCore(**self._core_parameters)
        self._shadow_runtime = None
        self._close_owner_collector = (CloseOwnerEvidenceCollector(
            timeout_s=(min(self._core.odom_timeout, self._core.command_timeout,
                           self._core.actuator_timeout, 0.35)
                       if self.enable_global_stop_collection else 0.35),
            future_skew_ms=(min(self._core.max_future_skew, 20.0)
                            if self.enable_global_stop_collection else 20.0),
            manifest_id=self._owner_manifest.name)
            if self.enable_close_owner_collection else None)
        self._global_stop = (GlobalStopEvidence(
            linear_limit=min(self._core.stationary_linear, 0.02),
            angular_limit=min(self._core.stationary_angular, 0.05),
            timeout_s=min(self._core.odom_timeout, self._core.command_timeout,
                          self._core.actuator_timeout, 0.35),
            future_skew_ms=min(self._core.max_future_skew, 20.0),
            manifest_id=self._owner_manifest.name,
            require_recovery_on_change=self.enable_shadow_session_cleanup,
            require_through_status=self.require_through_status)
            if self.enable_global_stop_collection else None)
        if self.enable_shadow_session_cleanup:
            from fire_robot_mission.shadow_budget import ShadowBudgetStore
            from fire_robot_mission.shadow_session_runtime import ShadowSessionRuntime
            store = ShadowBudgetStore(self.get_parameter('shadow_budget_file').value,
                                      initialize=self.get_parameter('initialize_shadow_budget').value)
            self._shadow_runtime = ShadowSessionRuntime(
                store, self._owner_manifest, timeout_s=self._global_stop.timeout_s,
                absence_hold_s=max(1.5, self._core.pre_spray_clear_hold))
            self._core._session_runtime = self._shadow_runtime
        if self._core._fire_session is not None:
            self._session_source_sequence = 0
        self._detection_stamp_guard = DetectionStampGuard(
            self._core.max_source_age, self._core.max_future_skew)
        self._actuator_stamp_guard = ActuatorStampGuard(
            max_age_ms=self._core.actuator_timeout * 1000.0,
            max_future_skew_ms=self._core.max_future_skew)
        qos = sensor_qos()
        self._state_publisher = self.create_publisher(
            String, self.state_topic, qos)
        self.create_subscription(
            String, self.detections_topic, self._on_detection, qos)
        self.create_subscription(
            String, self.alignment_topic, self._on_alignment, qos)
        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos)
        self.create_subscription(
            Twist, self.nav_command_topic, self._on_nav_command, qos)
        self.create_subscription(
            Twist, self.raw_command_topic, self._on_raw_command, qos)
        status_subscription = self.create_subscription(
            GoalStatusArray,
            self.nav_status_topic,
            self._on_nav_status,
            action_status_qos(),
        )
        if self._global_stop is not None:
            status_subscription._fire_stop_metadata_callback = self._on_nav_status
            command_subscription = self.create_subscription(
                Twist, '/cmd_vel', self._on_stop_cmd, qos)
            command_subscription._fire_stop_metadata_callback = self._on_stop_cmd
            if self.require_through_status:
                through_subscription = self.create_subscription(
                    GoalStatusArray, THROUGH_STOP_INPUT[0], self._on_through_nav_status,
                    action_status_qos())
                through_subscription._fire_stop_metadata_callback = self._on_through_nav_status
        self.create_subscription(
            String, self.post_spray_review_topic, self._on_review, qos)
        self.create_subscription(
            String, self.actuator_state_topic, self._on_actuator, qos)
        self.create_subscription(
            String, self.ir_range_topic, self._on_ir_range, qos)
        self.create_subscription(
            String, self.pump_state_topic, self._on_pump_state, qos)
        if self._close_owner_collector is not None:
            for owner, topic in CLOSE_OWNER_TOPICS.items():
                # Pump telemetry already has a subscription and JSON decoder.
                if owner != 'pump_demo_controller' and owner in self._owner_manifest.required:
                    self.create_subscription(
                        String, topic,
                        lambda message, *, role=owner: self._on_close_owner(role, message), qos)
        if self._shadow_runtime is not None:
            from std_srvs.srv import Trigger
            self.create_service(Trigger, '/fire_mission/recover_shadow_session', self._recover_shadow)
            self.create_service(Trigger, '/fire_mission/reset_shadow_encounter', self._reset_shadow)
        self.create_timer(1.0 / self.publish_rate, self._publish_state)
        self.get_logger().warning(
            '[MISSION] OBSERVE-ONLY: no Twist, Nav2 action or actuator '
            'publisher exists in this node'
        )
        self.get_logger().info(
            f'[MISSION] telemetry -> {self.state_topic} at '
            f'{self.publish_rate:.1f} Hz'
        )

    def _declare_parameters(self):
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter(
            'alignment_topic', '/yolo/alignment_preview')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('nav_command_topic', '/cmd_vel_nav')
        self.declare_parameter('raw_command_topic', '/cmd_vel_raw')
        self.declare_parameter(
            'nav_status_topic', '/navigate_to_pose/_action/status')
        self.declare_parameter(
            'post_spray_review_topic', '/fire_mission/post_spray_review')
        self.declare_parameter(
            'actuator_state_topic', '/fire_sensors/actuator_state')
        self.declare_parameter(
            'ir_range_topic', '/fire_mission/ir_range_preview')
        self.declare_parameter(
            'pump_state_topic', '/fire_mission/pump_demo_state')
        self.declare_parameter('state_topic', '/fire_mission/state')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('detection_timeout_s', 0.50)
        self.declare_parameter('alignment_timeout_s', 0.50)
        self.declare_parameter('odom_timeout_s', 0.50)
        self.declare_parameter('command_timeout_s', 0.50)
        self.declare_parameter('fire_acquire_hold_s', 0.50)
        self.declare_parameter('fire_reacquire_hold_s', 0.20)
        self.declare_parameter('centered_hold_s', 0.50)
        self.declare_parameter('max_source_age_ms', 300.0)
        self.declare_parameter('max_future_skew_ms', 20.0)
        self.declare_parameter('stationary_linear_speed_m_s', 0.02)
        self.declare_parameter(
            'stationary_angular_speed_rad_s', 0.05)
        self.declare_parameter(
            'turn_comparison_epsilon_rad_s', 0.03)
        self.declare_parameter('review_timeout_s', 0.6)
        self.declare_parameter('actuator_timeout_s', 0.35)
        self.declare_parameter('pre_spray_clear_hold_s', 1.5)
        self.declare_parameter('enable_fire_session_observer', False)
        self.declare_parameter('enable_close_owner_collection', False)
        self.declare_parameter('close_owner_manifest', '')
        self.declare_parameter('enable_global_stop_collection', False)
        self.declare_parameter('enable_shadow_session_cleanup', False)
        self.declare_parameter('shadow_budget_file', '')
        self.declare_parameter('initialize_shadow_budget', False)
        self.declare_parameter('require_through_status', False)

    def _read_parameters(self):
        topic_names = [
            'detections_topic',
            'alignment_topic',
            'odom_topic',
            'nav_command_topic',
            'raw_command_topic',
            'nav_status_topic',
            'post_spray_review_topic',
            'actuator_state_topic',
            'ir_range_topic',
            'pump_state_topic',
            'state_topic',
        ]
        for name in topic_names:
            setattr(self, name, str(self.get_parameter(name).value))
            if not getattr(self, name):
                raise ValueError(f'{name} cannot be empty')
        if self.state_topic in FORBIDDEN_OUTPUT_TOPICS:
            raise ValueError(
                f'state_topic cannot be a command topic: {self.state_topic}')
        self.publish_rate = float(
            self.get_parameter('publish_rate_hz').value)
        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self._core_parameters = {
            name: float(self.get_parameter(name).value)
            for name in (
                'detection_timeout_s',
                'alignment_timeout_s',
                'odom_timeout_s',
                'command_timeout_s',
                'fire_acquire_hold_s',
                'fire_reacquire_hold_s',
                'centered_hold_s',
                'max_source_age_ms',
                'max_future_skew_ms',
                'stationary_linear_speed_m_s',
                'stationary_angular_speed_rad_s',
                'turn_comparison_epsilon_rad_s',
                'review_timeout_s',
                'actuator_timeout_s',
                'pre_spray_clear_hold_s',
            )
        }
        self._core_parameters['enable_fire_session_observer'] = (
            self.get_parameter('enable_fire_session_observer').value)
        self.enable_close_owner_collection = self.get_parameter(
            'enable_close_owner_collection').value
        self.enable_shadow_session_cleanup = self.get_parameter('enable_shadow_session_cleanup').value
        if type(self.enable_shadow_session_cleanup) is not bool:
            raise ValueError('enable_shadow_session_cleanup must be a bool')
        self._owner_manifest = runtime_manifest(CLOSE_OWNER_MANIFEST)
        if type(self.enable_close_owner_collection) is not bool:
            raise ValueError('enable_close_owner_collection must be a bool')
        if self.enable_close_owner_collection:
            if self._core_parameters['enable_fire_session_observer'] is not True:
                raise ValueError('close owner collection requires the fire session observer')
            manifest_name = self.get_parameter('close_owner_manifest').value
            if self.enable_shadow_session_cleanup:
                self._owner_manifest = runtime_manifest(manifest_name)
            elif manifest_name != CLOSE_OWNER_MANIFEST:
                raise ValueError('close owner collection requires all_four_external_v1 manifest')
            if (self.state_topic != '/fire_mission/state' or
                    self.pump_state_topic != CLOSE_OWNER_TOPICS['pump_demo_controller']):
                raise ValueError('close owner collection requires canonical mission/pump topics')
        self.enable_global_stop_collection = self.get_parameter('enable_global_stop_collection').value
        if type(self.enable_global_stop_collection) is not bool:
            raise ValueError('enable_global_stop_collection must be a bool')
        if self.enable_global_stop_collection:
            if not self.enable_close_owner_collection:
                raise ValueError('global stop collection requires owner collection')
            expected = {'odom_topic': '/odom', 'raw_command_topic': '/cmd_vel_raw',
                        'nav_status_topic': '/navigate_to_pose/_action/status',
                        'actuator_state_topic': '/fire_sensors/actuator_state'}
            if any(getattr(self, name) != value for name, value in expected.items()):
                raise ValueError('global stop collection requires canonical input topics')
            if self.get_parameter('use_sim_time').value is not False:
                raise ValueError('DDS timestamps require system ROS time in global stop mode')
        if self.enable_shadow_session_cleanup and not self.enable_global_stop_collection:
            raise ValueError('shadow cleanup requires global stop collection')
        self.require_through_status = self.get_parameter('require_through_status').value
        if type(self.require_through_status) is not bool:
            raise ValueError('require_through_status must be a bool')
        if self.require_through_status and not self.enable_shadow_session_cleanup:
            raise ValueError('through-status proof requires shadow cleanup')

    @staticmethod
    def _now():
        return time.monotonic()

    def _decode_json(self, message, mark_error, update):
        now = self._now()
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('JSON root is not an object')
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            mark_error(str(exc), now)
            return
        update(payload, now)

    def _on_detection(self, message):
        self._decode_json(
            message,
            self._core.mark_detection_error,
            self._store_detection,
        )

    def _store_detection(self, payload, received):
        """Admit stamped ok observations; preserve explicit health faults."""
        if payload.get('state') != 'ok':
            self._core.update_detection(payload, received)
            return
        ros_now_ns = self.get_clock().now().nanoseconds
        checked_received = self._now()
        admitted = self._detection_stamp_guard.admit(payload, ros_now_ns)
        if admitted is None:
            reason = self._detection_stamp_guard.reason
            self._core.mark_detection_error(reason, received, blocker=reason)
            return
        barrier = getattr(self, '_shadow_input_barrier_ns', None)
        if barrier is not None:
            # Source age includes camera/YOLO transit. Frames queued BEFORE
            # cleanup cannot become a candidate in the next session.
            effective_ns = min(self._detection_stamp_guard.last_stamp_ns,
                               ros_now_ns - math.ceil(max(0.0, admitted['source']['age_ms']) * 1e6))
            if effective_ns <= barrier + round(self._core.max_future_skew * 1e6):
                self._core.mark_detection_error('old session frame', received,
                                                blocker='DETECTION_BEFORE_SESSION_CLEANUP')
                return
        self._core.update_detection(admitted, checked_received)

    def _on_alignment(self, message):
        self._decode_json(
            message,
            self._core.mark_alignment_error,
            self._core.update_alignment,
        )

    def _on_odom(self, message):
        if getattr(self, '_global_stop', None) is not None:
            self._observe_stop_odom(message)
        received = admit_odom(self, message, self._core.odom_timeout)
        if received is None:
            return
        self._core.update_odom(
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
            received,
        )

    def _on_nav_command(self, message):
        self._core.update_nav_command(
            message.linear.x, message.angular.z, self._now())

    def _on_raw_command(self, message):
        self._core.update_raw_command(
            message.linear.x, message.angular.z, self._now())

    def _on_nav_status(self, message, metadata=None):
        records = [(bytes(item.goal_info.goal_id.uuid).hex(), item.status)
                   for item in message.status_list]
        if getattr(self, '_global_stop', None) is not None:
            source_id = self._stop_source_id('nav_status')
            ros_ns, received = _stop_clock_pair(self)
            self._global_stop.observe_status(
                records, stamp_ns=self._stop_metadata_stamp(metadata),
                source_id=source_id, ros_ns=ros_ns, now=received)
            if getattr(self, '_shadow_runtime', None) is not None:
                sample = self._global_stop.samples['nav_status']
                if sample is None:
                    self._core._goal_status_received = None
                    self._core._goal_status_records, self._core._goal_statuses = [], []
                else:
                    self._core.update_goal_statuses(records, sample.effective_at)
                return
        self._core.update_goal_statuses(records, self._now())

    def _single_expected_publisher(self, topic, node_name):
        infos = self.get_publishers_info_by_topic(topic)
        return (len(infos) == 1 and
                infos[0].node_namespace == '/' and
                infos[0].node_name == node_name and
                infos[0].topic_type == 'std_msgs/msg/String' and
                infos[0].qos_profile.reliability ==
                ReliabilityPolicy.BEST_EFFORT)

    def _on_through_nav_status(self, message, metadata=None):
        records = [(bytes(item.goal_info.goal_id.uuid).hex(), item.status)
                   for item in message.status_list]
        source_id = self._stop_source_id('nav_through_status')
        ros_ns, received = _stop_clock_pair(self)
        self._global_stop.observe_status(
            records, stamp_ns=self._stop_metadata_stamp(metadata), source_id=source_id,
            ros_ns=ros_ns, now=received, role='nav_through_status')

    def _on_review(self, message):
        if not self._single_expected_publisher(
                self.post_spray_review_topic, 'post_spray_review'):
            self._core.update_review(None, self._now())
            return
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self._core.update_review(payload, self._now())

    def _on_actuator(self, message):
        global_stop = getattr(self, '_global_stop', None)
        source_id = self._stop_source_id('actuator') if global_stop is not None else None
        source_ok = (source_id is not None if global_stop is not None else
                     self._single_expected_publisher(self.actuator_state_topic, 'serial_bridge_node'))
        if not source_ok:
            if global_stop is not None:
                ros_ns, received = _stop_clock_pair(self)
                global_stop.observe_actuator(None, source_id=None, ros_ns=ros_ns, now=received)
            self._core.update_actuator(
                None, self._now(), blocker='ACTUATOR_PUBLISHER_INVALID')
            return
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        ros_now_ns, received = _stop_clock_pair(self)
        if global_stop is not None:
            global_stop.observe_actuator(payload, source_id=source_id, ros_ns=ros_now_ns, now=received)
        age_ms = self._actuator_stamp_guard.admit(payload, ros_now_ns)
        if age_ms is None:
            self._core.update_actuator(
                None, received, blocker=self._actuator_stamp_guard.reason)
            return
        if self._actuator_stamp_guard.context_changed:
            self._core.update_actuator(
                None, received, blocker='ACTUATOR_SOURCE_CONTEXT_CHANGED')
        self._core.update_actuator(payload, received, source_age_ms=age_ms)

    def _on_ir_range(self, message):
        if not self._single_expected_publisher(
                self.ir_range_topic, 'ir_range_preview'):
            self._core.update_ir(None, self._now())
            return
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self._core.update_ir(payload, self._now())

    def _on_pump_state(self, message):
        source_ok = self._single_expected_publisher(
            self.pump_state_topic, 'pump_demo_controller')
        if not source_ok:
            self._collect_close_owner('pump_demo_controller', None, source_ok=False)
            return
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self._collect_close_owner('pump_demo_controller', payload, source_ok=source_ok)
        self._core.update_pump(payload, self._now())

    def _on_close_owner(self, owner, message):
        """Selected mode only: fixed endpoint, one decode, latest sample only."""
        if getattr(self, '_close_owner_collector', None) is None:
            return
        source_ok = self._single_expected_publisher(CLOSE_OWNER_TOPICS[owner], owner)
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self._collect_close_owner(owner, payload, source_ok=source_ok)

    def _collect_close_owner(self, owner, payload, *, source_ok):
        collector = getattr(self, '_close_owner_collector', None)
        if collector is None:
            return
        report = payload.get('close_owner_observation') if isinstance(payload, dict) else None
        # Read clocks after graph/decode work; never renew an old source stamp.
        ros_now_ns, received = _stop_clock_pair(self)
        runtime = getattr(self, '_shadow_runtime', None)
        if runtime is not None:
            if owner in runtime.owners:
                runtime.owners[owner] = None
            if not isinstance(report, dict) or report.get('live_enabled') is not False:
                runtime.recovery_latch = 'SHADOW_OWNER_LIVE_OR_UNDECLARED:' + owner
                collector.observe(owner, None, ros_now_ns=ros_now_ns,
                                  received_at=received, source_ok=False)
                return
        admitted = collector.observe(owner, report, ros_now_ns=ros_now_ns,
                                     received_at=received, source_ok=source_ok)
        if admitted and runtime is not None:
            sample = collector._samples[owner]
            runtime.observe_owner(owner, report, sample.received_at - sample.source_age_s)

    @staticmethod
    def _stop_metadata_stamp(metadata):
        stamp = metadata.get('source_timestamp') if isinstance(metadata, dict) else None
        return stamp if type(stamp) is int and stamp > 0 else None

    def _stop_source_id(self, role):
        if role in CLOSE_OWNER_TOPICS:
            topic, node, kind, status = CLOSE_OWNER_TOPICS[role], role, 'std_msgs/msg/String', False
        else:
            topic, node, kind, status = (THROUGH_STOP_INPUT if role == 'nav_through_status'
                                         else STOP_INPUTS[role])
        infos = self.get_publishers_info_by_topic(topic)
        if len(infos) != 1:
            return None
        info = infos[0]
        reliability = ReliabilityPolicy.RELIABLE if status else ReliabilityPolicy.BEST_EFFORT
        durability = DurabilityPolicy.TRANSIENT_LOCAL if status else DurabilityPolicy.VOLATILE
        if (info.node_name != node or info.node_namespace != '/' or info.topic_type != kind or
                info.qos_profile.reliability != reliability or info.qos_profile.durability != durability):
            return None
        gid = getattr(info, 'endpoint_gid', None)
        if (type(gid) not in (list, tuple, bytes) or len(gid) not in (16, 24) or
                any(type(byte) is not int or not 0 <= byte <= 255 for byte in gid) or not any(gid)):
            return None
        return bytes(gid)

    def _observe_stop_odom(self, message):
        source_id = self._stop_source_id('odom')
        stamp = message.header.stamp
        stamp_ns = (stamp.sec * 1_000_000_000 + stamp.nanosec
                    if type(stamp.sec) is int and stamp.sec >= 0 and type(stamp.nanosec) is int
                    and 0 <= stamp.nanosec < 1_000_000_000 else None)
        values = (_full_twist(message.twist.twist)
                  if message.header.frame_id == 'odom' and message.child_frame_id == 'base_link' else None)
        ros_ns, received = _stop_clock_pair(self)
        self._global_stop.observe_motion('odom', values, stamp_ns=stamp_ns,
                                         source_id=source_id, ros_ns=ros_ns, now=received)

    def _on_stop_cmd(self, message, metadata=None):
        if getattr(self, '_global_stop', None) is None:
            return
        source_id = self._stop_source_id('cmd')
        ros_ns, received = _stop_clock_pair(self)
        self._global_stop.observe_motion(
            'cmd', _full_twist(message), stamp_ns=self._stop_metadata_stamp(metadata),
            source_id=source_id, ros_ns=ros_ns, now=received)

    def _publish_state(self):
        session = self._core._fire_session
        # Timestamp before graph queries/evaluation so their cost is included
        # in the age seen by consumers, instead of re-dating an old snapshot.
        global_stop = getattr(self, '_global_stop', None)
        if global_stop is not None:
            stamp_ns, graph_started_at = _stop_clock_pair(self)
        else:
            stamp_ns = (self.get_clock().now().nanoseconds
                        if session is not None else None)
        runtime = getattr(self, '_shadow_runtime', None)
        if runtime is not None:
            runtime.recovery_latch = (runtime.recovery_latch or global_stop.fault or
                                      self._close_owner_collector.fault)
            if global_stop.require_through_status:
                admission_ros, admission_at = _stop_clock_pair(self)
                runtime.navigation_blocker = global_stop.through_navigation_blocker(
                    ros_ns=admission_ros, now=admission_at)
        pump_count = self.count_publishers('/pump_cmd')
        self._core.update_pump_owner_count(pump_count)
        payload = self._core.evaluate(self._now())
        if session is not None:
            observer = payload['lifecycle']['fire_session_observer']
            if type(stamp_ns) is int and stamp_ns > 0:
                self._session_source_sequence += 1
                sec, nanosec = divmod(stamp_ns, 1_000_000_000)
                observer['source'] = {
                    'instance_id': session.source_instance_id,
                    'sequence': self._session_source_sequence,
                    'stamp': {'sec': sec, 'nanosec': nanosec},
                }
                observer['source_blocker'] = None
            else:
                observer['source'] = None
                observer['source_blocker'] = 'SESSION_CLOCK_INVALID'
        collector = getattr(self, '_close_owner_collector', None)
        if collector is not None:
            if (collector.request is not None and
                    session.close_request != collector.request):
                # A changed/lost pending round must block this same payload,
                # rather than briefly attaching old readiness to a new close.
                collector.begin_close(session.close_request,
                                      announced_ros_ns=collector.announced_ros_ns,
                                      announced_at=collector.announced_at)
            if (global_stop is not None and global_stop.request is not None and
                    global_stop.request != session.close_request):
                global_stop.begin_close(session.close_request,
                                        announced_ros_ns=global_stop.barrier_ros_ns,
                                        announced_at=global_stop.barrier_at)
            sources = None
            if global_stop is not None and global_stop.request is not None:
                sources = {role: self._stop_source_id(role) for role in global_stop.source_roles}
                raw_count = self.count_publishers('/cmd_vel_raw')
                omitted_present = None
                if global_stop.manifest.omitted:
                    nodes = self.get_node_names_and_namespaces()
                    omitted_present = any(name in global_stop.manifest.omitted for name, _ in nodes)
                    for owner in global_stop.manifest.omitted:
                        # A different node publishing the omitted role's state
                        # also invalidates the chosen deployment manifest.
                        omitted_present |= bool(self.get_publishers_info_by_topic(CLOSE_OWNER_TOPICS[owner]))
            ros_now_ns, assessed_at = _stop_clock_pair(self)
            if global_stop is not None and sources is not None:
                global_stop.observe_graph(
                    raw_count=raw_count, pump_count=pump_count, sources=sources,
                    started_ros_ns=stamp_ns, started_at=graph_started_at,
                    ros_ns=ros_now_ns, now=assessed_at, omitted_present=omitted_present)
            absence_at = (global_stop.graph[1] if global_stop is not None and
                          global_stop.graph is not None else None)
            result = collector.assess(ros_now_ns=ros_now_ns, now=assessed_at,
                                      absence_observed_at=absence_at)
            payload['lifecycle']['close_owner_collection'] = {
                'mode': 'OBSERVE_ONLY', 'manifest': collector.manifest.name,
                'required_owners': list(collector.manifest.required),
                'session_id': result.session_id,
                'nav_goal_uuid': result.nav_goal_uuid,
                'close_generation': result.close_generation,
                'owner_reports_ready': result.ready,
                'blockers': list(result.blockers),
                'cleanup_authorized': False,
                'actuation_authorized': False,
            }
            if global_stop is not None:
                self._global_stop_result = global_stop.assess(result, ros_ns=ros_now_ns, now=assessed_at)
                evidence = self._global_stop_result.evidence
                payload['lifecycle']['global_stop_collection'] = {
                    'mode': 'OBSERVE_ONLY', 'stop_evidence_ready': self._global_stop_result.ready,
                    'session_id': None if global_stop.request is None else global_stop.request.session_id,
                    'nav_goal_uuid': None if global_stop.request is None else global_stop.request.nav_goal_uuid,
                    'close_generation': None if global_stop.request is None else global_stop.request.generation,
                    'nav_terminal_status': None if evidence is None else evidence.nav_terminal_status,
                    'blockers': list(self._global_stop_result.blockers),
                    'cleanup_authorized': False, 'actuation_authorized': False,
                    'physical_motion_or_actuator_verified': False,
                }
        if runtime is not None:
            runtime.recovery_latch = runtime.recovery_latch or global_stop.fault or collector.fault
            payload['lifecycle']['shadow_session_runtime'] = runtime.snapshot()
            payload['lifecycle']['shadow_session_runtime']['clock_sampling'] = {
                **global_stop.clock_sampling,
                'first_fault': global_stop.clock_fault_evidence}
        message = String()
        message.data = json.dumps(
            payload, separators=(',', ':'), ensure_ascii=True)
        self._state_publisher.publish(message)
        # Only a successfully published STOPPING heartbeat announces a round.
        # Take its barrier AFTER publish; retries must not re-date the round.
        if (collector is not None and session.close_request is not None and
                collector.request is None):
            announced_ros_ns, announced_at = _stop_clock_pair(self)
            collector.begin_close(session.close_request,
                                  announced_ros_ns=announced_ros_ns,
                                  announced_at=announced_at)
            if global_stop is not None and collector.request is not None:
                global_stop.begin_close(collector.request,
                                        announced_ros_ns=collector.announced_ros_ns,
                                        announced_at=collector.announced_at)
        if runtime is not None and collector.request is not None:
            # Re-evaluate against the time AFTER publication. No new graph
            # query and no re-dating of cached observations is permitted.
            ros_ns, now = _stop_clock_pair(self)
            owners = collector.assess(
                ros_now_ns=ros_ns, now=now,
                absence_observed_at=None if global_stop.graph is None else global_stop.graph[1])
            result = global_stop.assess(owners, ros_ns=ros_ns, now=now)
            finished = runtime.finish_after_publish(
                self._core, result, collector, global_stop,
                now=now, clock=self._now, recheck=self._shadow_current_stop_result)
            if finished:
                self._shadow_input_barrier_ns = self.get_clock().now().nanoseconds

    def _shadow_current_stop_result(self):
        global_stop, collector = self._global_stop, self._close_owner_collector
        ros_ns, now = _stop_clock_pair(self)
        owners = collector.assess(
            ros_now_ns=ros_ns, now=now,
            absence_observed_at=None if global_stop.graph is None else global_stop.graph[1])
        return global_stop.assess(owners, ros_ns=ros_ns, now=now)

    def _recover_shadow(self, request, response):
        return self._shadow_operator_request(response, reset=False)

    def _reset_shadow(self, request, response):
        return self._shadow_operator_request(response, reset=True)

    def _shadow_operator_request(self, response, *, reset):
        runtime = self._shadow_runtime
        try:
            runtime.operator_request(self._core._fire_session, self._now(), reset_encounter=reset,
                                     idle_goal=self._core._active_goal_uuid())
        except ValueError as error:
            response.success, response.message = False, str(error)
            return response
        # Only this explicit operator action may establish new source pins.
        old = self._global_stop
        self._close_owner_collector = CloseOwnerEvidenceCollector(
            timeout_s=old.timeout_s, future_skew_ms=old.future_ns / 1e6,
            manifest_id=runtime.manifest.name)
        self._global_stop = GlobalStopEvidence(
            timeout_s=old.timeout_s, future_skew_ms=old.future_ns / 1e6,
            linear_limit=old.linear_limit, angular_limit=old.angular_limit,
            manifest_id=runtime.manifest.name, require_recovery_on_change=True,
            require_through_status=old.require_through_status)
        self._global_stop_result = None
        response.success = True
        response.message = 'New shadow close proof required; no arm or quota reset'
        return response

    def destroy_node(self):
        runtime = getattr(self, '_shadow_runtime', None)
        try:
            return super().destroy_node()
        finally:
            if runtime is not None:
                runtime.store.close()


def main(args=None):
    """Run the observe-only mission supervisor."""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = FireMissionSupervisor()
        if node.enable_global_stop_collection:
            from fire_robot_mission.stop_metadata_executor import StopMetadataExecutor
            executor = StopMetadataExecutor()
            executor.add_node(node)
            executor.spin()
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
