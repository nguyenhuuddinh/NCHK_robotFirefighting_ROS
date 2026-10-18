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
            pre_spray_clear_hold_s=1.5):
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

        self._detection = None
        self._detection_received = None
        self._detection_error = ''
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

    def mark_detection_error(self, error, now):
        """Record malformed detection telemetry as a fail-safe input."""
        self._detection = None
        self._detection_received = float(now)
        self._detection_error = str(error)

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

    def update_actuator(self, payload, now):
        """Store Pi bridge telemetry; never command the actuator."""
        self._actuator = payload
        self._actuator_received = float(now)

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
        actuator_age = _age_seconds(self._actuator_received, now)
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
        if _age_seconds(self._actuator_received, now) is None or \
                _age_seconds(self._actuator_received, now) > \
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
            blockers.append('DETECTION_INVALID_JSON')
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
            source_age = float(source_age)
            if source_age > self.max_source_age:
                blockers.append('SOURCE_TIMESTAMP_STALE')
            elif source_age < -self.max_future_skew:
                blockers.append('SOURCE_TIMESTAMP_IN_FUTURE')
            elif source_age < 0.0:
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
        if not target_present and not hard_blockers and \
                (self._reviewed_clear(now, nav_active) or pre_spray_clear):
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
        return {
            'schema_version': 1,
            'observe_only': True,
            'state': state,
            'lifecycle': {
                'pre_spray_clear_goal_uuid': self._pre_spray_clear_uuid,
            },
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
        self.create_subscription(
            GoalStatusArray,
            self.nav_status_topic,
            self._on_nav_status,
            action_status_qos(),
        )
        self.create_subscription(
            String, self.post_spray_review_topic, self._on_review, qos)
        self.create_subscription(
            String, self.actuator_state_topic, self._on_actuator, qos)
        self.create_subscription(
            String, self.ir_range_topic, self._on_ir_range, qos)
        self.create_subscription(
            String, self.pump_state_topic, self._on_pump_state, qos)
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
            self._core.update_detection,
        )

    def _on_alignment(self, message):
        self._decode_json(
            message,
            self._core.mark_alignment_error,
            self._core.update_alignment,
        )

    def _on_odom(self, message):
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

    def _on_nav_status(self, message):
        self._core.update_goal_statuses(
            [(bytes(item.goal_info.goal_id.uuid).hex(), item.status)
             for item in message.status_list], self._now())

    def _single_expected_publisher(self, topic, node_name):
        infos = self.get_publishers_info_by_topic(topic)
        return (len(infos) == 1 and
                infos[0].node_namespace == '/' and
                infos[0].node_name == node_name and
                infos[0].topic_type == 'std_msgs/msg/String' and
                infos[0].qos_profile.reliability ==
                ReliabilityPolicy.BEST_EFFORT)

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
        if not self._single_expected_publisher(
                self.actuator_state_topic, 'serial_bridge_node'):
            self._core.update_actuator(None, self._now())
            return
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self._core.update_actuator(payload, self._now())

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
        if not self._single_expected_publisher(
                self.pump_state_topic, 'pump_demo_controller'):
            return
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self._core.update_pump(payload, self._now())

    def _publish_state(self):
        self._core.update_pump_owner_count(
            self.count_publishers('/pump_cmd'))
        payload = self._core.evaluate(self._now())
        message = String()
        message.data = json.dumps(
            payload, separators=(',', ':'), ensure_ascii=True)
        self._state_publisher.publish(message)


def main(args=None):
    """Run the observe-only mission supervisor."""
    rclpy.init(args=args)
    node = None
    try:
        node = FireMissionSupervisor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
