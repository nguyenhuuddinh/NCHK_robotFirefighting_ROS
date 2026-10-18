"""Opt-in, one-shot cancellation of a specific Nav2 goal."""

from collections import Counter
import json
import math
import time

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from fire_robot_safety.odom_freshness import OdomStampGuard, admit_odom
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool, String

from fire_robot_mission.alignment_handoff_shadow import arm_qos
from fire_robot_mission.mission_supervisor import action_status_qos, sensor_qos


ACTIVE_STATUSES = {1, 2, 3}
FORBIDDEN_OUTPUTS = {
    '/cmd_vel', '/cmd_vel_raw', '/cmd_vel_nav', '/pump_cmd', '/fire_target',
}
CANONICAL_ENDPOINTS = {
    'intent_topic': '/fire_mission/nav2_cancel_intent_shadow',
    'status_topic': '/navigate_to_pose/_action/status',
    'cancel_service': '/navigate_to_pose/_action/cancel_goal',
    'arm_topic': '/fire_mission/nav_cancel_arm',
    'odom_topic': '/odom',
    'cmd_topic': '/cmd_vel',
    'raw_cmd_topic': '/cmd_vel_raw',
    'shadow_nav_cmd_topic': '/fire_mission/nav2_cmd_vel_shadow',
    'alignment_state_topic': '/fire_mission/live_alignment_state',
    'ir_approach_state_topic': '/fire_mission/ir_approach_live_state',
    'mission_state_topic': '/fire_mission/state',
    'state_topic': '/fire_mission/nav2_cancel_guard_state',
}
ACTIVE_FIRE_STATES = {
    'FIRE_TRACKING': {'LEFT_PREVIEW', 'RIGHT_PREVIEW'},
    'CENTERED_HOLD': {'HOLD_CENTER_PREVIEW'},
    'ALIGNMENT_READY': {'HOLD_CENTER_PREVIEW'},
}
EXPECTED_SHADOW_PUBLISHERS = Counter({
    ('/', 'velocity_smoother', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.RELIABLE): 1,
    ('/', 'behavior_server', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.RELIABLE): 2,
})
ALIGNMENT_RAW_ENDPOINT = Counter({
    ('/', 'live_alignment_takeover', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.BEST_EFFORT): 1,
})
IR_APPROACH_RAW_ENDPOINT = Counter({
    ('/', 'ir_approach_live', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.BEST_EFFORT): 1,
})


def _age(received, now):
    return None if received is None else max(0.0, now - received)


def _valid_uuid(value):
    return isinstance(value, str) and len(value) == 32 and \
        value != '0' * 32 and all(
            character in '0123456789abcdef' for character in value.lower())


def _zero(values, linear_limit, angular_limit):
    return values is not None and len(values) == 6 and \
        all(math.isfinite(v) for v in values) and \
        all(abs(v) <= linear_limit for v in values[:3]) and \
        all(abs(v) <= angular_limit for v in values[3:])


def make_cancel_request(uuid_hex):
    """Use nonzero UUID and zero stamp: cancel exactly this goal, never all."""
    if not _valid_uuid(uuid_hex):
        raise ValueError('a nonzero 16-byte goal UUID is required')
    request = CancelGoal.Request()
    request.goal_info.goal_id.uuid = list(bytes.fromhex(uuid_hex))
    request.goal_info.stamp.sec = 0
    request.goal_info.stamp.nanosec = 0
    return request


class Nav2CancelGuardCore:
    """One-shot admission and post-cancel stationary verification."""

    def __init__(
            self, enabled=False, intent_timeout_s=0.35,
            arm_timeout_s=0.40, telemetry_timeout_s=0.35,
            response_timeout_s=2.0, cancel_timeout_s=5.0,
            zero_hold_s=0.50, stationary_linear_m_s=0.02,
            stationary_angular_rad_s=0.05,
            command_zero_epsilon=0.001):
        values = (intent_timeout_s, arm_timeout_s, telemetry_timeout_s,
                  response_timeout_s, cancel_timeout_s, zero_hold_s,
                  stationary_linear_m_s, stationary_angular_rad_s,
                  command_zero_epsilon)
        if any(not math.isfinite(float(v)) or float(v) <= 0 for v in values):
            raise ValueError('cancel guard thresholds must be positive')
        self.enabled = bool(enabled)
        self.intent_timeout = float(intent_timeout_s)
        self.arm_timeout = float(arm_timeout_s)
        self.telemetry_timeout = float(telemetry_timeout_s)
        self.response_timeout = float(response_timeout_s)
        self.cancel_timeout = float(cancel_timeout_s)
        self.zero_hold = float(zero_hold_s)
        self.stationary_linear = float(stationary_linear_m_s)
        self.stationary_angular = float(stationary_angular_rad_s)
        self.command_zero_epsilon = float(command_zero_epsilon)
        self.intent = None
        self.intent_received = None
        self.arm = False
        self.arm_received = None
        self.statuses = None
        self.status_received = None
        self.status_publishers = 0
        self.shadow_publishers = 0
        self.arm_publishers = 0
        self.nav_shadow_sources = ()
        self.raw_publishers = 0
        self.raw_sources = ()
        self.cmd_publishers = 0
        self.guard_publishers = 0
        self.service_ready = False
        self.cmd = None
        self.cmd_received = None
        self.odom = None
        self.odom_received = None
        self.goal_uuid = None
        self.sent_at = None
        self.accepted_at = None
        self.canceled_at = None
        self.zero_since = None
        self.failure = None
        self.complete = False
        self.send_count = 0
        self.failure_evidence = None
        self.used_goal_uuids = set()
        self.alignment_started_at = None
        self.alignment = None
        self.alignment_received = None
        self.post_alignment_zero_since = None
        self.ir_approach = None
        self.ir_approach_received = None
        self.ir_started_at = None
        self.ir_release_seen_at = None
        self.post_ir_zero_since = None
        self.ir_state_source_ok = False
        self.pump_publishers = 0
        self.mission = None
        self.mission_received = None
        self.mission_source_ok = False

    def update_intent(self, payload, now):
        self.intent = payload
        self.intent_received = float(now)

    def update_arm(self, armed, now):
        self.arm = armed is True
        self.arm_received = float(now)

    def update_statuses(self, statuses, now):
        self.statuses = [(str(uid).lower(), int(status))
                         for uid, status in statuses]
        self.status_received = float(now)

    def update_topology(
            self, status, shadow, arm, nav_shadow_sources, raw, cmd, guard,
            service_ready, raw_sources=(), ir_state_source_ok=False,
            pump_publishers=0):
        status = int(status)
        if status != 1:
            self.statuses = None
            self.status_received = None
        self.status_publishers = status
        self.shadow_publishers = int(shadow)
        self.arm_publishers = int(arm)
        self.nav_shadow_sources = tuple(nav_shadow_sources)
        self.raw_publishers = int(raw)
        self.raw_sources = tuple(raw_sources)
        self.ir_state_source_ok = bool(ir_state_source_ok)
        self.pump_publishers = int(pump_publishers)
        self.cmd_publishers = int(cmd)
        self.guard_publishers = int(guard)
        self.service_ready = bool(service_ready)

    def update_alignment(self, payload, now):
        self.alignment = payload
        self.alignment_received = float(now)

    def update_ir_approach(self, payload, now):
        self.ir_approach = payload
        self.ir_approach_received = float(now)

    def update_mission(self, payload, now, source_ok=True):
        self.mission = payload
        self.mission_received = float(now)
        self.mission_source_ok = source_ok is True

    def _pre_spray_clear_proven(self, now):
        mission = self.mission if isinstance(self.mission, dict) else {}
        lifecycle = mission.get('lifecycle')
        perception = mission.get('perception') or {}
        active = self._active_goals()
        # The operator may submit the next goal before this node consumes the
        # clear proof. It is safe to retire the old UUID while the new goal is
        # still stationary; never treat the old goal as active again.
        next_goal_only = (len(active) <= 1 and all(
            _valid_uuid(uid) and uid != self.goal_uuid and
            uid not in self.used_goal_uuids for uid in active))
        return (self.mission_source_ok and
                _age(self.mission_received, now) is not None and
                _age(self.mission_received, now) <= self.telemetry_timeout and
                mission.get('schema_version') == 1 and
                mission.get('state') in ('IDLE', 'NAV2_NAVIGATING') and
                perception.get('detected_now') is False and
                perception.get('confirmed') is False and
                isinstance(lifecycle, dict) and
                lifecycle.get('pre_spray_clear_goal_uuid') ==
                self.goal_uuid and
                self.complete and self.canceled_at is not None and
                self.raw_publishers == 0 and self.pump_publishers == 0 and
                next_goal_only and
                self.status_publishers == 1 and
                _age(self.status_received, now) is not None and
                _age(self.status_received, now) <= self.telemetry_timeout and
                all(status == 5 for uid, status in self.statuses
                    if uid == self.goal_uuid) and
                self._cmd_zero(now) and self._odom_stationary(now))

    def _clear_cycle(self):
        self.used_goal_uuids.add(self.goal_uuid)
        self.goal_uuid = None
        self.sent_at = None
        self.accepted_at = None
        self.canceled_at = None
        self.zero_since = None
        self.complete = False
        self.failure = None
        self.send_count = 0
        self.failure_evidence = None
        self.alignment = None
        self.alignment_received = None
        self.alignment_started_at = None
        self.post_alignment_zero_since = None
        self.ir_approach = None
        self.ir_approach_received = None
        self.ir_started_at = None
        self.ir_release_seen_at = None
        self.post_ir_zero_since = None
        self.intent = None
        self.intent_received = None

    def _alignment_owner(self):
        return self.raw_publishers == 1 and \
            Counter(self.raw_sources) == ALIGNMENT_RAW_ENDPOINT

    def _ir_approach_owner(self):
        return self.raw_publishers == 1 and \
            Counter(self.raw_sources) == IR_APPROACH_RAW_ENDPOINT

    def _ir_approach_report(self, now, *, moving):
        report = self.ir_approach if isinstance(
            self.ir_approach, dict) else {}
        age = _age(self.ir_approach_received, now)
        command = report.get('command') or {}
        continuous = report.get('continuous_approach') is True
        speed_limit = 0.08 if continuous else 0.06
        return (age is not None and age <= self.telemetry_timeout and
                report.get('schema_version') == 1 and
                report.get('enabled') is True and
                report.get('one_shot') is True and
                report.get('integrated_handoff') is True and
                (report.get('continuous_approach') is None or
                 type(report.get('continuous_approach')) is bool) and
                report.get('goal_uuid') == self.goal_uuid and
                report.get('publisher_owned') is moving and
                report.get('direction') == 'FORWARD' and
                (report.get('safety') or {}).get(
                    'pump_command_publisher') is False and
                isinstance(command.get('linear_x_m_s'), (int, float)) and
                0 <= command['linear_x_m_s'] <= speed_limit and
                command.get('angular_z_rad_s') == 0.0 and
                (report.get('state') in
                 ('ACQUIRING', 'MOVING', 'STOPPING') if moving else
                 report.get('state') == 'COMPLETE' and
                 report.get('stop_reason') in (
                     ('SPRAY_RANGE_REACHED', 'REALIGN_REQUIRED',
                      'APPROACH_DISTANCE_LIMIT') if continuous else
                     ('STEP_DISTANCE_REACHED',
                      'STEP_PROGRESS_TIME_LIMIT',
                      'SPRAY_RANGE_REACHED'))))

    def _ir_motion_bounded(self, now):
        cmd_age = _age(self.cmd_received, now)
        odom_age = _age(self.odom_received, now)
        return (self.cmd_publishers == 1 and
                cmd_age is not None and cmd_age <= self.telemetry_timeout and
                odom_age is not None and odom_age <= self.telemetry_timeout and
                self.cmd is not None and self.odom is not None and
                len(self.cmd) == 6 and len(self.odom) == 6 and
                all(math.isfinite(v) for v in self.cmd + self.odom) and
                0 <= self.cmd[0] <= (
                    0.09 if isinstance(self.ir_approach, dict) and
                    self.ir_approach.get('continuous_approach') is True
                    else 0.07) and
                all(abs(v) <= 0.001 for v in self.cmd[1:]) and
                abs(self.odom[0]) <= 0.12 and
                all(abs(v) <= 0.02 for v in self.odom[1:3]) and
                all(abs(v) <= 0.15 for v in self.odom[3:]))

    def _alignment_complete(self, now):
        report = self.alignment if isinstance(self.alignment, dict) else {}
        age = _age(self.alignment_received, now)
        return (age is not None and age <= self.telemetry_timeout and
                report.get('state') == 'COMPLETE' and
                report.get('goal_uuid') == self.goal_uuid and
                report.get('stop_reason') == 'TARGET_CENTERED' and
                report.get('publisher_owned') is False and
                (report.get('command') or {}).get(
                    'angular_z_rad_s') == 0.0)

    def _alignment_active_report(self, now):
        report = self.alignment if isinstance(self.alignment, dict) else {}
        age = _age(self.alignment_received, now)
        command = report.get('command') or {}
        angular = command.get('angular_z_rad_s')
        return (age is not None and age <= self.telemetry_timeout and
                report.get('schema_version') == 1 and
                report.get('enabled') is True and
                report.get('goal_uuid') == self.goal_uuid and
                report.get('state') in ('ACQUIRING', 'TURNING', 'STOPPING') and
                report.get('publisher_owned') is True and
                command.get('linear_x_m_s') == 0.0 and
                isinstance(angular, (int, float)) and
                math.isfinite(angular) and abs(angular) <= 0.60 and
                (report.get('safety') or {}).get(
                    'pump_command_publisher') is False)

    def _alignment_motion_bounded(self, now):
        cmd_age = _age(self.cmd_received, now)
        odom_age = _age(self.odom_received, now)
        return (self.cmd_publishers == 1 and
                cmd_age is not None and cmd_age <= self.telemetry_timeout and
                odom_age is not None and odom_age <= self.telemetry_timeout and
                self.cmd is not None and self.odom is not None and
                len(self.cmd) == 6 and len(self.odom) == 6 and
                all(math.isfinite(v) for v in self.cmd + self.odom) and
                all(abs(v) <= 0.001 for v in self.cmd[:3]) and
                all(abs(v) <= 0.001 for v in self.cmd[3:5]) and
                abs(self.cmd[5]) <= 0.61 and
                all(abs(v) <= 0.02 for v in self.odom[:3]) and
                all(abs(v) <= 0.81 for v in self.odom[3:]))

    def update_cmd(self, values, now):
        self.cmd = tuple(float(value) for value in values)
        self.cmd_received = float(now)

    def update_odom(self, values, now):
        self.odom = tuple(float(value) for value in values)
        self.odom_received = float(now)

    def on_cancel_response(self, response, now):
        """Accept only a response listing exactly the requested goal."""
        if self.goal_uuid is None or self.accepted_at is not None or \
                self.failure:
            return
        if not isinstance(response, CancelGoal.Response):
            self.failure = 'CANCEL_SERVICE_ERROR'
            return
        uuids = [bytes(goal.goal_id.uuid).hex()
                 for goal in response.goals_canceling]
        if response.return_code != CancelGoal.Response.ERROR_NONE or \
                uuids != [self.goal_uuid]:
            self.failure = 'CANCEL_RESPONSE_REJECTED_OR_WRONG_GOAL'
            return
        self.accepted_at = float(now)

    def _arm_fresh(self, now):
        age = _age(self.arm_received, now)
        return self.arm and age is not None and age <= self.arm_timeout

    def _cmd_zero(self, now):
        age = _age(self.cmd_received, now)
        return self.cmd_publishers == 1 and age is not None and \
            age <= self.telemetry_timeout and _zero(
                self.cmd, self.command_zero_epsilon,
                self.command_zero_epsilon)

    def _command_evidence(self, now):
        age = _age(self.cmd_received, now)
        return {
            'publisher_count': self.cmd_publishers,
            'receive_age_ms': None if age is None else round(age * 1000, 1),
            'fresh': age is not None and age <= self.telemetry_timeout,
            'values': list(self.cmd) if self.cmd is not None else None,
            'fresh_zero': self._cmd_zero(now),
        }

    def _odom_stationary(self, now):
        age = _age(self.odom_received, now)
        return age is not None and age <= self.telemetry_timeout and \
            _zero(self.odom, self.stationary_linear,
                  self.stationary_angular)

    def _active_goals(self):
        return [] if self.statuses is None else [
            uid for uid, status in self.statuses
            if status in ACTIVE_STATUSES]

    def _shadow_isolated(self):
        """Require only the three known Nav2 shadow output endpoints."""
        return Counter(self.nav_shadow_sources) == \
            EXPECTED_SHADOW_PUBLISHERS

    def _preflight(self, now):
        blockers = []
        if not self.enabled:
            blockers.append('REAL_CANCEL_DISABLED')
        if not self._arm_fresh(now):
            blockers.append('OPERATOR_ARM_MISSING_OR_STALE')
        if self.status_publishers != 1 or self.statuses is None:
            blockers.append('NAV_STATUS_UNAVAILABLE')
        if self.shadow_publishers != 1:
            blockers.append('CANCEL_INTENT_PUBLISHER_UNAVAILABLE')
        if self.arm_publishers != 1:
            blockers.append('OPERATOR_ARM_PUBLISHER_UNAVAILABLE')
        if self.guard_publishers != 1:
            blockers.append('MULTIPLE_OR_MISSING_CANCEL_GUARDS')
        if self.raw_publishers != 0 or self.cmd_publishers != 1 or \
                not self._shadow_isolated():
            blockers.append('SHADOW_MOTION_ISOLATION_NOT_PROVEN')
        if not self.service_ready:
            blockers.append('NAV_CANCEL_SERVICE_UNAVAILABLE')
        if not self._cmd_zero(now):
            blockers.append('CMD_VEL_NOT_FRESH_ZERO')
        if not self._odom_stationary(now):
            blockers.append('ODOM_NOT_FRESH_STATIONARY')
        age = _age(self.intent_received, now)
        if age is None or age > self.intent_timeout or \
                not isinstance(self.intent, dict):
            blockers.append('CANCEL_INTENT_MISSING_OR_STALE')
            return blockers, None
        intent = self.intent
        safety = intent.get('safety')
        navigation = intent.get('navigation')
        mission = intent.get('mission')
        if not isinstance(safety, dict) or \
                not isinstance(navigation, dict) or \
                not isinstance(mission, dict):
            blockers.append('CANCEL_INTENT_MALFORMED')
            return blockers, None
        mission_state = mission.get('state')
        mission_intent = mission.get('intent')
        active_fire_intents = ACTIVE_FIRE_STATES.get(
            mission_state, set()) if isinstance(mission_state, str) else set()
        if intent.get('schema_version') != 1 or \
                intent.get('shadow_only') is not True or \
                intent.get('state') != 'REQUEST_CANCEL' or \
                intent.get('request_cancel') is not True or \
                intent.get('reason') != 'FIRE_ACQUIRED' or \
                not isinstance(mission_intent, str) or \
                mission_intent not in active_fire_intents or \
                intent.get('blockers') != [] or \
                navigation.get('mission_goal_active') is not True or \
                navigation.get('status_publisher_count') != 1 or \
                safety.get('nav2_action_client') is not False or \
                safety.get('cancel_request_sent') is not False or \
                safety.get('motion_command_publisher') is not False or \
                safety.get('actuator_command_publisher') is not False:
            blockers.append('CANCEL_INTENT_CONTRACT_INVALID')
        candidates = navigation.get('active_goals')
        if not isinstance(candidates, list) or len(candidates) != 1 or \
                not isinstance(candidates[0], dict):
            blockers.append('CANCEL_INTENT_GOAL_NOT_UNIQUE')
            return blockers, None
        uuid_hex = candidates[0].get('uuid')
        if not _valid_uuid(uuid_hex) or \
                candidates[0].get('status') not in (1, 2) or \
                self._active_goals() != [uuid_hex]:
            blockers.append('CANCEL_INTENT_GOAL_MISMATCH')
        if self.statuses is not None and any(
                uid == uuid_hex and status not in {1, 2}
                for uid, status in self.statuses):
            blockers.append('NAV_GOAL_NOT_CANCELABLE')
        return blockers, uuid_hex

    def step(self, now):
        """Return telemetry and at most one UUID requiring a cancel call."""
        now = float(now)
        send_uuid = None
        blockers = []
        # A completed cancellation belongs to one UUID only. A distinct
        # active Nav2 goal can begin a new cycle, never a failed old cycle.
        active = self._active_goals()
        if self.goal_uuid and self._pre_spray_clear_proven(now):
            self._clear_cycle()
        if self.complete and not self.failure and self.goal_uuid and \
                len(active) == 1 and active[0] != self.goal_uuid and \
                _valid_uuid(active[0]) and \
                active[0] not in self.used_goal_uuids and \
                self.status_publishers == 1 and \
                _age(self.status_received, now) is not None and \
                _age(self.status_received, now) <= self.telemetry_timeout and \
                self.canceled_at is not None and \
                all(status == 5 for uid, status in self.statuses
                    if uid == self.goal_uuid) and \
                self.raw_publishers == 0 and \
                self.alignment_started_at is None and \
                self._cmd_zero(now) and self._odom_stationary(now) and \
                self._arm_fresh(now):
            self._clear_cycle()
        if self.goal_uuid is None:
            blockers, candidate = self._preflight(now)
            if candidate in self.used_goal_uuids:
                blockers.append('GOAL_UUID_ALREADY_USED')
            state = 'DISABLED' if not self.enabled else \
                'WAITING_FOR_ADMISSION'
            if not blockers:
                self.goal_uuid = candidate
                self.sent_at = now
                self.send_count = 1
                send_uuid = candidate
                state = 'REQUEST_SENT'
        else:
            state = 'WAITING_FOR_CANCEL'
            if self.failure:
                state = 'FAILED'
                blockers.append(self.failure)
            elif self.shadow_publishers != 1 or \
                    self.arm_publishers != 1 or \
                    not self._shadow_isolated() or \
                    self.status_publishers != 1 or \
                    self.guard_publishers != 1:
                self.failure = 'SHADOW_TOPOLOGY_LOST'
            elif not self._arm_fresh(now):
                self.failure = 'OPERATOR_ARM_LOST'
            elif self.complete and self._alignment_owner():
                if self.alignment_started_at is None:
                    self.alignment_started_at = now
                if (self.statuses is None or
                        any(status != 5 for uid, status in self.statuses
                            if uid == self.goal_uuid) or
                        self._active_goals() or
                        not self._alignment_motion_bounded(now)):
                    self.failure = 'ALIGNMENT_MOTION_OR_CANCEL_PROOF_LOST'
                elif not self._alignment_active_report(now) and \
                        now - self.alignment_started_at > 0.75:
                    self.failure = 'ALIGNMENT_STATE_STALE_OR_INVALID'
                else:
                    state = 'ALIGNMENT_IN_PROGRESS'
            elif self.complete and self._ir_approach_owner():
                self.ir_release_seen_at = None
                if self.ir_started_at is None:
                    self.ir_started_at = now
                status = None if self.statuses is None else next(
                    (item_status for uid, item_status in self.statuses
                     if uid == self.goal_uuid), None)
                if (not self.ir_state_source_ok or
                        self.alignment_started_at is not None or
                        not self._alignment_complete(now) or
                        status not in (5, None) or
                        self.canceled_at is None or self._active_goals()):
                    self.failure = 'IR_APPROACH_CANCEL_PROOF_LOST'
                elif (self.ir_approach or {}).get(
                        'continuous_approach') is not True and \
                        now - self.ir_started_at > 3.0:
                    self.failure = 'IR_APPROACH_HANDOFF_TIMEOUT'
                elif (self._ir_approach_report(now, moving=True) and
                      self._ir_motion_bounded(now)):
                    state = 'IR_APPROACH_IN_PROGRESS'
                elif now - self.ir_started_at > 0.25:
                    self.failure = 'IR_APPROACH_CONTRACT_INVALID'
                else:
                    state = 'IR_APPROACH_WAITING_FOR_PROOF'
            elif self.raw_publishers != 0:
                self.failure = 'SHADOW_TOPOLOGY_LOST'
            elif self.complete:
                status = None if self.statuses is None else next(
                    (item_status for uid, item_status in self.statuses
                     if uid == self.goal_uuid), None)
                if status not in (5, None) or \
                        self.canceled_at is None or \
                        self._active_goals():
                    self.failure = 'POST_CANCEL_SAFETY_LOST'
                elif self.ir_started_at is not None:
                    if self.ir_release_seen_at is None:
                        self.ir_release_seen_at = now
                    report = self.ir_approach if isinstance(
                        self.ir_approach, dict) else {}
                    released_report_pending = (
                        now - self.ir_release_seen_at <= 0.25 and
                        report.get('goal_uuid') == self.goal_uuid and
                        report.get('publisher_owned') is True and
                        report.get('state') in ('STOPPING', 'COMPLETE') and
                        (report.get('command') or {}).get('linear_x_m_s') ==
                        0.0 and self._cmd_zero(now) and
                        self._odom_stationary(now))
                    if (not self.ir_state_source_ok or
                            ((self.ir_approach or {}).get(
                                'continuous_approach') is not True and
                             now - self.ir_started_at > 3.0) or
                            not (self._ir_approach_report(
                                now, moving=False) or
                                 released_report_pending)):
                        self.failure = 'IR_APPROACH_STOP_PROOF_INVALID'
                    elif released_report_pending:
                        self.post_ir_zero_since = None
                        state = 'WAITING_FOR_IR_STOP'
                    elif self._cmd_zero(now) and self._odom_stationary(now):
                        if self.post_ir_zero_since is None:
                            self.post_ir_zero_since = now
                        if now - self.post_ir_zero_since >= self.zero_hold:
                            self.ir_started_at = None
                            self.ir_release_seen_at = None
                            self.post_ir_zero_since = None
                            state = 'CANCEL_CONFIRMED_STOPPED'
                        else:
                            state = 'WAITING_FOR_IR_STOP'
                    else:
                        self.post_ir_zero_since = None
                        state = 'WAITING_FOR_IR_STOP'
                elif self.alignment_started_at is not None:
                    if self._alignment_complete(now) and \
                            self._cmd_zero(now) and \
                            self._odom_stationary(now):
                        if self.post_alignment_zero_since is None:
                            self.post_alignment_zero_since = now
                        if now - self.post_alignment_zero_since >= \
                                self.zero_hold:
                            self.alignment_started_at = None
                            state = 'CANCEL_CONFIRMED_STOPPED'
                        else:
                            state = 'WAITING_FOR_ALIGNMENT_STOP'
                    else:
                        self.post_alignment_zero_since = None
                        if _age(self.alignment_received, now) is None or \
                                _age(self.alignment_received, now) > \
                                self.telemetry_timeout or \
                                (isinstance(self.alignment, dict) and
                                 self.alignment.get('state') == 'COMPLETE' and
                                 self.alignment.get('stop_reason') !=
                                 'TARGET_CENTERED'):
                            self.failure = 'ALIGNMENT_STOP_PROOF_INVALID'
                        else:
                            state = 'WAITING_FOR_ALIGNMENT_STOP'
                elif not self._cmd_zero(now) or \
                        not self._odom_stationary(now):
                    self.failure = 'POST_CANCEL_SAFETY_LOST'
                else:
                    state = 'CANCEL_CONFIRMED_STOPPED'
            elif not self.service_ready and self.accepted_at is None:
                self.failure = 'NAV_CANCEL_SERVICE_LOST'
            elif self.accepted_at is None and \
                    now - self.sent_at > self.response_timeout:
                self.failure = 'CANCEL_RESPONSE_TIMEOUT'
            elif self.accepted_at is not None:
                status = None if self.statuses is None else next(
                    (item_status for uid, item_status in self.statuses
                     if uid == self.goal_uuid), None)
                other_active = [uid for uid in self._active_goals()
                                if uid != self.goal_uuid]
                if other_active:
                    self.failure = 'NEW_GOAL_ACTIVE'
                elif status in {4, 6}:
                    self.failure = 'GOAL_ENDED_WITHOUT_CANCEL'
                elif status == 5:
                    if self.canceled_at is None:
                        self.canceled_at = now
                    if self._cmd_zero(now) and self._odom_stationary(now):
                        if self.zero_since is None:
                            self.zero_since = now
                        if now - self.zero_since >= self.zero_hold:
                            self.complete = True
                            state = 'CANCEL_CONFIRMED_STOPPED'
                    else:
                        self.zero_since = None
                    if not self.complete and \
                            now - self.canceled_at > self.cancel_timeout:
                        self.failure = 'ROBOT_STOP_VERIFICATION_TIMEOUT'
                elif now - self.accepted_at > self.cancel_timeout:
                    self.failure = 'GOAL_CANCEL_STATUS_TIMEOUT'
            if self.failure:
                state = 'FAILED'
                blockers.append(self.failure)
                if self.failure_evidence is None:
                    self.failure_evidence = {
                        'command': self._command_evidence(now),
                        'odom_fresh_stationary': self._odom_stationary(now),
                        'active_goals': self._active_goals(),
                        'raw_sources': list(self.raw_sources),
                        'alignment_state': (self.alignment or {}).get(
                            'state') if isinstance(self.alignment, dict)
                        else None,
                        'ir_approach_state': (
                            self.ir_approach or {}).get('state') if
                        isinstance(self.ir_approach, dict) else None,
                        'shadow_publishers': self.shadow_publishers,
                        'arm_publishers': self.arm_publishers,
                        'status_publishers': self.status_publishers,
                        'guard_publishers': self.guard_publishers,
                    }
        return {
            'schema_version': 1,
            'state': state,
            'enabled': self.enabled,
            'goal_uuid': self.goal_uuid,
            'send_count': self.send_count,
            'cancel_service_accepted': self.accepted_at is not None,
            'cancel_status_seen': self.canceled_at is not None,
            'goal_status': None if self.goal_uuid is None or
            self.statuses is None else next(
                (status for uid, status in self.statuses
                 if uid == self.goal_uuid),
                5 if self.canceled_at is not None and not
                self._active_goals() else None),
            'cmd_vel_fresh_zero': self._cmd_zero(now),
            'odom_fresh_stationary': self._odom_stationary(now),
            'arm_fresh': self._arm_fresh(now),
            'nav_shadow_endpoints': len(self.nav_shadow_sources),
            'nav_shadow_fingerprint_ok': self._shadow_isolated(),
            'raw_cmd_publishers': self.raw_publishers,
            'alignment_handoff_active': self.alignment_started_at is not None,
            'ir_approach_handoff_active': self.ir_started_at is not None,
            'command_evidence': self._command_evidence(now),
            'failure_evidence': self.failure_evidence,
            'blockers': sorted(set(blockers)),
            'safety': {
                'motion_command_publisher': False,
                'actuator_command_publisher': False,
                'requires_operator_arm': True,
                'requires_shadow_nav2': True,
            },
        }, send_uuid


class Nav2CancelGuard(Node):
    """Only service client is goal-specific CancelGoal; no motion output."""

    def __init__(self):
        super().__init__('nav2_cancel_guard')
        self._odom_stamp_guard = OdomStampGuard()
        defaults = {
            'enable_real_cancel': False,
            'intent_topic': '/fire_mission/nav2_cancel_intent_shadow',
            'status_topic': '/navigate_to_pose/_action/status',
            'cancel_service': '/navigate_to_pose/_action/cancel_goal',
            'arm_topic': '/fire_mission/nav_cancel_arm',
            'odom_topic': '/odom',
            'cmd_topic': '/cmd_vel',
            'raw_cmd_topic': '/cmd_vel_raw',
            'shadow_nav_cmd_topic': '/fire_mission/nav2_cmd_vel_shadow',
            'alignment_state_topic': '/fire_mission/live_alignment_state',
            'ir_approach_state_topic': '/fire_mission/ir_approach_live_state',
            'mission_state_topic': '/fire_mission/state',
            'state_topic': '/fire_mission/nav2_cancel_guard_state',
            'publish_rate_hz': 10.0,
            'intent_timeout_s': 0.35,
            'arm_timeout_s': 0.40,
            'telemetry_timeout_s': 0.35,
            'response_timeout_s': 2.0,
            'cancel_timeout_s': 5.0,
            'zero_hold_s': 0.50,
            'stationary_linear_m_s': 0.02,
            'stationary_angular_rad_s': 0.05,
            'command_zero_epsilon': 0.001,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        params = {name: self.get_parameter(name).value for name in defaults}
        if not isinstance(params['enable_real_cancel'], bool):
            raise ValueError('enable_real_cancel must be a boolean')
        for name in defaults:
            if name.endswith('_topic') or name == 'cancel_service':
                if not isinstance(params[name], str) or not params[name]:
                    raise ValueError(f'{name} must be a nonempty topic')
        for name, required in CANONICAL_ENDPOINTS.items():
            if params[name] != required:
                raise ValueError(
                    f'{name} must remain {required} for this safety gate')
        if params['state_topic'] in FORBIDDEN_OUTPUTS or \
                not params['state_topic'].startswith('/fire_mission/'):
            raise ValueError('guard telemetry cannot be a robot command')
        if params['arm_topic'] == params['state_topic'] or \
                params['intent_topic'] == params['state_topic']:
            raise ValueError('guard topics must be distinct')
        self.core = Nav2CancelGuardCore(
            enabled=params['enable_real_cancel'],
            **{name: params[name] for name in (
                'intent_timeout_s', 'arm_timeout_s',
                'telemetry_timeout_s', 'response_timeout_s',
                'cancel_timeout_s', 'zero_hold_s',
                'stationary_linear_m_s', 'stationary_angular_rad_s',
                'command_zero_epsilon')})
        self.intent_topic = params['intent_topic']
        self.status_topic = params['status_topic']
        self.raw_cmd_topic = params['raw_cmd_topic']
        self.shadow_nav_cmd_topic = params['shadow_nav_cmd_topic']
        self.cmd_topic = params['cmd_topic']
        self.state_topic = params['state_topic']
        self.cancel_client = self.create_client(
            CancelGoal, params['cancel_service'])
        self.publisher = self.create_publisher(
            String, params['state_topic'], sensor_qos())
        self.create_subscription(
            String, self.intent_topic, self._on_intent, sensor_qos())
        self.create_subscription(
            String, params['alignment_state_topic'],
            self._on_alignment, sensor_qos())
        self.create_subscription(
            String, params['ir_approach_state_topic'],
            self._on_ir_approach, sensor_qos())
        self.create_subscription(
            String, params['mission_state_topic'],
            self._on_mission, sensor_qos())
        self.create_subscription(
            Bool, params['arm_topic'], self._on_arm, arm_qos())
        self.create_subscription(
            GoalStatusArray, self.status_topic, self._on_status,
            action_status_qos())
        self.create_subscription(
            Odometry, params['odom_topic'], self._on_odom, sensor_qos())
        self.create_subscription(
            Twist, self.cmd_topic, self._on_cmd, sensor_qos())
        rate = float(params['publish_rate_hz'])
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate_hz must be positive')
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().warning(
            '[MISSION] NAV CANCEL GUARD: enabled=%s, one-shot exact UUID, '
            'shadow Nav2 only' % self.core.enabled)

    def _on_intent(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self.core.update_intent(payload, time.monotonic())

    def _on_alignment(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self.core.update_alignment(payload, time.monotonic())

    def _on_ir_approach(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self.core.update_ir_approach(payload, time.monotonic())

    def _on_mission(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        infos = self.get_publishers_info_by_topic(
            CANONICAL_ENDPOINTS['mission_state_topic'])
        source_ok = (len(infos) == 1 and
                     infos[0].node_namespace == '/' and
                     infos[0].node_name == 'fire_mission_supervisor' and
                     infos[0].topic_type == 'std_msgs/msg/String' and
                     infos[0].qos_profile.reliability ==
                     ReliabilityPolicy.BEST_EFFORT)
        self.core.update_mission(payload, time.monotonic(), source_ok)

    def _on_arm(self, message):
        self.core.update_arm(message.data, time.monotonic())

    def _on_status(self, message):
        self.core.update_statuses([
            (bytes(item.goal_info.goal_id.uuid).hex(), item.status)
            for item in message.status_list], time.monotonic())

    @staticmethod
    def _twist_values(twist):
        return (twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z)

    def _on_odom(self, message):
        received = admit_odom(self, message, self.core.telemetry_timeout)
        if received is None:
            return
        self.core.update_odom(
            self._twist_values(message.twist.twist), received)

    def _on_cmd(self, message):
        self.core.update_cmd(self._twist_values(message), time.monotonic())

    def _on_response(self, future):
        try:
            response = future.result()
        except Exception as exc:  # ROS service failures must remain visible.
            self.get_logger().error(f'CancelGoal service failed: {exc}')
            response = None
        self.core.on_cancel_response(response, time.monotonic())

    def _ir_state_source_ok(self):
        infos = self.get_publishers_info_by_topic(
            CANONICAL_ENDPOINTS['ir_approach_state_topic'])
        return (len(infos) == 1 and infos[0].node_namespace == '/' and
                infos[0].node_name == 'ir_approach_live' and
                infos[0].topic_type == 'std_msgs/msg/String' and
                infos[0].qos_profile.reliability ==
                ReliabilityPolicy.BEST_EFFORT)

    def _tick(self):
        nav_shadow_sources = [
            (info.node_namespace, info.node_name, info.topic_type,
             info.qos_profile.reliability)
            for info in self.get_publishers_info_by_topic(
                self.shadow_nav_cmd_topic)]
        self.core.update_topology(
            self.count_publishers(self.status_topic),
            self.count_publishers(self.intent_topic),
            self.count_publishers(CANONICAL_ENDPOINTS['arm_topic']),
            nav_shadow_sources,
            self.count_publishers(self.raw_cmd_topic),
            self.count_publishers(self.cmd_topic),
            self.count_publishers(self.state_topic),
            self.cancel_client.service_is_ready(),
            [(info.node_namespace, info.node_name, info.topic_type,
              info.qos_profile.reliability)
             for info in self.get_publishers_info_by_topic(
                 self.raw_cmd_topic)],
            ir_state_source_ok=self._ir_state_source_ok(),
            pump_publishers=self.count_publishers('/pump_cmd'))
        payload, uuid_hex = self.core.step(time.monotonic())
        if uuid_hex is not None:
            try:
                future = self.cancel_client.call_async(
                    make_cancel_request(uuid_hex))
                future.add_done_callback(self._on_response)
                self.get_logger().warning(
                    f'[MISSION] CancelGoal sent for UUID {uuid_hex}')
            except Exception as exc:
                self.core.failure = 'CANCEL_SERVICE_SEND_FAILED'
                self.get_logger().error(f'CancelGoal could not be sent: {exc}')
                payload, _ = self.core.step(time.monotonic())
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        self.publisher.publish(message)


def main(args=None):
    """Run the opt-in goal-specific cancel guard on the laptop."""
    rclpy.init(args=args)
    node = None
    try:
        node = Nav2CancelGuard()
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
