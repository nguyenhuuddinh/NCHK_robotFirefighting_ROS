"""
One-shot, opt-in body alignment after an exact Nav2 cancel proof.

The old guard and handoff shadow nodes deliberately revoke their proofs when
this node creates a real raw-command publisher.  Their *fresh pre-takeover*
proof is therefore captured once, then this node independently checks the
mission, alignment, action status, odometry, SafetyGate and ROS graph until it
has sent zero and released its own publisher.  It never commands translation
or any actuator.
"""

from collections import Counter
import json
import math
import signal
import time

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from fire_robot_safety.odom_freshness import OdomStampGuard, admit_odom
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan

from fire_robot_mission.alignment_handoff_shadow import arm_qos
from fire_robot_mission.live_nav_stop_gateway import (
    SAFETY_ENDPOINT, _fresh,
)
from fire_robot_mission.mission_supervisor import (
    action_status_qos, sensor_qos,
)
from fire_robot_mission.nav2_cancel_guard import (
    ACTIVE_STATUSES, EXPECTED_SHADOW_PUBLISHERS, _valid_uuid,
)


OWN_ENDPOINT = Counter({
    ('/', 'live_alignment_takeover', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.BEST_EFFORT): 1,
})
CANONICAL_TOPICS = (
    '/cmd_vel_raw', '/cmd_vel', '/odom', '/scan',
    '/fire_mission/state', '/yolo/alignment_preview',
    '/fire_mission/alignment_handoff_state',
    '/fire_mission/live_nav_stop_state',
    '/fire_mission/nav2_cancel_guard_state',
    '/fire_mission/nav2_cmd_vel_shadow',
    '/fire_mission/live_alignment_arm',
    '/fire_mission/live_alignment_state',
    '/fire_mission/post_spray_review',
    '/fire_sensors/actuator_state',
    '/navigate_to_pose/_action/status',
)
FIRE_STATES = {'FIRE_TRACKING', 'CENTERED_HOLD', 'ALIGNMENT_READY'}


class LiveAlignmentCore:
    """One owned turn: zero on target loss, resume only on fresh fire proof."""

    def __init__(self, enabled=False, input_timeout_s=0.35,
                 arm_timeout_s=0.40, discovery_timeout_s=0.75,
                 zero_hold_s=0.50,
                 max_angular_speed_rad_s=0.60,
                 max_angular_accel_rad_s2=0.40,
                 max_observed_angular_rad_s=0.80,
                 stationary_linear_m_s=0.02,
                 stationary_angular_rad_s=0.05,
                 command_zero_epsilon=0.001,
                 min_rotation_clearance_m=0.30,
                 max_total_rotation_rad=2.0 * math.pi):
        values = (input_timeout_s, arm_timeout_s, discovery_timeout_s,
                  zero_hold_s,
                  max_angular_speed_rad_s, max_angular_accel_rad_s2,
                  max_observed_angular_rad_s,
                  stationary_linear_m_s, stationary_angular_rad_s,
                  command_zero_epsilon, min_rotation_clearance_m,
                  max_total_rotation_rad)
        if type(enabled) is not bool or any(
                not isinstance(value, (int, float)) or
                not math.isfinite(float(value)) or value <= 0
                for value in values):
            raise ValueError('live alignment parameters must be positive')
        if max_angular_speed_rad_s > 0.60 or \
                max_angular_accel_rad_s2 > 0.50 or \
                max_observed_angular_rad_s > 0.80 or \
                max_observed_angular_rad_s < max_angular_speed_rad_s or \
                input_timeout_s > 0.50 or arm_timeout_s > 0.50 or \
                discovery_timeout_s > 1.0 or zero_hold_s < 0.50 or \
                stationary_linear_m_s > 0.02 or \
                stationary_angular_rad_s > 0.05 or \
                command_zero_epsilon > 0.001 or \
                min_rotation_clearance_m < 0.30 or \
                max_total_rotation_rad > 2.0 * math.pi:
            raise ValueError('live alignment hard safety limit exceeded')
        self.enabled = enabled
        self.input_timeout = float(input_timeout_s)
        self.arm_timeout = float(arm_timeout_s)
        self.discovery_timeout = float(discovery_timeout_s)
        self.zero_hold = float(zero_hold_s)
        self.max_angular = float(max_angular_speed_rad_s)
        self.max_accel = float(max_angular_accel_rad_s2)
        self.max_observed_angular = float(max_observed_angular_rad_s)
        self.stationary_linear = float(stationary_linear_m_s)
        self.stationary_angular = float(stationary_angular_rad_s)
        self.zero_epsilon = float(command_zero_epsilon)
        self.min_clearance = float(min_rotation_clearance_m)
        self.max_total_rotation = float(max_total_rotation_rad)
        self.inputs = {}
        self.arm = False
        self.arm_at = None
        self.odom = None
        self.odom_at = None
        self.cmd = None
        self.cmd_at = None
        self.scan = None
        self.scan_at = None
        self.statuses = None
        self.phase = 'WAITING_FOR_ADMISSION'
        self.goal_uuid = None
        self.acquired_at = None
        self.last_step_at = None
        self.last_angular = 0.0
        self.stop_reason = ''
        self.stop_alignment_input = None
        self.zero_since = None
        self.target_missing_since = None
        self.progress_at = None
        self.best_abs_error = None
        self.stalled_since = None
        self.total_rotation_rad = 0.0
        self.last_realign_ir_step = 0
        self.last_review_pulse = 0
        self.pending_review = None
        self.used_goal_uuids = set()

    def update(self, name, payload, now):
        """Store a payload; malformed data becomes a fail-closed None."""
        self.inputs[name] = (payload, float(now))

    def update_arm(self, value, now):
        self.arm = value is True
        self.arm_at = float(now)

    def update_odom(self, values, now):
        self.odom = tuple(values)
        self.odom_at = float(now)

    def update_cmd(self, values, now):
        self.cmd = tuple(values)
        self.cmd_at = float(now)

    def update_statuses(self, statuses):
        self.statuses = tuple(statuses)

    def update_scan(self, angle_min, angle_max, ranges, now,
                    range_max=8.0):
        self.scan = (float(angle_min), float(angle_max),
                     float(range_max), tuple(ranges))
        self.scan_at = float(now)

    @staticmethod
    def _valid_twist(values):
        return values is not None and len(values) == 6 and all(
            isinstance(v, (int, float)) and math.isfinite(v)
            for v in values)

    def _scan_clear(self, now):
        if not _fresh(self.scan_at, now, self.input_timeout) or \
                self.scan is None:
            return False
        angle_min, angle_max, range_max, ranges = self.scan
        if not math.isfinite(angle_min) or not math.isfinite(angle_max) or \
                not math.isfinite(range_max) or \
                range_max <= self.min_clearance or \
                abs(angle_max - angle_min) < 5.8 or len(ranges) < 180:
            return False
        valid = 0
        finite = 0
        for value in ranges:
            if isinstance(value, (int, float)) and value > 0.0 and \
                    math.isfinite(value):
                if value < self.min_clearance:
                    return False
                if value <= range_max:
                    valid += 1
                    finite += 1
            elif value == math.inf:
                valid += 1
        return valid >= int(0.8 * len(ranges)) and finite >= 10

    def _input(self, name, now):
        payload, received = self.inputs.get(name, (None, None))
        return payload if _fresh(received, now, self.input_timeout) and \
            isinstance(payload, dict) else None

    def _stationary(self, now):
        values = self.odom
        return bool(_fresh(self.odom_at, now, self.input_timeout) and
                    self._valid_twist(values) and
                    all(abs(v) <= self.stationary_linear for v in values[:3])
                    and all(abs(v) <= self.stationary_angular
                            for v in values[3:]))

    def _cmd_zero(self, now):
        values = self.cmd
        return bool(_fresh(self.cmd_at, now, self.input_timeout) and
                    self._valid_twist(values) and
                    all(abs(v) <= self.zero_epsilon for v in values))

    def _arm_ok(self, now, counts):
        return counts.get('arm') == 1 and self.arm and \
            _fresh(self.arm_at, now, self.arm_timeout)

    def _fire_and_direction(self, now):
        mission = self._input('mission', now)
        preview = self._input('preview', now)
        if mission is None or preview is None:
            return 'INPUT_MISSING_OR_STALE', 0.0, None
        perception = mission.get('perception') or {}
        navigation = mission.get('navigation') or {}
        robot = mission.get('robot') or {}
        health = mission.get('health') or {}
        safety = mission.get('safety') or {}
        if mission.get('schema_version') != 1 or \
                mission.get('observe_only') is not True or \
                mission.get('state') not in FIRE_STATES or \
                perception.get('detected_now') is not True or \
                perception.get('confirmed') is not True or \
                navigation.get('goal_active') is not False or \
                robot.get('odom_fresh') is not True or \
                health.get('status') != 'OK' or health.get('blockers') or \
                (mission.get('decision') or {}).get(
                    'command_published') is not False or \
                safety.get('motion_command_publisher') is not False or \
                safety.get('actuator_command_publisher') is not False or \
                safety.get('collision_check_bypassed') is not False:
            return 'MISSION_FIRE_OR_SAFETY_LOST', 0.0, None
        preview_safety = preview.get('safety') or {}
        state = preview.get('state')
        target = (preview.get('preview') or {}).get(
            'target_angular_z_rad_s')
        if preview.get('observe_only') is not True or \
                preview_safety.get(
                    'motion_command_publisher') is not False or \
                preview_safety.get(
                    'actuator_command_publisher') is not False or \
                not isinstance(target, (int, float)) or \
                not math.isfinite(target):
            return 'ALIGNMENT_CONTRACT_INVALID', 0.0, None
        if state == 'CENTERED' and abs(target) <= 0.03:
            return '', 0.0, state
        if (state == 'ALIGNING_LEFT' and target > 0.0) or \
                (state == 'ALIGNING_RIGHT' and target < 0.0):
            return '', float(target), state
        return 'ALIGNMENT_DIRECTION_INVALID', 0.0, None

    def _safe_target_miss(self, now):
        """Classify a safe missing target; never reuse a stale turn command."""
        mission = self._input('mission', now) or {}
        preview = self._input('preview', now) or {}
        perception = mission.get('perception') or {}
        navigation = mission.get('navigation') or {}
        health = mission.get('health') or {}
        safety = mission.get('safety') or {}
        preview_safety = preview.get('safety') or {}
        return (mission.get('schema_version') == 1 and
                mission.get('observe_only') is True and
                mission.get('state') in FIRE_STATES | {
                    'TARGET_LOST', 'FIRE_CANDIDATE',
                    'WAITING_FOR_ALIGNMENT'} and
                navigation.get('goal_active') is False and
                (mission.get('robot') or {}).get('odom_fresh') is True and
                health.get('status') == 'OK' and not health.get('blockers') and
                (mission.get('decision') or {}).get(
                    'command_published') is False and
                safety.get('motion_command_publisher') is False and
                safety.get('actuator_command_publisher') is False and
                safety.get('collision_check_bypassed') is False and
                preview.get('observe_only') is True and
                preview_safety.get('motion_command_publisher') is False and
                preview_safety.get('actuator_command_publisher') is False and
                (mission.get('state') == 'FIRE_CANDIDATE' or
                 preview.get('state') == 'NO_TARGET' or
                 perception.get('detected_now') is False or
                 perception.get('confirmed') is False))

    def _stop_input_snapshot(self, now):
        """Latch read-only evidence from the frame that stopped the turn."""
        preview, received = self.inputs.get('preview', (None, None))
        mission, _ = self.inputs.get('mission', (None, None))
        preview = preview if isinstance(preview, dict) else {}
        mission = mission if isinstance(mission, dict) else {}
        values = preview.get('preview') or {}
        values = values if isinstance(values, dict) else {}

        def finite_number(value):
            return float(value) if isinstance(value, (int, float)) and \
                not isinstance(value, bool) and math.isfinite(value) else None

        return {
            'mission_state': mission.get('state'),
            'preview_state': preview.get('state'),
            'preview_direction': values.get('direction'),
            'error_x_normalized': finite_number(
                values.get('error_x_normalized')),
            'target_angular_z_rad_s': finite_number(
                values.get('target_angular_z_rad_s')),
            'preview_receive_age_ms': round(max(0.0, now - received) * 1000, 1)
            if received is not None else None,
        }

    def _record_centering_progress(self, now):
        """Renew the turn window only on a fresh confirmed best error."""
        mission = self._input('mission', now) or {}
        preview = self._input('preview', now) or {}
        value = (preview.get('preview') or {}).get('error_x_normalized')
        if (mission.get('state') not in FIRE_STATES or
                (mission.get('perception') or {}).get(
                    'detected_now') is not True or
                (mission.get('perception') or {}).get(
                    'confirmed') is not True or
                preview.get('state') not in (
                    'ALIGNING_LEFT', 'ALIGNING_RIGHT', 'CENTERED') or
                not isinstance(value, (int, float)) or
                isinstance(value, bool) or not math.isfinite(value) or
                abs(value) > 1.0):
            return
        error = abs(float(value))
        if self.best_abs_error is None or \
                self.best_abs_error - error >= 0.03:
            self.best_abs_error = error
            self.progress_at = now

    def _status_ok(self):
        if self.statuses is None or any(
                status in ACTIVE_STATUSES for _, status in self.statuses):
            return False
        matching = [status for uuid, status in self.statuses
                    if uuid == self.goal_uuid]
        return not matching or matching == [5]

    def _preflight(self, now, raw, nav, safety, counts):
        if not self.enabled:
            return 'LIVE_ALIGNMENT_DISABLED'
        if raw or Counter(nav) != EXPECTED_SHADOW_PUBLISHERS or \
                Counter(safety) != SAFETY_ENDPOINT:
            return 'COMMAND_TOPOLOGY_INVALID'
        if any(counts.get(name) != 1 for name in (
                'mission', 'preview', 'handoff', 'gateway', 'guard',
                'odom', 'cmd', 'scan', 'status', 'arm')):
            return 'INPUT_PUBLISHER_INVALID'
        if not self._arm_ok(now, counts):
            return 'OPERATOR_ARM_MISSING_OR_STALE'
        if not self._stationary(now) or not self._cmd_zero(now):
            return 'ROBOT_NOT_FRESH_STATIONARY'
        if not self._scan_clear(now):
            return 'SCAN_MISSING_OR_UNSAFE'
        handoff = self._input('handoff', now) or {}
        gateway = self._input('gateway', now) or {}
        guard = self._input('guard', now) or {}
        proof = handoff.get('live_cancel_proof') or {}
        uuid = proof.get('goal_uuid')
        if not (_valid_uuid(uuid) and handoff.get('shadow_only') is True and
                handoff.get('active') is True and
                handoff.get('admission') == {
                    'eligible': True, 'blockers': []} and
                proof.get('required') is True and
                proof.get('gateway_state') == 'HANDOFF_READY' and
                proof.get('cancel_guard_state') ==
                'CANCEL_CONFIRMED_STOPPED' and
                gateway.get('state') == 'HANDOFF_READY' and
                gateway.get('handoff_ready') is True and
                gateway.get('released') is True and
                gateway.get('stop_reason') == 'FIRE_ACQUIRED' and
                gateway.get('raw_publishers') == 0 and
                (gateway.get('safety') or {}).get(
                    'raw_command_publisher') is False and
                guard.get('state') == 'CANCEL_CONFIRMED_STOPPED' and
                guard.get('goal_uuid') == uuid and
                guard.get('send_count') == 1 and
                guard.get('cancel_service_accepted') is True and
                guard.get('cancel_status_seen') is True and
                guard.get('goal_status') == 5 and
                guard.get('arm_fresh') is True and
                guard.get('raw_cmd_publishers') == 0 and
                guard.get('blockers') == []):
            return 'EXACT_CANCEL_HANDOFF_PROOF_INVALID'
        if uuid in self.used_goal_uuids:
            return 'GOAL_UUID_ALREADY_USED'
        self.goal_uuid = uuid
        if not self._status_ok():
            return 'NAV_STATUS_NOT_CANCELED'
        reason, _, direction = self._fire_and_direction(now)
        if reason:
            return reason
        if direction == 'CENTERED':
            return 'ALREADY_CENTERED'
        return ''

    def step(self, now, publisher_owned=False, raw_sources=(),
             nav_sources=(), safety_sources=(), counts=None):
        """Return (angular, telemetry, CREATE/RELEASE/None)."""
        now = float(now)
        counts = counts or {}
        ir = self._input('ir', now) or {}
        guard = self._input('guard', now) or {}
        review = self._input('review', now) or {}
        actuator = self._input('actuator', now) or {}
        if (self.phase == 'COMPLETE' and self.goal_uuid and
                counts.get('review') == 1 and
                counts.get('review_source_ok') is True and
                review.get('schema_version') == 1 and
                review.get('mode') == 'SHADOW_ONLY' and
                review.get('state') == 'REALIGN_REQUIRED' and
                review.get('goal_uuid') == self.goal_uuid and
                review.get('blockers') == [] and
                type(review.get('pulse_index')) is int and
                review['pulse_index'] > self.last_review_pulse):
            self.pending_review = (review['pulse_index'], now)
        if (self.pending_review is not None and
                now - self.pending_review[1] > 30.0):
            self.pending_review = None
        review_realign = (
            self.pending_review is not None and
            counts.get('pump') == 0 and
            counts.get('actuator_source_ok') is True and
            actuator.get('pump_pwm_commanded_on') is False and
            actuator.get('servo_pwm_commanded_on') is False and
            actuator.get('servo_phase') == 'IDLE' and
            isinstance(actuator.get('receive_age_ms'), (int, float)) and
            0 <= actuator['receive_age_ms'] < 350)
        # Centering can drift while IR has not yet acquired raw motion.
        # A waiting IR node cannot issue REALIGN_REQUIRED, so accept this
        # narrow, stationary same-goal handoff before its first segment.
        pre_ir_realign = (
            counts.get('ir') == 1 and
            counts.get('ir_source_ok') is True and
            ir.get('schema_version') == 1 and
            ir.get('state') == 'WAITING_FOR_ADMISSION' and
            ir.get('goal_uuid') in (None, self.goal_uuid) and
            ir.get('step_count') == 0 and
            ir.get('publisher_owned') is False and
            counts.get('pump') == 0 and
            (self._input('mission', now) or {}).get('state') ==
            'FIRE_TRACKING' and
            (self._input('preview', now) or {}).get('state') in (
                'ALIGNING_LEFT', 'ALIGNING_RIGHT'))
        if (self.phase == 'COMPLETE' and not publisher_owned and
                self.stop_reason == 'TARGET_CENTERED' and self.goal_uuid and
                ((counts.get('ir') == 1 and
                  counts.get('ir_source_ok') is True and
                  ir.get('schema_version') == 1 and
                  ir.get('state') == 'COMPLETE' and
                  ir.get('stop_reason') == 'REALIGN_REQUIRED' and
                  ir.get('goal_uuid') == self.goal_uuid and
                  ir.get('publisher_owned') is False and
                  type(ir.get('step_count')) is int and
                  ir['step_count'] > self.last_realign_ir_step and
                  (ir.get('recovery_required') is not True or
                   ir.get('recovery_ready') is True)) or
                 review_realign or pre_ir_realign) and
                guard.get('state') == 'CANCEL_CONFIRMED_STOPPED' and
                guard.get('goal_uuid') == self.goal_uuid and
                guard.get('blockers') == [] and not raw_sources and
                self._cmd_zero(now) and self._stationary(now)):
            if review_realign:
                self.last_review_pulse = self.pending_review[0]
                self.pending_review = None
            else:
                self.last_realign_ir_step = ir['step_count']
            self.phase = 'WAITING_FOR_ADMISSION'
            self.acquired_at = None
            self.last_step_at = None
            self.last_angular = 0.0
            self.stop_reason = ''
            self.stop_alignment_input = None
            self.zero_since = None
            self.target_missing_since = None
            self.progress_at = None
            self.best_abs_error = None
            self.stalled_since = None
            self.total_rotation_rad = 0.0
        if self.phase == 'COMPLETE' and not publisher_owned and \
                self.goal_uuid and self.statuses is not None:
            active = [uuid for uuid, status in self.statuses
                      if status in ACTIVE_STATUSES]
            mission = self._input('mission', now) or {}
            lifecycle = mission.get('lifecycle') or {}
            perception = mission.get('perception') or {}
            next_goal_only = (len(active) <= 1 and all(
                _valid_uuid(uid) and uid != self.goal_uuid and
                uid not in self.used_goal_uuids for uid in active))
            pre_spray_clear = (
                counts.get('mission_source_ok') is True and
                counts.get('pump') == 0 and
                counts.get('actuator_source_ok') is True and
                actuator.get('pump_pwm_commanded_on') is False and
                actuator.get('servo_pwm_commanded_on') is False and
                actuator.get('servo_phase') == 'IDLE' and
                mission.get('state') in ('IDLE', 'NAV2_NAVIGATING') and
                perception.get('detected_now') is False and
                perception.get('confirmed') is False and
                lifecycle.get('pre_spray_clear_goal_uuid') ==
                self.goal_uuid and next_goal_only and not raw_sources and
                self._cmd_zero(now) and self._stationary(now))
            normal_next_goal = (
                self.stop_reason == 'TARGET_CENTERED' and
                len(active) == 1 and _valid_uuid(active[0]) and
                active[0] != self.goal_uuid and
                active[0] not in self.used_goal_uuids)
            if pre_spray_clear or normal_next_goal:
                self.used_goal_uuids.add(self.goal_uuid)
                self.phase = 'WAITING_FOR_ADMISSION'
                self.goal_uuid = None
                self.acquired_at = None
                self.last_step_at = None
                self.last_angular = 0.0
                self.stop_reason = ''
                self.stop_alignment_input = None
                self.zero_since = None
                self.target_missing_since = None
                self.progress_at = None
                self.best_abs_error = None
                self.stalled_since = None
                self.total_rotation_rad = 0.0
                self.last_realign_ir_step = 0
                self.last_review_pulse = 0
                self.pending_review = None
                self.inputs.clear()
        action = None
        angular = 0.0
        blockers = []
        if self.phase == 'WAITING_FOR_ADMISSION':
            reason = self._preflight(
                now, raw_sources, nav_sources, safety_sources, counts)
            if reason == 'ALREADY_CENTERED':
                # Exact canceled-goal, scan and stationary proofs passed.
                # Do not rotate merely to create a motion attempt.
                self.phase = 'COMPLETE'
                self.stop_reason = 'TARGET_CENTERED'
            elif reason:
                blockers.append(reason)
            else:
                self.phase = 'ACQUIRING'
                self.acquired_at = now
                self.progress_at = now
                action = 'CREATE'
        elif self.phase in ('ACQUIRING', 'TURNING'):
            reason = ''
            if publisher_owned and self.last_step_at is not None and \
                    _fresh(self.odom_at, now, self.input_timeout) and \
                    self._valid_twist(self.odom):
                dt = min(0.1, max(0.0, now - self.last_step_at))
                self.total_rotation_rad += dt * abs(self.odom[5])
            if not publisher_owned or not self._arm_ok(now, counts):
                reason = 'PUBLISHER_OR_ARM_LOST'
            elif Counter(nav_sources) != EXPECTED_SHADOW_PUBLISHERS or \
                    Counter(safety_sources) != SAFETY_ENDPOINT or \
                    any(counts.get(name) != 1 for name in (
                        'mission', 'preview', 'odom', 'cmd', 'scan',
                        'status', 'arm')):
                reason = 'RUNTIME_TOPOLOGY_INVALID'
            elif raw_sources and Counter(raw_sources) != OWN_ENDPOINT:
                reason = 'RAW_OWNER_CONFLICT'
            elif self.total_rotation_rad >= self.max_total_rotation:
                reason = 'TURN_ROTATION_LIMIT'
            elif not _fresh(self.odom_at, now, self.input_timeout) or \
                    not _fresh(self.cmd_at, now, self.input_timeout):
                reason = 'ODOM_OR_SAFETY_COMMAND_STALE'
            elif not self._valid_twist(self.odom) or \
                    not self._valid_twist(self.cmd) or \
                    any(abs(v) > self.zero_epsilon for v in self.cmd[:3]):
                reason = 'ODOM_OR_SAFETY_COMMAND_INVALID'
            elif not self._scan_clear(now):
                reason = 'SCAN_MISSING_OR_UNSAFE'
            elif not self._status_ok():
                reason = 'NEW_OR_CHANGED_NAV_GOAL'
            else:
                reason, target, direction = self._fire_and_direction(now)
                mission_now = self._input('mission', now) or {}
                lifecycle_now = mission_now.get('lifecycle') or {}
                if (mission_now.get('state') == 'IDLE' and
                        lifecycle_now.get('pre_spray_clear_goal_uuid') ==
                        self.goal_uuid):
                    reason = 'PRE_SPRAY_FIRE_CLEARED'
                if reason in ('ALIGNMENT_DIRECTION_INVALID',
                              'MISSION_FIRE_OR_SAFETY_LOST') and \
                        self._safe_target_miss(now):
                    if self.target_missing_since is None:
                        self.target_missing_since = now
                    # Stay zero and retain the same exact-goal ownership.
                    # Waiting for a target is stationary; never reuse a
                    # command from the last visible frame.
                    reason, target, direction = '', 0.0, 'REACQUIRING'
                else:
                    if self.target_missing_since is not None and not reason:
                        self.progress_at = now
                    self.target_missing_since = None
                if not reason and direction != 'REACQUIRING':
                    self._record_centering_progress(now)
                if not reason and direction == 'CENTERED':
                    reason = 'TARGET_CENTERED'
                elif not reason and self.odom is not None and \
                        abs(self.odom[5]) > self.max_observed_angular:
                    reason = 'ODOM_ANGULAR_OVERSPEED'
            if not reason and not raw_sources:
                if now - self.acquired_at > self.discovery_timeout:
                    reason = 'OWN_PUBLISHER_NOT_DISCOVERED'
            elif not reason:
                self.phase = 'TURNING'
                if direction == 'REACQUIRING':
                    self.last_angular = 0.0
                requested = max(-self.max_angular,
                                min(self.max_angular, target))
                if requested * self.last_angular < 0.0:
                    self.last_angular = 0.0
                dt = min(0.1, max(0.0, now - (
                    self.last_step_at if self.last_step_at is not None
                    else now)))
                delta = self.max_accel * dt
                angular = max(self.last_angular - delta,
                              min(self.last_angular + delta, requested))
                self.last_angular = angular
                if abs(angular) >= 0.20 and \
                        abs(self.odom[5]) <= 0.02:
                    if self.stalled_since is None:
                        self.stalled_since = now
                    elif now - self.stalled_since >= 2.0:
                        reason = 'ODOM_TURN_STALLED'
                        angular = 0.0
                else:
                    self.stalled_since = None
            if reason:
                self.phase = 'STOPPING'
                self.stop_reason = reason
                self.stop_alignment_input = self._stop_input_snapshot(now)
                self.stop_alignment_input['target_wait_s'] = (
                    None if self.target_missing_since is None else
                    round(max(0.0, now - self.target_missing_since), 2))
                self.target_missing_since = None
                self.last_angular = 0.0
                self.stalled_since = None
        elif self.phase == 'STOPPING':
            if publisher_owned and Counter(raw_sources) == OWN_ENDPOINT and \
                    Counter(safety_sources) == SAFETY_ENDPOINT and \
                    self._stationary(now) and self._cmd_zero(now):
                if self.zero_since is None:
                    self.zero_since = now
                elif now - self.zero_since >= self.zero_hold:
                    self.phase = 'COMPLETE'
                    action = 'RELEASE'
            else:
                self.zero_since = None
        self.last_step_at = now
        telemetry = {
            'schema_version': 1,
            'state': 'DISABLED' if not self.enabled and
            self.phase == 'WAITING_FOR_ADMISSION' else self.phase,
            'enabled': self.enabled,
            'one_shot': True,
            'goal_uuid': self.goal_uuid,
            'publisher_owned': publisher_owned,
            'command': {'linear_x_m_s': 0.0,
                        'angular_z_rad_s': round(angular, 4)},
            'stop_reason': self.stop_reason,
            'stop_alignment_input': self.stop_alignment_input,
            'target_reacquiring': self.target_missing_since is not None and
            self.phase == 'TURNING',
            'target_wait_s': None if self.target_missing_since is None else
            round(max(0.0, now - self.target_missing_since), 2),
            'turn_progress': {
                'best_abs_error_x': self.best_abs_error,
                'elapsed_s': None if self.acquired_at is None else
                round(max(0.0, now - self.acquired_at), 2),
                'since_progress_s': None if self.progress_at is None else
                round(max(0.0, now - self.progress_at), 2),
                'no_progress_limit_s': None,
                'total_limit_s': None,
                'total_rotation_rad': round(self.total_rotation_rad, 3),
                'rotation_limit_rad': self.max_total_rotation,
            },
            'blockers': blockers,
            'safety': {'pump_command_publisher': False,
                       'linear_motion': False,
                       'requires_operator_arm': True,
                       'fail_closed_zero': angular == 0.0},
        }
        return angular, telemetry, action


class LiveAlignmentTakeover(Node):
    """Sole raw-command owner for one post-cancel body alignment attempt."""

    def __init__(self):
        super().__init__('live_alignment_takeover')
        self._odom_stamp_guard = OdomStampGuard()
        for topic in CANONICAL_TOPICS:
            if self.resolve_topic_name(topic) != topic:
                raise ValueError(f'live alignment forbids remapping {topic}')
        self.declare_parameter('enable_live_alignment', False)
        self.declare_parameter('publish_rate_hz', 20.0)
        defaults = {
            'input_timeout_s': 0.35,
            'arm_timeout_s': 0.40,
            'discovery_timeout_s': 0.75,
            'zero_hold_s': 0.50,
            'max_angular_speed_rad_s': 0.60,
            'max_angular_accel_rad_s2': 0.40,
            'max_observed_angular_rad_s': 0.80,
            'stationary_linear_m_s': 0.02,
            'stationary_angular_rad_s': 0.05,
            'command_zero_epsilon': 0.001,
            'min_rotation_clearance_m': 0.30,
            'max_total_rotation_rad': 2.0 * math.pi,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not math.isfinite(rate) or not 15.0 <= rate <= 50.0:
            raise ValueError('publish_rate_hz must be within 15-50 Hz')
        params = {name: self.get_parameter(name).value for name in defaults}
        self.core = LiveAlignmentCore(
            enabled=self.get_parameter('enable_live_alignment').value,
            **params)
        self._raw_publisher = None
        qos = sensor_qos()
        self._state_publisher = self.create_publisher(
            String, '/fire_mission/live_alignment_state', qos)
        for name, topic in (
                ('mission', '/fire_mission/state'),
                ('preview', '/yolo/alignment_preview'),
                ('handoff', '/fire_mission/alignment_handoff_state'),
                ('gateway', '/fire_mission/live_nav_stop_state'),
                ('guard', '/fire_mission/nav2_cancel_guard_state'),
                ('ir', '/fire_mission/ir_approach_live_state'),
                ('review', '/fire_mission/post_spray_review'),
                ('actuator', '/fire_sensors/actuator_state')):
            self.create_subscription(
                String, topic,
                lambda msg, key=name: self._on_json(key, msg), qos)
        self.create_subscription(
            Bool, '/fire_mission/live_alignment_arm',
            self._on_arm, arm_qos())
        self.create_subscription(
            Odometry, '/odom', self._on_odom, qos)
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos)
        self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd, qos)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._on_status, action_status_qos())
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().warning(
            '[LIVE ALIGNMENT] enabled=%s; default OFF, exact cancel proof '
            'and separate operator heartbeat required' % self.core.enabled)

    def _on_json(self, name, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self.core.update(name, payload, time.monotonic())

    def _on_arm(self, message):
        self.core.update_arm(message.data, time.monotonic())

    @staticmethod
    def _values(twist):
        return (twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z)

    def _on_odom(self, message):
        received = admit_odom(self, message, self.core.input_timeout)
        if received is None:
            return
        self.core.update_odom(
            self._values(message.twist.twist), received)

    def _on_cmd(self, message):
        self.core.update_cmd(self._values(message), time.monotonic())

    def _on_scan(self, message):
        self.core.update_scan(message.angle_min, message.angle_max,
                              message.ranges, time.monotonic(),
                              range_max=message.range_max)

    def _on_status(self, message):
        self.core.update_statuses([
            (bytes(item.goal_info.goal_id.uuid).hex(), item.status)
            for item in message.status_list])

    def _sources(self, topic):
        return [(info.node_namespace, info.node_name, info.topic_type,
                 info.qos_profile.reliability)
                for info in self.get_publishers_info_by_topic(topic)]

    def _tick(self):
        now = time.monotonic()
        counts = {
            name: self.count_publishers(topic)
            for name, topic in (
                ('mission', '/fire_mission/state'),
                ('preview', '/yolo/alignment_preview'),
                ('handoff', '/fire_mission/alignment_handoff_state'),
                ('gateway', '/fire_mission/live_nav_stop_state'),
                ('guard', '/fire_mission/nav2_cancel_guard_state'),
                ('odom', '/odom'), ('cmd', '/cmd_vel'),
                ('scan', '/scan'),
                ('status', '/navigate_to_pose/_action/status'),
                ('arm', '/fire_mission/live_alignment_arm'),
                ('review', '/fire_mission/post_spray_review'),
                ('actuator', '/fire_sensors/actuator_state'),
                ('pump', '/pump_cmd'))
        }
        ir_sources = self._sources('/fire_mission/ir_approach_live_state')
        counts['ir'] = len(ir_sources)
        counts['ir_source_ok'] = ir_sources == [
            ('/', 'ir_approach_live', 'std_msgs/msg/String',
             ReliabilityPolicy.BEST_EFFORT)]
        counts['review_source_ok'] = self._sources(
            '/fire_mission/post_spray_review') == [
                ('/', 'post_spray_review', 'std_msgs/msg/String',
                 ReliabilityPolicy.BEST_EFFORT)]
        counts['actuator_source_ok'] = self._sources(
            '/fire_sensors/actuator_state') == [
                ('/', 'serial_bridge_node', 'std_msgs/msg/String',
                 ReliabilityPolicy.BEST_EFFORT)]
        counts['mission_source_ok'] = self._sources(
            '/fire_mission/state') == [
                ('/', 'fire_mission_supervisor', 'std_msgs/msg/String',
                 ReliabilityPolicy.BEST_EFFORT)]
        angular, payload, action = self.core.step(
            now, self._raw_publisher is not None,
            self._sources('/cmd_vel_raw'),
            self._sources('/fire_mission/nav2_cmd_vel_shadow'),
            self._sources('/cmd_vel'), counts)
        if action == 'CREATE':
            self._raw_publisher = self.create_publisher(
                Twist, '/cmd_vel_raw', sensor_qos())
            self.get_logger().warning(
                '[LIVE ALIGNMENT] acquired one-shot raw publisher; zero '
                'until exclusive graph ownership is discovered')
        if self._raw_publisher is not None:
            command = Twist()
            command.angular.z = angular
            self._raw_publisher.publish(command)
        if action == 'RELEASE' and self._raw_publisher is not None:
            self.destroy_publisher(self._raw_publisher)
            self._raw_publisher = None
            self.get_logger().warning(
                '[LIVE ALIGNMENT] zero/stationary hold complete; raw '
                'publisher released and one-shot attempt finished')
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        self._state_publisher.publish(message)

    def destroy_node(self):
        if self._raw_publisher is not None:
            for _ in range(6):
                if not rclpy.ok():
                    break
                self._raw_publisher.publish(Twist())
                time.sleep(0.05)
        return super().destroy_node()


def main(args=None):
    """Run the optional body-alignment command owner on the laptop."""
    def stop_handler(_signum, _frame):
        raise KeyboardInterrupt()

    previous_int = signal.getsignal(signal.SIGINT)
    previous_term = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    node = None
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = LiveAlignmentTakeover()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)


if __name__ == '__main__':
    main()
