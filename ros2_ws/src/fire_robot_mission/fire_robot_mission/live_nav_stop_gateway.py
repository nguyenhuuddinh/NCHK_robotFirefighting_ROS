"""
One-shot Nav2 command gateway: navigate, stop, then release raw ownership.

Live output is opt-in and additionally needs an operator heartbeat.  This node
never drives alignment or actuators.  After stopping it destroys its sole
``/cmd_vel_raw`` publisher, allowing the existing shadow-isolated CancelGoal
guard to perform the already-tested UUID-specific cancellation.
"""

from collections import Counter
from copy import deepcopy
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

from fire_robot_mission.alignment_handoff_shadow import arm_qos
from fire_robot_mission.mission_supervisor import action_status_qos, sensor_qos
from fire_robot_mission.nav2_cancel_guard import (
    ACTIVE_STATUSES, EXPECTED_SHADOW_PUBLISHERS, _valid_uuid,
)
from fire_robot_mission.pump_demo_core import POSSIBLE_EARLY_EXTINGUISH


RAW_ENDPOINT = Counter({
    ('/', 'live_nav_stop_gateway', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.BEST_EFFORT): 1,
})
SAFETY_ENDPOINT = Counter({
    ('/', 'safety_watchdog', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.BEST_EFFORT): 1,
})
ACTUATOR_ENDPOINT = Counter({
    ('/', 'serial_bridge_node', 'std_msgs/msg/String',
     ReliabilityPolicy.BEST_EFFORT): 1,
})
ACTUATOR_MAX_SOURCE_AGE_S = 0.5
FIRE_STATES = {'FIRE_TRACKING', 'CENTERED_HOLD', 'ALIGNMENT_READY',
               'WAITING_FOR_ALIGNMENT', 'TARGET_LOST'}
NAV_STATES = {'NAV2_NAVIGATING', 'FIRE_CANDIDATE'}
CANONICAL_TOPICS = (
    '/cmd_vel_raw', '/cmd_vel', '/odom',
    '/fire_mission/state', '/fire_mission/nav2_cmd_vel_shadow',
    '/fire_mission/live_nav_arm', '/fire_mission/nav2_cancel_guard_state',
    '/fire_mission/live_nav_stop_state',
    '/fire_mission/live_alignment_state', '/fire_mission/pump_demo_state',
    '/fire_mission/post_spray_review',
    '/fire_sensors/actuator_state',
    '/navigate_to_pose/_action/status',
)


def _fresh(received, now, timeout):
    return received is not None and 0.0 <= now - received <= timeout


def _finite(values):
    return all(math.isfinite(value) for value in values)


class LiveNavStopCore:
    """Pure one-shot state machine; all time inputs use a monotonic clock."""

    def __init__(
            self, enabled=False, mission_timeout_s=0.35,
            nav_timeout_s=0.35, odom_timeout_s=0.35,
            arm_timeout_s=0.40, guard_timeout_s=0.35,
            stationary_hold_s=0.50, discovery_timeout_s=0.75,
            stationary_linear_m_s=0.02,
            stationary_angular_rad_s=0.05,
            command_zero_epsilon=0.001,
            max_nav_linear_m_s=0.10, max_nav_angular_rad_s=0.8):
        values = (mission_timeout_s, nav_timeout_s, odom_timeout_s,
                  arm_timeout_s, guard_timeout_s, stationary_hold_s,
                  discovery_timeout_s, stationary_linear_m_s,
                  stationary_angular_rad_s, command_zero_epsilon,
                  max_nav_linear_m_s,
                  max_nav_angular_rad_s)
        if any(not math.isfinite(float(v)) or float(v) <= 0 for v in values):
            raise ValueError('live gateway thresholds must be positive')
        if float(max_nav_angular_rad_s) > 0.8:
            raise ValueError(
                'live gateway Nav2 angular limit exceeds 0.8 rad/s')
        self.enabled = bool(enabled)
        self.mission_timeout = float(mission_timeout_s)
        self.nav_timeout = float(nav_timeout_s)
        self.odom_timeout = float(odom_timeout_s)
        self.arm_timeout = float(arm_timeout_s)
        self.guard_timeout = float(guard_timeout_s)
        self.stationary_hold = float(stationary_hold_s)
        self.discovery_timeout = float(discovery_timeout_s)
        self.stationary_linear = float(stationary_linear_m_s)
        self.stationary_angular = float(stationary_angular_rad_s)
        self.command_zero_epsilon = float(command_zero_epsilon)
        self.max_nav_linear = float(max_nav_linear_m_s)
        self.max_nav_angular = float(max_nav_angular_rad_s)
        self.mission = None
        self.mission_at = None
        self.nav = None
        self.nav_at = None
        self.odom = None
        self.odom_at = None
        self.cmd = None
        self.cmd_at = None
        self.arm = False
        self.arm_at = None
        self.guard = None
        self.guard_at = None
        self.ever_forwarded = False
        self.stop_latched = False
        self.fire_latched = False
        self.stop_reason = ''
        self.first_stop = None
        self.stop_since = None
        self.publisher_created_at = None
        self.released = False
        self.statuses = None
        self.status_at = None
        self.goal_uuid = None
        self.normal_terminal_uuid = None
        self.normal_terminal_status = None
        self.used_goal_uuids = set()
        self.cancel_confirmed_uuid = None
        self.alignment_completed_uuid = None
        self.pump_completed_uuid = None
        self.reviewed_clear_uuid = None
        self.pre_spray_clear_uuid = None
        self.pump_started_uuid = None
        self.actuator = None
        self.actuator_at = None
        self.clear_since = None

    def update_mission(self, payload, now):
        self.mission = payload
        self.mission_at = float(now)
        lifecycle = payload.get('lifecycle') if isinstance(payload, dict) \
            else None
        if (isinstance(payload, dict) and
                payload.get('schema_version') == 1 and
                payload.get('observe_only') is True and
                isinstance(lifecycle, dict) and
                lifecycle.get('pre_spray_clear_goal_uuid') ==
                self.goal_uuid and
                self.goal_uuid is not None):
            # This exact-goal proof persists in mission telemetry.  An IDLE
            # sample can be lost before the next goal becomes active.
            self.pre_spray_clear_uuid = self.goal_uuid

    def update_nav(self, values, now):
        self.nav = tuple(float(value) for value in values)
        self.nav_at = float(now)

    def update_odom(self, values, now):
        self.odom = tuple(float(value) for value in values)
        self.odom_at = float(now)

    def update_cmd(self, values, now):
        self.cmd = tuple(float(value) for value in values)
        self.cmd_at = float(now)

    def update_arm(self, value, now):
        self.arm = value is True
        self.arm_at = float(now)

    def update_guard(self, payload, now):
        self.guard = payload
        self.guard_at = float(now)
        if isinstance(payload, dict) and \
                payload.get('state') == 'CANCEL_CONFIRMED_STOPPED' and \
                payload.get('goal_uuid') == self.goal_uuid and \
                payload.get('blockers') == []:
            self.cancel_confirmed_uuid = self.goal_uuid

    def update_statuses(self, statuses, now):
        self.statuses = tuple((str(uid).lower(), int(status))
                              for uid, status in statuses)
        self.status_at = float(now)
        # Action status is event-driven and may later prune a terminal goal.
        # Latch only the exact current UUID, never a different active goal.
        if self.goal_uuid and not any(
                uid == self.goal_uuid and status in ACTIVE_STATUSES
                for uid, status in self.statuses):
            terminal = [status for uid, status in self.statuses
                        if uid == self.goal_uuid and status in (4, 5, 6)]
            if len(terminal) == 1:
                self.normal_terminal_uuid = self.goal_uuid
                self.normal_terminal_status = terminal[0]

    def update_alignment(self, payload, now):
        if isinstance(payload, dict) and \
                payload.get('state') == 'COMPLETE' and \
                payload.get('stop_reason') == 'TARGET_CENTERED' and \
                payload.get('publisher_owned') is False and \
                payload.get('goal_uuid') == self.goal_uuid:
            self.alignment_completed_uuid = self.goal_uuid

    def update_pump(self, payload, now):
        if (isinstance(payload, dict) and
                payload.get('state') == 'FIRING' and
                payload.get('pump_on') is True and
                payload.get('goal_uuid') == self.goal_uuid):
            self.pump_started_uuid = self.goal_uuid
        if isinstance(payload, dict) and \
                ((payload.get('state') == 'COMPLETE' and
                  payload.get('stop_reason') == 'PULSE_TIME_LIMIT') or
                 (payload.get('state') == 'ABORTED' and
                  payload.get('stop_reason') in
                  POSSIBLE_EARLY_EXTINGUISH)) and \
                payload.get('pump_on') is False and \
                payload.get('goal_uuid') == self.goal_uuid:
            self.pump_completed_uuid = self.goal_uuid

    def update_review(self, payload, now):
        if isinstance(payload, dict) and \
                payload.get('mode') == 'SHADOW_ONLY' and \
                payload.get('state') == 'NO_FIRE_EVIDENCE' and \
                payload.get('goal_uuid') == self.goal_uuid and \
                payload.get('pulse_index', 0) >= 1 and \
                payload.get('blockers') == []:
            self.reviewed_clear_uuid = self.goal_uuid

    def update_actuator(self, payload, now):
        self.actuator = payload
        self.actuator_at = float(now)

    def _actuator_off(self, now, actuator_ok):
        state = self.actuator if isinstance(self.actuator, dict) else {}
        source_age = state.get('receive_age_ms')
        elapsed = now - self.actuator_at if self.actuator_at is not None \
            else float('inf')
        return (actuator_ok and
                _fresh(self.actuator_at, now, self.odom_timeout) and
                type(state.get('schema_version')) is int and
                state['schema_version'] == 1 and
                state.get('mode') == 'COMMAND_TELEMETRY_ONLY' and
                type(source_age) in (int, float) and
                math.isfinite(source_age) and
                0 <= source_age / 1000.0 + elapsed <=
                ACTUATOR_MAX_SOURCE_AGE_S and
                state.get('pump_pwm_commanded_on') is False and
                state.get('servo_phase') == 'IDLE' and
                state.get('servo_pwm_commanded_on') is False)

    def _active_goal(self, now):
        # Action status is event-driven (and transient-local), not a velocity
        # heartbeat. Keep its exact active UUID until a newer status changes
        # it. Fresh Nav2 command, mission, odom, arm and graph ownership are
        # checked separately on every cycle by _nav_eligible().
        if self.status_at is None or now < self.status_at:
            return None
        active = [uid for uid, status in self.statuses or ()
                  if status in ACTIVE_STATUSES]
        return active[0] if len(active) == 1 and \
            _valid_uuid(active[0]) else None

    def _clear_for_next_goal(self, now, stationary, cmd_zero, actuator_ok):
        mission = self.mission if isinstance(self.mission, dict) else {}
        perception = mission.get('perception') or {}
        no_fire = (_fresh(self.mission_at, now, self.mission_timeout) and
                   mission.get('state') in ('IDLE', 'NAV2_NAVIGATING') and
                   perception.get('detected_now') is False and
                   perception.get('confirmed') is False)
        if not (no_fire and stationary and cmd_zero and actuator_ok):
            self.clear_since = None
        elif _fresh(self.actuator_at, now, self.odom_timeout):
            # A fresh ON/invalid report breaks the proof. A gap between OFF
            # reports does not: otherwise a slow telemetry publisher can
            # prevent the one-second stop hold from ever completing.
            if self._actuator_off(now, actuator_ok):
                if self.clear_since is None:
                    self.clear_since = now
            else:
                self.clear_since = None
        return (self.clear_since is not None and
                now - self.clear_since >= 1.0 and
                self._actuator_off(now, actuator_ok))

    def _cycle_clear_blockers(self, now, guard_count):
        """Explain the same exact-goal proofs used for rearming."""
        goal = self.goal_uuid
        if not _valid_uuid(goal):
            return ['OLD_GOAL_UUID_MISSING']
        normal_no_fire = (not self.fire_latched and
                          self.stop_reason == 'NAV_GOAL_TERMINAL_NO_FIRE' and
                          self.normal_terminal_uuid == goal and
                          self.normal_terminal_status in (4, 5, 6))
        fire_cycle_clear = (self.fire_latched and
                            self.cancel_confirmed_uuid == goal and
                            self.alignment_completed_uuid == goal and
                            self.pump_completed_uuid == goal and
                            self.reviewed_clear_uuid == goal)
        guard_reset = (guard_count == 1 and
                       _fresh(self.guard_at, now, self.guard_timeout) and
                       isinstance(self.guard, dict) and
                       self.guard.get('state') == 'WAITING_FOR_ADMISSION' and
                       self.guard.get('goal_uuid') is None and
                       self.guard.get('send_count') == 0)
        pre_spray_clear = (self.fire_latched and
                           self.cancel_confirmed_uuid == goal and
                           self.pre_spray_clear_uuid == goal and
                           self.pump_started_uuid != goal and guard_reset)
        if normal_no_fire or fire_cycle_clear or pre_spray_clear:
            return []
        if not self.fire_latched:
            return ['OLD_GOAL_TERMINAL_PROOF_INVALID']
        blockers = []
        if self.cancel_confirmed_uuid != goal:
            blockers.append('CANCEL_PROOF_MISSING')
        if self.pump_started_uuid == goal:
            if self.alignment_completed_uuid != goal:
                blockers.append('ALIGNMENT_PROOF_MISSING')
            if self.pump_completed_uuid != goal:
                blockers.append('PUMP_COMPLETION_PROOF_MISSING')
            if self.reviewed_clear_uuid != goal:
                blockers.append('POST_SPRAY_REVIEW_PROOF_MISSING')
        elif self.pre_spray_clear_uuid != goal:
            blockers.append('PRE_SPRAY_CLEAR_PROOF_MISSING')
        elif not guard_reset:
            blockers.append('CANCEL_GUARD_RESET_PROOF_MISSING')
        if not blockers:
            blockers.append('FIRE_CYCLE_PROOF_INCOMPLETE')
        return blockers

    def _next_goal_blockers(self, now, stationary, cmd_zero, raw_count,
                            pump_count, guard_count, actuator_ok,
                            clear_ready):
        if not self.released:
            return []
        blockers = []
        new_goal = self._active_goal(now)
        if raw_count:
            blockers.append('RAW_OWNER_PRESENT')
        if pump_count:
            blockers.append('PUMP_OWNER_PRESENT')
        blockers.extend(self._cycle_clear_blockers(now, guard_count))
        if not new_goal or new_goal == self.goal_uuid or \
                new_goal in self.used_goal_uuids:
            blockers.append('NEW_GOAL_UUID_MISSING_OR_REUSED')
        if not stationary:
            blockers.append('ODOM_NOT_STATIONARY')
        if not cmd_zero:
            blockers.append('CMD_VEL_NOT_ZERO')
        if not clear_ready:
            blockers.append('NO_FIRE_CLEAR_HOLD_INCOMPLETE')
        if not actuator_ok:
            blockers.append('ACTUATOR_PUBLISHER_INVALID')
        if not self._actuator_off(now, actuator_ok):
            blockers.append('ACTUATOR_OFF_TELEMETRY_MISSING_OR_STALE')
        return blockers

    def _rearm_if_new_goal(self, now, stationary, cmd_zero, raw_count,
                           pump_count=0, guard_count=0,
                           clear_ready=False, actuator_ok=False):
        new_goal = self._active_goal(now)
        if not (self.released and raw_count == 0 and pump_count == 0 and
                self.goal_uuid and
                new_goal and new_goal != self.goal_uuid and
                new_goal not in self.used_goal_uuids and
                not self._cycle_clear_blockers(now, guard_count) and
                clear_ready and
                stationary and cmd_zero and
                self._actuator_off(now, actuator_ok)):
            return False
        self.used_goal_uuids.add(self.goal_uuid)
        self.goal_uuid = new_goal
        self.normal_terminal_uuid = None
        self.normal_terminal_status = None
        self.ever_forwarded = False
        self.stop_latched = False
        self.fire_latched = False
        self.stop_reason = ''
        self.first_stop = None
        self.stop_since = None
        self.publisher_created_at = None
        self.released = False
        self.clear_since = None
        self.cancel_confirmed_uuid = None
        self.alignment_completed_uuid = None
        self.pump_completed_uuid = None
        self.reviewed_clear_uuid = None
        self.pre_spray_clear_uuid = None
        self.pump_started_uuid = None
        return True

    def publisher_created(self, now):
        self.publisher_created_at = float(now)

    def publisher_released(self):
        self.released = True
        self.publisher_created_at = None

    def _stationary(self, now):
        fresh = _fresh(self.odom_at, now, self.odom_timeout)
        values = self.odom
        stationary = bool(fresh and values is not None and len(values) == 6
                          and _finite(values) and all(
                              abs(v) <= self.stationary_linear
                              for v in values[:3]) and all(
                                  abs(v) <= self.stationary_angular
                                  for v in values[3:]))
        return fresh, stationary

    def _cmd_zero(self, now, safety_ok):
        return bool(safety_ok and
                    _fresh(self.cmd_at, now, self.odom_timeout) and
                    self.cmd is not None and len(self.cmd) == 6 and
                    _finite(self.cmd) and
                    all(abs(v) <= self.command_zero_epsilon
                        for v in self.cmd))

    def _nav_eligible(self, now, nav_ok, safety_ok, status_count,
                      mission_count, odom_count, arm_count, odom_fresh):
        blockers = []
        if mission_count != 1:
            blockers.append('MISSION_PUBLISHER_INVALID')
        if odom_count != 1:
            blockers.append('ODOM_PUBLISHER_INVALID')
        if arm_count != 1:
            blockers.append('LIVE_ARM_PUBLISHER_INVALID')
        if not _fresh(self.arm_at, now, self.arm_timeout) or not self.arm:
            blockers.append('LIVE_ARM_MISSING_OR_STALE')
        if not _fresh(self.mission_at, now, self.mission_timeout) or \
                not isinstance(self.mission, dict):
            blockers.append('MISSION_MISSING_OR_STALE')
        else:
            mission = self.mission
            decision = mission.get('decision') or {}
            navigation = mission.get('navigation') or {}
            health = mission.get('health') or {}
            robot = mission.get('robot') or {}
            if mission.get('schema_version') != 1 or \
                    mission.get('observe_only') is not True or \
                    decision.get('command_published') is not False or \
                    (mission.get('safety') or {}).get(
                        'motion_command_publisher') is not False or \
                    (mission.get('safety') or {}).get(
                        'actuator_command_publisher') is not False or \
                    (mission.get('safety') or {}).get(
                        'nav2_action_client') is not False or \
                    (mission.get('safety') or {}).get(
                        'collision_check_bypassed') is not False:
                blockers.append('MISSION_CONTRACT_INVALID')
            if mission.get('state') not in NAV_STATES or \
                    decision.get('intent') != 'KEEP_NAV2' or \
                    navigation.get('goal_active') is not True or \
                    navigation.get('observed_motion_owner') != 'NAV2':
                blockers.append('NAVIGATION_NOT_OWNED')
            if health.get('status') != 'OK' or health.get('blockers') or \
                    robot.get('odom_fresh') is not True:
                blockers.append('MISSION_HEALTH_BLOCKED')
        if not odom_fresh:
            blockers.append('ODOM_NOT_FRESH')
        if not _fresh(self.nav_at, now, self.nav_timeout) or \
                self.nav is None or len(self.nav) != 6 or \
                not _finite(self.nav):
            blockers.append('NAV_COMMAND_MISSING_OR_INVALID')
        elif any(abs(v) > 1e-6 for v in (
                self.nav[1], self.nav[2], self.nav[3], self.nav[4])) or \
                abs(self.nav[0]) > self.max_nav_linear or \
                abs(self.nav[5]) > self.max_nav_angular:
            blockers.append('NAV_COMMAND_OUT_OF_BOUNDS')
        if not nav_ok:
            blockers.append('NAV_SHADOW_TOPOLOGY_INVALID')
        if not safety_ok:
            blockers.append('SAFETY_GATE_TOPOLOGY_INVALID')
        if status_count != 1:
            blockers.append('NAV_STATUS_PUBLISHER_INVALID')
        if self._active_goal(now) is None or self.goal_uuid is None:
            blockers.append('NAV_GOAL_UUID_UNAVAILABLE')
        return blockers

    def _capture_first_stop(self, now, blockers, nav_ok, raw_ok, safety_ok,
                            status_count, mission_count, odom_count, arm_count):
        """Freeze the first stop decision; diagnostics never grant admission."""
        if not self.stop_latched or self.first_stop is not None:
            return
        mission = self.mission if isinstance(self.mission, dict) else {}
        health = mission.get('health') or {}
        health = health if isinstance(health, dict) else {}
        health_blockers = health.get('blockers')
        # Keep external text bounded; avoid retaining an entire mission payload.
        if not isinstance(health_blockers, (list, tuple)):
            health_blockers = []
        def age_ms(received):
            age = None if received is None else (now - received) * 1000.0
            return age if age is None or math.isfinite(age) else None
        self.first_stop = {
            'monotonic_s': now,
            'cycle_goal_uuid': self.goal_uuid,
            'reason': self.stop_reason,
            'blockers': sorted(set(blockers)),
            'mission_state': mission.get('state'),
            'mission_health': {
                'status': health.get('status'),
                'blockers': [str(v)[:128] for v in health_blockers[:32]],
            },
            # Callback receive ages, NOT source/header ages. Null means no
            # currently admitted sample (including invalidated odometry).
            'receive_age_ms': {
                'mission': age_ms(self.mission_at),
                'nav_command': age_ms(self.nav_at),
                'odom_admitted': age_ms(self.odom_at),
                'arm': age_ms(self.arm_at),
                'cmd_vel': age_ms(self.cmd_at),
            },
            'timeout_ms': {
                'mission': self.mission_timeout * 1000.0,
                'nav_command': self.nav_timeout * 1000.0,
                'odom_admitted': self.odom_timeout * 1000.0,
                'arm': self.arm_timeout * 1000.0,
            },
            'topology': {
                'nav_shadow_ok': nav_ok, 'raw_owner_ok': raw_ok,
                'safety_gate_ok': safety_ok,
                'nav_status_publishers': status_count,
                'mission_publishers': mission_count,
                'odom_publishers': odom_count,
                'live_arm_publishers': arm_count,
            },
        }

    def _handoff_ready(self, now, stationary, cmd_zero, raw_count,
                       nav_ok, safety_ok, status_count, mission_count,
                       odom_count, guard_count):
        if not stationary or not cmd_zero or raw_count != 0 or \
                not nav_ok or not safety_ok or status_count != 1 or \
                mission_count != 1 or odom_count != 1 or \
                guard_count != 1 or not \
                _fresh(self.guard_at, now, self.guard_timeout) or \
                not isinstance(self.guard, dict) or \
                not _fresh(self.mission_at, now, self.mission_timeout) or \
                not isinstance(self.mission, dict):
            return False
        guard = self.guard
        mission = self.mission
        decision = mission.get('decision') or {}
        robot = mission.get('robot') or {}
        navigation = mission.get('navigation') or {}
        return (self.fire_latched and
                guard.get('schema_version') == 1 and
                guard.get('state') == 'CANCEL_CONFIRMED_STOPPED' and
                guard.get('enabled') is True and
                guard.get('send_count') == 1 and
                _valid_uuid(guard.get('goal_uuid')) and
                guard.get('cancel_service_accepted') is True and
                guard.get('cancel_status_seen') is True and
                guard.get('goal_status') == 5 and
                guard.get('arm_fresh') is True and
                guard.get('cmd_vel_fresh_zero') is True and
                guard.get('odom_fresh_stationary') is True and
                guard.get('nav_shadow_fingerprint_ok') is True and
                guard.get('raw_cmd_publishers') == 0 and
                guard.get('blockers') == [] and
                mission.get('schema_version') == 1 and
                mission.get('state') in {
                    'FIRE_TRACKING', 'CENTERED_HOLD', 'ALIGNMENT_READY'} and
                mission.get('observe_only') is True and
                decision.get('eligible_for_future_alignment_handoff') is True
                and decision.get('readiness_blockers') == [] and
                navigation.get('goal_active') is False and
                robot.get('odom_fresh') is True and
                robot.get('stationary') is True)

    def evaluate(self, now, publisher_owned, raw_sources, nav_sources,
                 cmd_sources, status_publishers, mission_publishers=0,
                 odom_publishers=0, arm_publishers=0,
                 guard_publishers=0, pump_publishers=0,
                 actuator_sources=()):
        """Return (Twist planar pair, state JSON, CREATE/RELEASE/None)."""
        now = float(now)
        raw_count = len(raw_sources)
        raw_ok = Counter(raw_sources) == (
            RAW_ENDPOINT if publisher_owned else Counter())
        nav_ok = Counter(nav_sources) == EXPECTED_SHADOW_PUBLISHERS
        safety_ok = Counter(cmd_sources) == SAFETY_ENDPOINT
        actuator_ok = Counter(actuator_sources) == ACTUATOR_ENDPOINT
        odom_fresh, stationary = self._stationary(now)
        cmd_zero = self._cmd_zero(now, safety_ok)
        mission = self.mission if isinstance(self.mission, dict) else {}
        mission_state = mission.get('state')
        active_goal = self._active_goal(now)
        if not self.released and self.goal_uuid is None and active_goal and \
                (mission.get('navigation') or {}).get('goal_active') is True:
            self.goal_uuid = active_goal
        if pump_publishers > 0 and self.fire_latched and self.goal_uuid:
            self.pump_started_uuid = self.goal_uuid
        # A fire seen before any Nav2 goal is not a completed handoff cycle.
        # Keep observing, but never release the gateway without goal identity.
        if self.goal_uuid is not None and active_goal == self.goal_uuid and \
                mission_state in FIRE_STATES:
            self.fire_latched = True
        blockers = self._nav_eligible(
            now, nav_ok, safety_ok, status_publishers,
            mission_publishers, odom_publishers, arm_publishers,
            odom_fresh)
        if self.goal_uuid is not None and active_goal is not None and \
                active_goal != self.goal_uuid and not self.released:
            blockers.append('NAV_GOAL_UUID_CHANGED')
        normal_done = (
            not self.fire_latched and
            self.normal_terminal_uuid == self.goal_uuid and
            self.normal_terminal_status in (4, 5, 6) and
            status_publishers == 1 and
            active_goal != self.goal_uuid)
        # Mission/nav callbacks may arrive before the action terminal event.
        # Keep stopping, but reclassify only after exact terminal evidence.
        if normal_done and self.stop_reason in (
                'NAV_INPUT_OR_TOPOLOGY_LOST', 'ARMING_INPUT_LOST'):
            self.stop_reason = 'NAV_GOAL_TERMINAL_NO_FIRE'
        action = None
        linear = angular = 0.0
        clear_ready = False
        rearm_blockers = []

        if not self.enabled:
            state = 'DISABLED'
            if publisher_owned:
                action = 'RELEASE'
        elif self.released:
            clear_ready = self._clear_for_next_goal(
                now, stationary, cmd_zero, actuator_ok)
            rearm_blockers = self._next_goal_blockers(
                now, stationary, cmd_zero, raw_count, pump_publishers,
                guard_publishers, actuator_ok, clear_ready)
            rearmed = self._rearm_if_new_goal(
                now, stationary, cmd_zero, raw_count, pump_publishers,
                guard_publishers, clear_ready, actuator_ok)
            if rearmed:
                state = 'WAITING_FOR_INPUT'
                rearm_blockers = []
            else:
                state = ('HANDOFF_READY' if self._handoff_ready(
                    now, stationary, cmd_zero, raw_count, nav_ok, safety_ok,
                    status_publishers, mission_publishers, odom_publishers,
                    guard_publishers) else 'RELEASED_STOPPED')
        else:
            if self.fire_latched or normal_done or \
                    (self.ever_forwarded and blockers):
                self.stop_latched = True
                if not self.stop_reason:
                    self.stop_reason = (
                        'FIRE_ACQUIRED' if self.fire_latched else
                        'NAV_GOAL_TERMINAL_NO_FIRE' if normal_done else
                        'NAV_INPUT_OR_TOPOLOGY_LOST')
            if self.stop_reason == 'NAV_GOAL_TERMINAL_NO_FIRE':
                self._clear_for_next_goal(
                    now, stationary, cmd_zero, actuator_ok)
            if publisher_owned and not raw_ok:
                discovery_pending = raw_count == 0 and \
                    self.publisher_created_at is not None and \
                    now - self.publisher_created_at < self.discovery_timeout
                if not discovery_pending:
                    self.stop_latched = True
                    self.stop_reason = 'RAW_OWNERSHIP_LOST'
            if self.stop_latched:
                state = 'STOPPING'
                if stationary and cmd_zero:
                    if self.stop_since is None:
                        self.stop_since = now
                else:
                    self.stop_since = None
                if not publisher_owned or (self.stop_since is not None and
                                           now - self.stop_since >=
                                           self.stationary_hold):
                    action = 'RELEASE' if publisher_owned else None
                    state = 'RELEASING'
                    if not publisher_owned:
                        self.released = True
            elif blockers:
                state = 'WAITING_FOR_INPUT'
                if publisher_owned:
                    self.stop_latched = True
                    self.stop_reason = 'ARMING_INPUT_LOST'
                    state = 'STOPPING'
            elif not publisher_owned:
                state = 'ARMING'
                if raw_count == 0:
                    action = 'CREATE'
                else:
                    state = 'WAITING_FOR_RAW_OWNER'
            elif not raw_ok:
                state = 'ARMING'
            else:
                self.ever_forwarded = True
                state = 'NAVIGATION'
                linear, angular = self.nav[0], self.nav[5]

        self._capture_first_stop(
            now, blockers, nav_ok, raw_ok, safety_ok, status_publishers,
            mission_publishers, odom_publishers, arm_publishers)
        payload = {
            'schema_version': 1,
            'state': state,
            'enabled': self.enabled,
            'one_shot': True,
            'cycle_goal_uuid': self.goal_uuid,
            'fire_latched': self.fire_latched,
            'ever_forwarded': self.ever_forwarded,
            'released': self.released,
            'stop_reason': self.stop_reason,
            'first_stop': deepcopy(self.first_stop),
            'handoff_ready': state == 'HANDOFF_READY',
            'nav_shadow_fingerprint_ok': nav_ok,
            'raw_owner_fingerprint_ok': raw_ok,
            'raw_publishers': raw_count,
            'safety_gate_fingerprint_ok': safety_ok,
            'nav_status_publishers': status_publishers,
            'mission_publishers': mission_publishers,
            'odom_publishers': odom_publishers,
            'live_arm_publishers': arm_publishers,
            'cancel_guard_publishers': guard_publishers,
            'actuator_fingerprint_ok': actuator_ok,
            'arm_fresh': _fresh(self.arm_at, now, self.arm_timeout) and
            self.arm,
            'odom_fresh': odom_fresh,
            'odom_stationary': stationary,
            'cmd_vel_fresh_zero': cmd_zero,
            'goal_active': (mission.get('navigation') or {}).get(
                'goal_active'),
            'command': {'linear_x_m_s': linear,
                        'angular_z_rad_s': angular},
            'blockers': sorted(set(blockers)),
            'rearm_blockers': rearm_blockers,
            'next_goal_clear_ready': bool(
                self.released and self.stop_reason ==
                'NAV_GOAL_TERMINAL_NO_FIRE' and not self.fire_latched and
                self.normal_terminal_uuid == self.goal_uuid and
                self.normal_terminal_status in (4, 5, 6) and clear_ready and
                raw_count == 0 and pump_publishers == 0 and nav_ok and
                safety_ok and status_publishers == 1 and
                mission_publishers == 1 and odom_publishers == 1 and
                arm_publishers == 1 and
                _fresh(self.arm_at, now, self.arm_timeout) and self.arm and
                actuator_ok),
            'safety': {
                'raw_command_publisher': publisher_owned,
                'actuator_command_publisher': False,
                'alignment_command_publisher': False,
                'fail_closed_zero': state not in {'NAVIGATION'},
            },
        }
        return linear, angular, payload, action


class LiveNavStopGateway(Node):
    """Publish to the SafetyGate input only after explicit live opt-in."""

    def __init__(self):
        super().__init__('live_nav_stop_gateway')
        self._odom_stamp_guard = OdomStampGuard()
        # A ROS CLI remap must not redirect this safety gate's live output or
        # make it observe a different mission/command stream.
        for topic in CANONICAL_TOPICS:
            if self.resolve_topic_name(topic) != topic:
                raise ValueError(
                    f'live gateway forbids remapping {topic}')
        self.declare_parameter('enable_live_output', False)
        self.declare_parameter('publish_rate_hz', 20.0)
        defaults = {
            'mission_timeout_s': 0.35, 'nav_timeout_s': 0.35,
            'odom_timeout_s': 0.35, 'arm_timeout_s': 0.40,
            'guard_timeout_s': 0.35, 'stationary_hold_s': 0.50,
            'discovery_timeout_s': 0.75,
            'stationary_linear_m_s': 0.02,
            'stationary_angular_rad_s': 0.05,
            'command_zero_epsilon': 0.001,
            'max_nav_linear_m_s': 0.10,
            'max_nav_angular_rad_s': 0.8,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate_hz must be positive')
        enabled = self.get_parameter('enable_live_output').value
        if not isinstance(enabled, bool):
            raise ValueError('enable_live_output must be a boolean')
        self.core = LiveNavStopCore(
            enabled=enabled,
            **{name: self.get_parameter(name).value for name in defaults})
        self._raw_publisher = None
        qos = sensor_qos()
        self._state_publisher = self.create_publisher(
            String, '/fire_mission/live_nav_stop_state', qos)
        self.create_subscription(
            String, '/fire_mission/state', self._on_mission, qos)
        self.create_subscription(
            Twist, '/fire_mission/nav2_cmd_vel_shadow', self._on_nav, qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, qos)
        self.create_subscription(
            Bool, '/fire_mission/live_nav_arm', self._on_arm, arm_qos())
        self.create_subscription(
            String, '/fire_mission/nav2_cancel_guard_state',
            self._on_guard, qos)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._on_status, action_status_qos())
        self.create_subscription(
            String, '/fire_mission/live_alignment_state',
            self._on_alignment, qos)
        self.create_subscription(
            String, '/fire_mission/pump_demo_state', self._on_pump, qos)
        self.create_subscription(
            String, '/fire_mission/post_spray_review',
            self._on_review, qos)
        self.create_subscription(
            String, '/fire_sensors/actuator_state', self._on_actuator, qos)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().warning(
            '[LIVE NAV GATEWAY] live_output=%s, one-shot, operator arm '
            'required; alignment and actuators remain disconnected' %
            self.core.enabled)

    @staticmethod
    def _values(twist):
        return (twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z)

    def _on_mission(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self.core.update_mission(payload, time.monotonic())

    def _on_nav(self, message):
        self.core.update_nav(self._values(message), time.monotonic())

    def _on_odom(self, message):
        received = admit_odom(self, message, self.core.odom_timeout)
        if received is None:
            return
        self.core.update_odom(
            self._values(message.twist.twist), received)

    def _on_cmd(self, message):
        self.core.update_cmd(self._values(message), time.monotonic())

    def _on_arm(self, message):
        self.core.update_arm(message.data, time.monotonic())

    def _on_guard(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self.core.update_guard(payload, time.monotonic())

    def _on_status(self, message):
        self.core.update_statuses([
            (bytes(item.goal_info.goal_id.uuid).hex(), item.status)
            for item in message.status_list], time.monotonic())

    @staticmethod
    def _json_payload(message):
        try:
            return json.loads(message.data)
        except (TypeError, ValueError):
            return None

    def _on_alignment(self, message):
        self.core.update_alignment(
            self._json_payload(message), time.monotonic())

    def _on_pump(self, message):
        self.core.update_pump(self._json_payload(message), time.monotonic())

    def _on_review(self, message):
        self.core.update_review(
            self._json_payload(message), time.monotonic())

    def _on_actuator(self, message):
        self.core.update_actuator(
            self._json_payload(message), time.monotonic())

    def _sources(self, topic):
        return [(info.node_namespace, info.node_name, info.topic_type,
                 info.qos_profile.reliability)
                for info in self.get_publishers_info_by_topic(topic)]

    def _tick(self):
        now = time.monotonic()
        linear, angular, payload, action = self.core.evaluate(
            now, self._raw_publisher is not None,
            self._sources('/cmd_vel_raw'),
            self._sources('/fire_mission/nav2_cmd_vel_shadow'),
            self._sources('/cmd_vel'),
            self.count_publishers('/navigate_to_pose/_action/status'),
            self.count_publishers('/fire_mission/state'),
            self.count_publishers('/odom'),
            self.count_publishers('/fire_mission/live_nav_arm'),
            self.count_publishers(
                '/fire_mission/nav2_cancel_guard_state'),
            self.count_publishers('/pump_cmd'),
            self._sources('/fire_sensors/actuator_state'))
        if action == 'CREATE':
            self._raw_publisher = self.create_publisher(
                Twist, '/cmd_vel_raw', sensor_qos())
            self.core.publisher_created(now)
            self.get_logger().warning(
                '[LIVE NAV GATEWAY] acquired raw publisher; zero until '
                'exclusive ROS graph ownership is visible')
        if self._raw_publisher is not None:
            command = Twist()
            command.linear.x = linear
            command.angular.z = angular
            self._raw_publisher.publish(command)
        if action == 'RELEASE' and self._raw_publisher is not None:
            self.destroy_publisher(self._raw_publisher)
            self._raw_publisher = None
            self.core.publisher_released()
            self.get_logger().warning(
                '[LIVE NAV GATEWAY] raw publisher released after STOP; '
                'new goal requires completed cycle and safe clear proof')
        state = String()
        state.data = json.dumps(payload, separators=(',', ':'))
        self._state_publisher.publish(state)

    def destroy_node(self):
        if self._raw_publisher is not None:
            # Graceful shutdown sends several zero frames before the Pi's
            # independent command timeout becomes the final fallback.
            for _ in range(6):
                if not rclpy.ok():
                    break
                self._raw_publisher.publish(Twist())
                time.sleep(0.05)
        return super().destroy_node()


def main(args=None):
    """Run the opt-in laptop gateway without touching Pi configuration."""
    def stop_handler(_signum, _frame):
        raise KeyboardInterrupt()

    previous_int = signal.getsignal(signal.SIGINT)
    previous_term = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    node = None
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = LiveNavStopGateway()
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
