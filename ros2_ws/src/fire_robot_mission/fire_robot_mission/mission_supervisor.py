"""Observe Nav2 and fire perception without commanding the robot."""

import json
import math
import time

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
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


class MissionSupervisorCore:
    """Pure state machine used by the ROS node and unit tests."""

    def __init__(
            self, detection_timeout_s=0.5, alignment_timeout_s=0.5,
            odom_timeout_s=0.5, command_timeout_s=0.5,
            fire_acquire_hold_s=0.5, centered_hold_s=0.5,
            max_source_age_ms=300.0, max_future_skew_ms=20.0,
            stationary_linear_speed_m_s=0.02,
            stationary_angular_speed_rad_s=0.05,
            turn_comparison_epsilon_rad_s=0.03):
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
        }
        invalid = [name for name, value in positive.items() if value <= 0.0]
        if invalid:
            raise ValueError(
                'mission supervisor parameters must be positive: '
                + ', '.join(invalid)
            )
        self.detection_timeout = float(detection_timeout_s)
        self.alignment_timeout = float(alignment_timeout_s)
        self.odom_timeout = float(odom_timeout_s)
        self.command_timeout = float(command_timeout_s)
        self.fire_acquire_hold = float(fire_acquire_hold_s)
        self.centered_hold = float(centered_hold_s)
        self.max_source_age = float(max_source_age_ms)
        self.max_future_skew = float(max_future_skew_ms)
        self.stationary_linear = float(stationary_linear_speed_m_s)
        self.stationary_angular = float(stationary_angular_speed_rad_s)
        self.turn_epsilon = float(turn_comparison_epsilon_rad_s)

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
        self._goal_status_received = None
        self._candidate_since = None
        self._centered_since = None
        self._ever_acquired = False
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
        """Store integer status values from NavigateToPose."""
        self._goal_statuses = [int(status) for status in statuses]
        self._goal_status_received = float(now)

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

        alignment_state, direction, fire_angular = (
            self._alignment_values())
        intent = 'HOLD_STOP'
        if self._detection_received is None:
            state = 'WAITING_FOR_INPUT'
        elif hard_blockers:
            state = 'FAULT'
            intent = 'REQUEST_SAFE_STOP'
            self._candidate_since = None
            self._centered_since = None
        elif target_present:
            if self._candidate_since is None:
                self._candidate_since = now
            acquired_for = now - self._candidate_since
            if acquired_for < self.fire_acquire_hold:
                state = 'FIRE_CANDIDATE'
                intent = 'KEEP_NAV2' if nav_active else 'HOLD_STOP'
                self._centered_since = None
            else:
                self._ever_acquired = True
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
            self._candidate_since = None
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
        self.declare_parameter('state_topic', '/fire_mission/state')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('detection_timeout_s', 0.50)
        self.declare_parameter('alignment_timeout_s', 0.50)
        self.declare_parameter('odom_timeout_s', 0.50)
        self.declare_parameter('command_timeout_s', 0.50)
        self.declare_parameter('fire_acquire_hold_s', 0.50)
        self.declare_parameter('centered_hold_s', 0.50)
        self.declare_parameter('max_source_age_ms', 300.0)
        self.declare_parameter('max_future_skew_ms', 20.0)
        self.declare_parameter('stationary_linear_speed_m_s', 0.02)
        self.declare_parameter(
            'stationary_angular_speed_rad_s', 0.05)
        self.declare_parameter(
            'turn_comparison_epsilon_rad_s', 0.03)

    def _read_parameters(self):
        topic_names = [
            'detections_topic',
            'alignment_topic',
            'odom_topic',
            'nav_command_topic',
            'raw_command_topic',
            'nav_status_topic',
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
                'centered_hold_s',
                'max_source_age_ms',
                'max_future_skew_ms',
                'stationary_linear_speed_m_s',
                'stationary_angular_speed_rad_s',
                'turn_comparison_epsilon_rad_s',
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
        self._core.update_odom(
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
            self._now(),
        )

    def _on_nav_command(self, message):
        self._core.update_nav_command(
            message.linear.x, message.angular.z, self._now())

    def _on_raw_command(self, message):
        self._core.update_raw_command(
            message.linear.x, message.angular.z, self._now())

    def _on_nav_status(self, message):
        self._core.update_goal_statuses(
            [item.status for item in message.status_list], self._now())

    def _publish_state(self):
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
