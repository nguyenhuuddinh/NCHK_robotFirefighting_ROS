"""
Exercise exclusive motion ownership without commanding the robot.

The node chooses between the observed Nav2 command and the admitted fire
alignment shadow command.  Its output remains under ``/fire_mission`` and is
never connected to the Pi SafetyGate or the ESP32 motor controller.
"""

import json
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fire_robot_mission.mission_supervisor import sensor_qos


FORBIDDEN_COMMAND_TOPICS = {
    '/cmd_vel',
    '/cmd_vel_raw',
    '/cmd_vel_nav',
    '/pump_cmd',
    '/fire_target',
}
HANDOFF_ACTIVE_STATES = {
    'ARMED_SHADOW_TRACKING',
    'ARMED_SHADOW_CENTERED',
}
ALIGNMENT_MISSION_STATES = {
    'FIRE_TRACKING',
    'CENTERED_HOLD',
    'ALIGNMENT_READY',
}
KNOWN_MISSION_STATES = {
    'WAITING_FOR_INPUT',
    'FAULT',
    'FIRE_CANDIDATE',
    'FIRE_TRACKING',
    'CENTERED_HOLD',
    'ALIGNMENT_READY',
    'WAITING_FOR_ALIGNMENT',
    'TARGET_LOST',
    'NAV2_NAVIGATING',
    'IDLE',
}


def _finite_number(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _age(received_at, now):
    if received_at is None:
        return None
    return max(0.0, float(now) - float(received_at))


def _age_ms(received_at, now):
    age = _age(received_at, now)
    return None if age is None else round(age * 1000.0, 1)


class MotionArbiterShadowCore:
    """Pure, fail-closed state machine for future motion ownership."""

    def __init__(
            self, mission_timeout_s=0.35, handoff_timeout_s=0.35,
            nav_command_timeout_s=0.35,
            alignment_command_timeout_s=0.35,
            max_nav_linear_speed_m_s=0.10,
            max_nav_angular_speed_rad_s=0.8,
            max_alignment_angular_speed_rad_s=0.35,
            command_match_epsilon=0.01):
        positive = {
            'mission_timeout_s': mission_timeout_s,
            'handoff_timeout_s': handoff_timeout_s,
            'nav_command_timeout_s': nav_command_timeout_s,
            'alignment_command_timeout_s': alignment_command_timeout_s,
            'max_nav_linear_speed_m_s': max_nav_linear_speed_m_s,
            'max_nav_angular_speed_rad_s': max_nav_angular_speed_rad_s,
            'max_alignment_angular_speed_rad_s': (
                max_alignment_angular_speed_rad_s),
            'command_match_epsilon': command_match_epsilon,
        }
        invalid = [name for name, value in positive.items()
                   if value <= 0.0]
        if invalid:
            raise ValueError(
                'motion arbiter shadow parameters must be positive: '
                + ', '.join(invalid))

        self.mission_timeout = float(mission_timeout_s)
        self.handoff_timeout = float(handoff_timeout_s)
        self.nav_timeout = float(nav_command_timeout_s)
        self.alignment_timeout = float(alignment_command_timeout_s)
        self.max_nav_linear = float(max_nav_linear_speed_m_s)
        self.max_nav_angular = float(max_nav_angular_speed_rad_s)
        self.max_alignment_angular = float(
            max_alignment_angular_speed_rad_s)
        self.command_epsilon = float(command_match_epsilon)

        self._mission = None
        self._mission_received = None
        self._mission_error = ''
        self._handoff = None
        self._handoff_received = None
        self._handoff_error = ''
        self._nav_command = None
        self._nav_command_received = None
        self._alignment_command = None
        self._alignment_command_received = None

    def update_mission(self, payload, now):
        self._mission = payload
        self._mission_received = float(now)
        self._mission_error = ''

    def mark_mission_error(self, error, now):
        self._mission = None
        self._mission_received = float(now)
        self._mission_error = str(error)

    def update_handoff(self, payload, now):
        self._handoff = payload
        self._handoff_received = float(now)
        self._handoff_error = ''

    def mark_handoff_error(self, error, now):
        self._handoff = None
        self._handoff_received = float(now)
        self._handoff_error = str(error)

    @staticmethod
    def _command(linear_x, angular_z, other_components):
        return {
            'linear_x': float(linear_x),
            'angular_z': float(angular_z),
            'other_components': tuple(
                float(value) for value in other_components),
        }

    def update_nav_command(
            self, linear_x, angular_z, now, other_components=()):
        self._nav_command = self._command(
            linear_x, angular_z, other_components)
        self._nav_command_received = float(now)

    def update_alignment_command(
            self, linear_x, angular_z, now, other_components=()):
        self._alignment_command = self._command(
            linear_x, angular_z, other_components)
        self._alignment_command_received = float(now)

    def _mission_status(self, now):
        blockers = []
        if self._mission_received is None:
            return blockers + ['MISSION_NOT_RECEIVED'], False, '', '', ''
        if _age(self._mission_received, now) > self.mission_timeout:
            blockers.append('MISSION_TIMEOUT')
        if self._mission_error:
            return blockers + ['MISSION_INVALID_JSON'], False, '', '', ''
        if not isinstance(self._mission, dict):
            return blockers + ['MISSION_INVALID_PAYLOAD'], False, '', '', ''

        mission = self._mission
        state = str(mission.get('state', 'missing'))
        if mission.get('observe_only') is not True:
            blockers.append('MISSION_NOT_OBSERVE_ONLY')
        if state not in KNOWN_MISSION_STATES:
            blockers.append('MISSION_STATE_UNKNOWN')

        decision = mission.get('decision') or {}
        intent = str(decision.get('intent', 'missing'))
        if decision.get('command_published') is not False:
            blockers.append('MISSION_COMMAND_CONTRACT')
        if intent == 'REQUEST_SAFE_STOP':
            blockers.append('MISSION_REQUESTED_SAFE_STOP')

        health = mission.get('health') or {}
        if health.get('status') != 'OK' or health.get('blockers'):
            blockers.append('MISSION_HEALTH_BLOCKED')

        robot = mission.get('robot') or {}
        if robot.get('odom_fresh') is not True:
            blockers.append('ODOM_NOT_FRESH')

        navigation = mission.get('navigation') or {}
        goal_active = navigation.get('goal_active')
        if not isinstance(goal_active, bool):
            blockers.append('NAV2_GOAL_STATE_UNKNOWN')
            goal_active = False
        observed_owner = str(
            navigation.get('observed_motion_owner', 'missing'))
        expected_owner = 'NAV2' if goal_active else 'NONE'
        if observed_owner != expected_owner:
            blockers.append('MISSION_MOTION_OWNER_MISMATCH')

        safety = mission.get('safety') or {}
        if safety.get('motion_command_publisher') is not False or \
                safety.get('actuator_command_publisher') is not False or \
                safety.get('nav2_action_client') is not False or \
                safety.get('collision_check_bypassed') is not False:
            blockers.append('MISSION_SAFETY_CONTRACT')
        return blockers, goal_active, state, intent, observed_owner

    def _handoff_status(self, now):
        blockers = []
        if self._handoff_received is None:
            return blockers + ['HANDOFF_NOT_RECEIVED'], False, '', 0.0
        if _age(self._handoff_received, now) > self.handoff_timeout:
            blockers.append('HANDOFF_TIMEOUT')
        if self._handoff_error:
            return blockers + ['HANDOFF_INVALID_JSON'], False, '', 0.0
        if not isinstance(self._handoff, dict):
            return blockers + ['HANDOFF_INVALID_PAYLOAD'], False, '', 0.0

        handoff = self._handoff
        state = str(handoff.get('state', 'missing'))
        active_value = handoff.get('active')
        if not isinstance(active_value, bool):
            blockers.append('HANDOFF_ACTIVE_STATE_UNKNOWN')
        active = active_value is True
        if handoff.get('shadow_only') is not True:
            blockers.append('HANDOFF_NOT_SHADOW_ONLY')

        safety = handoff.get('safety') or {}
        if safety.get('real_motion_output') is not False or \
                safety.get('actuator_command_publisher') is not False or \
                safety.get('nav2_action_client') is not False or \
                safety.get('requires_arm_heartbeat') is not True:
            blockers.append('HANDOFF_SAFETY_CONTRACT')

        admission = handoff.get('admission') or {}
        admission_blockers = admission.get('blockers')
        if not isinstance(admission_blockers, list):
            blockers.append('HANDOFF_BLOCKER_LIST_INVALID')
            admission_blockers = []
        if active:
            if state not in HANDOFF_ACTIVE_STATES:
                blockers.append('HANDOFF_ACTIVE_STATE_MISMATCH')
            if admission.get('eligible') is not True or admission_blockers:
                blockers.append('HANDOFF_ACTIVE_ADMISSION_MISMATCH')
        elif state in HANDOFF_ACTIVE_STATES:
            blockers.append('HANDOFF_INACTIVE_STATE_MISMATCH')

        command = handoff.get('shadow_command') or {}
        status_linear = command.get('linear_x_m_s')
        status_angular = command.get('angular_z_rad_s')
        if not _finite_number(status_linear) or \
                not _finite_number(status_angular):
            blockers.append('HANDOFF_COMMAND_INVALID')
            status_angular = 0.0
        elif abs(float(status_linear)) > self.command_epsilon:
            blockers.append('HANDOFF_LINEAR_COMMAND_NONZERO')
        if not active and _finite_number(status_angular) and \
                abs(float(status_angular)) > self.command_epsilon:
            blockers.append('HANDOFF_INACTIVE_COMMAND_NONZERO')
        return blockers, active, state, float(status_angular)

    def _command_status(self, command, received_at, timeout, prefix, now):
        blockers = []
        if received_at is None or command is None:
            return [f'{prefix}_NOT_RECEIVED']
        if _age(received_at, now) > timeout:
            blockers.append(f'{prefix}_TIMEOUT')
        values = (
            command['linear_x'], command['angular_z'],
            *command['other_components'],
        )
        if not all(_finite_number(value) for value in values):
            blockers.append(f'{prefix}_NONFINITE')
            return blockers
        if any(abs(value) > self.command_epsilon
               for value in command['other_components']):
            blockers.append(f'{prefix}_NONPLANAR')
        return blockers

    @staticmethod
    def _clamp(value, limit):
        output = max(-limit, min(limit, value))
        return output, output != value

    def evaluate(self, now):
        """Return planar shadow command and one JSON-safe ownership state."""
        now = float(now)
        mission_blockers, nav_requested, mission_state, intent, owner = (
            self._mission_status(now))
        handoff_blockers, alignment_requested, handoff_state, \
            handoff_angular = self._handoff_status(now)
        blockers = mission_blockers + handoff_blockers

        if nav_requested and alignment_requested:
            blockers.append('OWNERSHIP_REQUEST_CONFLICT')

        state = 'SAFE_STOP'
        selected_owner = 'NONE'
        linear = 0.0
        angular = 0.0
        linear_clamped = False
        angular_clamped = False

        if not blockers and alignment_requested:
            if mission_state not in ALIGNMENT_MISSION_STATES:
                blockers.append('MISSION_NOT_ALIGNMENT_READY')
            if owner != 'NONE':
                blockers.append('OTHER_MOTION_OWNER_ACTIVE')
            blockers.extend(self._command_status(
                self._alignment_command,
                self._alignment_command_received,
                self.alignment_timeout,
                'ALIGNMENT_COMMAND',
                now,
            ))
            if not blockers:
                alignment_linear = self._alignment_command['linear_x']
                alignment_angular = self._alignment_command['angular_z']
                if abs(alignment_linear) > self.command_epsilon:
                    blockers.append('ALIGNMENT_LINEAR_COMMAND_NONZERO')
                if abs(alignment_angular - handoff_angular) > \
                        self.command_epsilon:
                    blockers.append('ALIGNMENT_COMMAND_STATE_MISMATCH')
            if not blockers:
                angular, angular_clamped = self._clamp(
                    alignment_angular, self.max_alignment_angular)
                state = 'ALIGNMENT'
                selected_owner = 'ALIGNMENT'

        elif not blockers and nav_requested:
            if intent != 'KEEP_NAV2':
                blockers.append('MISSION_DID_NOT_KEEP_NAV2')
            if owner != 'NAV2':
                blockers.append('NAVIGATION_OWNER_NOT_CONFIRMED')
            blockers.extend(self._command_status(
                self._nav_command,
                self._nav_command_received,
                self.nav_timeout,
                'NAV_COMMAND',
                now,
            ))
            if not blockers:
                linear, linear_clamped = self._clamp(
                    self._nav_command['linear_x'], self.max_nav_linear)
                angular, angular_clamped = self._clamp(
                    self._nav_command['angular_z'], self.max_nav_angular)
                state = 'NAVIGATION'
                selected_owner = 'NAVIGATION'

        elif not blockers:
            if mission_state == 'IDLE' and intent == 'HOLD_STOP':
                state = 'IDLE'
            else:
                blockers.append('NO_OWNER_FOR_MISSION_STATE')

        blockers = sorted(set(blockers))
        if blockers:
            state = 'SAFE_STOP'
            selected_owner = 'NONE'
            linear = 0.0
            angular = 0.0
            linear_clamped = False
            angular_clamped = False

        active = state in {'NAVIGATION', 'ALIGNMENT'}
        status = {
            'schema_version': 1,
            'shadow_only': True,
            'state': state,
            'owner': selected_owner,
            'active': active,
            'requests': {
                'navigation': nav_requested,
                'alignment': alignment_requested,
                'conflict': nav_requested and alignment_requested,
            },
            'mission': {
                'state': mission_state,
                'intent': intent,
                'observed_motion_owner': owner,
                'receive_age_ms': _age_ms(
                    self._mission_received, now),
            },
            'handoff': {
                'state': handoff_state,
                'active': alignment_requested,
                'receive_age_ms': _age_ms(
                    self._handoff_received, now),
            },
            'sources': {
                'nav_command_receive_age_ms': _age_ms(
                    self._nav_command_received, now),
                'alignment_command_receive_age_ms': _age_ms(
                    self._alignment_command_received, now),
            },
            'admission': {
                'accepted': not blockers,
                'blockers': blockers,
            },
            'shadow_command': {
                'linear_x_m_s': round(linear, 4),
                'angular_z_rad_s': round(angular, 4),
                'linear_clamped': linear_clamped,
                'angular_clamped': angular_clamped,
                'nonzero': abs(linear) > 0.0 or abs(angular) > 0.0,
            },
            'limits': {
                'nav_linear_m_s': self.max_nav_linear,
                'nav_angular_rad_s': self.max_nav_angular,
                'alignment_angular_rad_s': self.max_alignment_angular,
            },
            'safety': {
                'real_motion_output': False,
                'actuator_command_publisher': False,
                'nav2_action_client': False,
                'exclusive_owner': True,
                'fail_closed_zero': state == 'SAFE_STOP',
            },
        }
        return linear, angular, status


class MotionArbiterShadow(Node):
    """Publish only an isolated shadow result from exclusive ownership."""

    def __init__(self):
        super().__init__('motion_arbiter_shadow')
        self._declare_parameters()
        self._read_parameters()
        self._core = MotionArbiterShadowCore(**self._core_parameters)
        self._validate_output_topics()

        qos = sensor_qos()
        self._command_publisher = self.create_publisher(
            Twist, self.shadow_command_topic, qos)
        self._state_publisher = self.create_publisher(
            String, self.arbiter_state_topic, qos)
        self.create_subscription(
            String, self.mission_state_topic, self._on_mission, qos)
        self.create_subscription(
            String, self.handoff_state_topic, self._on_handoff, qos)
        self.create_subscription(
            Twist, self.nav_command_topic, self._on_nav_command, qos)
        self.create_subscription(
            Twist, self.alignment_command_topic,
            self._on_alignment_command, qos)
        self.create_timer(1.0 / self.publish_rate, self._publish)
        self.get_logger().warning(
            '[ARBITER] SHADOW-ONLY: output is isolated from /cmd_vel_raw; '
            'this node cannot move the robot')

    def _declare_parameters(self):
        self.declare_parameter('mission_state_topic', '/fire_mission/state')
        self.declare_parameter(
            'handoff_state_topic',
            '/fire_mission/alignment_handoff_state')
        self.declare_parameter(
            'nav_command_topic', '/fire_mission/nav2_cmd_vel_shadow')
        self.declare_parameter(
            'alignment_command_topic',
            '/fire_mission/alignment_cmd_vel_shadow')
        self.declare_parameter(
            'shadow_command_topic',
            '/fire_mission/cmd_vel_arbiter_shadow')
        self.declare_parameter(
            'arbiter_state_topic', '/fire_mission/motion_arbiter_state')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('mission_timeout_s', 0.35)
        self.declare_parameter('handoff_timeout_s', 0.35)
        self.declare_parameter('nav_command_timeout_s', 0.35)
        self.declare_parameter('alignment_command_timeout_s', 0.35)
        self.declare_parameter('max_nav_linear_speed_m_s', 0.10)
        self.declare_parameter('max_nav_angular_speed_rad_s', 0.8)
        self.declare_parameter(
            'max_alignment_angular_speed_rad_s', 0.35)
        self.declare_parameter('command_match_epsilon', 0.01)

    def _read_parameters(self):
        topics = (
            'mission_state_topic',
            'handoff_state_topic',
            'nav_command_topic',
            'alignment_command_topic',
            'shadow_command_topic',
            'arbiter_state_topic',
        )
        for name in topics:
            setattr(self, name, str(self.get_parameter(name).value))
            if not getattr(self, name):
                raise ValueError(f'{name} cannot be empty')
        self.publish_rate = float(
            self.get_parameter('publish_rate_hz').value)
        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self._core_parameters = {
            name: float(self.get_parameter(name).value)
            for name in (
                'mission_timeout_s',
                'handoff_timeout_s',
                'nav_command_timeout_s',
                'alignment_command_timeout_s',
                'max_nav_linear_speed_m_s',
                'max_nav_angular_speed_rad_s',
                'max_alignment_angular_speed_rad_s',
                'command_match_epsilon',
            )
        }

    def _validate_output_topics(self):
        nav_input_topic = self.resolve_topic_name(self.nav_command_topic)
        alignment_input_topic = self.resolve_topic_name(
            self.alignment_command_topic)
        command_topic = self.resolve_topic_name(self.shadow_command_topic)
        for name, topic in (
                ('nav_command_topic', nav_input_topic),
                ('alignment_command_topic', alignment_input_topic)):
            if topic in FORBIDDEN_COMMAND_TOPICS or \
                    not topic.startswith('/fire_mission/') or \
                    not topic.endswith('_shadow'):
                raise ValueError(
                    f'{name} must stay isolated under /fire_mission/ '
                    'and end with _shadow')
        if command_topic in FORBIDDEN_COMMAND_TOPICS or \
                not command_topic.startswith('/fire_mission/') or \
                not command_topic.endswith('_shadow'):
            raise ValueError(
                'shadow_command_topic must stay isolated under '
                '/fire_mission/ and end with _shadow')
        state_topic = self.resolve_topic_name(self.arbiter_state_topic)
        if state_topic in FORBIDDEN_COMMAND_TOPICS:
            raise ValueError('arbiter_state_topic cannot be a command topic')
        if command_topic == state_topic:
            raise ValueError('arbiter output topics must be different')
        if command_topic in {nav_input_topic, alignment_input_topic}:
            raise ValueError(
                'arbiter shadow output cannot reuse a command input topic')

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

    def _on_mission(self, message):
        self._decode_json(
            message,
            self._core.mark_mission_error,
            self._core.update_mission,
        )

    def _on_handoff(self, message):
        self._decode_json(
            message,
            self._core.mark_handoff_error,
            self._core.update_handoff,
        )

    @staticmethod
    def _other_components(message):
        return (
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
        )

    def _on_nav_command(self, message):
        self._core.update_nav_command(
            message.linear.x,
            message.angular.z,
            self._now(),
            self._other_components(message),
        )

    def _on_alignment_command(self, message):
        self._core.update_alignment_command(
            message.linear.x,
            message.angular.z,
            self._now(),
            self._other_components(message),
        )

    def _publish(self):
        linear, angular, status = self._core.evaluate(self._now())
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._command_publisher.publish(command)
        state = String()
        state.data = json.dumps(
            status, separators=(',', ':'), ensure_ascii=True)
        self._state_publisher.publish(state)


def main(args=None):
    """Run the shadow-only motion arbiter."""
    rclpy.init(args=args)
    node = None
    try:
        node = MotionArbiterShadow()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == '__main__':
    main()
