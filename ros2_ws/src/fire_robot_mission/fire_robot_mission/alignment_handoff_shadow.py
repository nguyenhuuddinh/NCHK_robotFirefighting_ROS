"""
Exercise fire-alignment handoff admission without commanding the robot.

This node deliberately publishes ``Twist`` only on an isolated ``*_shadow``
topic.  It never owns ``/cmd_vel``, ``/cmd_vel_raw`` or ``/cmd_vel_nav``.
The arm input is a heartbeat: stopping the publisher makes the gate fail
closed after ``arm_timeout_s``.
"""

import json
import math
import re
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from fire_robot_mission.mission_supervisor import sensor_qos


FORBIDDEN_COMMAND_TOPICS = {
    '/cmd_vel',
    '/cmd_vel_raw',
    '/cmd_vel_nav',
    '/pump_cmd',
    '/fire_target',
}
TRACKING_STATES = {'ALIGNING_LEFT', 'ALIGNING_RIGHT'}
CENTERED_STATES = {'CENTERED'}
MISSION_HANDOFF_STATES = {
    'FIRE_TRACKING',
    'CENTERED_HOLD',
    'ALIGNMENT_READY',
}
GOAL_UUID_PATTERN = re.compile(r'^[0-9a-fA-F]{32}$')


def arm_qos():
    """Return a reliable, no-backlog profile for the arm heartbeat."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _finite_number(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _age(received_at, now):
    if received_at is None:
        return None
    return max(0.0, float(now) - float(received_at))


class AlignmentHandoffShadowCore:
    """Fail-closed admission logic for a future alignment command owner."""

    def __init__(
            self, mission_timeout_s=0.35, alignment_timeout_s=0.35,
            arm_timeout_s=0.40, max_angular_speed_rad_s=0.35,
            centered_angular_epsilon_rad_s=0.03,
            require_live_cancel_proof=False, gateway_timeout_s=0.35,
            cancel_guard_timeout_s=0.35):
        values = {
            'mission_timeout_s': mission_timeout_s,
            'alignment_timeout_s': alignment_timeout_s,
            'arm_timeout_s': arm_timeout_s,
            'max_angular_speed_rad_s': max_angular_speed_rad_s,
            'centered_angular_epsilon_rad_s': (
                centered_angular_epsilon_rad_s),
            'gateway_timeout_s': gateway_timeout_s,
            'cancel_guard_timeout_s': cancel_guard_timeout_s,
        }
        invalid = [name for name, value in values.items()
                   if not _finite_number(value) or value <= 0.0]
        if invalid:
            raise ValueError(
                'handoff shadow parameters must be positive and finite: '
                + ', '.join(invalid))
        if type(require_live_cancel_proof) is not bool:
            raise ValueError('require_live_cancel_proof must be a bool')
        self.mission_timeout = float(mission_timeout_s)
        self.alignment_timeout = float(alignment_timeout_s)
        self.arm_timeout = float(arm_timeout_s)
        self.max_angular = float(max_angular_speed_rad_s)
        self.centered_epsilon = float(
            centered_angular_epsilon_rad_s)
        self.require_live_cancel_proof = bool(require_live_cancel_proof)
        self.gateway_timeout = float(gateway_timeout_s)
        self.cancel_guard_timeout = float(cancel_guard_timeout_s)

        self._mission = None
        self._mission_received = None
        self._mission_error = ''
        self._alignment = None
        self._alignment_received = None
        self._alignment_error = ''
        self._arm_requested = False
        self._arm_received = None
        self._gateway = None
        self._gateway_received = None
        self._gateway_error = ''
        self._cancel_guard = None
        self._cancel_guard_received = None
        self._cancel_guard_error = ''

    def update_mission(self, payload, now):
        """Store one decoded supervisor state."""
        self._mission = payload
        self._mission_received = float(now)
        self._mission_error = ''

    def mark_mission_error(self, error, now):
        """Fail closed after malformed supervisor JSON."""
        self._mission = None
        self._mission_received = float(now)
        self._mission_error = str(error)

    def update_alignment(self, payload, now):
        """Store one decoded alignment preview."""
        self._alignment = payload
        self._alignment_received = float(now)
        self._alignment_error = ''

    def mark_alignment_error(self, error, now):
        """Fail closed after malformed alignment JSON."""
        self._alignment = None
        self._alignment_received = float(now)
        self._alignment_error = str(error)

    def update_arm(self, requested, now):
        """Store the latest operator heartbeat state."""
        self._arm_requested = bool(requested)
        self._arm_received = float(now)

    def update_gateway(self, payload, now):
        """Store the live gateway's release/handoff proof."""
        self._gateway = payload
        self._gateway_received = float(now)
        self._gateway_error = ''

    def mark_gateway_error(self, error, now):
        self._gateway = None
        self._gateway_received = float(now)
        self._gateway_error = str(error)

    def update_cancel_guard(self, payload, now):
        """Store the exact-goal cancel guard proof."""
        self._cancel_guard = payload
        self._cancel_guard_received = float(now)
        self._cancel_guard_error = ''

    def mark_cancel_guard_error(self, error, now):
        self._cancel_guard = None
        self._cancel_guard_received = float(now)
        self._cancel_guard_error = str(error)

    def _mission_blockers(self, now):
        blockers = []
        age = _age(self._mission_received, now)
        if self._mission_received is None:
            return ['MISSION_NOT_RECEIVED']
        if age > self.mission_timeout:
            blockers.append('MISSION_TIMEOUT')
        if self._mission_error:
            return blockers + ['MISSION_INVALID_JSON']
        if not isinstance(self._mission, dict):
            return blockers + ['MISSION_INVALID_PAYLOAD']

        mission = self._mission
        if mission.get('observe_only') is not True:
            blockers.append('MISSION_NOT_OBSERVE_ONLY')
        if str(mission.get('state', 'missing')) not in \
                MISSION_HANDOFF_STATES:
            blockers.append('MISSION_STATE_NOT_HANDOFF_READY')

        decision = mission.get('decision') or {}
        if decision.get(
                'eligible_for_future_alignment_handoff') is not True:
            blockers.append('MISSION_NOT_ELIGIBLE')
        if decision.get('command_published') is not False:
            blockers.append('MISSION_COMMAND_CONTRACT')
        readiness = decision.get('readiness_blockers')
        if not isinstance(readiness, list) or readiness:
            blockers.append('MISSION_READINESS_BLOCKED')

        health = mission.get('health') or {}
        if health.get('status') != 'OK' or health.get('blockers'):
            blockers.append('MISSION_HEALTH_BLOCKED')

        robot = mission.get('robot') or {}
        if robot.get('odom_fresh') is not True:
            blockers.append('ODOM_NOT_FRESH')
        if robot.get('stationary') is not True:
            blockers.append('ROBOT_NOT_STATIONARY')

        navigation = mission.get('navigation') or {}
        if navigation.get('goal_active') is not False:
            blockers.append('NAV2_GOAL_ACTIVE_OR_UNKNOWN')
        if navigation.get('observed_motion_owner') != 'NONE':
            blockers.append('OTHER_MOTION_OWNER_ACTIVE')

        safety = mission.get('safety') or {}
        if safety.get('motion_command_publisher') is not False or \
                safety.get('actuator_command_publisher') is not False or \
                safety.get('nav2_action_client') is not False:
            blockers.append('MISSION_SAFETY_CONTRACT')
        return blockers

    def _alignment_blockers(self, now):
        blockers = []
        age = _age(self._alignment_received, now)
        if self._alignment_received is None:
            return ['ALIGNMENT_NOT_RECEIVED'], 0.0, 'NO_TARGET'
        if age > self.alignment_timeout:
            blockers.append('ALIGNMENT_TIMEOUT')
        if self._alignment_error:
            return blockers + ['ALIGNMENT_INVALID_JSON'], 0.0, 'ERROR'
        if not isinstance(self._alignment, dict):
            return blockers + ['ALIGNMENT_INVALID_PAYLOAD'], 0.0, 'ERROR'

        alignment = self._alignment
        if alignment.get('observe_only') is not True:
            blockers.append('ALIGNMENT_NOT_OBSERVE_ONLY')
        safety = alignment.get('safety') or {}
        if safety.get('motion_command_publisher') is not False or \
                safety.get('actuator_command_publisher') is not False:
            blockers.append('ALIGNMENT_SAFETY_CONTRACT')

        state = str(alignment.get('state', 'NO_TARGET'))
        preview = alignment.get('preview') or {}
        angular = preview.get('target_angular_z_rad_s')
        if not _finite_number(angular):
            blockers.append('ALIGNMENT_ANGULAR_INVALID')
            angular = 0.0
        angular = float(angular)

        if state in TRACKING_STATES:
            if state == 'ALIGNING_LEFT' and angular <= 0.0:
                blockers.append('ALIGNMENT_DIRECTION_SIGN_MISMATCH')
            if state == 'ALIGNING_RIGHT' and angular >= 0.0:
                blockers.append('ALIGNMENT_DIRECTION_SIGN_MISMATCH')
        elif state in CENTERED_STATES:
            if abs(angular) > self.centered_epsilon:
                blockers.append('CENTERED_COMMAND_NONZERO')
            angular = 0.0
        else:
            blockers.append('ALIGNMENT_STATE_NOT_TRACKABLE')
            angular = 0.0
        return blockers, angular, state

    def _arm_blockers(self, now):
        age = _age(self._arm_received, now)
        if self._arm_received is None:
            return ['ARM_HEARTBEAT_NOT_RECEIVED'], age, False
        if age > self.arm_timeout:
            return ['ARM_HEARTBEAT_TIMEOUT'], age, False
        if not self._arm_requested:
            return ['OPERATOR_DISARMED'], age, True
        return [], age, True

    def _live_cancel_proof_blockers(self, now, gateway_publishers,
                                    cancel_guard_publishers):
        if not self.require_live_cancel_proof:
            return []
        blockers = []
        if gateway_publishers != 1:
            blockers.append('GATEWAY_PUBLISHER_INVALID')
        if cancel_guard_publishers != 1:
            blockers.append('CANCEL_GUARD_PUBLISHER_INVALID')

        if self._gateway_received is None:
            blockers.append('GATEWAY_NOT_RECEIVED')
        elif _age(self._gateway_received, now) > self.gateway_timeout:
            blockers.append('GATEWAY_TIMEOUT')
        if self._gateway_error or (self._gateway_received is not None and
                                   not isinstance(self._gateway, dict)):
            blockers.append('GATEWAY_INVALID_PAYLOAD')
        gateway = self._gateway if isinstance(self._gateway, dict) else {}
        gateway_safety = gateway.get('safety')
        if not isinstance(gateway_safety, dict):
            gateway_safety = {}
        if not (
                gateway.get('schema_version') == 1 and
                gateway.get('state') == 'HANDOFF_READY' and
                gateway.get('handoff_ready') is True and
                gateway.get('enabled') is True and
                gateway.get('one_shot') is True and
                gateway.get('fire_latched') is True and
                gateway.get('ever_forwarded') is True and
                gateway.get('released') is True and
                gateway.get('stop_reason') == 'FIRE_ACQUIRED' and
                gateway.get('raw_publishers') == 0 and
                gateway.get('goal_active') is False and
                gateway.get('nav_shadow_fingerprint_ok') is True and
                gateway.get('safety_gate_fingerprint_ok') is True and
                gateway.get('cancel_guard_publishers') == 1 and
                gateway_safety.get('raw_command_publisher') is False and
                gateway_safety.get('actuator_command_publisher') is False and
                gateway_safety.get('alignment_command_publisher') is False and
                gateway_safety.get('fail_closed_zero') is True):
            blockers.append('GATEWAY_HANDOFF_PROOF_INVALID')

        if self._cancel_guard_received is None:
            blockers.append('CANCEL_GUARD_NOT_RECEIVED')
        elif _age(self._cancel_guard_received, now) > \
                self.cancel_guard_timeout:
            blockers.append('CANCEL_GUARD_TIMEOUT')
        if self._cancel_guard_error or (
                self._cancel_guard_received is not None and
                not isinstance(self._cancel_guard, dict)):
            blockers.append('CANCEL_GUARD_INVALID_PAYLOAD')
        guard = (self._cancel_guard if
                 isinstance(self._cancel_guard, dict) else {})
        goal_uuid = guard.get('goal_uuid')
        guard_safety = guard.get('safety')
        if not isinstance(guard_safety, dict):
            guard_safety = {}
        if not (
                guard.get('schema_version') == 1 and
                guard.get('state') == 'CANCEL_CONFIRMED_STOPPED' and
                guard.get('enabled') is True and
                isinstance(goal_uuid, str) and
                GOAL_UUID_PATTERN.fullmatch(goal_uuid) is not None and
                int(goal_uuid, 16) != 0 and
                guard.get('send_count') == 1 and
                guard.get('cancel_service_accepted') is True and
                guard.get('cancel_status_seen') is True and
                guard.get('goal_status') == 5 and
                guard.get('arm_fresh') is True and
                guard.get('cmd_vel_fresh_zero') is True and
                guard.get('odom_fresh_stationary') is True and
                guard.get('nav_shadow_fingerprint_ok') is True and
                guard.get('raw_cmd_publishers') == 0 and
                guard.get('blockers') == [] and
                guard_safety.get('motion_command_publisher') is False and
                guard_safety.get('actuator_command_publisher') is False):
            blockers.append('CANCEL_GUARD_PROOF_INVALID')
        return blockers

    def evaluate(self, now, gateway_publishers=0,
                 cancel_guard_publishers=0):
        """Return the current shadow command and a JSON-safe status."""
        now = float(now)
        blockers = self._mission_blockers(now)
        alignment_blockers, requested_angular, alignment_state = (
            self._alignment_blockers(now))
        arm_blockers, arm_age, arm_fresh = self._arm_blockers(now)
        blockers.extend(alignment_blockers)
        blockers.extend(arm_blockers)
        blockers.extend(self._live_cancel_proof_blockers(
            now, gateway_publishers, cancel_guard_publishers))
        blockers = sorted(set(blockers))

        active = not blockers
        angular = 0.0
        clamped = False
        if active and alignment_state in TRACKING_STATES:
            angular = max(
                -self.max_angular,
                min(self.max_angular, requested_angular),
            )
            clamped = angular != requested_angular
        state = 'DISARMED'
        if active:
            state = (
                'ARMED_SHADOW_CENTERED'
                if alignment_state in CENTERED_STATES
                else 'ARMED_SHADOW_TRACKING'
            )
        elif self._arm_requested and not arm_blockers:
            state = 'BLOCKED'

        status = {
            'schema_version': 1,
            'shadow_only': True,
            'state': state,
            'active': active,
            'operator_arm': {
                'requested': self._arm_requested,
                'fresh': arm_fresh,
                'receive_age_ms': (
                    None if arm_age is None else round(arm_age * 1000.0, 1)
                ),
            },
            'admission': {
                'eligible': active,
                'blockers': blockers,
            },
            'live_cancel_proof': {
                'required': self.require_live_cancel_proof,
                'gateway_publishers': gateway_publishers,
                'cancel_guard_publishers': cancel_guard_publishers,
                'gateway_state': (self._gateway or {}).get('state') if
                isinstance(self._gateway, dict) else None,
                'cancel_guard_state': (self._cancel_guard or {}).get('state')
                if isinstance(self._cancel_guard, dict) else None,
                'goal_uuid': (self._cancel_guard or {}).get('goal_uuid') if
                isinstance(self._cancel_guard, dict) else None,
            },
            'alignment': {
                'state': alignment_state,
                'requested_angular_z_rad_s': round(requested_angular, 4),
                'shadow_angular_z_rad_s': round(angular, 4),
                'clamped': clamped,
            },
            'shadow_command': {
                'linear_x_m_s': 0.0,
                'angular_z_rad_s': round(angular, 4),
                'nonzero': abs(angular) > 0.0,
            },
            'safety': {
                'real_motion_output': False,
                'actuator_command_publisher': False,
                'nav2_action_client': False,
                'requires_arm_heartbeat': True,
                'fail_closed_zero': not active,
            },
        }
        return angular, status


class AlignmentHandoffShadow(Node):
    """Publish an isolated shadow Twist after all admission gates pass."""

    def __init__(self):
        super().__init__('alignment_handoff_shadow')
        self._declare_parameters()
        self._read_parameters()
        self._core = AlignmentHandoffShadowCore(**self._core_parameters)

        command_topic = self.resolve_topic_name(self.shadow_command_topic)
        if command_topic in FORBIDDEN_COMMAND_TOPICS or \
                not command_topic.startswith('/fire_mission/') or \
                not command_topic.endswith('_shadow'):
            raise ValueError(
                'shadow_command_topic must stay isolated under '
                '/fire_mission/ and end with _shadow')

        qos = sensor_qos()
        self._shadow_publisher = self.create_publisher(
            Twist, self.shadow_command_topic, qos)
        self._state_publisher = self.create_publisher(
            String, self.handoff_state_topic, qos)
        self.create_subscription(
            String, self.mission_state_topic, self._on_mission, qos)
        self.create_subscription(
            String, self.alignment_topic, self._on_alignment, qos)
        self.create_subscription(
            Bool, self.arm_topic, self._on_arm, arm_qos())
        if self._core.require_live_cancel_proof:
            for topic in (self.gateway_state_topic,
                          self.cancel_guard_state_topic):
                if self.resolve_topic_name(topic) != topic:
                    raise ValueError('live proof topics cannot be remapped')
            self.create_subscription(
                String, self.gateway_state_topic, self._on_gateway, qos)
            self.create_subscription(
                String, self.cancel_guard_state_topic,
                self._on_cancel_guard, qos)
        self.create_timer(1.0 / self.publish_rate, self._publish)
        self.get_logger().warning(
            '[HANDOFF] SHADOW-ONLY: output is isolated from every robot '
            'command topic; motor motion is impossible from this node'
        )

    def _declare_parameters(self):
        self.declare_parameter('mission_state_topic', '/fire_mission/state')
        self.declare_parameter(
            'alignment_topic', '/yolo/alignment_preview')
        self.declare_parameter(
            'arm_topic', '/fire_mission/alignment_arm_shadow')
        self.declare_parameter(
            'shadow_command_topic',
            '/fire_mission/alignment_cmd_vel_shadow')
        self.declare_parameter(
            'handoff_state_topic', '/fire_mission/alignment_handoff_state')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('mission_timeout_s', 0.35)
        self.declare_parameter('alignment_timeout_s', 0.35)
        self.declare_parameter('arm_timeout_s', 0.40)
        self.declare_parameter('max_angular_speed_rad_s', 0.35)
        self.declare_parameter(
            'centered_angular_epsilon_rad_s', 0.03)
        self.declare_parameter('require_live_cancel_proof', False)
        self.declare_parameter(
            'gateway_state_topic', '/fire_mission/live_nav_stop_state')
        self.declare_parameter(
            'cancel_guard_state_topic',
            '/fire_mission/nav2_cancel_guard_state')
        self.declare_parameter('gateway_timeout_s', 0.35)
        self.declare_parameter('cancel_guard_timeout_s', 0.35)

    def _read_parameters(self):
        topics = (
            'mission_state_topic',
            'alignment_topic',
            'arm_topic',
            'shadow_command_topic',
            'handoff_state_topic',
            'gateway_state_topic',
            'cancel_guard_state_topic',
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
                'alignment_timeout_s',
                'arm_timeout_s',
                'max_angular_speed_rad_s',
                'centered_angular_epsilon_rad_s',
                'gateway_timeout_s',
                'cancel_guard_timeout_s',
            )
        }
        self._core_parameters['require_live_cancel_proof'] = (
            self.get_parameter('require_live_cancel_proof').value)
        if self._core_parameters['require_live_cancel_proof']:
            if self.gateway_state_topic != \
                    '/fire_mission/live_nav_stop_state' or \
                    self.cancel_guard_state_topic != \
                    '/fire_mission/nav2_cancel_guard_state':
                raise ValueError('live proof topics must be canonical')

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

    def _on_alignment(self, message):
        self._decode_json(
            message,
            self._core.mark_alignment_error,
            self._core.update_alignment,
        )

    def _on_arm(self, message):
        self._core.update_arm(message.data, self._now())

    def _on_gateway(self, message):
        self._decode_json(
            message, self._core.mark_gateway_error,
            self._core.update_gateway)

    def _on_cancel_guard(self, message):
        self._decode_json(
            message, self._core.mark_cancel_guard_error,
            self._core.update_cancel_guard)

    def _publish(self):
        gateway_publishers = cancel_guard_publishers = 0
        if self._core.require_live_cancel_proof:
            gateway_publishers = self.count_publishers(
                self.gateway_state_topic)
            cancel_guard_publishers = self.count_publishers(
                self.cancel_guard_state_topic)
        angular, status = self._core.evaluate(
            self._now(), gateway_publishers, cancel_guard_publishers)
        command = Twist()
        command.angular.z = angular
        self._shadow_publisher.publish(command)
        message = String()
        message.data = json.dumps(
            status, separators=(',', ':'), ensure_ascii=True)
        self._state_publisher.publish(message)


def main(args=None):
    """Run the shadow-only alignment handoff gate."""
    rclpy.init(args=args)
    node = None
    try:
        node = AlignmentHandoffShadow()
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
