"""Preview Nav2 goal cancellation without creating an action client."""

import json
import time

from action_msgs.msg import GoalStatusArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fire_robot_mission.mission_supervisor import (
    action_status_qos,
    sensor_qos,
)


ACTIVE_STATUSES = {1, 2, 3}
FIRE_STATE_INTENTS = {
    'FIRE_TRACKING': {'LEFT_PREVIEW', 'RIGHT_PREVIEW'},
    'CENTERED_HOLD': {'HOLD_CENTER_PREVIEW'},
    'ALIGNMENT_READY': {'HOLD_CENTER_PREVIEW'},
    'WAITING_FOR_ALIGNMENT': {'REQUEST_SAFE_STOP'},
    'TARGET_LOST': {'REQUEST_SAFE_STOP'},
}
COMMAND_TOPICS = {
    '/cmd_vel', '/cmd_vel_raw', '/cmd_vel_nav',
    '/pump_cmd', '/fire_target',
}


def _age(received_at, now):
    return None if received_at is None else max(0.0, now - received_at)


def _object(value):
    return value if isinstance(value, dict) else {}


class Nav2CancelIntentShadowCore:
    """Cross-check mission and Nav2 status before suggesting a cancellation."""

    def __init__(self, mission_timeout_s=0.35):
        if mission_timeout_s <= 0.0:
            raise ValueError('mission timeout must be positive')
        self.mission_timeout = float(mission_timeout_s)
        self.mission = None
        self.mission_received = None
        self.mission_error = False
        self.statuses = None
        self.status_received = None
        self.status_publisher_count = 0

    def update_mission(self, payload, now):
        self.mission = payload
        self.mission_received = float(now)
        self.mission_error = False

    def mark_mission_error(self, now):
        self.mission = None
        self.mission_received = float(now)
        self.mission_error = True

    def update_statuses(self, statuses, now):
        """Accept (UUID hex, status integer) pairs from one status message."""
        self.statuses = [(str(uid), int(status)) for uid, status in statuses]
        self.status_received = float(now)

    def update_status_publisher_count(self, count):
        """Require a currently discovered Nav2 status publisher."""
        self.status_publisher_count = int(count)
        if self.status_publisher_count != 1:
            # Do not reuse a cached goal after the action server disappears.
            self.statuses = None
            self.status_received = None

    def evaluate(self, now):
        now = float(now)
        blockers = []
        mission = self.mission if isinstance(self.mission, dict) else {}
        state = str(mission.get('state', 'UNKNOWN'))
        decision = _object(mission.get('decision'))
        navigation = _object(mission.get('navigation'))
        intent = str(decision.get('intent', 'UNKNOWN'))
        goal_active = navigation.get('goal_active')
        active = [] if self.statuses is None else [
            {'uuid': uid, 'status': status}
            for uid, status in self.statuses if status in ACTIVE_STATUSES
        ]
        mission_age = _age(self.mission_received, now)
        status_age = _age(self.status_received, now)

        if mission_age is None:
            blockers.append('MISSION_NOT_RECEIVED')
        elif mission_age > self.mission_timeout:
            blockers.append('MISSION_TIMEOUT')
        if self.mission_error or (self.mission_received is not None and
                                  not isinstance(self.mission, dict)):
            blockers.append('MISSION_INVALID_JSON_OR_PAYLOAD')
        if status_age is None:
            blockers.append('NAV_STATUS_NOT_RECEIVED')
        if self.status_publisher_count != 1:
            blockers.append('NAV_STATUS_PUBLISHER_UNAVAILABLE_OR_AMBIGUOUS')
        if self.statuses is not None and any(
                status not in range(7) or len(uid) != 32 or
                any(c not in '0123456789abcdef' for c in uid.lower())
                for uid, status in self.statuses):
            blockers.append('NAV_STATUS_INVALID')

        if (self.mission_received is not None and
                isinstance(self.mission, dict)):
            safety = _object(mission.get('safety'))
            if mission.get('observe_only') is not True or \
                    decision.get('command_published') is not False or \
                    safety.get('motion_command_publisher') is not False or \
                    safety.get('actuator_command_publisher') is not False or \
                    safety.get('nav2_action_client') is not False:
                blockers.append('MISSION_SAFETY_CONTRACT')
            health = _object(mission.get('health'))
            if state != 'FAULT' and (health.get('status') != 'OK' or
                                     health.get('blockers')):
                blockers.append('MISSION_HEALTH_NOT_OK')
            if not isinstance(goal_active, bool):
                blockers.append('MISSION_GOAL_STATE_UNKNOWN')
            elif goal_active != bool(active) and status_age is not None:
                blockers.append('GOAL_STATE_MISMATCH')

        if len(active) > 1:
            blockers.append('MULTIPLE_ACTIVE_GOALS')

        fire_acquired = (
            intent in FIRE_STATE_INTENTS.get(state, set())
        )
        fault_stop = state == 'FAULT' and intent == 'REQUEST_SAFE_STOP'
        request_reason = 'FIRE_ACQUIRED' if fire_acquired else \
            'MISSION_FAULT' if fault_stop else 'NONE'

        status = 'BLOCKED' if blockers else 'NO_CANCEL_NEEDED'
        if not blockers and active:
            if active[0]['status'] == 3:
                status = 'CANCEL_ALREADY_IN_PROGRESS'
            elif request_reason != 'NONE' and goal_active is True:
                status = 'REQUEST_CANCEL'
        return {
            'schema_version': 1,
            'shadow_only': True,
            'state': status,
            'request_cancel': status == 'REQUEST_CANCEL',
            'reason': request_reason,
            'mission': {'state': state, 'intent': intent,
                        'receive_age_ms': None if mission_age is None else
                        round(mission_age * 1000.0, 1)},
            'navigation': {
                'mission_goal_active': goal_active,
                'active_goals': active,
                'status_receive_age_ms': None if status_age is None else
                round(status_age * 1000.0, 1),
                'status_publisher_count': self.status_publisher_count,
            },
            'blockers': sorted(set(blockers)),
            'safety': {
                'nav2_action_client': False,
                'cancel_request_sent': False,
                'motion_command_publisher': False,
                'actuator_command_publisher': False,
            },
        }


class Nav2CancelIntentShadow(Node):
    """Publish a JSON cancellation suggestion, never a Nav2 action request."""

    def __init__(self):
        super().__init__('nav2_cancel_intent_shadow')
        self.declare_parameter('mission_state_topic', '/fire_mission/state')
        self.declare_parameter(
            'nav_status_topic', '/navigate_to_pose/_action/status')
        self.declare_parameter(
            'intent_topic', '/fire_mission/nav2_cancel_intent_shadow')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('mission_timeout_s', 0.35)
        mission_topic = str(self.get_parameter('mission_state_topic').value)
        status_topic = str(self.get_parameter('nav_status_topic').value)
        intent_topic = str(self.get_parameter('intent_topic').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not mission_topic or not status_topic or rate <= 0.0:
            raise ValueError(
                'cancel shadow input topics and rate are required')
        if not intent_topic.startswith('/fire_mission/') or \
                not intent_topic.endswith('_shadow') or \
                intent_topic in COMMAND_TOPICS or \
                intent_topic in {mission_topic, status_topic}:
            raise ValueError('cancel intent output must be isolated')
        self.core = Nav2CancelIntentShadowCore(
            self.get_parameter('mission_timeout_s').value)
        self.status_topic = status_topic
        self.publisher = self.create_publisher(
            String, intent_topic, sensor_qos())
        self.create_subscription(
            String, mission_topic, self._on_mission, sensor_qos())
        self.create_subscription(
            GoalStatusArray, status_topic, self._on_status,
            action_status_qos())
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().warning(
            '[MISSION] CANCEL SHADOW ONLY: no Nav2 action client or '
            'robot command publisher')

    def _on_mission(self, message):
        now = time.monotonic()
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('mission JSON root must be an object')
        except (ValueError, TypeError):
            self.core.mark_mission_error(now)
            return
        self.core.update_mission(payload, now)

    def _on_status(self, message):
        statuses = [
            (bytes(item.goal_info.goal_id.uuid).hex(), item.status)
            for item in message.status_list
        ]
        self.core.update_statuses(statuses, time.monotonic())

    def _publish(self):
        self.core.update_status_publisher_count(
            self.count_publishers(self.status_topic))
        payload = self.core.evaluate(time.monotonic())
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        self.publisher.publish(message)


def main(args=None):
    """Run cancellation-intent telemetry on the laptop."""
    rclpy.init(args=args)
    node = None
    try:
        node = Nav2CancelIntentShadow()
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
