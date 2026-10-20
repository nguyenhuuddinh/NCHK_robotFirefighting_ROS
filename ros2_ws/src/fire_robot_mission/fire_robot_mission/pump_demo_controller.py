"""Opt-in ROS wrapper for a short, one-shot S3 pump pulse."""

import json
import time

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from fire_robot_safety.odom_freshness import OdomStampGuard, admit_odom
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from fire_robot_mission.alignment_handoff_shadow import arm_qos
from fire_robot_mission.mission_supervisor import action_status_qos, sensor_qos
from fire_robot_mission.pump_demo_core import PumpDemoCore
from fire_robot_mission.pump_release import PumpReleaseProof
from fire_robot_mission.session_context import SessionContextGuard
from fire_robot_mission.close_owner_adapter import (
    append_owner_observation, configure_owner_observation, start_owner_observation,
)


EXPECTED_PUBLISHERS = {
    'ir': ('/fire_mission/ir_range_preview', 'ir_range_preview',
           'std_msgs/msg/String', ReliabilityPolicy.BEST_EFFORT),
    'mission': ('/fire_mission/state', 'fire_mission_supervisor',
                'std_msgs/msg/String', ReliabilityPolicy.BEST_EFFORT),
    'alignment': ('/fire_mission/live_alignment_state',
                  'live_alignment_takeover', 'std_msgs/msg/String',
                  ReliabilityPolicy.BEST_EFFORT),
    'gateway': ('/fire_mission/live_nav_stop_state',
                'live_nav_stop_gateway', 'std_msgs/msg/String',
                ReliabilityPolicy.BEST_EFFORT),
    'guard': ('/fire_mission/nav2_cancel_guard_state', 'nav2_cancel_guard',
              'std_msgs/msg/String', ReliabilityPolicy.BEST_EFFORT),
    'odom': ('/odom', 'serial_bridge_node', 'nav_msgs/msg/Odometry',
             ReliabilityPolicy.BEST_EFFORT),
    'cmd': ('/cmd_vel', 'safety_watchdog', 'geometry_msgs/msg/Twist',
            ReliabilityPolicy.BEST_EFFORT),
    'pump': ('/pump_cmd', 'pump_demo_controller', 'std_msgs/msg/Bool',
             ReliabilityPolicy.RELIABLE),
    'status': ('/navigate_to_pose/_action/status', 'bt_navigator',
               'action_msgs/msg/GoalStatusArray', ReliabilityPolicy.RELIABLE),
}


class PumpDemoController(Node):
    """Never own motion or servo; publish a bounded Bool command only."""

    def __init__(self):
        super().__init__('pump_demo_controller')
        self._odom_stamp_guard = OdomStampGuard()
        self.declare_parameter('enable_pump_demo', False)
        self.declare_parameter('allow_provisional_ir', False)
        self.declare_parameter('pulse_s', 5.0)
        self.declare_parameter('hold_s', 0.4)
        self.declare_parameter('release_pump_on_terminal', False)
        self.declare_parameter('require_fire_session_context', False)
        release_on_terminal = self.get_parameter('release_pump_on_terminal').value
        require_session = self.get_parameter('require_fire_session_context').value
        if type(release_on_terminal) is not bool:
            raise ValueError('release_pump_on_terminal must be bool')
        if type(require_session) is not bool or (require_session and not release_on_terminal):
            raise ValueError('require_fire_session_context needs terminal release enabled')
        topics = ('/pump_cmd', '/fire_mission/pump_demo_arm',
                      '/fire_mission/ir_range_preview',
                      '/fire_mission/state',
                      '/fire_mission/live_alignment_state',
                      '/fire_mission/live_nav_stop_state',
                      '/fire_mission/nav2_cancel_guard_state',
                      '/navigate_to_pose/_action/status',
                      '/fire_mission/pump_demo_state', '/odom', '/cmd_vel',
                      '/cmd_vel_raw')
        if release_on_terminal:
            topics += ('/fire_sensors/actuator_state',)
        for topic in topics:
            if self.resolve_topic_name(topic) != topic:
                raise ValueError(f'pump demo forbids remapping {topic}')
        self.core = PumpDemoCore(
            enabled=self.get_parameter('enable_pump_demo').value,
            allow_provisional_ir=self.get_parameter(
                'allow_provisional_ir').value,
            pulse_s=self.get_parameter('pulse_s').value,
            hold_s=self.get_parameter('hold_s').value)
        qos = sensor_qos()
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE)
        self.pump_pub = None
        self._pump_qos = reliable
        self._pump_release = (PumpReleaseProof(timeout_s=self.core.input_timeout_s)
                              if release_on_terminal and self.core.enabled else None)
        self._session_context = (SessionContextGuard(timeout_s=self.core.input_timeout_s)
                                 if require_session and self.core.enabled else None)
        configure_owner_observation(
            self, 'pump_demo_controller', live_enabled=self.core.enabled,
            context=self._session_context, timeout_s=self.core.input_timeout_s)
        if self.core.enabled and self._pump_release is None:
            self._create_pump_publisher()
        self.state_pub = self.create_publisher(
            String, '/fire_mission/pump_demo_state', qos)
        for name, topic in (
                ('ir', '/fire_mission/ir_range_preview'),
                ('mission', '/fire_mission/state'),
                ('alignment', '/fire_mission/live_alignment_state'),
                ('gateway', '/fire_mission/live_nav_stop_state'),
                ('guard', '/fire_mission/nav2_cancel_guard_state')):
            self.create_subscription(
                String, topic,
                lambda msg, key=name: self._on_json(key, msg), qos)
        self.create_subscription(
            Bool, '/fire_mission/pump_demo_arm', self._on_arm, arm_qos())
        self.create_subscription(Odometry, '/odom', self._on_odom, qos)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, qos)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._on_status, action_status_qos())
        if self._pump_release is not None:
            self.create_subscription(
                String, '/fire_sensors/actuator_state', self._on_actuator, qos)
        self.create_timer(0.1, self._tick)
        self.get_logger().warning(
            '[PUMP DEMO] enabled=%s provisional_override=%s; one-shot '
            'pulse_s=%.1f s (5 s maximum), servo disabled' %
            (self.core.enabled, self.core.allow_provisional_ir,
             self.core.pulse_s))

    def _create_pump_publisher(self, *, session_binding=None):
        self.pump_pub = self.create_publisher(Bool, '/pump_cmd', self._pump_qos)
        if self._pump_release is not None:
            self._pump_release.owner_created(session_binding=session_binding)

    def _actuator_source_ok(self):
        infos = self.get_publishers_info_by_topic('/fire_sensors/actuator_state')
        return (len(infos) == 1 and infos[0].node_namespace == '/' and
                infos[0].node_name == 'serial_bridge_node' and
                infos[0].topic_type == 'std_msgs/msg/String' and
                infos[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT)

    def _on_actuator(self, msg):
        if self._pump_release is None:
            return
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            payload = None
        source_ok = self._actuator_source_ok()
        self._pump_release.update(
            payload, self.get_clock().now().nanoseconds, time.monotonic(),
            source_ok=source_ok)

    def _on_json(self, name, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            payload = None
        context = getattr(self, '_session_context', None)
        observed_context = context or getattr(self, '_close_owner_context', None)
        if name == 'mission' and observed_context is not None:
            source_ok = self._exact_publisher_count('mission') == 1
            observed_context.update(payload, self.get_clock().now().nanoseconds,
                                    time.monotonic(), source_ok=source_ok)
            if context is not None and context.sample is None:
                payload = None
        self.core.update(name, payload, time.monotonic())
        if name == 'mission' and context is not None and self.pump_pub is not None:
            # A replacement good report before the next timer must not hide
            # a context break that happened while this owner was acquired.
            self._guard_owned_session(time.monotonic(), source_ok=source_ok)

    def _guard_owned_session(self, now, *, source_ok):
        binding = self._pump_release.session_binding
        if binding is None:
            raise RuntimeError('PUMP_SESSION_BINDING_MISSING')
        blocker = ('SESSION_PUBLISHER_INVALID' if source_ok is not True else
                   self._session_context.blocker_for(binding, now))
        alignment = self.core._input('alignment', now) or {}
        if blocker is None and alignment.get('goal_uuid') != binding.nav_goal_uuid:
            blocker = 'SESSION_BINDING_CHANGED'
        if blocker == 'SESSION_CLOSE_REQUESTED':
            observer = self._session_context.current(now)
            if not self._pump_release.accept_close_request(
                    session_id=observer['session_id'], nav_goal_uuid=observer['nav_goal_uuid'],
                    generation=observer['close_generation'], source=observer['source']):
                blocker = 'SESSION_CLOSE_CONTEXT_CHANGED'
        if blocker is not None:
            self.core.abort_for_session(
                'SESSION_CLOSE_REQUESTED' if blocker == 'SESSION_CLOSE_REQUESTED'
                else 'SESSION_CONTEXT_INVALID', binding.nav_goal_uuid)
        return blocker

    def _on_arm(self, msg):
        self.core.update_arm(msg.data, time.monotonic())

    @staticmethod
    def _twist_values(twist):
        return (twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z)

    def _on_odom(self, msg):
        received = admit_odom(self, msg, self.core.input_timeout_s)
        if received is None:
            return
        self.core.update_odom(
            self._twist_values(msg.twist.twist), received)

    def _on_cmd(self, msg):
        self.core.update_cmd(self._twist_values(msg), time.monotonic())

    def _on_status(self, msg):
        self.core.update_statuses([
            (bytes(item.goal_info.goal_id.uuid).hex(), item.status,
             item.goal_info.stamp.sec, item.goal_info.stamp.nanosec)
            for item in msg.status_list], time.monotonic())

    def _exact_publisher_count(self, key):
        topic, name, msg_type, reliability = EXPECTED_PUBLISHERS[key]
        infos = self.get_publishers_info_by_topic(topic)
        if (len(infos) != 1 or infos[0].node_namespace != '/' or
                infos[0].node_name != name or
                infos[0].topic_type != msg_type or
                infos[0].qos_profile.reliability != reliability):
            return 0
        return 1

    def _exact_bridge_subscriber_count(self):
        infos = self.get_subscriptions_info_by_topic('/pump_cmd')
        if (len(infos) != 1 or infos[0].node_namespace != '/' or
                infos[0].node_name != 'serial_bridge_node' or
                infos[0].topic_type != 'std_msgs/msg/Bool' or
                infos[0].qos_profile.reliability !=
                ReliabilityPolicy.RELIABLE):
            return 0
        return 1

    def _tick(self):
        report_stamp_ns = start_owner_observation(self)
        counts = {key: self._exact_publisher_count(key)
                  for key in EXPECTED_PUBLISHERS}
        counts.update({
            'arm': self.count_publishers('/fire_mission/pump_demo_arm'),
            'pump_subscribers': self._exact_bridge_subscriber_count(),
            'raw': self.count_publishers('/cmd_vel_raw'),
        })
        now = time.monotonic()
        release = self._pump_release
        context = getattr(self, '_session_context', None)
        session_blocker = None
        if release is not None:
            total_pump = self.count_publishers('/pump_cmd')
            now = time.monotonic()
            if (self.pump_pub is None and self.core.state == 'WAITING_FOR_ADMISSION'
                    and total_pump == 0 and counts['pump'] == 0 and
                    release.fresh_off(now) and
                    set(self.core.blockers(now, counts)) == {'PUMP_OWNER_CONFLICT'}):
                # This only creates an OFF publisher. Full step() still uses
                # the actual graph counts and must pass its normal ON hold.
                if context is None:
                    self._create_pump_publisher()
                else:
                    alignment = self.core._input('alignment', now) or {}
                    binding = context.bind(alignment.get('goal_uuid'), now)
                    if binding is not None:
                        self._create_pump_publisher(session_binding=binding)
            if context is not None and self.pump_pub is not None:
                session_blocker = self._guard_owned_session(
                    time.monotonic(), source_ok=counts['mission'] == 1)
            on, report = self.core.step(
                time.monotonic(), counts, owner_present=self.pump_pub is not None,
                allow_rearm=self.pump_pub is None and release.receipt is not None)
        else:
            on, report = self.core.step(now, counts)
        if self.pump_pub is not None:
            command = Bool()
            command.data = on
            self.pump_pub.publish(command)
            if release is not None and not on and self.core.state in ('COMPLETE', 'ABORTED'):
                requested = release.request_after_off_publish(
                    self.core.goal_uuid, self.get_clock().now().nanoseconds,
                    time.monotonic())
                if requested and release.ready(time.monotonic()):
                    if not (counts['pump'] == 1 and total_pump == 1 and
                            counts['pump_subscribers'] == 1):
                        release.blocker = 'PUMP_RELEASE_ENDPOINTS_NOT_EXCLUSIVE'
                    elif not self._actuator_source_ok():
                        release.blocker = 'ACTUATOR_PUBLISHER_INVALID'
                    else:
                        checked_at = time.monotonic()
                        if release.ready(checked_at):
                            if self.destroy_publisher(self.pump_pub) is not True:
                                raise RuntimeError('PUMP_PUBLISHER_RELEASE_UNCONFIRMED')
                            self.pump_pub = None
                            release.destroyed(
                                ros_now_ns=self.get_clock().now().nanoseconds,
                                checked_at=checked_at)
                            # A released treatment must not inherit a continuously
                            # high heartbeat as fresh operator authorization.
                            self.core.arm = False
                            self.core.arm_at = None
                            self.core.arm_false_seen = False
                            self.core.arm_cycle_seen = False
        if release is not None:
            report['pump_release'] = release.snapshot(owned=self.pump_pub is not None)
        if context is not None:
            report['session_context'] = {
                'required': True,
                'blocker': session_blocker if self.pump_pub is not None else context.reason,
            }
        append_owner_observation(self, report, report_stamp_ns,
                                 publisher_owned=self.pump_pub is not None)
        msg = String()
        msg.data = json.dumps(report, separators=(',', ':'), allow_nan=False)
        self.state_pub.publish(msg)

    def destroy_node(self):
        if self.pump_pub is not None:
            off = Bool()
            off.data = False
            for _ in range(3):
                self.pump_pub.publish(off)
            self.pump_pub = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PumpDemoController()
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
