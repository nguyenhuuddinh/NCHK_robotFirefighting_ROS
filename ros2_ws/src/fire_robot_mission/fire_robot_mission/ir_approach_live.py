"""Opt-in one-shot IR-only jog through the existing Pi SafetyGate."""

from collections import Counter
import json
import math
import time

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from fire_robot_mission.alignment_handoff_shadow import arm_qos
from fire_robot_mission.ir_approach_live_core import (
    IrApproachLiveCore, VerifiedEndpointPins, verified_raw_gate,
)
from fire_robot_mission.mission_supervisor import (
    action_status_qos, sensor_qos,
)
from fire_robot_mission.nav2_cancel_guard import (
    ACTIVE_STATUSES, EXPECTED_SHADOW_PUBLISHERS, _valid_uuid,
)


TOPICS = (
    '/fire_mission/ir_range_preview', '/scan', '/odom', '/cmd_vel',
    '/cmd_vel_raw', '/fire_mission/ir_approach_arm',
    '/fire_mission/ir_approach_live_state',
    '/navigate_to_pose/_action/status', '/cmd_vel_nav', '/pump_cmd',
    '/fire_mission/nav2_cmd_vel_shadow', '/fire_mission/state',
    '/fire_mission/live_alignment_state',
    '/fire_mission/live_nav_stop_state',
    '/fire_mission/nav2_cancel_guard_state',
    '/fire_mission/post_spray_review', '/fire_sensors/actuator_state',
)


class IrApproachLiveNode(Node):
    """Create the sole raw Twist publisher for one short operator-armed step."""

    def __init__(self):
        super().__init__('ir_approach_live')
        for topic in TOPICS:
            if self.resolve_topic_name(topic) != topic:
                raise ValueError('IR jog forbids remapping safety topics')
        self.declare_parameter('enable_live_ir_jog', False)
        self.declare_parameter('allow_provisional_ir', False)
        self.declare_parameter('enable_reverse_jog', False)
        self.declare_parameter('integrated_handoff', False)
        self.declare_parameter('continuous_approach', False)
        defaults = {
            'step_m': 0.05, 'speed_m_s': 0.05,
            'max_duration_s': 1.0, 'input_timeout_s': 0.35,
            'continuous_distance_m': 0.30,
            'max_total_approach_m': 2.0,
            'arm_timeout_s': 0.40, 'discovery_timeout_s': 0.75,
            'admission_hold_s': 0.3, 'zero_hold_s': 0.5,
            'clearance_margin_m': 0.45,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.core = IrApproachLiveCore(
            enabled=self.get_parameter('enable_live_ir_jog').value,
            allow_provisional_ir=self.get_parameter(
                'allow_provisional_ir').value,
            enable_reverse_jog=self.get_parameter(
                'enable_reverse_jog').value,
            integrated_handoff=self.get_parameter(
                'integrated_handoff').value,
            continuous_approach=self.get_parameter(
                'continuous_approach').value,
            **{key: self.get_parameter(key).value for key in defaults},
        )
        self._endpoint_pins = VerifiedEndpointPins()
        self._raw_publisher = None
        self._proofs = {}
        self._statuses = None
        qos = sensor_qos()
        self.state_publisher = self.create_publisher(
            String, '/fire_mission/ir_approach_live_state', qos)
        self.create_subscription(
            String, '/fire_mission/ir_range_preview', self._on_ir, qos)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, qos)
        self.create_subscription(
            Bool, '/fire_mission/ir_approach_arm', self._on_arm, arm_qos())
        if self.core.integrated_handoff:
            for key, topic in (
                    ('mission', '/fire_mission/state'),
                    ('alignment', '/fire_mission/live_alignment_state'),
                    ('gateway', '/fire_mission/live_nav_stop_state'),
                    ('guard', '/fire_mission/nav2_cancel_guard_state'),
                    ('review', '/fire_mission/post_spray_review'),
                    ('actuator', '/fire_sensors/actuator_state')):
                self.create_subscription(
                    String, topic,
                    lambda msg, name=key: self._on_proof(name, msg), qos)
            self.create_subscription(
                GoalStatusArray, '/navigate_to_pose/_action/status',
                self._on_status, action_status_qos())
        self.create_timer(0.05, self._tick)
        self.get_logger().warning(
            'IR jog opt-in=%s, integrated=%s, continuous=%s, '
            'provisional override=%s; no pump owner, never run on a table' % (
                self.core.enabled, self.core.integrated_handoff,
                self.core.continuous_approach,
                self.core.allow_provisional_ir))

    def _on_ir(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            payload = None
        self.core.update('ir', payload, time.monotonic())

    def _on_scan(self, msg):
        self.core.update('scan', (
            msg.angle_min, msg.angle_increment, msg.range_min,
            msg.range_max, tuple(msg.ranges)), time.monotonic())

    def _on_odom(self, msg):
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        pose = msg.pose.pose.position
        self.core.update('odom', (
            (pose.x, pose.y),
            (linear.x, linear.y, linear.z,
             angular.x, angular.y, angular.z),
            msg.header.frame_id), time.monotonic())

    def _on_cmd(self, msg):
        self.core.update('cmd', (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z), time.monotonic())

    def _on_arm(self, msg):
        self.core.update_arm(msg.data, time.monotonic())

    def _on_proof(self, name, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            payload = None
        self._proofs[name] = (payload, time.monotonic())
        if name in ('review', 'actuator'):
            self.core.update(name, payload, time.monotonic())

    def _on_status(self, msg):
        self._statuses = tuple(
            (bytes(item.goal_info.goal_id.uuid).hex(), item.status)
            for item in msg.status_list)

    def _proof(self, name, now):
        payload, received = self._proofs.get(name, (None, None))
        return payload if received is not None and isinstance(payload, dict) \
            and 0 <= now - received <= self.core.input_timeout else None

    def _handoff_evidence(self, now, counts):
        """Exact canceled-goal proof is required before claiming raw motion."""
        mission = self._proof('mission', now) or {}
        alignment = self._proof('alignment', now) or {}
        gateway = self._proof('gateway', now) or {}
        guard = self._proof('guard', now) or {}
        ir = self.core._input('ir', now) or {}
        uuid = guard.get('goal_uuid')
        # Action status is event-driven, not a heartbeat. Its exact endpoint
        # stays pinned while fresh guard and mission telemetry carry liveness.
        status_ok = (self._statuses is not None and
                     not any(status in ACTIVE_STATUSES
                             for _, status in self._statuses) and
                     not any(uid == uuid and status != 5
                             for uid, status in self._statuses) and
                     guard.get('cancel_status_seen') is True and
                     guard.get('goal_status') == 5)
        exact_goal = (
            all(counts.get(key + '_source_ok') for key in (
                'mission', 'alignment', 'gateway', 'guard', 'status')) and
            _valid_uuid(uuid) and status_ok and
            mission.get('schema_version') == 1 and
            mission.get('observe_only') is True and
            (mission.get('navigation') or {}).get('goal_active') is False and
            (mission.get('navigation') or {}).get('goal_state') ==
            'CANCELED' and
            (mission.get('safety') or {}).get(
                'motion_command_publisher') is False and
            (mission.get('safety') or {}).get(
                'actuator_command_publisher') is False and
            alignment.get('schema_version') == 1 and
            alignment.get('state') == 'COMPLETE' and
            alignment.get('goal_uuid') == uuid and
            alignment.get('stop_reason') == 'TARGET_CENTERED' and
            alignment.get('publisher_owned') is False and
            (alignment.get('command') or {}).get(
                'angular_z_rad_s') == 0.0 and
            gateway.get('schema_version') == 1 and
            gateway.get('cycle_goal_uuid') == uuid and
            gateway.get('released') is True and
            gateway.get('fire_latched') is True and
            gateway.get('ever_forwarded') is True and
            gateway.get('stop_reason') == 'FIRE_ACQUIRED' and
            guard.get('schema_version') == 1 and
            guard.get('enabled') is True and
            guard.get('send_count') == 1 and
            guard.get('cancel_service_accepted') is True and
            guard.get('cancel_status_seen') is True and
            guard.get('goal_status') == 5 and
            guard.get('arm_fresh') is True and
            guard.get('nav_shadow_fingerprint_ok') is True and
            guard.get('blockers') == [] and
            counts.get('nav_shadow_ok') is True)
        # Preserve identity across a brief camera/IR dropout so the core can
        # stop and recover this *same* goal. Live motion still needs every
        # current perception/health check below; identity alone grants none.
        base = (
            exact_goal and
            mission.get('state') in (
                'FIRE_TRACKING', 'CENTERED_HOLD', 'ALIGNMENT_READY') and
            (mission.get('perception') or {}).get('confirmed') is True and
            (mission.get('perception') or {}).get('alignment_state') ==
            'CENTERED' and
            (mission.get('robot') or {}).get('odom_fresh') is True and
            (mission.get('health') or {}).get('status') == 'OK' and
            not (mission.get('health') or {}).get('blockers'))
        ir_zone_ok = (
            ir.get('zone') in ('APPROACH', 'TOO_FAR', 'SPRAY_RANGE') and
            ir.get('zone') == ir.get('ir_band') and
            ir.get('blockers') == [])
        stopped_proof = bool(
            base and ir_zone_ok and
            gateway.get('state') == 'HANDOFF_READY' and
            gateway.get('handoff_ready') is True and
            guard.get('state') == 'CANCEL_CONFIRMED_STOPPED' and
            guard.get('raw_cmd_publishers') == 0 and
            gateway.get('raw_publishers') == 0 and
            gateway.get('cmd_vel_fresh_zero') is True and
            gateway.get('odom_stationary') is True and
            mission.get('state') in ('CENTERED_HOLD', 'ALIGNMENT_READY'))
        return (uuid if exact_goal else None,
                bool(stopped_proof and ir.get('zone') in (
                    'APPROACH', 'TOO_FAR')),
                bool(base and ir_zone_ok and gateway.get('state') in (
                    'HANDOFF_READY', 'RELEASED_STOPPED') and
                     guard.get('state') in (
                         'CANCEL_CONFIRMED_STOPPED',
                         'IR_APPROACH_WAITING_FOR_PROOF',
                         'IR_APPROACH_IN_PROGRESS',
                         'WAITING_FOR_IR_STOP')),
                bool(stopped_proof and ir.get('zone') == 'SPRAY_RANGE'))

    @staticmethod
    def _exact_endpoint(info, name, message_type, reliability):
        return (info.node_name == name and info.node_namespace == '/' and
                info.topic_type == message_type and
                info.qos_profile.reliability == reliability)

    def _counts(self):
        now = time.monotonic()
        ir = self.get_publishers_info_by_topic(
            '/fire_mission/ir_range_preview')
        scan = self.get_publishers_info_by_topic('/scan')
        odom = self.get_publishers_info_by_topic('/odom')
        safety = self.get_publishers_info_by_topic('/cmd_vel')
        raw = self.get_publishers_info_by_topic('/cmd_vel_raw')
        gate = self.get_subscriptions_info_by_topic('/cmd_vel_raw')
        counts = {
            'ir': len(ir),
            'ir_owner': len(ir) == 1 and self._exact_endpoint(
                ir[0], 'ir_range_preview', 'std_msgs/msg/String',
                ReliabilityPolicy.BEST_EFFORT),
            'scan': len(scan),
            'scan_owner': len(scan) == 1 and self._endpoint_pins.matches(
                'scan', scan[0], name='scan_qos_relay',
                message_type='sensor_msgs/msg/LaserScan',
                reliability=ReliabilityPolicy.BEST_EFFORT),
            'odom': len(odom),
            'odom_owner': len(odom) == 1 and self._endpoint_pins.matches(
                'odom', odom[0], name='serial_bridge_node',
                message_type='nav_msgs/msg/Odometry',
                reliability=ReliabilityPolicy.BEST_EFFORT),
            'cmd': len(safety),
            'arm': self.count_publishers('/fire_mission/ir_approach_arm'),
            'raw': len(raw),
            'raw_owned': len(raw) == 1 and self._exact_endpoint(
                raw[0], 'ir_approach_live', 'geometry_msgs/msg/Twist',
                ReliabilityPolicy.BEST_EFFORT),
            'raw_gate': verified_raw_gate(
                self._endpoint_pins, gate, ReliabilityPolicy.BEST_EFFORT,
                (ReliabilityPolicy.BEST_EFFORT,
                 ReliabilityPolicy.RELIABLE),
                require_supervisor=self.core.integrated_handoff),
            'safety_owner': (len(safety) == 1 and
                             self._endpoint_pins.matches(
                                 'safety_output', safety[0],
                                 name='safety_watchdog',
                                 message_type='geometry_msgs/msg/Twist',
                                 reliability=ReliabilityPolicy.BEST_EFFORT)),
            'nav': self.count_publishers(
                '/navigate_to_pose/_action/status'),
            'nav_cmd': self.count_publishers('/cmd_vel_nav'),
            'pump': self.count_publishers('/pump_cmd'),
        }
        if self.core.integrated_handoff:
            for key, topic in (
                    ('mission', '/fire_mission/state'),
                    ('alignment', '/fire_mission/live_alignment_state'),
                    ('gateway', '/fire_mission/live_nav_stop_state'),
                    ('guard', '/fire_mission/nav2_cancel_guard_state'),
                    ('status', '/navigate_to_pose/_action/status')):
                infos = self.get_publishers_info_by_topic(topic)
                owner = {'mission': 'fire_mission_supervisor',
                         'alignment': 'live_alignment_takeover',
                         'gateway': 'live_nav_stop_gateway',
                         'guard': 'nav2_cancel_guard',
                         'status': 'bt_navigator'}[key]
                msg_type = ('action_msgs/msg/GoalStatusArray'
                            if key == 'status' else 'std_msgs/msg/String')
                reliability = (ReliabilityPolicy.RELIABLE
                               if key == 'status' else
                               ReliabilityPolicy.BEST_EFFORT)
                counts[key + '_source_ok'] = (
                    len(infos) == 1 and self._exact_endpoint(
                        infos[0], owner, msg_type, reliability))
            sources = [(info.node_namespace, info.node_name,
                        info.topic_type, info.qos_profile.reliability)
                       for info in self.get_publishers_info_by_topic(
                           '/fire_mission/nav2_cmd_vel_shadow')]
            counts['nav_shadow_ok'] = (
                Counter(sources) == EXPECTED_SHADOW_PUBLISHERS)
            uuid, admission, runtime, range_ready = self._handoff_evidence(
                now, counts)
            counts.update(goal_uuid=uuid, handoff_ok=admission,
                          runtime_ok=runtime, range_handoff_ok=range_ready)
            for key, topic, owner in (
                    ('review', '/fire_mission/post_spray_review',
                     'post_spray_review'),
                    ('actuator', '/fire_sensors/actuator_state',
                     'serial_bridge_node')):
                infos = self.get_publishers_info_by_topic(topic)
                counts[key + '_source_ok'] = (
                    len(infos) == 1 and self._exact_endpoint(
                        infos[0], owner, 'std_msgs/msg/String',
                        ReliabilityPolicy.BEST_EFFORT))
            guard = self._proof('guard', now) or {}
            counts['guard_owner_ack'] = (
                guard.get('state') == 'IR_APPROACH_IN_PROGRESS' and
                guard.get('ir_approach_handoff_active') is True and
                guard.get('goal_uuid') == uuid and
                guard.get('raw_cmd_publishers') == 1 and
                guard.get('blockers') == [])
        return counts

    def _tick(self):
        now = time.monotonic()
        speed, payload, action = self.core.step(
            now, self._counts(), self._raw_publisher is not None)
        if action == 'CREATE':
            self._raw_publisher = self.create_publisher(
                Twist, '/cmd_vel_raw', sensor_qos())
            self.get_logger().warning(
                'IR jog claimed raw command topic; waiting for exclusive '
                'publisher discovery before moving')
        if self._raw_publisher is not None:
            command = Twist()
            command.linear.x = float(speed) if math.isfinite(speed) else 0.0
            self._raw_publisher.publish(command)
        if action == 'RELEASE' and self._raw_publisher is not None:
            self.destroy_publisher(self._raw_publisher)
            self._raw_publisher = None
            self.get_logger().warning(
                'IR jog finished: %s; raw owner released' %
                self.core.stop_reason)
        state = String()
        state.data = json.dumps(payload, separators=(',', ':'),
                                allow_nan=False)
        self.state_publisher.publish(state)

    def destroy_node(self):
        if self._raw_publisher is not None:
            for _ in range(3):
                self._raw_publisher.publish(Twist())
                time.sleep(0.05)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IrApproachLiveNode()
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
