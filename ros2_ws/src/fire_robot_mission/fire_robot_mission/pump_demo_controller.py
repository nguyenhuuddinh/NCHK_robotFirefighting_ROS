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
        for topic in ('/pump_cmd', '/fire_mission/pump_demo_arm',
                      '/fire_mission/ir_range_preview',
                      '/fire_mission/state',
                      '/fire_mission/live_alignment_state',
                      '/fire_mission/live_nav_stop_state',
                      '/fire_mission/nav2_cancel_guard_state',
                      '/navigate_to_pose/_action/status',
                      '/fire_mission/pump_demo_state', '/odom', '/cmd_vel',
                      '/cmd_vel_raw'):
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
        if self.core.enabled:
            self.pump_pub = self.create_publisher(Bool, '/pump_cmd', reliable)
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
        self.create_timer(0.1, self._tick)
        self.get_logger().warning(
            '[PUMP DEMO] enabled=%s provisional_override=%s; one-shot '
            'pulse_s=%.1f s (5 s maximum), servo disabled' %
            (self.core.enabled, self.core.allow_provisional_ir,
             self.core.pulse_s))

    def _on_json(self, name, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            payload = None
        self.core.update(name, payload, time.monotonic())

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
        counts = {key: self._exact_publisher_count(key)
                  for key in EXPECTED_PUBLISHERS}
        counts.update({
            'arm': self.count_publishers('/fire_mission/pump_demo_arm'),
            'pump_subscribers': self._exact_bridge_subscriber_count(),
            'raw': self.count_publishers('/cmd_vel_raw'),
        })
        on, report = self.core.step(time.monotonic(), counts)
        if self.pump_pub is not None:
            command = Bool()
            command.data = on
            self.pump_pub.publish(command)
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
