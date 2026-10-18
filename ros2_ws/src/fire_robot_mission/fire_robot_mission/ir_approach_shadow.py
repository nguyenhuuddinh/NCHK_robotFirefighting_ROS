"""Observe IR, odom and front LiDAR before any live approach work."""

import json
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from fire_robot_safety.odom_freshness import OdomStampGuard, admit_odom
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from fire_robot_mission.ir_approach_shadow_core import IrApproachShadowCore
from fire_robot_mission.mission_supervisor import sensor_qos


class IrApproachShadowNode(Node):
    """Publish only an isolated String decision preview."""

    def __init__(self):
        super().__init__('ir_approach_shadow')
        self._odom_stamp_guard = OdomStampGuard()
        self.declare_parameter('step_m', 0.05)
        self.declare_parameter('front_half_angle_rad', 0.35)
        self.declare_parameter('front_margin_m', 0.45)
        self.declare_parameter('timeout_s', 0.35)
        self.core = IrApproachShadowCore(**{
            key: self.get_parameter(key).value for key in
            ('step_m', 'front_half_angle_rad', 'front_margin_m',
             'timeout_s')
        })
        qos = sensor_qos()
        self.create_subscription(String, '/fire_mission/ir_range_preview',
                                 self._ir, qos)
        self.create_subscription(LaserScan, '/scan', self._scan, qos)
        self.create_subscription(Odometry, '/odom', self._odom, qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd, qos)
        self.publisher = self.create_publisher(
            String, '/fire_mission/ir_approach_shadow', qos)
        self.create_timer(0.1, self._publish)
        self.get_logger().warning(
            'IR approach SHADOW ONLY: this node cannot move or pump')

    def _ir(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            payload = None
        self.core.update('ir', payload, time.monotonic())

    def _scan(self, msg):
        self.core.update('scan', (
            msg.angle_min, msg.angle_increment, msg.range_min,
            msg.range_max, tuple(msg.ranges)), time.monotonic())

    def _odom(self, msg):
        received = admit_odom(self, msg, self.core.timeout_s)
        if received is None:
            return
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        self.core.update('odom', (linear.x, linear.y, linear.z,
                                  angular.x, angular.y, angular.z),
                         received)

    def _cmd(self, msg):
        self.core.update('cmd', (msg.linear.x, msg.linear.y, msg.linear.z,
                                 msg.angular.x, msg.angular.y, msg.angular.z),
                         time.monotonic())

    def _publish(self):
        counts = {
            'ir': self.count_publishers('/fire_mission/ir_range_preview'),
            'scan': self.count_publishers('/scan'),
            'odom': self.count_publishers('/odom'),
            'cmd': self.count_publishers('/cmd_vel'),
            'raw': self.count_publishers('/cmd_vel_raw'),
        }
        msg = String()
        msg.data = json.dumps(self.core.snapshot(time.monotonic(), counts),
                              separators=(',', ':'), allow_nan=False)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IrApproachShadowNode()
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
