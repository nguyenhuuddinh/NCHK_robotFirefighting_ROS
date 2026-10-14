"""Provisional, observe-only IR standoff bands; never command the robot."""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fire_robot_mission.mission_supervisor import sensor_qos
from fire_robot_mission.ir_range_core import IrRangePreviewCore, _number


FORBIDDEN_OUTPUT_TOPICS = {
    '/cmd_vel', '/cmd_vel_raw', '/cmd_vel_nav', '/pump_cmd', '/fire_target',
}


class IrRangePreviewNode(Node):
    """Subscribe to IR and vision; publish one isolated String preview."""

    def __init__(self):
        super().__init__('ir_range_preview')
        defaults = {
            'ir_topic': '/fire_sensors/ir_raw',
            'detection_topic': '/yolo/detections',
            'alignment_topic': '/yolo/alignment_preview',
            'preview_topic': '/fire_mission/ir_range_preview',
            'baseline_left': 26060,
            'baseline_center': 26140,
            'baseline_right': 25825,
            'baseline_tolerance': 500,
            'min_active_drop': 1000,
            'right_too_close_max': 850,
            'right_spray_max': 1100,
            'right_approach_max': 1850,
            'median_window': 5,
            'stable_samples': 3,
            'spray_confirm_samples': 5,
            'spray_reentry_margin_counts': 50,
            'ir_timeout_s': 0.6,
            'vision_timeout_s': 0.5,
            'max_ir_age_ms': 300.0,
            'max_image_age_ms': 300.0,
            'publish_rate_hz': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def p(key):
            return self.get_parameter(key).value
        topics = [p(key) for key in (
            'ir_topic', 'detection_topic', 'alignment_topic', 'preview_topic')]
        if any(not isinstance(topic, str) or not topic for topic in topics):
            raise ValueError('IR preview topic names must be nonempty')
        if len(set(topics)) != len(topics):
            raise ValueError('IR preview input/output topics must be unique')
        if topics[3] in FORBIDDEN_OUTPUT_TOPICS:
            raise ValueError('IR preview cannot publish a robot command')
        rate = p('publish_rate_hz')
        if not _number(rate) or rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.core = IrRangePreviewCore(
            baselines=tuple(p('baseline_' + channel) for channel in
                            ('left', 'center', 'right')),
            baseline_tolerance=p('baseline_tolerance'),
            min_active_drop=p('min_active_drop'),
            right_thresholds=tuple(p(key) for key in (
                'right_too_close_max', 'right_spray_max',
                'right_approach_max')),
            median_window=p('median_window'),
            stable_samples=p('stable_samples'),
            spray_confirm_samples=p('spray_confirm_samples'),
            spray_reentry_margin_counts=p('spray_reentry_margin_counts'),
            ir_timeout_s=p('ir_timeout_s'),
            vision_timeout_s=p('vision_timeout_s'),
            max_ir_age_ms=p('max_ir_age_ms'),
            max_image_age_ms=p('max_image_age_ms'),
        )
        qos = sensor_qos()
        self.create_subscription(String, topics[0], self._ir_callback, qos)
        self.create_subscription(String, topics[1], self._detection_callback,
                                 qos)
        self.create_subscription(String, topics[2], self._alignment_callback,
                                 qos)
        self._publisher = self.create_publisher(String, topics[3], qos)
        self.create_timer(1.0 / float(rate), self._publish)
        self.get_logger().warning(
            'IR range is a provisional OBSERVE-ONLY preview; no motion or '
            'pump command is published')

    def _ir_callback(self, msg):
        now = time.monotonic()
        try:
            self.core.update_ir(json.loads(msg.data), now)
        except (ValueError, TypeError):
            self.core.mark_ir_error(now)

    def _detection_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            payload = None
        self.core.update_detection(payload, time.monotonic())

    def _alignment_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            payload = None
        self.core.update_alignment(payload, time.monotonic())

    def _publish(self):
        msg = String()
        msg.data = json.dumps(self.core.snapshot(time.monotonic()),
                              separators=(',', ':'), allow_nan=False)
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IrRangePreviewNode()
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
