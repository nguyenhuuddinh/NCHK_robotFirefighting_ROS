"""Dashboard-facing, subscriber-only review of a completed pump pulse."""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fire_robot_mission.mission_supervisor import sensor_qos
from fire_robot_mission.post_spray_review_core import PostSprayReviewCore


class PostSprayReviewNode(Node):
    def __init__(self):
        super().__init__('post_spray_review')
        self.declare_parameter('settle_s', 1.0)
        self.declare_parameter('confirm_s', 1.0)
        self.declare_parameter('input_timeout_s', 0.6)
        self.declare_parameter('max_pulses', 3)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.core = PostSprayReviewCore(
            settle_s=self.get_parameter('settle_s').value,
            confirm_s=self.get_parameter('confirm_s').value,
            input_timeout_s=self.get_parameter('input_timeout_s').value,
            max_pulses=self.get_parameter('max_pulses').value)
        rate = self.get_parameter('publish_rate_hz').value
        if type(rate) not in (int, float) or not 1.0 <= rate <= 10.0:
            raise ValueError('publish_rate_hz must be 1-10 Hz')
        topics = {
            'pump': '/fire_mission/pump_demo_state',
            'vision': '/yolo/detections',
            'ir': '/fire_mission/ir_range_preview',
            'alignment': '/yolo/alignment_preview',
            'mission': '/fire_mission/state',
        }
        for name, topic in topics.items():
            if self.resolve_topic_name(topic) != topic:
                raise ValueError(
                    f'post-spray review forbids remapping {topic}')
            self.create_subscription(
                String, topic,
                lambda msg, key=name: self._on_json(key, msg), sensor_qos())
        output = '/fire_mission/post_spray_review'
        if self.resolve_topic_name(output) != output:
            raise ValueError('post-spray review output remap forbidden')
        self.publisher = self.create_publisher(String, output, sensor_qos())
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().warning(
            'Post-spray review is SHADOW_ONLY: no pump, motion or Nav2 action')

    def _on_json(self, name, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            payload = None
        self.core.update(name, payload, time.monotonic())

    def _tick(self):
        msg = String()
        msg.data = json.dumps(self.core.step(time.monotonic()),
                              separators=(',', ':'), allow_nan=False)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PostSprayReviewNode()
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
