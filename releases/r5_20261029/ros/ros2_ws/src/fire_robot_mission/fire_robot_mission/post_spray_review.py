"""Dashboard-facing, subscriber-only review of a completed pump pulse."""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import ReliabilityPolicy as RP
from .operational_contract import PULSE_NODE,PULSE_STATE_TOPIC,exact_endpoint,valid_gid
from .operational_review import OperationalReview

from fire_robot_mission.mission_supervisor import sensor_qos
from fire_robot_mission.post_spray_review_core import PostSprayReviewCore


class PostSprayReviewNode(Node):
    def __init__(self):
        super().__init__('post_spray_review')
        self.declare_parameter('use_operational_actor',False)
        self.declare_parameter('ir_confirm_samples',5)
        self.operational=self.get_parameter('use_operational_actor').value is True
        self.source_pins={};self.source_fault=None;self.sources={}
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
        if self.operational:
            self.core=OperationalReview(settle_s=self.get_parameter('settle_s').value,
                confirm_s=self.get_parameter('confirm_s').value,
                ir_confirm_samples=self.get_parameter('ir_confirm_samples').value)
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
        if self.operational:
            topics.pop('pump');topics['actor']=PULSE_STATE_TOPIC
            nodes=dict(actor=PULSE_NODE,vision='yolo_observer',ir='ir_range_preview',
                       alignment='fire_alignment_observer',mission='fire_mission_supervisor')
            self.sources={k:(t,nodes[k]) for k,t in topics.items()}
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

    def _sources_current(self):
        good=True
        for key,(topic,name) in self.sources.items():
            infos=self.get_publishers_info_by_topic(topic)
            if not exact_endpoint(infos,name,'std_msgs/msg/String',RP.BEST_EFFORT):
                good=False;continue
            gid=bytes(infos[0].endpoint_gid).hex()
            if key in self.source_pins and self.source_pins[key]!=gid:
                self.source_fault='REVIEW_SOURCE_GID_CHANGED:'+topic;good=False
            self.source_pins.setdefault(key,gid)
        return good and self.source_fault is None

    def _tick(self):
        msg = String()
        report=(self.core.step(time.monotonic(),graph_ok=self._sources_current())
                if self.operational else self.core.step(time.monotonic()))
        msg.data = json.dumps(report,
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
