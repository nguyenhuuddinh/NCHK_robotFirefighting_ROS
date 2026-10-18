"""Optional, bounded, read-only ROS telemetry observer.

Run this executable separately from the robot launch. It never creates a
publisher, action client, service client, or command endpoint.
"""

import argparse
import json
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu, LaserScan
from std_msgs.msg import String


TOPICS = {
    'pi': (
        ('/odom', Odometry),
        ('/imu/data', Imu),
        ('/scan', LaserScan),
        ('/fire_sensors/ir_raw', String),
    ),
    'laptop': (
        ('/odom', Odometry),
        ('/scan', LaserScan),
        ('/yolo/detections', String),
    ),
}
MAX_DURATION_S = 60.0
MAX_FILE_BYTES = 64 * 1024 * 1024


class BoundedJsonl:
    """Stop recording at the byte budget without blocking robot callbacks."""

    def __init__(self, path, max_bytes):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('x', encoding='utf-8', buffering=65536)
        self.max_bytes = max_bytes
        self.bytes_written = 0
        self.full = False

    def write(self, item):
        line = json.dumps(item, separators=(',', ':'), allow_nan=False) + '\n'
        size = len(line.encode('utf-8'))
        if self.bytes_written + size > self.max_bytes:
            self.full = True
            return False
        self.stream.write(line)
        self.bytes_written += size
        return True

    def close(self):
        self.stream.close()


class RuntimeDiagnostics(Node):
    """Record receive gaps and header stamps without deserializing images."""

    def __init__(self, role, mode, output, include_image=False):
        super().__init__('runtime_diagnostics')
        self.mode = mode
        self.output = output
        self.window_started = time.monotonic()
        self.last = {}
        self.counts = {}
        self.max_gaps_ms = {}
        topics = TOPICS[role]
        if include_image:
            topics += (('/image_raw/compressed', CompressedImage),)
        for topic, message_type in topics:
            self.create_subscription(
                message_type, topic,
                lambda message, name=topic: self._on_message(name, message),
                qos_profile_sensor_data,
            )

    def _on_message(self, topic, message):
        now = time.monotonic()
        previous = self.last.get(topic)
        gap_ms = None if previous is None else (now - previous) * 1000.0
        self.last[topic] = now
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if gap_ms is not None:
            self.max_gaps_ms[topic] = max(
                self.max_gaps_ms.get(topic, 0.0), gap_ms)
        if self.mode == 'trace':
            header = getattr(message, 'header', None)
            stamp = getattr(header, 'stamp', None)
            self.output.write({
                'kind': 'event', 'topic': topic, 'mono_ns': time.monotonic_ns(),
                'wall_ns': time.time_ns(),
                'header_ns': (stamp.sec * 1_000_000_000 + stamp.nanosec)
                if stamp is not None else None,
                'gap_ms': gap_ms,
                'payload_bytes': len(message.data)
                if isinstance(message, CompressedImage) else None,
            })

    def flush_summary(self):
        now = time.monotonic()
        self.output.write({
            'kind': 'summary', 'mono_ns': time.monotonic_ns(),
            'window_s': now - self.window_started,
            'counts': self.counts,
            'max_gaps_ms': self.max_gaps_ms,
        })
        self.window_started = now
        self.counts = {}
        self.max_gaps_ms = {}


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=sorted(TOPICS), required=True)
    parser.add_argument('--mode', choices=('summary', 'trace'), required=True)
    parser.add_argument('--duration-s', type=float, default=30.0)
    parser.add_argument('--max-bytes', type=int, default=MAX_FILE_BYTES)
    parser.add_argument('--include-image', action='store_true',
                        help='Subscribe to compressed JPEG payloads (extra load)')
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args(args)
    if not 0.0 < options.duration_s <= MAX_DURATION_S:
        parser.error(f'duration-s must be within (0, {MAX_DURATION_S}]')
    if not 1024 <= options.max_bytes <= MAX_FILE_BYTES:
        parser.error(f'max-bytes must be within [1024, {MAX_FILE_BYTES}]')
    return options


def main(args=None):
    options = parse_args(args)
    output = BoundedJsonl(options.output, options.max_bytes)
    node = None
    reason = 'duration_elapsed'
    started = time.monotonic()
    try:
        rclpy.init()
        node = RuntimeDiagnostics(options.role, options.mode, output,
                                  include_image=options.include_image)
        deadline = started + options.duration_s
        next_summary = started + 1.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            rclpy.spin_once(node, timeout_sec=max(
                0.0, min(0.2, deadline - now)))
            now = time.monotonic()
            if options.mode == 'summary' and now >= next_summary:
                node.flush_summary()
                next_summary = now + 1.0
            if output.full:
                reason = 'size_budget'
                break
        if node is not None and options.mode == 'summary' and not output.full:
            node.flush_summary()
        if not rclpy.ok() and reason == 'duration_elapsed':
            reason = 'ros_shutdown'
    except KeyboardInterrupt:
        reason = 'interrupted'
    except BaseException:
        reason = 'error'
        raise
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                output.close()
                receipt = options.output.with_name(
                    options.output.name + '.receipt.json')
                receipt.write_text(json.dumps({
                    'role': options.role,
                    'mode': options.mode,
                    'include_image': options.include_image,
                    'reason': reason,
                    'duration_s': time.monotonic() - started,
                    'bytes_written': output.bytes_written,
                    'max_bytes': options.max_bytes,
                }, indent=2) + '\n')


if __name__ == '__main__':
    main()
