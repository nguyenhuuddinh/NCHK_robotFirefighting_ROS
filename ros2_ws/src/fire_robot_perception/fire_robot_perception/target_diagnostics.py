"""
Summarize observe-only fire-target telemetry without publishing commands.

The diagnostic subscribes to the JSON stream produced by ``yolo_observer``.
It owns no publisher and is safe to run while the robot is stationary.
"""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


def diagnostic_qos():
    """Use volatile depth-one telemetry so diagnostics cannot make backlog."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def metric_summary(values):
    """Return compact descriptive statistics for a numeric sample list."""
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    percentile_index = max(0, int(0.95 * len(ordered) + 0.999999) - 1)
    return {
        'samples': len(ordered),
        'min': round(ordered[0], 3),
        'mean': round(statistics.mean(ordered), 3),
        'p95': round(ordered[percentile_index], 3),
        'max': round(ordered[-1], 3),
    }


def axis_summary(values):
    """Summarize center position and jitter along one normalized axis."""
    if not values:
        return None
    numeric = [float(value) for value in values]
    jitter = statistics.pstdev(numeric) if len(numeric) > 1 else 0.0
    return {
        'samples': len(numeric),
        'mean': round(statistics.mean(numeric), 4),
        'median': round(statistics.median(numeric), 4),
        'min': round(min(numeric), 4),
        'max': round(max(numeric), 4),
        'jitter_stddev': round(jitter, 4),
    }


class TargetDiagnosticsAccumulator:
    """Accumulate detection, tracking and latency evidence from JSON frames."""

    def __init__(self):
        self.message_count = 0
        self.invalid_json_count = 0
        self.observe_only_violations = 0
        self.state_counts = Counter()
        self.class_counts = Counter()
        self.aim_state_counts = Counter()
        self.center_filter_counts = Counter()
        self.detected_frames = 0
        self.confirmed_frames = 0
        self.confirmation_events = 0
        self.lost_events = 0
        self._previous_confirmed = False
        self._lost_since = None
        self.reacquire_ms = []
        self.candidate_confidences = []
        self.candidate_x = []
        self.candidate_y = []
        self.smoothed_x = []
        self.smoothed_y = []
        self.source_age_ms = []
        self.processing_ms = []
        self.inference_ms = []
        self.first_receive_time = None
        self.last_receive_time = None

    def add_invalid_json(self):
        """Count one message that could not be decoded as a JSON object."""
        self.invalid_json_count += 1

    def add(self, payload, receive_time=None):
        """Consume one decoded observer payload."""
        receive_time = (
            time.monotonic() if receive_time is None else receive_time
        )
        if self.first_receive_time is None:
            self.first_receive_time = receive_time
        self.last_receive_time = receive_time
        self.message_count += 1

        if payload.get('observe_only') is not True:
            self.observe_only_violations += 1
        self.state_counts[str(payload.get('state', 'missing'))] += 1

        for detection in payload.get('detections') or []:
            class_name = str(detection.get('class_name', 'unknown'))
            self.class_counts[class_name] += 1

        source = payload.get('source') or {}
        self._append_number(self.source_age_ms, source.get('age_ms'))
        self._append_number(self.processing_ms, payload.get('processing_ms'))
        self._append_number(self.inference_ms, payload.get('inference_ms'))

        fire = payload.get('fire') or {}
        center_filter = fire.get('center_filter') or {}
        filter_method = center_filter.get('method')
        if filter_method:
            alpha = center_filter.get('alpha')
            label = str(filter_method)
            if isinstance(alpha, (int, float)):
                label += f'(alpha={float(alpha):.2f})'
            self.center_filter_counts[label] += 1
        detected = bool(fire.get('detected_now', False))
        confirmed = bool(fire.get('confirmed', False))
        if detected:
            self.detected_frames += 1
        if confirmed:
            self.confirmed_frames += 1
        if confirmed and not self._previous_confirmed:
            self.confirmation_events += 1
            if self._lost_since is not None:
                self.reacquire_ms.append(
                    max(0.0, receive_time - self._lost_since) * 1000.0)
                self._lost_since = None
        if self._previous_confirmed and not confirmed:
            self.lost_events += 1
            self._lost_since = receive_time
        self._previous_confirmed = confirmed

        candidate = fire.get('candidate')
        if candidate:
            self._append_number(
                self.candidate_confidences, candidate.get('confidence'))
            center = candidate.get('center_normalized')
            self._append_center(center, self.candidate_x, self.candidate_y)

        smoothed = fire.get('smoothed_center_normalized')
        self._append_center(smoothed, self.smoothed_x, self.smoothed_y)
        aim = fire.get('aim') or {}
        self.aim_state_counts[str(aim.get('state', 'NO_TARGET'))] += 1

    @staticmethod
    def _append_number(destination, value):
        """Append a finite-looking numeric value while ignoring nulls."""
        if isinstance(value, (int, float)):
            destination.append(float(value))

    @classmethod
    def _append_center(cls, center, destination_x, destination_y):
        """Append a two-axis center when both values are numeric."""
        if not isinstance(center, (list, tuple)) or len(center) != 2:
            return
        if not all(isinstance(value, (int, float)) for value in center):
            return
        destination_x.append(float(center[0]))
        destination_y.append(float(center[1]))

    def summary(self):
        """Build a JSON-safe final report."""
        duration = 0.0
        if self.first_receive_time is not None and \
                self.last_receive_time is not None:
            duration = max(
                0.0, self.last_receive_time - self.first_receive_time
            )
        rate = 0.0
        if duration > 0.0 and self.message_count > 1:
            rate = (self.message_count - 1) / duration
        detected_rate = (
            self.detected_frames / self.message_count
            if self.message_count else 0.0
        )
        confirmed_rate = (
            self.confirmed_frames / self.message_count
            if self.message_count else 0.0
        )
        return {
            'mode': 'OBSERVE_ONLY',
            'subscriber_only': True,
            'messages': self.message_count,
            'invalid_json': self.invalid_json_count,
            'observe_only_violations': self.observe_only_violations,
            'duration_s': round(duration, 3),
            'message_rate_hz': round(rate, 3),
            'state_counts': dict(sorted(self.state_counts.items())),
            'class_detection_counts': dict(sorted(self.class_counts.items())),
            'fire': {
                'detected_frames': self.detected_frames,
                'detected_rate': round(detected_rate, 4),
                'confirmed_frames': self.confirmed_frames,
                'confirmed_rate': round(confirmed_rate, 4),
                'confirmation_events': self.confirmation_events,
                'lost_events': self.lost_events,
                'reacquire_ms': metric_summary(self.reacquire_ms),
                'confidence': metric_summary(self.candidate_confidences),
                'candidate_center': {
                    'x': axis_summary(self.candidate_x),
                    'y': axis_summary(self.candidate_y),
                },
                'smoothed_center': {
                    'x': axis_summary(self.smoothed_x),
                    'y': axis_summary(self.smoothed_y),
                },
                'aim_state_counts': dict(
                    sorted(self.aim_state_counts.items())),
                'center_filter_counts': dict(
                    sorted(self.center_filter_counts.items())),
            },
            'latency_ms': {
                'source_age': metric_summary(self.source_age_ms),
                'inference': metric_summary(self.inference_ms),
                'processing': metric_summary(self.processing_ms),
            },
        }


def progress_line(payload):
    """Format one compact human-readable observation line."""
    state = str(payload.get('state', 'missing'))
    fire = payload.get('fire') or {}
    candidate = fire.get('candidate') or {}
    aim = fire.get('aim') or {}
    confidence = candidate.get('confidence')
    confidence_text = '-' if confidence is None else f'{confidence:.2f}'
    center = fire.get('smoothed_center_normalized')
    center_text = '(-,-)'
    if isinstance(center, (list, tuple)) and len(center) == 2:
        center_text = f'({center[0]:+.2f},{center[1]:+.2f})'
    source_age = (payload.get('source') or {}).get('age_ms')
    age_text = '-' if source_age is None else f'{source_age:.0f}'
    processing = payload.get('processing_ms')
    processing_text = '-' if processing is None else f'{processing:.0f}'
    center_filter = fire.get('center_filter') or {}
    filter_text = str(center_filter.get('method', '-'))
    alpha = center_filter.get('alpha')
    if isinstance(alpha, (int, float)):
        filter_text += f'({float(alpha):.2f})'
    return (
        f'state={state} detected={bool(fire.get("detected_now", False))} '
        f'confirmed={bool(fire.get("confirmed", False))} '
        f'hits={fire.get("hits", 0)}/{fire.get("window_frames", 0)} '
        f'conf={confidence_text} center={center_text} '
        f'filter={filter_text} '
        f'aim={aim.get("state", "NO_TARGET")} '
        f'age={age_text}ms proc={processing_text}ms'
    )


class TargetDiagnosticsNode(Node):
    """Subscribe to target telemetry and retain the latest decoded payload."""

    def __init__(self, topic, accumulator):
        super().__init__('yolo_target_diagnostics')
        self.accumulator = accumulator
        self.latest_payload = None
        self.create_subscription(
            String,
            topic,
            self._on_message,
            diagnostic_qos(),
        )

    def _on_message(self, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('JSON root is not an object')
        except (json.JSONDecodeError, TypeError, ValueError):
            self.accumulator.add_invalid_json()
            return
        self.accumulator.add(payload)
        self.latest_payload = payload


def _argument_parser():
    parser = argparse.ArgumentParser(
        description='Observe-only YOLO target stability and latency report')
    parser.add_argument(
        '--topic', default='/yolo/detections',
        help='std_msgs/String JSON observation topic')
    parser.add_argument(
        '--seconds', type=float, default=30.0,
        help='measurement duration in seconds')
    parser.add_argument(
        '--progress-period', type=float, default=1.0,
        help='seconds between compact progress lines')
    parser.add_argument(
        '--output', default='',
        help='optional path for the final JSON report')
    return parser


def main(args=None):
    """Run the subscriber-only diagnostic for a bounded duration."""
    parser = _argument_parser()
    options, ros_args = parser.parse_known_args(args)
    if options.seconds <= 0.0:
        parser.error('--seconds must be positive')
    if options.progress_period <= 0.0:
        parser.error('--progress-period must be positive')

    rclpy.init(args=ros_args)
    accumulator = TargetDiagnosticsAccumulator()
    node = TargetDiagnosticsNode(options.topic, accumulator)
    started = time.monotonic()
    deadline = started + options.seconds
    next_progress = started + options.progress_period
    print('=== YOLO TARGET DIAGNOSTICS START ===', flush=True)
    print(
        f'MODE OBSERVE_ONLY subscriber_only=True topic={options.topic} '
        f'duration_s={options.seconds:.1f}',
        flush=True,
    )
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(node, timeout_sec=min(0.2, remaining))
            now = time.monotonic()
            if now >= next_progress:
                if node.latest_payload is None:
                    print('WAITING no telemetry received', flush=True)
                else:
                    print(progress_line(node.latest_payload), flush=True)
                next_progress = now + options.progress_period
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        report = accumulator.summary()
        report['generated_utc'] = datetime.now(timezone.utc).isoformat()
        report['topic'] = options.topic
        report_text = json.dumps(report, indent=2, sort_keys=True)
        print('=== YOLO TARGET DIAGNOSTICS REPORT ===')
        print(report_text)
        if options.output:
            output_path = Path(options.output).expanduser()
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_text(report_text + '\n', encoding='utf-8')
            print(f'REPORT_FILE {output_path}')
        print('=== YOLO TARGET DIAGNOSTICS END ===')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
