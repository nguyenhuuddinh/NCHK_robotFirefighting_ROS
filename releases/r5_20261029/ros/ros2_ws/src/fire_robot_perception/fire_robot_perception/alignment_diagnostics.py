"""Summarize observe-only yaw previews without publishing anything."""

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
from std_msgs.msg import String

from fire_robot_perception.fire_alignment_observer import telemetry_qos


def metric_summary(values):
    """Return descriptive statistics for a numeric list."""
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    percentile_index = max(0, int(0.95 * len(ordered) + 0.999999) - 1)
    return {
        'samples': len(ordered),
        'min': round(ordered[0], 4),
        'mean': round(statistics.mean(ordered), 4),
        'p95': round(ordered[percentile_index], 4),
        'max': round(ordered[-1], 4),
    }


class AlignmentDiagnosticsAccumulator:
    """Accumulate preview states, magnitudes and direction transitions."""

    def __init__(self):
        self.message_count = 0
        self.invalid_json_count = 0
        self.observe_only_violations = 0
        self.safety_contract_violations = 0
        self.direction_sign_violations = 0
        self.stop_nonzero_violations = 0
        self.speed_limit_violations = 0
        self.state_counts = Counter()
        self.direction_counts = Counter()
        self.slew_limited_frames = 0
        self.saturated_frames = 0
        self.direction_changes = 0
        self._previous_turn_direction = None
        self.error_x = []
        self.target_speed = []
        self.proposed_speed = []
        self.first_receive_time = None
        self.last_receive_time = None

    def add_invalid_json(self):
        """Count one malformed telemetry message."""
        self.invalid_json_count += 1

    def add(self, payload, receive_time=None):
        """Consume one decoded alignment preview."""
        receive_time = (
            time.monotonic() if receive_time is None else receive_time
        )
        if self.first_receive_time is None:
            self.first_receive_time = receive_time
        self.last_receive_time = receive_time
        self.message_count += 1
        if payload.get('observe_only') is not True:
            self.observe_only_violations += 1
        safety = payload.get('safety') or {}
        if safety.get('motion_command_publisher') is not False or \
                safety.get('actuator_command_publisher') is not False:
            self.safety_contract_violations += 1

        state = str(payload.get('state', 'missing'))
        self.state_counts[state] += 1
        preview = payload.get('preview') or {}
        direction = str(preview.get('direction', 'missing'))
        self.direction_counts[direction] += 1
        proposed = preview.get('proposed_angular_z_rad_s')
        if isinstance(proposed, (int, float)):
            proposed = float(proposed)
            if direction == 'RIGHT' and proposed > 1e-6:
                self.direction_sign_violations += 1
            elif direction == 'LEFT' and proposed < -1e-6:
                self.direction_sign_violations += 1
            elif direction == 'STOP' and abs(proposed) > 1e-6:
                self.stop_nonzero_violations += 1
            max_speed = (payload.get('limits') or {}).get(
                'max_angular_speed_rad_s')
            if isinstance(max_speed, (int, float)) and \
                    abs(proposed) > float(max_speed) + 1e-6:
                self.speed_limit_violations += 1
        if preview.get('slew_limited') is True:
            self.slew_limited_frames += 1
        if preview.get('saturated') is True:
            self.saturated_frames += 1

        if direction in ('LEFT', 'RIGHT'):
            if self._previous_turn_direction is not None and \
                    direction != self._previous_turn_direction:
                self.direction_changes += 1
            self._previous_turn_direction = direction
        self._append_number(
            self.error_x, preview.get('error_x_normalized'))
        self._append_number(
            self.target_speed, preview.get('target_angular_z_rad_s'))
        self._append_number(
            self.proposed_speed, preview.get('proposed_angular_z_rad_s'))

    @staticmethod
    def _append_number(destination, value):
        if isinstance(value, (int, float)):
            destination.append(float(value))

    def summary(self):
        """Build the final JSON-safe report."""
        duration = 0.0
        if self.first_receive_time is not None and \
                self.last_receive_time is not None:
            duration = max(
                0.0, self.last_receive_time - self.first_receive_time
            )
        rate = 0.0
        if duration > 0.0 and self.message_count > 1:
            rate = (self.message_count - 1) / duration
        return {
            'mode': 'OBSERVE_ONLY',
            'subscriber_only': True,
            'messages': self.message_count,
            'invalid_json': self.invalid_json_count,
            'observe_only_violations': self.observe_only_violations,
            'safety_contract_violations': self.safety_contract_violations,
            'direction_sign_violations': self.direction_sign_violations,
            'stop_nonzero_violations': self.stop_nonzero_violations,
            'speed_limit_violations': self.speed_limit_violations,
            'duration_s': round(duration, 3),
            'message_rate_hz': round(rate, 3),
            'state_counts': dict(sorted(self.state_counts.items())),
            'direction_counts': dict(sorted(self.direction_counts.items())),
            'direction_changes': self.direction_changes,
            'slew_limited_frames': self.slew_limited_frames,
            'saturated_frames': self.saturated_frames,
            'error_x_normalized': metric_summary(self.error_x),
            'target_angular_z_rad_s': metric_summary(self.target_speed),
            'proposed_angular_z_rad_s': metric_summary(
                self.proposed_speed),
        }


def progress_line(payload):
    """Format one compact alignment preview line."""
    preview = payload.get('preview') or {}
    error_x = preview.get('error_x_normalized')
    error_text = '-' if error_x is None else f'{float(error_x):+.3f}'
    target = float(preview.get('target_angular_z_rad_s', 0.0))
    proposed = float(preview.get('proposed_angular_z_rad_s', 0.0))
    return (
        f'state={payload.get("state", "missing")} '
        f'direction={preview.get("direction", "missing")} '
        f'error_x={error_text} target_wz={target:+.3f} '
        f'preview_wz={proposed:+.3f} '
        f'slew={bool(preview.get("slew_limited", False))} '
        f'saturated={bool(preview.get("saturated", False))}'
    )


class AlignmentDiagnosticsNode(Node):
    """Subscribe to alignment previews and retain the latest payload."""

    def __init__(self, topic, accumulator):
        super().__init__('alignment_preview_diagnostics')
        self.accumulator = accumulator
        self.latest_payload = None
        self.create_subscription(
            String, topic, self._on_message, telemetry_qos())

    def _on_message(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('JSON root is not an object')
        except (json.JSONDecodeError, TypeError, ValueError):
            self.accumulator.add_invalid_json()
            return
        self.accumulator.add(payload)
        self.latest_payload = payload


def _argument_parser():
    parser = argparse.ArgumentParser(
        description='Observe-only fire alignment preview report')
    parser.add_argument(
        '--topic', default='/yolo/alignment_preview',
        help='std_msgs/String JSON alignment preview topic')
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
    """Run subscriber-only diagnostics for a bounded duration."""
    parser = _argument_parser()
    options, ros_args = parser.parse_known_args(args)
    if options.seconds <= 0.0:
        parser.error('--seconds must be positive')
    if options.progress_period <= 0.0:
        parser.error('--progress-period must be positive')

    rclpy.init(args=ros_args)
    accumulator = AlignmentDiagnosticsAccumulator()
    node = AlignmentDiagnosticsNode(options.topic, accumulator)
    started = time.monotonic()
    deadline = started + options.seconds
    next_progress = started + options.progress_period
    print('=== ALIGNMENT PREVIEW DIAGNOSTICS START ===', flush=True)
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
                    print('WAITING no preview received', flush=True)
                else:
                    print(progress_line(node.latest_payload), flush=True)
                next_progress = now + options.progress_period
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        report = accumulator.summary()
        report['generated_utc'] = datetime.now(timezone.utc).isoformat()
        report['topic'] = options.topic
        print('=== ALIGNMENT PREVIEW DIAGNOSTICS REPORT ===')
        print(json.dumps(report, indent=2, sort_keys=True))
        if options.output:
            output_path = Path(options.output).expanduser()
            output_path.write_text(
                json.dumps(report, indent=2, sort_keys=True) + '\n')
            print(f'REPORT_FILE {output_path}')
        print('=== ALIGNMENT PREVIEW DIAGNOSTICS END ===')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
