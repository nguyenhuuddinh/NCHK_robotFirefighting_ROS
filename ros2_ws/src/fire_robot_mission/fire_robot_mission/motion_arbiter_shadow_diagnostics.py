"""Summarize shadow motion ownership without publishing commands."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fire_robot_mission.mission_supervisor import sensor_qos


class MotionArbiterShadowDiagnosticsAccumulator:
    """Accumulate arbiter states and fail-closed contract violations."""

    def __init__(
            self, max_nav_linear_speed_m_s=0.10,
            max_nav_angular_speed_rad_s=0.8,
            max_alignment_angular_speed_rad_s=0.35):
        limits = (
            max_nav_linear_speed_m_s,
            max_nav_angular_speed_rad_s,
            max_alignment_angular_speed_rad_s,
        )
        if any(limit <= 0.0 for limit in limits):
            raise ValueError('motion arbiter limits must be positive')
        self.max_nav_linear = float(max_nav_linear_speed_m_s)
        self.max_nav_angular = float(max_nav_angular_speed_rad_s)
        self.max_alignment_angular = float(
            max_alignment_angular_speed_rad_s)
        self.messages = 0
        self.invalid_json = 0
        self.shadow_contract_violations = 0
        self.exclusive_owner_violations = 0
        self.zero_state_violations = 0
        self.output_limit_violations = 0
        self.owner_state_violations = 0
        self.state_counts = Counter()
        self.owner_counts = Counter()
        self.blocker_counts = Counter()
        self.request_counts = Counter()
        self.clamped_frames = 0
        self.max_abs_linear = 0.0
        self.max_abs_angular = 0.0
        self.first_receive_time = None
        self.last_receive_time = None

    def add_invalid_json(self):
        self.invalid_json += 1

    def add(self, payload, receive_time=None):
        """Consume one decoded motion arbiter shadow state."""
        receive_time = (
            time.monotonic() if receive_time is None else receive_time)
        if self.first_receive_time is None:
            self.first_receive_time = receive_time
        self.last_receive_time = receive_time
        self.messages += 1

        safety = payload.get('safety') or {}
        if payload.get('shadow_only') is not True or \
                safety.get('real_motion_output') is not False or \
                safety.get('actuator_command_publisher') is not False or \
                safety.get('nav2_action_client') is not False or \
                safety.get('exclusive_owner') is not True:
            self.shadow_contract_violations += 1

        state = str(payload.get('state', 'missing'))
        owner = str(payload.get('owner', 'missing'))
        self.state_counts[state] += 1
        self.owner_counts[owner] += 1
        expected_owner = {
            'NAVIGATION': 'NAVIGATION',
            'ALIGNMENT': 'ALIGNMENT',
            'IDLE': 'NONE',
            'SAFE_STOP': 'NONE',
        }.get(state)
        if expected_owner is None or owner != expected_owner:
            self.owner_state_violations += 1

        requests = payload.get('requests') or {}
        nav_requested = requests.get('navigation') is True
        alignment_requested = requests.get('alignment') is True
        conflict = nav_requested and alignment_requested
        request_label = (
            'CONFLICT' if conflict else
            'NAVIGATION' if nav_requested else
            'ALIGNMENT' if alignment_requested else 'NONE'
        )
        self.request_counts[request_label] += 1
        if conflict and (state != 'SAFE_STOP' or owner != 'NONE'):
            self.exclusive_owner_violations += 1

        admission = payload.get('admission') or {}
        blockers = admission.get('blockers', [])
        if isinstance(blockers, list):
            self.blocker_counts.update(str(item) for item in blockers)
        else:
            self.blocker_counts['INVALID_BLOCKER_LIST'] += 1

        command = payload.get('shadow_command') or {}
        linear = command.get('linear_x_m_s')
        angular = command.get('angular_z_rad_s')
        if not isinstance(linear, (int, float)) or \
                not math.isfinite(float(linear)):
            self.output_limit_violations += 1
            linear = 0.0
        if not isinstance(angular, (int, float)) or \
                not math.isfinite(float(angular)):
            self.output_limit_violations += 1
            angular = 0.0
        linear = float(linear)
        angular = float(angular)
        self.max_abs_linear = max(self.max_abs_linear, abs(linear))
        self.max_abs_angular = max(self.max_abs_angular, abs(angular))

        if state in {'IDLE', 'SAFE_STOP'} and \
                (abs(linear) > 1e-9 or abs(angular) > 1e-9):
            self.zero_state_violations += 1
        if state == 'NAVIGATION' and (
                abs(linear) > self.max_nav_linear + 1e-9 or
                abs(angular) > self.max_nav_angular + 1e-9):
            self.output_limit_violations += 1
        if state == 'ALIGNMENT' and (
                abs(linear) > 1e-9 or
                abs(angular) > self.max_alignment_angular + 1e-9):
            self.output_limit_violations += 1
        if command.get('linear_clamped') is True or \
                command.get('angular_clamped') is True:
            self.clamped_frames += 1

    def summary(self):
        """Return one JSON-safe final report."""
        duration = 0.0
        if self.first_receive_time is not None and \
                self.last_receive_time is not None:
            duration = max(
                0.0, self.last_receive_time - self.first_receive_time)
        rate = 0.0
        if duration > 0.0 and self.messages > 1:
            rate = (self.messages - 1) / duration
        return {
            'mode': 'SHADOW_ONLY',
            'subscriber_only': True,
            'messages': self.messages,
            'duration_s': round(duration, 3),
            'message_rate_hz': round(rate, 3),
            'invalid_json': self.invalid_json,
            'shadow_contract_violations': (
                self.shadow_contract_violations),
            'exclusive_owner_violations': (
                self.exclusive_owner_violations),
            'owner_state_violations': self.owner_state_violations,
            'zero_state_violations': self.zero_state_violations,
            'output_limit_violations': self.output_limit_violations,
            'clamped_frames': self.clamped_frames,
            'max_abs_shadow_linear_x_m_s': round(
                self.max_abs_linear, 4),
            'max_abs_shadow_angular_z_rad_s': round(
                self.max_abs_angular, 4),
            'state_counts': dict(sorted(self.state_counts.items())),
            'owner_counts': dict(sorted(self.owner_counts.items())),
            'request_counts': dict(sorted(self.request_counts.items())),
            'blocker_counts': dict(sorted(self.blocker_counts.items())),
        }


def progress_line(payload):
    """Format one compact ownership status line."""
    requests = payload.get('requests') or {}
    command = payload.get('shadow_command') or {}
    admission = payload.get('admission') or {}
    blockers = admission.get('blockers') or []
    return (
        f'state={payload.get("state", "missing")} '
        f'owner={payload.get("owner", "missing")} '
        f'req_nav={requests.get("navigation", False)} '
        f'req_align={requests.get("alignment", False)} '
        f'vx={command.get("linear_x_m_s", 0.0):+.3f} '
        f'wz={command.get("angular_z_rad_s", 0.0):+.3f} '
        f'blockers={len(blockers)}'
    )


class MotionArbiterShadowDiagnosticsNode(Node):
    """Subscribe to arbiter state and retain only the latest payload."""

    def __init__(self, topic, accumulator):
        super().__init__('motion_arbiter_shadow_diagnostics')
        self.accumulator = accumulator
        self.latest_payload = None
        self.create_subscription(
            String, topic, self._on_message, sensor_qos())

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
        description='Shadow-only motion arbiter ownership report')
    parser.add_argument(
        '--topic', default='/fire_mission/motion_arbiter_state',
        help='std_msgs/String JSON arbiter state topic')
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
    """Run subscriber-only arbiter diagnostics for a bounded duration."""
    parser = _argument_parser()
    options, ros_args = parser.parse_known_args(args)
    if options.seconds <= 0.0:
        parser.error('--seconds must be positive')
    if options.progress_period <= 0.0:
        parser.error('--progress-period must be positive')

    rclpy.init(args=ros_args)
    accumulator = MotionArbiterShadowDiagnosticsAccumulator()
    node = MotionArbiterShadowDiagnosticsNode(options.topic, accumulator)
    started = time.monotonic()
    deadline = started + options.seconds
    next_progress = started + options.progress_period
    print('=== MOTION ARBITER SHADOW DIAGNOSTICS START ===', flush=True)
    print(
        f'MODE SHADOW_ONLY subscriber_only=True topic={options.topic} '
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
                    print('WAITING no arbiter state received', flush=True)
                else:
                    print(progress_line(node.latest_payload), flush=True)
                next_progress = now + options.progress_period
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        report = accumulator.summary()
        report['generated_utc'] = datetime.now(timezone.utc).isoformat()
        report['topic'] = options.topic
        print('=== MOTION ARBITER SHADOW DIAGNOSTICS REPORT ===')
        print(json.dumps(report, indent=2, sort_keys=True))
        if options.output:
            output_path = Path(options.output).expanduser()
            output_path.write_text(
                json.dumps(report, indent=2, sort_keys=True) + '\n')
            print(f'REPORT_FILE {output_path}')
        print('=== MOTION ARBITER SHADOW DIAGNOSTICS END ===')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
