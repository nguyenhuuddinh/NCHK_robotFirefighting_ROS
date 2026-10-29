"""Summarize alignment handoff shadow state without publishing anything."""

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


class HandoffShadowDiagnosticsAccumulator:
    """Accumulate shadow admission states and safety-contract violations."""

    def __init__(self, max_angular_speed_rad_s=0.35):
        if max_angular_speed_rad_s <= 0.0:
            raise ValueError('max_angular_speed_rad_s must be positive')
        self.max_angular = float(max_angular_speed_rad_s)
        self.messages = 0
        self.invalid_json = 0
        self.shadow_contract_violations = 0
        self.nonzero_while_inactive_violations = 0
        self.linear_motion_violations = 0
        self.angular_limit_violations = 0
        self.direction_sign_violations = 0
        self.active_alignment_state_violations = 0
        self.state_counts = Counter()
        self.alignment_state_counts = Counter()
        self.active_direction_counts = Counter()
        self.active_direction_transitions = Counter()
        self.previous_active_direction = None
        self.blocker_counts = Counter()
        self.active_counts = Counter()
        self.arm_fresh_counts = Counter()
        self.clamped_frames = 0
        self.max_abs_angular = 0.0
        self.first_receive_time = None
        self.last_receive_time = None

    @staticmethod
    def _label(value):
        if value is True:
            return 'true'
        if value is False:
            return 'false'
        return 'missing'

    def add_invalid_json(self):
        """Count one malformed state message."""
        self.invalid_json += 1

    def add(self, payload, receive_time=None):
        """Consume one decoded handoff shadow state."""
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
                safety.get('requires_arm_heartbeat') is not True:
            self.shadow_contract_violations += 1

        state = str(payload.get('state', 'missing'))
        active = payload.get('active') is True
        self.state_counts[state] += 1
        self.active_counts['active' if active else 'inactive'] += 1
        arm = payload.get('operator_arm') or {}
        self.arm_fresh_counts[self._label(arm.get('fresh'))] += 1

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
                not math.isfinite(float(linear)) or abs(float(linear)) > 1e-9:
            self.linear_motion_violations += 1
        if not isinstance(angular, (int, float)) or \
                not math.isfinite(float(angular)):
            self.angular_limit_violations += 1
            angular = 0.0
        angular = float(angular)
        self.max_abs_angular = max(self.max_abs_angular, abs(angular))
        if abs(angular) > self.max_angular + 1e-9:
            self.angular_limit_violations += 1
        if not active and abs(angular) > 1e-9:
            self.nonzero_while_inactive_violations += 1
        alignment = payload.get('alignment') or {}
        if alignment.get('clamped') is True:
            self.clamped_frames += 1
        alignment_state = str(alignment.get('state', 'missing'))
        self.alignment_state_counts[alignment_state] += 1
        if active:
            direction = {
                'ALIGNING_LEFT': 'LEFT',
                'ALIGNING_RIGHT': 'RIGHT',
                'CENTERED': 'CENTERED',
            }.get(alignment_state)
            if direction is None:
                self.active_alignment_state_violations += 1
                self.previous_active_direction = None
            else:
                self.active_direction_counts[direction] += 1
                if self.previous_active_direction is not None and \
                        self.previous_active_direction != direction:
                    transition = (
                        f'{self.previous_active_direction}->{direction}')
                    self.active_direction_transitions[transition] += 1
                self.previous_active_direction = direction
                expected_state = (
                    'ARMED_SHADOW_CENTERED' if direction == 'CENTERED'
                    else 'ARMED_SHADOW_TRACKING')
                if state != expected_state:
                    self.active_alignment_state_violations += 1
                if (direction == 'LEFT' and angular <= 0.0) or \
                        (direction == 'RIGHT' and angular >= 0.0) or \
                        (direction == 'CENTERED' and abs(angular) > 1e-9):
                    self.direction_sign_violations += 1
        else:
            self.previous_active_direction = None

    def summary(self):
        """Return the final JSON-safe report."""
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
            'nonzero_while_inactive_violations': (
                self.nonzero_while_inactive_violations),
            'linear_motion_violations': self.linear_motion_violations,
            'angular_limit_violations': self.angular_limit_violations,
            'direction_sign_violations': self.direction_sign_violations,
            'active_alignment_state_violations': (
                self.active_alignment_state_violations),
            'alignment_state_counts': dict(sorted(
                self.alignment_state_counts.items())),
            'active_direction_counts': dict(sorted(
                self.active_direction_counts.items())),
            'active_direction_transitions': dict(sorted(
                self.active_direction_transitions.items())),
            'directional_coverage_complete': all(
                self.active_direction_counts[direction] > 0
                for direction in ('LEFT', 'RIGHT', 'CENTERED')),
            'max_abs_shadow_angular_z_rad_s': round(
                self.max_abs_angular, 4),
            'clamped_frames': self.clamped_frames,
            'state_counts': dict(sorted(self.state_counts.items())),
            'active_counts': dict(sorted(self.active_counts.items())),
            'arm_fresh_counts': dict(sorted(
                self.arm_fresh_counts.items())),
            'blocker_counts': dict(sorted(self.blocker_counts.items())),
        }


def progress_line(payload):
    """Format one compact handoff shadow status line."""
    admission = payload.get('admission') or {}
    arm = payload.get('operator_arm') or {}
    alignment = payload.get('alignment') or {}
    command = payload.get('shadow_command') or {}
    blockers = admission.get('blockers') or []
    return (
        f'state={payload.get("state", "missing")} '
        f'active={payload.get("active", False)} '
        f'arm={arm.get("requested", False)}/{arm.get("fresh", False)} '
        f'align={alignment.get("state", "missing")} '
        f'shadow_wz={command.get("angular_z_rad_s", 0.0):+.3f} '
        f'clamped={alignment.get("clamped", False)} '
        f'blockers={len(blockers)}'
    )


class HandoffShadowDiagnosticsNode(Node):
    """Subscribe to handoff state and retain the latest payload."""

    def __init__(self, topic, accumulator):
        super().__init__('handoff_shadow_diagnostics')
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
        description='Shadow-only fire alignment handoff report')
    parser.add_argument(
        '--topic', default='/fire_mission/alignment_handoff_state',
        help='std_msgs/String JSON handoff state topic')
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
    """Run subscriber-only handoff diagnostics for a bounded duration."""
    parser = _argument_parser()
    options, ros_args = parser.parse_known_args(args)
    if options.seconds <= 0.0:
        parser.error('--seconds must be positive')
    if options.progress_period <= 0.0:
        parser.error('--progress-period must be positive')

    rclpy.init(args=ros_args)
    accumulator = HandoffShadowDiagnosticsAccumulator()
    node = HandoffShadowDiagnosticsNode(options.topic, accumulator)
    started = time.monotonic()
    deadline = started + options.seconds
    next_progress = started + options.progress_period
    print('=== ALIGNMENT HANDOFF SHADOW DIAGNOSTICS START ===', flush=True)
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
                    print('WAITING no handoff state received', flush=True)
                else:
                    print(progress_line(node.latest_payload), flush=True)
                next_progress = now + options.progress_period
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        report = accumulator.summary()
        report['generated_utc'] = datetime.now(timezone.utc).isoformat()
        report['topic'] = options.topic
        print('=== ALIGNMENT HANDOFF SHADOW DIAGNOSTICS REPORT ===')
        print(json.dumps(report, indent=2, sort_keys=True))
        if options.output:
            output_path = Path(options.output).expanduser()
            output_path.write_text(
                json.dumps(report, indent=2, sort_keys=True) + '\n')
            print(f'REPORT_FILE {output_path}')
        print('=== ALIGNMENT HANDOFF SHADOW DIAGNOSTICS END ===')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
