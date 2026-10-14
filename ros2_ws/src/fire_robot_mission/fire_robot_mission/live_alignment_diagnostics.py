"""Read-only report for one-shot live body alignment and observed odometry."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fire_robot_mission.mission_supervisor import sensor_qos


STATES = {
    'DISABLED', 'WAITING_FOR_ADMISSION', 'ACQUIRING',
    'TURNING', 'STOPPING', 'COMPLETE',
}


class AlignmentAccumulator:
    """Summarize live alignment without creating any command publisher."""

    def __init__(self):
        self.messages = 0
        self.invalid_json = 0
        self.contract_violations = 0
        self.state_counts = Counter()
        self.blocker_counts = Counter()
        self.stop_reason_counts = Counter()
        self.stop_alignment_input = None
        self.transitions = Counter()
        self.previous_state = None
        self.seen_complete = False
        self.max_abs_command_angular = 0.0
        self.max_abs_odom_angular_during_turn = 0.0
        self.odom_samples_during_turn = 0
        self.first_at = None
        self.last_at = None

    def add(self, payload, now):
        """Check a state frame and flag nonzero output outside TURNING."""
        if self.first_at is None:
            self.first_at = now
        self.last_at = now
        self.messages += 1
        state = payload.get('state')
        self.state_counts[str(state)] += 1
        if self.previous_state is not None and state != self.previous_state:
            self.transitions[f'{self.previous_state}->{state}'] += 1
        self.previous_state = state
        blockers = payload.get('blockers')
        if isinstance(blockers, list):
            self.blocker_counts.update(str(item) for item in blockers)
        else:
            self.contract_violations += 1
            blockers = []
        reason = payload.get('stop_reason')
        if isinstance(reason, str) and reason:
            self.stop_reason_counts[reason] += 1
            if self.stop_alignment_input is None and isinstance(
                    payload.get('stop_alignment_input'), dict):
                self.stop_alignment_input = payload['stop_alignment_input']
        command = payload.get('command')
        safety = payload.get('safety')
        if not isinstance(command, dict) or not isinstance(safety, dict):
            self.contract_violations += 1
            return
        linear = command.get('linear_x_m_s')
        angular = command.get('angular_z_rad_s')
        if isinstance(linear, bool) or isinstance(angular, bool) or \
                not isinstance(linear, (int, float)) or \
                not isinstance(angular, (int, float)) or \
                not math.isfinite(linear) or not math.isfinite(angular):
            self.contract_violations += 1
            return
        self.max_abs_command_angular = max(
            self.max_abs_command_angular, abs(angular))
        violated = state not in STATES or \
            payload.get('schema_version') != 1 or \
            payload.get('one_shot') is not True or \
            safety.get('pump_command_publisher') is not False or \
            safety.get('linear_motion') is not False or \
            safety.get('requires_operator_arm') is not True or \
            abs(linear) > 1e-9 or abs(angular) > 0.60 + 1e-6
        if state == 'TURNING':
            violated = violated or self.seen_complete or \
                payload.get('enabled') is not True or \
                payload.get('publisher_owned') is not True or \
                safety.get('fail_closed_zero') is not False or \
                blockers != []
        else:
            violated = violated or abs(angular) > 1e-9 or \
                safety.get('fail_closed_zero') is not True
        if state == 'DISABLED':
            violated = violated or payload.get('enabled') is not False or \
                payload.get('publisher_owned') is not False
        if state == 'WAITING_FOR_ADMISSION':
            violated = violated or payload.get('publisher_owned') is not False
        if state == 'COMPLETE':
            self.seen_complete = True
        if violated:
            self.contract_violations += 1

    def add_odom(self, angular_z):
        """Observe physical yaw speed only while alignment says TURNING."""
        if self.previous_state == 'TURNING' and \
                isinstance(angular_z, (int, float)) and \
                not isinstance(angular_z, bool) and math.isfinite(angular_z):
            self.odom_samples_during_turn += 1
            self.max_abs_odom_angular_during_turn = max(
                self.max_abs_odom_angular_during_turn, abs(angular_z))

    def summary(self):
        """Return a JSON-serializable report without a motor-pass claim."""
        duration = 0.0 if self.first_at is None else \
            max(0.0, self.last_at - self.first_at)
        rate = 0.0 if duration == 0.0 else \
            (self.messages - 1) / duration
        return {
            'mode': 'READ_ONLY_SUBSCRIBER',
            'subscriber_only': True,
            'messages': self.messages,
            'duration_s': round(duration, 3),
            'message_rate_hz': round(rate, 3),
            'invalid_json': self.invalid_json,
            'contract_violations': self.contract_violations,
            'state_counts': dict(sorted(self.state_counts.items())),
            'blocker_counts': dict(sorted(self.blocker_counts.items())),
            'stop_reason_counts': dict(sorted(
                self.stop_reason_counts.items())),
            'stop_alignment_input': self.stop_alignment_input,
            'transitions': dict(sorted(self.transitions.items())),
            'max_abs_command_angular_z_rad_s': round(
                self.max_abs_command_angular, 4),
            'odom_samples_during_turn': self.odom_samples_during_turn,
            'max_abs_odom_angular_z_rad_s_during_turn': round(
                self.max_abs_odom_angular_during_turn, 4),
            'physical_motion_verified': False,
        }


class LiveAlignmentDiagnostics(Node):
    """Subscribe to alignment state and odom; never publish commands."""

    def __init__(self, topic, accumulator):
        super().__init__('live_alignment_diagnostics')
        self.accumulator = accumulator
        self.create_subscription(
            String, topic, self._on_state, sensor_qos())
        self.create_subscription(
            Odometry, '/odom', self._on_odom, sensor_qos())

    def _on_state(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('alignment state must be a JSON object')
        except (TypeError, ValueError):
            self.accumulator.invalid_json += 1
            return
        self.accumulator.add(payload, time.monotonic())

    def _on_odom(self, message):
        self.accumulator.add_odom(message.twist.twist.angular.z)


def main(args=None):
    """Observe a bounded interval and optionally save a JSON report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument(
        '--topic', default='/fire_mission/live_alignment_state')
    parser.add_argument('--output')
    arguments = parser.parse_args(args)
    if arguments.seconds <= 0.0:
        parser.error('--seconds must be positive')
    accumulator = AlignmentAccumulator()
    rclpy.init()
    node = LiveAlignmentDiagnostics(arguments.topic, accumulator)
    print('=== LIVE ALIGNMENT DIAGNOSTICS START ===', flush=True)
    deadline = time.monotonic() + arguments.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    report = accumulator.summary()
    report['topic'] = arguments.topic
    report['generated_utc'] = datetime.now(timezone.utc).isoformat()
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    if arguments.output:
        output = Path(arguments.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
        print(f'REPORT_FILE {output}', flush=True)
    print('=== LIVE ALIGNMENT DIAGNOSTICS END ===', flush=True)


if __name__ == '__main__':
    main()
