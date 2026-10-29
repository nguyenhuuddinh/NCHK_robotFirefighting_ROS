"""Read-only report for the opt-in Nav2 live-stop gateway."""

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


class GatewayAccumulator:
    """Check one-shot motion ownership and summarize observed telemetry."""

    def __init__(self):
        self.messages = 0
        self.invalid_json = 0
        self.contract_violations = 0
        self.state_counts = Counter()
        self.blocker_counts = Counter()
        self.transitions = Counter()
        self.previous_state = None
        self.seen_release = False
        self.max_abs_linear = 0.0
        self.max_abs_angular = 0.0
        self.first_at = None
        self.last_at = None

    def add(self, payload, now):
        """Record one JSON state and flag impossible or unsafe outputs."""
        if self.first_at is None:
            self.first_at = now
        self.last_at = now
        self.messages += 1
        state = payload.get('state')
        self.state_counts[str(state)] += 1
        if self.previous_state is not None and state != self.previous_state:
            self.transitions[
                f'{self.previous_state}->{state}'] += 1
        self.previous_state = state
        blockers = payload.get('blockers')
        if isinstance(blockers, list):
            self.blocker_counts.update(str(item) for item in blockers)
        else:
            self.contract_violations += 1
        command = payload.get('command')
        safety = payload.get('safety')
        if not isinstance(command, dict) or not isinstance(safety, dict):
            self.contract_violations += 1
            return
        linear = command.get('linear_x_m_s')
        angular = command.get('angular_z_rad_s')
        if not isinstance(linear, (int, float)) or \
                not isinstance(angular, (int, float)) or \
                not math.isfinite(linear) or not math.isfinite(angular):
            self.contract_violations += 1
            return
        self.max_abs_linear = max(self.max_abs_linear, abs(linear))
        self.max_abs_angular = max(self.max_abs_angular, abs(angular))
        violated = payload.get('schema_version') != 1 or \
            payload.get('one_shot') is not True or \
            safety.get('actuator_command_publisher') is not False or \
            safety.get('alignment_command_publisher') is not False or \
            abs(linear) > 0.10 + 1e-6 or abs(angular) > 0.8 + 1e-6
        if state == 'NAVIGATION':
            violated = violated or self.seen_release or \
                payload.get('enabled') is not True or \
                payload.get('fire_latched') is not False or \
                payload.get('arm_fresh') is not True or \
                payload.get('nav_shadow_fingerprint_ok') is not True or \
                payload.get('raw_owner_fingerprint_ok') is not True or \
                payload.get('safety_gate_fingerprint_ok') is not True or \
                payload.get('raw_publishers') != 1 or \
                payload.get('mission_publishers') != 1 or \
                payload.get('odom_publishers') != 1 or \
                payload.get('live_arm_publishers') != 1 or \
                payload.get('nav_status_publishers') != 1 or \
                blockers != []
        elif abs(linear) > 1e-9 or abs(angular) > 1e-9:
            violated = True
        if state == 'HANDOFF_READY':
            violated = violated or \
                payload.get('handoff_ready') is not True or \
                payload.get('released') is not True or \
                payload.get('raw_publishers') != 0 or \
                payload.get('odom_stationary') is not True or \
                payload.get('cmd_vel_fresh_zero') is not True or \
                payload.get('cancel_guard_publishers') != 1
        if state in {'RELEASED_STOPPED', 'HANDOFF_READY'}:
            self.seen_release = True
        if violated:
            self.contract_violations += 1

    def summary(self):
        """Return compact, JSON-serializable diagnostics."""
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
            'transitions': dict(sorted(self.transitions.items())),
            'max_abs_linear_x_m_s': round(self.max_abs_linear, 4),
            'max_abs_angular_z_rad_s': round(self.max_abs_angular, 4),
        }


class GatewayDiagnostics(Node):
    """Subscribe to gateway state without owning any robot command topic."""

    def __init__(self, topic, accumulator):
        super().__init__('live_nav_stop_diagnostics')
        self.accumulator = accumulator
        self.create_subscription(String, topic, self._on_message, sensor_qos())

    def _on_message(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('gateway state must be a JSON object')
        except (TypeError, ValueError):
            self.accumulator.invalid_json += 1
            return
        self.accumulator.add(payload, time.monotonic())


def main(args=None):
    """Observe a bounded interval and optionally save a JSON report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument(
        '--topic', default='/fire_mission/live_nav_stop_state')
    parser.add_argument('--output')
    arguments = parser.parse_args(args)
    if arguments.seconds <= 0.0:
        parser.error('--seconds must be positive')
    accumulator = GatewayAccumulator()
    rclpy.init()
    node = GatewayDiagnostics(arguments.topic, accumulator)
    print('=== LIVE NAV STOP DIAGNOSTICS START ===', flush=True)
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
    print('=== LIVE NAV STOP DIAGNOSTICS END ===', flush=True)


if __name__ == '__main__':
    main()
