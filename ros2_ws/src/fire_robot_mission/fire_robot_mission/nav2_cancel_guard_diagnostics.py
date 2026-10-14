"""Read-only report for the one-shot Nav2 cancel guard."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fire_robot_mission.mission_supervisor import sensor_qos


class GuardAccumulator:
    """Summarize guard state without publishing any ROS command."""

    def __init__(self):
        self.messages = 0
        self.invalid_json = 0
        self.contract_violations = 0
        self.states = Counter()
        self.blockers = Counter()
        self.uuids = set()
        self.max_send_count = 0
        self.previous_state = None
        self.transitions = Counter()
        self.first_at = None
        self.last_at = None

    def add(self, payload, now):
        if self.first_at is None:
            self.first_at = now
        self.last_at = now
        self.messages += 1
        state = str(payload.get('state', 'UNKNOWN'))
        self.states[state] += 1
        if self.previous_state is not None and self.previous_state != state:
            self.transitions[f'{self.previous_state}->{state}'] += 1
        self.previous_state = state
        blockers = payload.get('blockers')
        if isinstance(blockers, list):
            self.blockers.update(str(value) for value in blockers)
        else:
            self.contract_violations += 1
        uuid = payload.get('goal_uuid')
        if isinstance(uuid, str):
            self.uuids.add(uuid)
        send_count = payload.get('send_count')
        if isinstance(send_count, int):
            self.max_send_count = max(self.max_send_count, send_count)
        else:
            self.contract_violations += 1
        safety = payload.get('safety')
        if not isinstance(safety, dict) or \
                safety.get('motion_command_publisher') is not False or \
                safety.get('actuator_command_publisher') is not False or \
                safety.get('requires_operator_arm') is not True or \
                safety.get('requires_shadow_nav2') is not True or \
                self.max_send_count > 1 or \
                (state in {'REQUEST_SENT', 'WAITING_FOR_CANCEL',
                           'CANCEL_CONFIRMED_STOPPED'} and (
                    payload.get('nav_shadow_endpoints') != 3 or
                    payload.get('nav_shadow_fingerprint_ok') is not True or
                    payload.get('raw_cmd_publishers') != 0)) or \
                (state == 'CANCEL_CONFIRMED_STOPPED' and (
                    payload.get('cancel_service_accepted') is not True or
                    payload.get('cancel_status_seen') is not True or
                    payload.get('goal_status') not in (5, None) or
                    payload.get('arm_fresh') is not True or
                    payload.get('cmd_vel_fresh_zero') is not True or
                    payload.get('odom_fresh_stationary') is not True)):
            self.contract_violations += 1

    def summary(self):
        duration = 0.0 if self.first_at is None else \
            max(0.0, self.last_at - self.first_at)
        rate = 0.0 if duration == 0.0 else \
            (self.messages - 1) / duration
        return {
            'mode': 'OPT_IN_CANCEL_GUARD',
            'subscriber_only': True,
            'messages': self.messages,
            'duration_s': round(duration, 3),
            'message_rate_hz': round(rate, 3),
            'invalid_json': self.invalid_json,
            'contract_violations': self.contract_violations,
            'state_counts': dict(sorted(self.states.items())),
            'blocker_counts': dict(sorted(self.blockers.items())),
            'transitions': dict(sorted(self.transitions.items())),
            'goal_uuids': sorted(self.uuids),
            'max_send_count': self.max_send_count,
        }


class GuardDiagnostics(Node):
    """Subscribe only to guard telemetry."""

    def __init__(self, topic, accumulator):
        super().__init__('nav2_cancel_guard_diagnostics')
        self.accumulator = accumulator
        self.create_subscription(String, topic, self._on_message, sensor_qos())

    def _on_message(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('guard payload must be a JSON object')
        except (TypeError, ValueError):
            self.accumulator.invalid_json += 1
            return
        self.accumulator.add(payload, time.monotonic())


def main(args=None):
    """Observe a bounded interval and optionally save a JSON report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument(
        '--topic', default='/fire_mission/nav2_cancel_guard_state')
    parser.add_argument('--output')
    arguments = parser.parse_args(args)
    if arguments.seconds <= 0.0:
        parser.error('--seconds must be positive')
    accumulator = GuardAccumulator()
    rclpy.init()
    node = GuardDiagnostics(arguments.topic, accumulator)
    print('=== NAV2 CANCEL GUARD DIAGNOSTICS START ===', flush=True)
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
    print('=== NAV2 CANCEL GUARD DIAGNOSTICS END ===', flush=True)


if __name__ == '__main__':
    main()
