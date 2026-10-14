"""Read and summarize Nav2 cancellation intent shadow telemetry."""

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


class CancelIntentAccumulator:
    """Count shadow transitions and verify the no-action contract."""

    def __init__(self):
        self.messages = 0
        self.invalid_json = 0
        self.contract_violations = 0
        self.state_counts = Counter()
        self.reason_counts = Counter()
        self.blocker_counts = Counter()
        self.transitions = Counter()
        self.previous_state = None
        self.first_time = None
        self.last_time = None

    def add(self, payload, now):
        if self.first_time is None:
            self.first_time = now
        self.last_time = now
        self.messages += 1
        state = str(payload.get('state', 'UNKNOWN'))
        self.state_counts[state] += 1
        self.reason_counts[str(payload.get('reason', 'UNKNOWN'))] += 1
        if self.previous_state is not None and self.previous_state != state:
            self.transitions[f'{self.previous_state}->{state}'] += 1
        self.previous_state = state
        blockers = payload.get('blockers')
        if isinstance(blockers, list):
            self.blocker_counts.update(str(item) for item in blockers)
        else:
            self.contract_violations += 1
        safety = payload.get('safety') or {}
        if payload.get('shadow_only') is not True or \
                safety.get('nav2_action_client') is not False or \
                safety.get('cancel_request_sent') is not False or \
                safety.get('motion_command_publisher') is not False or \
                safety.get('actuator_command_publisher') is not False or \
                payload.get('request_cancel') is not \
                (state == 'REQUEST_CANCEL') or \
                (state == 'REQUEST_CANCEL' and blockers):
            self.contract_violations += 1

    def summary(self):
        duration = 0.0 if self.first_time is None else \
            max(0.0, self.last_time - self.first_time)
        rate = 0.0 if duration == 0.0 else \
            (self.messages - 1) / duration
        return {
            'mode': 'SHADOW_ONLY',
            'subscriber_only': True,
            'messages': self.messages,
            'duration_s': round(duration, 3),
            'message_rate_hz': round(rate, 3),
            'invalid_json': self.invalid_json,
            'contract_violations': self.contract_violations,
            'state_counts': dict(sorted(self.state_counts.items())),
            'reason_counts': dict(sorted(self.reason_counts.items())),
            'blocker_counts': dict(sorted(self.blocker_counts.items())),
            'transitions': dict(sorted(self.transitions.items())),
        }


class CancelIntentDiagnostics(Node):
    """Subscribe to one isolated telemetry topic."""

    def __init__(self, topic, accumulator):
        super().__init__('nav2_cancel_intent_diagnostics')
        self.accumulator = accumulator
        self.create_subscription(String, topic, self._on_message, sensor_qos())

    def _on_message(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('payload must be an object')
        except (ValueError, TypeError):
            self.accumulator.invalid_json += 1
            return
        self.accumulator.add(payload, time.monotonic())


def main(args=None):
    """Observe a bounded window and optionally save one JSON report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--topic', default=(
        '/fire_mission/nav2_cancel_intent_shadow'))
    parser.add_argument('--output')
    arguments = parser.parse_args(args)
    if arguments.seconds <= 0.0:
        parser.error('--seconds must be positive')
    accumulator = CancelIntentAccumulator()
    rclpy.init()
    node = CancelIntentDiagnostics(arguments.topic, accumulator)
    print('=== NAV2 CANCEL INTENT SHADOW DIAGNOSTICS START ===', flush=True)
    print(f'topic={arguments.topic} duration_s={arguments.seconds}',
          flush=True)
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
    report['generated_utc'] = datetime.now(timezone.utc).isoformat()
    report['topic'] = arguments.topic
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    if arguments.output:
        output = Path(arguments.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
        print(f'REPORT_FILE {output}', flush=True)
    print('=== NAV2 CANCEL INTENT SHADOW DIAGNOSTICS END ===', flush=True)


if __name__ == '__main__':
    main()
