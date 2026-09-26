"""Summarize observe-only mission state without publishing anything."""

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


class MissionDiagnosticsAccumulator:
    """Accumulate mission states and enforce the observe-only contract."""

    def __init__(self):
        self.messages = 0
        self.invalid_json = 0
        self.observe_only_violations = 0
        self.safety_contract_violations = 0
        self.command_published_violations = 0
        self.safe_stop_intent_violations = 0
        self.state_counts = Counter()
        self.intent_counts = Counter()
        self.turn_comparison_counts = Counter()
        self.health_counts = Counter()
        self.blocker_counts = Counter()
        self.readiness_blocker_counts = Counter()
        self.eligibility_counts = Counter()
        self.odom_fresh_counts = Counter()
        self.stationary_counts = Counter()
        self.transitions = Counter()
        self.eligible_frames = 0
        self._previous_state = None
        self.first_receive_time = None
        self.last_receive_time = None

    def add_invalid_json(self):
        """Count one malformed state message."""
        self.invalid_json += 1

    def add(self, payload, receive_time=None):
        """Consume one decoded mission-state payload."""
        receive_time = (
            time.monotonic() if receive_time is None else receive_time)
        if self.first_receive_time is None:
            self.first_receive_time = receive_time
        self.last_receive_time = receive_time
        self.messages += 1
        if payload.get('observe_only') is not True:
            self.observe_only_violations += 1
        safety = payload.get('safety') or {}
        if safety.get('motion_command_publisher') is not False or \
                safety.get('actuator_command_publisher') is not False or \
                safety.get('nav2_action_client') is not False:
            self.safety_contract_violations += 1
        decision = payload.get('decision') or {}
        if decision.get('command_published') is not False:
            self.command_published_violations += 1

        state = str(payload.get('state', 'missing'))
        intent = str(decision.get('intent', 'missing'))
        comparison = str(
            (payload.get('navigation') or {}).get(
                'fire_nav_turn_comparison', 'missing'))
        health = payload.get('health') or {}
        health_status = str(health.get('status', 'missing'))
        self.state_counts[state] += 1
        self.intent_counts[intent] += 1
        self.turn_comparison_counts[comparison] += 1
        self.health_counts[health_status] += 1
        self.blocker_counts.update(
            str(item) for item in health.get('blockers', []))
        readiness_blockers = decision.get('readiness_blockers', [])
        if isinstance(readiness_blockers, list):
            self.readiness_blocker_counts.update(
                str(item) for item in readiness_blockers)
        else:
            self.readiness_blocker_counts['INVALID_READINESS_BLOCKERS'] += 1
        eligible = (
            decision.get('eligible_for_future_alignment_handoff') is True)
        self.eligibility_counts['eligible' if eligible else 'ineligible'] += 1
        if eligible:
            self.eligible_frames += 1
        robot = payload.get('robot') or {}
        self.odom_fresh_counts[
            self._bool_label(robot.get('odom_fresh'))] += 1
        self.stationary_counts[
            self._bool_label(robot.get('stationary'))] += 1
        if state in ('TARGET_LOST', 'FAULT') and \
                intent != 'REQUEST_SAFE_STOP':
            self.safe_stop_intent_violations += 1
        if self._previous_state is not None and state != self._previous_state:
            self.transitions[f'{self._previous_state}->{state}'] += 1
        self._previous_state = state

    @staticmethod
    def _bool_label(value):
        """Return a stable report key for true, false, or absent values."""
        if value is True:
            return 'true'
        if value is False:
            return 'false'
        return 'missing'

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
            'mode': 'OBSERVE_ONLY',
            'subscriber_only': True,
            'messages': self.messages,
            'duration_s': round(duration, 3),
            'message_rate_hz': round(rate, 3),
            'invalid_json': self.invalid_json,
            'observe_only_violations': self.observe_only_violations,
            'safety_contract_violations': (
                self.safety_contract_violations),
            'command_published_violations': (
                self.command_published_violations),
            'safe_stop_intent_violations': (
                self.safe_stop_intent_violations),
            'eligible_frames': self.eligible_frames,
            'eligibility_counts': dict(sorted(
                self.eligibility_counts.items())),
            'readiness_blocker_counts': dict(sorted(
                self.readiness_blocker_counts.items())),
            'odom_fresh_counts': dict(sorted(
                self.odom_fresh_counts.items())),
            'stationary_counts': dict(sorted(
                self.stationary_counts.items())),
            'state_counts': dict(sorted(self.state_counts.items())),
            'intent_counts': dict(sorted(self.intent_counts.items())),
            'turn_comparison_counts': dict(
                sorted(self.turn_comparison_counts.items())),
            'health_counts': dict(sorted(self.health_counts.items())),
            'blocker_counts': dict(sorted(self.blocker_counts.items())),
            'transitions': dict(sorted(self.transitions.items())),
        }


def progress_line(payload):
    """Format one compact state-machine status line."""
    perception = payload.get('perception') or {}
    navigation = payload.get('navigation') or {}
    decision = payload.get('decision') or {}
    health = payload.get('health') or {}
    robot = payload.get('robot') or {}
    readiness_blockers = decision.get('readiness_blockers') or []
    return (
        f'state={payload.get("state", "missing")} '
        f'intent={decision.get("intent", "missing")} '
        f'fire={perception.get("detected_now", False)}/'
        f'{perception.get("confirmed", False)} '
        f'align={perception.get("alignment_state", "missing")} '
        f'nav={navigation.get("goal_state", "missing")} '
        f'compare={navigation.get("fire_nav_turn_comparison", "missing")} '
        f'eligible={decision.get("eligible_for_future_alignment_handoff", False)} '
        f'odom={robot.get("odom_fresh", "missing")}/'
        f'{robot.get("stationary", "missing")} '
        f'ready_blockers={len(readiness_blockers)} '
        f'health={health.get("status", "missing")} '
        f'blockers={len(health.get("blockers", []))}'
    )


class MissionDiagnosticsNode(Node):
    """Subscribe to mission state and retain the latest payload."""

    def __init__(self, topic, accumulator):
        super().__init__('mission_preview_diagnostics')
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
        description='Observe-only fire mission supervisor report')
    parser.add_argument(
        '--topic', default='/fire_mission/state',
        help='std_msgs/String JSON mission-state topic')
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
    accumulator = MissionDiagnosticsAccumulator()
    node = MissionDiagnosticsNode(options.topic, accumulator)
    started = time.monotonic()
    deadline = started + options.seconds
    next_progress = started + options.progress_period
    print('=== FIRE MISSION DIAGNOSTICS START ===', flush=True)
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
                    print('WAITING no mission state received', flush=True)
                else:
                    print(progress_line(node.latest_payload), flush=True)
                next_progress = now + options.progress_period
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        report = accumulator.summary()
        report['generated_utc'] = datetime.now(timezone.utc).isoformat()
        report['topic'] = options.topic
        print('=== FIRE MISSION DIAGNOSTICS REPORT ===')
        print(json.dumps(report, indent=2, sort_keys=True))
        if options.output:
            output_path = Path(options.output).expanduser()
            output_path.write_text(
                json.dumps(report, indent=2, sort_keys=True) + '\n')
            print(f'REPORT_FILE {output_path}')
        print('=== FIRE MISSION DIAGNOSTICS END ===')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
