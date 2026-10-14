"""
Read-only, cross-topic evidence for Nav2-to-fire handoff trials.

This report is telemetry evidence, not authorization for real motion. It
creates subscriptions only; it never publishes commands or calls CancelGoal.
"""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fire_robot_mission.mission_supervisor import sensor_qos


TOPICS = {
    'mission': '/fire_mission/state',
    'gateway': '/fire_mission/live_nav_stop_state',
    'guard': '/fire_mission/nav2_cancel_guard_state',
    'handoff': '/fire_mission/alignment_handoff_state',
    'arbiter': '/fire_mission/motion_arbiter_state',
}
STAGES = (
    'NAVIGATION_OBSERVED',
    'FIRE_CONFIRMED',
    'FIRE_STOPPING',
    'RAW_RELEASED',
    'GOAL_CANCEL_SENT',
    'GOAL_CANCEL_CONFIRMED',
    'GATEWAY_HANDOFF_READY',
    'HANDOFF_SHADOW_ACTIVE',
    'ARBITER_SHADOW_ALIGNMENT',
)
UUID_PATTERN = re.compile(r'^[0-9a-fA-F]{32}$')
ORDER_TOLERANCE_S = 0.25


def _object(value):
    return value if isinstance(value, dict) else {}


def _valid_uuid(value):
    return isinstance(value, str) and \
        UUID_PATTERN.fullmatch(value) is not None and int(value, 16) != 0


class HandoffSequenceAccumulator:
    """Correlate five state streams by local reception time, without writes."""

    def __init__(self):
        self.messages = Counter()
        self.invalid_json = Counter()
        self.violations = Counter()
        self.events = {}
        self.first_at = None
        self.last_at = None
        self.last_state = {}
        self.guard_goal_uuid = None
        self.handoff_goal_uuid = None
        self.release_seen = False

    def add_invalid_json(self, source):
        """Count malformed JSON without converting it into evidence."""
        self.invalid_json[source] += 1

    def _record(self, stage, now):
        if stage not in self.events:
            self.events[stage] = now

    def add(self, source, payload, now):
        """Consume one decoded state from a named, read-only topic."""
        if source not in TOPICS:
            raise ValueError(f'unknown source: {source}')
        now = float(now)
        if self.first_at is None:
            self.first_at = now
        self.last_at = now
        self.messages[source] += 1
        if payload.get('schema_version') != 1:
            self.violations[f'{source.upper()}_SCHEMA_INVALID'] += 1
            return
        state = payload.get('state')
        self.last_state[source] = state
        safety = _object(payload.get('safety'))

        if source == 'mission':
            perception = _object(payload.get('perception'))
            if perception.get('confirmed') is True and state in {
                    'FIRE_TRACKING', 'CENTERED_HOLD', 'ALIGNMENT_READY'}:
                if payload.get('observe_only') is True and \
                        safety.get('motion_command_publisher') is False:
                    self._record('FIRE_CONFIRMED', now)
                else:
                    self.violations['MISSION_FIRE_CONTRACT_INVALID'] += 1

        elif source == 'gateway':
            command = _object(payload.get('command'))
            command_zero = (command.get('linear_x_m_s') == 0.0 and
                            command.get('angular_z_rad_s') == 0.0)
            if state == 'NAVIGATION':
                if self.release_seen:
                    self.violations['NAVIGATION_AFTER_RELEASE'] += 1
                if payload.get('enabled') is True and \
                        payload.get('raw_publishers') == 1 and \
                        safety.get('raw_command_publisher') is True:
                    self._record('NAVIGATION_OBSERVED', now)
                else:
                    self.violations['GATEWAY_NAV_CONTRACT_INVALID'] += 1
            if state == 'STOPPING' and payload.get('fire_latched') is True:
                if payload.get('stop_reason') == 'FIRE_ACQUIRED' and \
                        command_zero:
                    self._record('FIRE_STOPPING', now)
                else:
                    self.violations['FIRE_STOP_CONTRACT_INVALID'] += 1
            if state in {'RELEASED_STOPPED', 'HANDOFF_READY'} and \
                    payload.get('released') is True:
                self.release_seen = True
                if payload.get('raw_publishers') == 0 and command_zero and \
                        payload.get('odom_stationary') is True and \
                        payload.get('cmd_vel_fresh_zero') is True and \
                        safety.get('raw_command_publisher') is False:
                    self._record('RAW_RELEASED', now)
                else:
                    self.violations['RAW_RELEASE_CONTRACT_INVALID'] += 1
            if state == 'HANDOFF_READY':
                if payload.get('handoff_ready') is True and \
                        payload.get('released') is True and \
                        payload.get('raw_publishers') == 0 and \
                        payload.get('goal_active') is False and \
                        payload.get('cancel_guard_publishers') == 1 and \
                        command_zero:
                    self._record('GATEWAY_HANDOFF_READY', now)
                else:
                    self.violations['GATEWAY_HANDOFF_CONTRACT_INVALID'] += 1

        elif source == 'guard':
            uuid = payload.get('goal_uuid')
            if payload.get('send_count') == 1 and _valid_uuid(uuid):
                if self.guard_goal_uuid is not None and \
                        self.guard_goal_uuid != uuid:
                    self.violations['MULTIPLE_GOAL_UUIDS'] += 1
                self.guard_goal_uuid = uuid
                self._record('GOAL_CANCEL_SENT', now)
            if state == 'CANCEL_CONFIRMED_STOPPED':
                if payload.get('enabled') is True and \
                        payload.get('send_count') == 1 and \
                        _valid_uuid(uuid) and \
                        payload.get('cancel_service_accepted') is True and \
                        payload.get('cancel_status_seen') is True and \
                        payload.get('goal_status') == 5 and \
                        payload.get('cmd_vel_fresh_zero') is True and \
                        payload.get('odom_fresh_stationary') is True and \
                        payload.get('raw_cmd_publishers') == 0 and \
                        payload.get('blockers') == [] and \
                        safety.get('motion_command_publisher') is False:
                    self._record('GOAL_CANCEL_CONFIRMED', now)
                else:
                    self.violations['GUARD_CONFIRM_CONTRACT_INVALID'] += 1

        elif source == 'handoff' and payload.get('active') is True:
            proof = _object(payload.get('live_cancel_proof'))
            admission = _object(payload.get('admission'))
            uuid = proof.get('goal_uuid')
            if payload.get('shadow_only') is True and \
                    safety.get('real_motion_output') is False and \
                    proof.get('required') is True and \
                    proof.get('gateway_publishers') == 1 and \
                    proof.get('cancel_guard_publishers') == 1 and \
                    proof.get('gateway_state') == 'HANDOFF_READY' and \
                    proof.get('cancel_guard_state') == \
                    'CANCEL_CONFIRMED_STOPPED' and \
                    admission.get('blockers') == [] and _valid_uuid(uuid):
                self.handoff_goal_uuid = uuid
                self._record('HANDOFF_SHADOW_ACTIVE', now)
            else:
                self.violations['HANDOFF_PROOF_INVALID'] += 1

        elif source == 'arbiter' and state == 'ALIGNMENT':
            requests = _object(payload.get('requests'))
            handoff = _object(payload.get('handoff'))
            if payload.get('shadow_only') is True and \
                    payload.get('owner') == 'ALIGNMENT' and \
                    safety.get('real_motion_output') is False and \
                    safety.get('exclusive_owner') is True and \
                    requests.get('navigation') is False and \
                    requests.get('alignment') is True and \
                    requests.get('conflict') is False and \
                    handoff.get('active') is True:
                self._record('ARBITER_SHADOW_ALIGNMENT', now)
            else:
                self.violations['ARBITER_OWNERSHIP_INVALID'] += 1

    def summary(self):
        """Return a fail-closed, JSON-safe report for this observation run."""
        missing = [stage for stage in STAGES if stage not in self.events]
        order_violations = []
        for earlier, later in zip(STAGES, STAGES[1:]):
            if earlier in self.events and later in self.events and \
                    self.events[earlier] > self.events[later] + \
                    ORDER_TOLERANCE_S:
                order_violations.append(f'{later}_BEFORE_{earlier}')
        if self.handoff_goal_uuid is not None and \
                self.guard_goal_uuid != self.handoff_goal_uuid:
            order_violations.append('HANDOFF_GOAL_UUID_MISMATCH')
        if self.violations or order_violations or any(
                self.invalid_json.values()):
            result = 'FAIL'
        elif missing:
            result = 'INCOMPLETE'
        else:
            result = 'PASS_TELEMETRY_ONLY'
        origin = self.first_at
        duration = 0.0 if origin is None else self.last_at - origin
        return {
            'mode': 'READ_ONLY_SUBSCRIBER',
            'subscriber_only': True,
            'status': result,
            'note': 'Telemetry sequence only; not proof of physical stop',
            'duration_s': round(duration, 3),
            'message_counts': dict(sorted(self.messages.items())),
            'invalid_json_counts': dict(sorted(self.invalid_json.items())),
            'contract_violations': dict(sorted(self.violations.items())),
            'order_violations': order_violations,
            'missing_stages': missing,
            'stage_first_seen_s': {
                stage: round(self.events[stage] - origin, 3)
                for stage in STAGES if stage in self.events
            },
            'last_states': dict(sorted(self.last_state.items())),
            'guard_goal_uuid': self.guard_goal_uuid,
            'handoff_goal_uuid': self.handoff_goal_uuid,
        }


class HandoffSequenceDiagnostics(Node):
    """Subscribe to the five existing JSON telemetry topics only."""

    def __init__(self, accumulator):
        super().__init__('handoff_sequence_diagnostics')
        self.accumulator = accumulator
        for source, topic in TOPICS.items():
            self.create_subscription(
                String, topic,
                lambda message, source=source: self._on_message(
                    source, message),
                sensor_qos())

    def _on_message(self, source, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('JSON root is not an object')
        except (TypeError, ValueError):
            self.accumulator.add_invalid_json(source)
            return
        self.accumulator.add(source, payload, time.monotonic())


def main(args=None):
    """Capture a bounded, read-only Nav2-to-fire handoff sequence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=45.0)
    parser.add_argument('--progress-period', type=float, default=1.0)
    parser.add_argument('--output')
    options, ros_args = parser.parse_known_args(args)
    if options.seconds <= 0.0 or options.progress_period <= 0.0:
        parser.error('--seconds and --progress-period must be positive')
    accumulator = HandoffSequenceAccumulator()
    rclpy.init(args=ros_args)
    node = HandoffSequenceDiagnostics(accumulator)
    started = time.monotonic()
    deadline = started + options.seconds
    next_progress = started + options.progress_period
    print('=== HANDOFF SEQUENCE DIAGNOSTICS START ===', flush=True)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if now >= next_progress:
                seen = [stage for stage in STAGES
                        if stage in accumulator.events]
                print('seen=' + ','.join(seen), flush=True)
                next_progress = now + options.progress_period
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        report = accumulator.summary()
        report['topics'] = TOPICS
        report['generated_utc'] = datetime.now(timezone.utc).isoformat()
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        if options.output:
            output = Path(options.output).expanduser()
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, indent=2,
                                         sort_keys=True) + '\n')
            print(f'REPORT_FILE {output}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print('=== HANDOFF SEQUENCE DIAGNOSTICS END ===', flush=True)


if __name__ == '__main__':
    main()
