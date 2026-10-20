#!/usr/bin/env python3
"""Opt-in bounded M3 runtime observer; no goals, arms or command publishers."""

import argparse
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatusArray
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

from fire_robot_mission.mission_supervisor import sensor_qos, action_status_qos
from fire_robot_mission.stop_metadata_executor import StopMetadataExecutor


class Observer(Node):
    def __init__(self):
        super().__init__('m3_bounded_runtime_observer')
        self.start = time.monotonic()
        self.errors = set()
        self.transitions = []
        self.sessions = {}
        self.last_phase = None
        self.instance = self.sequence = None
        self.closed = 0
        self.latest = None
        self.stats = {role: {'count': 0, 'last_mono': None, 'last_stamp': None,
                            'max_gap_ms': 0.0, 'max_age_ms': 0.0, 'last_age_ms': 0.0}
                      for role in ('mission', 'pose', 'through')}
        self.graph_samples = 0
        self.gids = {}
        self.topics = {'mission': '/fire_mission/state',
                       'pose': '/navigate_to_pose/_action/status',
                       'through': '/navigate_through_poses/_action/status'}
        for role, topic in self.topics.items():
            callback = lambda msg, metadata, role=role: self.observe(role, msg, metadata)
            sub = self.create_subscription(String if role == 'mission' else GoalStatusArray,
                topic, lambda msg: None, sensor_qos() if role == 'mission' else action_status_qos())
            sub._fire_stop_metadata_callback = callback
        self.create_timer(1.0, self.graph)

    def error(self, reason):
        if len(self.errors) < 32:
            self.errors.add(reason)

    def observe(self, role, message, metadata):
        now = time.monotonic()
        stamp = metadata.get('source_timestamp') if isinstance(metadata, dict) else None
        if type(stamp) is not int or stamp <= 0:
            self.error('DDS_STAMP_MISSING:' + role)
            return
        age_ms = (self.get_clock().now().nanoseconds - stamp) / 1e6
        stats = self.stats[role]
        if stats['last_stamp'] is not None and stamp <= stats['last_stamp']:
            self.error('DDS_STAMP_NON_INCREASING:' + role)
        if stats['last_mono'] is not None:
            stats['max_gap_ms'] = max(stats['max_gap_ms'], (now - stats['last_mono']) * 1000)
        stats.update(count=stats['count'] + 1, last_mono=now, last_stamp=stamp,
                     max_age_ms=max(stats['max_age_ms'], age_ms), last_age_ms=max(0.0, age_ms))
        if age_ms < -20:
            self.error('DDS_CLOCK_IN_FUTURE:' + role)
        if role != 'mission':
            return
        try:
            payload = json.loads(message.data)
            observer = payload['lifecycle']['fire_session_observer']
            runtime = payload['lifecycle']['shadow_session_runtime']
            source = observer['source']
            seq, instance = source['sequence'], source['instance_id']
            if payload['observe_only'] is not True or observer['actuation_authorized'] is not False:
                self.error('MISSION_AUTHORITY_NOT_SHADOW')
            if runtime['actuation_authorized'] is not False or runtime['budget']['planning_only'] is not True:
                self.error('RUNTIME_AUTHORITY_NOT_SHADOW')
            if self.instance is not None and instance != self.instance:
                self.error('SUPERVISOR_RESTART_DURING_ACCEPTANCE')
            if type(seq) is not int or (self.sequence is not None and seq <= self.sequence):
                self.error('SESSION_SOURCE_SEQUENCE_INVALID')
            self.instance, self.sequence = instance, seq
            phase, identity = observer['phase'], observer['session_id']
            key = (phase, identity, observer['close_generation'])
            if key != self.last_phase:
                if len(self.transitions) >= 64:
                    self.error('TRANSITION_CAPTURE_LIMIT')
                else:
                    self.transitions.append({'elapsed_s': round(now - self.start, 3),
                                             'phase': phase, 'session_id': identity,
                                             'reason': observer['close_reason']})
                self.last_phase = key
            if identity is not None:
                if identity not in self.sessions and len(self.sessions) >= 8:
                    self.error('SESSION_CAPTURE_LIMIT')
                else:
                    record = self.sessions.setdefault(identity, {'active': False, 'stop_ready': False})
                    record['active'] |= phase == 'ACTIVE'
                    proof = payload['lifecycle']['global_stop_collection']
                    record['stop_ready'] |= (phase == 'STOPPING' and proof['stop_evidence_ready'] is True
                                            and proof['session_id'] == identity
                                            and proof['close_generation'] == observer['close_generation'])
            count = runtime['closed_count']
            if type(count) is not int or count < self.closed:
                self.error('CLOSED_COUNT_INVALID')
            else:
                self.closed = count
            self.latest = payload
        except (ValueError, TypeError, KeyError):
            self.error('MISSION_PAYLOAD_INVALID')

    def graph(self):
        self.graph_samples += 1
        for topic in ('/cmd_vel_raw', '/pump_cmd', '/fire_robot_static/cmd_vel_raw', '/fire_target'):
            if self.count_publishers(topic) != 0:
                self.error('UNEXPECTED_COMMAND_PUBLISHER:' + topic)
        names = [name for name, _ in self.get_node_names_and_namespaces()]
        for name in ('live_nav_stop_gateway', 'live_alignment_takeover', 'ir_approach_live', 'pump_demo_controller'):
            if name in names:
                self.error('OMITTED_OWNER_RUNNING:' + name)
        for role, topic in self.topics.items():
            infos = self.get_publishers_info_by_topic(topic)
            expected = 'fire_mission_supervisor' if role == 'mission' else 'bt_navigator'
            if len(infos) != 1 or infos[0].node_name != expected or infos[0].node_namespace != '/':
                self.error('PUBLISHER_INVALID:' + role)
                continue
            gid = bytes(infos[0].endpoint_gid)
            if not any(gid) or (role in self.gids and gid != self.gids[role]):
                self.error('PUBLISHER_CHANGED:' + role)
            self.gids[role] = gid

    def receipt(self):
        now = time.monotonic()
        completed_fire = sum(record['active'] and record['stop_ready'] for record in self.sessions.values())
        checks = {
            'no_observed_safety_or_source_errors': not self.errors,
            'two_fire_sessions_with_postclose_proof': completed_fire >= 2 and self.closed >= 2,
            'idle_at_end': self.latest is not None and self.latest['lifecycle']['fire_session_observer']['phase'] == 'IDLE',
            'runtime_clear_at_end': self.latest is not None and
                self.latest['lifecycle']['shadow_session_runtime'].get('blocker') is None,
            'snapshot_sources_current': all(item['count'] >= 30 and item['last_mono'] is not None
                and 0 <= now - item['last_mono'] <= .35
                and item['last_age_ms'] + (now - item['last_mono']) * 1000 <= 350
                for item in self.stats.values()),
            'graph_observed': self.graph_samples >= 20,
        }
        return {'verdict': 'PASS_M3_STATIC_SESSION_RUNTIME' if all(checks.values()) else 'INCOMPLETE_OR_BLOCKED',
                'scope': 'stationary shadow session runtime; no goal, motor or actuator acceptance',
                'elapsed_s': round(now - self.start, 3), 'checks': checks,
                'errors': sorted(self.errors), 'closed_count': self.closed,
                'session_records': self.sessions, 'transitions': self.transitions,
                'stats': self.stats, 'graph_samples': self.graph_samples,
                'final_runtime': None if self.latest is None else self.latest['lifecycle']['shadow_session_runtime']}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--seconds', type=float, default=300)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 60 <= args.seconds <= 450:
        parser.error('seconds must be 60..450')
    output = Path(args.output)
    if not output.parent.is_dir() or output.exists():
        parser.error('choose a NEW output file in an existing directory')
    rclpy.init()
    node = Observer()
    executor = StopMetadataExecutor()
    executor.add_node(node)
    try:
        deadline = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        result = node.receipt()
        output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        print('FINAL_VERDICT=' + result['verdict'])
        return 0 if result['verdict'] == 'PASS_M3_STATIC_SESSION_RUNTIME' else 2
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
