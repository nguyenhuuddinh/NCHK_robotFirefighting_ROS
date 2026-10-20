"""Actual publish callback with a real pure core; no ROS context or hardware."""

from copy import deepcopy
import json
from types import SimpleNamespace

import pytest

from fire_robot_mission.fire_session_core import FireSessionCore
from fire_robot_mission.mission_supervisor import (
    FireMissionSupervisor, MissionSupervisorCore,
)
from test_mission_fire_session_observer import acquire, GOAL


STAMP = 100_123_456_789


class Harness:
    _publish_state = FireMissionSupervisor._publish_state

    def __init__(self, *, enabled=True):
        self._core = MissionSupervisorCore(
            fire_acquire_hold_s=0.1, enable_fire_session_observer=enabled)
        if enabled:
            self._session_source_sequence = 0
        self.stamp = STAMP
        self.now = 0.11
        self.events = []
        self.messages = []
        self._state_publisher = SimpleNamespace(publish=self.publish)

    def get_clock(self):
        self.events.append('clock')
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.stamp))

    def count_publishers(self, topic):
        assert topic == '/pump_cmd'
        self.events.append('graph')
        # Simulate graph work taking time without real waiting.
        self.stamp += 100_000_000
        return 0

    def _now(self):
        self.events.append('evaluate')
        return self.now

    def publish(self, message):
        self.events.append('publish')
        self.messages.append(json.loads(message.data))

    def observer(self):
        return self.messages[-1]['lifecycle']['fire_session_observer']


def test_publish_stamps_before_graph_work_and_does_not_change_control_fields():
    node = Harness()
    node._core.update_pump_owner_count(0)
    expected = deepcopy(node._core.evaluate(node.now))
    node._publish_state()
    assert node.events == ['clock', 'graph', 'evaluate', 'publish']
    source = node.observer()['source']
    assert source['stamp'] == {'sec': 100, 'nanosec': 123_456_789}
    assert source['sequence'] == 1
    assert source['instance_id'] == node._core._fire_session.source_instance_id
    assert node.observer()['source_blocker'] is None
    actual = deepcopy(node.messages[-1])
    actual['lifecycle']['fire_session_observer'].pop('source')
    actual['lifecycle']['fire_session_observer'].pop('source_blocker')
    assert actual == expected


@pytest.mark.parametrize('phase', ['IDLE', 'ACTIVE', 'STOPPING'])
def test_heartbeats_keep_source_identity_and_close_generation_but_advance_sequence(phase):
    node = Harness()
    if phase != 'IDLE':
        acquire(node._core)
    if phase == 'STOPPING':
        identity = node._core._fire_session.identity
        node._core._fire_session.request_close(identity.session_id, 'OPERATOR_CANCEL', 0.11)
    node._publish_state()
    first = deepcopy(node.observer())
    node._publish_state()
    second = node.observer()
    assert second['phase'] == first['phase'] == phase
    assert second['session_id'] == first['session_id']
    assert second['close_generation'] == first['close_generation']
    assert second['source']['instance_id'] == first['source']['instance_id']
    assert second['source']['sequence'] == first['source']['sequence'] + 1
    assert second['source']['stamp'] != first['source']['stamp']
    assert second['actuation_authorized'] is False


def test_default_off_does_not_read_clock_create_sequence_or_add_source_fields():
    node = Harness(enabled=False)
    node._publish_state()
    assert node.events == ['graph', 'evaluate', 'publish']
    assert not hasattr(node, '_session_source_sequence')
    assert 'fire_session_observer' not in node.messages[-1]['lifecycle']


@pytest.mark.parametrize('stamp', [0, -1, True, None])
def test_invalid_clock_does_not_emit_a_usable_source_or_advance_sequence(stamp):
    node = Harness()
    node.stamp = stamp
    # Keep the simulated graph from doing arithmetic on invalid input.
    node.count_publishers = lambda topic: 0
    node._publish_state()
    assert node.observer()['source'] is None
    assert node.observer()['source_blocker'] == 'SESSION_CLOCK_INVALID'
    assert node._session_source_sequence == 0


def test_same_ros_time_has_distinct_sequence_without_fabricating_a_new_stamp():
    node = Harness()
    node._publish_state()
    node.stamp = STAMP
    node._publish_state()
    first, second = [p['lifecycle']['fire_session_observer']['source'] for p in node.messages]
    assert first['stamp'] == second['stamp']
    assert second['sequence'] == 2


def test_restarted_core_does_not_reuse_the_source_namespace():
    first = FireSessionCore(proof_timeout_s=0.35)
    second = FireSessionCore(proof_timeout_s=0.35)
    assert first.source_instance_id != second.source_instance_id
    instance = first.source_instance_id
    identity = first.begin(GOAL)
    assert identity.session_id.split(':')[1] == instance
    first.request_close(identity.session_id, 'SAFETY_ABORT', 1.0)
    assert first.source_instance_id == instance
