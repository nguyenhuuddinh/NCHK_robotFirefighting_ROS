"""Gateway handoff identity and graph latency with real core/tick, no ROS init."""

import json
from types import SimpleNamespace

import pytest

from fire_robot_mission.live_nav_stop_gateway import LiveNavStopGateway
from test_live_nav_stop_gateway import (
    ACTUATOR_SOURCE, NAV_SOURCES, RAW_SOURCE, SAFETY_SOURCE, ZERO,
    mission, navigate, ready_core, step,
)


GOAL = 'ab' * 16


def canceled_guard(goal=GOAL):
    return {
        'schema_version': 1, 'state': 'CANCEL_CONFIRMED_STOPPED',
        'enabled': True, 'send_count': 1, 'goal_uuid': goal,
        'cancel_service_accepted': True, 'cancel_status_seen': True,
        'goal_status': 5, 'arm_fresh': True,
        'cmd_vel_fresh_zero': True, 'odom_fresh_stationary': True,
        'nav_shadow_fingerprint_ok': True, 'raw_cmd_publishers': 0,
        'blockers': [],
    }


def released_fire_core():
    core = ready_core()
    navigate(core)
    core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 10.15)
    core.update_odom(ZERO, 10.15)
    core.update_cmd(ZERO, 10.15)
    step(core, 10.16, owned=True, raw=RAW_SOURCE)
    core.update_odom(ZERO, 10.70)
    core.update_cmd(ZERO, 10.70)
    assert step(core, 10.71, owned=True, raw=RAW_SOURCE)[3] == 'RELEASE'
    core.publisher_released()
    core.update_statuses([(GOAL, 5)], 10.72)
    core.update_mission(mission('ALIGNMENT_READY', 'HOLD_CENTER_PREVIEW',
                                goal=False, eligible=True), 10.72)
    core.update_guard(canceled_guard(), 10.72)
    core.update_odom(ZERO, 10.72)
    core.update_cmd(ZERO, 10.72)
    assert step(core, 10.73)[2]['handoff_ready'] is True
    return core


@pytest.mark.parametrize('wrong_goal', ['cd' * 16, 'ef' * 16])
def test_fresh_successful_cancel_for_another_valid_goal_cannot_unlock_handoff(wrong_goal):
    core = released_fire_core()
    # Earlier correct evidence must not make the latest wrong-goal guard valid.
    assert core.cancel_confirmed_uuid == GOAL
    core.update_guard(canceled_guard(wrong_goal), 10.74)
    linear, angular, state, action = step(core, 10.75)
    assert state['cycle_goal_uuid'] == GOAL
    assert state['state'] == 'RELEASED_STOPPED'
    assert state['handoff_ready'] is False
    assert (linear, angular, action) == (0.0, 0.0, None)
    # A later matching report restores readiness, without acquiring motion.
    core.update_guard(canceled_guard(), 10.76)
    linear, angular, state, action = step(core, 10.77)
    assert state['handoff_ready'] is True
    assert (linear, angular, action) == (0.0, 0.0, None)


@pytest.mark.parametrize('cycle', [None, 'ef' * 16])
def test_missing_or_different_cycle_identity_cannot_adopt_guard_goal(cycle):
    core = released_fire_core()
    core.goal_uuid = cycle
    _, _, state, action = step(core, 10.74)
    assert state['handoff_ready'] is False
    assert core.goal_uuid == cycle
    assert action is None


def test_matching_handoff_keeps_event_driven_terminal_status_pruning_support():
    core = released_fire_core()
    core.update_statuses([], 10.74)
    _, _, state, action = step(core, 10.75)
    assert state['handoff_ready'] is True
    assert state['cycle_goal_uuid'] == GOAL
    assert action is None


class Publisher:
    def __init__(self, node, *, state=False):
        self.node, self.state = node, state
        self.messages = []

    def publish(self, msg):
        self.node.events.append('state' if self.state else 'command')
        self.messages.append(json.loads(msg.data) if self.state else msg)


class Harness:
    _tick = LiveNavStopGateway._tick

    def __init__(self, monkeypatch):
        from fire_robot_mission import live_nav_stop_gateway as module
        self.now = 10.05
        monkeypatch.setattr(module.time, 'monotonic', lambda: self.now)
        self.core = ready_core()
        self._raw_publisher = None
        self._state_publisher = Publisher(self, state=True)
        self.events, self.queries, self.created = [], [], []
        self.query_delay = self.create_delay = 0.0

    def query(self, topic):
        self.queries.append(topic)
        self.now += self.query_delay

    def _sources(self, topic):
        self.query(topic)
        if topic == '/cmd_vel_raw':
            return RAW_SOURCE if self._raw_publisher is not None else []
        return {
            '/fire_mission/nav2_cmd_vel_shadow': NAV_SOURCES,
            '/cmd_vel': SAFETY_SOURCE,
            '/fire_sensors/actuator_state': ACTUATOR_SOURCE,
        }[topic]

    def count_publishers(self, topic):
        self.query(topic)
        return 0 if topic == '/pump_cmd' else 1

    def create_publisher(self, _type, topic, _qos):
        assert topic == '/cmd_vel_raw'
        self.now += self.create_delay
        self.events.append('create')
        result = Publisher(self)
        self.created.append(result)
        return result

    def destroy_publisher(self, publisher):
        assert publisher is self._raw_publisher
        self.events.append('destroy')
        return True

    def get_logger(self):
        return SimpleNamespace(warning=lambda msg: None)

    def navigating(self):
        navigate(self.core)
        self.now = 10.10
        self._raw_publisher = Publisher(self)


def test_slow_graph_queries_cannot_acquire_with_expired_inputs(monkeypatch):
    node = Harness(monkeypatch)
    node.query_delay = 0.04
    node._tick()
    assert node.now > 10.4
    assert len(node.queries) == len(set(node.queries)) == 10
    assert node._raw_publisher is None and not node.created
    assert node._state_publisher.messages[-1]['state'] == 'WAITING_FOR_INPUT'
    assert 'MISSION_MISSING_OR_STALE' in node._state_publisher.messages[-1]['blockers']


def test_slow_graph_queries_stop_forwarding_when_source_inputs_expire(monkeypatch):
    node = Harness(monkeypatch)
    node.navigating()
    node.query_delay = 0.04
    node._tick()
    msg = node._raw_publisher.messages[-1]
    assert msg.linear.x == msg.angular.z == 0.0
    assert node.core.stop_latched is True
    assert node.core.released is False
    assert node._state_publisher.messages[-1]['state'] == 'STOPPING'


def test_graph_delay_cannot_complete_release_using_old_stationary_sample(monkeypatch):
    node = Harness(monkeypatch)
    node.navigating()
    node.core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 10.15)
    node.core.update_odom(ZERO, 10.15)
    node.core.update_cmd(ZERO, 10.15)
    step(node.core, 10.16, owned=True, raw=RAW_SOURCE)
    node.core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 10.70)
    node.core.update_odom(ZERO, 10.70)
    node.core.update_cmd(ZERO, 10.70)
    node.now, node.query_delay = 10.71, 0.04
    node._tick()
    assert node._raw_publisher is not None
    assert node._raw_publisher.messages[-1].linear.x == 0.0
    assert 'destroy' not in node.events
    assert node.core.stop_since is None and node.core.released is False


def test_creation_time_records_actual_create_completion_and_first_command_is_zero(monkeypatch):
    node = Harness(monkeypatch)
    node.create_delay = 0.2
    node._tick()
    assert len(node.created) == 1
    assert node.core.publisher_created_at == node.now == pytest.approx(10.25)
    msg = node.created[0].messages[0]
    assert msg.linear.x == msg.angular.z == 0.0
    assert len(node.queries) == len(set(node.queries)) == 10
