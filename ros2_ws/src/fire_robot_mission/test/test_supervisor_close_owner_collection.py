"""Actual supervisor collector callbacks/publication, fake ROS effects only."""

from copy import deepcopy
import json
from types import SimpleNamespace

import pytest

from fire_robot_mission.close_owner_evidence import (
    CLOSE_OWNERS, CloseOwnerEvidenceCollector, OwnerCloseReporter,
)
from fire_robot_mission.mission_supervisor import (
    CLOSE_OWNER_MANIFEST, CLOSE_OWNER_TOPICS,
    FireMissionSupervisor, MissionSupervisorCore, ReliabilityPolicy,
)
from fire_robot_mission.session_context import SessionContextGuard


GOAL = 'ab' * 16
ORIGIN = 100_000_000_000


def ros(now):
    return ORIGIN + round(now * 1e9)


class Harness:
    _publish_state = FireMissionSupervisor._publish_state
    _on_close_owner = FireMissionSupervisor._on_close_owner
    _collect_close_owner = FireMissionSupervisor._collect_close_owner
    _on_pump_state = FireMissionSupervisor._on_pump_state
    _single_expected_publisher = FireMissionSupervisor._single_expected_publisher

    def __init__(self, *, selected=True, goal=GOAL, stopping=True):
        self._core = MissionSupervisorCore(enable_fire_session_observer=True)
        self._session_source_sequence = 0
        self._close_owner_collector = CloseOwnerEvidenceCollector() if selected else None
        self.pump_state_topic = CLOSE_OWNER_TOPICS['pump_demo_controller']
        self.now, self.stamp = 10.0, ros(10.0)
        self.events, self.messages = [], []
        self.infos = {topic: [SimpleNamespace(
            node_namespace='/', node_name=owner, topic_type='std_msgs/msg/String',
            qos_profile=SimpleNamespace(reliability=ReliabilityPolicy.BEST_EFFORT))]
            for owner, topic in CLOSE_OWNER_TOPICS.items()}
        self._state_publisher = SimpleNamespace(publish=self.publish)
        self.fail_publish = False
        self.graph_delay = 0.0
        self.publish_delay = 0.0
        session = self._core._fire_session
        if stopping:
            identity = session.begin(goal)
            session.request_close(identity.session_id, 'OPERATOR_CANCEL', self.now)
        self.contexts = {owner: SessionContextGuard() for owner in CLOSE_OWNERS}
        self.reporters = {owner: OwnerCloseReporter(owner) for owner in CLOSE_OWNERS}

    def advance(self, delta):
        self.now += delta
        self.stamp += round(delta * 1e9)

    def get_clock(self):
        self.events.append('clock')
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.stamp))

    def _now(self):
        return self.now

    def count_publishers(self, topic):
        assert topic == '/pump_cmd'
        self.events.append('pump_graph')
        return 0

    def get_publishers_info_by_topic(self, topic):
        self.events.append(('endpoint', topic))
        self.advance(self.graph_delay)
        return self.infos[topic]

    def publish(self, message):
        self.events.append('publish')
        if self.fail_publish:
            raise RuntimeError('publish failed')
        self.messages.append(json.loads(message.data))
        self.advance(self.publish_delay)

    def payload(self, owner, *, owned=False):
        context = self.contexts[owner]
        source = self.messages[-1]['lifecycle']['fire_session_observer']['source']
        if (context.last_sequence != source['sequence'] or
                context.last_instance != source['instance_id']):
            context.update(self.messages[-1], self.stamp, self.now, source_ok=True)
        assert context.current(self.now) is not None
        return {'close_owner_observation': self.reporters[owner].report(
            context, publisher_owned=owned, stamp_ns=self.stamp, now=self.now)}

    def receive(self, owner, payload=None):
        if payload is None:
            payload = self.payload(owner)
        message = SimpleNamespace(data=json.dumps(payload))
        if owner == 'pump_demo_controller':
            self._on_pump_state(message)
        else:
            self._on_close_owner(owner, message)

    def reports(self):
        return self.messages[-1]['lifecycle']['close_owner_collection']


@pytest.mark.parametrize('goal', [GOAL, None])
def test_actual_callbacks_correlate_four_current_acks_without_finishing_session(goal):
    node = Harness(goal=goal)
    node._publish_state()
    collector = node._close_owner_collector
    request = node._core._fire_session.close_request
    assert collector.request is request
    assert node.reports()['owner_reports_ready'] is False
    assert node.reports()['blockers'] == ['OWNER_CLOSE_ROUND_NOT_STARTED']
    assert node.events.index('publish') < len(node.events) - 1
    node.advance(0.1)
    for owner in CLOSE_OWNERS:
        node.receive(owner)
    node._publish_state()
    result = node.reports()
    assert result['owner_reports_ready'] is True and result['blockers'] == []
    assert result['nav_goal_uuid'] == goal and result['close_generation'] == request.generation
    assert result['required_owners'] == list(CLOSE_OWNERS)
    assert result['manifest'] == CLOSE_OWNER_MANIFEST
    assert result['actuation_authorized'] is result['cleanup_authorized'] is False
    assert node._core._fire_session.phase == 'STOPPING'
    assert node._core._fire_session.last_closed is None
    before = (collector.announced_ros_ns, collector.announced_at)
    node.advance(0.01)
    node._publish_state()
    assert (collector.announced_ros_ns, collector.announced_at) == before


def test_only_successful_publish_announces_close_and_slow_publish_is_inside_barrier():
    node = Harness()
    node.fail_publish = True
    with pytest.raises(RuntimeError):
        node._publish_state()
    assert node._close_owner_collector.request is None
    node.fail_publish, node.publish_delay = False, 0.2
    node._publish_state()
    collector = node._close_owner_collector
    assert collector.announced_at == pytest.approx(10.2)
    assert collector.announced_ros_ns == ros(10.2)
    assert node.messages[-1]['lifecycle']['fire_session_observer']['source']['stamp']['sec'] == 110


def test_idle_supervisor_never_starts_or_fabricates_a_close_round():
    node = Harness(stopping=False)
    node._publish_state()
    assert node._close_owner_collector.request is None
    assert node.messages[-1]['lifecycle']['fire_session_observer']['phase'] == 'IDLE'
    assert not node.reports()['owner_reports_ready']


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
def test_still_owned_report_never_becomes_ready_through_supervisor(owner):
    node = Harness()
    node._publish_state()
    node.advance(0.1)
    for role in CLOSE_OWNERS:
        node.receive(role, node.payload(role, owned=role == owner))
    node._publish_state()
    assert not node.reports()['owner_reports_ready']
    assert owner + ':OWNER_STILL_OWNED' in node.reports()['blockers']


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
def test_unlaunched_or_silent_role_remains_required_without_fabricating_no_owner(owner):
    node = Harness()
    node._publish_state()
    node.advance(0.1)
    for other in CLOSE_OWNERS:
        if other != owner:
            node.receive(other)
    node._publish_state()
    assert not node.reports()['owner_reports_ready']
    assert owner + ':OWNER_REPORT_NOT_RECEIVED' in node.reports()['blockers']


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
@pytest.mark.parametrize('bad_source', ['duplicate', 'wrong_name', 'namespace', 'type', 'qos'])
def test_actual_endpoint_checks_invalidate_previous_good_sample(owner, bad_source):
    node = Harness()
    node._publish_state()
    node.advance(0.1)
    for role in CLOSE_OWNERS:
        node.receive(role)
    node.advance(0.01)
    entry = node.infos[CLOSE_OWNER_TOPICS[owner]][0]
    if bad_source == 'duplicate':
        node.infos[CLOSE_OWNER_TOPICS[owner]].append(deepcopy(entry))
    elif bad_source == 'wrong_name':
        entry.node_name = 'other_node'
    elif bad_source == 'namespace':
        entry.node_namespace = '/other'
    elif bad_source == 'type':
        entry.topic_type = 'std_msgs/msg/Bool'
    else:
        entry.qos_profile.reliability = ReliabilityPolicy.RELIABLE
    node.receive(owner)
    node._publish_state()
    assert owner + ':OWNER_PUBLISHER_INVALID' in node.reports()['blockers']
    assert not node.reports()['owner_reports_ready']


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
@pytest.mark.parametrize('data', ['{', 'null', '[]', '{}', '{"close_owner_observation":{}}'])
def test_malformed_legacy_or_missing_observation_clears_sample(owner, data):
    node = Harness()
    node._publish_state()
    node.advance(0.1)
    node.receive(owner)
    node.advance(0.01)
    message = SimpleNamespace(data=data)
    if owner == 'pump_demo_controller':
        node._on_pump_state(message)
    else:
        node._on_close_owner(owner, message)
    assert node._close_owner_collector._samples[owner] is None


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
def test_graph_delay_is_included_in_age_instead_of_redating_sample(owner):
    node = Harness()
    node._publish_state()
    node.advance(0.1)
    payload = node.payload(owner)
    node.graph_delay = 0.4
    node.receive(owner, payload)
    assert node._close_owner_collector._samples[owner] is None
    assert 'STALE' in node._close_owner_collector._reasons[owner]


def test_caches_age_during_missing_messages_and_restarted_owner_cannot_rebaseline():
    node = Harness()
    node._publish_state()
    node.advance(0.1)
    for owner in CLOSE_OWNERS:
        node.receive(owner)
    node._publish_state()
    assert node.reports()['owner_reports_ready']
    node.advance(0.36)
    node._publish_state()
    assert not node.reports()['owner_reports_ready']
    owner = CLOSE_OWNERS[0]
    node.reporters[owner] = OwnerCloseReporter(owner)
    node.receive(owner)
    node._publish_state()
    assert node._close_owner_collector.fault == 'OWNER_PROCESS_CHANGED:' + owner
    assert not node.reports()['owner_reports_ready']


def test_new_pending_close_round_is_latched_not_replaced_or_rearmed():
    from dataclasses import replace
    node = Harness()
    node._publish_state()
    collector = node._close_owner_collector
    original = collector.request
    node.advance(0.1)
    for owner in CLOSE_OWNERS:
        node.receive(owner)
    node._publish_state()
    assert node.reports()['owner_reports_ready']
    node._core._fire_session.close_request = replace(original, generation=original.generation + 1)
    node.advance(0.1)
    node._publish_state()
    assert collector.request is original
    assert collector.fault == 'OWNER_CLOSE_ROUND_CHANGED'
    assert collector.fault in node.reports()['blockers']
    assert not node.reports()['owner_reports_ready']


def test_selected_mode_does_not_query_owner_graph_from_publish_timer():
    node = Harness()
    node._publish_state()
    node.advance(0.1)
    node.events.clear()
    node._publish_state()
    assert node.events.count('pump_graph') == 1
    assert not any(isinstance(event, tuple) for event in node.events)
    # Repeated heartbeats do not take another announcement timestamp.
    assert node.events.count('clock') == 2


def test_pump_reuses_endpoint_query_and_updates_existing_core_when_collection_off():
    node = Harness(selected=False)
    updates = []
    node._core.update_pump = lambda payload, now: updates.append((payload, now))
    node._on_pump_state(SimpleNamespace(data='{"state":"DISABLED"}'))
    assert updates == [({'state': 'DISABLED'}, node.now)]
    assert node.events == [('endpoint', node.pump_state_topic)]
    node.events.clear()
    node._publish_state()
    assert 'close_owner_collection' not in node.messages[-1]['lifecycle']
    assert node.events == ['clock', 'pump_graph', 'publish']


def test_off_callbacks_do_not_decode_query_or_read_clock_for_optional_sources():
    node = Harness(selected=False)
    for owner in CLOSE_OWNERS[:-1]:
        node._on_close_owner(owner, SimpleNamespace(data='{'))
    assert node.events == []


def test_clock_rollback_stays_blocked_without_replacing_collector():
    node = Harness()
    node._publish_state()
    collector = node._close_owner_collector
    node.advance(0.1)
    for owner in CLOSE_OWNERS:
        node.receive(owner)
    node.stamp -= 1_000_000_000
    node._publish_state()
    assert collector.fault == 'OWNER_COLLECTOR_CLOCK_BACKWARD'
    node.advance(2.0)
    node._publish_state()
    assert node._close_owner_collector is collector
    assert collector.fault in node.reports()['blockers']


def test_packaged_yaml_keeps_collection_off_and_manifest_requires_all_reporters():
    from pathlib import Path
    import yaml
    package = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((package / 'config/mission_params.yaml').read_text())[
        'fire_mission_supervisor']['ros__parameters']
    assert params['enable_close_owner_collection'] is False
    assert params['close_owner_manifest'] == ''
    assert tuple(CLOSE_OWNER_TOPICS) == CLOSE_OWNERS
    assert len(set(CLOSE_OWNER_TOPICS.values())) == 4


@pytest.mark.parametrize('mode', [
    'off', 'selected', 'bad_bool', 'observer_off', 'no_manifest',
    'unknown_manifest', 'pump_remap', 'state_remap',
])
def test_actual_constructor_validates_explicit_roster_before_ros_effects(monkeypatch, mode):
    from rclpy.node import Node
    cls = FireMissionSupervisor
    params, subscriptions, timers, publishers = {}, [], [], []
    overrides = {} if mode == 'off' else {
        'enable_fire_session_observer': True,
        'enable_close_owner_collection': True,
        'close_owner_manifest': CLOSE_OWNER_MANIFEST,
    }
    if mode == 'bad_bool':
        overrides['enable_close_owner_collection'] = 'false'
    elif mode == 'observer_off':
        overrides['enable_fire_session_observer'] = False
    elif mode == 'no_manifest':
        overrides['close_owner_manifest'] = ''
    elif mode == 'unknown_manifest':
        overrides['close_owner_manifest'] = 'auto_detect_graph'
    elif mode == 'pump_remap':
        overrides['pump_state_topic'] = '/other'
    elif mode == 'state_remap':
        overrides['state_topic'] = '/other'
    monkeypatch.setattr(Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(cls, 'declare_parameter',
                        lambda self, name, value: params.setdefault(name, overrides.get(name, value)))
    monkeypatch.setattr(cls, 'get_parameter', lambda self, name: SimpleNamespace(value=params[name]))
    monkeypatch.setattr(cls, 'create_publisher',
                        lambda self, kind, topic, qos: (publishers.append(topic), object())[1])
    monkeypatch.setattr(cls, 'create_subscription',
                        lambda self, kind, topic, callback, qos: subscriptions.append((topic, callback)))
    monkeypatch.setattr(cls, 'create_timer', lambda self, period, callback: timers.append(period))
    monkeypatch.setattr(cls, 'get_logger', lambda self: SimpleNamespace(
        warning=lambda msg: None, info=lambda msg: None))
    if mode not in ('off', 'selected'):
        with pytest.raises(ValueError):
            cls()
        assert not publishers and not subscriptions and not timers
        return
    node = cls()
    assert (node._close_owner_collector is not None) is (mode == 'selected')
    assert len(timers) == 1 and publishers == ['/fire_mission/state']
    assert len(subscriptions) == (13 if mode == 'selected' else 10)
    assert [topic for topic, _ in subscriptions].count(CLOSE_OWNER_TOPICS['pump_demo_controller']) == 1
    if mode == 'selected':
        from inspect import signature
        received = []
        node._on_close_owner = lambda role, message: received.append(role)
        for owner, topic in CLOSE_OWNER_TOPICS.items():
            if owner != 'pump_demo_controller':
                callback = next(callback for sub_topic, callback in subscriptions if sub_topic == topic)
                # One positional message argument; role cannot be MessageInfo.
                with pytest.raises(TypeError):
                    signature(callback).bind(object(), object())
                callback(SimpleNamespace(data='{}'))
        assert received == list(CLOSE_OWNERS[:-1])
