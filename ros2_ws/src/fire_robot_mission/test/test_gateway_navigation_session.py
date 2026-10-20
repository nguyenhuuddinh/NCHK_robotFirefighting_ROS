"""Real gateway callbacks/core/tick/effects with fake graph/clock, no ROS init."""

from copy import deepcopy
import json
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry

from fire_robot_mission.live_nav_stop_gateway import (
    LiveNavStopGateway, MISSION_ENDPOINT, ODOM_ENDPOINT, STATUS_ENDPOINT,
)
from fire_robot_mission.motion_release import MotionReleaseProof
from fire_robot_mission.session_context import SessionContextGuard
from fire_robot_safety.odom_freshness import OdomStampGuard
from test_live_nav_stop_gateway import (
    ACTUATOR_SOURCE, MOVING, NAV_SOURCES, RAW_SOURCE, SAFETY_SOURCE, ZERO,
    mission, ready_core,
)
from test_raw_publisher_release import command_values
from test_session_context import INSTANCE, ORIGIN, report


GOAL = 'ab' * 16


class Publisher:
    def __init__(self, node, *, state=False):
        self.node, self.state = node, state
        self.messages = []

    def publish(self, message):
        self.node.events.append('state' if self.state else 'command')
        self.messages.append(json.loads(message.data) if self.state else message)
        if not self.state:
            self.node.now += self.node.publish_delay


class Harness:
    _tick = LiveNavStopGateway._tick
    _tick_navigation_session = LiveNavStopGateway._tick_navigation_session
    _guard_owned_navigation = LiveNavStopGateway._guard_owned_navigation
    _navigation_inputs_ok = LiveNavStopGateway._navigation_inputs_ok
    _navigation_release_evidence = LiveNavStopGateway._navigation_release_evidence
    _on_mission = LiveNavStopGateway._on_mission
    _on_status = LiveNavStopGateway._on_status
    _on_odom = LiveNavStopGateway._on_odom
    _values = staticmethod(LiveNavStopGateway._values)

    def __init__(self, monkeypatch, *, feature=True):
        from fire_robot_mission import live_nav_stop_gateway as module
        self.now = 10.0
        monkeypatch.setattr(module.time, 'monotonic', lambda: self.now)
        self.core = ready_core()
        self._odom_stamp_guard = OdomStampGuard()
        self._session_context = SessionContextGuard() if feature else None
        self._motion_release = (MotionReleaseProof(owner='live_nav_stop_gateway')
                                if feature else None)
        self._raw_publisher = None
        self._state_publisher = Publisher(self, state=True)
        self.events, self.created, self.queries = [], [], []
        self.seq = self.clock_reads = 0
        self.visible = False
        self.raw_override = None
        self.query_delay = self.create_delay = self.publish_delay = 0.0
        self.destroy_result = True
        self.sources = {
            '/fire_mission/nav2_cmd_vel_shadow': NAV_SOURCES,
            '/cmd_vel': SAFETY_SOURCE,
            '/fire_sensors/actuator_state': ACTUATOR_SOURCE,
            '/fire_mission/state': list(MISSION_ENDPOINT.elements()),
            '/odom': list(ODOM_ENDPOINT.elements()),
            '/navigate_to_pose/_action/status': list(STATUS_ENDPOINT.elements()),
        }
        self.counts = {}

    def get_clock(self):
        self.clock_reads += 1
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=ORIGIN + round(self.now * 1e9)))

    def query(self, topic):
        self.queries.append(topic)
        self.now += self.query_delay

    def _sources(self, topic):
        self.query(topic)
        if topic == '/cmd_vel_raw':
            if self.raw_override is not None:
                return self.raw_override
            return RAW_SOURCE if self.visible and self._raw_publisher is not None else []
        return self.sources[topic]

    def count_publishers(self, topic):
        self.query(topic)
        return self.counts.get(topic, 0 if topic == '/pump_cmd' else 1)

    def create_publisher(self, _type, topic, _qos):
        assert topic == '/cmd_vel_raw'
        self.events.append('create')
        self.now += self.create_delay
        result = Publisher(self)
        self.created.append(result)
        return result

    def destroy_publisher(self, publisher):
        assert publisher is self._raw_publisher
        self.events.append('destroy')
        if isinstance(self.destroy_result, Exception):
            raise self.destroy_result
        return self.destroy_result

    def get_logger(self):
        return SimpleNamespace(warning=lambda msg: None, warn=lambda msg: None,
                               info=lambda msg: None)

    def status(self, goal=GOAL, status=2):
        self._on_status(SimpleNamespace(status_list=[SimpleNamespace(
            goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=bytes.fromhex(goal))),
            status=status)]))

    def feed(self, *, phase='IDLE', generation=None, session=7, goal=GOAL,
             instance=INSTANCE, old_ms=10, navigating=True, velocities=ZERO):
        self.seq += 1
        payload = mission() if navigating else mission('IDLE', 'NONE', goal=False)
        if phase != 'IDLE':
            payload = mission('FIRE_TRACKING', 'LEFT_PREVIEW')
        payload.update(report(
            ORIGIN + round(self.now * 1e9) - round(old_ms * 1e6), self.seq,
            phase=phase, generation=generation, session=session,
            goal=None if phase == 'IDLE' else goal, instance=instance))
        self._on_mission(SimpleNamespace(data=json.dumps(payload)))
        self.core.update_nav(MOVING, self.now)
        self.core.update_cmd(ZERO, self.now)
        self.core.update_arm(True, self.now)
        msg = Odometry()
        sec, ns = divmod(ORIGIN + round(self.now * 1e9) - 10_000_000, 1_000_000_000)
        msg.header.stamp.sec, msg.header.stamp.nanosec = sec, ns
        (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
         msg.twist.twist.angular.x, msg.twist.twist.angular.y,
         msg.twist.twist.angular.z) = velocities
        self._on_odom(msg)

    def start(self):
        self.status()
        self.feed()
        self._tick()
        assert len(self.created) == 1
        assert command_values(self.created[0].messages[-1]) == ZERO
        self.visible = True
        self.now = 10.1
        self.feed()
        self._tick()
        assert command_values(self.created[0].messages[-1]) == MOVING

    def close(self, *, final=True):
        for now in ((10.2, 10.5, 10.8) if final else (10.2, 10.5)):
            self.now = now
            self.feed(phase='STOPPING', generation=1)
            self._tick()


def test_real_nav_before_fire_then_first_close_stop_publish_destroy_receipt(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    lease = node._motion_release
    binding, token, created = lease.binding, lease.owner_token, lease.created_at
    assert binding.nav_goal_uuid == GOAL and not hasattr(binding, 'session_id')
    assert lease.fire_binding is None and lease.zero_published_at is None
    raw = node._raw_publisher
    node.close()
    assert node.events[-3:] == ['command', 'destroy', 'state']
    assert command_values(raw.messages[-1]) == ZERO
    assert lease.binding is binding and lease.owner_token == token and lease.created_at == created
    receipt = lease.receipt
    assert receipt['nav_goal_uuid'] == GOAL
    assert receipt['session_id'] == f'fire:{INSTANCE}:7'
    assert receipt['close_generation'] == 1
    assert receipt['fire_associated_mono'] == receipt['close_received_mono'] == 10.2
    assert receipt['zero_published_mono'] == receipt['stationary_hold_started_mono'] == 10.2
    assert receipt['odom_evidence_mono'] > receipt['zero_published_mono']
    assert receipt['physical_motion_verified'] is False
    state = node._state_publisher.messages[-1]
    assert state['released'] is True and state['session_owner']['publisher_owned'] is False
    assert state['handoff_ready'] is state['next_goal_clear_ready'] is False


@pytest.mark.parametrize('terminal', [4, 5, 6])
def test_no_fire_terminal_releases_real_goal_without_inventing_fire(monkeypatch, terminal):
    node = Harness(monkeypatch)
    node.start()
    node.now = 10.2
    node.status(status=terminal)
    for now in (10.2, 10.5, 10.8):
        node.now = now
        node.feed(navigating=False)
        node._tick()
    receipt = node._motion_release.receipt
    assert receipt['nav_goal_uuid'] == GOAL
    assert receipt['session_id'] is receipt['session_binding'] is receipt['close_generation'] is None
    assert node.core.normal_terminal_status == terminal
    assert node.core.stop_reason == 'NAV_GOAL_TERMINAL_NO_FIRE'
    assert node._raw_publisher is None


@pytest.mark.parametrize('context', [
    {'phase': 'ACTIVE'}, {'phase': 'STOPPING', 'generation': 1}, {'old_ms': 400},
])
def test_no_pre_fire_idle_context_never_acquires_or_fakes_release(monkeypatch, context):
    node = Harness(monkeypatch)
    for now in (10.0, 10.2, 10.5):
        node.now = now
        node.feed(**context)
        node._tick()
    assert not node.created and node._raw_publisher is None
    assert node.core.released is False
    assert node._motion_release.binding is node._motion_release.receipt is None


@pytest.mark.parametrize('topic', ['/fire_mission/state', '/odom',
                                 '/navigate_to_pose/_action/status'])
@pytest.mark.parametrize('change', ['name', 'namespace', 'type', 'qos', 'duplicate'])
def test_wrong_exact_source_cannot_acquire(monkeypatch, topic, change):
    node = Harness(monkeypatch)
    source = list(node.sources[topic][0])
    if change == 'duplicate':
        node.sources[topic] *= 2
    else:
        index = {'namespace': 0, 'name': 1, 'type': 2, 'qos': 3}[change]
        source[index] = 99 if change == 'qos' else 'foreign'
        node.sources[topic] = [tuple(source)]
    node.status()
    node.feed()
    node._tick()
    assert not node.created and node._motion_release.binding is None


@pytest.mark.parametrize('bad', ['malformed', 'source', 'stale', 'instance', 'goal'])
def test_bad_then_good_callback_before_tick_still_stops_original_lease(monkeypatch, bad):
    node = Harness(monkeypatch)
    node.start()
    binding, token = node._motion_release.binding, node._motion_release.owner_token
    node.now = 10.2
    if bad == 'malformed':
        node._on_mission(SimpleNamespace(data='{broken'))
    elif bad == 'source':
        original = node.sources['/fire_mission/state']
        node.sources['/fire_mission/state'] = []
        node.feed()
        node.sources['/fire_mission/state'] = original
    elif bad == 'stale':
        node.feed(old_ms=400)
    elif bad == 'instance':
        node.feed(instance='ef' * 16)
    else:
        node.feed(phase='ACTIVE', goal='ef' * 16)
    node.now = 10.21
    node.feed()
    node._tick()
    assert command_values(node.created[0].messages[-1]) == ZERO
    assert node.core.stop_latched and node._motion_release.stop_latched
    assert node._motion_release.binding is binding and node._motion_release.owner_token == token
    assert node._motion_release.fire_binding is None


def test_wrong_status_callback_cannot_resume_when_source_returns(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.now = 10.2
    original = node.sources['/navigate_to_pose/_action/status']
    node.sources['/navigate_to_pose/_action/status'] = []
    node.status(status=4)
    assert node.core.normal_terminal_uuid is None and node.core.statuses == ()
    node.sources['/navigate_to_pose/_action/status'] = original
    node.now = 10.21
    node.status()
    node.feed()
    node._tick()
    assert node.core.stop_latched
    assert command_values(node.created[0].messages[-1]) == ZERO


@pytest.mark.parametrize('other_goal', ['ef' * 16, 'cd' * 16])
def test_goal_change_and_return_between_ticks_does_not_resume_old_navigation(monkeypatch, other_goal):
    node = Harness(monkeypatch)
    node.start()
    binding = node._motion_release.binding
    node.now = 10.2
    node.status(other_goal)
    node.now = 10.21
    node.status()
    node.feed()
    node._tick()
    assert node.core.stop_latched and node._motion_release.stop_latched
    assert node._motion_release.binding is binding and node.core.goal_uuid == GOAL
    assert command_values(node.created[0].messages[-1]) == ZERO


def test_first_fire_preserves_navigation_identity_and_duplicate_close_anchors(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.now = 10.2
    node.feed(phase='ACTIVE')
    node._tick()
    lease = node._motion_release
    fire = lease.fire_binding
    assert lease.fire_associated_at == lease.zero_published_at == 10.2
    assert lease.close_generation is None
    node.now = 10.4
    node.feed(phase='STOPPING', generation=1)
    node._tick()
    close = deepcopy(lease.close_source)
    assert lease.zero_published_at == lease.close_received_at == node.core.stop_since == 10.4
    node.now = 10.6
    node.feed(phase='STOPPING', generation=1)
    node._tick()
    assert lease.fire_binding is fire and lease.fire_associated_at == 10.2
    assert lease.zero_published_at == lease.close_received_at == node.core.stop_since == 10.4
    assert lease.close_source == close
    node.now = 10.7
    node.feed(phase='STOPPING', generation=2)
    node._tick()
    assert lease.close_generation == 1 and lease.close_source == close
    assert lease.receipt is None
    assert node._state_publisher.messages[-1]['session_owner']['blocker'] == 'SESSION_CLOSE_CONTEXT_CHANGED'


@pytest.mark.parametrize('changes', [{'session': 8}, {'goal': 'ef' * 16},
                                     {'instance': 'ef' * 16}, {'phase': 'IDLE'}])
def test_associated_fire_cannot_switch_identity_or_resume_nav(monkeypatch, changes):
    node = Harness(monkeypatch)
    node.start()
    node.now = 10.2
    node.feed(phase='ACTIVE')
    node._tick()
    lease = node._motion_release
    binding = lease.fire_binding
    node.now = 10.3
    node.feed(**({'phase': 'ACTIVE'} | changes))
    node._tick()
    assert lease.fire_binding is binding and node.core.stop_latched
    assert command_values(node.created[0].messages[-1]) == ZERO


def test_old_event_driven_status_does_not_need_a_fake_heartbeat(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    for now in (20.0, 20.1):
        node.now = now
        node.feed()
        node._tick()
    assert node.core.status_at == 10.0
    assert command_values(node.created[0].messages[-1]) == MOVING


def test_initial_discovery_only_sends_zero_then_first_visible_owner_may_navigate(monkeypatch):
    node = Harness(monkeypatch)
    for now in (10.0, 10.1):
        node.now = now
        node.feed()
        node._tick()
    assert len(node.created) == 1 and not node.core.stop_latched
    assert all(command_values(msg) == ZERO for msg in node.created[0].messages)
    node.visible = True
    node.now = 10.2
    node.feed()
    node._tick()
    assert command_values(node.created[0].messages[-1]) == MOVING
    assert node._motion_release.zero_published_at is None


@pytest.mark.parametrize('raw', [[], RAW_SOURCE * 2, SAFETY_SOURCE])
def test_missing_foreign_or_duplicate_raw_owner_after_motion_stops_and_blocks_receipt(monkeypatch, raw):
    node = Harness(monkeypatch)
    node.start()
    node.raw_override = raw
    for now in (10.2, 10.5, 10.8):
        node.now = now
        node.feed(phase='STOPPING', generation=1)
        node._tick()
    assert node.core.stop_latched and not node.core.released
    assert node._motion_release.receipt is None
    assert command_values(node.created[0].messages[-1]) == ZERO


def test_arm_loss_stops_and_a_new_heartbeat_does_not_resume_the_lease(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.now = 10.2
    node.feed()
    node.core.update_arm(False, node.now)
    node._tick()
    assert node.core.stop_latched and node._motion_release.stop_latched
    node.now = 10.3
    node.feed()
    node._tick()
    assert command_values(node.created[0].messages[-1]) == ZERO
    assert node._motion_release.receipt is None


@pytest.mark.parametrize('stage', ['create', 'motion', 'release'])
def test_post_compute_expiry_blocks_create_motion_and_release(monkeypatch, stage):
    node = Harness(monkeypatch)
    if stage != 'create':
        node.start()
    if stage == 'release':
        node.close(final=False)
        node.now = 10.8
        node.feed(phase='STOPPING', generation=1)
    elif stage == 'motion':
        node.now = 10.2
        node.feed()
    else:
        node.feed()
    evaluate = node.core.evaluate

    def slow(*args, **kwargs):
        result = evaluate(*args, **kwargs)
        node.now += 0.4
        return result

    monkeypatch.setattr(node.core, 'evaluate', slow)
    if stage == 'create':
        with pytest.raises(RuntimeError, match='NAVIGATION_SESSION_ACQUIRE_UNCONFIRMED'):
            node._tick()
        assert not node.created and node._motion_release.binding is None
    else:
        node._tick()
        assert node._raw_publisher is not None
        assert command_values(node._raw_publisher.messages[-1]) == ZERO
        assert node._state_publisher.messages[-1]['command'] == {
            'linear_x_m_s': 0.0, 'angular_z_rad_s': 0.0}
    assert node._motion_release.receipt is None and not node.core.released
    assert 'destroy' not in node.events


def test_graph_query_delay_is_counted_and_exact_sources_reuse_the_count_queries(monkeypatch):
    node = Harness(monkeypatch)
    node.feed()
    node.queries.clear()
    node.query_delay = 0.04
    node._tick()
    assert len(node.queries) == len(set(node.queries)) == 10 and node.now > 10.35
    assert not node.created


def test_actual_creation_completion_is_anchor_and_slow_create_first_publishes_zero(monkeypatch):
    node = Harness(monkeypatch)
    node.feed()
    node.create_delay = 0.4
    node._tick()
    assert node._motion_release.created_at == node.core.publisher_created_at == pytest.approx(10.4)
    assert command_values(node.created[0].messages[-1]) == ZERO
    assert node._motion_release.zero_published_at == pytest.approx(10.4)


def test_post_publish_expiry_keeps_owner_without_a_receipt(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.close(final=False)
    node.now = 10.8
    node.feed(phase='STOPPING', generation=1)
    node.publish_delay = 0.4
    node._tick()
    assert node._raw_publisher is not None and node.core.stop_since is None
    assert not node.core.released and node._motion_release.receipt is None
    assert 'destroy' not in node.events


def test_create_api_failure_never_creates_a_lease_or_acknowledges_acquisition(monkeypatch):
    node = Harness(monkeypatch)
    node.feed()

    def fail(*args):
        raise RuntimeError('create failed')

    monkeypatch.setattr(node, 'create_publisher', fail)
    with pytest.raises(RuntimeError, match='create failed'):
        node._tick()
    assert node._raw_publisher is None and node.core.publisher_created_at is None
    assert node._motion_release.binding is node._motion_release.owner_token is None


def test_publish_failure_cannot_record_zero_or_release(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    assert node._motion_release.zero_published_at is None
    node.now = 10.2
    node.feed(phase='STOPPING', generation=1)

    def fail(message):
        raise RuntimeError('publish failed')

    monkeypatch.setattr(node._raw_publisher, 'publish', fail)
    with pytest.raises(RuntimeError, match='publish failed'):
        node._tick()
    assert node._raw_publisher is node.created[0]
    assert node._motion_release.zero_published_at is node._motion_release.receipt is None
    assert not node.core.released and 'destroy' not in node.events


@pytest.mark.parametrize('result', [False, None, RuntimeError('destroy failed')])
def test_destroy_requires_true_before_receipt_reference_clear_or_ack(monkeypatch, result):
    node = Harness(monkeypatch)
    node.start()
    node.close(final=False)
    node.now = 10.8
    node.feed(phase='STOPPING', generation=1)
    node.destroy_result = result
    state_count = len(node._state_publisher.messages)
    with pytest.raises(RuntimeError):
        node._tick()
    assert node._raw_publisher is node.created[0]
    assert command_values(node.created[0].messages[-1]) == ZERO
    assert not node.core.released and node._motion_release.receipt is None
    assert len(node._state_publisher.messages) == state_count


@pytest.mark.parametrize('topic', ['/odom', '/cmd_vel', '/navigate_to_pose/_action/status'])
def test_bad_motion_source_stops_and_cannot_accumulate_a_release_hold(monkeypatch, topic):
    node = Harness(monkeypatch)
    node.start()
    node.sources[topic] = []
    for now in (10.2, 10.5, 10.8):
        node.now = now
        node.feed(phase='STOPPING', generation=1)
        node._tick()
    assert node._raw_publisher is not None and node._motion_release.receipt is None
    assert command_values(node.created[0].messages[-1]) == ZERO
    if topic == '/odom':
        assert node.core.odom is node.core.odom_at is None


@pytest.mark.parametrize('sample', ['odom', 'cmd'])
def test_nonstationary_or_nonzero_sample_breaks_stop_hold(monkeypatch, sample):
    node = Harness(monkeypatch)
    node.start()
    node.close(final=False)
    node.now = 10.8
    node.feed(phase='STOPPING', generation=1)
    getattr(node.core, 'update_' + sample)((0.03, 0, 0, 0, 0, 0), node.now)
    node._tick()
    assert node.core.stop_since is None and node._motion_release.receipt is None
    assert 'destroy' not in node.events


def test_release_history_not_retagged_by_late_close_new_goal_or_new_session(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.now = 10.2
    node.core.request_navigation_stop('OPERATOR_TEST_STOP')
    for now in (10.2, 10.5, 10.8):
        node.now = now
        node.feed()
        node._tick()
    lease = node._motion_release
    receipt, token, binding = deepcopy(lease.receipt), lease.owner_token, lease.binding
    assert receipt['session_id'] is None
    for now in (11.0, 11.3, 11.6):
        node.now = now
        node.status('ef' * 16)
        node.feed(phase='STOPPING', generation=1, goal='ef' * 16, session=9)
        node._tick()
    assert lease.receipt == receipt and lease.binding is binding and lease.owner_token == token
    assert lease.fire_binding is None and len(node.created) == 1
    assert node.core.goal_uuid == GOAL and node.core.released
    assert node.core.used_goal_uuids == set()


def test_feature_off_retains_legacy_graph_budget_and_omits_context_clock_receipts(monkeypatch):
    node = Harness(monkeypatch, feature=False)
    node.core.update_mission(mission(), node.now)
    node._on_mission(SimpleNamespace(data=json.dumps(mission())))
    assert node.clock_reads == 0
    node.queries.clear()
    node._tick()
    assert len(node.queries) == len(set(node.queries)) == 10
    assert node.clock_reads == 0 and node._motion_release is node._session_context is None
    assert 'session_owner' not in node._state_publisher.messages[-1]


@pytest.mark.parametrize('overrides,valid,helper', [
    ({}, True, False),
    ({'require_fire_session_context': 'false'}, False, False),
    ({'require_fire_session_context': True}, True, False),
    ({'require_fire_session_context': True, 'enable_live_output': True}, True, True),
    ({'require_fire_session_context': True, 'stationary_hold_s': 0.49}, False, False),
    ({'require_fire_session_context': True, 'odom_timeout_s': 0.36}, False, False),
    ({'require_fire_session_context': True, 'mission_timeout_s': 0.36}, False, False),
    ({'require_fire_session_context': True, 'max_nav_linear_m_s': 0.11}, False, False),
    ({'require_fire_session_context': True, 'stationary_linear_m_s': 0.03}, False, False),
])
def test_actual_constructor_default_off_and_opt_in_limits(monkeypatch, overrides, valid, helper):
    from fire_robot_mission import live_nav_stop_gateway as module
    params = {}
    monkeypatch.setattr(module.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(LiveNavStopGateway, 'resolve_topic_name', lambda self, name: name)
    monkeypatch.setattr(LiveNavStopGateway, 'declare_parameter',
                        lambda self, name, value: params.setdefault(name, overrides.get(name, value)))
    monkeypatch.setattr(LiveNavStopGateway, 'get_parameter',
                        lambda self, name: SimpleNamespace(value=params[name]))
    monkeypatch.setattr(LiveNavStopGateway, 'create_publisher', lambda *args: object())
    monkeypatch.setattr(LiveNavStopGateway, 'create_subscription', lambda *args: None)
    monkeypatch.setattr(LiveNavStopGateway, 'create_timer', lambda *args: None)
    monkeypatch.setattr(LiveNavStopGateway, 'get_logger',
                        lambda self: SimpleNamespace(warning=lambda msg: None))
    if not valid:
        with pytest.raises(ValueError):
            LiveNavStopGateway()
        return
    node = LiveNavStopGateway()
    assert (node._session_context is not None) is helper
    assert (node._motion_release is not None) is helper
