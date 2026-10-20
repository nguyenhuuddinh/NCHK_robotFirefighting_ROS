"""Actual owner callbacks/ticks/effects with fake graph/clock; no ROS init."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from fire_robot_mission.close_owner_evidence import CLOSE_OWNERS, OwnerCloseReporter
from fire_robot_mission.ir_approach_live import IrApproachLiveNode
from fire_robot_mission.live_alignment_takeover import LiveAlignmentTakeover
from fire_robot_mission.live_nav_stop_gateway import LiveNavStopGateway
from fire_robot_mission.pump_demo_controller import PumpDemoController
from fire_robot_mission.session_context import SessionContextGuard
from test_alignment_session_binding import Harness as AlignmentHarness
from test_gateway_navigation_session import Harness as GatewayHarness
from test_ir_session_binding import Harness as IrHarness
from test_pump_session_binding import ContextHarness as PumpHarness
from test_pump_demo_core import refresh
from test_session_context import ORIGIN


CASES = {
    'live_nav_stop_gateway': (GatewayHarness, LiveNavStopGateway, 'enable_live_output'),
    'live_alignment_takeover': (AlignmentHarness, LiveAlignmentTakeover, 'enable_live_alignment'),
    'ir_approach_live': (IrHarness, IrApproachLiveNode, 'enable_live_ir_jog'),
    'pump_demo_controller': (PumpHarness, PumpDemoController, 'enable_pump_demo'),
}


def make(monkeypatch, owner, *, selected=True, report_only=False):
    node = CASES[owner][0](monkeypatch)
    context = node._session_context
    if report_only:
        node.core.enabled = False
        node._session_context = None
        if owner == 'pump_demo_controller':
            node._pump_release = None
        else:
            node._motion_release = None
        context = SessionContextGuard()
    node._close_owner_context = context if selected else None
    node._close_owner_reporter = OwnerCloseReporter(owner) if selected else None
    node._close_owner_live_enabled = node.core.enabled if selected else None
    return node


def state_pub(node, owner):
    if owner == 'pump_demo_controller':
        return node.state_pub
    if owner == 'ir_approach_live':
        return node.state_publisher
    return node._state_publisher


def owned(node, owner):
    return (node.pump_pub if owner == 'pump_demo_controller' else node._raw_publisher) is not None


def feed(node, owner, *, phase='STOPPING', generation=1):
    if owner == 'pump_demo_controller':
        refresh(node.core, node.now)
        node.mission(phase=phase, generation=generation)
    else:
        node.feed(phase=phase, generation=generation)


def close(node, owner):
    if owner != 'pump_demo_controller':
        node.close()
    else:
        node.now = 1.93
        feed(node, owner)
        node._tick()
        assert node.pump_pub is not None
        node.now = 2.03
        feed(node, owner)
        node.receive(seq=2)
        node._tick()


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
def test_disabled_owner_uses_real_callback_context_and_never_acquires_to_ack_close(monkeypatch, owner):
    node = make(monkeypatch, owner, report_only=True)
    feed(node, owner)
    node._tick()
    report = state_pub(node, owner).messages[-1]['close_owner_observation']
    assert not owned(node, owner) and not node.created
    assert node._session_context is None and not node.core.enabled
    assert report['publisher_owned'] is False and report['owner'] == owner
    assert report['actuation_authorized'] is False
    assert report['live_enabled'] is False
    ack = report['close_ack']
    assert ack['kind'] == 'NO_OWNER_AT_CLOSE' and ack['close_generation'] == 1
    assert ack['session_id'] == node._close_owner_context.current(node.now)['session_id']


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
def test_actual_acquire_and_release_publish_current_reference_after_true_destroy(monkeypatch, owner):
    node = make(monkeypatch, owner)
    node.start()
    pub = state_pub(node, owner)
    before = pub.messages[-1]['close_owner_observation']
    assert before['live_enabled'] is True
    assert owned(node, owner) and before['publisher_owned'] is True
    assert before['close_ack'] is None
    close(node, owner)
    after = pub.messages[-1]['close_owner_observation']
    assert not owned(node, owner) and after['publisher_owned'] is False
    assert after['close_ack']['kind'] == 'NO_OWNER_AT_CLOSE'
    assert after['close_ack']['close_generation'] == 1
    assert after['source']['instance_id'] == before['source']['instance_id']
    assert after['source']['sequence'] > before['source']['sequence']
    assert node.events[-3:] == (['OFF', 'destroy', 'state'] if owner == 'pump_demo_controller'
                               else ['command', 'destroy', 'state'])
    lease = node._pump_release if owner == 'pump_demo_controller' else node._motion_release
    receipt = deepcopy(lease.receipt)
    assert receipt is not None
    node.now += 0.1
    feed(node, owner)
    node._tick()
    assert lease.receipt == receipt
    assert len(node.created) == 1


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
@pytest.mark.parametrize('result', [False, None, RuntimeError('destroy failed')])
def test_destroy_failure_cannot_emit_unowned_ack_or_success_state(monkeypatch, owner, result):
    node = make(monkeypatch, owner)
    node.start()
    first_index = len(state_pub(node, owner).messages)
    node.destroy_result = result
    with pytest.raises(RuntimeError):
        close(node, owner)
    assert owned(node, owner)
    assert node.events[-1] == 'destroy'
    for state in state_pub(node, owner).messages[first_index:]:
        report = state['close_owner_observation']
        assert report['publisher_owned'] is True and report['close_ack'] is None
    lease = node._pump_release if owner == 'pump_demo_controller' else node._motion_release
    assert lease.receipt is None


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
def test_slow_actual_destroy_does_not_redate_report_or_renew_close_context(monkeypatch, owner):
    node = make(monkeypatch, owner)
    node.start()
    destroy = node.destroy_publisher

    def slow(publisher):
        result = destroy(publisher)
        node.now += 0.4
        return result

    monkeypatch.setattr(node, 'destroy_publisher', slow)
    close(node, owner)
    report = state_pub(node, owner).messages[-1]['close_owner_observation']
    assert not owned(node, owner) and report['publisher_owned'] is False
    stamp = report['source']['stamp']
    stamp_ns = stamp['sec'] * 1_000_000_000 + stamp['nanosec']
    assert ORIGIN + round(node.now * 1e9) - stamp_ns >= 399_000_000
    assert report['close_ack'] is None


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
def test_source_stamp_precedes_graph_compute_and_context_is_checked_after_work(monkeypatch, owner):
    node = make(monkeypatch, owner, report_only=True)
    feed(node, owner)
    at_start = node.now
    method = 'evaluate' if owner == 'live_nav_stop_gateway' else 'step'
    evaluate = getattr(node.core, method)

    def slow(*args, **kwargs):
        result = evaluate(*args, **kwargs)
        node.now += 0.4
        return result

    monkeypatch.setattr(node.core, method, slow)
    node._tick()
    report = state_pub(node, owner).messages[-1]['close_owner_observation']
    sec, ns = divmod(ORIGIN + round(at_start * 1e9), 1_000_000_000)
    assert report['source']['stamp'] == {'sec': sec, 'nanosec': ns}
    assert report['close_ack'] is None and not node.created


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
def test_report_only_wrong_mission_endpoint_cannot_ack_and_recovery_does_not_enable_live(monkeypatch, owner):
    node = make(monkeypatch, owner, report_only=True)
    if owner == 'live_nav_stop_gateway':
        original = node.sources['/fire_mission/state']
        node.sources['/fire_mission/state'] = []
        restore = lambda: node.sources.update({'/fire_mission/state': original})
    elif owner == 'live_alignment_takeover':
        original = node._sources
        monkeypatch.setattr(node, '_sources', lambda topic: [] if topic == '/fire_mission/state' else original(topic))
        restore = lambda: monkeypatch.setattr(node, '_sources', original)
    elif owner == 'ir_approach_live':
        node.mission_source_ok = False
        restore = lambda: setattr(node, 'mission_source_ok', True)
    else:
        node.mission_endpoint_ok = False
        restore = lambda: setattr(node, 'mission_endpoint_ok', True)
    feed(node, owner)
    node._tick()
    assert state_pub(node, owner).messages[-1]['close_owner_observation']['close_ack'] is None
    restore()
    node.now += 0.1
    feed(node, owner)
    node._tick()
    assert state_pub(node, owner).messages[-1]['close_owner_observation']['close_ack'] is not None
    assert not node.core.enabled and not node.created


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
def test_disabled_report_flag_omits_new_fields_and_additional_tick_clock(monkeypatch, owner):
    node = make(monkeypatch, owner, selected=False)
    calls = []
    clock = node.get_clock
    monkeypatch.setattr(node, 'get_clock', lambda: (calls.append('clock'), clock())[1])
    if owner == 'pump_demo_controller':
        refresh(node.core, node.now)
        node.receive()
        node.mission()
    else:
        node.feed()
    calls.clear()
    node._tick()
    assert calls == []
    assert 'close_owner_observation' not in state_pub(node, owner).messages[-1]
    assert node._close_owner_reporter is node._close_owner_context is None


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
@pytest.mark.parametrize('mode', ['off', 'report_only', 'bad_type', 'live_legacy', 'live_bound'])
def test_actual_constructors_keep_observation_separate_from_actuation_and_validate_mode(monkeypatch, owner, mode):
    cls, live_flag = CASES[owner][1:]
    params, subscriptions, timers, publishers = {}, [], [], []
    overrides = {}
    if mode != 'off':
        overrides['enable_close_owner_observation'] = 'false' if mode == 'bad_type' else True
    if mode in ('live_legacy', 'live_bound'):
        overrides[live_flag] = True
    if mode == 'live_bound':
        overrides['require_fire_session_context'] = True
        if owner == 'ir_approach_live':
            overrides['integrated_handoff'] = True
        if owner == 'pump_demo_controller':
            overrides['release_pump_on_terminal'] = True
    # Actual constructors with all ROS effects stubbed, as in existing owner tests.
    from rclpy.node import Node
    monkeypatch.setattr(Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(cls, 'resolve_topic_name', lambda self, name: name)
    monkeypatch.setattr(cls, 'declare_parameter',
                        lambda self, name, value: params.setdefault(name, overrides.get(name, value)))
    monkeypatch.setattr(cls, 'get_parameter', lambda self, name: SimpleNamespace(value=params[name]))
    monkeypatch.setattr(cls, 'create_publisher',
                        lambda self, kind, topic, qos: (publishers.append(topic), object())[1])
    monkeypatch.setattr(cls, 'create_subscription',
                        lambda self, kind, topic, callback, qos: subscriptions.append(topic))
    monkeypatch.setattr(cls, 'create_timer', lambda self, period, callback: timers.append(period))
    monkeypatch.setattr(cls, 'get_logger',
                        lambda self: SimpleNamespace(warning=lambda msg: None))
    if mode in ('bad_type', 'live_legacy'):
        with pytest.raises(ValueError):
            cls()
        assert '/cmd_vel_raw' not in publishers and '/pump_cmd' not in publishers
        return
    node = cls()
    selected = mode != 'off'
    assert (node._close_owner_reporter is not None) is selected
    assert node._close_owner_live_enabled is (node.core.enabled if selected else None)
    assert len(timers) == 1
    assert '/cmd_vel_raw' not in publishers and '/pump_cmd' not in publishers
    if mode == 'report_only':
        assert node.core.enabled is False and node._session_context is None
        assert node._close_owner_context is not None
    elif mode == 'live_bound':
        assert node._close_owner_context is node._session_context
    else:
        assert node._close_owner_context is None
    expected_mission = int(selected or owner != 'ir_approach_live')
    assert subscriptions.count('/fire_mission/state') == expected_mission
