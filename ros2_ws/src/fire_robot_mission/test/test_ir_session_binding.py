"""Real IR callbacks/counts/core/effects; no ROS init or hardware access."""

from copy import deepcopy
import json
from types import SimpleNamespace

import pytest
from rclpy.qos import ReliabilityPolicy

from fire_robot_mission.ir_approach_live import IrApproachLiveNode
from fire_robot_mission.ir_approach_live_core import VerifiedEndpointPins
from fire_robot_mission.motion_release import MotionReleaseProof
from fire_robot_mission.session_context import SessionContextGuard
from test_ir_approach_live_core import (
    centered_ir, continuous_core, endpoint, odom, scan,
)
from test_ir_approach_live_handoff import UUID, ready
from test_session_context import INSTANCE, ORIGIN, report


BE = ReliabilityPolicy.BEST_EFFORT
RELIABLE = ReliabilityPolicy.RELIABLE


class Publisher:
    def __init__(self, node, *, state=False):
        self.node, self.state = node, state
        self.messages = []

    def publish(self, msg):
        self.node.events.append('state' if self.state else 'command')
        self.messages.append(json.loads(msg.data) if self.state else msg)


class Harness:
    _tick = IrApproachLiveNode._tick
    _counts = IrApproachLiveNode._counts
    _on_proof = IrApproachLiveNode._on_proof
    _proof = IrApproachLiveNode._proof
    _handoff_evidence = IrApproachLiveNode._handoff_evidence
    _guard_owned_session = IrApproachLiveNode._guard_owned_session
    _release_evidence = IrApproachLiveNode._release_evidence
    _exact_endpoint = staticmethod(IrApproachLiveNode._exact_endpoint)

    def __init__(self, monkeypatch, *, feature=True):
        from fire_robot_mission import ir_approach_live as module
        self.now = 10.0
        monkeypatch.setattr(module.time, 'monotonic', lambda: self.now)
        self.core = continuous_core()
        self.core.update_arm(False, self.now - 0.1)
        self.core.update_arm(True, self.now)
        self._session_context = SessionContextGuard() if feature else None
        self._motion_release = MotionReleaseProof(owner='ir_approach_live') if feature else None
        self._raw_publisher = None
        self._endpoint_pins = VerifiedEndpointPins()
        self._proofs, self.events, self.created = {}, [], []
        self.state_publisher = Publisher(self, state=True)
        self._statuses = ((UUID, 5),)
        self.visible = False
        self.mission_source_ok = self.odom_source_ok = True
        self.destroy_result = True
        self.clock_reads = self.graph_calls = self.seq = 0
        self.query_delay = 0.0
        self.x = 0.0

    def get_clock(self):
        self.clock_reads += 1
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=ORIGIN + round(self.now * 1e9)))

    def get_publishers_info_by_topic(self, topic):
        self.graph_calls += 1
        self.now += self.query_delay
        if topic == '/pump_cmd':
            return []
        if topic == '/cmd_vel_raw':
            return ([endpoint(gid=20, name='ir_approach_live', reliability=BE)]
                    if self.visible and self._raw_publisher is not None else [])
        if topic == '/fire_mission/nav2_cmd_vel_shadow':
            return [endpoint(gid=21, name='velocity_smoother', reliability=RELIABLE),
                    endpoint(gid=22, name='behavior_server', reliability=RELIABLE),
                    endpoint(gid=23, name='behavior_server', reliability=RELIABLE)]
        expected = {
            '/fire_mission/ir_range_preview': ('ir_range_preview', 'std_msgs/msg/String', BE),
            '/scan': ('scan_qos_relay', 'sensor_msgs/msg/LaserScan', BE),
            '/odom': ('serial_bridge_node' if self.odom_source_ok else 'foreign',
                      'nav_msgs/msg/Odometry', BE),
            '/cmd_vel': ('safety_watchdog', 'geometry_msgs/msg/Twist', BE),
            '/fire_mission/state': ('fire_mission_supervisor' if self.mission_source_ok else 'foreign',
                                    'std_msgs/msg/String', BE),
            '/fire_mission/live_alignment_state': ('live_alignment_takeover', 'std_msgs/msg/String', BE),
            '/fire_mission/live_nav_stop_state': ('live_nav_stop_gateway', 'std_msgs/msg/String', BE),
            '/fire_mission/nav2_cancel_guard_state': ('nav2_cancel_guard', 'std_msgs/msg/String', BE),
            '/navigate_to_pose/_action/status': ('bt_navigator', 'action_msgs/msg/GoalStatusArray', RELIABLE),
            '/fire_mission/post_spray_review': ('post_spray_review', 'std_msgs/msg/String', BE),
            '/fire_sensors/actuator_state': ('serial_bridge_node', 'std_msgs/msg/String', BE),
        }
        name, message_type, reliability = expected[topic]
        return [endpoint(name=name, message_type=message_type, reliability=reliability)]

    def get_subscriptions_info_by_topic(self, topic):
        assert topic == '/cmd_vel_raw'
        self.graph_calls += 1
        self.now += self.query_delay
        return [endpoint(gid=30, name='safety_watchdog', reliability=BE),
                endpoint(gid=31, name='fire_mission_supervisor', reliability=BE)]

    def count_publishers(self, topic):
        return 0 if topic in ('/pump_cmd', '/cmd_vel_nav') else 1

    def create_publisher(self, _type, topic, _qos):
        assert topic == '/cmd_vel_raw'
        self.events.append('create')
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
        return SimpleNamespace(warning=lambda msg: None)

    def feed(self, *, with_mission=True, phase='ACTIVE', generation=None,
             session=1, goal=UUID, old_ms=10, instance=INSTANCE):
        self.core.update('ir', centered_ir(), self.now)
        self.core.update('scan', scan(), self.now)
        # Effective odom time already includes header source age via admit_odom.
        self.core.update('odom', odom(self.x), self.now - 0.01)
        self.core.update('cmd', (0.0,) * 6, self.now)
        self.core.update_arm(True, self.now)
        payloads = {name: deepcopy(payload) for name, (payload, _) in ready()[0]._proofs.items()}
        if self.visible and self._raw_publisher is not None:
            payloads['gateway'].update(state='RELEASED_STOPPED', handoff_ready=False)
            payloads['guard'].update(state='IR_APPROACH_IN_PROGRESS', raw_cmd_publishers=1,
                                     ir_approach_handoff_active=True)
        for name, payload in payloads.items():
            if name == 'mission':
                if not with_mission:
                    continue
                self.seq += 1
                payload.update(report(
                    ORIGIN + round(self.now * 1e9) - round(old_ms * 1e6), self.seq,
                    phase=phase, generation=generation, session=session, goal=goal,
                    instance=instance))
            self._on_proof(name, SimpleNamespace(data=json.dumps(payload)))

    def start(self, **context):
        self.feed(**context)
        self._tick()
        assert self._raw_publisher is None
        self.now = 10.31
        self.feed(**context)
        self._tick()
        assert self._raw_publisher is not None
        assert self._raw_publisher.messages[-1].linear.x == 0.0
        self.visible = True
        self.now = 10.36
        self.feed(**context)
        self._tick()
        assert self._raw_publisher.messages[-1].linear.x == 0.08

    def close(self, *, final=True):
        for now in ((10.5, 10.8, 11.05) if final else (10.5, 10.8)):
            self.now = now
            self.feed(phase='STOPPING', generation=1)
            self._tick()


def test_actual_binding_stop_hold_destroy_receipt_and_budget_commit(monkeypatch):
    node = Harness(monkeypatch)
    node.core.total_progress_m = 0.4
    node.start()
    node.x = 0.02
    raw = node._raw_publisher
    node.close()
    assert node._raw_publisher is None
    assert node.events[-3:] == ['command', 'destroy', 'state']
    assert raw.messages[-1].linear.x == raw.messages[-1].angular.z == 0.0
    event = node._motion_release.receipt
    assert event['owner'] == 'ir_approach_live'
    assert event['session_id'] == f'fire:{INSTANCE}:1'
    assert event['nav_goal_uuid'] == UUID and event['close_generation'] == 1
    assert event['physical_motion_verified'] is False
    assert node.core.phase == 'ABORTED'
    assert node.core.step_count == 1
    assert node.core.total_progress_m == pytest.approx(0.42)
    assert node.state_publisher.messages[-1]['session_owner']['publisher_owned'] is False
    assert node.state_publisher.messages[-1]['session_owner']['receipt'] == event


@pytest.mark.parametrize('result', [False, None, RuntimeError('destroy failed')])
def test_failed_destroy_does_not_commit_budget_or_receipt(monkeypatch, result):
    node = Harness(monkeypatch)
    node.start()
    node.x = 0.02
    node.destroy_result = result
    raw = node._raw_publisher
    with pytest.raises(RuntimeError):
        node.close()
    assert node._raw_publisher is raw
    assert raw.messages[-1].linear.x == 0.0
    assert node._motion_release.receipt is None
    assert node.core.phase == 'STOPPING'
    assert node.core.step_count == 0 and node.core.total_progress_m == 0.0
    assert node.core.step_progress_m == pytest.approx(0.02)
    assert node.events[-1] == 'destroy'


@pytest.mark.parametrize('changes', [
    {'old_ms': 400}, {'phase': 'IDLE', 'goal': None}, {'goal': None},
    {'goal': 'ef' * 16}, {'phase': 'STOPPING', 'generation': 1},
])
def test_missing_stale_or_wrong_goal_context_never_acquires(monkeypatch, changes):
    node = Harness(monkeypatch)
    for now in (10.0, 10.31, 10.6):
        node.now = now
        node.feed(**changes)
        node._tick()
    assert not node.created


@pytest.mark.parametrize('session', [2, 57])
def test_first_acquisition_accepts_the_current_session_without_assuming_one(monkeypatch, session):
    node = Harness(monkeypatch)
    node.start(session=session)
    binding = node._motion_release.binding
    assert binding.session_id == f'fire:{INSTANCE}:{session}'
    assert binding.nav_goal_uuid == UUID
    assert len(node.created) == 1
    assert node.core.phase == 'MOVING'
    assert node._raw_publisher.messages[-1].linear.x == 0.08


@pytest.mark.parametrize('fault', ['source', 'goal', 'instance', 'session'])
def test_bad_then_good_callback_before_tick_cannot_resume_motion(monkeypatch, fault):
    node = Harness(monkeypatch)
    node.start()
    binding = node._motion_release.binding
    node.now = 10.4
    node.mission_source_ok = fault != 'source'
    changes = {'goal': 'ef' * 16} if fault == 'goal' else (
        {'instance': 'ef' * 16} if fault == 'instance' else (
            {'session': 2} if fault == 'session' else {}))
    node.feed(**changes)
    node.now = 10.41
    node.mission_source_ok = True
    node.feed()
    node._tick()
    assert node.core.phase == 'STOPPING'
    assert node._raw_publisher.messages[-1].linear.x == 0.0
    assert node._motion_release.binding == binding
    assert node._motion_release.stop_latched


def test_stale_session_cache_cannot_be_hidden_by_fresh_legacy_proofs(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.now = 10.8
    node.feed(with_mission=False)
    payload, _ = node._proofs['mission']
    node._proofs['mission'] = (payload, node.now)
    node._tick()
    assert node.core.phase == 'STOPPING'
    assert node._raw_publisher.messages[-1].linear.x == 0.0


def test_malformed_then_good_mission_cannot_hide_owned_context_break(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.now = 10.4
    node._on_proof('mission', SimpleNamespace(data='{broken'))
    node.now = 10.41
    node.feed()
    node._tick()
    assert node.core.phase == 'STOPPING'
    assert node._raw_publisher.messages[-1].linear.x == 0.0


def test_foreign_raw_owner_prevents_motion_and_release(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    get_infos = node.get_publishers_info_by_topic

    def conflict(topic):
        infos = get_infos(topic)
        return (infos + [endpoint(gid=99, name='foreign', reliability=BE)]
                if topic == '/cmd_vel_raw' else infos)

    monkeypatch.setattr(node, 'get_publishers_info_by_topic', conflict)
    for now in (10.5, 10.8, 11.1):
        node.now = now
        node.feed()
        node._tick()
    assert node.core.phase == 'STOPPING'
    assert node._raw_publisher.messages[-1].linear.x == 0.0
    assert node._motion_release.receipt is None
    assert 'destroy' not in node.events


def test_close_duplicate_keeps_first_generation_and_restarts_hold_once(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.now = 10.4
    node.core.phase = 'STOPPING'
    node.core.stop_reason = 'SPRAY_CANDIDATE_STOP'
    node.core.stopping_at = 10.4
    node.core.zero_since = 10.0
    node._motion_release.command_published(0.0, 10.4)
    node.close(final=False)
    lease = node._motion_release
    assert lease.zero_published_at == lease.close_received_at == 10.5
    source = deepcopy(lease.close_source)
    assert node.core.stop_reason == 'SESSION_CLOSE_REQUESTED'
    node.now = 10.9
    node.feed(phase='STOPPING', generation=2)
    node._tick()
    assert lease.close_generation == 1 and lease.close_source == source
    assert lease.receipt is None
    assert node.core.zero_since == 10.5


def test_slow_graph_queries_are_included_in_input_and_proof_age(monkeypatch):
    node = Harness(monkeypatch)
    node.feed()
    node.query_delay = 0.03
    node._tick()
    assert node.now > 10.35
    assert not node.created and node.core.admission_since is None


def test_slow_core_cannot_create_publisher_from_an_expired_binding(monkeypatch):
    node = Harness(monkeypatch)
    node.feed()
    node._tick()
    node.now = 10.31
    node.feed()
    step = node.core.step

    def slow(*args, **kwargs):
        result = step(*args, **kwargs)
        node.now += 0.4
        return result

    monkeypatch.setattr(node.core, 'step', slow)
    with pytest.raises(RuntimeError, match='IR_SESSION_ACQUIRE_UNCONFIRMED'):
        node._tick()
    assert node._raw_publisher is None and not node.created
    assert node._motion_release.binding is None
    assert node._motion_release.receipt is None


@pytest.mark.parametrize('stage', ['motion', 'release'])
def test_slow_core_cannot_publish_stale_motion_or_release(monkeypatch, stage):
    node = Harness(monkeypatch)
    node.start()
    if stage == 'release':
        node.close(final=False)
        node.now = 11.05
        node.feed(phase='STOPPING', generation=1)
    else:
        node.now = 10.4
        node.feed()
    step = node.core.step

    def slow(*args, **kwargs):
        result = step(*args, **kwargs)
        node.now += 0.4
        return result

    monkeypatch.setattr(node.core, 'step', slow)
    node._tick()
    assert node._raw_publisher is not None
    assert node._raw_publisher.messages[-1].linear.x == 0.0
    assert node.state_publisher.messages[-1]['command']['linear_x_m_s'] == 0.0
    assert node.core.phase == 'STOPPING'
    assert node._motion_release.receipt is None
    assert 'destroy' not in node.events


def test_wrong_odom_source_stops_and_invalidates_release_evidence(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.odom_source_ok = False
    for now in (10.5, 10.8, 11.1):
        node.now = now
        node.feed(phase='STOPPING', generation=1)
        node._tick()
    assert node.core.phase == 'STOPPING'
    assert node.core.inputs['odom'][0] is None
    assert node._raw_publisher.messages[-1].linear.x == 0.0
    assert node._motion_release.receipt is None


def test_released_owner_cannot_rebind_reset_budget_or_retag_receipt(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.x = 0.02
    node.close()
    event = deepcopy(node._motion_release.receipt)
    budget = (node.core.step_count, node.core.total_progress_m)
    for now in (11.1, 11.5, 11.9):
        node.now = now
        node.feed(session=2)
        node._proofs['mission'][0]['state'] = 'IDLE'
        node._proofs['mission'][0]['lifecycle']['pre_spray_clear_goal_uuid'] = UUID
        node._tick()
    assert len(node.created) == 1
    assert node._motion_release.receipt == event
    assert (node.core.step_count, node.core.total_progress_m) == budget
    assert node.core.goal_uuid == UUID


def test_normal_release_receipt_is_not_retagged_by_a_later_close(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.x = 0.03
    node.core.phase, node.core.stop_reason = 'STOPPING', 'APPROACH_DISTANCE_LIMIT'
    for now in (10.5, 10.8, 11.05):
        node.now = now
        node.feed()
        node._tick()
    assert node.core.phase == 'COMPLETE'
    event = deepcopy(node._motion_release.receipt)
    assert event['close_generation'] is None
    node.now = 11.1
    node.feed(phase='STOPPING', generation=1)
    node._tick()
    assert node._motion_release.receipt == event
    assert node._motion_release.close_generation is None
    assert len(node.created) == 1


def test_spray_range_without_acquisition_does_not_invent_owner_receipt(monkeypatch):
    node = Harness(monkeypatch)
    for now in (10.0, 10.31, 10.6):
        node.now = now
        node.feed()
        node.core.update('ir', centered_ir('SPRAY_RANGE'), now)
        node._tick()
    assert not node.created
    assert node._motion_release.binding is node._motion_release.receipt is None


@pytest.mark.parametrize('moving_sample', ['odom', 'cmd'])
def test_nonstationary_or_nonzero_sample_breaks_release_hold(monkeypatch, moving_sample):
    node = Harness(monkeypatch)
    node.start()
    node.close(final=False)
    node.now = 11.05
    node.feed(phase='STOPPING', generation=1)
    sample = odom(vx=0.03) if moving_sample == 'odom' else (0.02, 0, 0, 0, 0, 0)
    node.core.update(moving_sample, sample, node.now)
    node._tick()
    assert node.core.zero_since is None
    assert node._raw_publisher.messages[-1].linear.x == 0.0
    assert node._motion_release.receipt is None
    assert 'destroy' not in node.events


def test_feature_off_keeps_legacy_flow_without_context_clock_or_receipt(monkeypatch):
    node = Harness(monkeypatch, feature=False)
    node.start()
    assert node.clock_reads == 0
    assert 'session_owner' not in node.state_publisher.messages[-1]
    assert node._motion_release is node._session_context is None


def test_complete_release_commit_is_single_use_and_keeps_normal_budget():
    core = continuous_core()
    core.phase, core.stop_reason = 'STOPPING', 'STEP_DISTANCE_REACHED'
    core.step_progress_m, core.total_progress_m = 0.05, 0.2
    core.publisher_released()
    assert core.phase == 'COMPLETE'
    assert core.step_count == 1 and core.total_progress_m == 0.25
    with pytest.raises(RuntimeError):
        core.publisher_released()
    assert core.total_progress_m == 0.25


def test_migration_commit_accounts_aborted_distance_without_resetting_budget():
    core = continuous_core()
    core.phase, core.stop_reason = 'STOPPING', 'APPROACH_TOTAL_DISTANCE_LIMIT'
    core.step_progress_m, core.total_progress_m = 0.3, 1.8
    core.publisher_released(account_abort=True)
    assert core.phase == 'ABORTED'
    assert core.step_count == 1
    assert core.total_progress_m == pytest.approx(2.1)


@pytest.mark.parametrize('bad', [False, None, True, 'other_motion_owner'])
def test_motion_receipt_owner_is_not_free_form(bad):
    with pytest.raises(ValueError):
        MotionReleaseProof(owner=bad)


@pytest.mark.parametrize('overrides,valid,helper', [
    ({}, True, False),
    ({'require_fire_session_context': 'false'}, False, False),
    ({'require_fire_session_context': True}, False, False),
    ({'require_fire_session_context': True, 'integrated_handoff': True}, True, False),
    ({'require_fire_session_context': True, 'integrated_handoff': True,
      'enable_live_ir_jog': True}, True, True),
    ({'require_fire_session_context': True, 'integrated_handoff': True,
      'continuous_approach': True, 'max_total_approach_m': 0.0}, False, False),
])
def test_actual_constructor_validates_opt_in_and_keeps_off_helpers_absent(
        monkeypatch, overrides, valid, helper):
    from fire_robot_mission import ir_approach_live as module
    params = {}
    monkeypatch.setattr(module.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(IrApproachLiveNode, 'resolve_topic_name', lambda self, name: name)
    monkeypatch.setattr(IrApproachLiveNode, 'declare_parameter',
                        lambda self, name, value: params.setdefault(name, overrides.get(name, value)))
    monkeypatch.setattr(IrApproachLiveNode, 'get_parameter',
                        lambda self, name: SimpleNamespace(value=params[name]))
    monkeypatch.setattr(IrApproachLiveNode, 'create_publisher', lambda *args: object())
    monkeypatch.setattr(IrApproachLiveNode, 'create_subscription', lambda *args: None)
    monkeypatch.setattr(IrApproachLiveNode, 'create_timer', lambda *args: None)
    monkeypatch.setattr(IrApproachLiveNode, 'get_logger',
                        lambda self: SimpleNamespace(warning=lambda msg: None))
    if not valid:
        with pytest.raises(ValueError):
            IrApproachLiveNode()
        return
    node = IrApproachLiveNode()
    assert (node._session_context is not None) is helper
    assert (node._motion_release is not None) is helper
