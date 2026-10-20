"""Real supervisor callbacks/publication/services, fake ROS effects, no hardware."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from rclpy.node import Node

from fire_robot_mission.close_owner_evidence import CloseOwnerEvidenceCollector
from fire_robot_mission.global_stop_evidence import GlobalStopEvidence
from fire_robot_mission.fire_session_core import FireSessionCore
from fire_robot_mission.mission_supervisor import FireMissionSupervisor, CLOSE_OWNER_TOPICS
from fire_robot_mission.runtime_manifest import runtime_manifest
from fire_robot_mission.shadow_budget import ShadowBudgetStore
from fire_robot_mission.shadow_session_runtime import ShadowSessionRuntime
from test_global_stop_node import Harness as GlobalHarness
from test_global_stop_evidence import GOAL, actuator
from test_supervisor_close_owner_collection import ros


class Harness(GlobalHarness):
    _shadow_current_stop_result = FireMissionSupervisor._shadow_current_stop_result
    _shadow_operator_request = FireMissionSupervisor._shadow_operator_request
    _recover_shadow = FireMissionSupervisor._recover_shadow
    _reset_shadow = FireMissionSupervisor._reset_shadow

    def __init__(self, monkeypatch, tmp_path, *, goal=GOAL, manifest='all_four_external_v1'):
        super().__init__(monkeypatch, goal=goal)
        self.manifest = runtime_manifest(manifest)
        self._close_owner_collector = CloseOwnerEvidenceCollector(manifest_id=manifest)
        self._global_stop = GlobalStopEvidence(manifest_id=manifest, require_recovery_on_change=True)
        store = ShadowBudgetStore(str(tmp_path / 'budget.json'), initialize=True)
        self._shadow_runtime = ShadowSessionRuntime(store, self.manifest)
        self._core._session_runtime = self._shadow_runtime
        identity = self._core._fire_session.identity
        store.reserve_session(identity.session_id, goal)
        self.node_names = [(owner, '/') for owner in self.manifest.required]
        for owner in self.manifest.omitted:
            self.infos[CLOSE_OWNER_TOPICS[owner]] = []

    def get_node_names_and_namespaces(self):
        return self.node_names

    def payload(self, owner, **kwargs):
        value = super().payload(owner, **kwargs)
        value['close_owner_observation']['live_enabled'] = False
        return value

    def tick(self, now, *, records=None, epoch=1):
        self.advance(now - self.now)
        self.seq += 1
        self.motion()
        self.statuses(records)
        self._on_actuator(SimpleNamespace(data=json.dumps(actuator(self.now, self.seq, epoch=epoch))))
        for owner in self.manifest.required:
            self.receive(owner)
        self._publish_state()


@pytest.fixture
def node(monkeypatch, tmp_path):
    result = Harness(monkeypatch, tmp_path)
    yield result
    result._shadow_runtime.store.close()


def close_round(node, start=10):
    node._publish_state()
    for delta in (.1, .2, .3, .4, .5, .61):
        node.tick(start + delta)


@pytest.mark.parametrize('goal', [GOAL, None])
def test_actual_publish_finishes_shadow_session_then_next_identity_needs_new_proof(monkeypatch, tmp_path, goal):
    node = Harness(monkeypatch, tmp_path, goal=goal)
    try:
        session = node._core._fire_session
        first = session.identity
        close_round(node)
        assert session.phase == 'IDLE'
        assert node._close_owner_collector.request is node._global_stop.request is None
        assert node._shadow_runtime.closed_count == 1
        assert node.messages[-1]['lifecycle']['fire_session_observer']['phase'] == 'STOPPING'
        # The final proof follows publication; IDLE appears in the NEXT heartbeat.
        node.advance(.01)
        node._publish_state()
        assert node.messages[-1]['lifecycle']['fire_session_observer']['phase'] == 'IDLE'
        assert not node.messages[-1]['lifecycle']['shadow_session_runtime']['actuation_authorized']
        identity = node._shadow_runtime.try_begin(session, goal, node.now)
        assert identity.session_id != first.session_id
        session.request_close(identity.session_id, 'OPERATOR_CANCEL', node.now)
        node._publish_state()
        assert session.phase == 'STOPPING'
        assert not node._global_stop_result.ready
        assert node._shadow_runtime.store.state['used']['sessions'] == 2
        close_round(node, start=node.now)
        assert session.phase == 'IDLE'
    finally:
        node._shadow_runtime.store.close()


def test_failed_publish_cannot_start_or_finish_close_round(node):
    node.fail_publish = True
    with pytest.raises(RuntimeError, match='publish failed'):
        node._publish_state()
    assert node._global_stop.request is None
    assert node._core._fire_session.phase == 'STOPPING'
    assert node._shadow_runtime.store.state['pending']['completion_marked'] is False


def test_slow_final_publish_expires_old_proof_before_cleanup(node):
    node._publish_state()
    for now in (10.1, 10.2, 10.3, 10.4, 10.5):
        node.tick(now)
    node.publish_delay = .4
    node.tick(10.61)
    assert node._global_stop_result.ready  # pre-publish observation
    assert node._core._fire_session.phase == 'STOPPING'
    assert node._shadow_runtime.closed_count == 0
    assert node._shadow_runtime.store.state['pending']['completion_marked'] is False


@pytest.mark.parametrize('manifest', ['wireless_static_observation_v1', 'io_nav_static_no_owners_v1'])
def test_explicit_omitted_manifest_requires_node_and_endpoint_absence(monkeypatch, tmp_path, manifest):
    node = Harness(monkeypatch, tmp_path, manifest=manifest)
    try:
        close_round(node)
        assert node._core._fire_session.phase == 'IDLE'
        assert node._shadow_runtime.closed_count == 1
        assert node.reports()['required_owners'] == list(node.manifest.required)
        assert all(node.infos[CLOSE_OWNER_TOPICS[owner]] == [] for owner in node.manifest.omitted)
    finally:
        node._shadow_runtime.store.close()


@pytest.mark.parametrize('presence', ['node', 'endpoint', 'raw', 'pump'])
def test_sparse_manifest_does_not_hide_present_role_or_command_publisher(monkeypatch, tmp_path, presence):
    node = Harness(monkeypatch, tmp_path, manifest='io_nav_static_no_owners_v1')
    try:
        if presence == 'node':
            node.node_names.append(('live_alignment_takeover', '/other'))
        elif presence == 'endpoint':
            node.infos[CLOSE_OWNER_TOPICS['live_alignment_takeover']] = [object()]
        elif presence == 'raw':
            node.raw_count = 1
        else:
            node.pump_count = 1
        close_round(node)
        assert node._core._fire_session.phase == 'STOPPING'
        assert node._shadow_runtime.closed_count == 0
    finally:
        node._shadow_runtime.store.close()


@pytest.mark.parametrize('live', [True, None, 0])
def test_live_or_unknown_owner_never_permits_shadow_cleanup(node, live):
    node._publish_state()
    node.advance(.1)
    owner = node.manifest.required[0]
    value = node.payload(owner)
    value['close_owner_observation']['live_enabled'] = live
    node.receive(owner, value)
    for now in (10.2, 10.3, 10.4, 10.5, 10.61, 10.7):
        node.tick(now)
    assert node._shadow_runtime.recovery_latch.startswith('SHADOW_OWNER_LIVE_OR_UNDECLARED')
    assert node._core._fire_session.phase == 'STOPPING'


def test_explicit_recovery_renews_round_then_waits_for_new_published_context(node):
    node._publish_state()
    old = node._core._fire_session.close_request
    node.advance(.1)
    reply = node._recover_shadow(None, SimpleNamespace())
    assert reply.success and node._global_stop.request is None
    request = node._core._fire_session.close_request
    assert request.generation > old.generation
    assert node._shadow_runtime.store.state['used']['sessions'] == 1
    node._publish_state()
    assert node._global_stop.request is request
    for delta in (.1, .2, .3, .4, .5, .61):
        node.tick(10.1 + delta)
    assert node._core._fire_session.phase == 'IDLE'


def test_actuator_epoch_fault_cannot_be_cleared_by_fresh_frames(node):
    node._publish_state()
    node.tick(10.1)
    node.tick(10.2, epoch=2)
    node.tick(10.3, epoch=2)
    assert node._shadow_runtime.recovery_latch is not None
    assert node._core._fire_session.phase == 'STOPPING'
    assert node._shadow_runtime.store.state['used']['sessions'] == 1


def test_budget_commit_failure_never_publishes_idle(node, monkeypatch):
    node._publish_state()
    for now in (10.1, 10.2, 10.3, 10.4, 10.5):
        node.tick(now)
    pending_id = node._shadow_runtime.store.state['pending']['session_id']
    def fail(session_id, *, reset_verified=False):
        assert session_id == pending_id
        assert reset_verified is False
        raise OSError('disk full')
    monkeypatch.setattr(node._shadow_runtime.store, 'finish_completion', fail)
    with pytest.raises(OSError, match='disk full'):
        node.tick(10.61)
    assert all(msg['lifecycle']['fire_session_observer']['phase'] != 'IDLE' for msg in node.messages)
    pending = node._shadow_runtime.store.state['pending']
    assert pending['session_id'] == pending_id and pending['completion_marked'] is True
    assert node._close_owner_collector.request is not None
    assert node._global_stop.request is not None
    assert node._shadow_runtime.closed_count == 0


def test_reset_service_does_not_reset_quota_until_fresh_absence_and_stop_proof(node):
    close_round(node)
    original = node._shadow_runtime.store.state['encounter_id']
    node.advance(.01)
    response = node._reset_shadow(None, SimpleNamespace())
    assert response.success
    node._publish_state()
    node.statuses = lambda records=None: GlobalHarness.statuses(node, [] if records is None else records)
    start = node.now
    for delta in (.1, .3, .5, .7, .9, 1.1, 1.3, 1.5, 1.7):
        # Same clock and source-age path as admitted detection updates; no fire.
        from test_mission_fire_session_observer import feed
        feed(node._core, start + delta, present=False)
        node.tick(start + delta)
    assert node._shadow_runtime.store.state['pending'] is None
    assert node._shadow_runtime.store.state['used']['sessions'] == 0
    assert node._shadow_runtime.store.state['encounter_id'] != original


@pytest.mark.parametrize('manifest', ['all_four_external_v1', 'wireless_static_observation_v1',
                                      'io_nav_static_no_owners_v1'])
@pytest.mark.parametrize('through', [False, True])
def test_actual_constructor_opt_in_creates_only_selected_subscriptions_and_two_services(monkeypatch, tmp_path, manifest, through):
    cls = FireMissionSupervisor
    params = {'use_sim_time': False}
    overrides = {'enable_fire_session_observer': True, 'enable_close_owner_collection': True,
                 'enable_global_stop_collection': True, 'enable_shadow_session_cleanup': True,
                 'close_owner_manifest': manifest, 'initialize_shadow_budget': True,
                 'require_through_status': through,
                 'shadow_budget_file': str(tmp_path / 'budget.json')}
    pubs, subs, services, timers = [], [], [], []
    monkeypatch.setattr(Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(cls, 'declare_parameter',
                        lambda self, name, value: params.setdefault(name, overrides.get(name, value)))
    monkeypatch.setattr(cls, 'get_parameter', lambda self, name: SimpleNamespace(value=params[name]))
    monkeypatch.setattr(cls, 'create_publisher', lambda self, kind, topic, qos: pubs.append(topic))
    monkeypatch.setattr(cls, 'create_subscription', lambda self, kind, topic, callback, qos:
                        (subs.append(topic), SimpleNamespace())[1])
    monkeypatch.setattr(cls, 'create_timer', lambda self, period, callback: timers.append(period))
    monkeypatch.setattr(cls, 'create_service', lambda self, kind, name, callback: services.append(name))
    monkeypatch.setattr(cls, 'get_logger', lambda self:
                        SimpleNamespace(warning=lambda text: None, info=lambda text: None))
    node = cls()
    try:
        assert pubs == ['/fire_mission/state'] and len(timers) == 1
        expected = 11 + len([owner for owner in runtime_manifest(manifest).required
                             if owner != 'pump_demo_controller'])
        assert len(subs) == expected + int(through)
        assert ('/navigate_through_poses/_action/status' in subs) is through
        assert services == ['/fire_mission/recover_shadow_session', '/fire_mission/reset_shadow_encounter']
        assert node._global_stop.require_recovery_on_change is True
    finally:
        node._shadow_runtime.store.close()


def test_packaged_defaults_keep_new_mode_off_and_no_state_file_is_chosen():
    root = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((root / 'config/mission_params.yaml').read_text())['fire_mission_supervisor']['ros__parameters']
    assert params['enable_shadow_session_cleanup'] is False
    assert params['initialize_shadow_budget'] is False
    assert params['shadow_budget_file'] == ''
    assert '<exec_depend>std_srvs</exec_depend>' in (root / 'package.xml').read_text()


@pytest.mark.parametrize('name', ['', 'auto', 'missing_pump', None, True])
def test_unapproved_manifest_is_not_derived_from_discovery(name):
    with pytest.raises(ValueError):
        runtime_manifest(name)


def test_no_owner_collector_still_requires_explicit_current_graph_absence():
    session = FireSessionCore(proof_timeout_s=.35)
    identity = session.begin(None)
    session.request_close(identity.session_id, 'OPERATOR_CANCEL', 10)
    collector = CloseOwnerEvidenceCollector(manifest_id='io_nav_static_no_owners_v1')
    collector.begin_close(session.close_request, announced_ros_ns=ros(10), announced_at=10)
    assert not collector.assess(ros_now_ns=ros(10.1), now=10.1).ready
    assert not collector.assess(ros_now_ns=ros(10.1), now=10.1, absence_observed_at=10).ready
    assert collector.assess(ros_now_ns=ros(10.1), now=10.1, absence_observed_at=10.05).ready
    assert not collector.assess(ros_now_ns=ros(10.5), now=10.5, absence_observed_at=10.05).ready


def test_cannot_retire_collector_from_arbitrary_or_unaccepted_evidence(node):
    node._publish_state()
    with pytest.raises(ValueError):
        node._close_owner_collector.retire_after_close(node._core._fire_session, None, node.now)
    with pytest.raises(ValueError):
        node._global_stop.retire_after_close(node._core._fire_session, None, node.now)
    assert node._global_stop.request is not None


def test_sparse_role_appearance_stays_blocked_after_it_disappears(monkeypatch, tmp_path):
    node = Harness(monkeypatch, tmp_path, manifest='io_nav_static_no_owners_v1')
    try:
        node.node_names.append(('pump_demo_controller', '/'))
        node._publish_state()
        node.tick(10.1)
        node.node_names.clear()
        for now in (10.2, 10.3, 10.4, 10.5, 10.61, 10.7):
            node.tick(now)
        assert node._shadow_runtime.recovery_latch == 'GLOBAL_OMITTED_OWNER_PRESENT'
        assert node._core._fire_session.phase == 'STOPPING'
    finally:
        node._shadow_runtime.store.close()


def test_telemetry_ticks_do_not_write_durable_state_before_actual_completion(node, monkeypatch):
    def unexpected(*args):
        raise AssertionError('no ledger I/O on telemetry/timer work')
    monkeypatch.setattr(node._shadow_runtime.store, '_write', unexpected)
    node._publish_state()
    for now in (10.1, 10.2, 10.3, 10.4, 10.5):
        node.tick(now)
    assert node._shadow_runtime.closed_count == 0


@pytest.mark.parametrize('overrides', [
    {'enable_shadow_session_cleanup': 'true'},
    {'enable_shadow_session_cleanup': True},
    {'enable_fire_session_observer': True, 'enable_close_owner_collection': True,
     'close_owner_manifest': 'io_nav_static_no_owners_v1'},
    {'enable_fire_session_observer': True, 'enable_close_owner_collection': True,
     'enable_shadow_session_cleanup': True, 'close_owner_manifest': 'auto'},
])
def test_invalid_cleanup_configuration_rejected_before_ros_effects(overrides):
    class Reader:
        _declare_parameters = FireMissionSupervisor._declare_parameters
        _read_parameters = FireMissionSupervisor._read_parameters
        def __init__(self):
            self.params = {'use_sim_time': False}
        def declare_parameter(self, name, value):
            self.params.setdefault(name, overrides.get(name, value))
        def get_parameter(self, name):
            return SimpleNamespace(value=self.params[name])
    node = Reader()
    node._declare_parameters()
    with pytest.raises(ValueError):
        node._read_parameters()


def test_owner_endpoint_gid_change_between_sessions_requires_explicit_recovery(node):
    close_round(node)
    old_pins = dict(node._global_stop._pins)
    node.advance(.01)
    node._publish_state()
    identity = node._shadow_runtime.try_begin(node._core._fire_session, GOAL, node.now)
    assert identity is not None
    node._core._fire_session.request_close(identity.session_id, 'OPERATOR_CANCEL', node.now)
    node._publish_state()
    owner = node.manifest.required[0]
    node.infos[CLOSE_OWNER_TOPICS[owner]][0].endpoint_gid = [99] * 24
    node.tick(node.now + .1)
    assert node._global_stop._pins[owner] == old_pins[owner]
    assert node._shadow_runtime.recovery_latch == 'GLOBAL_SOURCE_CHANGED:' + owner
    assert node._core._fire_session.phase == 'STOPPING'


def test_old_or_unstamped_nav_status_cannot_refresh_shadow_admission(node):
    node._publish_state()
    node.advance(.1)
    node.statuses()
    assert node._core._goal_status_received == pytest.approx(node.now)
    node.advance(.1)
    node.statuses(metadata=False)
    assert node._core._goal_status_received is None
    node.statuses(old_s=1)
    assert node._core._goal_status_received is None


def test_reset_absence_camera_age_is_charged_again_after_durable_io(node, monkeypatch):
    from test_mission_fire_session_observer import feed
    close_round(node)
    node.advance(.01)
    assert node._reset_shadow(None, SimpleNamespace()).success
    node._publish_state()
    node.statuses = lambda records=None: GlobalHarness.statuses(node, [] if records is None else records)
    start = node.now
    mark = node._shadow_runtime.store.mark_completion
    def delayed_mark(session_id):
        mark(session_id)
        node.advance(.05)
    monkeypatch.setattr(node._shadow_runtime.store, 'mark_completion', delayed_mark)
    for delta in (.1, .3, .5, .7, .9, 1.1, 1.3, 1.5, 1.7):
        feed(node._core, start + delta, present=False, age_ms=299)
        node.tick(start + delta)
    assert node._core._fire_session.phase == 'STOPPING'
    assert node._shadow_runtime.store.state['pending']['completion_marked'] is True
    assert node._shadow_runtime.blocker == 'RESET_ABSENCE_EXPIRED_DURING_COMMIT'
