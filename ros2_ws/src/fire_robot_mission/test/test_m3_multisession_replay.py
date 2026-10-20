"""Bounded multi-session replay through actual supervisor callbacks/effects."""

import json
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus, GoalStatusArray

from fire_robot_mission.close_owner_evidence import CloseOwnerEvidenceCollector
from fire_robot_mission.detection_freshness import DetectionStampGuard
from fire_robot_mission.global_stop_evidence import GlobalStopEvidence
from fire_robot_mission.mission_supervisor import (
    FireMissionSupervisor, MissionSupervisorCore, THROUGH_STOP_INPUT, CLOSE_OWNER_TOPICS,
)
from fire_robot_mission.runtime_manifest import runtime_manifest
from fire_robot_mission.shadow_budget import ShadowBudgetStore
from fire_robot_mission.shadow_session_runtime import ShadowSessionRuntime
from test_global_stop_node import Harness as BaseHarness
from test_global_stop_evidence import GOAL, OTHER, actuator


class Replay(BaseHarness):
    _decode_json = FireMissionSupervisor._decode_json
    _store_detection = FireMissionSupervisor._store_detection
    _on_detection = FireMissionSupervisor._on_detection
    _on_through_nav_status = FireMissionSupervisor._on_through_nav_status
    _shadow_current_stop_result = FireMissionSupervisor._shadow_current_stop_result
    _shadow_operator_request = FireMissionSupervisor._shadow_operator_request
    _recover_shadow = FireMissionSupervisor._recover_shadow
    _reset_shadow = FireMissionSupervisor._reset_shadow

    def __init__(self, monkeypatch, tmp_path):
        super().__init__(monkeypatch)
        self._core = MissionSupervisorCore(enable_fire_session_observer=True, fire_acquire_hold_s=.1)
        manifest = 'io_nav_static_no_owners_v1'
        self._close_owner_collector = CloseOwnerEvidenceCollector(manifest_id=manifest)
        self._global_stop = GlobalStopEvidence(manifest_id=manifest,
            require_recovery_on_change=True, require_through_status=True)
        store = ShadowBudgetStore(str(tmp_path / 'replay_budget.json'), initialize=True)
        self._shadow_runtime = ShadowSessionRuntime(store, runtime_manifest(manifest))
        self._core._session_runtime = self._shadow_runtime
        self._detection_stamp_guard = DetectionStampGuard()
        for topic in CLOSE_OWNER_TOPICS.values():
            self.infos[topic] = []
        info = self.infos['/navigate_to_pose/_action/status'][0]
        self.infos[THROUGH_STOP_INPUT[0]] = [SimpleNamespace(
            node_name=info.node_name, node_namespace=info.node_namespace, topic_type=info.topic_type,
            qos_profile=info.qos_profile, endpoint_gid=[123] * 24)]
        self.trace = []

    def get_node_names_and_namespaces(self):
        return []

    def status_message(self, records, through=False, metadata=True, old_s=0):
        msg = GoalStatusArray()
        for uuid, status in records:
            item = GoalStatus()
            item.goal_info.goal_id.uuid = list(bytes.fromhex(uuid))
            item.status = status
            msg.status_list.append(item)
        callback = self._on_through_nav_status if through else self._on_nav_status
        callback(msg, {'source_timestamp': self.stamp - round(old_s * 1e9)} if metadata else None)

    def detection(self, present, *, old_s=0):
        sec, ns = divmod(self.stamp - round(old_s * 1e9), 1_000_000_000)
        self._on_detection(SimpleNamespace(data=json.dumps({
            'observe_only': True, 'state': 'ok',
            'source': {'age_ms': old_s * 1000, 'stamp': {'sec': sec, 'nanosec': ns}},
            'fire': {'detected_now': present, 'confirmed': present}})))

    def tick(self, delta=.11, *, fire=False, records=(), through=(), nav_age=0, detection=True):
        self.advance(delta)
        self.seq += 1
        self.motion()
        self.status_message(records, old_s=nav_age)
        self.status_message(through, through=True)
        self._on_actuator(SimpleNamespace(data=json.dumps(actuator(self.now, self.seq))))
        if detection:
            self.detection(fire)
        self._core.update_alignment({
            'observe_only': True, 'state': 'CENTERED', 'preview': {},
            'safety': {'motion_command_publisher': False, 'actuator_command_publisher': False}}, self.now)
        self._publish_state()
        session = self._core._fire_session
        self.trace.append((self.now, session.phase,
                           None if session.identity is None else session.identity.session_id))

    def acquire(self, goal=None):
        for _ in range(3):
            self.tick(fire=True, records=() if goal is None else [(goal, 2)])
        assert self._core._fire_session.phase == 'ACTIVE'
        return self._core._fire_session.identity

    def absent_close(self, goal=None, terminal=5):
        for _ in range(23):
            self.tick(records=() if goal is None else [(goal, terminal)])
        assert self._core._fire_session.phase == 'IDLE'


@pytest.fixture
def node(monkeypatch, tmp_path):
    result = Replay(monkeypatch, tmp_path)
    yield result
    result._shadow_runtime.store.close()


@pytest.mark.parametrize('terminal', [4, 5, 6])
def test_navigation_without_fire_accepts_next_goal_without_fire_reservation(node, terminal):
    for records in ([(GOAL, 2)], [(GOAL, terminal)], [], [(OTHER, 2)], [(OTHER, 4)]):
        node.tick(records=records)
        assert node._core._fire_session.phase == 'IDLE'
    assert node._shadow_runtime.store.state['used']['sessions'] == 0
    assert node._shadow_runtime.store.state['pending'] is None


@pytest.mark.parametrize('terminal', [4, 5, 6])
def test_goal_fire_close_new_no_goal_fire_then_new_goal_share_one_encounter(node, terminal):
    first = node.acquire(GOAL)
    node.absent_close(GOAL, terminal)
    second = node.acquire()
    assert first.session_id != second.session_id and second.nav_goal_uuid is None
    node.absent_close()
    third = node.acquire(OTHER)
    assert third.session_id not in (first.session_id, second.session_id)
    node.absent_close(OTHER)
    assert node._shadow_runtime.closed_count == 3
    assert node._shadow_runtime.store.state['used']['sessions'] == 3
    for _ in range(4):
        node.tick(fire=True)
    assert node._core._fire_session.phase == 'IDLE'
    assert node._shadow_runtime.blocker == 'SHADOW_ENCOUNTER_BUDGET_EXHAUSTED'
    assert any(phase == 'STOPPING' for _, phase, _ in node.trace)
    assert all(not msg['lifecycle']['fire_session_observer']['actuation_authorized'] for msg in node.messages)


def test_short_miss_preserves_session_but_missing_images_closes_safely(node):
    identity = node.acquire()
    node.tick(fire=False)
    node.tick(fire=True)
    assert node._core._fire_session.identity == identity
    node.tick(delta=.6, detection=False)
    assert node._core._fire_session.phase == 'STOPPING'
    assert node._core._fire_session.close_reason == 'SAFETY_ABORT'
    assert node._shadow_runtime.store.state['used']['sessions'] == 1


def test_old_target_frame_received_after_cleanup_cannot_start_new_session(node, monkeypatch):
    node.acquire()
    mark = node._shadow_runtime.store.mark_completion
    def commit_delay(session_id):
        mark(session_id)
        node.advance(.03)
    monkeypatch.setattr(node._shadow_runtime.store, 'mark_completion', commit_delay)
    for _ in range(23):
        node.tick()
        if node._core._fire_session.phase == 'IDLE':
            break
    assert node._core._fire_session.phase == 'IDLE'
    barrier = node._shadow_input_barrier_ns
    # A queued image can have a stamp newer than the prior admitted image yet
    # predate the durable cleanup. It must not be admitted into session B.
    node.advance(.05)
    sec, ns = divmod(barrier, 1_000_000_000)
    assert node._detection_stamp_guard.last_stamp_ns < barrier
    node._on_detection(SimpleNamespace(data=json.dumps({
        'observe_only': True, 'state': 'ok',
        'source': {'age_ms': 50, 'stamp': {'sec': sec, 'nanosec': ns}},
        'fire': {'detected_now': True, 'confirmed': True}})))
    assert node._core._detection is None
    assert node._core._detection_error_blocker == 'DETECTION_BEFORE_SESSION_CLEANUP'
    assert node._core._fire_session.phase == 'IDLE'
    identity = node.acquire()
    assert identity is not None and node._shadow_runtime.store.state['used']['sessions'] == 2


@pytest.mark.parametrize('state', [0, 1, 2, 3])
def test_through_poses_active_or_unknown_never_looks_like_no_navigation(node, state):
    for _ in range(4):
        node.tick(fire=True, through=[(OTHER, state)])
    assert node._core._fire_session.phase == 'IDLE'
    assert node._shadow_runtime.store.state['used']['sessions'] == 0
    assert node._shadow_runtime.navigation_blocker == 'THROUGH_NAVIGATION_ACTIVE_OR_UNKNOWN'


def test_no_goal_still_needs_new_original_status_from_both_servers(node):
    for _ in range(4):
        node.tick(fire=True, nav_age=1)
    assert node._core._fire_session.phase == 'IDLE'
    assert node._shadow_runtime.store.state['used']['sessions'] == 0
    node.acquire()
    assert node._core._fire_session.identity.nav_goal_uuid is None


def test_operator_recovery_requires_new_close_round_and_keeps_reservations(node):
    identity = node.acquire(GOAL)
    node.tick(delta=.6, detection=False, records=[(GOAL, 2)])
    old = node._core._fire_session.close_request
    node.advance(.01)
    response = node._recover_shadow(None, SimpleNamespace())
    assert response.success
    assert node._core._fire_session.close_request.generation > old.generation
    assert node._global_stop.request is None
    for _ in range(9):
        node.tick(records=[(GOAL, 5)])
    assert node._core._fire_session.phase == 'IDLE'
    assert node._core._fire_session.last_closed[0].session_id == identity.session_id
    assert node._shadow_runtime.store.state['used']['sessions'] == 1
    assert node._shadow_runtime.store.state['recoveries'] == 1


def test_wrong_goal_terminal_cannot_release_bound_session(node):
    node.acquire(GOAL)
    for _ in range(23):
        node.tick(records=[(OTHER, 5)])
    assert node._core._fire_session.phase == 'STOPPING'
    assert 'GLOBAL_EXACT_NAV_TERMINAL_MISSING' in node._global_stop_result.blockers
    assert node._shadow_runtime.store.state['pending'] is not None


def test_through_poses_start_during_fire_latches_close_and_blocks_cleanup(node):
    node.acquire()
    for _ in range(10):
        node.tick(fire=True, through=[(OTHER, 2)])
    assert node._core._fire_session.phase == 'STOPPING'
    assert node._core._fire_session.close_reason == 'SAFETY_ABORT'
    assert node._shadow_runtime.closed_count == 0


def test_missing_through_status_after_close_blocks_until_a_new_source_snapshot(node):
    node.acquire()
    for _ in range(15):
        node.tick()
    assert node._core._fire_session.phase == 'STOPPING'
    callback = node._on_through_nav_status
    node._on_through_nav_status = lambda msg, metadata: None
    for _ in range(7):
        node.tick()
    assert node._core._fire_session.phase == 'STOPPING'
    assert node._shadow_runtime.navigation_blocker == 'THROUGH_NAV_STATUS_NOT_CURRENT'
    node._on_through_nav_status = callback
    for _ in range(7):
        node.tick()
    assert node._core._fire_session.phase == 'IDLE'


def test_old_owner_ack_or_old_typed_stop_evidence_cannot_close_new_session(node):
    first = node.acquire()
    node.absent_close()
    old_evidence = node._core._fire_session.last_closed_evidence
    second = node.acquire()
    node.advance(.01)
    node._recover_shadow(None, SimpleNamespace())
    assert first != second
    assert not node._core.finish_verified_session(old_evidence, node.now)
    assert node._core._fire_session.phase == 'STOPPING'
    assert node._shadow_runtime.store.state['pending']['session_id'] == second.session_id


def test_restart_with_pending_session_needs_explicit_recovery_and_retains_real_goal(node):
    original = node.acquire(GOAL)
    path = str(node._shadow_runtime.store.path)
    reserved = dict(node._shadow_runtime.store.state['used'])
    node._shadow_runtime.store.close()
    node._core = MissionSupervisorCore(enable_fire_session_observer=True, fire_acquire_hold_s=.1)
    manifest = 'io_nav_static_no_owners_v1'
    node._shadow_runtime = ShadowSessionRuntime(ShadowBudgetStore(path), runtime_manifest(manifest))
    node._core._session_runtime = node._shadow_runtime
    node._close_owner_collector = CloseOwnerEvidenceCollector(manifest_id=manifest)
    node._global_stop = GlobalStopEvidence(manifest_id=manifest,
        require_recovery_on_change=True, require_through_status=True)
    node._detection_stamp_guard = DetectionStampGuard()
    node._session_source_sequence = 0
    for _ in range(3):
        node.tick(fire=True)
    assert node._core._fire_session.phase == 'IDLE'
    assert node._shadow_runtime.recovery_latch == 'PERSISTED_SESSION_REQUIRES_RECOVERY'
    node.advance(.01)
    assert node._recover_shadow(None, SimpleNamespace()).success
    request = node._core._fire_session.close_request
    assert request.nav_goal_uuid == GOAL and request.session_id != original.session_id
    assert node._shadow_runtime.store.state['pending']['origin_session_id'] == original.session_id
    for _ in range(9):
        node.tick(records=[(GOAL, 5)])
    assert node._core._fire_session.phase == 'IDLE'
    assert node._shadow_runtime.store.state['used'] == reserved
    assert node._shadow_runtime.store.state['recoveries'] == 1


def test_reset_is_a_fresh_maintenance_probe_then_allows_new_encounter(node):
    node.acquire()
    node.absent_close()
    encounter = node._shadow_runtime.store.state['encounter_id']
    node.advance(.01)
    assert node._reset_shadow(None, SimpleNamespace()).success
    for _ in range(12):
        node.tick()
    assert node._core._fire_session.phase == 'STOPPING'
    assert node._shadow_runtime.store.state['used']['sessions'] == 1
    for _ in range(8):
        node.tick()
    assert node._core._fire_session.phase == 'IDLE'
    assert node._shadow_runtime.store.state['used']['sessions'] == 0
    assert node._shadow_runtime.store.state['encounter_id'] != encounter
    node.acquire()
    assert node._shadow_runtime.store.state['used']['sessions'] == 1
