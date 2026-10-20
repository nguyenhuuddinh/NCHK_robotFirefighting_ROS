"""Real pump callbacks and pure pump core with fake ROS publishers/graph."""

from copy import deepcopy
import json
from types import SimpleNamespace

import pytest

from fire_robot_mission.pump_demo_controller import PumpDemoController
from fire_robot_mission.session_context import SessionContextGuard
from test_pump_demo_core import ready_core, refresh
from test_pump_release_node import Harness
from test_session_context import GOAL, INSTANCE, ORIGIN, report


class ContextHarness(Harness):
    _on_json = PumpDemoController._on_json
    _guard_owned_session = PumpDemoController._guard_owned_session

    def __init__(self, monkeypatch):
        super().__init__(monkeypatch, owned=False, core=ready_core(pulse_s=1.0))
        self._session_context = SessionContextGuard()
        self.mission_endpoint_ok = True
        self.session_seq = 0

    def _exact_publisher_count(self, key):
        if key == 'mission' and not self.mission_endpoint_ok:
            return 0
        return super()._exact_publisher_count(key)

    def mission(self, *, old_ms=10, **changes):
        self.session_seq += 1
        payload = deepcopy(ready_core().inputs['mission'][0])
        payload.update(report(
            ORIGIN + round(self.now * 1e9) - round(old_ms * 1e6),
            self.session_seq, **changes))
        self._on_json('mission', SimpleNamespace(data=json.dumps(payload)))

    def acquire(self):
        refresh(self.core, self.now)
        self.receive()
        self.mission()
        self._tick()
        assert self.pump_pub is not None
        assert self.pump_pub.messages == [False]

    def start(self):
        self.acquire()
        self.graph_visible = True
        for now in (1.51, 1.92):
            self.now = now
            refresh(self.core, now)
            self.mission()
            self._tick()
        assert self.pump_pub.messages[-1] is True


def test_bind_before_on_and_preserve_real_owner_discovery_and_hold(monkeypatch):
    node = ContextHarness(monkeypatch)
    node.acquire()
    binding = node._pump_release.session_binding
    assert binding.session_id == f'fire:{INSTANCE}:1'
    assert binding.nav_goal_uuid == GOAL
    assert binding.source_sequence == 1
    assert node.core.ready_since is None  # No synthesized graph count1.
    assert node.state_pub.messages[-1]['session_context']['required'] is True


@pytest.mark.parametrize('change', [{'old_ms': 400}, {'goal': None},
                                   {'phase': 'STOPPING', 'generation': 1}])
def test_unusable_session_never_creates_a_pump_publisher(monkeypatch, change):
    node = ContextHarness(monkeypatch)
    refresh(node.core, node.now)
    node.receive()
    node.mission(**change)
    node._tick()
    assert node.pump_pub is None
    assert node.created == []
    assert node.state_pub.messages[-1]['pump_on'] is False


@pytest.mark.parametrize('change', [{'session': 2}, {'goal': 'b' * 32},
                                   {'instance': 'ef' * 16}])
def test_owned_pump_stops_on_context_change_and_cannot_rebind(monkeypatch, change):
    node = ContextHarness(monkeypatch)
    node.start()
    binding = node._pump_release.session_binding
    node.now = 1.93
    refresh(node.core, node.now)
    node.mission(**change)
    node._tick()
    assert node.pump_pub.messages[-1] is False
    assert node.core.state == 'ABORTED'
    assert node.core.stop_reason == 'SESSION_CONTEXT_INVALID'
    assert node._pump_release.session_binding is binding
    assert node.core.goal_uuid == GOAL
    # A fresh report cannot turn a fatal abort into an automatic resume.
    node.now = 1.94
    refresh(node.core, node.now)
    node.mission(**change)
    node._tick()
    assert node.pump_pub.messages[-1] is False
    assert node.core.state == 'ABORTED'


def test_cache_expiry_stops_pump_even_when_legacy_inputs_are_refreshed(monkeypatch):
    node = ContextHarness(monkeypatch)
    node.start()
    node.now = 2.28  # 10ms source age + 360ms cache exceeds 350ms.
    refresh(node.core, node.now)
    node._tick()
    assert node.core.state == 'ABORTED'
    assert node.pump_pub.messages[-1] is False
    assert node.state_pub.messages[-1]['session_context']['blocker'] == 'SESSION_SOURCE_STALE'


def test_invalid_mission_source_clears_cache_and_stops_owned_pump(monkeypatch):
    node = ContextHarness(monkeypatch)
    node.start()
    node.now = 1.93
    node.mission_endpoint_ok = False
    node.mission()
    assert node._session_context.sample is None
    assert node.core.inputs['mission'][0] is None
    node._tick()
    assert node.core.state == 'ABORTED'
    assert node.pump_pub.messages[-1] is False


def test_bad_then_good_source_before_timer_cannot_hide_the_latched_stop(monkeypatch):
    node = ContextHarness(monkeypatch)
    node.start()
    node.now = 1.93
    node.mission_endpoint_ok = False
    node.mission()
    assert node.core.state == 'ABORTED'  # Callback latches before timer.
    node.now = 1.94
    node.mission_endpoint_ok = True
    refresh(node.core, node.now)
    node.mission()
    assert node._session_context.sample is not None
    node._tick()
    assert node.core.state == 'ABORTED'
    assert node.core.stop_reason == 'SESSION_CONTEXT_INVALID'
    assert node.pump_pub.messages[-1] is False


def test_close_while_waiting_releases_after_new_off_with_original_request_generation(monkeypatch):
    node = ContextHarness(monkeypatch)
    node.acquire()
    binding = node._pump_release.session_binding
    node.graph_visible = True
    node.now = 1.51
    refresh(node.core, node.now)
    node.mission(phase='STOPPING', generation=1)
    node._tick()
    assert node.core.state == 'ABORTED'
    assert node.core.stop_reason == 'SESSION_CLOSE_REQUESTED'
    assert node.core.goal_uuid == GOAL
    assert node.pump_pub.messages == [False, False]
    assert node._pump_release.receipt is None
    first_request = deepcopy(node._pump_release.close_request_source)
    node.now = 1.61
    refresh(node.core, node.now)
    node.mission(phase='STOPPING', generation=1)
    node.receive(seq=2)
    node._tick()
    assert node.pump_pub is None
    receipt = node._pump_release.receipt
    assert receipt['session_id'] == binding.session_id
    assert receipt['session_binding']['source_sequence'] == binding.source_sequence
    assert receipt['close_generation'] == 1
    assert receipt['close_request_source'] == first_request
    assert receipt['physical_pump_or_nozzle_verified'] is False


def test_terminal_release_never_relabels_the_old_binding_from_a_new_snapshot(monkeypatch):
    node = ContextHarness(monkeypatch)
    node.start()
    binding = node._pump_release.session_binding
    node.now = 2.93
    refresh(node.core, node.now)
    node.mission()
    node._tick()
    assert node.core.state == 'COMPLETE'
    node.now = 3.03
    refresh(node.core, node.now)
    node.mission(session=2)
    node.receive(seq=2)
    node._tick()
    receipt = deepcopy(node._pump_release.receipt)
    assert node.pump_pub is None
    assert receipt['session_id'] == binding.session_id
    assert receipt['session_binding']['source_sequence'] == binding.source_sequence
    assert receipt['close_generation'] is None
    assert node._pump_release.snapshot(owned=False)['receipt'] == receipt
    assert node.core.pulse_s == 1.0
    close = report(ORIGIN + round(node.now * 1e9), 100, phase='STOPPING', generation=1)
    observer = close['lifecycle']['fire_session_observer']
    assert not node._pump_release.accept_close_request(
        session_id=observer['session_id'], nav_goal_uuid=GOAL,
        generation=1, source=observer['source'])
    assert node._pump_release.receipt == receipt


def test_close_after_terminal_discards_old_off_sample_and_requires_another_off_barrier(monkeypatch):
    node = ContextHarness(monkeypatch)
    node.start()
    node.now = 2.93
    refresh(node.core, node.now)
    node.mission()
    node._tick()
    old_barrier = node._pump_release.request_ros_ns
    node.now = 3.03
    node.receive(seq=2)  # OFF received before this close request was processed.
    refresh(node.core, node.now)
    node.mission(phase='STOPPING', generation=5)
    assert node._pump_release.sample is None
    node._tick()
    assert node.pump_pub is not None
    assert node._pump_release.request_ros_ns > old_barrier
    source = deepcopy(node._pump_release.close_request_source)
    node.now = 3.13
    refresh(node.core, node.now)
    node.mission(phase='STOPPING', generation=5)
    node.receive(seq=3)
    node._tick()
    assert node.pump_pub is None
    assert node._pump_release.receipt['close_generation'] == 5
    assert node._pump_release.receipt['close_request_source'] == source


def test_duplicate_close_does_not_reset_barrier_and_changed_generation_is_rejected(monkeypatch):
    node = ContextHarness(monkeypatch)
    node.acquire()
    node.graph_visible = True
    node.now = 1.51
    refresh(node.core, node.now)
    node.mission(phase='STOPPING', generation=1)
    node._tick()
    barrier = node._pump_release.request_ros_ns
    source = deepcopy(node._pump_release.close_request_source)
    node.now = 1.52
    refresh(node.core, node.now)
    node.mission(phase='STOPPING', generation=1)
    node._tick()
    assert node._pump_release.request_ros_ns == barrier
    assert node._pump_release.close_request_source == source
    assert not node._pump_release.accept_close_request(
        session_id=f'fire:{INSTANCE}:1', nav_goal_uuid=GOAL,
        generation=2, source=source)
    assert node._pump_release.close_generation == 1
    assert node._pump_release.request_ros_ns == barrier


def test_alignment_proof_for_another_goal_cannot_keep_the_bound_pump_on(monkeypatch):
    node = ContextHarness(monkeypatch)
    node.start()
    node.now = 1.93
    refresh(node.core, node.now)
    node.mission()
    alignment = deepcopy(node.core.inputs['alignment'][0])
    alignment['goal_uuid'] = 'b' * 32
    node.core.update('alignment', alignment, node.now)
    node._tick()
    assert node.core.state == 'ABORTED'
    assert node.pump_pub.messages[-1] is False
    assert node._pump_release.session_binding.nav_goal_uuid == GOAL


def test_context_feature_off_does_not_add_source_queries_or_ros_clock_reads(monkeypatch):
    node = Harness(monkeypatch, feature=False)
    def unexpected(*args):
        raise AssertionError('context OFF must not read source graph or ROS clock')
    node.get_clock = unexpected
    node._exact_publisher_count = unexpected
    PumpDemoController._on_json(node, 'mission', SimpleNamespace(data='{"observe_only":true}'))
    assert node.core.inputs['mission'][0] == {'observe_only': True}
