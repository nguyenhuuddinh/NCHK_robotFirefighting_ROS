"""Actual alignment callbacks/tick with real core and fake ROS graph/publishers."""

from copy import deepcopy
import json
from types import SimpleNamespace

import pytest

from fire_robot_mission.live_alignment_takeover import LiveAlignmentTakeover
from fire_robot_mission.motion_release import MotionReleaseProof
from fire_robot_mission.session_context import SessionContextGuard
from test_live_alignment_takeover import (
    COUNTS, NAV, RAW, SAFETY, UUID, ZERO, ready, preview,
)
from test_session_context import INSTANCE, ORIGIN, report


class Publisher:
    def __init__(self, node, *, state=False):
        self.node, self.state = node, state
        self.messages = []

    def publish(self, message):
        self.node.events.append('state' if self.state else 'command')
        self.messages.append(json.loads(message.data) if self.state else message)


class Harness:
    _tick = LiveAlignmentTakeover._tick
    _on_json = LiveAlignmentTakeover._on_json
    _guard_owned_session = LiveAlignmentTakeover._guard_owned_session

    def __init__(self, monkeypatch, *, feature=True):
        from fire_robot_mission import live_alignment_takeover as module
        self.now = 10.0
        monkeypatch.setattr(module.time, 'monotonic', lambda: self.now)
        self.core = ready()
        self._session_context = SessionContextGuard() if feature else None
        self._motion_release = MotionReleaseProof() if feature else None
        self._raw_publisher = None
        self.events, self.created, self.states = [], [], []
        self._state_publisher = Publisher(self, state=True)
        self.visible = False
        self.odom_source_ok = True
        self.destroy_result = True
        self.seq = 0
        self.clock_reads = 0

    def get_clock(self):
        self.clock_reads += 1
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=ORIGIN + round(self.now * 1e9)))

    def _sources(self, topic):
        if topic == '/cmd_vel_raw':
            return RAW if self.visible and self._raw_publisher is not None else []
        if topic == '/fire_mission/nav2_cmd_vel_shadow':
            return NAV
        if topic == '/cmd_vel':
            return SAFETY
        if topic == '/fire_mission/state':
            return [('/', 'fire_mission_supervisor', 'std_msgs/msg/String', RAW[0][3])]
        if topic == '/odom' and self.odom_source_ok:
            return [('/', 'serial_bridge_node', 'nav_msgs/msg/Odometry', RAW[0][3])]
        return []

    def count_publishers(self, topic):
        if topic == '/pump_cmd':
            return 0
        if topic in ('/fire_mission/ir_approach_live_state',
                     '/fire_mission/post_spray_review', '/fire_sensors/actuator_state'):
            return 0
        return 1

    def create_publisher(self, _type, topic, _qos):
        assert topic == '/cmd_vel_raw'
        self.events.append('create')
        self.visible = False
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
        return SimpleNamespace(warning=lambda message: None)

    def feed(self, *, with_mission=True, phase='ACTIVE', generation=None,
             old_ms=10, session=1, goal=UUID):
        for name, (payload, _) in tuple(self.core.inputs.items()):
            self.core.update(name, payload, self.now)
        self.core.update_arm(True, self.now)
        # Effective odom time already includes header source age in production.
        self.core.update_odom(ZERO, self.now - 0.01)
        self.core.update_cmd(ZERO, self.now)
        self.core.update_scan(-3.14, 3.14, [1.0] * 360, self.now)
        if with_mission:
            self.seq += 1
            payload = deepcopy(ready().inputs['mission'][0])
            payload.update(report(ORIGIN + round(self.now * 1e9) - round(old_ms * 1e6),
                                  self.seq, phase=phase, generation=generation,
                                  session=session, goal=goal))
            self._on_json('mission', SimpleNamespace(data=json.dumps(payload)))

    def start(self):
        self.feed()
        self._tick()
        assert self._raw_publisher is not None
        assert self._raw_publisher.messages[-1].angular.z == 0.0
        self.visible = True
        self.now = 10.05
        self.feed()
        self._tick()
        assert self._raw_publisher.messages[-1].angular.z > 0.0

    def close(self):
        self.now = 10.1
        self.feed(phase='STOPPING', generation=1)
        self._tick()
        self.now = 10.4
        self.feed(phase='STOPPING', generation=1)
        self._tick()
        assert self._raw_publisher is not None
        self.now = 10.65
        self.feed(phase='STOPPING', generation=1)
        self._tick()


def test_binding_precedes_rotation_and_close_receipt_follows_zero_hold_destroy(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    binding = node._motion_release.binding
    assert binding.session_id == f'fire:{INSTANCE}:1'
    raw = node._raw_publisher
    node.close()
    assert node._raw_publisher is None
    assert node.events[-3:] == ['command', 'destroy', 'state']
    assert raw.messages[-1].angular.z == 0.0
    assert raw.messages[-1].linear.x == raw.messages[-1].linear.y == 0.0
    event = node._motion_release.receipt
    assert event['session_id'] == binding.session_id
    assert event['close_generation'] == 1
    assert event['physical_motion_verified'] is False
    assert node._state_publisher.messages[-1]['session_owner']['publisher_owned'] is False
    old = deepcopy(event)
    node.now = 10.7
    node.feed(session=2)
    node._tick()
    assert node._motion_release.receipt == old
    assert len(node.created) == 1


@pytest.mark.parametrize('result', [False, None, RuntimeError('destroy failed')])
def test_destroy_failure_never_creates_a_release_receipt(monkeypatch, result):
    node = Harness(monkeypatch)
    node.start()
    raw = node._raw_publisher
    node.destroy_result = result
    with pytest.raises(RuntimeError):
        node.close()
    assert node._raw_publisher is raw
    assert raw.messages[-1].angular.z == 0.0
    assert node._motion_release.receipt is None
    assert node.events[-1] == 'destroy'


@pytest.mark.parametrize('changes', [{'old_ms': 400}, {'goal': None},
                                    {'phase': 'STOPPING', 'generation': 1}])
def test_context_not_admitted_keeps_raw_topic_unowned(monkeypatch, changes):
    node = Harness(monkeypatch)
    node.feed(**changes)
    node._tick()
    assert node._raw_publisher is None and node.created == []
    assert node.core.phase == 'WAITING_FOR_ADMISSION'
    assert node._motion_release.receipt is None


def test_bad_then_good_session_before_timer_stays_stopped_and_keeps_rotation_budget(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.core.total_rotation_rad = 1.25
    node.now = 10.1
    node.feed(session=2)
    assert node.core.phase == 'STOPPING'
    node.now = 10.11
    node.feed(session=1)
    node._tick()
    assert node._raw_publisher.messages[-1].angular.z == 0.0
    assert node.core.phase == 'STOPPING'
    assert node.core.total_rotation_rad == 1.25
    assert node._motion_release.binding.session_id == f'fire:{INSTANCE}:1'


def test_expired_source_is_not_renewed_by_refreshing_legacy_payloads(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.now = 10.42
    node.feed(with_mission=False)
    node._tick()
    assert node.core.phase == 'STOPPING'
    assert node._raw_publisher.messages[-1].angular.z == 0.0


def test_close_during_existing_stop_requires_another_full_zero_hold(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.now = 10.1
    node.feed()
    node.core.update('preview', preview('CENTERED', 0.0), node.now)
    node._tick()
    assert node.core.phase == 'STOPPING'
    node.now = 10.4
    node.feed(phase='STOPPING', generation=1)
    node._tick()
    assert node._motion_release.zero_published_at == 10.4
    node.now = 10.65
    node.feed(phase='STOPPING', generation=1)
    node._tick()
    assert node._raw_publisher is not None
    node.now = 10.95
    node.feed(phase='STOPPING', generation=1)
    node._tick()
    assert node._raw_publisher is None


def test_wrong_odom_publisher_prevents_release_and_clears_the_hold(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    node.now = 10.1
    node.feed(phase='STOPPING', generation=1)
    node._tick()
    node.now = 10.65
    node.feed(phase='STOPPING', generation=1)
    node.odom_source_ok = False
    node._tick()
    assert node._raw_publisher is not None
    assert node.core.zero_since is None
    assert node._motion_release.receipt is None


def test_wrong_odom_source_never_creates_an_owner_before_admission(monkeypatch):
    node = Harness(monkeypatch)
    node.odom_source_ok = False
    node.feed()
    node._tick()
    assert node.created == []
    assert node.core.phase == 'WAITING_FOR_ADMISSION'


def test_slow_core_evaluation_cannot_destroy_using_expired_stop_evidence(monkeypatch):
    node = Harness(monkeypatch)
    node.start()
    step = node.core.step
    def slow_step(*args, **kwargs):
        result = step(*args, **kwargs)
        if result[2] == 'RELEASE':
            node.now += 0.4
        return result
    node.core.step = slow_step
    node.close()
    assert node._raw_publisher is not None
    assert node.core.phase == 'STOPPING'
    assert node._motion_release.receipt is None
    assert 'destroy' not in node.events
    assert node._raw_publisher.messages[-1].angular.z == 0.0


def test_already_centered_does_not_invent_an_owner_or_receipt(monkeypatch):
    node = Harness(monkeypatch)
    node.feed()
    node.core.update('preview', preview('CENTERED', 0.0), node.now)
    node._tick()
    assert node.core.phase == 'COMPLETE'
    assert node.created == [] and node._motion_release.receipt is None


def test_feature_off_has_no_session_clock_or_receipt(monkeypatch):
    node = Harness(monkeypatch, feature=False)
    node.feed()
    node._tick()
    assert node._raw_publisher is not None
    assert node.clock_reads == 0
    assert 'session_owner' not in node._state_publisher.messages[-1]


def test_core_migration_gate_does_not_reset_progress_or_rearm_on_a_new_goal():
    core = ready()
    core.phase, core.goal_uuid, core.stop_reason = 'COMPLETE', UUID, 'TARGET_CENTERED'
    core.total_rotation_rad = 1.25
    core.update_statuses([('b' * 32, 2)])
    core.step(10.1, False, [], NAV, SAFETY, COUNTS, allow_rearm=False)
    assert core.phase == 'COMPLETE'
    assert core.goal_uuid == UUID and core.total_rotation_rad == 1.25
