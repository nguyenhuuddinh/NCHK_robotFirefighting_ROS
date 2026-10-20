"""Real pump callbacks with fake graph/publishers and the existing pure core."""

from copy import deepcopy
from types import SimpleNamespace

import pytest
from rclpy.qos import ReliabilityPolicy

from fire_robot_mission.pump_demo_controller import PumpDemoController
from fire_robot_mission.pump_release import PumpReleaseProof
from test_pump_demo_core import COUNTS, ready_core, refresh
from test_pump_release import ORIGIN, actor


GOAL = 'a' * 32
NEXT_GOAL = 'b' * 32


class Publisher:
    def __init__(self, node, *, state=False):
        self.node, self.state = node, state
        self.messages = []

    def publish(self, msg):
        import json
        self.node.events.append('state' if self.state else 'ON' if msg.data else 'OFF')
        self.messages.append(json.loads(msg.data) if self.state else msg.data)


class Harness:
    _tick = PumpDemoController._tick
    _on_actuator = PumpDemoController._on_actuator
    _actuator_source_ok = PumpDemoController._actuator_source_ok
    _create_pump_publisher = PumpDemoController._create_pump_publisher

    def __init__(self, monkeypatch, *, feature=True, owned=True, core=None):
        from fire_robot_mission import pump_demo_controller as module
        self.now = 1.5
        monkeypatch.setattr(module.time, 'monotonic', lambda: self.now)
        self.core = core or ready_core(pulse_s=1.0)
        if core is None:
            self.core.step(1.0, COUNTS)
            refresh(self.core, 1.41)
            assert self.core.step(1.41, COUNTS)[0]
        self._pump_release = PumpReleaseProof() if feature else None
        self._pump_qos = object()
        self.pump_pub = None
        self.events, self.created = [], []
        self.graph_visible = True
        self.foreign = 0
        self.foreign_exact = False
        self.source_ok = True
        self.source_delay = 0.0
        self.destroy_result = True
        self.subscribers = 1
        self.total_reads = 0
        self.state_pub = Publisher(self, state=True)
        if owned:
            self._create_pump_publisher()
            self.graph_visible = True
        self.events.clear()

    def create_publisher(self, msg_type, topic, qos):
        assert topic == '/pump_cmd'
        self.events.append('create')
        self.graph_visible = False
        publisher = Publisher(self)
        self.created.append(publisher)
        return publisher

    def destroy_publisher(self, publisher):
        assert publisher is self.pump_pub
        self.events.append('destroy')
        if isinstance(self.destroy_result, Exception):
            raise self.destroy_result
        return self.destroy_result

    def _exact_publisher_count(self, key):
        if key == 'pump':
            if self.foreign_exact:
                return 1
            return int(self.pump_pub is not None and self.graph_visible and self.foreign == 0)
        return COUNTS[key]

    def _exact_bridge_subscriber_count(self):
        return self.subscribers

    def count_publishers(self, topic):
        if topic == '/pump_cmd':
            self.total_reads += 1
            return int(self.pump_pub is not None and self.graph_visible) + self.foreign
        return 0 if topic == '/cmd_vel_raw' else 1

    def get_publishers_info_by_topic(self, topic):
        assert topic == '/fire_sensors/actuator_state'
        self.now += self.source_delay
        if not self.source_ok:
            return []
        return [SimpleNamespace(
            node_namespace='/', node_name='serial_bridge_node',
            topic_type='std_msgs/msg/String',
            qos_profile=SimpleNamespace(reliability=ReliabilityPolicy.BEST_EFFORT))]

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=ORIGIN + round(self.now * 1e9)))

    def receive(self, *, seq=1, on=False, epoch=1):
        import json
        payload = actor(ORIGIN + round(self.now * 1e9) - 10_000_000,
                        seq, on=on, epoch=epoch)
        self._on_actuator(SimpleNamespace(data=json.dumps(payload)))

    def terminal(self):
        self.receive(on=True)
        self.now = 2.42
        refresh(self.core, self.now)
        self._tick()
        assert self.core.state == 'COMPLETE'

    def release(self):
        self.terminal()
        self.now = 2.52
        self.receive(seq=2)
        self._tick()
        assert self.pump_pub is None


def new_goal_proofs(core, now):
    template = ready_core(pulse_s=core.pulse_s)
    for key, (payload, _) in template.inputs.items():
        payload = deepcopy(payload)
        if 'goal_uuid' in payload:
            payload['goal_uuid'] = NEXT_GOAL
        core.update(key, payload, now)
    core.update_statuses([(NEXT_GOAL, 5, 200, 0)], now)
    core.update_arm(True, now)
    core.update_odom((0.0,) * 6, now)
    core.update_cmd((0.0,) * 6, now)


def test_terminal_off_then_fresh_act_then_destroy_and_immutable_receipt(monkeypatch):
    node = Harness(monkeypatch)
    node.terminal()
    assert node.events == ['OFF', 'state']
    assert node._pump_release.receipt is None
    old_owner = node.pump_pub
    node.events.clear()
    node.now = 2.52
    node.receive(seq=2)
    node._tick()
    assert node.events == ['OFF', 'destroy', 'state']
    assert old_owner.messages == [False, False]
    assert node.pump_pub is None
    receipt = deepcopy(node._pump_release.receipt)
    assert receipt['goal_uuid'] == GOAL
    assert receipt['session_id'] is None
    assert node.core.arm_cycle_seen is False
    assert node.core.goal_uuid == GOAL
    node.now = 2.53
    node._tick()
    assert node._pump_release.receipt == receipt
    assert node.state_pub.messages[-1]['pump_release']['publisher_owned'] is False


def test_pending_release_freezes_goal_rearm(monkeypatch):
    node = Harness(monkeypatch)
    node.terminal()
    node.now = 2.45
    node.core.update_statuses([(NEXT_GOAL, 2, 200, 0)], node.now)
    node._tick()
    assert node.core.state == 'COMPLETE'
    assert node.core.goal_uuid == GOAL
    assert node._pump_release.goal_uuid == GOAL
    assert node._pump_release.pending is True


@pytest.mark.parametrize('result', [False, None, RuntimeError('destroy failed')])
def test_destroy_failure_never_drops_owner_or_publishes_release_receipt(monkeypatch, result):
    node = Harness(monkeypatch)
    node.terminal()
    old_owner = node.pump_pub
    old_state_count = len(node.state_pub.messages)
    node.destroy_result = result
    node.now = 2.52
    node.receive(seq=2)
    with pytest.raises(RuntimeError):
        node._tick()
    assert node.pump_pub is old_owner
    assert old_owner.messages[-1] is False
    assert node._pump_release.receipt is None
    assert len(node.state_pub.messages) == old_state_count
    assert node.core.arm_cycle_seen is True  # Reset only after successful release.


def test_new_goal_needs_new_arm_and_real_owner_discovery_before_on(monkeypatch):
    node = Harness(monkeypatch)
    node.release()
    old_token = node._pump_release.receipt['owner_token']
    node.now = 2.6
    node.core.update_statuses([(NEXT_GOAL, 2, 200, 0)], node.now)
    node._tick()
    assert node.core.state == 'WAITING_FOR_ADMISSION'
    assert GOAL in node.core.used_goal_uuids
    node.now = 2.7
    new_goal_proofs(node.core, node.now)
    node.receive(seq=3)
    node._tick()
    assert node.pump_pub is None  # A high heartbeat alone is insufficient.
    node.core.update_arm(False, 2.71)
    node.core.update_arm(True, 2.72)
    node.now = 2.72
    new_goal_proofs(node.core, node.now)
    node._tick()
    assert node.pump_pub is not None
    assert node.pump_pub.messages == [False]
    assert node._pump_release.owner_token != old_token
    assert node.core.ready_since is None  # Actual graph snapshot was count0.
    node.graph_visible = True
    node.now = 2.73
    new_goal_proofs(node.core, node.now)
    node._tick()
    assert node.pump_pub.messages[-1] is False
    node.now = 3.14
    new_goal_proofs(node.core, node.now)
    node._tick()
    assert node.pump_pub.messages[-1] is True
    assert node.core.goal_uuid == NEXT_GOAL
    assert GOAL in node.core.used_goal_uuids
    assert node.core.pulse_s == 1.0


def test_a_foreign_exact_name_publisher_cannot_impersonate_this_nodes_owner(monkeypatch):
    node = Harness(monkeypatch, owned=False, core=ready_core())
    node.foreign, node.foreign_exact = 1, True
    for now in (1.5, 2.0):
        node.now = now
        refresh(node.core, now)
        node._tick()
        assert node.pump_pub is None
        assert node.state_pub.messages[-1]['pump_on'] is False
        assert 'PUMP_PUBLISHER_NOT_OWNED' in node.state_pub.messages[-1]['blockers']
    assert node.created == []


@pytest.mark.parametrize('conflict', ['actuator', 'pump', 'subscriber'])
def test_release_waits_when_endpoint_proof_is_not_exclusive(monkeypatch, conflict):
    node = Harness(monkeypatch)
    node.terminal()
    node.now = 2.52
    node.receive(seq=2)
    if conflict == 'actuator':
        node.source_ok = False
    elif conflict == 'pump':
        node.foreign = 1
    else:
        node.subscribers = 0
    node._tick()
    assert node.pump_pub is not None
    assert node.pump_pub.messages[-1] is False
    assert node._pump_release.receipt is None


def test_slow_endpoint_query_cannot_extend_off_freshness(monkeypatch):
    node = Harness(monkeypatch)
    node.terminal()
    node.now = 2.52
    node.receive(seq=2)
    node.source_delay = 0.4
    node._tick()
    assert node._pump_release.receipt is None
    assert node.pump_pub is not None


def test_feature_off_preserves_existing_publisher_and_has_no_actuator_queries(monkeypatch):
    node = Harness(monkeypatch, feature=False)
    node.now = 2.42
    refresh(node.core, node.now)
    node._tick()
    assert node.pump_pub is not None
    assert node.pump_pub.messages == [False]
    assert node.total_reads == 0
    assert 'pump_release' not in node.state_pub.messages[-1]


def test_migration_idle_with_no_evidence_keeps_pump_topic_unowned(monkeypatch):
    from fire_robot_mission.pump_demo_core import PumpDemoCore
    node = Harness(monkeypatch, owned=False, core=PumpDemoCore(
        enabled=True, allow_provisional_ir=True))
    node._tick()
    assert node.pump_pub is None
    assert node.created == []
    assert node.state_pub.messages[-1]['pump_on'] is False


def test_initial_acquisition_is_off_until_real_owner_count_and_hold(monkeypatch):
    node = Harness(monkeypatch, owned=False, core=ready_core())
    refresh(node.core, node.now)
    node.receive()
    node._tick()
    assert node.pump_pub is not None
    assert node.pump_pub.messages == [False]
    assert node.core.ready_since is None
    assert node.state_pub.messages[-1]['pump_on'] is False


def test_core_cannot_inherit_hold_time_while_no_owner_exists():
    core = ready_core()
    for now in (1.0, 1.5):
        refresh(core, now)
        assert core.step(now, COUNTS, owner_present=False)[0] is False
        assert core.ready_since is None
    refresh(core, 1.6)
    assert core.step(1.6, COUNTS, owner_present=True)[0] is False
    refresh(core, 2.01)
    assert core.step(2.01, COUNTS, owner_present=True)[0] is True
