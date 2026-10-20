"""Actual supervisor callbacks, graph snapshots and publish; no ROS init."""

from copy import deepcopy
import json
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from fire_robot_mission.actuator_freshness import ActuatorStampGuard
from fire_robot_mission.global_stop_evidence import GlobalStopEvidence, STOP_SOURCES
from fire_robot_mission import mission_supervisor as module
from fire_robot_mission.mission_supervisor import (
    CLOSE_OWNER_MANIFEST, CLOSE_OWNER_TOPICS, STOP_INPUTS,
    DurabilityPolicy, FireMissionSupervisor, ReliabilityPolicy,
)
from test_global_stop_evidence import GOAL, OTHER, SOURCES, actuator
from test_supervisor_close_owner_collection import Harness as OwnerHarness


class Harness(OwnerHarness):
    _on_odom = FireMissionSupervisor._on_odom
    _on_nav_status = FireMissionSupervisor._on_nav_status
    _on_actuator = FireMissionSupervisor._on_actuator
    _on_stop_cmd = FireMissionSupervisor._on_stop_cmd
    _observe_stop_odom = FireMissionSupervisor._observe_stop_odom
    _stop_source_id = FireMissionSupervisor._stop_source_id
    _stop_metadata_stamp = staticmethod(FireMissionSupervisor._stop_metadata_stamp)

    def __init__(self, monkeypatch, *, goal=GOAL):
        super().__init__(goal=goal)
        self._global_stop = GlobalStopEvidence()
        self._actuator_stamp_guard = ActuatorStampGuard(max_age_ms=350, max_future_skew_ms=20)
        self.actuator_state_topic = STOP_INPUTS['actuator'][0]
        self.raw_count, self.pump_count, self.seq = 0, 0, 0
        # Legacy odom admission has its own dedicated tests. This stub only
        # avoids real monotonic time; the global callback still validates data.
        monkeypatch.setattr(module, 'admit_odom', lambda node, msg, timeout: node.now)
        for role, (topic, name, kind, status) in STOP_INPUTS.items():
            self.infos[topic] = [SimpleNamespace(
                node_name=name, node_namespace='/', topic_type=kind,
                endpoint_gid=list(SOURCES[role]),
                qos_profile=SimpleNamespace(
                    reliability=ReliabilityPolicy.RELIABLE if status else ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL if status else DurabilityPolicy.VOLATILE))]
        for owner, topic in CLOSE_OWNER_TOPICS.items():
            self.infos[topic][0].qos_profile.durability = DurabilityPolicy.VOLATILE
            self.infos[topic][0].endpoint_gid = list(SOURCES[owner])

    def count_publishers(self, topic):
        self.events.append(('count', topic))
        if topic == '/pump_cmd':
            return self.pump_count
        assert topic == '/cmd_vel_raw'
        return self.raw_count

    def motion(self, *, axis=None, moving=False, stale_s=0, frame='odom'):
        odom, cmd = Odometry(), Twist()
        stamp = self.stamp - round(stale_s * 1e9)
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        odom.header.frame_id, odom.child_frame_id = frame, 'base_link'
        if axis is not None:
            vector, field = [('linear', 'x'), ('linear', 'y'), ('linear', 'z'),
                             ('angular', 'x'), ('angular', 'y'), ('angular', 'z')][axis]
            setattr(getattr(odom.twist.twist if moving else cmd, vector), field, 0.2)
        self._on_odom(odom)
        self._on_stop_cmd(cmd, {'source_timestamp': stamp})

    def statuses(self, records=None, *, metadata=True, old_s=0):
        message = GoalStatusArray()
        for goal, state in (([] if self._global_stop.request.nav_goal_uuid is None else [(GOAL, 5)])
                            if records is None else records):
            item = GoalStatus()
            item.goal_info.goal_id.uuid = list(bytes.fromhex(goal))
            item.status = state
            message.status_list.append(item)
        self._on_nav_status(message, {'source_timestamp': self.stamp - round(old_s * 1e9)} if metadata else None)

    def tick(self, now, *, records=None):
        self.advance(now - self.now)
        self.seq += 1
        self.motion()
        self.statuses(records)
        self._on_actuator(SimpleNamespace(data=json.dumps(actuator(self.now, self.seq))))
        for owner in CLOSE_OWNER_TOPICS:
            self.receive(owner)
        self._publish_state()

    def ready(self):
        self._publish_state()
        for now in (10.1, 10.2, 10.3, 10.4, 10.5, 10.61):
            self.tick(now)
        return self._global_stop_result

    def global_report(self):
        return self.messages[-1]['lifecycle']['global_stop_collection']


@pytest.mark.parametrize('goal', [GOAL, None])
def test_real_callback_pipeline_produces_stop_evidence_but_never_finishes_session(monkeypatch, goal):
    node = Harness(monkeypatch, goal=goal)
    result = node.ready()
    assert result.ready and result.evidence.nav_goal_uuid == goal
    report = node.global_report()
    assert report['stop_evidence_ready'] and report['blockers'] == []
    assert report['cleanup_authorized'] is report['actuation_authorized'] is False
    assert report['physical_motion_or_actuator_verified'] is False
    assert node._core._fire_session.phase == 'STOPPING'
    assert node._global_stop.barrier_at == node._close_owner_collector.announced_at
    assert node._global_stop.barrier_ros_ns == node._close_owner_collector.announced_ros_ns


@pytest.mark.parametrize('axis', range(6))
@pytest.mark.parametrize('moving', [False, True])
def test_real_motion_callbacks_do_not_discard_unused_axes(monkeypatch, axis, moving):
    node = Harness(monkeypatch)
    assert node.ready().ready
    node.advance(.05)
    node.motion(axis=axis, moving=moving)
    node._publish_state()
    assert not node._global_stop_result.ready
    assert node._global_stop._hold_since is None


@pytest.mark.parametrize('role', ['cmd', 'nav_status'])
@pytest.mark.parametrize('metadata', [None, {}, {'source_timestamp': 0}, {'source_timestamp': True}])
def test_unstamped_inputs_fail_closed_without_receipt_time_fallback(monkeypatch, role, metadata):
    node = Harness(monkeypatch)
    assert node.ready().ready
    node.advance(.05)
    if role == 'cmd':
        node._on_stop_cmd(Twist(), metadata)
    else:
        node._on_nav_status(GoalStatusArray(), metadata)
    node._publish_state()
    assert not node._global_stop_result.ready
    assert node._global_stop.samples[role] is None


@pytest.mark.parametrize('old_s', [0.4, 2.0])
def test_latched_old_terminal_status_cannot_be_refreshed_by_receiving_it_again(monkeypatch, old_s):
    node = Harness(monkeypatch)
    assert node.ready().ready
    node.advance(.05)
    node.statuses(old_s=old_s)
    node._publish_state()
    assert not node._global_stop_result.ready
    assert node._global_stop.reasons['nav_status'] == 'SOURCE_STALE'


@pytest.mark.parametrize('role', STOP_SOURCES)
@pytest.mark.parametrize('change', ['duplicate', 'name', 'namespace', 'type', 'reliability', 'durability', 'gid'])
def test_global_graph_rechecks_every_endpoint_in_same_publish_tick(monkeypatch, role, change):
    node = Harness(monkeypatch)
    assert node.ready().ready
    node.advance(.05)
    topic = CLOSE_OWNER_TOPICS[role] if role in CLOSE_OWNER_TOPICS else STOP_INPUTS[role][0]
    entry = node.infos[topic][0]
    if change == 'duplicate':
        node.infos[topic].append(deepcopy(entry))
    elif change == 'name':
        entry.node_name = 'other'
    elif change == 'namespace':
        entry.node_namespace = '/other'
    elif change == 'type':
        entry.topic_type = 'std_msgs/msg/Bool'
    elif change == 'reliability':
        entry.qos_profile.reliability = (ReliabilityPolicy.BEST_EFFORT if role == 'nav_status'
                                         else ReliabilityPolicy.RELIABLE)
    elif change == 'durability':
        entry.qos_profile.durability = (DurabilityPolicy.VOLATILE if role == 'nav_status'
                                        else DurabilityPolicy.TRANSIENT_LOCAL)
    else:
        entry.endpoint_gid = [0] * 24
    node._publish_state()
    assert not node._global_stop_result.ready


def test_process_replacement_is_detected_from_graph_even_without_new_report(monkeypatch):
    node = Harness(monkeypatch)
    assert node.ready().ready
    node.advance(.05)
    role = 'live_alignment_takeover'
    node.infos[CLOSE_OWNER_TOPICS[role]][0].endpoint_gid = [255] * 24
    node._publish_state()
    assert node._global_stop.fault == 'GLOBAL_SOURCE_CHANGED:' + role
    assert not node._global_stop_result.ready


def test_snapshot_work_delay_and_pump_query_are_accounted_for_once(monkeypatch):
    node = Harness(monkeypatch)
    assert node.ready().ready
    node.advance(.05)
    node.events.clear()
    node.graph_delay = .05  # Eight sources: >350ms work, no actual sleep.
    node._publish_state()
    assert not node._global_stop_result.ready
    assert node._global_stop.graph is None
    assert node.events.count(('count', '/pump_cmd')) == 1
    assert node.events.count(('count', '/cmd_vel_raw')) == 1
    assert len([event for event in node.events if isinstance(event, tuple) and event[0] == 'endpoint']) == 8


def test_actuator_context_change_and_wrong_frames_block_global_proof(monkeypatch):
    node = Harness(monkeypatch)
    assert node.ready().ready
    node.advance(.05)
    node.motion(frame='map')
    node._publish_state()
    assert not node._global_stop_result.ready
    node.advance(.05)
    payload = actuator(node.now, 100, epoch=2)
    node._on_actuator(SimpleNamespace(data=json.dumps(payload)))
    assert node._global_stop.fault == 'GLOBAL_ACTUATOR_CONTEXT_CHANGED'


@pytest.mark.parametrize('records', [[(OTHER, 5)], [(GOAL, 5), (OTHER, 2)], [(GOAL, 3)]])
def test_actual_nav_callback_keeps_exact_goal_and_all_active_records(monkeypatch, records):
    node = Harness(monkeypatch)
    assert node.ready().ready
    node.tick(10.7, records=records)
    assert not node._global_stop_result.ready


def test_failed_first_publish_does_not_announce_either_collector(monkeypatch):
    node = Harness(monkeypatch)
    node.fail_publish = True
    with pytest.raises(RuntimeError):
        node._publish_state()
    assert node._close_owner_collector.request is node._global_stop.request is None


@pytest.mark.parametrize('mode', [
    'off', 'selected', 'tight_limits', 'bad_bool', 'owners_off', 'sim_time',
    'odom_remap', 'raw_remap', 'act_remap', 'status_remap',
])
def test_actual_constructor_selects_only_one_extra_subscription_and_rejects_bad_config(monkeypatch, mode):
    from rclpy.node import Node
    cls = FireMissionSupervisor
    params = {'use_sim_time': False}
    subscriptions, publishers, timers = [], [], []
    overrides = {} if mode == 'off' else {
        'enable_fire_session_observer': True, 'enable_close_owner_collection': True,
        'close_owner_manifest': CLOSE_OWNER_MANIFEST, 'enable_global_stop_collection': True,
    }
    if mode == 'bad_bool':
        overrides['enable_global_stop_collection'] = 'false'
    elif mode == 'owners_off':
        overrides['enable_close_owner_collection'] = False
    elif mode == 'sim_time':
        params['use_sim_time'] = True
    elif mode == 'tight_limits':
        overrides.update(actuator_timeout_s=.2, max_future_skew_ms=5.0,
                         stationary_linear_speed_m_s=.01, stationary_angular_speed_rad_s=.03)
    elif mode.endswith('_remap'):
        name = {'odom_remap': 'odom_topic', 'raw_remap': 'raw_command_topic',
                'act_remap': 'actuator_state_topic', 'status_remap': 'nav_status_topic'}[mode]
        overrides[name] = '/other'
    monkeypatch.setattr(Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(cls, 'declare_parameter',
                        lambda self, name, value: params.setdefault(name, overrides.get(name, value)))
    monkeypatch.setattr(cls, 'get_parameter', lambda self, name: SimpleNamespace(value=params[name]))
    monkeypatch.setattr(cls, 'create_publisher',
                        lambda self, kind, topic, qos: (publishers.append(topic), object())[1])
    def subscribe(self, kind, topic, callback, qos):
        sub = SimpleNamespace(topic=topic, callback=callback)
        subscriptions.append(sub)
        return sub
    monkeypatch.setattr(cls, 'create_subscription', subscribe)
    monkeypatch.setattr(cls, 'create_timer', lambda self, period, callback: timers.append(period))
    monkeypatch.setattr(cls, 'get_logger', lambda self: SimpleNamespace(warning=lambda msg: None, info=lambda msg: None))
    if mode not in ('off', 'selected', 'tight_limits'):
        with pytest.raises(ValueError):
            cls()
        assert not subscriptions and not publishers and not timers
        return
    node = cls()
    selected = mode != 'off'
    assert (node._global_stop is not None) is selected
    assert len(subscriptions) == (14 if selected else 10)
    assert publishers == ['/fire_mission/state'] and len(timers) == 1
    marked = [sub.topic for sub in subscriptions if hasattr(sub, '_fire_stop_metadata_callback')]
    assert marked == (['/navigate_to_pose/_action/status', '/cmd_vel'] if selected else [])
    if mode == 'tight_limits':
        assert node._global_stop.timeout_s == node._close_owner_collector.timeout_s == .2
        assert node._global_stop.future_ns == node._close_owner_collector.future_ns == 5_000_000
        assert node._global_stop.linear_limit == .01 and node._global_stop.angular_limit == .03


@pytest.mark.parametrize('selected', [False, True])
def test_main_selects_metadata_executor_only_for_selected_mode_and_cleans_up(monkeypatch, selected):
    from fire_robot_mission import stop_metadata_executor
    events = []
    node = SimpleNamespace(enable_global_stop_collection=selected,
                           destroy_node=lambda: events.append('destroy_node'))
    def stop():
        raise KeyboardInterrupt
    class Executor:
        def __init__(self):
            events.append('executor')
        def add_node(self, added):
            assert added is node
        def spin(self):
            stop()
        def shutdown(self):
            events.append('executor_shutdown')
    monkeypatch.setattr(module, 'FireMissionSupervisor', lambda: node)
    monkeypatch.setattr(module.rclpy, 'init', lambda args: events.append('init'))
    monkeypatch.setattr(module.rclpy, 'spin', lambda node: stop())
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'shutdown', lambda: events.append('shutdown'))
    monkeypatch.setattr(stop_metadata_executor, 'StopMetadataExecutor', Executor)
    module.main([])
    assert events == (['init', 'executor', 'executor_shutdown', 'destroy_node', 'shutdown']
                      if selected else ['init', 'destroy_node', 'shutdown'])
