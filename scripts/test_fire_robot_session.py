"""Pure/offline checks for the one-command orchestration gates."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest


MODULE_PATH = Path(__file__).with_name('fire_robot_session.py')
SPEC = importlib.util.spec_from_file_location('fire_robot_session', MODULE_PATH)
SESSION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SESSION)
UUID = 'a' * 32


@pytest.mark.parametrize(('output', 'expected'), [
    ('active [3]\n', True),
    ('inactive [2]\n', False),
    ('unconfigured [1]\n', False),
    ('inactive [3]\n', False),
    ('active [2]\n', False),
    ('/bt_navigator: active [3]\n', False),
])
def test_lifecycle_readiness_requires_exact_active_state(monkeypatch,
                                                         output, expected):
    calls = []
    def fake_run(args, **kwargs):
        calls.append(args)
        return SimpleNamespace(stdout=output)
    monkeypatch.setattr(SESSION, 'run', fake_run)
    assert SESSION.lifecycle_active('/bt_navigator') is expected
    assert calls == [['ros2', 'lifecycle', 'get', '/bt_navigator']]


def test_no_fire_ready_message_requires_gateway_clear_proof():
    gateway = {
        'state': 'RELEASED_STOPPED',
        'stop_reason': 'NAV_GOAL_TERMINAL_NO_FIRE',
        'cycle_goal_uuid': UUID,
    }
    assert not SESSION.no_fire_goal_ready(gateway)
    gateway['next_goal_clear_ready'] = False
    assert not SESSION.no_fire_goal_ready(gateway)
    gateway['next_goal_clear_ready'] = True
    assert SESSION.no_fire_goal_ready(gateway)
    gateway['state'] = 'STOPPING'
    assert not SESSION.no_fire_goal_ready(gateway)


def evidence():
    """Build a complete synthetic proof for a centered stopped goal."""
    ir = {
        'goal_uuid': UUID, 'state': 'COMPLETE',
        'stop_reason': 'SPRAY_RANGE_REACHED', 'publisher_owned': False,
        'step_count': 1,
    }
    guard = {
        'goal_uuid': UUID, 'state': 'CANCEL_CONFIRMED_STOPPED',
        'goal_status': 5, 'raw_cmd_publishers': 0, 'blockers': [],
    }
    gateway = {
        'state': 'HANDOFF_READY', 'cycle_goal_uuid': UUID,
        'handoff_ready': True, 'cmd_vel_fresh_zero': True,
        'odom_stationary': True,
    }
    alignment = {
        'state': 'COMPLETE', 'stop_reason': 'TARGET_CENTERED',
        'goal_uuid': UUID, 'publisher_owned': False,
    }
    mission = {
        'state': 'CENTERED_HOLD',
        'perception': {'alignment_state': 'CENTERED'},
        'robot': {'stationary': True},
    }
    preview = {
        'zone': 'SPRAY_RANGE', 'ir_band': 'SPRAY_RANGE', 'blockers': [],
    }
    return ir, guard, gateway, alignment, mission, preview


def admitted(values):
    """Evaluate the pump sequencing gate with no ROS publishers."""
    with patch.object(SESSION, 'publisher_count', return_value=0):
        return SESSION.pump_admission(*values)


def test_pump_only_after_exact_stopped_ir_handoff():
    """Reject each broken link of the exact-goal proof chain."""
    values = evidence()
    assert admitted(values)
    for index, field, invalid in (
        (0, 'publisher_owned', True),
        (0, 'stop_reason', 'STEP_TIME_LIMIT'),
        (1, 'state', 'IR_APPROACH_IN_PROGRESS'),
        (1, 'goal_uuid', 'b' * 32),
        (2, 'state', 'RELEASED_STOPPED'),
        (3, 'stop_reason', 'TURN_TIME_LIMIT'),
        (4, 'state', 'FIRE_TRACKING'),
        (5, 'zone', 'APPROACH'),
    ):
        changed = list(evidence())
        changed[index][field] = invalid
        assert not admitted(changed)


def test_target_wait_is_not_a_terminal_alignment_or_pump_proof():
    values = list(evidence())
    values[3] = {
        'state': 'TURNING', 'goal_uuid': UUID,
        'stop_reason': '', 'publisher_owned': True,
        'target_reacquiring': True,
        'command': {'linear_x_m_s': 0.0, 'angular_z_rad_s': 0.0},
    }
    assert not admitted(values)
    outcome, detail = SESSION.alignment_demo_outcome(
        values[3], values[1], values[2], 1)
    assert (outcome, detail) == (None, None)


def test_pump_rejects_any_live_motion_or_existing_pump_owner():
    """Never allow pump startup with motion or pump ownership active."""
    for topic in ('/cmd_vel_raw', '/pump_cmd'):
        with patch.object(SESSION, 'publisher_count',
                          side_effect=lambda value: int(value == topic)):
            assert not SESSION.pump_admission(*evidence())


def test_only_expected_off_pump_results_continue_to_post_spray_review():
    for state, reason in (
            ('COMPLETE', 'PULSE_TIME_LIMIT'),
            ('ABORTED', 'IR_NOT_CONFIRMED_SPRAY_RANGE'),
            ('ABORTED', 'MISSION_NOT_SAFE_AND_CENTERED')):
        result = {'goal_uuid': UUID, 'state': state,
                  'stop_reason': reason, 'pump_on': False}
        assert SESSION.reviewable_pump_result(result, True, UUID)
        assert not SESSION.reviewable_pump_result(result, False, UUID)
        assert not SESSION.reviewable_pump_result(result, True, 'b' * 32)
        assert not SESSION.reviewable_pump_result(
            dict(result, pump_on=True), True, UUID)
    assert not SESSION.reviewable_pump_result(
        {'goal_uuid': UUID, 'state': 'ABORTED',
         'stop_reason': 'PUBLISHER_OR_ARM_LOST', 'pump_on': False}, True, UUID)


def test_direct_spray_range_without_ir_motion_is_admissible():
    """Permit a centered flame already in range without a needless jog."""
    values = list(evidence())
    values[0] = {
        'state': 'WAITING_FOR_ADMISSION', 'publisher_owned': False,
        'step_count': 0,
    }
    assert admitted(values)
    values[0]['step_count'] = 1
    assert not admitted(values)


@pytest.mark.parametrize('reason', [
    'SPRAY_RANGE_REACHED', 'STEP_DISTANCE_REACHED',
    'STEP_PROGRESS_TIME_LIMIT',
])
def test_completed_ir_step_can_handoff_to_pump(reason):
    """Allow a completed, released IR step when the range is now sprayable."""
    values = list(evidence())
    values[0]['stop_reason'] = reason
    assert admitted(values)
    values[0]['step_count'] = 0
    assert not admitted(values)


def test_continuous_ir_pump_handoff_requires_confirmed_spray_stop():
    values = list(evidence())
    values[0]['continuous_approach'] = True
    assert admitted(values)
    for reason in ('SPRAY_CANDIDATE_STOP', 'APPROACH_DISTANCE_LIMIT',
                   'APPROACH_TIME_LIMIT', 'STEP_DISTANCE_REACHED'):
        changed = list(evidence())
        changed[0]['continuous_approach'] = True
        changed[0]['stop_reason'] = reason
        assert not admitted(changed)


def test_ir_only_outcome_never_claims_success_before_raw_release():
    ir, guard, gateway, _, _, preview = evidence()
    ir['continuous_approach'] = True
    assert SESSION.ir_demo_outcome(
        ir, guard, gateway, preview, 0) == ('COMPLETE', UUID)
    assert SESSION.ir_demo_outcome(
        ir, guard, gateway, preview, 1) == (None, None)
    ir['state'] = 'ABORTED'
    ir['stop_reason'] = 'APPROACH_DISTANCE_LIMIT'
    assert SESSION.ir_demo_outcome(
        ir, guard, gateway, preview, 0) == (
        'FAILED', 'APPROACH_DISTANCE_LIMIT')
    ir['state'] = 'COMPLETE'
    ir['stop_reason'] = 'REALIGN_REQUIRED'
    assert SESSION.ir_demo_outcome(
        ir, guard, gateway, preview, 0) == (None, None)
    ir['stop_reason'] = 'APPROACH_DISTANCE_LIMIT'
    assert SESSION.ir_demo_outcome(
        ir, guard, gateway, preview, 0) == (None, None)


@pytest.mark.parametrize('reason,category', [
    ('APPROACH_DISTANCE_LIMIT', 'APPROACH_BUDGET'),
    ('SCAN_OBSTACLE_OR_INSUFFICIENT_MARGIN',
     'FRONT_CLEARANCE_OR_SCAN'),
    ('IR_BAND_INVALID_OR_STALE', 'IR_OR_TARGET_EVIDENCE'),
    ('EXACT_HANDOFF_PROOF_INVALID', 'TARGET_OR_HANDOFF_PROOF'),
    ('ODOM_PROGRESS_LOST', 'MOTION_FEEDBACK_OR_SAFETY'),
])
def test_ir_failure_event_preserves_evidence_without_authorizing_motion(
        reason, category):
    ir = {'goal_uuid': UUID, 'stop_reason': reason,
          'stop_trigger_blockers': [reason], 'ir_band': 'UNKNOWN'}
    preview = {'zone': 'UNKNOWN', 'blockers': ['FIRE_NOT_CENTERED']}
    mission = {'state': 'FIRE_TRACKING',
               'perception': {'alignment_state': 'NO_TARGET'}}
    details = SESSION.ir_failure_evidence(ir, preview, mission)
    assert details['category'] == category
    assert details['goal_uuid'] == UUID
    assert details['preview_blockers'] == ['FIRE_NOT_CENTERED']
    assert details['stop_trigger_blockers'] == [reason]
    assert details['recovery'] == 'STOPPED_OPERATOR_REVIEW_REQUIRED'


def test_ir_stop_without_release_proof_escalates_but_never_completes():
    assert not SESSION.ir_stop_proof_overdue('STOPPING', 10.0, 13.9)
    assert SESSION.ir_stop_proof_overdue('STOPPING', 10.0, 14.0)
    assert not SESSION.ir_stop_proof_overdue('COMPLETE', 10.0, 20.0)
    assert not SESSION.ir_stop_proof_overdue('STOPPING', None, 20.0)


def test_ir_telemetry_silence_is_a_terminal_session_fault():
    assert not SESSION.ir_telemetry_overdue({}, 10.0, 11.0)
    assert not SESSION.ir_telemetry_overdue(None, 10.0, 10.9)
    assert SESSION.ir_telemetry_overdue(None, 10.0, 11.0)


def test_selected_map_must_be_bare_filename(tmp_path):
    """The mapping marker cannot inject arbitrary map paths."""
    marker = tmp_path / 'selected_map.txt'
    with patch.object(SESSION, 'SELECTED_MAP', marker):
        marker.write_text('fire_room_demo.yaml\n', encoding='utf-8')
        assert SESSION.selected_map() == SESSION.MAPS / 'fire_room_demo.yaml'
        marker.write_text('../other.yaml\n', encoding='utf-8')
        with pytest.raises(SESSION.SessionError):
            SESSION.selected_map()


def test_unknown_arm_topic_is_zero_publishers():
    """Absent optional arm topics must not crash an idle preflight."""
    with patch.object(SESSION, 'run', return_value=SimpleNamespace(
            returncode=1,
            stdout="Unknown topic '/fire_mission/ir_approach_arm'\n")):
        assert SESSION.publisher_count('/fire_mission/ir_approach_arm') == 0


def test_publisher_count_does_not_hide_ros_cli_failures():
    """A transport/CLI failure must never look like zero publishers."""
    with patch.object(SESSION, 'run', return_value=SimpleNamespace(
            returncode=1, stdout='DDS graph unavailable\n')):
        with pytest.raises(SESSION.SessionError):
            SESSION.publisher_count('/fire_mission/live_nav_arm')


def test_existing_publisher_count_is_preserved():
    """Still detect a competing arm publisher instead of bypassing the gate."""
    with patch.object(SESSION, 'run', return_value=SimpleNamespace(
            returncode=0, stdout='Type: std_msgs/msg/Bool\n'
            'Publisher count: 1\nSubscription count: 1\n')):
        assert SESSION.publisher_count('/fire_mission/live_nav_arm') == 1


def test_ir_arm_ack_rejects_missing_edge_stale_or_extra_publisher():
    arm = {
        'false_seen': True, 'cycle_seen': True,
        'heartbeat_true': True, 'receive_fresh': True,
        'publisher_count': 1,
    }
    report = {'schema_version': 1, 'arm': arm}
    assert SESSION.ir_arm_stage_ready(report, armed=True)
    assert not SESSION.ir_arm_stage_ready(report, armed=False)
    false_report = {'schema_version': 1, 'arm': dict(
        arm, cycle_seen=False, heartbeat_true=False)}
    assert SESSION.ir_arm_stage_ready(false_report, armed=False)
    for key, invalid in (
            ('false_seen', False), ('cycle_seen', False),
            ('heartbeat_true', False), ('receive_fresh', False),
            ('publisher_count', 2)):
        broken = {'schema_version': 1, 'arm': dict(arm, **{key: invalid})}
        assert not SESSION.ir_arm_stage_ready(broken, armed=True)
    assert not SESSION.ir_arm_stage_ready(None, armed=True)
    assert not SESSION.ir_arm_stage_ready({'schema_version': 1}, armed=True)
    assert not SESSION.ir_arm_stage_ready({
        'schema_version': 1,
        'arm': dict(false_report['arm'], receive_fresh=False)}, armed=False)


def test_ir_arm_loss_warning_requires_exact_stopped_handoff():
    ir = {'state': 'WAITING_FOR_ADMISSION',
          'blockers': ['OPERATOR_ARM_MISSING_OR_STALE']}
    guard = {'state': 'CANCEL_CONFIRMED_STOPPED',
             'goal_uuid': UUID, 'blockers': []}
    gateway = {'state': 'HANDOFF_READY', 'cycle_goal_uuid': UUID}
    alignment = {'state': 'COMPLETE', 'goal_uuid': UUID,
                 'stop_reason': 'TARGET_CENTERED'}
    assert SESSION.ir_arm_missing_after_handoff(
        ir, guard, gateway, alignment)
    assert not SESSION.ir_arm_missing_after_handoff(
        dict(ir, blockers=[]), guard, gateway, alignment)
    assert not SESSION.ir_arm_missing_after_handoff(
        dict(ir, state='MOVING'), guard, gateway, alignment)
    assert not SESSION.ir_arm_missing_after_handoff(
        ir, guard, dict(gateway, cycle_goal_uuid='b' * 32), alignment)
    assert not SESSION.ir_arm_missing_after_handoff(
        ir, dict(guard, state='ALIGNMENT_IN_PROGRESS'), gateway, alignment)
    assert not SESSION.ir_arm_missing_after_handoff(
        ir, guard, gateway, dict(alignment, stop_reason='TARGET_LOST'))


def test_ir_arm_false_then_true_requires_subscriber_ack_before_ready():
    events = []
    phase = ['initial']
    false_process = object()

    class Monitor:
        def latest(self, topic):
            assert topic == '/fire_mission/ir_approach_live_state'
            return {'schema_version': 1, 'arm': {
                'false_seen': phase[0] != 'initial',
                'cycle_seen': phase[0] == 'true',
                'heartbeat_true': phase[0] == 'true',
                'receive_fresh': True,
                'publisher_count': int(phase[0] != 'released'),
            }}

    class FakeSession:
        monitor = Monitor()

        def start(self, label, cmd, *, arm):
            assert label == 'ir_arm_false' and arm is True
            assert '{data: false}' in cmd
            events.append('false_started')
            phase[0] = 'false'
            return false_process

        def remove(self, process):
            assert process is false_process
            events.append('false_removed')
            phase[0] = 'released'

        def event(self, name):
            events.append(name)

    def wait(label, predicate, session, timeout):
        assert timeout == 8
        assert predicate(), label
        events.append(label)

    def true(session, topic, label):
        assert phase[0] == 'released'
        events.append('true_started')
        phase[0] = 'true'

    with patch.object(SESSION, 'wait_until', side_effect=wait), \
            patch.object(SESSION, 'publisher_count', return_value=0), \
            patch.object(SESSION, 'arm_true', side_effect=true):
        SESSION.arm_ir_with_ack(FakeSession())
    assert events == [
        'false_started', 'IR arm false đã tới node', 'false_removed',
        'IR arm false publisher đã nhả', 'true_started',
        'IR arm true đã tới node', 'IR_ARM_ACKNOWLEDGED',
    ]


def test_ir_arm_ack_timeout_removes_false_and_never_starts_true():
    false_process = object()
    removed = []
    fake = SimpleNamespace(
        start=lambda *args, **kwargs: false_process,
        remove=lambda process: removed.append(process),
        monitor=SimpleNamespace(latest=lambda topic: None))
    with patch.object(SESSION, 'wait_until', side_effect=SESSION.SessionError(
            'Không thấy false')), patch.object(SESSION, 'arm_true') as true:
        with pytest.raises(SESSION.SessionError):
            SESSION.arm_ir_with_ack(fake)
        true.assert_not_called()
    assert removed == [false_process]


def test_stuck_arm_publisher_remains_owned_and_blocks_next_edge():
    process = SimpleNamespace(pid=1234, poll=lambda: None)
    session = SESSION.Session.__new__(SESSION.Session)
    session.processes = [('ir_arm_false', process)]
    session.arms = [('ir_arm_false', process)]
    with patch.object(session, 'stop'):
        with pytest.raises(SESSION.SessionError, match='chưa thoát'):
            session.remove(process)
    assert session.processes == [('ir_arm_false', process)]
    assert session.arms == [('ir_arm_false', process)]


def test_map_validation_rejects_missing_image(tmp_path):
    """Do not start Nav2 from a YAML map with a missing PGM."""
    path = tmp_path / 'room.yaml'
    path.write_text('image: room.pgm\n', encoding='utf-8')
    with pytest.raises(SESSION.SessionError):
        SESSION.validate_map(str(path))
    (tmp_path / 'room.pgm').write_bytes(b'P5\n1 1\n255\n\0')
    assert SESSION.validate_map(str(path)) == path


def test_align_demo_waits_for_post_turn_stop_proof():
    """A centered state alone is not enough while guard is settling."""
    alignment = {'state': 'COMPLETE', 'goal_uuid': UUID,
                 'stop_reason': 'TARGET_CENTERED'}
    guard = {'state': 'WAITING_FOR_ALIGNMENT_STOP', 'goal_uuid': UUID}
    gateway = {'state': 'HANDOFF_READY', 'cycle_goal_uuid': UUID}
    assert SESSION.alignment_demo_outcome(
        alignment, guard, gateway, 0) == (None, None)
    guard['state'] = 'CANCEL_CONFIRMED_STOPPED'
    assert SESSION.alignment_demo_outcome(
        alignment, guard, gateway, 0) == ('COMPLETE', UUID)
    assert SESSION.alignment_demo_outcome(
        alignment, guard, gateway, 1) == (None, None)


def test_align_demo_reports_target_loss_without_auto_exit():
    """Return terminal failure for the observed lost-flame attempt."""
    alignment = {'state': 'COMPLETE', 'goal_uuid': UUID,
                 'stop_reason': 'TARGET_NOT_REACQUIRED'}
    guard = {'state': 'WAITING_FOR_ALIGNMENT_STOP', 'goal_uuid': UUID}
    gateway = {'state': 'RELEASED_STOPPED',
               'stop_reason': 'FIRE_ACQUIRED'}
    assert SESSION.alignment_demo_outcome(
        alignment, guard, gateway, 0) == (
            'FAILED', 'Alignment: TARGET_NOT_REACQUIRED')


def test_startup_requires_continuously_safe_window():
    window = SESSION.StableSafeWindow(1.0)
    assert not window.observe(True, 0.0)
    assert not window.observe(True, 0.8)
    assert not window.observe(False, 0.9)
    assert not window.observe(True, 1.0)
    assert window.observe(True, 2.01)
    safe = ({'state': 'WAITING_FOR_INPUT', 'fire_latched': False,
             'cmd_vel_fresh_zero': True, 'odom_stationary': True},
            {'state': 'WAITING_FOR_ADMISSION'},
            {'state': 'IDLE', 'health': {'status': 'OK'}})
    assert SESSION.safe_initial_state(*safe, 0, 0)
    assert not SESSION.safe_initial_state(*safe, 1, 0)
    assert not SESSION.safe_initial_state(*safe, 0, 1)
    bad = dict(safe[2], state='FAULT')
    assert not SESSION.safe_initial_state(safe[0], safe[1], bad, 0, 0)


def test_startup_readiness_uses_fresh_node_telemetry_not_graph_snapshots():
    states = {
        '/fire_mission/live_nav_stop_state': 'WAITING_FOR_INPUT',
        '/fire_mission/nav2_cancel_guard_state': 'WAITING_FOR_ADMISSION',
        '/fire_mission/live_alignment_state': 'WAITING_FOR_ADMISSION',
        '/fire_mission/ir_approach_live_state': 'WAITING_FOR_ADMISSION',
        '/fire_mission/post_spray_review': 'WAITING_FOR_PULSE',
    }
    reports = {topic: {'schema_version': 1, 'state': state,
                       'enabled': True}
               for topic, state in states.items()}
    monitor = SimpleNamespace(latest=lambda topic: reports.get(topic))
    with patch.object(SESSION, 'nodes', side_effect=SESSION.SessionError(
            'ROS graph CLI unavailable')):
        assert SESSION.startup_state_gaps(monitor, False, True) == []
        assert SESSION.startup_state_gaps(monitor, False, False) == []
    reports.pop('/fire_mission/ir_approach_live_state')
    assert SESSION.startup_state_gaps(monitor, False, True)
    assert SESSION.startup_state_gaps(monitor, True, False) == []
    reports.pop('/fire_mission/post_spray_review')
    assert SESSION.startup_state_gaps(monitor, False, False)
    assert SESSION.startup_state_gaps(monitor, True, False) == []


def test_early_graph_readiness_uses_one_snapshot_per_attempt():
    with patch.object(SESSION, 'nodes', side_effect=[
            ['/yolo_observer', '/fire_alignment_observer'], []]) as listing:
        assert SESSION.nodes_present(
            '/yolo_observer', '/fire_alignment_observer')
        assert listing.call_count == 1


def test_startup_readiness_rejects_stale_invalid_or_disabled_state():
    reports = {
        '/fire_mission/live_nav_stop_state': {
            'schema_version': 1, 'state': 'WAITING_FOR_INPUT',
            'enabled': True},
        '/fire_mission/nav2_cancel_guard_state': {
            'schema_version': 1, 'state': 'WAITING_FOR_ADMISSION',
            'enabled': True},
        '/fire_mission/live_alignment_state': {
            'schema_version': 1, 'state': 'WAITING_FOR_ADMISSION',
            'enabled': True},
    }
    monitor = SimpleNamespace(latest=lambda topic: reports.get(topic))
    assert SESSION.startup_state_gaps(monitor, True, False) == []
    for broken in (
            None,
            {'schema_version': 2, 'state': 'WAITING_FOR_INPUT',
             'enabled': True},
            {'schema_version': 1, 'state': 'NAVIGATION', 'enabled': True},
            {'schema_version': 1, 'state': 'WAITING_FOR_INPUT',
             'enabled': False}):
        reports['/fire_mission/live_nav_stop_state'] = broken
        assert SESSION.startup_state_gaps(monitor, True, False)


def test_startup_wait_reports_missing_topic_and_fails_on_dead_child():
    monitor = SimpleNamespace(latest=lambda topic: None)
    session = SimpleNamespace(monitor=monitor, assert_alive=lambda: None)
    with pytest.raises(SESSION.SessionError,
                       match='/fire_mission/live_nav_stop_state'):
        SESSION.wait_for_startup_states(session, True, False, timeout=0)
    session.assert_alive = lambda: (_ for _ in ()).throw(
        SESSION.SessionError('gateway thoát'))
    with pytest.raises(SESSION.SessionError, match='gateway thoát'):
        SESSION.wait_for_startup_states(session, True, False, timeout=1)


def test_readiness_timeout_keeps_latest_ros_graph_error():
    with patch.object(SESSION.time, 'monotonic', side_effect=[0, 0.1, 1]), \
            patch.object(SESSION.time, 'sleep'):
        with pytest.raises(SESSION.SessionError,
                           match='DDS graph unavailable'):
            SESSION.wait_until('startup', lambda: (_ for _ in ()).throw(
                SESSION.SessionError('DDS graph unavailable')), timeout=1)


def test_startup_preflight_resets_on_fault_and_graph_query_error():
    reports = {
        '/fire_mission/live_nav_stop_state': {
            'state': 'WAITING_FOR_INPUT', 'fire_latched': False,
            'cmd_vel_fresh_zero': True, 'odom_stationary': True},
        '/fire_mission/nav2_cancel_guard_state': {
            'state': 'WAITING_FOR_ADMISSION'},
        '/fire_mission/state': {
            'state': 'IDLE', 'health': {'status': 'OK'}},
    }
    monitor = SimpleNamespace(latest=lambda topic: reports.get(topic))
    window = SESSION.StableSafeWindow(1.0)
    with patch.object(SESSION, 'publisher_count', return_value=0) as count:
        assert not SESSION.startup_preflight_stable(monitor, window, now=0.0)
        reports['/fire_mission/state']['state'] = 'FAULT'
        assert not SESSION.startup_preflight_stable(monitor, window, now=0.5)
        assert count.call_count == 2  # no CLI graph load while in FAULT
        reports['/fire_mission/state']['state'] = 'IDLE'
        assert not SESSION.startup_preflight_stable(monitor, window, now=1.0)
        assert SESSION.startup_preflight_stable(monitor, window, now=2.1)
    with patch.object(SESSION, 'publisher_count', side_effect=
                      SESSION.SessionError('graph unavailable')):
        with pytest.raises(SESSION.SessionError):
            SESSION.startup_preflight_stable(monitor, window, now=2.2)
    with patch.object(SESSION, 'publisher_count', return_value=0):
        assert not SESSION.startup_preflight_stable(monitor, window, now=3.3)
        assert SESSION.startup_preflight_stable(monitor, window, now=4.4)


def test_startup_preflight_rechecks_state_after_slow_graph_queries():
    reports = {
        '/fire_mission/live_nav_stop_state': {
            'state': 'WAITING_FOR_INPUT', 'fire_latched': False,
            'cmd_vel_fresh_zero': True, 'odom_stationary': True},
        '/fire_mission/nav2_cancel_guard_state': {
            'state': 'WAITING_FOR_ADMISSION'},
        '/fire_mission/state': {
            'state': 'IDLE', 'health': {'status': 'OK'}},
    }
    monitor = SimpleNamespace(latest=lambda topic: reports.get(topic))
    window = SESSION.StableSafeWindow(1.0)

    def query(topic):
        if topic == '/pump_cmd':
            reports['/fire_mission/state'] = {'state': 'FAULT'}
        return 0

    with patch.object(SESSION, 'publisher_count', side_effect=query):
        assert not SESSION.startup_preflight_stable(monitor, window, now=2.0)
    assert window.since is None


def test_pre_spray_wait_only_for_unpumped_target_loss():
    mission = {'state': 'TARGET_LOST',
               'perception': {'detected_now': False, 'confirmed': False}}
    gateway = {'cycle_goal_uuid': UUID, 'fire_latched': True,
               'stop_reason': 'FIRE_ACQUIRED'}
    assert SESSION.pre_spray_clear_waiting(mission, gateway, set())
    assert not SESSION.pre_spray_clear_waiting(mission, gateway, {UUID})
    assert not SESSION.pre_spray_clear_waiting(
        dict(mission, state='FAULT'), gateway, set())
    assert not SESSION.pre_spray_clear_waiting(
        mission, dict(gateway, fire_latched=False), set())


def test_target_loss_survives_many_monitor_cycles_then_reports_reacquire():
    """Target-loss monitoring has no deadline that withdraws all arms."""
    gateway = {'cycle_goal_uuid': UUID, 'fire_latched': True,
               'stop_reason': 'FIRE_ACQUIRED'}
    lost = {'state': 'TARGET_LOST',
            'perception': {'detected_now': False, 'confirmed': False}}
    waiting, event = SESSION.target_recovery_event(None, lost, gateway, set())
    assert (waiting, event) == (UUID, 'TARGET_REACQUIRE_WAITING')
    for _ in range(200):
        waiting, event = SESSION.target_recovery_event(
            waiting, lost, gateway, set())
        assert (waiting, event) == (UUID, None)
    candidate = {'state': 'FIRE_CANDIDATE',
                 'perception': {'detected_now': True, 'confirmed': False}}
    waiting, event = SESSION.target_recovery_event(
        waiting, candidate, gateway, set())
    assert (waiting, event) == (UUID, None)
    tracking = {'state': 'FIRE_TRACKING',
                'perception': {'detected_now': True, 'confirmed': True}}
    waiting, event = SESSION.target_recovery_event(
        waiting, tracking, gateway, set())
    assert (waiting, event) == (None, 'TARGET_REACQUIRED')
    waiting, event = SESSION.target_recovery_event(
        waiting, lost, gateway, set())
    assert (waiting, event) == (UUID, 'TARGET_REACQUIRE_WAITING')


def test_target_recovery_never_confuses_unproven_fire_clear_or_new_goal():
    gateway = {'cycle_goal_uuid': UUID, 'fire_latched': True,
               'stop_reason': 'FIRE_ACQUIRED'}
    unconfirmed = {'state': 'FIRE_TRACKING',
                   'perception': {'detected_now': True, 'confirmed': False}}
    assert SESSION.target_recovery_event(
        UUID, unconfirmed, gateway, set()) == (UUID, None)
    clear = {'state': 'IDLE', 'lifecycle': {
        'pre_spray_clear_goal_uuid': UUID}}
    assert SESSION.target_recovery_event(
        UUID, clear, gateway, set()) == (None, None)
    new_gateway = dict(gateway, cycle_goal_uuid='b' * 32)
    assert SESSION.target_recovery_event(
        UUID, unconfirmed, new_gateway, set()) == (None, None)
    fault = {'state': 'FAULT'}
    assert SESSION.target_recovery_event(
        UUID, fault, gateway, set()) == (UUID, None)


def test_pump_arm_waits_for_subscriber_false_then_true():
    phase = ['initial']
    events = []
    false_process = object()

    def latest(topic):
        assert topic == '/fire_mission/pump_demo_state'
        return {'schema_version': 1, 'arm': {
            'false_seen': phase[0] != 'initial',
            'cycle_seen': phase[0] == 'true',
            'heartbeat_true': phase[0] == 'true',
            'receive_fresh': True,
            'publisher_count': int(phase[0] != 'released'),
        }}

    class FakeSession:
        monitor = SimpleNamespace(latest=latest)

        def start(self, label, cmd, *, arm):
            assert label == 'pump_arm_false' and arm
            assert '{data: false}' in cmd
            phase[0] = 'false'
            events.append('false_started')
            return false_process

        def remove(self, process):
            assert process is false_process
            phase[0] = 'released'
            events.append('false_removed')

        def event(self, name):
            events.append(name)

    def wait(label, predicate, session, timeout):
        assert timeout == 8 and predicate(), label
        events.append(label)

    def true(session, topic, label):
        assert phase[0] == 'released'
        assert topic == '/fire_mission/pump_demo_arm'
        phase[0] = 'true'
        events.append('true_started')

    with patch.object(SESSION, 'wait_until', side_effect=wait), \
            patch.object(SESSION, 'publisher_count', return_value=0), \
            patch.object(SESSION, 'arm_true', side_effect=true):
        SESSION.arm_pump_with_ack(FakeSession())
    assert events == [
        'false_started', 'pump arm false đã tới node', 'false_removed',
        'pump arm false publisher đã nhả', 'true_started',
        'pump arm true đã tới node', 'PUMP_ARM_ACKNOWLEDGED',
    ]


def test_detached_start_releases_capture_while_child_stays_alive(tmp_path):
    """Exercise real shell pipes as used by SSH, without ROS or hardware."""
    import os
    import shlex
    import signal
    import subprocess
    import sys
    import time

    pidfile = tmp_path / 'child.pid'
    logfile = tmp_path / 'log with spaces.txt'
    child = ('import os,time; from pathlib import Path; '
             f'Path({str(pidfile)!r}).write_text(str(os.getpid())); '
             'print("child ready",flush=True); time.sleep(4)')
    command = ('true && export QA_DETACH_FIXTURE=1 && exec ' +
               shlex.join([sys.executable, '-c', child]))
    child_pid = None
    try:
        # If a background setup shell retains either pipe, this times out.
        result = subprocess.run(
            ['bash', '-c', SESSION.detached_shell_command(command, logfile)],
            capture_output=True, text=True, timeout=1.5)
        assert result.returncode == 0
        deadline = time.monotonic() + 1
        while not pidfile.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        assert pidfile.exists()
        child_pid = int(pidfile.read_text())
        os.kill(child_pid, 0)
        assert 'child ready' in logfile.read_text()
        assert result.stdout == '' and result.stderr == ''
    finally:
        # An unsuccessful regression also releases its fixture eventually.
        if child_pid is None and pidfile.exists():
            child_pid = int(pidfile.read_text())
        if child_pid is not None:
            try:
                os.kill(child_pid, signal.SIGTERM)
            except ProcessLookupError:
                pass


def test_detached_setup_failure_is_logged_without_running_launch(tmp_path):
    import shlex
    import subprocess
    import time

    marker = tmp_path / 'unexpected_launch'
    logfile = tmp_path / 'setup.log'
    command = ("(printf 'setup failed\\n' >&2; false) && touch " +
               shlex.quote(str(marker)))
    result = subprocess.run(
        ['bash', '-c', SESSION.detached_shell_command(command, logfile)],
        capture_output=True, text=True, timeout=1.5)
    deadline = time.monotonic() + 1
    while (not logfile.exists() or not logfile.read_text()) and time.monotonic() < deadline:
        time.sleep(.02)
    assert result.returncode == 0  # Background start is not readiness proof.
    assert 'setup failed' in logfile.read_text()
    assert not marker.exists()


def test_ensure_pi_reuses_running_stack_without_ssh(monkeypatch):
    monkeypatch.setattr(SESSION, 'pi_ready', lambda: True)
    def forbidden(*args, **kwargs):
        pytest.fail('A ready Pi must not start another SSH launch')
    monkeypatch.setattr(SESSION, 'run', forbidden)
    SESSION.ensure_pi()


def test_ensure_pi_rejects_partial_stack_without_launch(monkeypatch):
    monkeypatch.setattr(SESSION, 'pi_ready', lambda: False)
    monkeypatch.setenv('FIRE_AUTO_START_PI', '1')
    def wait(*args, **kwargs):
        raise SESSION.SessionError('rediscovery timeout')
    monkeypatch.setattr(SESSION, 'wait_until', wait)
    calls = []
    def run(args, **kwargs):
        calls.append(args)
        return SimpleNamespace(stdout='/serial_bridge_node\n', returncode=0)
    monkeypatch.setattr(SESSION, 'run', run)
    with pytest.raises(SESSION.SessionError, match='Pi đã chạy một phần'):
        SESSION.ensure_pi()
    assert len(calls) == 1
    assert 'nohup' not in calls[0][-1]


def test_ensure_pi_detaches_then_requires_readiness(monkeypatch):
    import shlex
    monkeypatch.setattr(SESSION, 'pi_ready', lambda: False)
    monkeypatch.setenv('FIRE_AUTO_START_PI', '1')
    waits, calls = [], []
    def wait(label, *args, **kwargs):
        waits.append((label, kwargs))
        if len(waits) == 1:
            raise SESSION.SessionError('rediscovery timeout')
        raise SESSION.SessionError('Pi not ready')
    def run(args, **kwargs):
        calls.append(args)
        return SimpleNamespace(stdout='', returncode=0)
    monkeypatch.setattr(SESSION, 'wait_until', wait)
    monkeypatch.setattr(SESSION, 'run', run)
    with pytest.raises(SESSION.SessionError, match='Pi not ready'):
        SESSION.ensure_pi()
    assert len(calls) == 2
    background = shlex.split(calls[1][-1])[-1]
    assert background.startswith('nohup bash -c ')
    assert '&& exec ros2 launch' in background
    assert background.endswith('</dev/null >/tmp/fire_robot_pi_auto.log 2>&1 &')
    assert waits[-1][1]['timeout'] == 60


def test_state_log_keeps_gateway_cycle_uuid_first_stop_and_live_blockers():
    payload = {'state': 'RELEASED_STOPPED', 'cycle_goal_uuid': UUID,
               'stop_reason': 'NAV_INPUT_OR_TOPOLOGY_LOST',
               'blockers': [], 'rearm_blockers': ['OLD_GOAL_TERMINAL_PROOF_INVALID'],
               'first_stop': {'blockers': ['NAV_COMMAND_MISSING_OR_INVALID']}}
    details = SESSION.state_event_details(payload)
    assert details['goal_uuid'] == UUID
    assert details['blockers'] == []
    assert details['first_stop']['blockers'] == ['NAV_COMMAND_MISSING_OR_INVALID']
    assert details['rearm_blockers'] == payload['rearm_blockers']


def test_state_log_retains_mission_health_and_legacy_reports():
    assert SESSION.state_event_details({'state': 'IDLE'}) == {
        'state': 'IDLE', 'goal_uuid': None, 'stop_reason': None}
    info = SESSION.state_event_details({'state': 'FAULT', 'health': {
        'status': 'BLOCKED', 'blockers': ['DETECTION_STALE']}})
    assert info['health_status'] == 'BLOCKED'
    assert info['health_blockers'] == ['DETECTION_STALE']


def test_ir_failure_uuid_uses_gateway_before_cancel_guard_has_goal():
    gateway = {'cycle_goal_uuid': UUID, 'first_stop': {'blockers': ['ODOM_NOT_FRESH']}}
    guard = {'goal_uuid': None}
    info = SESSION.ir_demo_event_details(None, 'Gateway failure', gateway, guard, {})
    assert info['goal_uuid'] == UUID
    assert info['gateway']['first_stop']['blockers'] == ['ODOM_NOT_FRESH']
    assert guard['goal_uuid'] is None  # Logging must not alter cancel ownership.


@pytest.mark.parametrize('recovery_goal', [UUID, 'cd' * 16])
def test_ir_session_withdraws_arms_and_stays_latched_after_recovery(monkeypatch, recovery_goal):
    """Exercise actual demo loop with startup/I/O stubbed, never spawning ROS."""
    events, arms = [], []
    state = {'tick': 0}
    first_stop = {'cycle_goal_uuid': UUID, 'reason': 'NAV_INPUT_OR_TOPOLOGY_LOST',
                  'blockers': ['ODOM_NOT_FRESH']}
    def latest(topic):
        if topic == '/fire_mission/live_nav_stop_state':
            return {'state': 'RELEASED_STOPPED', 'cycle_goal_uuid': UUID,
                    'stop_reason': 'NAV_INPUT_OR_TOPOLOGY_LOST',
                    'blockers': ['ODOM_NOT_FRESH'] if state['tick'] == 1 else [],
                    'rearm_blockers': ['OLD_GOAL_TERMINAL_PROOF_INVALID'],
                    'first_stop': first_stop}
        if topic == '/fire_mission/state':
            return {'state': 'FAULT' if state['tick'] == 1 else 'NAV2_NAVIGATING',
                    'navigation': {'goal_uuid': recovery_goal}}
        return {'state': 'WAITING_FOR_ADMISSION', 'goal_uuid': None}
    class FakeSession:
        def __init__(self, **kwargs): pass
        def start_pi(self): pass
        def launch(self, *args): pass
        def event(self, name, **kw): events.append((name, kw))
        def assert_alive(self):
            state['tick'] += 1
            if state['tick'] == 4: raise KeyboardInterrupt()
        def disarm(self): arms.append('withdrawn')
        def close(self): events.append(('closed', {}))
    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.setattr(SESSION, 'Session', FakeSession)
    monkeypatch.setattr(SESSION, 'TelemetryMonitor', lambda **kw: SimpleNamespace(latest=latest))
    monkeypatch.setattr(SESSION, 'validate_map', lambda value: value)
    monkeypatch.setattr(SESSION, 'no_external_owners', lambda: None)
    monkeypatch.setattr(SESSION, 'pi_sensor_preflight', lambda **kw: None)
    monkeypatch.setattr(SESSION, 'wait_until', lambda *a, **kw: None)
    monkeypatch.setattr(SESSION, 'wait_for_startup_states', lambda *a, **kw: None)
    monkeypatch.setattr(SESSION, 'arm_true', lambda *a: arms.append('armed'))
    monkeypatch.setattr(SESSION, 'arm_ir_with_ack', lambda *a: arms.append('armed'))
    monkeypatch.setattr(SESSION.time, 'sleep', lambda *a: None)
    with pytest.raises(KeyboardInterrupt):
        SESSION.demo(SimpleNamespace(live=True, map='fixture.yaml', align_only=False, ir_only=True))
    assert arms == ['armed'] * 5 + ['withdrawn']
    failed = [data for name, data in events if name == 'IR_DEMO_FAILED']
    assert len(failed) == 1
    assert failed[0]['goal_uuid'] == UUID
    assert failed[0]['gateway']['first_stop'] == first_stop
    assert ('SESSION_INTERRUPTED', {'phase': 'live_demo'}) in events
    assert events[-1][0] == 'closed'


def test_actual_monitor_callback_keeps_evidence_when_initial_stop_message_was_missed():
    import threading
    import json
    events = []
    monitor = SESSION.TelemetryMonitor.__new__(SESSION.TelemetryMonitor)
    monitor.lock = threading.Lock()
    monitor.cache = {}
    monitor.on_change = lambda name, **kw: events.append((name, kw))
    payload = {'state': 'RELEASED_STOPPED', 'cycle_goal_uuid': UUID,
               'stop_reason': 'NAV_INPUT_OR_TOPOLOGY_LOST', 'blockers': [],
               'first_stop': {'blockers': ['NAV_SHADOW_TOPOLOGY_INVALID']}}
    monitor._on_message('/fire_mission/live_nav_stop_state',
                        SimpleNamespace(data=json.dumps(payload)))
    assert events[0][1]['goal_uuid'] == UUID
    assert events[0][1]['first_stop']['blockers'] == ['NAV_SHADOW_TOPOLOGY_INVALID']
