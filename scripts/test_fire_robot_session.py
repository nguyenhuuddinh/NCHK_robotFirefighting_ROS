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
