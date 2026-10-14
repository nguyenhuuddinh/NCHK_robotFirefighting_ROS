"""Unit tests for the observe-only mission state machine."""

import pytest

from fire_robot_mission.mission_supervisor import MissionSupervisorCore


GOAL_UUID = 'ab' * 16


def detection(
        detected=True, confirmed=True, age_ms=10.0, state='ok',
        observe_only=True):
    """Build one minimal YOLO telemetry payload."""
    return {
        'observe_only': observe_only,
        'state': state,
        'source': {'age_ms': age_ms},
        'fire': {
            'detected_now': detected,
            'confirmed': confirmed,
            'candidate': {'confidence': 0.75} if detected else None,
        },
    }


def alignment(
        state='ALIGNING_LEFT', direction='LEFT', angular=0.3,
        observe_only=True, safe=True):
    """Build one minimal alignment preview payload."""
    return {
        'observe_only': observe_only,
        'state': state,
        'preview': {
            'direction': direction,
            'target_angular_z_rad_s': angular,
        },
        'safety': {
            'motion_command_publisher': False if safe else True,
            'actuator_command_publisher': False,
        },
    }


def refresh_target(core, now, align=None):
    """Refresh both target inputs at one monotonic timestamp."""
    core.update_detection(detection(), now)
    core.update_alignment(align or alignment(), now)


def test_waits_for_first_detection_message():
    core = MissionSupervisorCore()
    result = core.evaluate(0.0)
    assert result['state'] == 'WAITING_FOR_INPUT'
    assert result['decision']['command_published'] is False
    assert result['safety']['motion_command_publisher'] is False
    assert result['safety']['nav2_action_client'] is False


def test_nav2_remains_owner_without_a_fire_target():
    core = MissionSupervisorCore()
    core.update_detection(
        detection(detected=False, confirmed=False), 0.0)
    core.update_goal_statuses([2], 0.0)
    result = core.evaluate(0.0)
    assert result['state'] == 'NAV2_NAVIGATING'
    assert result['decision']['intent'] == 'KEEP_NAV2'
    assert result['navigation']['observed_motion_owner'] == 'NAV2'


def test_single_detection_stays_candidate_then_tracks_left():
    core = MissionSupervisorCore(fire_acquire_hold_s=0.5)
    core.update_goal_statuses([2], 0.0)
    core.update_odom(0.0, 0.0, 0.0)
    refresh_target(core, 0.0)
    first = core.evaluate(0.0)
    assert first['state'] == 'FIRE_CANDIDATE'
    assert first['decision']['intent'] == 'KEEP_NAV2'

    refresh_target(core, 0.6)
    core.update_odom(0.0, 0.0, 0.6)
    tracked = core.evaluate(0.6)
    assert tracked['state'] == 'FIRE_TRACKING'
    assert tracked['decision']['intent'] == 'LEFT_PREVIEW'
    assert tracked['decision'][
        'eligible_for_future_alignment_handoff'] is False
    assert 'NAV2_GOAL_ACTIVE' in tracked['decision'][
        'readiness_blockers']


def test_first_acquisition_needs_two_fresh_confirmed_observations():
    core = MissionSupervisorCore()
    refresh_target(core, 0.0)
    assert core.evaluate(0.0)['state'] == 'FIRE_CANDIDATE'
    assert core.evaluate(0.5)['state'] == 'FIRE_CANDIDATE'
    refresh_target(core, 0.51)
    assert core.evaluate(0.51)['state'] == 'FIRE_TRACKING'


def test_center_requires_a_second_hold_before_alignment_ready():
    core = MissionSupervisorCore(
        fire_acquire_hold_s=0.5, centered_hold_s=0.5)
    centered = alignment('CENTERED', 'STOP', 0.0)
    core.update_odom(0.0, 0.0, 0.0)
    refresh_target(core, 0.0, centered)
    assert core.evaluate(0.0)['state'] == 'FIRE_CANDIDATE'

    core.update_odom(0.0, 0.0, 0.6)
    refresh_target(core, 0.6, centered)
    assert core.evaluate(0.6)['state'] == 'CENTERED_HOLD'

    core.update_odom(0.0, 0.0, 1.2)
    refresh_target(core, 1.2, centered)
    result = core.evaluate(1.2)
    assert result['state'] == 'ALIGNMENT_READY'
    assert result['decision']['intent'] == 'HOLD_CENTER_PREVIEW'
    assert result['decision'][
        'eligible_for_future_alignment_handoff'] is True


def test_target_loss_requests_safe_stop_immediately_after_acquire():
    core = MissionSupervisorCore(fire_acquire_hold_s=0.5)
    core.update_goal_statuses([(GOAL_UUID, 2)], 0.0)
    refresh_target(core, 0.0)
    core.evaluate(0.0)
    refresh_target(core, 0.6)
    assert core.evaluate(0.6)['state'] == 'FIRE_TRACKING'

    core.update_detection(
        detection(detected=False, confirmed=False), 0.61)
    result = core.evaluate(0.61)
    assert result['state'] == 'TARGET_LOST'
    assert result['decision']['intent'] == 'REQUEST_SAFE_STOP'
    assert result['decision']['command_published'] is False


def test_confirmed_detection_with_no_alignment_target_can_recover():
    core = MissionSupervisorCore()
    core.update_goal_statuses([(GOAL_UUID, 2)], 0.0)
    refresh_target(core, 0.0)
    core.evaluate(0.0)
    refresh_target(core, 0.6)
    assert core.evaluate(0.6)['state'] == 'FIRE_TRACKING'
    core.update_goal_statuses([(GOAL_UUID, 5)], 0.7)
    core.update_odom(0.0, 0.0, 0.7)
    refresh_target(core, 0.7, alignment('NO_TARGET', 'STOP', 0.0))
    waiting = core.evaluate(0.7)
    assert waiting['state'] == 'WAITING_FOR_ALIGNMENT'
    assert waiting['health']['status'] == 'OK'
    assert waiting['navigation']['goal_state'] == 'CANCELED'
    core.update_odom(0.0, 0.0, 0.8)
    refresh_target(core, 0.8)
    assert core.evaluate(0.8)['state'] == 'FIRE_TRACKING'


def test_same_canceled_goal_reacquires_then_returns_to_tracking():
    core = MissionSupervisorCore()
    core.update_goal_statuses([(GOAL_UUID, 2)], 0.0)
    refresh_target(core, 0.0)
    assert core.evaluate(0.0)['state'] == 'FIRE_CANDIDATE'
    refresh_target(core, 0.6)
    assert core.evaluate(0.6)['state'] == 'FIRE_TRACKING'
    # A confirmed target must remain tracked; reaching a new timer mode
    # must not restart acquisition without an intervening miss.
    refresh_target(core, 0.61)
    assert core.evaluate(0.61)['state'] == 'FIRE_TRACKING'

    core.update_goal_statuses([(GOAL_UUID, 5)], 0.62)
    core.update_detection(detection(False, False), 0.62)
    assert core.evaluate(0.62)['state'] == 'TARGET_LOST'
    refresh_target(core, 0.72)
    first = core.evaluate(0.72)
    assert first['state'] == 'FIRE_CANDIDATE'
    assert 'FIRE_NOT_ACQUIRED' in first['decision']['readiness_blockers']
    assert first['decision']['command_published'] is False

    # Timer evaluations alone cannot turn one cached frame into two samples.
    assert core.evaluate(0.94)['state'] == 'FIRE_CANDIDATE'
    refresh_target(core, 0.95)
    again = core.evaluate(0.95)
    assert again['state'] == 'FIRE_TRACKING'
    assert again['decision']['intent'] == 'LEFT_PREVIEW'
    assert core.evaluate(0.96)['state'] == 'FIRE_TRACKING'


def test_canceled_goal_proof_survives_status_pruning_for_reacquisition():
    core = MissionSupervisorCore()
    core.update_goal_statuses([(GOAL_UUID, 2)], 0.0)
    refresh_target(core, 0.0)
    core.evaluate(0.0)
    refresh_target(core, 0.6)
    assert core.evaluate(0.6)['state'] == 'FIRE_TRACKING'
    core.update_goal_statuses([(GOAL_UUID, 5)], 0.7)
    core.update_goal_statuses([], 50.0)
    core.update_detection(detection(False, False), 50.0)
    result = core.evaluate(50.0)
    assert result['state'] == 'TARGET_LOST'
    assert result['navigation']['goal_state'] == 'CANCELED'
    refresh_target(core, 50.1)
    assert core.evaluate(50.1)['state'] == 'FIRE_CANDIDATE'
    refresh_target(core, 50.31)
    assert core.evaluate(50.31)['state'] == 'FIRE_TRACKING'
    core.update_goal_statuses([(GOAL_UUID, 6)], 50.4)
    assert core._acquired_goal_canceled is False


def test_reacquisition_requires_consecutive_confirmed_updates():
    core = MissionSupervisorCore()
    core.update_goal_statuses([(GOAL_UUID, 2)], 0.0)
    refresh_target(core, 0.0)
    core.evaluate(0.0)
    refresh_target(core, 0.6)
    assert core.evaluate(0.6)['state'] == 'FIRE_TRACKING'
    core.update_goal_statuses([(GOAL_UUID, 5)], 0.7)
    core.update_detection(detection(False, False), 0.7)
    assert core.evaluate(0.7)['state'] == 'TARGET_LOST'
    refresh_target(core, 0.8)
    assert core.evaluate(0.8)['state'] == 'FIRE_CANDIDATE'
    core.update_detection(detection(True, False), 0.9)
    assert core.evaluate(0.9)['state'] == 'TARGET_LOST'
    refresh_target(core, 1.0)
    assert core.evaluate(1.0)['state'] == 'FIRE_CANDIDATE'
    refresh_target(core, 1.21)
    assert core.evaluate(1.21)['state'] == 'FIRE_TRACKING'


def test_new_goal_uses_full_acquire_hold_and_rebinds_review_goal():
    core = MissionSupervisorCore()
    core.update_goal_statuses([(GOAL_UUID, 2)], 0.0)
    refresh_target(core, 0.0)
    core.evaluate(0.0)
    refresh_target(core, 0.6)
    assert core.evaluate(0.6)['state'] == 'FIRE_TRACKING'
    core.update_detection(detection(False, False), 0.7)
    assert core.evaluate(0.7)['state'] == 'TARGET_LOST'

    next_uuid = 'cd' * 16
    core.update_goal_statuses([(GOAL_UUID, 5), (next_uuid, 2)], 0.8)
    refresh_target(core, 0.8)
    assert core.evaluate(0.8)['state'] == 'FIRE_CANDIDATE'
    refresh_target(core, 1.1)
    assert core.evaluate(1.1)['state'] == 'FIRE_CANDIDATE'
    refresh_target(core, 1.31)
    assert core.evaluate(1.31)['state'] == 'FIRE_TRACKING'
    assert core._acquired_goal_uuid == next_uuid


def test_missing_cancel_proof_cannot_use_fast_reacquire():
    core = MissionSupervisorCore()
    core.update_goal_statuses([(GOAL_UUID, 2)], 0.0)
    refresh_target(core, 0.0)
    core.evaluate(0.0)
    refresh_target(core, 0.6)
    assert core.evaluate(0.6)['state'] == 'FIRE_TRACKING'
    core.update_goal_statuses([], 0.7)
    core.update_detection(detection(False, False), 0.7)
    assert core.evaluate(0.7)['state'] == 'TARGET_LOST'
    refresh_target(core, 0.8)
    assert core.evaluate(0.8)['state'] == 'FIRE_CANDIDATE'
    refresh_target(core, 1.1)
    assert core.evaluate(1.1)['state'] == 'FIRE_CANDIDATE'
    refresh_target(core, 1.31)
    assert core.evaluate(1.31)['state'] == 'FIRE_TRACKING'


def test_stale_reacquisition_fails_closed_and_restarts_confirmation():
    core = MissionSupervisorCore()
    core.update_goal_statuses([(GOAL_UUID, 2)], 0.0)
    refresh_target(core, 0.0)
    core.evaluate(0.0)
    refresh_target(core, 0.6)
    assert core.evaluate(0.6)['state'] == 'FIRE_TRACKING'
    core.update_goal_statuses([(GOAL_UUID, 5)], 0.7)
    core.update_detection(detection(False, False), 0.7)
    assert core.evaluate(0.7)['state'] == 'TARGET_LOST'
    refresh_target(core, 0.8)
    assert core.evaluate(0.8)['state'] == 'FIRE_CANDIDATE'
    stale = core.evaluate(1.31)
    assert stale['state'] == 'FAULT'
    assert 'DETECTION_TIMEOUT' in stale['health']['blockers']
    refresh_target(core, 1.32)
    assert core.evaluate(1.32)['state'] == 'FIRE_CANDIDATE'
    refresh_target(core, 1.53)
    assert core.evaluate(1.53)['state'] == 'FIRE_TRACKING'


def test_reacquire_parameter_cannot_exceed_initial_hold():
    with pytest.raises(ValueError, match='fire_reacquire_hold_s'):
        MissionSupervisorCore(
            fire_acquire_hold_s=0.1, fire_reacquire_hold_s=0.2)


def test_fire_before_a_nav_goal_does_not_poison_the_next_cycle():
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1)
    refresh_target(core, 0.0)
    core.evaluate(0.0)
    refresh_target(core, 0.2)
    assert core.evaluate(0.2)['state'] == 'FIRE_TRACKING'
    core.update_detection(detection(False, False), 0.3)
    assert core.evaluate(0.3)['state'] == 'IDLE'


def test_fire_waiting_before_goal_still_latches_when_goal_starts():
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1)
    refresh_target(core, 0.0)
    core.evaluate(0.0)
    refresh_target(core, 0.2)
    assert core.evaluate(0.2)['state'] == 'FIRE_TRACKING'
    core.update_goal_statuses([(GOAL_UUID, 2)], 0.21)
    core.update_detection(detection(), 0.21)
    assert core.evaluate(0.21)['state'] == 'FIRE_TRACKING'
    core.update_goal_statuses([(GOAL_UUID, 5)], 0.22)
    core.update_detection(detection(False, False), 0.22)
    assert core.evaluate(0.22)['state'] == 'TARGET_LOST'


def acquired_then_lost():
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1)
    core.update_goal_statuses([(GOAL_UUID, 2)], 0.0)
    core.update_odom(0.0, 0.0, 0.0)
    refresh_target(core, 0.0)
    core.evaluate(0.0)
    core.update_odom(0.0, 0.0, 0.2)
    refresh_target(core, 0.2)
    assert core.evaluate(0.2)['state'] == 'FIRE_TRACKING'
    core.update_goal_statuses([(GOAL_UUID, 5)], 0.3)
    core.update_odom(0.0, 0.0, 0.3)
    core.update_detection(detection(False, False), 0.3)
    assert core.evaluate(0.3)['state'] == 'TARGET_LOST'
    return core


def clear_review(uuid=GOAL_UUID):
    return {
        'schema_version': 1, 'mode': 'SHADOW_ONLY',
        'state': 'NO_FIRE_EVIDENCE', 'goal_uuid': uuid,
        'pulse_index': 1, 'blockers': [],
        'safety': {'motion_command_publisher': False,
                   'pump_command_publisher': False,
                   'nav2_action_client': False,
                   'actuation_authorized': False},
    }


def actuator_off():
    return {'schema_version': 1, 'mode': 'COMMAND_TELEMETRY_ONLY',
            'pump_pwm_commanded_on': False, 'servo_phase': 'IDLE',
            'servo_pwm_commanded_on': False}


def test_exact_post_spray_review_and_actuator_off_clear_target_lost():
    core = acquired_then_lost()
    core.update_review(clear_review(), 0.31)
    core.update_actuator(actuator_off(), 0.31)
    core.update_odom(0.0, 0.0, 0.31)
    core.update_detection(detection(False, False), 0.31)
    assert core.evaluate(0.31)['state'] == 'IDLE'
    assert core.evaluate(0.32)['decision']['intent'] == 'HOLD_STOP'


@pytest.mark.parametrize('review', [
    clear_review('cd' * 16),
    {**clear_review(), 'pulse_index': 0},
    {**clear_review(), 'state': 'RESPRAY_READY_SHADOW'},
    {**clear_review(), 'blockers': ['VISION_IR_DISAGREE']},
    {**clear_review(), 'safety': {}},
])
def test_incomplete_or_wrong_goal_review_cannot_clear(review):
    core = acquired_then_lost()
    core.update_review(review, 0.31)
    core.update_actuator(actuator_off(), 0.31)
    core.update_odom(0.0, 0.0, 0.31)
    core.update_detection(detection(False, False), 0.31)
    assert core.evaluate(0.31)['state'] == 'TARGET_LOST'


def test_stale_review_or_active_pump_cannot_clear():
    core = acquired_then_lost()
    core.update_review(clear_review(), 0.31)
    core.update_actuator(actuator_off(), 0.31)
    core.update_odom(0.0, 0.0, 1.0)
    core.update_actuator(actuator_off(), 1.0)
    core.update_detection(detection(False, False), 1.0)
    assert core.evaluate(1.0)['state'] == 'TARGET_LOST'
    core.update_review(clear_review(), 1.01)
    core.update_actuator({**actuator_off(),
                          'pump_pwm_commanded_on': True}, 1.01)
    core.update_odom(0.0, 0.0, 1.01)
    core.update_detection(detection(False, False), 1.01)
    assert core.evaluate(1.01)['state'] == 'TARGET_LOST'


def test_stale_actuator_blocks_but_pruned_cancel_status_can_clear():
    core = acquired_then_lost()
    core.update_review(clear_review(), 0.7)
    core.update_odom(0.0, 0.0, 0.7)
    core.update_detection(detection(False, False), 0.7)
    core.update_actuator(actuator_off(), 0.31)
    assert core.evaluate(0.7)['state'] == 'TARGET_LOST'
    core.update_actuator(actuator_off(), 0.71)
    core.update_goal_statuses([], 0.71)
    core.update_review(clear_review(), 0.71)
    core.update_odom(0.0, 0.0, 0.71)
    core.update_detection(detection(False, False), 0.71)
    assert core.evaluate(0.71)['state'] == 'IDLE'


def test_pruned_first_goal_clears_then_second_goal_uses_full_acquisition():
    core = acquired_then_lost()
    core.update_goal_statuses([], 0.31)
    core.update_review(clear_review(), 0.31)
    core.update_actuator(actuator_off(), 0.31)
    core.update_odom(0.0, 0.0, 0.31)
    core.update_detection(detection(False, False), 0.31)
    assert core.evaluate(0.31)['state'] == 'IDLE'
    next_uuid = 'cd' * 16
    core.update_goal_statuses([(next_uuid, 2)], 0.4)
    refresh_target(core, 0.4)
    assert core.evaluate(0.4)['state'] == 'FIRE_CANDIDATE'
    refresh_target(core, 0.49)
    assert core.evaluate(0.49)['state'] == 'FIRE_CANDIDATE'
    refresh_target(core, 0.51)
    assert core.evaluate(0.51)['state'] == 'FIRE_TRACKING'
    assert core._acquired_goal_uuid == next_uuid


def test_pruned_status_without_latched_cancel_proof_cannot_clear():
    core = acquired_then_lost()
    core._acquired_goal_canceled = False
    core.update_goal_statuses([], 0.31)
    core.update_review(clear_review(), 0.31)
    core.update_actuator(actuator_off(), 0.31)
    core.update_odom(0.0, 0.0, 0.31)
    core.update_detection(detection(False, False), 0.31)
    assert core.evaluate(0.31)['state'] == 'TARGET_LOST'


def test_new_active_goal_or_robot_motion_cannot_clear():
    core = acquired_then_lost()
    core.update_review(clear_review(), 0.31)
    core.update_actuator(actuator_off(), 0.31)
    core.update_goal_statuses([(GOAL_UUID, 5), ('cd' * 16, 2)], 0.31)
    core.update_odom(0.0, 0.0, 0.31)
    core.update_detection(detection(False, False), 0.31)
    assert core.evaluate(0.31)['state'] == 'TARGET_LOST'
    core.update_goal_statuses([(GOAL_UUID, 5)], 0.32)
    core.update_odom(0.05, 0.0, 0.32)
    core.update_detection(detection(False, False), 0.32)
    assert core.evaluate(0.32)['state'] == 'TARGET_LOST'


def test_future_source_timestamp_blocks_tracking():
    core = MissionSupervisorCore(max_future_skew_ms=20.0)
    core.update_detection(detection(age_ms=-20.1), 0.0)
    core.update_alignment(alignment(), 0.0)
    result = core.evaluate(0.0)
    assert result['state'] == 'FAULT'
    assert 'SOURCE_TIMESTAMP_IN_FUTURE' in result['health']['blockers']


def test_small_negative_source_age_is_only_a_warning():
    core = MissionSupervisorCore(max_future_skew_ms=20.0)
    core.update_detection(detection(age_ms=-8.0), 0.0)
    core.update_alignment(alignment(), 0.0)
    result = core.evaluate(0.0)
    assert result['state'] == 'FIRE_CANDIDATE'
    assert result['health']['blockers'] == []
    assert result['health']['warnings'] == [
        'SMALL_NEGATIVE_SOURCE_AGE']


def test_stale_detection_and_unsafe_alignment_fail_closed():
    core = MissionSupervisorCore(detection_timeout_s=0.5)
    refresh_target(core, 0.0)
    stale = core.evaluate(0.51)
    assert stale['state'] == 'FAULT'
    assert 'DETECTION_TIMEOUT' in stale['health']['blockers']

    core = MissionSupervisorCore()
    core.update_detection(detection(), 0.0)
    core.update_alignment(alignment(safe=False), 0.0)
    unsafe = core.evaluate(0.0)
    assert unsafe['state'] == 'FAULT'
    assert 'ALIGNMENT_SAFETY_CONTRACT' in unsafe['health']['blockers']


def test_turn_comparison_reports_agreement_and_conflict():
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1)
    refresh_target(core, 0.0)
    core.update_nav_command(0.1, 0.2, 0.0)
    core.evaluate(0.0)
    refresh_target(core, 0.2)
    core.update_nav_command(0.1, 0.2, 0.2)
    agree = core.evaluate(0.2)
    assert agree['navigation']['fire_nav_turn_comparison'] == 'AGREE'

    refresh_target(core, 0.3)
    core.update_nav_command(0.1, -0.2, 0.3)
    conflict = core.evaluate(0.3)
    assert conflict['navigation']['fire_nav_turn_comparison'] == 'CONFLICT'


def test_non_nav_raw_command_blocks_future_handoff():
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1)
    core.update_odom(0.0, 0.0, 0.0)
    refresh_target(core, 0.0)
    core.evaluate(0.0)
    core.update_odom(0.0, 0.0, 0.2)
    core.update_raw_command(0.1, 0.0, 0.2)
    refresh_target(core, 0.2)
    result = core.evaluate(0.2)
    assert result['navigation']['observed_motion_owner'] == (
        'UNKNOWN_RAW_COMMAND_SOURCE')
    assert 'RAW_COMMAND_SOURCE_ACTIVE' in result['decision'][
        'readiness_blockers']


def test_malformed_json_marker_fails_closed():
    core = MissionSupervisorCore()
    core.mark_detection_error('bad json', 0.0)
    result = core.evaluate(0.0)
    assert result['state'] == 'FAULT'
    assert result['decision']['intent'] == 'REQUEST_SAFE_STOP'
    assert result['health']['blockers'] == ['DETECTION_INVALID_JSON']
