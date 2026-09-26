"""Unit tests for the observe-only mission state machine."""

from fire_robot_mission.mission_supervisor import MissionSupervisorCore


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
