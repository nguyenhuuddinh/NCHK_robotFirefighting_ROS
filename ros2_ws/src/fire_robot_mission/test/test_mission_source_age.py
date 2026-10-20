"""Replay mission decisions as a cached YOLO observation ages locally."""

import copy

import pytest

from fire_robot_mission.mission_supervisor import MissionSupervisorCore


def update_target(core, now, age_ms=10.0, present=True):
    """Supply fresh telemetry without ROS nodes, clocks, publishers or hardware."""
    payload = {
        'observe_only': True, 'state': 'ok',
        'source': {'age_ms': age_ms},
        'fire': {'detected_now': present, 'confirmed': present},
    }
    core.update_detection(payload, now)
    core.update_alignment({
        'observe_only': True, 'state': 'ALIGNING_LEFT',
        'preview': {'direction': 'LEFT', 'target_angular_z_rad_s': 0.3},
        'safety': {'motion_command_publisher': False,
                   'actuator_command_publisher': False},
    }, now)
    core.update_odom(0.0, 0.0, now)
    return payload


def test_cached_target_expires_before_receive_timeout_and_revokes_readiness():
    """An acquired 290ms target must stop after another 20ms in the cache."""
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1)
    update_target(core, 0.0)
    assert core.evaluate(0.0)['state'] == 'FIRE_CANDIDATE'
    update_target(core, 0.11)
    assert core.evaluate(0.11)['state'] == 'FIRE_TRACKING'
    update_target(core, 0.12, age_ms=290.0)
    assert core.evaluate(0.12)['decision'][
        'eligible_for_future_alignment_handoff'] is True

    expired = core.evaluate(0.14)
    assert expired['state'] == 'FAULT'
    assert expired['decision']['intent'] == 'REQUEST_SAFE_STOP'
    assert expired['decision'][
        'eligible_for_future_alignment_handoff'] is False
    assert 'SOURCE_TIMESTAMP_STALE' in expired['health']['blockers']
    assert 'DETECTION_TIMEOUT' not in expired['health']['blockers']
    assert expired['perception']['source_age_ms'] == pytest.approx(310.0)
    assert expired['decision']['command_published'] is False


def test_fresh_observations_recover_but_must_reconfirm_after_expiry():
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1)
    update_target(core, 0.0, age_ms=290.0)
    core.evaluate(0.0)
    assert core.evaluate(0.02)['state'] == 'FAULT'
    # Timer repetitions or one replacement frame cannot inherit acquisition.
    update_target(core, 0.03)
    assert core.evaluate(0.03)['state'] == 'FIRE_CANDIDATE'
    assert core.evaluate(0.14)['state'] == 'FIRE_CANDIDATE'
    update_target(core, 0.15)
    recovered = core.evaluate(0.15)
    assert recovered['state'] == 'FIRE_TRACKING'
    assert recovered['health']['blockers'] == []


def test_timer_evaluation_does_not_compound_age_or_mutate_input():
    core = MissionSupervisorCore()
    payload = update_target(core, 0.0, age_ms=50.0)
    original = copy.deepcopy(payload)
    first = core.evaluate(0.1)
    again = core.evaluate(0.1)
    later = core.evaluate(0.2)
    assert first['perception']['source_age_ms'] == pytest.approx(150.0)
    assert again['perception']['source_age_ms'] == pytest.approx(150.0)
    assert later['perception']['source_age_ms'] == pytest.approx(250.0)
    assert payload == original


def test_future_timestamp_does_not_become_valid_by_waiting():
    core = MissionSupervisorCore()
    update_target(core, 0.0, age_ms=-100.0)
    for now in (0.0, 0.1, 0.2):
        result = core.evaluate(now)
        assert result['state'] == 'FAULT'
        assert 'SOURCE_TIMESTAMP_IN_FUTURE' in result['health']['blockers']
        assert result['decision'][
            'eligible_for_future_alignment_handoff'] is False


def test_tolerated_clock_skew_does_not_extend_image_budget():
    core = MissionSupervisorCore()
    update_target(core, 0.0, age_ms=-8.0)
    initial = core.evaluate(0.0)
    assert initial['health']['warnings'] == ['SMALL_NEGATIVE_SOURCE_AGE']
    assert initial['perception']['source_age_ms'] == 0.0
    expired = core.evaluate(0.301)
    assert 'SOURCE_TIMESTAMP_STALE' in expired['health']['blockers']


def test_cached_no_fire_cannot_remain_keep_nav2_after_image_expiry():
    core = MissionSupervisorCore()
    core.update_goal_statuses([('ab' * 16, 2)], 0.0)
    update_target(core, 0.0, age_ms=295.0, present=False)
    assert core.evaluate(0.0)['decision']['intent'] == 'KEEP_NAV2'
    expired = core.evaluate(0.01)
    assert expired['state'] == 'FAULT'
    assert expired['decision']['intent'] == 'REQUEST_SAFE_STOP'
    assert 'SOURCE_TIMESTAMP_STALE' in expired['health']['blockers']


def test_superseding_message_uses_its_own_age_and_receipt_time():
    core = MissionSupervisorCore()
    update_target(core, 0.0, age_ms=290.0)
    assert core.evaluate(0.02)['state'] == 'FAULT'
    update_target(core, 0.03, age_ms=20.0)
    result = core.evaluate(0.04)
    assert result['state'] == 'FIRE_CANDIDATE'
    assert result['perception']['source_age_ms'] == pytest.approx(30.0)
    assert result['health']['blockers'] == []


@pytest.mark.parametrize('reported_age', [301.0, float('nan'), float('inf'), None])
def test_invalid_or_already_old_age_still_blocks(reported_age):
    core = MissionSupervisorCore()
    update_target(core, 0.0, age_ms=reported_age)
    result = core.evaluate(0.0)
    assert result['state'] == 'FAULT'
    expected = ('SOURCE_TIMESTAMP_STALE' if reported_age == 301.0
                else 'SOURCE_AGE_MISSING')
    assert expected in result['health']['blockers']
