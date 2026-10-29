"""Fail-closed tests for the read-only Nav2 cancel-intent preview."""

from fire_robot_mission.nav2_cancel_intent_shadow import (
    Nav2CancelIntentShadowCore,
)


GOAL_ID = 'ab' * 16


def mission(state='NAV2_NAVIGATING', intent='KEEP_NAV2', active=True):
    return {
        'observe_only': True,
        'state': state,
        'decision': {'intent': intent, 'command_published': False},
        'navigation': {'goal_active': active},
        'health': {'status': 'OK', 'blockers': []},
        'safety': {
            'motion_command_publisher': False,
            'actuator_command_publisher': False,
            'nav2_action_client': False,
        },
    }


def core_with_active_goal(state='NAV2_NAVIGATING', intent='KEEP_NAV2'):
    core = Nav2CancelIntentShadowCore()
    core.update_status_publisher_count(1)
    core.update_mission(mission(state, intent), 10.0)
    core.update_statuses([(GOAL_ID, 2)], 10.0)
    return core


def test_missing_inputs_block_cancellation():
    result = Nav2CancelIntentShadowCore().evaluate(10.0)
    assert result['state'] == 'BLOCKED'
    assert result['request_cancel'] is False
    assert 'MISSION_NOT_RECEIVED' in result['blockers']
    assert 'NAV_STATUS_NOT_RECEIVED' in result['blockers']


def test_navigation_and_candidate_do_not_request_cancel():
    core = core_with_active_goal()
    assert core.evaluate(10.1)['state'] == 'NO_CANCEL_NEEDED'
    core.update_mission(mission('FIRE_CANDIDATE', 'KEEP_NAV2'), 10.1)
    assert core.evaluate(10.2)['state'] == 'NO_CANCEL_NEEDED'


def test_acquired_fire_requests_only_shadow_cancel_for_exact_goal():
    core = core_with_active_goal('FIRE_TRACKING', 'LEFT_PREVIEW')
    result = core.evaluate(10.1)
    assert result['state'] == 'REQUEST_CANCEL'
    assert result['reason'] == 'FIRE_ACQUIRED'
    assert result['navigation']['active_goals'] == [
        {'uuid': GOAL_ID, 'status': 2}]
    assert result['shadow_only'] is True
    assert result['safety']['cancel_request_sent'] is False


def test_lost_target_still_requests_cancel_and_never_resumes_nav():
    core = core_with_active_goal('TARGET_LOST', 'REQUEST_SAFE_STOP')
    assert core.evaluate(10.1)['state'] == 'REQUEST_CANCEL'


def test_fault_requests_cancel_even_when_health_is_blocked():
    core = core_with_active_goal('FAULT', 'REQUEST_SAFE_STOP')
    payload = mission('FAULT', 'REQUEST_SAFE_STOP')
    payload['health'] = {'status': 'BLOCKED', 'blockers': ['DETECTION_STALE']}
    core.update_mission(payload, 10.0)
    assert core.evaluate(10.1)['state'] == 'REQUEST_CANCEL'


def test_canceling_goal_is_reported_but_not_requested_again():
    core = core_with_active_goal('FIRE_TRACKING', 'LEFT_PREVIEW')
    core.update_statuses([(GOAL_ID, 3)], 10.1)
    result = core.evaluate(10.2)
    assert result['state'] == 'CANCEL_ALREADY_IN_PROGRESS'
    assert result['request_cancel'] is False


def test_terminal_goal_needs_mission_agreement_before_no_cancel():
    core = core_with_active_goal('FIRE_TRACKING', 'LEFT_PREVIEW')
    core.update_statuses([(GOAL_ID, 5)], 10.1)
    assert core.evaluate(10.2)['state'] == 'BLOCKED'
    core.update_mission(mission('TARGET_LOST', 'REQUEST_SAFE_STOP', False),
                        10.2)
    assert core.evaluate(10.3)['state'] == 'NO_CANCEL_NEEDED'


def test_stale_inputs_and_goal_mismatch_block():
    core = core_with_active_goal('FIRE_TRACKING', 'LEFT_PREVIEW')
    assert 'MISSION_TIMEOUT' in core.evaluate(10.4)['blockers']
    core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 12.0)
    # Action status is event-driven; a long-lived EXECUTING state is valid.
    assert core.evaluate(12.1)['state'] == 'REQUEST_CANCEL'
    core.update_statuses([], 12.1)
    assert 'GOAL_STATE_MISMATCH' in core.evaluate(12.2)['blockers']


def test_missing_or_ambiguous_status_publisher_blocks_old_status():
    core = core_with_active_goal('FIRE_TRACKING', 'LEFT_PREVIEW')
    core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 20.0)
    assert core.evaluate(20.1)['state'] == 'REQUEST_CANCEL'
    core.update_status_publisher_count(0)
    result = core.evaluate(20.2)
    assert result['state'] == 'BLOCKED'
    assert 'NAV_STATUS_PUBLISHER_UNAVAILABLE_OR_AMBIGUOUS' in result[
        'blockers']
    core.update_status_publisher_count(2)
    assert core.evaluate(20.2)['state'] == 'BLOCKED'
    core.update_status_publisher_count(1)
    assert 'NAV_STATUS_NOT_RECEIVED' in core.evaluate(20.2)['blockers']


def test_multiple_goals_or_malformed_input_blocks():
    core = core_with_active_goal('FIRE_TRACKING', 'LEFT_PREVIEW')
    core.update_statuses([(GOAL_ID, 2), ('cd' * 16, 1)], 10.0)
    assert 'MULTIPLE_ACTIVE_GOALS' in core.evaluate(10.1)['blockers']
    core.update_statuses([(GOAL_ID, 2)], 10.0)
    core.mark_mission_error(10.0)
    assert 'MISSION_INVALID_JSON_OR_PAYLOAD' in core.evaluate(10.1)[
        'blockers']
    core.update_mission({}, 10.0)
    assert core.evaluate(10.1)['state'] == 'BLOCKED'


def test_contract_and_state_intent_mismatch_block_or_do_not_request():
    core = core_with_active_goal('FIRE_TRACKING', 'HOLD_CENTER_PREVIEW')
    assert core.evaluate(10.1)['state'] == 'NO_CANCEL_NEEDED'
    unsafe = mission('FIRE_TRACKING', 'LEFT_PREVIEW')
    unsafe['safety']['nav2_action_client'] = True
    core.update_mission(unsafe, 10.0)
    assert core.evaluate(10.1)['state'] == 'BLOCKED'
    malformed = mission('FIRE_TRACKING', 'LEFT_PREVIEW')
    malformed['decision'] = []
    core.update_mission(malformed, 10.0)
    assert core.evaluate(10.1)['state'] == 'BLOCKED'
