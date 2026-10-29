"""Unit tests for the fail-closed motion arbiter shadow."""

from fire_robot_mission.motion_arbiter_shadow import (
    MotionArbiterShadowCore,
)


def mission(
        state='IDLE', intent='HOLD_STOP', goal_active=False,
        owner='NONE', **overrides):
    """Build one valid supervisor payload."""
    payload = {
        'observe_only': True,
        'state': state,
        'decision': {
            'intent': intent,
            'command_published': False,
        },
        'health': {'status': 'OK', 'blockers': []},
        'robot': {'odom_fresh': True, 'stationary': True},
        'navigation': {
            'goal_active': goal_active,
            'observed_motion_owner': owner,
        },
        'safety': {
            'motion_command_publisher': False,
            'actuator_command_publisher': False,
            'nav2_action_client': False,
            'collision_check_bypassed': False,
        },
    }
    payload.update(overrides)
    return payload


def handoff(active=False, angular=0.0, state=None, **overrides):
    """Build one valid handoff shadow payload."""
    if state is None:
        state = 'ARMED_SHADOW_TRACKING' if active else 'DISARMED'
    payload = {
        'shadow_only': True,
        'state': state,
        'active': active,
        'admission': {
            'eligible': active,
            'blockers': [] if active else ['OPERATOR_DISARMED'],
        },
        'shadow_command': {
            'linear_x_m_s': 0.0,
            'angular_z_rad_s': angular,
        },
        'safety': {
            'real_motion_output': False,
            'actuator_command_publisher': False,
            'nav2_action_client': False,
            'requires_arm_heartbeat': True,
        },
    }
    payload.update(overrides)
    return payload


def test_idle_has_no_owner_and_zero_output():
    core = MotionArbiterShadowCore()
    core.update_mission(mission(), 1.0)
    core.update_handoff(handoff(), 1.0)
    linear, angular, status = core.evaluate(1.0)
    assert (linear, angular) == (0.0, 0.0)
    assert status['state'] == 'IDLE'
    assert status['owner'] == 'NONE'
    assert status['admission']['blockers'] == []


def test_navigation_owns_fresh_nav_command_and_clamps_limits():
    core = MotionArbiterShadowCore()
    core.update_mission(mission(
        state='NAV2_NAVIGATING', intent='KEEP_NAV2',
        goal_active=True, owner='NAV2'), 1.0)
    core.update_handoff(handoff(), 1.0)
    core.update_nav_command(0.2, -1.2, 1.0)
    linear, angular, status = core.evaluate(1.0)
    assert linear == 0.1
    assert angular == -0.8
    assert status['state'] == 'NAVIGATION'
    assert status['owner'] == 'NAVIGATION'
    assert status['shadow_command']['linear_clamped'] is True
    assert status['shadow_command']['angular_clamped'] is True


def test_navigation_accepts_eight_tenths_without_clamping():
    core = MotionArbiterShadowCore()
    core.update_mission(mission(
        state='NAV2_NAVIGATING', intent='KEEP_NAV2',
        goal_active=True, owner='NAV2'), 1.0)
    core.update_handoff(handoff(), 1.0)
    core.update_nav_command(0.0, 0.8, 1.0)
    linear, angular, status = core.evaluate(1.0)
    assert (linear, angular) == (0.0, 0.8)
    assert status['state'] == 'NAVIGATION'
    assert status['shadow_command']['angular_clamped'] is False


def test_alignment_owns_admitted_shadow_command():
    core = MotionArbiterShadowCore()
    core.update_mission(mission(
        state='FIRE_TRACKING', intent='LEFT_PREVIEW'), 1.0)
    core.update_handoff(handoff(active=True, angular=0.3), 1.0)
    core.update_alignment_command(0.0, 0.3, 1.0)
    linear, angular, status = core.evaluate(1.0)
    assert (linear, angular) == (0.0, 0.3)
    assert status['state'] == 'ALIGNMENT'
    assert status['owner'] == 'ALIGNMENT'
    assert status['active'] is True


def test_centered_alignment_can_own_a_zero_command():
    core = MotionArbiterShadowCore()
    core.update_mission(mission(
        state='ALIGNMENT_READY', intent='HOLD_CENTER_PREVIEW'), 1.0)
    core.update_handoff(handoff(
        active=True, angular=0.0,
        state='ARMED_SHADOW_CENTERED'), 1.0)
    core.update_alignment_command(0.0, 0.0, 1.0)
    linear, angular, status = core.evaluate(1.0)
    assert (linear, angular) == (0.0, 0.0)
    assert status['state'] == 'ALIGNMENT'
    assert status['owner'] == 'ALIGNMENT'


def test_simultaneous_owner_requests_fail_closed():
    core = MotionArbiterShadowCore()
    core.update_mission(mission(
        state='FIRE_CANDIDATE', intent='KEEP_NAV2',
        goal_active=True, owner='NAV2'), 1.0)
    core.update_handoff(handoff(active=True, angular=0.2), 1.0)
    core.update_nav_command(0.1, 0.0, 1.0)
    core.update_alignment_command(0.0, 0.2, 1.0)
    linear, angular, status = core.evaluate(1.0)
    assert (linear, angular) == (0.0, 0.0)
    assert status['state'] == 'SAFE_STOP'
    assert status['owner'] == 'NONE'
    assert 'OWNERSHIP_REQUEST_CONFLICT' in \
        status['admission']['blockers']


def test_target_loss_and_stale_mission_fail_closed():
    core = MotionArbiterShadowCore()
    core.update_mission(mission(
        state='TARGET_LOST', intent='REQUEST_SAFE_STOP'), 1.0)
    core.update_handoff(handoff(), 1.0)
    _, _, target_lost = core.evaluate(1.0)
    assert target_lost['state'] == 'SAFE_STOP'
    assert 'MISSION_REQUESTED_SAFE_STOP' in \
        target_lost['admission']['blockers']

    core = MotionArbiterShadowCore()
    core.update_mission(mission(), 1.0)
    core.update_handoff(handoff(), 1.4)
    _, _, stale = core.evaluate(1.4)
    assert stale['state'] == 'SAFE_STOP'
    assert 'MISSION_TIMEOUT' in stale['admission']['blockers']


def test_active_owner_requires_a_fresh_command():
    core = MotionArbiterShadowCore()
    core.update_mission(mission(
        state='NAV2_NAVIGATING', intent='KEEP_NAV2',
        goal_active=True, owner='NAV2'), 1.4)
    core.update_handoff(handoff(), 1.4)
    core.update_nav_command(0.1, 0.0, 1.0)
    _, _, status = core.evaluate(1.4)
    assert status['state'] == 'SAFE_STOP'
    assert 'NAV_COMMAND_TIMEOUT' in status['admission']['blockers']


def test_alignment_command_must_match_handoff_state():
    core = MotionArbiterShadowCore()
    core.update_mission(mission(
        state='FIRE_TRACKING', intent='LEFT_PREVIEW'), 1.0)
    core.update_handoff(handoff(active=True, angular=0.3), 1.0)
    core.update_alignment_command(0.0, 0.1, 1.0)
    _, _, status = core.evaluate(1.0)
    assert status['state'] == 'SAFE_STOP'
    assert 'ALIGNMENT_COMMAND_STATE_MISMATCH' in \
        status['admission']['blockers']


def test_nonplanar_or_nonfinite_commands_fail_closed():
    core = MotionArbiterShadowCore()
    core.update_mission(mission(
        state='NAV2_NAVIGATING', intent='KEEP_NAV2',
        goal_active=True, owner='NAV2'), 1.0)
    core.update_handoff(handoff(), 1.0)
    core.update_nav_command(0.1, 0.0, 1.0, (0.1, 0.0, 0.0, 0.0))
    _, _, nonplanar = core.evaluate(1.0)
    assert 'NAV_COMMAND_NONPLANAR' in nonplanar['admission']['blockers']

    core.update_nav_command(float('nan'), 0.0, 1.0)
    _, _, nonfinite = core.evaluate(1.0)
    assert 'NAV_COMMAND_NONFINITE' in nonfinite['admission']['blockers']


def test_invalid_contracts_and_parameters_are_rejected():
    core = MotionArbiterShadowCore()
    unsafe = mission()
    unsafe['safety']['collision_check_bypassed'] = True
    core.update_mission(unsafe, 1.0)
    core.update_handoff(handoff(), 1.0)
    _, _, status = core.evaluate(1.0)
    assert 'MISSION_SAFETY_CONTRACT' in status['admission']['blockers']

    try:
        MotionArbiterShadowCore(nav_command_timeout_s=0.0)
    except ValueError as exc:
        assert 'nav_command_timeout_s' in str(exc)
    else:
        raise AssertionError('zero timeout must be rejected')
