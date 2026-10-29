"""Unit tests for the fail-closed alignment handoff shadow gate."""

from copy import deepcopy

import pytest

from fire_robot_mission.alignment_handoff_shadow import (
    AlignmentHandoffShadowCore,
)


def mission(**overrides):
    """Build one minimal mission state that passes admission."""
    payload = {
        'observe_only': True,
        'state': 'FIRE_TRACKING',
        'decision': {
            'eligible_for_future_alignment_handoff': True,
            'readiness_blockers': [],
            'command_published': False,
        },
        'health': {'status': 'OK', 'blockers': []},
        'robot': {'odom_fresh': True, 'stationary': True},
        'navigation': {
            'goal_active': False,
            'observed_motion_owner': 'NONE',
        },
        'safety': {
            'motion_command_publisher': False,
            'actuator_command_publisher': False,
            'nav2_action_client': False,
        },
    }
    payload.update(overrides)
    return payload


def alignment(state='ALIGNING_LEFT', angular=0.25, **overrides):
    """Build one minimal safe alignment preview."""
    payload = {
        'observe_only': True,
        'state': state,
        'preview': {'target_angular_z_rad_s': angular},
        'safety': {
            'motion_command_publisher': False,
            'actuator_command_publisher': False,
        },
    }
    payload.update(overrides)
    return payload


def ready_core(now=1.0, angular=0.25):
    """Create a core with fresh mission, alignment and arm inputs."""
    core = AlignmentHandoffShadowCore()
    core.update_mission(mission(), now)
    core.update_alignment(alignment(angular=angular), now)
    core.update_arm(True, now)
    return core


def gateway_proof():
    return {
        'schema_version': 1,
        'state': 'HANDOFF_READY',
        'handoff_ready': True,
        'enabled': True,
        'one_shot': True,
        'fire_latched': True,
        'ever_forwarded': True,
        'released': True,
        'stop_reason': 'FIRE_ACQUIRED',
        'raw_publishers': 0,
        'goal_active': False,
        'nav_shadow_fingerprint_ok': True,
        'safety_gate_fingerprint_ok': True,
        'cancel_guard_publishers': 1,
        'safety': {
            'raw_command_publisher': False,
            'actuator_command_publisher': False,
            'alignment_command_publisher': False,
            'fail_closed_zero': True,
        },
    }


def cancel_guard_proof():
    return {
        'schema_version': 1,
        'state': 'CANCEL_CONFIRMED_STOPPED',
        'enabled': True,
        'goal_uuid': '1' * 32,
        'send_count': 1,
        'cancel_service_accepted': True,
        'cancel_status_seen': True,
        'goal_status': 5,
        'arm_fresh': True,
        'cmd_vel_fresh_zero': True,
        'odom_fresh_stationary': True,
        'nav_shadow_fingerprint_ok': True,
        'raw_cmd_publishers': 0,
        'blockers': [],
        'safety': {
            'motion_command_publisher': False,
            'actuator_command_publisher': False,
        },
    }


def proof_core(now=1.0):
    core = AlignmentHandoffShadowCore(require_live_cancel_proof=True)
    core.update_mission(mission(), now)
    core.update_alignment(alignment(), now)
    core.update_arm(True, now)
    return core


def evaluate_proof(core, now=1.0, gateway_publishers=1,
                   guard_publishers=1):
    return core.evaluate(now, gateway_publishers, guard_publishers)


def test_disarmed_gate_is_zero_even_when_inputs_are_ready():
    core = AlignmentHandoffShadowCore()
    core.update_mission(mission(), 1.0)
    core.update_alignment(alignment(), 1.0)
    core.update_arm(False, 1.0)
    angular, status = core.evaluate(1.0)
    assert angular == 0.0
    assert status['state'] == 'DISARMED'
    assert status['active'] is False
    assert status['operator_arm']['fresh'] is True
    assert 'OPERATOR_DISARMED' in status['admission']['blockers']
    assert status['safety']['real_motion_output'] is False


def test_fresh_arm_and_ready_inputs_admit_left_shadow_command():
    core = ready_core()
    angular, status = core.evaluate(1.1)
    assert angular == 0.25
    assert status['state'] == 'ARMED_SHADOW_TRACKING'
    assert status['active'] is True
    assert status['admission']['blockers'] == []
    assert status['shadow_command']['linear_x_m_s'] == 0.0


def test_shadow_speed_is_clamped_below_alignment_preview_limit():
    core = ready_core(angular=0.50)
    angular, status = core.evaluate(1.0)
    assert angular == 0.35
    assert status['alignment']['clamped'] is True


def test_right_state_requires_negative_angular_velocity():
    core = AlignmentHandoffShadowCore()
    core.update_mission(mission(), 1.0)
    core.update_alignment(
        alignment(state='ALIGNING_RIGHT', angular=0.2), 1.0)
    core.update_arm(True, 1.0)
    angular, status = core.evaluate(1.0)
    assert angular == 0.0
    assert status['state'] == 'BLOCKED'
    assert 'ALIGNMENT_DIRECTION_SIGN_MISMATCH' in \
        status['admission']['blockers']


def test_centered_state_always_outputs_zero():
    core = AlignmentHandoffShadowCore()
    centered_mission = mission(state='ALIGNMENT_READY')
    core.update_mission(centered_mission, 1.0)
    core.update_alignment(alignment('CENTERED', 0.0), 1.0)
    core.update_arm(True, 1.0)
    angular, status = core.evaluate(1.0)
    assert angular == 0.0
    assert status['state'] == 'ARMED_SHADOW_CENTERED'
    assert status['active'] is True


def test_arm_heartbeat_timeout_fails_closed():
    core = ready_core()
    angular, status = core.evaluate(1.41)
    assert angular == 0.0
    assert status['state'] == 'DISARMED'
    assert status['operator_arm']['fresh'] is False
    assert 'ARM_HEARTBEAT_TIMEOUT' in status['admission']['blockers']
    assert status['safety']['fail_closed_zero'] is True


def test_stale_mission_fails_closed_while_arm_is_refreshed():
    core = ready_core()
    core.update_arm(True, 1.40)
    core.update_alignment(alignment(), 1.40)
    angular, status = core.evaluate(1.40)
    assert angular == 0.0
    assert 'MISSION_TIMEOUT' in status['admission']['blockers']


def test_nav2_or_other_motion_owner_blocks_admission():
    core = AlignmentHandoffShadowCore()
    unsafe_mission = mission(
        navigation={
            'goal_active': True,
            'observed_motion_owner': 'NAV2',
        })
    core.update_mission(unsafe_mission, 1.0)
    core.update_alignment(alignment(), 1.0)
    core.update_arm(True, 1.0)
    angular, status = core.evaluate(1.0)
    assert angular == 0.0
    assert 'NAV2_GOAL_ACTIVE_OR_UNKNOWN' in \
        status['admission']['blockers']
    assert 'OTHER_MOTION_OWNER_ACTIVE' in status['admission']['blockers']


def test_nonstationary_or_stale_odom_blocks_admission():
    core = AlignmentHandoffShadowCore()
    unsafe_mission = mission(
        robot={'odom_fresh': False, 'stationary': False})
    core.update_mission(unsafe_mission, 1.0)
    core.update_alignment(alignment(), 1.0)
    core.update_arm(True, 1.0)
    angular, status = core.evaluate(1.0)
    assert angular == 0.0
    assert 'ODOM_NOT_FRESH' in status['admission']['blockers']
    assert 'ROBOT_NOT_STATIONARY' in status['admission']['blockers']


def test_malformed_or_non_observe_only_inputs_fail_closed():
    core = AlignmentHandoffShadowCore()
    core.mark_mission_error('bad json', 1.0)
    core.update_alignment(
        alignment(observe_only=False), 1.0)
    core.update_arm(True, 1.0)
    angular, status = core.evaluate(1.0)
    assert angular == 0.0
    assert 'MISSION_INVALID_JSON' in status['admission']['blockers']
    assert 'ALIGNMENT_NOT_OBSERVE_ONLY' in \
        status['admission']['blockers']


def test_all_numeric_limits_must_be_positive():
    try:
        AlignmentHandoffShadowCore(arm_timeout_s=0.0)
    except ValueError as exc:
        assert 'arm_timeout_s' in str(exc)
    else:
        raise AssertionError('zero timeout must be rejected')
    with pytest.raises(ValueError):
        AlignmentHandoffShadowCore(gateway_timeout_s=float('nan'))
    with pytest.raises(ValueError):
        AlignmentHandoffShadowCore(require_live_cancel_proof='false')


def test_live_proof_is_opt_in_and_absent_proof_fails_closed():
    assert ready_core().evaluate(1.0)[1]['active'] is True
    core = proof_core()
    angular, status = evaluate_proof(core)
    assert angular == 0.0
    assert status['active'] is False
    assert status['live_cancel_proof']['required'] is True
    assert 'GATEWAY_NOT_RECEIVED' in status['admission']['blockers']
    assert 'CANCEL_GUARD_NOT_RECEIVED' in status['admission']['blockers']


def test_fresh_exact_goal_proof_admits_only_shadow_command():
    core = proof_core()
    core.update_gateway(gateway_proof(), 1.0)
    core.update_cancel_guard(cancel_guard_proof(), 1.0)
    angular, status = evaluate_proof(core, 1.1)
    assert angular == 0.25
    assert status['active'] is True
    assert status['admission']['blockers'] == []
    assert status['safety']['real_motion_output'] is False
    assert status['live_cancel_proof']['goal_uuid'] == '1' * 32


@pytest.mark.parametrize('field,value', [
    ('state', 'NAVIGATION'),
    ('released', False),
    ('stop_reason', 'NAV_INPUT_OR_TOPOLOGY_LOST'),
    ('raw_publishers', 1),
    ('goal_active', True),
    ('cancel_guard_publishers', 0),
])
def test_invalid_gateway_proof_fails_closed(field, value):
    core = proof_core()
    gateway = gateway_proof()
    gateway[field] = value
    core.update_gateway(gateway, 1.0)
    core.update_cancel_guard(cancel_guard_proof(), 1.0)
    angular, status = evaluate_proof(core)
    assert angular == 0.0
    assert 'GATEWAY_HANDOFF_PROOF_INVALID' in status['admission']['blockers']


@pytest.mark.parametrize('field,value', [
    ('state', 'WAITING_FOR_CANCEL'),
    ('goal_uuid', '0' * 32),
    ('goal_uuid', 'not-a-uuid'),
    ('send_count', 2),
    ('cancel_status_seen', False),
    ('goal_status', 4),
    ('arm_fresh', False),
    ('cmd_vel_fresh_zero', False),
    ('odom_fresh_stationary', False),
    ('raw_cmd_publishers', 1),
    ('blockers', ['ROBOT_MOVING']),
])
def test_invalid_cancel_guard_proof_fails_closed(field, value):
    core = proof_core()
    guard = cancel_guard_proof()
    guard[field] = value
    core.update_gateway(gateway_proof(), 1.0)
    core.update_cancel_guard(guard, 1.0)
    angular, status = evaluate_proof(core)
    assert angular == 0.0
    assert 'CANCEL_GUARD_PROOF_INVALID' in status['admission']['blockers']


def test_stale_malformed_and_duplicate_proof_fails_closed():
    core = proof_core()
    core.update_gateway(gateway_proof(), 1.0)
    core.update_cancel_guard(cancel_guard_proof(), 1.0)
    core.update_mission(mission(), 1.4)
    core.update_alignment(alignment(), 1.4)
    core.update_arm(True, 1.4)
    angular, status = evaluate_proof(core, 1.4)
    assert angular == 0.0
    assert 'GATEWAY_TIMEOUT' in status['admission']['blockers']
    assert 'CANCEL_GUARD_TIMEOUT' in status['admission']['blockers']

    core.update_gateway(gateway_proof(), 1.4)
    core.mark_cancel_guard_error('bad json', 1.4)
    angular, status = evaluate_proof(core, 1.4)
    assert angular == 0.0
    assert 'CANCEL_GUARD_INVALID_PAYLOAD' in status['admission']['blockers']

    core.update_cancel_guard(cancel_guard_proof(), 1.4)
    angular, status = evaluate_proof(core, 1.4, gateway_publishers=2)
    assert angular == 0.0
    assert 'GATEWAY_PUBLISHER_INVALID' in status['admission']['blockers']

    angular, status = evaluate_proof(core, 1.4, guard_publishers=0)
    assert angular == 0.0
    assert 'CANCEL_GUARD_PUBLISHER_INVALID' in \
        status['admission']['blockers']


def test_empty_or_malformed_safety_proof_fails_closed():
    core = proof_core()
    core.update_gateway({}, 1.0)
    guard = deepcopy(cancel_guard_proof())
    guard['safety'] = 'bad'
    core.update_cancel_guard(guard, 1.0)
    angular, status = evaluate_proof(core)
    assert angular == 0.0
    assert 'GATEWAY_HANDOFF_PROOF_INVALID' in status['admission']['blockers']
    assert 'CANCEL_GUARD_PROOF_INVALID' in status['admission']['blockers']
