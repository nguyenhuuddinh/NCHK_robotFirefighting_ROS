"""Offline tests for motion arbiter shadow diagnostics."""

from fire_robot_mission.motion_arbiter_shadow_diagnostics import (
    MotionArbiterShadowDiagnosticsAccumulator,
    progress_line,
)


def payload(state='ALIGNMENT', owner='ALIGNMENT', linear=0.0, angular=0.2):
    """Build one representative arbiter shadow state."""
    return {
        'shadow_only': True,
        'state': state,
        'owner': owner,
        'requests': {
            'navigation': state == 'NAVIGATION',
            'alignment': state == 'ALIGNMENT',
        },
        'admission': {'accepted': True, 'blockers': []},
        'shadow_command': {
            'linear_x_m_s': linear,
            'angular_z_rad_s': angular,
            'linear_clamped': False,
            'angular_clamped': False,
        },
        'safety': {
            'real_motion_output': False,
            'actuator_command_publisher': False,
            'nav2_action_client': False,
            'exclusive_owner': True,
        },
    }


def test_valid_alignment_state_has_no_violation():
    accumulator = MotionArbiterShadowDiagnosticsAccumulator()
    accumulator.add(payload(), receive_time=1.0)
    accumulator.add(payload(), receive_time=1.1)
    report = accumulator.summary()
    assert report['messages'] == 2
    assert report['shadow_contract_violations'] == 0
    assert report['owner_state_violations'] == 0
    assert report['output_limit_violations'] == 0
    assert report['max_abs_shadow_angular_z_rad_s'] == 0.2


def test_nonzero_safe_stop_and_owner_mismatch_are_reported():
    accumulator = MotionArbiterShadowDiagnosticsAccumulator()
    unsafe = payload('SAFE_STOP', 'ALIGNMENT', 0.1, 0.2)
    unsafe['requests'] = {'navigation': True, 'alignment': True}
    accumulator.add(unsafe, receive_time=1.0)
    report = accumulator.summary()
    assert report['zero_state_violations'] == 1
    assert report['owner_state_violations'] == 1
    assert report['exclusive_owner_violations'] == 1


def test_owner_specific_limits_are_reported():
    accumulator = MotionArbiterShadowDiagnosticsAccumulator()
    accumulator.add(
        payload('NAVIGATION', 'NAVIGATION', 0.11, 1.01),
        receive_time=1.0)
    accumulator.add(
        payload('ALIGNMENT', 'ALIGNMENT', 0.0, 0.36),
        receive_time=1.1)
    assert accumulator.summary()['output_limit_violations'] == 2


def test_navigation_eight_tenths_is_valid_but_higher_is_not():
    accumulator = MotionArbiterShadowDiagnosticsAccumulator()
    accumulator.add(
        payload('NAVIGATION', 'NAVIGATION', 0.0, 0.8),
        receive_time=1.0)
    accumulator.add(
        payload('NAVIGATION', 'NAVIGATION', 0.0, 0.81),
        receive_time=1.1)
    assert accumulator.summary()['output_limit_violations'] == 1


def test_broken_shadow_contract_is_reported():
    accumulator = MotionArbiterShadowDiagnosticsAccumulator()
    unsafe = payload()
    unsafe['safety']['real_motion_output'] = True
    accumulator.add(unsafe, receive_time=1.0)
    assert accumulator.summary()['shadow_contract_violations'] == 1


def test_progress_line_contains_ownership_fields():
    line = progress_line(payload())
    assert 'state=ALIGNMENT' in line
    assert 'owner=ALIGNMENT' in line
    assert 'wz=+0.200' in line
