"""Offline tests for alignment handoff shadow diagnostics."""

from fire_robot_mission.handoff_shadow_diagnostics import (
    HandoffShadowDiagnosticsAccumulator,
    progress_line,
)


def payload(active=True, angular=0.2):
    """Build one representative handoff state."""
    return {
        'shadow_only': True,
        'state': 'ARMED_SHADOW_TRACKING' if active else 'BLOCKED',
        'active': active,
        'operator_arm': {'requested': True, 'fresh': True},
        'admission': {'eligible': active, 'blockers': []},
        'alignment': {'state': 'ALIGNING_LEFT', 'clamped': False},
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


def test_valid_active_shadow_state_has_no_violation():
    accumulator = HandoffShadowDiagnosticsAccumulator()
    accumulator.add(payload(), receive_time=1.0)
    accumulator.add(payload(), receive_time=1.1)
    report = accumulator.summary()
    assert report['messages'] == 2
    assert report['shadow_contract_violations'] == 0
    assert report['linear_motion_violations'] == 0
    assert report['nonzero_while_inactive_violations'] == 0
    assert report['max_abs_shadow_angular_z_rad_s'] == 0.2
    assert report['active_direction_counts'] == {'LEFT': 2}
    assert report['directional_coverage_complete'] is False
    assert report['direction_sign_violations'] == 0


def test_left_center_right_shadow_coverage_and_transitions():
    accumulator = HandoffShadowDiagnosticsAccumulator()
    left = payload(angular=0.2)
    centered = payload(angular=0.0)
    centered['state'] = 'ARMED_SHADOW_CENTERED'
    centered['alignment']['state'] = 'CENTERED'
    right = payload(angular=-0.2)
    right['alignment']['state'] = 'ALIGNING_RIGHT'

    accumulator.add(left, receive_time=1.0)
    accumulator.add(centered, receive_time=1.1)
    accumulator.add(right, receive_time=1.2)
    report = accumulator.summary()
    assert report['active_direction_counts'] == {
        'CENTERED': 1, 'LEFT': 1, 'RIGHT': 1}
    assert report['active_direction_transitions'] == {
        'CENTERED->RIGHT': 1, 'LEFT->CENTERED': 1}
    assert report['directional_coverage_complete'] is True
    assert report['direction_sign_violations'] == 0
    assert report['active_alignment_state_violations'] == 0


def test_wrong_direction_and_centered_nonzero_are_reported():
    accumulator = HandoffShadowDiagnosticsAccumulator()
    wrong_left = payload(angular=-0.1)
    wrong_right = payload(angular=0.1)
    wrong_right['alignment']['state'] = 'ALIGNING_RIGHT'
    wrong_center = payload(angular=0.1)
    wrong_center['state'] = 'ARMED_SHADOW_CENTERED'
    wrong_center['alignment']['state'] = 'CENTERED'
    for index, state in enumerate((wrong_left, wrong_right, wrong_center)):
        accumulator.add(state, receive_time=1.0 + index * 0.1)
    assert accumulator.summary()['direction_sign_violations'] == 3


def test_inactive_frame_breaks_transition_continuity():
    accumulator = HandoffShadowDiagnosticsAccumulator()
    accumulator.add(payload(angular=0.2), receive_time=1.0)
    accumulator.add(payload(active=False, angular=0.0), receive_time=1.1)
    right = payload(angular=-0.2)
    right['alignment']['state'] = 'ALIGNING_RIGHT'
    accumulator.add(right, receive_time=1.2)
    assert accumulator.summary()['active_direction_transitions'] == {}


def test_active_unknown_alignment_state_is_reported():
    accumulator = HandoffShadowDiagnosticsAccumulator()
    unknown = payload(angular=0.0)
    unknown['alignment']['state'] = 'NO_TARGET'
    accumulator.add(unknown, receive_time=1.0)
    assert accumulator.summary()['active_alignment_state_violations'] == 1


def test_nonzero_inactive_and_linear_motion_are_reported():
    accumulator = HandoffShadowDiagnosticsAccumulator()
    unsafe = payload(active=False, angular=0.2)
    unsafe['shadow_command']['linear_x_m_s'] = 0.1
    accumulator.add(unsafe, receive_time=1.0)
    report = accumulator.summary()
    assert report['nonzero_while_inactive_violations'] == 1
    assert report['linear_motion_violations'] == 1


def test_broken_shadow_contract_is_reported():
    accumulator = HandoffShadowDiagnosticsAccumulator()
    unsafe = payload()
    unsafe['safety']['real_motion_output'] = True
    accumulator.add(unsafe, receive_time=1.0)
    assert accumulator.summary()['shadow_contract_violations'] == 1


def test_angular_value_above_shadow_limit_is_reported():
    accumulator = HandoffShadowDiagnosticsAccumulator()
    accumulator.add(payload(angular=0.36), receive_time=1.0)
    assert accumulator.summary()['angular_limit_violations'] == 1


def test_progress_line_contains_admission_fields():
    line = progress_line(payload())
    assert 'active=True' in line
    assert 'arm=True/True' in line
    assert 'shadow_wz=+0.200' in line
