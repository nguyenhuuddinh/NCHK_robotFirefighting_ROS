"""Pure diagnostic checks for the one-shot alignment telemetry."""

from fire_robot_mission.live_alignment_diagnostics import (
    AlignmentAccumulator,
)


def frame(state, angular=0.0, **overrides):
    """Make a minimal valid alignment state frame."""
    payload = {
        'schema_version': 1,
        'state': state,
        'enabled': state != 'DISABLED',
        'one_shot': True,
        'goal_uuid': 'a' * 32 if state not in {
            'DISABLED', 'WAITING_FOR_ADMISSION'} else None,
        'publisher_owned': state in {'TURNING', 'STOPPING'},
        'command': {
            'linear_x_m_s': 0.0,
            'angular_z_rad_s': angular,
        },
        'stop_reason': '',
        'blockers': [],
        'safety': {
            'pump_command_publisher': False,
            'linear_motion': False,
            'requires_operator_arm': True,
            'fail_closed_zero': state != 'TURNING',
        },
    }
    payload.update(overrides)
    return payload


def test_valid_one_shot_turn_stop_release_and_odom():
    """Report command and odom separately across a normal one-shot run."""
    report = AlignmentAccumulator()
    report.add(frame('WAITING_FOR_ADMISSION'), 1.0)
    report.add(frame('ACQUIRING'), 1.1)
    report.add(frame('TURNING', 0.60), 1.2)
    report.add_odom(0.08)
    report.add(frame('STOPPING', stop_reason='TARGET_CENTERED'), 1.3)
    report.add(frame('COMPLETE', publisher_owned=True), 1.9)
    report.add(frame('COMPLETE', publisher_owned=False), 2.0)
    summary = report.summary()
    assert summary['contract_violations'] == 0
    assert summary['max_abs_command_angular_z_rad_s'] == 0.60
    assert summary['max_abs_odom_angular_z_rad_s_during_turn'] == 0.08
    assert summary['odom_samples_during_turn'] == 1
    assert summary['stop_reason_counts']['TARGET_CENTERED'] == 1
    assert summary['physical_motion_verified'] is False


def test_stop_input_is_latched_once_for_failure_diagnosis():
    report = AlignmentAccumulator()
    evidence = {
        'mission_state': 'FIRE_TRACKING',
        'preview_state': 'NO_TARGET',
        'preview_direction': 'STOP',
        'error_x_normalized': None,
        'target_angular_z_rad_s': 0.0,
        'preview_receive_age_ms': 22.0,
    }
    report.add(frame('STOPPING', stop_reason='ALIGNMENT_DIRECTION_INVALID',
                     stop_alignment_input=evidence), 1.0)
    report.add(frame('COMPLETE', stop_reason='ALIGNMENT_DIRECTION_INVALID',
                     stop_alignment_input={'preview_state': 'CENTERED'}), 1.1)
    assert report.summary()['stop_alignment_input'] == evidence


def test_nonzero_outside_turn_and_linear_command_violate_contract():
    """Flag movement reported in STOPPING or any linear command."""
    report = AlignmentAccumulator()
    report.add(frame('TURNING', 0.11), 1.0)
    report.add(frame('STOPPING', 0.01), 1.1)
    bad = frame('TURNING', 0.10)
    bad['command']['linear_x_m_s'] = 0.01
    report.add(bad, 1.2)
    assert report.summary()['contract_violations'] == 2


def test_command_above_live_alignment_cap_violates_contract():
    report = AlignmentAccumulator()
    report.add(frame('TURNING', 0.61), 1.0)
    assert report.summary()['contract_violations'] == 1


def test_turn_after_complete_is_violation():
    """One-shot state machine must never resume after release."""
    report = AlignmentAccumulator()
    report.add(frame('COMPLETE', publisher_owned=False), 1.0)
    report.add(frame('TURNING', 0.1), 1.1)
    assert report.summary()['contract_violations'] == 1


def test_disabled_is_zero_and_never_owns_raw():
    """Disabled telemetry cannot report a command publisher."""
    report = AlignmentAccumulator()
    report.add(frame('DISABLED'), 1.0)
    report.add(frame('DISABLED', publisher_owned=True), 1.1)
    assert report.summary()['contract_violations'] == 1


def test_odom_outside_turn_is_not_counted():
    """Do not attribute other controllers' motion to alignment."""
    report = AlignmentAccumulator()
    report.add(frame('WAITING_FOR_ADMISSION'), 1.0)
    report.add_odom(0.15)
    assert report.summary()['odom_samples_during_turn'] == 0
