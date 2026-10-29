"""Offline tests for subscriber-only alignment diagnostics."""

from pathlib import Path

from fire_robot_perception.alignment_diagnostics import (
    AlignmentDiagnosticsAccumulator,
    progress_line,
)


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def make_preview(
        *, state='ALIGNING_RIGHT', direction='RIGHT', error_x=0.4,
        target=-0.3, proposed=-0.1, slew=True, saturated=False):
    """Create one representative alignment preview payload."""
    return {
        'observe_only': True,
        'state': state,
        'limits': {'max_angular_speed_rad_s': 0.50},
        'safety': {
            'motion_command_publisher': False,
            'actuator_command_publisher': False,
        },
        'preview': {
            'state': state,
            'direction': direction,
            'error_x_normalized': error_x,
            'target_angular_z_rad_s': target,
            'proposed_angular_z_rad_s': proposed,
            'slew_limited': slew,
            'saturated': saturated,
        },
    }


def test_accumulator_reports_limits_and_direction_changes():
    """Summary exposes chatter, slew limiting and preview magnitude."""
    accumulator = AlignmentDiagnosticsAccumulator()
    accumulator.add(make_preview(), receive_time=1.0)
    accumulator.add(make_preview(
        state='CENTERED', direction='STOP', error_x=0.0,
        target=0.0, proposed=0.0, slew=False), receive_time=1.1)
    accumulator.add(make_preview(
        state='ALIGNING_LEFT', direction='LEFT', error_x=-0.5,
        target=0.4, proposed=0.08, saturated=True), receive_time=1.2)

    report = accumulator.summary()
    assert report['mode'] == 'OBSERVE_ONLY'
    assert report['subscriber_only'] is True
    assert report['messages'] == 3
    assert report['observe_only_violations'] == 0
    assert report['safety_contract_violations'] == 0
    assert report['direction_sign_violations'] == 0
    assert report['stop_nonzero_violations'] == 0
    assert report['speed_limit_violations'] == 0
    assert report['direction_changes'] == 1
    assert report['slew_limited_frames'] == 2
    assert report['saturated_frames'] == 1
    assert report['message_rate_hz'] == 10.0
    assert report['proposed_angular_z_rad_s']['max'] == 0.08


def test_accumulator_flags_unsafe_or_inconsistent_preview():
    """Diagnostics must expose safety, sign, stop and speed violations."""
    accumulator = AlignmentDiagnosticsAccumulator()
    unsafe = make_preview(proposed=0.6)
    unsafe['observe_only'] = False
    unsafe['safety']['motion_command_publisher'] = True
    accumulator.add(unsafe, receive_time=1.0)
    stopped = make_preview(
        state='CENTERED', direction='STOP', error_x=0.0,
        target=0.0, proposed=0.1, slew=False)
    accumulator.add(stopped, receive_time=1.1)

    report = accumulator.summary()
    assert report['observe_only_violations'] == 1
    assert report['safety_contract_violations'] == 1
    assert report['direction_sign_violations'] == 1
    assert report['stop_nonzero_violations'] == 1
    assert report['speed_limit_violations'] == 1


def test_progress_line_contains_action_free_preview_values():
    """Progress output shows direction and both bounded yaw values."""
    line = progress_line(make_preview())
    assert 'state=ALIGNING_RIGHT' in line
    assert 'direction=RIGHT' in line
    assert 'target_wz=-0.300' in line
    assert 'preview_wz=-0.100' in line


def test_diagnostics_source_has_no_publishers():
    """The alignment diagnostic remains subscriber-only."""
    source = (
        PACKAGE_DIR / 'fire_robot_perception' /
        'alignment_diagnostics.py'
    ).read_text()
    assert 'create_' + 'publisher' not in source
