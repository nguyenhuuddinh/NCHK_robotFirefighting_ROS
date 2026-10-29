"""Tests for read-only cancellation guard reporting."""

from fire_robot_mission.nav2_cancel_guard_diagnostics import GuardAccumulator


def test_guard_report_counts_one_shot_and_completion():
    accumulator = GuardAccumulator()
    base = {
        'state': 'REQUEST_SENT',
        'goal_uuid': 'ab' * 16,
        'send_count': 1,
        'blockers': [],
        'cancel_service_accepted': False,
        'cancel_status_seen': False,
        'arm_fresh': True,
        'nav_shadow_endpoints': 3,
        'nav_shadow_fingerprint_ok': True,
        'raw_cmd_publishers': 0,
        'goal_status': 2,
        'cmd_vel_fresh_zero': True,
        'odom_fresh_stationary': True,
        'safety': {
            'motion_command_publisher': False,
            'actuator_command_publisher': False,
            'requires_operator_arm': True,
            'requires_shadow_nav2': True,
        },
    }
    accumulator.add(base, 10.0)
    completed = dict(base)
    completed.update({
        'state': 'CANCEL_CONFIRMED_STOPPED',
        'cancel_service_accepted': True,
        'cancel_status_seen': True,
        'goal_status': 5,
    })
    accumulator.add(completed, 10.1)
    report = accumulator.summary()
    assert report['messages'] == 2
    assert report['max_send_count'] == 1
    assert report['contract_violations'] == 0
    assert report['state_counts']['CANCEL_CONFIRMED_STOPPED'] == 1


def test_guard_report_flags_false_completion():
    accumulator = GuardAccumulator()
    accumulator.add({
        'state': 'CANCEL_CONFIRMED_STOPPED',
        'goal_uuid': 'ab' * 16,
        'send_count': 2,
        'blockers': [],
        'cancel_service_accepted': False,
        'goal_status': 2,
        'cmd_vel_fresh_zero': False,
        'odom_fresh_stationary': False,
        'safety': {},
    }, 10.0)
    assert accumulator.summary()['contract_violations'] > 0


def test_guard_report_flags_request_with_unproven_shadow_isolation():
    accumulator = GuardAccumulator()
    accumulator.add({
        'state': 'REQUEST_SENT',
        'goal_uuid': 'ab' * 16,
        'send_count': 1,
        'blockers': [],
        'nav_shadow_endpoints': 3,
        'nav_shadow_fingerprint_ok': False,
        'raw_cmd_publishers': 0,
        'safety': {
            'motion_command_publisher': False,
            'actuator_command_publisher': False,
            'requires_operator_arm': True,
            'requires_shadow_nav2': True,
        },
    }, 10.0)
    assert accumulator.summary()['contract_violations'] == 1
