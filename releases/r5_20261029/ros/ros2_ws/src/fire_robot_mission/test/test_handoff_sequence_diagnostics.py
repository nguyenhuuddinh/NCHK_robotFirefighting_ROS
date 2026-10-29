"""Tests for the subscriber-only integrated handoff report."""

from pathlib import Path

from fire_robot_mission.handoff_sequence_diagnostics import (
    HandoffSequenceAccumulator,
)


UUID = '1' * 32


def mission():
    return {
        'schema_version': 1, 'state': 'FIRE_TRACKING',
        'observe_only': True,
        'perception': {'confirmed': True},
        'safety': {'motion_command_publisher': False},
    }


def gateway(state):
    released = state in {'RELEASED_STOPPED', 'HANDOFF_READY'}
    return {
        'schema_version': 1, 'state': state, 'enabled': True,
        'fire_latched': state != 'NAVIGATION',
        'stop_reason': 'FIRE_ACQUIRED',
        'raw_publishers': 0 if released else 1,
        'released': released, 'odom_stationary': True,
        'cmd_vel_fresh_zero': True,
        'handoff_ready': state == 'HANDOFF_READY',
        'goal_active': False,
        'cancel_guard_publishers': 1,
        'command': {'linear_x_m_s': 0.0,
                    'angular_z_rad_s': 0.0},
        'safety': {'raw_command_publisher': not released},
    }


def guard():
    return {
        'schema_version': 1, 'state': 'CANCEL_CONFIRMED_STOPPED',
        'enabled': True, 'goal_uuid': UUID, 'send_count': 1,
        'cancel_service_accepted': True, 'cancel_status_seen': True,
        'goal_status': 5, 'cmd_vel_fresh_zero': True,
        'odom_fresh_stationary': True, 'raw_cmd_publishers': 0,
        'blockers': [],
        'safety': {'motion_command_publisher': False},
    }


def handoff():
    return {
        'schema_version': 1, 'state': 'ARMED_SHADOW_TRACKING',
        'active': True, 'shadow_only': True,
        'admission': {'blockers': []},
        'live_cancel_proof': {
            'required': True, 'gateway_publishers': 1,
            'cancel_guard_publishers': 1,
            'gateway_state': 'HANDOFF_READY',
            'cancel_guard_state': 'CANCEL_CONFIRMED_STOPPED',
            'goal_uuid': UUID,
        },
        'safety': {'real_motion_output': False},
    }


def arbiter():
    return {
        'schema_version': 1, 'state': 'ALIGNMENT',
        'shadow_only': True, 'owner': 'ALIGNMENT',
        'requests': {'navigation': False, 'alignment': True,
                     'conflict': False},
        'handoff': {'active': True},
        'safety': {'real_motion_output': False,
                   'exclusive_owner': True},
    }


def full_sequence():
    accumulator = HandoffSequenceAccumulator()
    accumulator.add('gateway', gateway('NAVIGATION'), 0.0)
    accumulator.add('mission', mission(), 1.0)
    accumulator.add('gateway', gateway('STOPPING'), 2.0)
    accumulator.add('gateway', gateway('RELEASED_STOPPED'), 3.0)
    accumulator.add('guard', guard(), 4.0)
    accumulator.add('gateway', gateway('HANDOFF_READY'), 4.1)
    accumulator.add('handoff', handoff(), 5.0)
    accumulator.add('arbiter', arbiter(), 6.0)
    return accumulator


def test_complete_sequence_is_telemetry_pass_only():
    report = full_sequence().summary()
    assert report['status'] == 'PASS_TELEMETRY_ONLY'
    assert report['missing_stages'] == []
    assert report['contract_violations'] == {}
    assert report['order_violations'] == []
    assert report['guard_goal_uuid'] == report['handoff_goal_uuid'] == UUID
    assert report['subscriber_only'] is True


def test_missing_stages_never_pass():
    accumulator = HandoffSequenceAccumulator()
    accumulator.add('mission', mission(), 1.0)
    report = accumulator.summary()
    assert report['status'] == 'INCOMPLETE'
    assert 'GOAL_CANCEL_CONFIRMED' in report['missing_stages']


def test_wrong_goal_uuid_and_early_alignment_fail():
    accumulator = full_sequence()
    payload = handoff()
    payload['live_cancel_proof']['goal_uuid'] = '2' * 32
    accumulator.add('handoff', payload, 7.0)
    report = accumulator.summary()
    assert report['status'] == 'FAIL'
    assert 'HANDOFF_GOAL_UUID_MISMATCH' in report['order_violations']

    accumulator = HandoffSequenceAccumulator()
    accumulator.add('arbiter', arbiter(), 0.0)
    accumulator.add('handoff', handoff(), 2.0)
    report = accumulator.summary()
    assert report['status'] == 'FAIL'
    assert 'ARBITER_SHADOW_ALIGNMENT_BEFORE_HANDOFF_SHADOW_ACTIVE' in \
        report['order_violations']


def test_invalid_safety_and_json_fail():
    accumulator = HandoffSequenceAccumulator()
    payload = gateway('RELEASED_STOPPED')
    payload['raw_publishers'] = 1
    accumulator.add('gateway', payload, 0.0)
    accumulator.add_invalid_json('guard')
    report = accumulator.summary()
    assert report['status'] == 'FAIL'
    assert report['contract_violations']['RAW_RELEASE_CONTRACT_INVALID'] == 1
    assert report['invalid_json_counts']['guard'] == 1


def test_navigation_cannot_resume_after_release():
    accumulator = full_sequence()
    accumulator.add('gateway', gateway('NAVIGATION'), 7.0)
    assert accumulator.summary()['contract_violations'][
        'NAVIGATION_AFTER_RELEASE'] == 1


def test_diagnostic_has_no_command_or_action_interfaces():
    source = (Path(__file__).resolve().parents[1] /
              'fire_robot_mission' /
              'handoff_sequence_diagnostics.py').read_text()
    assert 'create_publisher(' not in source
    assert 'create_client(' not in source
    assert 'create_service(' not in source
    assert 'ActionClient(' not in source
