"""No ROS init, ports, goals or publishers: pure gateway transitions only."""
import pytest
from test_live_nav_stop_gateway import (
    ZERO, MOVING, RAW_SOURCE, mission, ready_core, navigate, step,
)

OLD = 'ab' * 16
NEW = 'cd' * 16
OFF = {'pump_pwm_commanded_on': False, 'servo_phase': 'IDLE',
       'servo_pwm_commanded_on': False}


def no_fire(active):
    data = mission() if active else mission('IDLE', 'HOLD_STOP', goal=False)
    data['perception'] = {'detected_now': False, 'confirmed': False}
    return data


def feed(core, now, active=False):
    core.update_mission(no_fire(active), now)
    core.update_nav(MOVING if active else ZERO, now)
    core.update_odom(ZERO, now)
    core.update_cmd(ZERO, now)
    core.update_arm(True, now)
    core.update_actuator(dict(OFF), now)


def finish_goal(status=4, pre_fault=False):
    core = ready_core()
    navigate(core)
    if pre_fault:
        feed(core, 10.15, active=True)
        core.update_arm(False, 10.15)
        assert step(core, 10.16, owned=True, raw=RAW_SOURCE)[2]['state'] == 'STOPPING'
    feed(core, 10.20)
    core.update_statuses([(OLD, status)], 10.20)
    linear, angular, report, _ = step(core, 10.21, owned=True, raw=RAW_SOURCE)
    assert (linear, angular, report['state']) == (0.0, 0.0, 'STOPPING')
    feed(core, 10.85)
    result = step(core, 10.85, owned=True, raw=RAW_SOURCE)
    assert result[3] == 'RELEASE'
    core.publisher_released()
    return core


def idle_hold(core):
    for index in range(12):
        now = 10.86 + index * 0.10
        feed(core, now)
        result = step(core, now)
        assert result[0:2] == (0.0, 0.0)
        assert result[3] is None
    return core


def propose(core, now=12.0):
    feed(core, now, active=True)
    core.update_statuses([(OLD, 4 if core.normal_terminal == 4 else 5), (NEW, 2)], now)


@pytest.mark.parametrize('status', [4, 5])
def test_completed_or_canceled_goal_accepts_new_uuid_after_clear_hold(status):
    core = idle_hold(finish_goal(status))
    assert core.normal_terminal == status
    assert core.normal_ready_at is not None
    propose(core)
    linear, angular, report, action = step(core, 12.01)
    assert (linear, angular, action) == (0.0, 0.0, None)
    assert report['cycle_goal_uuid'] == NEW
    assert report['state'] == 'WAITING_FOR_INPUT'
    assert OLD in core.used_goal_uuids
    assert core.normal_terminal is None
    assert step(core, 12.02)[3] == 'CREATE'
    core.publisher_created(12.02)
    linear, angular, report, action = step(core, 12.03, owned=True, raw=RAW_SOURCE)
    assert (linear, angular, report['state'], action) == (0.05, 0.2, 'NAVIGATION', None)


def test_aborted_goal_never_uses_normal_rearm_path():
    core = idle_hold(finish_goal(6))
    assert core.normal_terminal is None
    propose(core)
    assert step(core, 12.01)[2]['state'] == 'RELEASED_STOPPED'


def test_late_terminal_result_cannot_clear_prior_arm_fault():
    core = idle_hold(finish_goal(4, pre_fault=True))
    assert core.normal_terminal is None
    assert core.stop_reason == 'NAV_INPUT_OR_TOPOLOGY_LOST'
    propose(core)
    assert step(core, 12.01)[2]['state'] == 'RELEASED_STOPPED'


def test_new_goal_without_idle_hold_never_rearms():
    core = finish_goal()
    propose(core, 10.9)
    assert step(core, 10.91)[2]['state'] == 'RELEASED_STOPPED'


def test_sparse_idle_samples_do_not_prove_continuous_clear():
    core = finish_goal()
    for now in (10.86, 11.96):
        feed(core, now)
        step(core, now)
    assert core.normal_ready_at is None
    propose(core)
    assert step(core, 12.01)[2]['state'] == 'RELEASED_STOPPED'


def test_old_ready_proof_expires_even_if_new_inputs_are_fresh():
    core = idle_hold(finish_goal())
    propose(core, 13.0)
    assert step(core, 13.01)[2]['state'] == 'RELEASED_STOPPED'


@pytest.mark.parametrize('change', [
    'arm_false', 'stale_arm', 'stale_odom', 'moving_odom', 'nonzero_cmd',
    'stale_cmd', 'stale_mission', 'blocked_health', 'bad_contract',
    'pump_on', 'servo_on', 'stale_actuator', 'fire_candidate',
    'missing_nav', 'bad_nav', 'raw_conflict', 'missing_safety',
    'missing_status', 'duplicate_mission', 'duplicate_arm',
    'duplicate_odom', 'ambiguous_goal', 'same_goal', 'used_goal',
    'wrong_terminal', 'duplicate_terminal', 'future_status', 'disabled',
])
def test_invalid_next_goal_never_acquires_or_forwards(change):
    core = idle_hold(finish_goal())
    propose(core)
    kw = {}
    if change == 'arm_false': core.update_arm(False, 12.0)
    elif change == 'stale_arm': core.arm_at = 11.0
    elif change == 'stale_odom': core.odom_at = 11.0
    elif change == 'moving_odom': core.update_odom(MOVING, 12.0)
    elif change == 'nonzero_cmd': core.update_cmd(MOVING, 12.0)
    elif change == 'stale_cmd': core.cmd_at = 11.0
    elif change == 'stale_mission': core.mission_at = 11.0
    elif change == 'blocked_health': core.mission['health']['status'] = 'BLOCKED'
    elif change == 'bad_contract': core.mission['safety']['collision_check_bypassed'] = True
    elif change == 'pump_on': core.actuator['pump_pwm_commanded_on'] = True
    elif change == 'servo_on': core.actuator['servo_pwm_commanded_on'] = True
    elif change == 'stale_actuator': core.actuator_at = 11.0
    elif change == 'fire_candidate': core.mission['perception']['detected_now'] = True
    elif change == 'missing_nav': kw['nav'] = []
    elif change == 'bad_nav': core.update_nav((2.0, 0, 0, 0, 0, 0), 12.0)
    elif change == 'raw_conflict': kw['raw'] = RAW_SOURCE
    elif change == 'missing_safety': kw['safety'] = []
    elif change == 'missing_status': kw['status'] = 0
    elif change == 'duplicate_mission': kw['mission_count'] = 2
    elif change == 'duplicate_arm': kw['arm_count'] = 2
    elif change == 'duplicate_odom': kw['odom_count'] = 2
    elif change == 'ambiguous_goal': core.update_statuses([(OLD, 4), (NEW, 2), ('ef' * 16, 2)], 12.0)
    elif change == 'same_goal': core.update_statuses([(OLD, 2)], 12.0)
    elif change == 'used_goal': core.used_goal_uuids.add(NEW)
    elif change == 'wrong_terminal': core.update_statuses([(OLD, 6), (NEW, 2)], 12.0)
    elif change == 'duplicate_terminal': core.update_statuses([(OLD, 4), (OLD, 4), (NEW, 2)], 12.0)
    elif change == 'future_status': core.status_at = 13.0
    elif change == 'disabled': core.enabled = False
    linear, angular, report, action = step(core, 12.01, **kw)
    assert (linear, angular) == (0.0, 0.0)
    assert action != 'CREATE'
    assert report['cycle_goal_uuid'] == OLD


def test_bad_idle_sample_resets_hold_and_must_be_rebuilt():
    core = idle_hold(finish_goal())
    feed(core, 12.0)
    core.update_cmd(MOVING, 12.0)
    step(core, 12.0)
    assert core.normal_ready_at is None
    for i in range(12):
        now = 12.1 + i * 0.1
        feed(core, now)
        step(core, now)
    assert core.normal_ready_at is not None
    propose(core, 13.3)
    assert step(core, 13.31)[2]['cycle_goal_uuid'] == NEW


def test_fire_after_terminal_retains_fire_latch():
    core = idle_hold(finish_goal())
    feed(core, 12.0)
    core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 12.0)
    step(core, 12.01)
    assert core.fire_latched
    propose(core, 12.1)
    assert step(core, 12.11)[2]['cycle_goal_uuid'] == OLD


def test_status_before_mission_and_command_waits_without_losing_idle_proof():
    core = idle_hold(finish_goal())
    feed(core, 12.0)
    core.update_statuses([(OLD, 4), (NEW, 2)], 12.0)
    core.nav_at = 11.0
    linear, angular, report, action = step(core, 12.01)
    assert (linear, angular, action) == (0.0, 0.0, None)
    assert report['cycle_goal_uuid'] == OLD
    assert core.normal_ready_at is not None
    feed(core, 12.1, active=True)
    core.nav_at = 11.0
    assert step(core, 12.11)[2]['cycle_goal_uuid'] == OLD
    core.update_nav(MOVING, 12.12)
    assert step(core, 12.13)[2]['cycle_goal_uuid'] == NEW


def test_first_command_after_idle_proof_expiry_cannot_rearm():
    core = idle_hold(finish_goal())
    feed(core, 12.0)
    core.update_statuses([(OLD, 4), (NEW, 2)], 12.0)
    core.nav_at = 11.0
    assert step(core, 12.01)[2]['cycle_goal_uuid'] == OLD
    feed(core, 12.5, active=True)
    assert step(core, 12.51)[2]['cycle_goal_uuid'] == OLD


@pytest.mark.parametrize('fault', ['arm', 'odom', 'nav', 'raw', 'safety', 'status', 'health'])
def test_terminal_result_simultaneous_with_fault_is_not_normal_completion(fault):
    core = ready_core()
    navigate(core)
    feed(core, 10.2)
    core.update_statuses([(OLD, 4)], 10.2)
    kwargs = {'owned': True, 'raw': RAW_SOURCE}
    if fault == 'arm': core.update_arm(False, 10.2)
    elif fault == 'odom': core.odom_at = 9.0
    elif fault == 'nav': core.nav_at = 9.0
    elif fault == 'raw': kwargs['raw'] = RAW_SOURCE * 2
    elif fault == 'safety': kwargs['safety'] = []
    elif fault == 'status': kwargs['status'] = 0
    elif fault == 'health': core.mission['health']['status'] = 'BLOCKED'
    linear, angular, report, action = step(core, 10.21, **kwargs)
    assert (linear, angular) == (0.0, 0.0)
    assert report['state'] == 'STOPPING'
    assert core.normal_terminal is None
