"""No-hardware tests for real Nav2 forwarding, fail-closed stop and release."""

import pytest
from rclpy.qos import ReliabilityPolicy

from fire_robot_mission.live_nav_stop_gateway import LiveNavStopCore


ZERO = (0.0,) * 6
MOVING = (0.05, 0.0, 0.0, 0.0, 0.0, 0.2)
NAV_SOURCES = [
    ('/', 'velocity_smoother', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.RELIABLE),
    ('/', 'behavior_server', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.RELIABLE),
    ('/', 'behavior_server', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.RELIABLE),
]
RAW_SOURCE = [
    ('/', 'live_nav_stop_gateway', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.BEST_EFFORT),
]
SAFETY_SOURCE = [
    ('/', 'safety_watchdog', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.BEST_EFFORT),
]
ACTUATOR_SOURCE = [
    ('/', 'serial_bridge_node', 'std_msgs/msg/String',
     ReliabilityPolicy.BEST_EFFORT),
]


def actuator_off(**overrides):
    return {'schema_version': 1, 'mode': 'COMMAND_TELEMETRY_ONLY',
            'receive_age_ms': 10.0,
            'pump_pwm_commanded_on': False,
            'servo_pwm_commanded_on': False, 'servo_phase': 'IDLE',
            **overrides}


def mission(state='NAV2_NAVIGATING', intent='KEEP_NAV2', goal=True,
            eligible=False):
    return {
        'schema_version': 1,
        'observe_only': True,
        'state': state,
        'perception': {'detected_now': False, 'confirmed': False},
        'navigation': {
            'goal_active': goal,
            'observed_motion_owner': 'NAV2' if goal else 'NONE',
        },
        'robot': {'odom_fresh': True, 'stationary': not goal},
        'health': {'status': 'OK', 'blockers': []},
        'decision': {
            'intent': intent,
            'command_published': False,
            'eligible_for_future_alignment_handoff': eligible,
            'readiness_blockers': [] if eligible else ['NAV2_GOAL_ACTIVE'],
        },
        'safety': {
            'motion_command_publisher': False,
            'actuator_command_publisher': False,
            'nav2_action_client': False,
            'collision_check_bypassed': False,
        },
    }


def ready_core(enabled=True):
    core = LiveNavStopCore(enabled=enabled)
    core.update_mission(mission(), 10.0)
    core.update_nav(MOVING, 10.0)
    core.update_odom(ZERO, 10.0)
    core.update_cmd(ZERO, 10.0)
    core.update_arm(True, 10.0)
    core.update_statuses([('ab' * 16, 2)], 10.0)
    return core


@pytest.mark.parametrize('pump_state, stop_reason', [
    ('COMPLETE', 'PULSE_TIME_LIMIT'),
    ('ABORTED', 'IR_NOT_CONFIRMED_SPRAY_RANGE'),
])
def test_completed_fire_cycle_rearms_only_after_off_clear_and_new_uuid(
        pump_state, stop_reason):
    core = ready_core()
    navigate(core)
    old = 'ab' * 16
    new = 'cd' * 16
    assert core.goal_uuid == old
    core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 10.15)
    core.update_odom(ZERO, 10.15)
    core.update_cmd(ZERO, 10.15)
    step(core, 10.16, owned=True, raw=RAW_SOURCE)
    core.update_odom(ZERO, 10.70)
    core.update_cmd(ZERO, 10.70)
    assert step(core, 10.71, owned=True, raw=RAW_SOURCE)[3] == 'RELEASE'
    core.publisher_released()
    core.update_guard({'state': 'CANCEL_CONFIRMED_STOPPED',
                       'goal_uuid': old, 'blockers': []}, 10.72)
    core.update_alignment({'state': 'COMPLETE', 'goal_uuid': old,
                           'stop_reason': 'TARGET_CENTERED',
                           'publisher_owned': False}, 10.72)
    core.update_pump({'state': pump_state, 'goal_uuid': old,
                      'stop_reason': stop_reason,
                      'pump_on': False}, 10.72)
    core.update_review({'mode': 'SHADOW_ONLY',
                        'state': 'NO_FIRE_EVIDENCE', 'goal_uuid': old,
                        'pulse_index': 1, 'blockers': []}, 10.72)
    core.update_actuator(actuator_off(), 10.72)
    idle = mission('IDLE', 'HOLD_STOP', goal=False)
    idle['perception'] = {'detected_now': False, 'confirmed': False}
    for now in (10.8, 11.9):
        core.update_mission(idle, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_actuator(actuator_off(), now)
        assert step(core, now + 0.01)[2]['state'] == 'RELEASED_STOPPED'
    core.update_statuses([(old, 5), (new, 2)], 12.0)
    core.update_mission(mission(), 12.0)
    core.update_nav(MOVING, 12.0)
    core.update_odom(ZERO, 12.0)
    core.update_cmd(ZERO, 12.0)
    core.update_arm(True, 12.0)
    core.update_actuator(actuator_off(), 12.0)
    _, _, report, action = step(core, 12.01)
    assert report['state'] == 'WAITING_FOR_INPUT' and action is None
    assert report['cycle_goal_uuid'] == new
    _, _, report, action = step(core, 12.02)
    assert report['state'] == 'ARMING' and action == 'CREATE'


def test_new_goal_alone_cannot_rearm_released_gateway():
    core = ready_core()
    navigate(core)
    core.released = True
    core.update_statuses([('ab' * 16, 5), ('cd' * 16, 2)], 10.2)
    core.update_mission(mission(), 10.2)
    core.update_odom(ZERO, 10.2)
    core.update_cmd(ZERO, 10.2)
    assert step(core, 10.21)[2]['state'] == 'RELEASED_STOPPED'


@pytest.mark.parametrize('terminal_status', [4, 5, 6])
def test_no_fire_terminal_goal_rearms_after_safe_clear(terminal_status):
    core = ready_core()
    navigate(core)
    old, new = 'ab' * 16, 'cd' * 16
    core.update_statuses([(old, terminal_status)], 10.2)
    idle = mission('IDLE', 'HOLD_STOP', goal=False)
    idle['perception'] = {'detected_now': False, 'confirmed': False}
    off = actuator_off()
    for now in (10.21, 10.50, 10.75, 11.25):
        core.update_mission(idle, now - 0.01)
        core.update_nav(ZERO, now - 0.01)
        core.update_odom(ZERO, now - 0.01)
        core.update_cmd(ZERO, now - 0.01)
        core.update_arm(True, now - 0.01)
        core.update_actuator(off, now - 0.01)
        _, _, report, action = step(
            core, now, owned=not core.released,
            raw=RAW_SOURCE if not core.released else [])
        assert report['command']['linear_x_m_s'] == 0.0
        if action == 'RELEASE':
            core.publisher_released()
    assert core.released and not core.fire_latched
    assert core.stop_reason == 'NAV_GOAL_TERMINAL_NO_FIRE'
    assert report['next_goal_clear_ready'] is True
    core.update_statuses([(new, 2)], 11.3)
    core.update_mission(mission(), 11.3)
    core.update_nav(MOVING, 11.3)
    core.update_odom(ZERO, 11.3)
    core.update_cmd(ZERO, 11.3)
    core.update_arm(True, 11.3)
    core.update_actuator(off, 11.3)
    assert step(core, 11.31)[2]['cycle_goal_uuid'] == new
    assert step(core, 11.32)[3] == 'CREATE'


@pytest.mark.parametrize('terminal_status', [4, 5, 6])
def test_no_fire_next_goal_survives_gaps_between_off_telemetry(
        terminal_status):
    core = ready_core()
    navigate(core)
    old, new = 'ab' * 16, 'cd' * 16
    core.released = True
    core.stop_reason = 'NAV_GOAL_TERMINAL_NO_FIRE'
    core.update_statuses([(old, terminal_status), (new, 2)], 10.2)
    off = actuator_off()
    # Gateway evaluates at 20 Hz. An OFF telemetry sample arrives every
    # 0.6 s: its age briefly exceeds the 0.35 s freshness threshold.
    for tick in range(45):
        now = round(10.2 + tick * 0.05, 6)
        core.update_mission(mission(), now)
        core.update_nav(MOVING, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_arm(True, now)
        if tick % 12 == 0:
            core.update_actuator(off, now)
        _, _, report, action = step(core, now)
        assert report['command']['linear_x_m_s'] == 0.0
        if report['cycle_goal_uuid'] == new:
            assert action is None
            assert step(core, now + 0.01)[3] == 'CREATE'
            return
    pytest.fail('No-fire goal 2 stayed blocked despite repeated OFF samples')


def test_no_fire_next_goal_waits_for_fresh_off_sample_after_hold():
    core = ready_core()
    navigate(core)
    old, new = 'ab' * 16, 'cd' * 16
    core.released = True
    core.stop_reason = 'NAV_GOAL_TERMINAL_NO_FIRE'
    core.update_statuses([(old, 4), (new, 2)], 10.2)
    core.update_actuator(actuator_off(), 10.2)
    for tick in range(35):
        now = 10.2 + tick * 0.05
        core.update_mission(mission(), now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        report = step(core, now)[2]
        assert report['cycle_goal_uuid'] == old
        assert report['next_goal_clear_ready'] is False
    assert 'ACTUATOR_OFF_TELEMETRY_MISSING_OR_STALE' in \
        report['rearm_blockers']
    core.update_actuator(actuator_off(), 12.0)
    core.update_mission(mission(), 12.0)
    core.update_odom(ZERO, 12.0)
    core.update_cmd(ZERO, 12.0)
    assert step(core, 12.01)[2]['cycle_goal_uuid'] == new


def test_no_fire_next_goal_on_telemetry_breaks_clear_hold():
    core = ready_core()
    navigate(core)
    old, new = 'ab' * 16, 'cd' * 16
    core.released = True
    core.stop_reason = 'NAV_GOAL_TERMINAL_NO_FIRE'
    core.update_statuses([(old, 6), (new, 2)], 10.2)
    for tick in range(49):
        now = round(10.2 + tick * 0.05, 6)
        core.update_mission(mission(), now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        if tick % 12 == 0:
            core.update_actuator(
                actuator_off(pump_pwm_commanded_on=tick == 12), now)
        report = step(core, now)[2]
        if now < 12.4:
            assert report['cycle_goal_uuid'] == old
    assert report['cycle_goal_uuid'] == new


def test_pre_spray_clear_rearms_next_goal_but_never_with_pump_owner():
    core = ready_core()
    navigate(core)
    old, new = 'ab' * 16, 'cd' * 16
    core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 10.15)
    core.update_odom(ZERO, 10.15)
    core.update_cmd(ZERO, 10.15)
    assert step(core, 10.16, owned=True, raw=RAW_SOURCE)[2][
        'state'] == 'STOPPING'
    first = core.first_stop
    assert first['reason'] == 'FIRE_ACQUIRED'
    core.update_odom(ZERO, 10.70)
    core.update_cmd(ZERO, 10.70)
    assert step(core, 10.71, owned=True, raw=RAW_SOURCE)[3] == 'RELEASE'
    core.publisher_released()
    core.update_guard({'state': 'CANCEL_CONFIRMED_STOPPED',
                       'goal_uuid': old, 'blockers': []}, 10.72)
    idle = mission('IDLE', 'HOLD_STOP', goal=False)
    idle['perception'] = {'detected_now': False, 'confirmed': False}
    idle['lifecycle'] = {'pre_spray_clear_goal_uuid': old}
    off = actuator_off()
    for now in (10.8, 11.9):
        core.update_mission(idle, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_actuator(off, now)
        step(core, now + 0.01)
    core.update_statuses([(old, 5), (new, 2)], 12.0)
    core.update_mission(mission(), 12.0)
    core.update_nav(MOVING, 12.0)
    core.update_odom(ZERO, 12.0)
    core.update_cmd(ZERO, 12.0)
    core.update_arm(True, 12.0)
    core.update_actuator(off, 12.0)
    assert step(core, 12.01, guard_count=1)[2][
        'cycle_goal_uuid'] == old
    assert core.first_stop == first
    core.update_guard({'state': 'WAITING_FOR_ADMISSION',
                       'goal_uuid': None, 'send_count': 0}, 12.02)
    assert step(core, 12.03, guard_count=1)[2][
        'cycle_goal_uuid'] == new
    assert core.first_stop is None


def test_missed_idle_proof_can_arrive_after_next_goal_is_active():
    core = ready_core()
    old, new = 'ab' * 16, 'cd' * 16
    core.goal_uuid = old
    core.fire_latched = True
    core.released = True
    core.stop_reason = 'FIRE_ACQUIRED'
    core.cancel_confirmed_uuid = old
    core.clear_since = 10.0
    core.update_guard({'state': 'WAITING_FOR_ADMISSION',
                       'goal_uuid': None, 'send_count': 0}, 11.1)
    core.update_statuses([(old, 5), (new, 2)], 11.1)
    navigating = mission()
    navigating['lifecycle'] = {'pre_spray_clear_goal_uuid': old}
    core.update_mission(navigating, 11.1)
    core.update_odom(ZERO, 11.1)
    core.update_cmd(ZERO, 11.1)
    core.update_actuator(actuator_off(), 11.1)
    assert step(core, 11.11)[2]['cycle_goal_uuid'] == new


@pytest.mark.parametrize('invalid', [
    {'schema_version': 2},
    {'mode': 'SHADOW_ONLY'},
    {'receive_age_ms': 100000},
    {'receive_age_ms': float('nan')},
    {'pump_pwm_commanded_on': True},
    {'servo_pwm_commanded_on': True},
])
def test_bad_actuator_report_cannot_unlock_next_goal(invalid):
    core = ready_core()
    old, new = 'ab' * 16, 'cd' * 16
    core.goal_uuid = old
    core.released = True
    core.stop_reason = 'NAV_GOAL_TERMINAL_NO_FIRE'
    core.normal_terminal_uuid = old
    core.normal_terminal_status = 4
    core.clear_since = 10.0
    core.update_statuses([(old, 4), (new, 2)], 11.1)
    core.update_mission(mission(), 11.1)
    core.update_odom(ZERO, 11.1)
    core.update_cmd(ZERO, 11.1)
    core.update_actuator(actuator_off(**invalid), 11.1)
    report = step(core, 11.11)[2]
    assert report['cycle_goal_uuid'] == old
    assert 'ACTUATOR_OFF_TELEMETRY_MISSING_OR_STALE' in \
        report['rearm_blockers']
    core.update_actuator(actuator_off(), 11.2)
    core.update_mission(mission(), 11.2)
    core.update_odom(ZERO, 11.2)
    core.update_cmd(ZERO, 11.2)
    assert step(core, 11.21)[2]['cycle_goal_uuid'] == old
    for now in (11.8, 12.25):
        core.update_mission(mission(), now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_actuator(actuator_off(), now)
        report = step(core, now + 0.01)[2]
    assert report['cycle_goal_uuid'] == new


@pytest.mark.parametrize('sources', [
    [], ACTUATOR_SOURCE * 2,
    [('/', 'fake_bridge', 'std_msgs/msg/String',
      ReliabilityPolicy.BEST_EFFORT)],
])
def test_missing_duplicate_or_wrong_actuator_publisher_blocks_rearm(sources):
    core = ready_core()
    old, new = 'ab' * 16, 'cd' * 16
    core.goal_uuid = old
    core.released = True
    core.stop_reason = 'NAV_GOAL_TERMINAL_NO_FIRE'
    core.normal_terminal_uuid = old
    core.normal_terminal_status = 4
    core.clear_since = 10.0
    core.update_statuses([(old, 4), (new, 2)], 11.1)
    core.update_mission(mission(), 11.1)
    core.update_odom(ZERO, 11.1)
    core.update_cmd(ZERO, 11.1)
    core.update_actuator(actuator_off(), 11.1)
    report = step(core, 11.11, actuator_sources=sources)[2]
    assert report['cycle_goal_uuid'] == old
    assert 'ACTUATOR_PUBLISHER_INVALID' in report['rearm_blockers']


def test_rearm_blockers_explain_missing_cycle_proof():
    core = ready_core()
    old, new = 'ab' * 16, 'cd' * 16
    core.goal_uuid = old
    core.released = True
    core.stop_reason = 'NAV_INPUT_OR_TOPOLOGY_LOST'
    core.clear_since = 10.0
    core.update_statuses([(new, 2)], 11.1)
    core.update_mission(mission(), 11.1)
    core.update_odom(ZERO, 11.1)
    core.update_cmd(ZERO, 11.1)
    core.update_actuator(actuator_off(), 11.1)
    report = step(core, 11.11)[2]
    assert report['cycle_goal_uuid'] == old
    assert report['rearm_blockers'] == ['OLD_GOAL_TERMINAL_PROOF_INVALID']


def test_rearm_blockers_explain_missing_pre_spray_guard_reset():
    core = ready_core()
    old, new = 'ab' * 16, 'cd' * 16
    core.goal_uuid = old
    core.fire_latched = True
    core.released = True
    core.stop_reason = 'FIRE_ACQUIRED'
    core.cancel_confirmed_uuid = old
    core.pre_spray_clear_uuid = old
    core.clear_since = 10.0
    core.update_statuses([(old, 5), (new, 2)], 11.1)
    core.update_mission(mission(), 11.1)
    core.update_odom(ZERO, 11.1)
    core.update_cmd(ZERO, 11.1)
    core.update_actuator(actuator_off(), 11.1)
    report = step(core, 11.11)[2]
    assert report['cycle_goal_uuid'] == old
    assert 'CANCEL_GUARD_RESET_PROOF_MISSING' in report['rearm_blockers']
    core.update_guard({'state': 'WAITING_FOR_ADMISSION',
                       'goal_uuid': None, 'send_count': 0}, 11.12)
    assert step(core, 11.13)[2]['cycle_goal_uuid'] == new


def test_stale_source_age_plus_delivery_time_blocks_actuator_proof():
    core = ready_core()
    old, new = 'ab' * 16, 'cd' * 16
    core.goal_uuid = old
    core.released = True
    core.stop_reason = 'NAV_GOAL_TERMINAL_NO_FIRE'
    core.normal_terminal_uuid = old
    core.normal_terminal_status = 4
    core.clear_since = 10.0
    core.update_statuses([(old, 4), (new, 2)], 11.1)
    core.update_mission(mission(), 11.1)
    core.update_odom(ZERO, 11.1)
    core.update_cmd(ZERO, 11.1)
    core.update_actuator(actuator_off(receive_age_ms=400.0), 11.1)
    report = step(core, 11.25)[2]
    assert report['cycle_goal_uuid'] == old
    assert 'ACTUATOR_OFF_TELEMETRY_MISSING_OR_STALE' in \
        report['rearm_blockers']


def test_no_fire_next_goal_waits_for_pump_publisher_release():
    core = ready_core()
    core.goal_uuid = 'ab' * 16
    core.released = True
    core.stop_reason = 'NAV_GOAL_TERMINAL_NO_FIRE'
    core.normal_terminal_uuid = core.goal_uuid
    core.normal_terminal_status = 4
    core.clear_since = 10.0
    core.update_statuses([('cd' * 16, 2)], 11.1)
    core.update_mission(mission(), 11.1)
    core.update_actuator(actuator_off(), 11.1)
    core.update_odom(ZERO, 11.1)
    core.update_cmd(ZERO, 11.1)
    assert core.evaluate(11.11, False, [], NAV_SOURCES, SAFETY_SOURCE,
                         1, 1, 1, 1, 1, 1)[2]['cycle_goal_uuid'] == 'ab' * 16


def test_pre_spray_clear_refuses_cycle_with_past_pump_owner():
    core = ready_core()
    old, new = 'ab' * 16, 'cd' * 16
    core.goal_uuid = old
    core.fire_latched = True
    core.evaluate(10.01, False, [], NAV_SOURCES, SAFETY_SOURCE,
                  1, 1, 1, 1, 1, 1)
    assert core.pump_started_uuid == old
    core.released = True
    core.stop_reason = 'FIRE_ACQUIRED'
    core.cancel_confirmed_uuid = old
    core.pre_spray_clear_uuid = old
    core.clear_since = 10.0
    core.update_guard({'state': 'WAITING_FOR_ADMISSION',
                       'goal_uuid': None, 'send_count': 0}, 11.1)
    core.update_statuses([(old, 5), (new, 2)], 11.1)
    core.update_mission(mission(), 11.1)
    core.update_odom(ZERO, 11.1)
    core.update_cmd(ZERO, 11.1)
    core.update_actuator(actuator_off(), 11.1)
    assert step(core, 11.11)[2]['cycle_goal_uuid'] == old


def test_cancel_before_first_forward_also_releases_goal_for_next_uuid():
    core = ready_core()
    old, new = 'ab' * 16, 'cd' * 16
    # Nav2 was seen, but the gateway had not created its raw publisher yet.
    assert step(core, 10.01)[2]['cycle_goal_uuid'] == old
    assert not core.ever_forwarded
    core.update_statuses([(old, 5)], 10.2)
    idle = mission('IDLE', 'HOLD_STOP', goal=False)
    off = actuator_off()
    for now in (10.21, 10.75, 11.25):
        core.update_mission(idle, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_actuator(off, now)
        report = step(core, now + 0.01)[2]
        assert report['command']['linear_x_m_s'] == 0.0
    assert core.released
    assert core.stop_reason == 'NAV_GOAL_TERMINAL_NO_FIRE'
    core.update_statuses([(new, 2)], 11.3)
    core.update_mission(mission(), 11.3)
    core.update_odom(ZERO, 11.3)
    core.update_cmd(ZERO, 11.3)
    core.update_actuator(off, 11.3)
    assert step(core, 11.31)[2]['cycle_goal_uuid'] == new


def test_next_goal_active_in_same_status_snapshot_as_old_terminal():
    core = ready_core()
    navigate(core)
    old, new = 'ab' * 16, 'cd' * 16
    core.update_statuses([(old, 4), (new, 2)], 10.2)
    core.update_mission(mission(), 10.2)
    off = actuator_off()
    core.update_actuator(off, 10.2)
    core.update_odom(ZERO, 10.2)
    core.update_cmd(ZERO, 10.2)
    assert step(core, 10.21, owned=True, raw=RAW_SOURCE)[2][
        'stop_reason'] == 'NAV_GOAL_TERMINAL_NO_FIRE'
    for now in (10.75, 11.30):
        core.update_mission(mission(), now)
        core.update_nav(MOVING, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_actuator(off, now)
        _, _, _, action = step(
            core, now + 0.01, owned=not core.released,
            raw=RAW_SOURCE if not core.released else [])
        if action == 'RELEASE':
            core.publisher_released()
    assert core.goal_uuid == new
    assert core.stop_reason == ''


def test_terminal_status_delayed_after_mission_nav_input_change():
    core = ready_core()
    navigate(core)
    old = 'ab' * 16
    core.update_mission(mission('IDLE', 'HOLD_STOP', goal=False), 10.2)
    core.update_odom(ZERO, 10.2)
    core.update_cmd(ZERO, 10.2)
    assert step(core, 10.21, owned=True, raw=RAW_SOURCE)[2][
        'stop_reason'] == 'NAV_INPUT_OR_TOPOLOGY_LOST'
    core.update_statuses([(old, 5)], 10.3)
    core.update_mission(mission('IDLE', 'HOLD_STOP', goal=False), 10.3)
    assert step(core, 10.31, owned=True, raw=RAW_SOURCE)[2][
        'stop_reason'] == 'NAV_GOAL_TERMINAL_NO_FIRE'


def test_quick_new_goal_needs_continuous_stopped_clear_window():
    core = ready_core()
    navigate(core)
    old, new = 'ab' * 16, 'cd' * 16
    core.released = True
    core.stop_reason = 'NAV_GOAL_TERMINAL_NO_FIRE'
    core.normal_terminal_uuid = old
    core.normal_terminal_status = 4
    off = actuator_off()
    core.update_statuses([(old, 4)], 10.20)
    idle = mission('IDLE', 'HOLD_STOP', goal=False)
    core.update_mission(idle, 10.20)
    core.update_actuator(off, 10.20)
    core.update_odom(ZERO, 10.20)
    core.update_cmd(ZERO, 10.20)
    step(core, 10.21)
    core.update_statuses([(old, 4), (new, 2)], 10.40)
    for now, odom_values in ((10.4, ZERO), (10.6, MOVING),
                             (10.8, ZERO), (11.85, ZERO)):
        core.update_mission(mission(), now)
        core.update_nav(MOVING, now)
        core.update_arm(True, now)
        core.update_actuator(off, now)
        core.update_odom(odom_values, now)
        core.update_cmd(ZERO, now)
        report = step(core, now + 0.01)[2]
        if now < 11.85:
            assert report['cycle_goal_uuid'] == old
        else:
            assert report['cycle_goal_uuid'] == new


def test_fire_seen_only_after_goal_terminal_does_not_poison_old_cycle():
    core = ready_core()
    navigate(core)
    old = 'ab' * 16
    core.update_statuses([(old, 4)], 10.20)
    late_fire = mission('FIRE_TRACKING', 'LEFT_PREVIEW', goal=False)
    late_fire['perception'] = {'detected_now': True, 'confirmed': True}
    core.update_mission(late_fire, 10.20)
    core.update_nav(ZERO, 10.20)
    core.update_odom(ZERO, 10.20)
    core.update_cmd(ZERO, 10.20)
    _, _, report, _ = step(core, 10.21, owned=True, raw=RAW_SOURCE)
    assert report['fire_latched'] is False
    assert report['stop_reason'] == 'NAV_GOAL_TERMINAL_NO_FIRE'


def test_first_cycle_waits_for_exact_nav_goal_uuid_before_forwarding():
    core = ready_core()
    core.statuses = None
    core.status_at = None
    _, _, report, action = step(core, 10.05)
    assert report['state'] == 'WAITING_FOR_INPUT' and action is None
    assert 'NAV_GOAL_UUID_UNAVAILABLE' in report['blockers']
    core.update_statuses([('ab' * 16, 2)], 10.1)
    assert step(core, 10.11)[3] == 'CREATE'


def test_event_driven_status_does_not_expire_during_fresh_navigation():
    """Nav2 status is not a 0.35 s heartbeat; motion inputs still are."""
    core = ready_core()
    navigate(core)
    core.update_mission(mission(), 12.0)
    core.update_nav(MOVING, 12.0)
    core.update_odom(ZERO, 12.0)
    core.update_arm(True, 12.0)
    linear, angular, state, action = step(
        core, 12.01, owned=True, raw=RAW_SOURCE)
    assert state['state'] == 'NAVIGATION'
    assert 'NAV_GOAL_UUID_UNAVAILABLE' not in state['blockers']
    assert (linear, angular, action) == (0.05, 0.2, None)
    core.update_statuses([('ab' * 16, 5)], 12.02)
    _, _, state, _ = step(core, 12.03, owned=True, raw=RAW_SOURCE)
    assert state['state'] == 'STOPPING'
    assert 'NAV_GOAL_UUID_UNAVAILABLE' in state['blockers']


def test_retained_status_still_requires_status_publisher():
    """A missing action server must never retain navigation authority."""
    core = ready_core()
    navigate(core)
    core.update_mission(mission(), 12.0)
    core.update_nav(MOVING, 12.0)
    core.update_odom(ZERO, 12.0)
    core.update_arm(True, 12.0)
    _, _, state, _ = step(core, 12.01, owned=True,
                          raw=RAW_SOURCE, status=0)
    assert state['state'] == 'STOPPING'
    assert 'NAV_STATUS_PUBLISHER_INVALID' in state['blockers']


def test_multiple_active_statuses_revoke_exact_goal_ownership():
    """Retaining action status must not admit ambiguous active goals."""
    core = ready_core()
    navigate(core)
    core.update_mission(mission(), 12.0)
    core.update_nav(MOVING, 12.0)
    core.update_odom(ZERO, 12.0)
    core.update_arm(True, 12.0)
    core.update_statuses([('ab' * 16, 2), ('cd' * 16, 2)], 12.0)
    linear, angular, state, _ = step(
        core, 12.01, owned=True, raw=RAW_SOURCE)
    assert state['state'] == 'STOPPING'
    assert (linear, angular) == (0.0, 0.0)
    assert 'NAV_GOAL_UUID_UNAVAILABLE' in state['blockers']


def test_fire_before_any_goal_does_not_release_gateway():
    core = ready_core()
    core.statuses = None
    core.status_at = None
    core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 10.0)
    _, _, report, action = step(core, 10.05)
    assert report['state'] == 'WAITING_FOR_INPUT'
    assert report['fire_latched'] is False
    assert report['ever_forwarded'] is False
    assert report['cycle_goal_uuid'] is None
    assert action is None


def test_fire_with_exact_goal_still_stops_even_before_motion_forwarded():
    core = ready_core()
    core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 10.0)
    _, _, report, _ = step(core, 10.05)
    assert report['fire_latched'] is True
    assert report['stop_reason'] == 'FIRE_ACQUIRED'
    assert report['cycle_goal_uuid'] == 'ab' * 16


def step(core, now, owned=False, raw=None, nav=None, safety=None,
         status=1, mission_count=1, odom_count=1, arm_count=1,
         guard_count=1, actuator_sources=ACTUATOR_SOURCE):
    return core.evaluate(
        now, owned, [] if raw is None else raw,
        NAV_SOURCES if nav is None else nav,
        SAFETY_SOURCE if safety is None else safety, status,
        mission_count, odom_count, arm_count, guard_count,
        actuator_sources=actuator_sources)


def navigate(core):
    _, _, state, action = step(core, 10.05)
    assert state['state'] == 'ARMING'
    assert action == 'CREATE'
    core.publisher_created(10.05)
    linear, angular, state, action = step(
        core, 10.10, owned=True, raw=RAW_SOURCE)
    assert state['state'] == 'NAVIGATION'
    assert action is None
    assert linear == 0.05 and angular == 0.2


def test_disabled_never_acquires_or_forwards():
    core = ready_core(enabled=False)
    linear, angular, state, action = step(core, 10.1)
    assert (linear, angular) == (0.0, 0.0)
    assert state['state'] == 'DISABLED'
    assert action is None


def test_arm_and_exclusive_raw_owner_required():
    core = ready_core()
    core.update_arm(False, 10.0)
    _, _, state, action = step(core, 10.1)
    assert state['state'] == 'WAITING_FOR_INPUT'
    assert action is None
    core.update_arm(True, 10.1)
    _, _, state, action = step(core, 10.2, raw=[RAW_SOURCE[0]])
    assert state['state'] == 'WAITING_FOR_RAW_OWNER'
    assert action is None


def test_nav_topology_and_safety_gate_required():
    core = ready_core()
    _, _, state, action = step(core, 10.1, nav=NAV_SOURCES[:2])
    assert 'NAV_SHADOW_TOPOLOGY_INVALID' in state['blockers']
    assert action is None
    _, _, state, action = step(core, 10.1, safety=[])
    assert 'SAFETY_GATE_TOPOLOGY_INVALID' in state['blockers']
    assert action is None
    _, _, state, action = step(core, 10.1, status=2)
    assert 'NAV_STATUS_PUBLISHER_INVALID' in state['blockers']
    assert action is None


def test_multiple_or_missing_input_publishers_block():
    core = ready_core()
    for override in (
            {'mission_count': 0}, {'mission_count': 2},
            {'odom_count': 0}, {'odom_count': 2},
            {'arm_count': 0}, {'arm_count': 2}):
        _, _, state, action = step(core, 10.1, **override)
        assert state['state'] == 'WAITING_FOR_INPUT'
        assert action is None


def test_fire_stops_then_releases_only_after_stationary_hold():
    core = ready_core()
    navigate(core)
    core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 10.15)
    core.update_odom(MOVING, 10.15)
    linear, angular, state, action = step(
        core, 10.16, owned=True, raw=RAW_SOURCE)
    assert state['state'] == 'STOPPING'
    assert (linear, angular) == (0.0, 0.0)
    assert action is None
    core.update_odom(ZERO, 10.20)
    core.update_cmd(ZERO, 10.20)
    _, _, state, action = step(core, 10.21, owned=True, raw=RAW_SOURCE)
    assert state['state'] == 'STOPPING'
    assert action is None
    core.update_odom(ZERO, 10.72)
    core.update_cmd(ZERO, 10.72)
    _, _, state, action = step(core, 10.73, owned=True, raw=RAW_SOURCE)
    assert state['state'] == 'RELEASING'
    assert action == 'RELEASE'
    core.publisher_released()
    core.update_mission(mission(), 10.74)
    core.update_nav(MOVING, 10.74)
    core.update_odom(ZERO, 10.74)
    core.update_arm(True, 10.74)
    _, _, state, action = step(core, 10.75)
    assert state['state'] == 'RELEASED_STOPPED'
    # One-shot: never resume merely because fire vanished.
    assert action is None


def test_input_loss_during_motion_stops_without_auto_resume():
    core = ready_core()
    navigate(core)
    core.update_odom(MOVING, 10.40)
    linear, angular, state, action = step(
        core, 10.41, owned=True, raw=RAW_SOURCE)
    assert (linear, angular) == (0.0, 0.0)
    assert state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'NAV_INPUT_OR_TOPOLOGY_LOST'
    assert action is None


def test_nav_turn_at_eight_tenths_is_forwarded():
    core = ready_core()
    core.update_nav((0.0, 0.0, 0.0, 0.0, 0.0, 0.8), 10.0)
    _, _, state, action = step(core, 10.05)
    assert state['state'] == 'ARMING'
    assert action == 'CREATE'
    core.publisher_created(10.05)
    linear, angular, state, action = step(
        core, 10.10, owned=True, raw=RAW_SOURCE)
    assert (linear, angular) == (0.0, 0.8)
    assert state['state'] == 'NAVIGATION'
    assert action is None


def test_nav_turn_above_eight_tenths_latches_stop():
    core = ready_core()
    navigate(core)
    core.update_nav((0.0, 0.0, 0.0, 0.0, 0.0, 0.81), 10.15)
    linear, angular, state, action = step(
        core, 10.16, owned=True, raw=RAW_SOURCE)
    assert (linear, angular) == (0.0, 0.0)
    assert state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'NAV_INPUT_OR_TOPOLOGY_LOST'
    assert 'NAV_COMMAND_OUT_OF_BOUNDS' in state['blockers']
    assert action is None


def test_gateway_cannot_be_configured_above_approved_limit():
    with pytest.raises(ValueError, match='0.8 rad/s'):
        LiveNavStopCore(max_nav_angular_rad_s=0.81)


def test_candidate_does_not_stop_until_fire_is_acquired():
    core = ready_core()
    navigate(core)
    core.update_mission(mission('FIRE_CANDIDATE', 'KEEP_NAV2'), 10.15)
    core.update_nav(MOVING, 10.15)
    core.update_odom(MOVING, 10.15)
    core.update_arm(True, 10.15)
    linear, angular, state, _ = step(
        core, 10.16, owned=True, raw=RAW_SOURCE)
    assert state['state'] == 'NAVIGATION'
    assert (linear, angular) == (0.05, 0.2)
    core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 10.17)
    linear, angular, state, _ = step(
        core, 10.18, owned=True, raw=RAW_SOURCE)
    assert state['state'] == 'STOPPING'
    assert (linear, angular) == (0.0, 0.0)


def test_operator_arm_loss_during_motion_latches_stop():
    core = ready_core()
    navigate(core)
    core.update_mission(mission(), 10.15)
    core.update_nav(MOVING, 10.15)
    core.update_odom(MOVING, 10.15)
    core.update_arm(False, 10.15)
    linear, angular, state, _ = step(
        core, 10.16, owned=True, raw=RAW_SOURCE)
    assert state['state'] == 'STOPPING'
    assert (linear, angular) == (0.0, 0.0)
    assert 'LIVE_ARM_MISSING_OR_STALE' in state['blockers']


def test_raw_release_needs_fresh_safety_gate_zero():
    core = ready_core()
    navigate(core)
    core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 10.15)
    core.update_odom(ZERO, 10.15)
    core.update_cmd((0.05, 0, 0, 0, 0, 0.2), 10.15)
    step(core, 10.16, owned=True, raw=RAW_SOURCE)
    core.update_odom(ZERO, 10.80)
    core.update_cmd((0.05, 0, 0, 0, 0, 0.2), 10.80)
    _, _, state, action = step(core, 10.81, owned=True, raw=RAW_SOURCE)
    assert state['state'] == 'STOPPING'
    assert action is None
    core.update_cmd(ZERO, 10.82)
    step(core, 10.83, owned=True, raw=RAW_SOURCE)
    core.update_odom(ZERO, 11.34)
    core.update_cmd(ZERO, 11.34)
    _, _, state, action = step(core, 11.35, owned=True, raw=RAW_SOURCE)
    assert state['state'] == 'RELEASING'
    assert action == 'RELEASE'


def test_unknown_raw_publisher_fail_closes():
    core = ready_core()
    navigate(core)
    rogue = ('/', 'rogue', 'geometry_msgs/msg/Twist',
             ReliabilityPolicy.BEST_EFFORT)
    linear, angular, state, _ = step(
        core, 10.15, owned=True, raw=RAW_SOURCE + [rogue])
    assert (linear, angular) == (0.0, 0.0)
    assert state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'RAW_OWNERSHIP_LOST'


def test_handoff_ready_requires_verified_cancel_and_mission():
    core = ready_core()
    navigate(core)
    core.update_mission(mission('FIRE_TRACKING', 'LEFT_PREVIEW'), 10.15)
    core.update_odom(ZERO, 10.15)
    core.update_cmd(ZERO, 10.15)
    step(core, 10.16, owned=True, raw=RAW_SOURCE)
    core.update_odom(ZERO, 10.70)
    core.update_cmd(ZERO, 10.70)
    _, _, _, action = step(core, 10.71, owned=True, raw=RAW_SOURCE)
    assert action == 'RELEASE'
    core.publisher_released()
    core.update_mission(
        mission('ALIGNMENT_READY', 'HOLD_CENTER_PREVIEW',
                goal=False, eligible=True), 10.72)
    core.update_guard({
        'schema_version': 1,
        'state': 'CANCEL_CONFIRMED_STOPPED', 'enabled': True,
        'send_count': 1, 'goal_uuid': 'ab' * 16,
        'cancel_service_accepted': True, 'cancel_status_seen': True,
        'goal_status': 5, 'arm_fresh': True,
        'cmd_vel_fresh_zero': True, 'odom_fresh_stationary': True,
        'nav_shadow_fingerprint_ok': True, 'raw_cmd_publishers': 0,
        'blockers': [],
    }, 10.72)
    core.update_odom(ZERO, 10.72)
    _, _, state, action = step(core, 10.73)
    assert state['state'] == 'HANDOFF_READY'
    assert state['handoff_ready'] is True
    assert state['command'] == {
        'linear_x_m_s': 0.0, 'angular_z_rad_s': 0.0}
    assert action is None
    core.guard['goal_uuid'] = '0' * 32
    _, _, state, _ = step(core, 10.74)
    assert state['state'] == 'RELEASED_STOPPED'
    core.guard['goal_uuid'] = 'ab' * 16
    _, _, state, _ = step(core, 10.74, guard_count=0)
    assert state['state'] == 'RELEASED_STOPPED'


# Recovery diagnostics: isolate the FIRST failed input, then restore everything.
# Command/ownership expectations intentionally remain the pre-existing policy.
DIAGNOSTIC_LOSSES = [
    ('nav', 'NAV_COMMAND_MISSING_OR_INVALID'),
    ('mission', 'MISSION_MISSING_OR_STALE'),
    ('health', 'MISSION_HEALTH_BLOCKED'),
    ('odom', 'ODOM_NOT_FRESH'),
    ('arm', 'LIVE_ARM_MISSING_OR_STALE'),
    ('nav_graph', 'NAV_SHADOW_TOPOLOGY_INVALID'),
    ('safety_graph', 'SAFETY_GATE_TOPOLOGY_INVALID'),
    ('status_graph', 'NAV_STATUS_PUBLISHER_INVALID'),
    ('mission_graph', 'MISSION_PUBLISHER_INVALID'),
    ('odom_graph', 'ODOM_PUBLISHER_INVALID'),
    ('arm_graph', 'LIVE_ARM_PUBLISHER_INVALID'),
]


def refresh_diagnostic_inputs(core, now, report=None):
    core.update_mission(mission() if report is None else report, now)
    core.update_nav(MOVING, now)
    core.update_odom(ZERO, now)
    core.update_cmd(ZERO, now)
    core.update_arm(True, now)
    core.update_actuator(actuator_off(), now)


@pytest.mark.parametrize('loss, expected', DIAGNOSTIC_LOSSES)
def test_first_blocker_survives_recovery_without_rearming(loss, expected):
    from copy import deepcopy
    core = ready_core()
    navigate(core)
    assert core.first_stop is None
    refresh_diagnostic_inputs(core, 10.20)
    overrides = {}
    if loss in ('nav', 'mission', 'odom', 'arm'):
        setattr(core, loss + '_at', 9.0)
    elif loss == 'health':
        core.mission['health'] = {'status': 'BLOCKED', 'blockers': ['DETECTION_STALE']}
    else:
        overrides = {
            'nav_graph': {'nav': []}, 'safety_graph': {'safety': []},
            'status_graph': {'status': 0}, 'mission_graph': {'mission_count': 2},
            'odom_graph': {'odom_count': 2}, 'arm_graph': {'arm_count': 2},
        }[loss]
    linear, angular, report, action = step(
        core, 10.21, owned=True, raw=RAW_SOURCE, **overrides)
    assert (linear, angular, action) == (0., 0., None)
    assert report['state'] == 'STOPPING'
    assert report['stop_reason'] == 'NAV_INPUT_OR_TOPOLOGY_LOST'
    assert report['blockers'] == [expected]
    first = deepcopy(report['first_stop'])
    assert first['blockers'] == [expected]
    assert first['cycle_goal_uuid'] == 'ab' * 16
    assert first['monotonic_s'] == 10.21
    if loss in ('nav', 'mission', 'odom', 'arm'):
        key = {'nav': 'nav_command', 'odom': 'odom_admitted'}.get(loss, loss)
        assert first['receive_age_ms'][key] > first['timeout_ms'][key]
    if loss == 'health':
        assert first['mission_health']['blockers'] == ['DETECTION_STALE']
        core.mission['health']['blockers'].clear()
    # Consumers must not mutate the retained snapshot through the payload.
    report['first_stop']['blockers'].clear()
    for now in (10.30, 10.90):
        refresh_diagnostic_inputs(core, now)
        linear, angular, report, action = step(core, now, owned=True, raw=RAW_SOURCE)
        assert (linear, angular) == (0., 0.)
        assert report['blockers'] == []  # Data is back; original failure remains.
        assert report['first_stop'] == first
    assert action == 'RELEASE'
    core.publisher_released()
    refresh_diagnostic_inputs(core, 10.95)
    linear, angular, report, action = step(core, 10.96)
    assert (linear, angular, action) == (0., 0., None)
    assert report['state'] == 'RELEASED_STOPPED'
    assert report['first_stop'] == first
    # A new goal alone does not erase evidence or authorize recovery.
    core.update_statuses([('cd' * 16, 2)], 10.97)
    report = step(core, 10.98)[2]
    assert report['cycle_goal_uuid'] == 'ab' * 16
    assert report['first_stop'] == first
    assert 'OLD_GOAL_TERMINAL_PROOF_INVALID' in report['rearm_blockers']


def test_raw_ownership_loss_records_first_topology_even_after_graph_recovers():
    core = ready_core()
    navigate(core)
    refresh_diagnostic_inputs(core, 10.20)
    report = step(core, 10.21, owned=True, raw=RAW_SOURCE * 2)[2]
    assert report['stop_reason'] == 'RAW_OWNERSHIP_LOST'
    assert report['first_stop']['reason'] == 'RAW_OWNERSHIP_LOST'
    assert report['first_stop']['topology']['raw_owner_ok'] is False
    refresh_diagnostic_inputs(core, 10.30)
    report = step(core, 10.31, owned=True, raw=RAW_SOURCE)[2]
    assert report['raw_owner_fingerprint_ok'] is True
    assert report['first_stop']['topology']['raw_owner_ok'] is False


def test_reclassified_terminal_keeps_first_stop_evidence():
    core = ready_core()
    navigate(core)
    refresh_diagnostic_inputs(core, 10.20, mission('IDLE', 'HOLD_STOP', goal=False))
    first = step(core, 10.21, owned=True, raw=RAW_SOURCE)[2]['first_stop']
    assert first['reason'] == 'NAV_INPUT_OR_TOPOLOGY_LOST'
    core.update_statuses([('ab' * 16, 4)], 10.22)
    report = step(core, 10.23, owned=True, raw=RAW_SOURCE)[2]
    assert report['stop_reason'] == 'NAV_GOAL_TERMINAL_NO_FIRE'
    assert report['first_stop'] == first


def test_first_stop_resets_only_on_admitted_next_cycle_three_goals():
    core = ready_core()
    navigate(core)
    old_first = None
    for index, (old, new) in enumerate(zip(['ab'*16, 'cd'*16, 'ef'*16],
                                         ['cd'*16, 'ef'*16, '12'*16])):
        t = 10.20 + 2 * index
        core.update_statuses([(old, 4)], t)
        idle = mission('IDLE', 'HOLD_STOP', goal=False)
        for offset in (.01, .30, .60, 1.10):
            now = t + offset
            refresh_diagnostic_inputs(core, now, idle)
            _, _, report, action = step(core, now, owned=not core.released,
                                       raw=RAW_SOURCE if not core.released else [])
            if action == 'RELEASE': core.publisher_released()
            assert report['first_stop']['cycle_goal_uuid'] == old
        assert report['next_goal_clear_ready'] is True
        assert report['first_stop'] != old_first
        old_first = report['first_stop']
        core.update_statuses([(new, 2)], t+1.20)
        refresh_diagnostic_inputs(core, t+1.20)
        report = step(core, t+1.21)[2]
        assert report['cycle_goal_uuid'] == new
        assert report['first_stop'] is None
        assert step(core, t+1.22)[3] == 'CREATE'
        core.publisher_created(t+1.22)
        assert step(core, t+1.23, owned=True, raw=RAW_SOURCE)[2]['state'] == 'NAVIGATION'


def test_first_stop_external_health_text_is_bounded_and_json_serializable():
    import json
    core = ready_core()
    navigate(core)
    core.mission['health'] = {'status': 'BLOCKED', 'blockers': ['x'*1000]*100}
    snap = step(core, 10.20, owned=True, raw=RAW_SOURCE)[2]['first_stop']
    assert len(snap['mission_health']['blockers']) == 32
    assert all(len(s) == 128 for s in snap['mission_health']['blockers'])
    json.dumps(snap, allow_nan=False)
