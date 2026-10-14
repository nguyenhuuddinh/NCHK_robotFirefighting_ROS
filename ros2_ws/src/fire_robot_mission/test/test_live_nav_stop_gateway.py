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


def mission(state='NAV2_NAVIGATING', intent='KEEP_NAV2', goal=True,
            eligible=False):
    return {
        'schema_version': 1,
        'observe_only': True,
        'state': state,
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
    core.update_actuator({'pump_pwm_commanded_on': False,
                          'servo_phase': 'IDLE',
                          'servo_pwm_commanded_on': False}, 10.72)
    idle = mission('IDLE', 'HOLD_STOP', goal=False)
    idle['perception'] = {'detected_now': False, 'confirmed': False}
    for now in (10.8, 11.9):
        core.update_mission(idle, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_actuator({'pump_pwm_commanded_on': False,
                              'servo_phase': 'IDLE',
                              'servo_pwm_commanded_on': False}, now)
        assert step(core, now + 0.01)[2]['state'] == 'RELEASED_STOPPED'
    core.update_statuses([(old, 5), (new, 2)], 12.0)
    core.update_mission(mission(), 12.0)
    core.update_nav(MOVING, 12.0)
    core.update_odom(ZERO, 12.0)
    core.update_cmd(ZERO, 12.0)
    core.update_arm(True, 12.0)
    core.update_actuator({'pump_pwm_commanded_on': False,
                          'servo_phase': 'IDLE',
                          'servo_pwm_commanded_on': False}, 12.0)
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
         guard_count=1):
    return core.evaluate(
        now, owned, [] if raw is None else raw,
        NAV_SOURCES if nav is None else nav,
        SAFETY_SOURCE if safety is None else safety, status,
        mission_count, odom_count, arm_count, guard_count)


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
