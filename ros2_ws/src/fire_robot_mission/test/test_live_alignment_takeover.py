"""No-hardware regression tests for one-shot live alignment ownership."""

from rclpy.qos import ReliabilityPolicy

from fire_robot_mission.live_alignment_takeover import LiveAlignmentCore
from fire_robot_mission.mission_supervisor import MissionSupervisorCore


UUID = 'a' * 32
ZERO = (0.0,) * 6
MOVING = (0.0, 0.0, 0.0, 0.0, 0.0, 0.08)
RAW = [('/', 'live_alignment_takeover', 'geometry_msgs/msg/Twist',
        ReliabilityPolicy.BEST_EFFORT)]
FOREIGN = [('/', 'teleop_twist_keyboard', 'geometry_msgs/msg/Twist',
            ReliabilityPolicy.BEST_EFFORT)]
NAV = [
    ('/', 'velocity_smoother', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.RELIABLE),
    ('/', 'behavior_server', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.RELIABLE),
    ('/', 'behavior_server', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.RELIABLE),
]
SAFETY = [('/', 'safety_watchdog', 'geometry_msgs/msg/Twist',
           ReliabilityPolicy.BEST_EFFORT)]
COUNTS = dict.fromkeys((
    'mission', 'preview', 'handoff', 'gateway', 'guard', 'odom', 'cmd',
    'scan', 'status', 'arm'), 1)


def mission():
    return {
        'schema_version': 1, 'observe_only': True,
        'state': 'FIRE_TRACKING',
        'perception': {'detected_now': True, 'confirmed': True},
        'navigation': {'goal_active': False},
        'robot': {'odom_fresh': True},
        'health': {'status': 'OK', 'blockers': []},
        'decision': {'command_published': False},
        'safety': {'motion_command_publisher': False,
                   'actuator_command_publisher': False,
                   'collision_check_bypassed': False},
    }


def preview(state='ALIGNING_LEFT', angular=0.35, error=None):
    return {
        'observe_only': True, 'state': state,
        'preview': {'target_angular_z_rad_s': angular,
                    'error_x_normalized': error},
        'safety': {'motion_command_publisher': False,
                   'actuator_command_publisher': False},
    }


def handoff():
    return {
        'shadow_only': True, 'active': True,
        'admission': {'eligible': True, 'blockers': []},
        'live_cancel_proof': {
            'required': True, 'goal_uuid': UUID,
            'gateway_state': 'HANDOFF_READY',
            'cancel_guard_state': 'CANCEL_CONFIRMED_STOPPED',
        },
    }


def gateway():
    return {
        'state': 'HANDOFF_READY', 'handoff_ready': True,
        'released': True, 'stop_reason': 'FIRE_ACQUIRED',
        'raw_publishers': 0,
        'safety': {'raw_command_publisher': False},
    }


def guard():
    return {
        'state': 'CANCEL_CONFIRMED_STOPPED', 'goal_uuid': UUID,
        'send_count': 1, 'cancel_service_accepted': True,
        'cancel_status_seen': True, 'goal_status': 5,
        'arm_fresh': True, 'raw_cmd_publishers': 0,
        'blockers': [],
    }


def ready(enabled=True):
    core = LiveAlignmentCore(enabled=enabled)
    core.update('mission', mission(), 10.0)
    core.update('preview', preview(), 10.0)
    core.update('handoff', handoff(), 10.0)
    core.update('gateway', gateway(), 10.0)
    core.update('guard', guard(), 10.0)
    core.update_arm(True, 10.0)
    core.update_odom(ZERO, 10.0)
    core.update_cmd(ZERO, 10.0)
    core.update_scan(-3.14, 3.14, [1.0] * 360, 10.0)
    core.update_statuses([(UUID, 5)])
    return core


def step(core, now, owned=False, raw=None, nav=None, safety=None,
         counts=None):
    return core.step(
        now, owned, [] if raw is None else raw,
        NAV if nav is None else nav,
        SAFETY if safety is None else safety,
        COUNTS if counts is None else counts)


def acquired():
    core = ready()
    angular, report, action = step(core, 10.01)
    assert angular == 0.0
    assert report['state'] == 'ACQUIRING'
    assert action == 'CREATE'
    return core


def turning():
    core = acquired()
    angular, report, action = step(core, 10.06, owned=True, raw=RAW)
    assert action is None
    assert report['state'] == 'TURNING'
    assert 0.0 < angular <= 0.60
    return core


def test_disabled_never_acquires_or_commands():
    angular, report, action = step(ready(enabled=False), 10.01)
    assert (angular, action) == (0.0, None)
    assert report['state'] == 'DISABLED'
    assert report['blockers'] == ['LIVE_ALIGNMENT_DISABLED']


def test_preflight_needs_unique_arm_and_exact_cancel_proof():
    core = ready()
    core.update_arm(False, 10.0)
    assert step(core, 10.01)[2] is None
    core.update_arm(True, 10.02)
    bad_counts = dict(COUNTS, arm=2)
    assert step(core, 10.03, counts=bad_counts)[2] is None
    core.update('guard', dict(guard(), goal_uuid='b' * 32), 10.04)
    assert step(core, 10.05)[1]['blockers'] == [
        'EXACT_CANCEL_HANDOFF_PROOF_INVALID']
    core.update('guard', guard(), 10.06)
    core.update_statuses([(UUID, 2)])
    assert step(core, 10.07)[2] is None
    core.update_statuses([(UUID, 5)])
    assert step(core, 10.08)[2] == 'CREATE'


def test_preflight_requires_zero_raw_and_stationary_robot():
    core = ready()
    assert step(core, 10.01, raw=FOREIGN)[2] is None
    core.update_odom(MOVING, 10.02)
    assert step(core, 10.03)[2] is None
    core.update_odom(ZERO, 10.04)
    assert step(core, 10.05)[2] == 'CREATE'


def test_scan_must_cover_rotation_and_be_clear_before_and_during_turn():
    core = ready()
    core.update_scan(-1.0, 1.0, [1.0] * 360, 10.0)
    assert step(core, 10.01)[1]['blockers'] == ['SCAN_MISSING_OR_UNSAFE']
    core.update_scan(-3.14, 3.14, [0.2] + [1.0] * 359, 10.02)
    assert step(core, 10.03)[2] is None
    core.update_scan(-3.14, 3.14, [float('inf')] * 360, 10.03)
    assert step(core, 10.035)[2] is None
    core.update_scan(-3.14, 3.14, [9.0] * 360, 10.035,
                     range_max=8.0)
    assert step(core, 10.038)[2] is None
    core.update_scan(-3.14, 3.14, [1.0] * 360, 10.04)
    assert step(core, 10.05)[2] == 'CREATE'
    reverse = ready()
    reverse.update_scan(0.0, -6.26, [1.0] * 400, 10.0)
    assert step(reverse, 10.01)[2] == 'CREATE'
    core = turning()
    core.update_scan(-3.14, 3.14, [0.2] + [1.0] * 359, 10.10)
    assert step(core, 10.11, owned=True, raw=RAW)[1][
        'stop_reason'] == 'SCAN_MISSING_OR_UNSAFE'


def test_only_own_publisher_may_turn_and_speed_is_bounded():
    core = acquired()
    assert step(core, 10.05, owned=True, raw=[])[0] == 0.0
    angular, report, action = step(core, 10.10, owned=True, raw=RAW)
    assert action is None
    assert report['state'] == 'TURNING'
    assert report['command']['linear_x_m_s'] == 0.0
    assert 0.0 < angular <= 0.60


def test_sustained_large_error_reaches_new_turn_cap_without_linear_motion():
    core = acquired()
    core.update('preview', preview(angular=0.60), 10.05)
    for index in range(1, 19):
        now = 10.06 + index * 0.10
        core.update('mission', mission(), now)
        core.update('preview', preview(angular=0.60), now)
        core.update_arm(True, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_scan(-3.14, 3.14, [1.0] * 360, now)
        angular, report, action = step(
            core, now, owned=True, raw=RAW)
        assert report['state'] == 'TURNING'
        assert report['command']['linear_x_m_s'] == 0.0
        assert action is None
    assert abs(angular - 0.60) < 1e-9


def test_centered_stops_zero_then_releases_once_stationary():
    core = turning()
    core.update('preview', preview('CENTERED', 0.0), 10.10)
    angular, report, action = step(core, 10.11, owned=True, raw=RAW)
    assert (angular, action) == (0.0, None)
    assert report['state'] == 'STOPPING'
    assert report['stop_reason'] == 'TARGET_CENTERED'
    assert report['stop_alignment_input']['preview_state'] == 'CENTERED'
    assert report['stop_alignment_input']['target_angular_z_rad_s'] == 0.0
    core.update_odom(MOVING, 10.20)
    assert step(core, 10.21, owned=True, raw=RAW)[2] is None
    core.update_odom(ZERO, 10.25)
    core.update_cmd(ZERO, 10.25)
    assert step(core, 10.26, owned=True, raw=RAW)[2] is None
    core.update_odom(ZERO, 10.75)
    core.update_cmd(ZERO, 10.75)
    _, report, action = step(core, 10.77, owned=True, raw=RAW)
    assert report['state'] == 'COMPLETE'
    assert report['stop_alignment_input']['preview_state'] == 'CENTERED'
    assert action == 'RELEASE'
    assert step(core, 10.80)[2] is None


def test_arm_loss_or_foreign_owner_latches_zero():
    core = turning()
    core.update_arm(False, 10.10)
    assert step(core, 10.11, owned=True, raw=RAW)[0] == 0.0
    assert core.phase == 'STOPPING'
    core = turning()
    angular, report, _ = step(
        core, 10.11, owned=True, raw=RAW + FOREIGN)
    assert angular == 0.0
    assert report['stop_reason'] == 'RAW_OWNER_CONFLICT'
    core.update_odom(ZERO, 10.18)
    core.update_cmd(ZERO, 10.18)
    assert step(core, 10.20, owned=True, raw=RAW + FOREIGN)[2] is None
    core.update_odom(ZERO, 10.68)
    core.update_cmd(ZERO, 10.68)
    assert step(core, 10.70, owned=True, raw=RAW + FOREIGN)[2] is None


def test_missing_own_publisher_and_bad_safety_command_stop():
    core = acquired()
    core.update_arm(True, 10.70)
    core.update_odom(ZERO, 10.70)
    core.update_cmd(ZERO, 10.70)
    core.update_scan(0.0, -6.26, [1.0] * 400, 10.70)
    core.update('mission', mission(), 10.70)
    core.update('preview', preview(), 10.70)
    assert step(core, 10.78, owned=True, raw=[])[1][
        'stop_reason'] == 'OWN_PUBLISHER_NOT_DISCOVERED'
    core = turning()
    core.update_cmd((0.05, 0, 0, 0, 0, 0), 10.10)
    assert step(core, 10.11, owned=True, raw=RAW)[1][
        'stop_reason'] == 'ODOM_OR_SAFETY_COMMAND_INVALID'


def test_fire_loss_stale_input_and_new_goal_stop():
    core = turning()
    lost = mission()
    lost['perception']['detected_now'] = False
    core.update('mission', lost, 10.10)
    assert step(core, 10.11, owned=True, raw=RAW)[0] == 0.0
    core = turning()
    core.update_statuses([(UUID, 5), ('b' * 32, 2)])
    assert step(core, 10.11, owned=True, raw=RAW)[1][
        'stop_reason'] == 'NEW_OR_CHANGED_NAV_GOAL'
    core = turning()
    core.update_arm(True, 10.35)
    core.update_odom(ZERO, 10.35)
    core.update_cmd(ZERO, 10.35)
    core.update_scan(-3.14, 3.14, [1.0] * 360, 10.35)
    assert step(core, 10.40, owned=True, raw=RAW)[1][
        'stop_reason'] == 'INPUT_MISSING_OR_STALE'


def test_mission_collision_bypass_or_wrong_preview_sign_is_rejected():
    core = ready()
    bad = mission()
    bad['safety']['collision_check_bypassed'] = True
    core.update('mission', bad, 10.0)
    assert step(core, 10.01)[2] is None
    core = ready()
    core.update('preview', preview('ALIGNING_LEFT', -0.2), 10.0)
    assert step(core, 10.01)[1]['blockers'] == [
        'ALIGNMENT_DIRECTION_INVALID']


def test_long_target_gap_holds_zero_then_reconfirms_same_goal():
    core = turning()
    core.update('preview', preview('NO_TARGET', 0.0), 10.10)
    angular, report, _ = step(core, 10.11, owned=True, raw=RAW)
    assert angular == 0.0
    assert report['state'] == 'TURNING'
    assert report['target_reacquiring'] is True
    for now in (10.5, 11.1, 12.0, 13.0):
        lost = mission()
        lost['state'] = 'TARGET_LOST'
        lost['perception'] = {'detected_now': False, 'confirmed': False}
        core.update('mission', lost, now)
        core.update('preview', preview('NO_TARGET', 0.0), now)
        core.update_arm(True, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_scan(-3.14, 3.14, [1.0] * 360, now)
        angular, report, action = step(core, now + 0.01,
                                       owned=True, raw=RAW)
        assert angular == 0.0 and action is None
        assert report['state'] == 'TURNING'
        assert report['stop_reason'] == ''
        assert report['target_reacquiring'] is True
        assert report['safety']['fail_closed_zero'] is True
        assert report['goal_uuid'] == UUID
    assert report['target_wait_s'] > 2.0

    candidate = mission()
    candidate['state'] = 'FIRE_CANDIDATE'
    core.update('mission', candidate, 13.1)
    core.update('preview', preview(), 13.1)
    core.update_arm(True, 13.1)
    core.update_odom(ZERO, 13.1)
    core.update_cmd(ZERO, 13.1)
    core.update_scan(-3.14, 3.14, [1.0] * 360, 13.1)
    angular, report, _ = step(core, 13.11, owned=True, raw=RAW)
    assert angular == 0.0 and report['target_reacquiring'] is True

    core.update('mission', mission(), 13.3)
    core.update('preview', preview(error=-0.25), 13.3)
    core.update_arm(True, 13.3)
    core.update_odom(ZERO, 13.3)
    core.update_cmd(ZERO, 13.3)
    core.update_scan(-3.14, 3.14, [1.0] * 360, 13.3)
    angular, report, action = step(core, 13.31, owned=True, raw=RAW)
    assert angular > 0.0 and action is None
    assert report['state'] == 'TURNING'
    assert report['target_reacquiring'] is False
    assert report['goal_uuid'] == UUID
    assert report['turn_progress']['since_progress_s'] == 0.0

    core.update('preview', preview('CENTERED', 0.0, error=0.02), 13.32)
    angular, report, action = step(core, 13.33, owned=True, raw=RAW)
    assert angular == 0.0 and action is None
    assert report['state'] == 'STOPPING'
    assert report['stop_reason'] == 'TARGET_CENTERED'


def test_confirmed_detection_without_alignment_preview_waits_then_resumes():
    """A NO_TARGET preview may lag a still-confirmed YOLO detection."""
    core = turning()
    waiting = mission()
    waiting['state'] = 'WAITING_FOR_ALIGNMENT'
    for now in (10.10, 11.10, 12.10):
        core.update('mission', waiting, now)
        core.update('preview', preview('NO_TARGET', 0.0), now)
        core.update_arm(True, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_scan(-3.14, 3.14, [1.0] * 360, now)
        angular, report, action = step(core, now + 0.01,
                                       owned=True, raw=RAW)
        assert (angular, action) == (0.0, None)
        assert report['state'] == 'TURNING'
        assert report['target_reacquiring'] is True
        assert report['stop_reason'] == ''
        assert report['goal_uuid'] == UUID
    now = 12.20
    core.update('mission', mission(), now)
    core.update('preview', preview(error=-0.25), now)
    core.update_arm(True, now)
    core.update_odom(ZERO, now)
    core.update_cmd(ZERO, now)
    core.update_scan(-3.14, 3.14, [1.0] * 360, now)
    angular, report, action = step(core, now + 0.01,
                                   owned=True, raw=RAW)
    assert angular > 0.0 and action is None
    assert report['state'] == 'TURNING'
    assert report['target_reacquiring'] is False


def test_waiting_for_alignment_cannot_hide_unsafe_preview():
    core = turning()
    waiting = mission()
    waiting['state'] = 'WAITING_FOR_ALIGNMENT'
    core.update('mission', waiting, 10.10)
    core.update('preview', preview('INVALID', 0.0), 10.10)
    angular, report, action = step(core, 10.11, owned=True, raw=RAW)
    assert (angular, action) == (0.0, None)
    assert report['state'] == 'STOPPING'
    assert report['stop_reason'] == 'MISSION_FIRE_OR_SAFETY_LOST'


def test_recorded_two_camera_dropouts_do_not_latch_complete():
    """Regression for 23:16 bag: tracking returned after the old 0.9 s cap."""
    core = turning()
    for now, state, visible in (
            (10.10, 'TARGET_LOST', False),
            (10.40, 'FIRE_CANDIDATE', True),
            (10.60, 'FIRE_TRACKING', True),
            (10.80, 'TARGET_LOST', False),
            (11.90, 'FIRE_CANDIDATE', True),
            (12.10, 'FIRE_TRACKING', True)):
        current = mission()
        current['state'] = state
        current['perception'] = {
            'detected_now': visible, 'confirmed': visible}
        current_preview = preview(error=-0.3) if visible else preview(
            'NO_TARGET', 0.0)
        core.update('mission', current, now)
        core.update('preview', current_preview, now)
        core.update_arm(True, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_scan(-3.14, 3.14, [1.0] * 360, now)
        angular, report, action = step(core, now + 0.01,
                                       owned=True, raw=RAW)
        assert report['state'] == 'TURNING'
        assert report['goal_uuid'] == UUID
        assert report['stop_reason'] == ''
        assert action is None
        assert (angular > 0.0) is (state == 'FIRE_TRACKING')


def test_missing_target_can_reacquire_after_old_total_limit():
    core = turning()
    lost = mission()
    lost['state'] = 'TARGET_LOST'
    lost['perception'] = {'detected_now': False, 'confirmed': False}
    for now in (10.1, 30.0, 54.95):
        core.update('mission', lost, now)
        core.update('preview', preview('NO_TARGET', 0.0), now)
        core.update_arm(True, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_scan(-3.14, 3.14, [1.0] * 360, now)
        angular, report, _ = step(core, now + 0.01,
                                  owned=True, raw=RAW)
        assert angular == 0.0
        assert report['state'] == 'TURNING'
    now = 55.10
    core.update('mission', mission(), now)
    core.update('preview', preview(error=-0.05), now)
    core.update_arm(True, now)
    core.update_odom(ZERO, now)
    core.update_cmd(ZERO, now)
    core.update_scan(-3.14, 3.14, [1.0] * 360, now)
    angular, report, action = step(core, now + 0.01,
                                   owned=True, raw=RAW)
    assert angular > 0.0 and action is None
    assert report['state'] == 'TURNING'
    assert report['stop_reason'] == ''
    assert report['target_wait_s'] is None
    assert report['turn_progress']['total_limit_s'] is None


def test_multiple_target_losses_resume_same_goal_without_command_while_blind():
    core = turning()
    for index in range(4):
        now = 15.0 + index * 20.0
        lost = mission()
        lost['state'] = 'TARGET_LOST'
        lost['perception'] = {'detected_now': False, 'confirmed': False}
        for offset in (0.0, 5.0, 10.0):
            at = now + offset
            core.update('mission', lost, at)
            core.update('preview', preview('NO_TARGET', 0.0), at)
            core.update_arm(True, at)
            core.update_odom(ZERO, at)
            core.update_cmd(ZERO, at)
            core.update_scan(-3.14, 3.14, [1.0] * 360, at)
            angular, report, action = step(
                core, at + 0.01, owned=True, raw=RAW)
            assert (angular, action) == (0.0, None)
            assert report['state'] == 'TURNING'
            assert report['target_reacquiring'] is True
        at = now + 10.2
        core.update('mission', mission(), at)
        core.update('preview', preview(error=-0.3), at)
        core.update_arm(True, at)
        core.update_odom(ZERO, at)
        core.update_cmd(ZERO, at)
        core.update_scan(-3.14, 3.14, [1.0] * 360, at)
        angular, report, _ = step(core, at + 0.01,
                                  owned=True, raw=RAW)
        assert angular > 0.0 and report['state'] == 'TURNING'
        assert report['goal_uuid'] == UUID


def test_commanded_turn_with_no_measured_odom_motion_stops_as_stall():
    core = turning()
    result = None
    for index in range(40):
        now = 10.1 + index * 0.1
        core.update('mission', mission(), now)
        core.update('preview', preview(error=-0.3), now)
        core.update_arm(True, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_scan(-3.14, 3.14, [1.0] * 360, now)
        _, result, _ = step(core, now + 0.01,
                            owned=True, raw=RAW)
        if result['state'] == 'STOPPING':
            break
    assert result['stop_reason'] == 'ODOM_TURN_STALLED'


def test_measured_rotation_budget_stops_persistent_false_target():
    core = turning()
    core.max_total_rotation = 0.24
    result = None
    for index in range(6):
        now = 10.16 + index * 0.1
        core.update('mission', mission(), now)
        core.update('preview', preview(error=-0.3), now)
        core.update_arm(True, now)
        core.update_odom((0, 0, 0, 0, 0, 0.8), now)
        core.update_cmd(ZERO, now)
        core.update_scan(-3.14, 3.14, [1.0] * 360, now)
        _, result, _ = step(core, now + 0.01,
                            owned=True, raw=RAW)
        if result['state'] == 'STOPPING':
            break
    assert result['stop_reason'] == 'TURN_ROTATION_LIMIT'
    assert result['turn_progress']['total_rotation_rad'] >= 0.24


def test_safety_loss_during_target_wait_stops_instead_of_resuming():
    for failure in ('arm', 'scan', 'owner', 'status'):
        core = turning()
        core.update('preview', preview('NO_TARGET', 0.0), 10.10)
        assert step(core, 10.11, owned=True, raw=RAW)[0] == 0.0
        core.update_arm(True, 10.2)
        core.update_odom(ZERO, 10.2)
        core.update_cmd(ZERO, 10.2)
        core.update_scan(-3.14, 3.14, [1.0] * 360, 10.2)
        core.update('mission', mission(), 10.2)
        core.update('preview', preview(error=-0.2), 10.2)
        if failure == 'arm':
            core.update_arm(False, 10.2)
        elif failure == 'scan':
            core.update_scan(-3.14, 3.14, [0.2] * 360, 10.2)
        elif failure == 'status':
            core.update_statuses([(UUID, 5), ('b' * 32, 2)])
        raw = RAW + FOREIGN if failure == 'owner' else RAW
        angular, report, action = step(core, 10.21,
                                       owned=True, raw=raw)
        assert angular == 0.0 and action is None
        assert report['state'] == 'STOPPING'
        assert report['stop_reason'] != 'TARGET_CENTERED'


def test_candidate_pauses_until_mission_reconfirms_same_turn():
    core = turning()
    lost = mission()
    lost['state'] = 'TARGET_LOST'
    lost['perception'] = {'detected_now': False, 'confirmed': False}
    core.update('mission', lost, 10.10)
    core.update('preview', preview('NO_TARGET', 0.0), 10.10)
    angular, report, _ = step(core, 10.11, owned=True, raw=RAW)
    assert angular == 0.0 and report['target_reacquiring'] is True

    candidate = mission()
    candidate['state'] = 'FIRE_CANDIDATE'
    for now in (10.30, 10.60, 10.80):
        core.update('mission', candidate, now)
        core.update('preview', preview(), now)
        core.update_arm(True, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_scan(-3.14, 3.14, [1.0] * 360, now)
        angular, report, _ = step(core, now + 0.01, owned=True, raw=RAW)
        assert angular == 0.0
        assert report['state'] == 'TURNING'
        assert report['goal_uuid'] == UUID
        assert report['target_reacquiring'] is True

    core.update('mission', mission(), 10.90)
    core.update('preview', preview(), 10.90)
    core.update_arm(True, 10.90)
    core.update_odom(ZERO, 10.90)
    core.update_cmd(ZERO, 10.90)
    core.update_scan(-3.14, 3.14, [1.0] * 360, 10.90)
    angular, report, _ = step(core, 10.91, owned=True, raw=RAW)
    assert report['state'] == 'TURNING'
    assert report['target_reacquiring'] is False
    assert angular > 0.0


def test_mission_reconfirmation_resumes_owned_alignment_for_same_goal():
    supervisor = MissionSupervisorCore()
    supervisor.update_goal_statuses([(UUID, 2)], 9.0)
    supervisor.update_detection({
        'observe_only': True, 'state': 'ok',
        'source': {'age_ms': 10.0},
        'fire': {'detected_now': True, 'confirmed': True}}, 9.0)
    supervisor.update_alignment(preview(), 9.0)
    assert supervisor.evaluate(9.0)['state'] == 'FIRE_CANDIDATE'
    supervisor.update_detection({
        'observe_only': True, 'state': 'ok',
        'source': {'age_ms': 10.0},
        'fire': {'detected_now': True, 'confirmed': True}}, 9.6)
    supervisor.update_alignment(preview(), 9.6)
    assert supervisor.evaluate(9.6)['state'] == 'FIRE_TRACKING'
    supervisor.update_goal_statuses([(UUID, 5)], 10.0)

    core = turning()
    observations = (
        (10.10, False, 'TARGET_LOST'),
        (10.30, True, 'FIRE_CANDIDATE'),
        (10.51, True, 'FIRE_TRACKING'),
    )
    for now, detected, state in observations:
        current_preview = preview() if detected else preview(
            'NO_TARGET', 0.0)
        supervisor.update_detection({
            'observe_only': True, 'state': 'ok',
            'source': {'age_ms': 10.0},
            'fire': {'detected_now': detected, 'confirmed': detected}}, now)
        supervisor.update_alignment(current_preview, now)
        supervisor.update_odom(0.0, 0.0, now)
        report = supervisor.evaluate(now)
        assert report['state'] == state
        core.update('mission', report, now)
        core.update('preview', current_preview, now)
        core.update_arm(True, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_scan(-3.14, 3.14, [1.0] * 360, now)
        angular, turn, _ = step(core, now + 0.01, owned=True, raw=RAW)
        assert turn['state'] == 'TURNING'
        assert (angular > 0.0) is (state == 'FIRE_TRACKING')


def test_candidate_cannot_start_turn_or_complete_center_handoff():
    candidate = mission()
    candidate['state'] = 'FIRE_CANDIDATE'
    core = ready()
    core.update('mission', candidate, 10.0)
    assert step(core, 10.01)[2] is None

    core = turning()
    core.update('mission', candidate, 10.10)
    core.update('preview', preview('CENTERED', 0.0, error=0.09), 10.10)
    angular, report, action = step(core, 10.11, owned=True, raw=RAW)
    assert (angular, action) == (0.0, None)
    assert report['state'] == 'TURNING'
    assert report['stop_reason'] == ''
    core.update('mission', mission(), 10.12)
    angular, report, _ = step(core, 10.13, owned=True, raw=RAW)
    assert report['state'] == 'STOPPING'
    assert report['stop_reason'] == 'TARGET_CENTERED'


def test_candidate_without_current_confirmed_fire_keeps_zero_command():
    core = turning()
    candidate = mission()
    candidate['state'] = 'FIRE_CANDIDATE'
    candidate['perception'] = {'detected_now': False, 'confirmed': False}
    core.update('mission', candidate, 10.10)
    core.update('preview', preview('NO_TARGET', 0.0), 10.10)
    angular, report, _ = step(core, 10.11, owned=True, raw=RAW)
    assert angular == 0.0
    assert report['state'] == 'TURNING'
    assert report['target_reacquiring'] is True


def test_candidate_reconfirmation_never_hides_a_mission_fault():
    core = turning()
    fault = mission()
    fault['state'] = 'FAULT'
    core.update('mission', fault, 10.10)
    core.update('preview', preview('NO_TARGET', 0.0), 10.10)
    angular, report, _ = step(core, 10.11, owned=True, raw=RAW)
    assert angular == 0.0
    assert report['state'] == 'STOPPING'
    assert report['stop_reason'] == 'MISSION_FIRE_OR_SAFETY_LOST'


def test_one_frame_target_miss_pauses_then_reacquires_from_zero():
    core = turning()
    core.update('preview', preview('NO_TARGET', 0.0), 10.10)
    angular, report, _ = step(core, 10.11, owned=True, raw=RAW)
    assert angular == 0.0 and report['target_reacquiring'] is True
    core.update('preview', preview('ALIGNING_LEFT', 0.35), 10.12)
    angular, report, _ = step(core, 10.13, owned=True, raw=RAW)
    assert report['state'] == 'TURNING'
    assert report['target_reacquiring'] is False
    assert angular > 0.0


def test_completed_alignment_rearms_only_for_distinct_active_goal():
    core = ready()
    core.phase = 'COMPLETE'
    core.goal_uuid = UUID
    core.stop_reason = 'TARGET_CENTERED'
    core.update_statuses([(UUID, 5), ('b' * 32, 2)])
    angular, report, action = step(core, 10.01)
    assert angular == 0.0 and action is None
    assert report['state'] == 'WAITING_FOR_ADMISSION'
    assert report['goal_uuid'] is None
    assert UUID in core.used_goal_uuids
    assert core.inputs == {}


def test_same_goal_realign_requires_verified_ir_stop_and_zero_owner():
    core = ready()
    core.phase = 'COMPLETE'
    core.goal_uuid = UUID
    core.stop_reason = 'TARGET_CENTERED'
    core.update('ir', {
        'schema_version': 1, 'state': 'COMPLETE',
        'stop_reason': 'REALIGN_REQUIRED', 'goal_uuid': UUID,
        'publisher_owned': False, 'step_count': 1}, 10.0)
    shifted = mission()
    shifted['state'] = 'FIRE_TRACKING'
    core.update('mission', shifted, 10.0)
    counts = dict(COUNTS, ir=1, ir_source_ok=False)
    assert step(core, 10.01, counts=counts)[1]['state'] == 'COMPLETE'
    counts['ir_source_ok'] = True
    assert step(core, 10.02, raw=RAW, counts=counts)[1][
        'state'] == 'COMPLETE'
    angular, report, action = step(core, 10.03, counts=counts)
    assert angular == 0.0 and action == 'CREATE'
    assert report['state'] == 'ACQUIRING'
    assert report['goal_uuid'] == UUID
    assert core.last_realign_ir_step == 1


def test_ir_sensor_recovery_does_not_turn_before_new_samples():
    core = ready()
    core.phase = 'COMPLETE'
    core.goal_uuid = UUID
    core.stop_reason = 'TARGET_CENTERED'
    report = {
        'schema_version': 1, 'state': 'COMPLETE',
        'stop_reason': 'REALIGN_REQUIRED', 'goal_uuid': UUID,
        'publisher_owned': False, 'step_count': 1,
        'stop_trigger_blockers': ['IR_BAND_INVALID_OR_STALE',
                                  'IR_BAND_HOLD'],
        'recovery_required': True,
        'recovery_ready': False,
    }
    core.update('ir', report, 10.0)
    shifted = mission()
    shifted['state'] = 'FIRE_TRACKING'
    core.update('mission', shifted, 10.0)
    counts = dict(COUNTS, ir=1, ir_source_ok=True)
    angular, state, action = step(core, 10.01, counts=counts)
    assert angular == 0.0 and action is None
    assert state['state'] == 'COMPLETE'
    core.update('ir', dict(report, recovery_ready=True), 10.02)
    angular, state, action = step(core, 10.03, counts=counts)
    assert angular == 0.0 and action == 'CREATE'
    assert state['state'] == 'ACQUIRING'


def test_realign_when_fire_already_centered_uses_zero_command():
    core = ready()
    core.phase = 'COMPLETE'
    core.goal_uuid = UUID
    core.stop_reason = 'TARGET_CENTERED'
    core.update('ir', {
        'schema_version': 1, 'state': 'COMPLETE',
        'stop_reason': 'REALIGN_REQUIRED', 'goal_uuid': UUID,
        'publisher_owned': False, 'step_count': 1}, 10.0)
    centered = mission()
    centered['state'] = 'ALIGNMENT_READY'
    core.update('mission', centered, 10.0)
    core.update('preview', preview('CENTERED', 0.0), 10.0)
    counts = dict(COUNTS, ir=1, ir_source_ok=True)
    angular, report, action = step(core, 10.01, counts=counts)
    assert angular == 0.0 and action is None
    assert report['state'] == 'COMPLETE'
    assert report['stop_reason'] == 'TARGET_CENTERED'


def test_post_spray_realign_waits_for_pump_release_and_actuator_off():
    core = ready()
    core.phase = 'COMPLETE'
    core.goal_uuid = UUID
    core.stop_reason = 'TARGET_CENTERED'
    core.update('review', {
        'schema_version': 1, 'mode': 'SHADOW_ONLY',
        'state': 'REALIGN_REQUIRED', 'goal_uuid': UUID,
        'pulse_index': 1, 'blockers': []}, 10.0)
    core.update('actuator', {
        'pump_pwm_commanded_on': False,
        'servo_pwm_commanded_on': False,
        'servo_phase': 'IDLE', 'receive_age_ms': 20.0}, 10.0)
    counts = dict(COUNTS, review=1, review_source_ok=True,
                  actuator_source_ok=True, pump=1)
    assert step(core, 10.01, counts=counts)[1]['state'] == 'COMPLETE'
    assert core.pending_review[0] == 1
    counts['pump'] = 0
    counts['actuator_source_ok'] = False
    assert step(core, 10.02, counts=counts)[1]['state'] == 'COMPLETE'
    counts['actuator_source_ok'] = True
    angular, report, action = step(core, 10.03, counts=counts)
    assert angular == 0.0 and action == 'CREATE'
    assert report['state'] == 'ACQUIRING'
    assert core.last_review_pulse == 1


def test_failed_alignment_cannot_rearm_for_new_goal():
    core = ready()
    core.phase = 'COMPLETE'
    core.goal_uuid = UUID
    core.stop_reason = 'TURN_TIME_LIMIT'
    core.update_statuses([(UUID, 5), ('b' * 32, 2)])
    assert step(core, 10.01)[1]['state'] == 'COMPLETE'


def test_turn_elapsed_time_is_not_terminal_but_overspeed_stops():
    core = turning()
    for now in (25.00, 25.03):
        core.update('mission', mission(), now)
        core.update('preview', preview(), now)
        core.update_arm(True, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_scan(-3.14, 3.14, [1.0] * 360, now)
        report = step(core, now, owned=True, raw=RAW)[1]
        if now == 25.00:
            assert report['state'] == 'TURNING'
    assert report['state'] == 'TURNING'
    assert report['stop_reason'] == ''
    core = turning()
    core.update_odom((0, 0, 0, 0, 0, 0.79), 10.10)
    assert step(core, 10.11, owned=True, raw=RAW)[1]['state'] == 'TURNING'
    core.update_odom((0, 0, 0, 0, 0, 0.81), 10.12)
    assert step(core, 10.13, owned=True, raw=RAW)[1][
        'stop_reason'] == 'ODOM_ANGULAR_OVERSPEED'


def test_real_progress_continues_past_old_total_cap():
    core = turning()
    for now, error in ((14.0, -0.25), (19.0, -0.18),
                       (24.0, -0.11)):
        core.update('mission', mission(), now)
        core.update('preview', preview(error=error), now)
        core.update_arm(True, now)
        core.update_odom(ZERO, now)
        core.update_cmd(ZERO, now)
        core.update_scan(-3.14, 3.14, [1.0] * 360, now)
        _, report, _ = step(core, now + 0.01, owned=True, raw=RAW)
        assert report['state'] == 'TURNING'
    core.update('mission', mission(), 55.10)
    core.update('preview', preview(error=-0.04), 55.10)
    core.update_arm(True, 55.10)
    core.update_odom(ZERO, 55.10)
    core.update_cmd(ZERO, 55.10)
    core.update_scan(-3.14, 3.14, [1.0] * 360, 55.10)
    _, report, _ = step(core, 55.11, owned=True, raw=RAW)
    assert report['state'] == 'TURNING'
    assert report['stop_reason'] == ''


def test_right_sign_and_reversal_do_not_command_old_direction():
    core = turning()
    core.update('preview', preview('ALIGNING_RIGHT', -0.35), 10.10)
    angular, report, _ = step(core, 10.11, owned=True, raw=RAW)
    assert report['state'] == 'TURNING'
    assert angular < 0.0


def test_old_guard_may_fail_after_takeover_without_revoking_live_owner():
    core = turning()
    core.update('guard', dict(guard(), state='FAILED'), 10.10)
    core.update('gateway', dict(gateway(), state='RELEASED_STOPPED'), 10.10)
    core.update('handoff', dict(handoff(), active=False), 10.10)
    angular, report, _ = step(core, 10.11, owned=True, raw=RAW)
    assert report['state'] == 'TURNING'
    assert angular > 0.0


def test_hard_speed_and_arm_limits_cannot_be_overridden():
    for kwargs in ({'max_angular_speed_rad_s': 0.61},
                   {'max_observed_angular_rad_s': 0.81},
                   {'max_observed_angular_rad_s': 0.59},
                   {'arm_timeout_s': 0.51},
                   {'input_timeout_s': 0.51},
                   {'zero_hold_s': 0.49},
                   {'min_rotation_clearance_m': 0.29},
                   {'max_total_rotation_rad': 2 * 3.141592653589793 + 0.01},
                   {'stationary_angular_rad_s': 0.06},
                   {'max_angular_accel_rad_s2': 0.51}):
        try:
            LiveAlignmentCore(enabled=True, **kwargs)
        except ValueError:
            pass
        else:
            raise AssertionError('unsafe parameter was accepted')
