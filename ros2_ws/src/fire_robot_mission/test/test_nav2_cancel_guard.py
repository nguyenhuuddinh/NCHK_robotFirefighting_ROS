"""Unit tests for one-shot, goal-specific Nav2 cancellation."""

from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
import pytest
from rclpy.qos import ReliabilityPolicy

from fire_robot_mission.nav2_cancel_guard import (
    Nav2CancelGuardCore, make_cancel_request,
)


UUID = 'ab' * 16
OTHER = 'cd' * 16
ZERO = (0.0,) * 6
SHADOW_SOURCES = [
    ('/', 'velocity_smoother', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.RELIABLE),
    ('/', 'behavior_server', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.RELIABLE),
    ('/', 'behavior_server', 'geometry_msgs/msg/Twist',
     ReliabilityPolicy.RELIABLE),
]
ALIGNMENT_RAW = [('/', 'live_alignment_takeover',
                  'geometry_msgs/msg/Twist', ReliabilityPolicy.BEST_EFFORT)]
IR_RAW = [('/', 'ir_approach_live', 'geometry_msgs/msg/Twist',
           ReliabilityPolicy.BEST_EFFORT)]


def intent(uuid=UUID):
    return {
        'schema_version': 1,
        'shadow_only': True,
        'state': 'REQUEST_CANCEL',
        'request_cancel': True,
        'reason': 'FIRE_ACQUIRED',
        'blockers': [],
        'mission': {'state': 'FIRE_TRACKING', 'intent': 'LEFT_PREVIEW'},
        'navigation': {
            'mission_goal_active': True,
            'active_goals': [{'uuid': uuid, 'status': 2}],
            'status_publisher_count': 1,
        },
        'safety': {
            'nav2_action_client': False,
            'cancel_request_sent': False,
            'motion_command_publisher': False,
            'actuator_command_publisher': False,
        },
    }


def ready_core(enabled=True):
    core = Nav2CancelGuardCore(enabled=enabled)
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 0, 1, 1, True)
    core.update_arm(True, 10.0)
    core.update_intent(intent(), 10.0)
    core.update_statuses([(UUID, 2)], 10.0)
    core.update_cmd(ZERO, 10.0)
    core.update_odom(ZERO, 10.0)
    return core


def accepted_response(uuid=UUID):
    response = CancelGoal.Response()
    response.return_code = CancelGoal.Response.ERROR_NONE
    goal = GoalInfo()
    goal.goal_id.uuid = list(bytes.fromhex(uuid))
    response.goals_canceling = [goal]
    return response


def test_cancel_request_can_only_target_exact_nonzero_uuid():
    request = make_cancel_request(UUID)
    assert bytes(request.goal_info.goal_id.uuid).hex() == UUID
    assert request.goal_info.stamp.sec == 0
    assert request.goal_info.stamp.nanosec == 0
    with pytest.raises(ValueError):
        make_cancel_request('0' * 32)
    with pytest.raises(ValueError):
        make_cancel_request('bad')


def test_disabled_guard_never_sends():
    payload, send = ready_core(enabled=False).step(10.1)
    assert payload['state'] == 'DISABLED'
    assert send is None
    assert payload['send_count'] == 0


def test_missing_operator_arm_blocks():
    core = ready_core()
    core.update_arm(False, 10.0)
    payload, send = core.step(10.1)
    assert send is None
    assert 'OPERATOR_ARM_MISSING_OR_STALE' in payload['blockers']


def test_shadow_isolation_and_fresh_zero_are_required():
    core = ready_core()
    core.update_topology(1, 1, 1, [], 1, 1, 1, True)
    payload, send = core.step(10.1)
    assert send is None
    assert 'SHADOW_MOTION_ISOLATION_NOT_PROVEN' in payload['blockers']
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 0, 1, 1, True)
    core.update_cmd((0.1, 0, 0, 0, 0, 0), 10.1)
    payload, send = core.step(10.2)
    assert send is None
    assert 'CMD_VEL_NOT_FRESH_ZERO' in payload['blockers']


def test_real_motion_publisher_blocks_even_with_valid_shadow_sources():
    core = ready_core()
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 1, 1, 1, True)
    payload, send = core.step(10.1)
    assert send is None
    assert payload['nav_shadow_fingerprint_ok'] is True
    assert payload['raw_cmd_publishers'] == 1
    assert 'SHADOW_MOTION_ISOLATION_NOT_PROVEN' in payload['blockers']


def test_post_cancel_failure_latches_exact_command_evidence():
    core = ready_core()
    core.goal_uuid = UUID
    core.accepted_at = 10.0
    core.canceled_at = 10.0
    core.complete = True
    core.update_statuses([(UUID, 5)], 10.0)
    core.update_cmd(ZERO, 10.0)
    report, _ = core.step(10.1)
    assert report['state'] == 'CANCEL_CONFIRMED_STOPPED'
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 0, 2, 1, True)
    report, _ = core.step(10.2)
    assert report['state'] == 'FAILED'
    assert report['blockers'] == ['POST_CANCEL_SAFETY_LOST']
    assert report['failure_evidence']['command']['publisher_count'] == 2
    assert report['failure_evidence']['command']['values'] == list(ZERO)
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 0, 1, 1, True)
    report, _ = core.step(10.3)
    assert report['command_evidence']['fresh_zero'] is True
    assert report['failure_evidence']['command']['publisher_count'] == 2


def test_completed_cancel_rearms_only_for_fresh_distinct_goal():
    core = ready_core()
    core.goal_uuid = UUID
    core.accepted_at = 10.0
    core.canceled_at = 10.0
    core.complete = True
    core.send_count = 1
    core.update_statuses([(UUID, 5)], 10.0)
    assert core.step(10.1)[0]['state'] == 'CANCEL_CONFIRMED_STOPPED'
    core.update_statuses([(UUID, 5), (OTHER, 2)], 10.2)
    core.update_arm(True, 10.2)
    core.update_cmd(ZERO, 10.2)
    core.update_odom(ZERO, 10.2)
    report, send = core.step(10.21)
    assert send is None
    assert report['state'] == 'WAITING_FOR_ADMISSION'
    assert report['goal_uuid'] is None
    assert UUID in core.used_goal_uuids
    core.update_intent(intent(OTHER), 10.22)
    report, send = core.step(10.23)
    assert report['state'] == 'REQUEST_SENT'
    assert report['goal_uuid'] == OTHER and send == OTHER


def test_cancel_guard_rearms_second_goal_when_old_status_was_pruned():
    core = ready_core()
    core.goal_uuid = UUID
    core.accepted_at = 10.0
    core.canceled_at = 10.0
    core.complete = True
    core.send_count = 1
    core.update_statuses([(OTHER, 2)], 10.2)
    core.update_arm(True, 10.2)
    core.update_cmd(ZERO, 10.2)
    core.update_odom(ZERO, 10.2)
    report, send = core.step(10.21)
    assert send is None and report['goal_uuid'] is None
    assert UUID in core.used_goal_uuids
    core.update_intent(intent(OTHER), 10.22)
    report, send = core.step(10.23)
    assert report['state'] == 'REQUEST_SENT'
    assert send == OTHER


def test_failed_cancel_cannot_rearm_on_new_goal():
    core = ready_core()
    core.goal_uuid = UUID
    core.complete = True
    core.failure = 'POST_CANCEL_SAFETY_LOST'
    core.update_statuses([(UUID, 5), (OTHER, 2)], 10.2)
    core.update_arm(True, 10.2)
    core.update_cmd(ZERO, 10.2)
    core.update_odom(ZERO, 10.2)
    report, send = core.step(10.21)
    assert report['state'] == 'FAILED'
    assert report['goal_uuid'] == UUID and send is None


def test_exact_alignment_owner_temporarily_revokes_then_restores_proof():
    core = ready_core()
    core.goal_uuid = UUID
    core.accepted_at = 10.0
    core.canceled_at = 10.0
    core.complete = True
    core.send_count = 1
    core.update_statuses([(UUID, 5)], 10.0)
    assert core.step(10.1)[0]['state'] == 'CANCEL_CONFIRMED_STOPPED'
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 1, 1, 1, True,
                         ALIGNMENT_RAW)
    core.update_arm(True, 10.2)
    core.update_cmd((0, 0, 0, 0, 0, 0.2), 10.2)
    assert core.step(10.21)[0]['state'] == 'ALIGNMENT_IN_PROGRESS'
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 0, 1, 1, True)
    core.update_alignment({'state': 'COMPLETE', 'goal_uuid': UUID,
                           'stop_reason': 'TARGET_CENTERED',
                           'publisher_owned': False,
                           'command': {'angular_z_rad_s': 0.0}}, 10.3)
    core.update_cmd(ZERO, 10.3)
    core.update_odom(ZERO, 10.3)
    assert core.step(10.31)[0]['state'] == 'WAITING_FOR_ALIGNMENT_STOP'
    core.update_alignment(core.alignment, 10.85)
    core.update_arm(True, 10.85)
    core.update_cmd(ZERO, 10.85)
    core.update_odom(ZERO, 10.85)
    assert core.step(10.86)[0]['state'] == 'CANCEL_CONFIRMED_STOPPED'


def test_guard_keeps_exact_alignment_owner_during_long_zero_target_wait():
    core = ready_core()
    core.goal_uuid = UUID
    core.accepted_at = 10.0
    core.canceled_at = 10.0
    core.complete = True
    core.send_count = 1
    core.update_statuses([(UUID, 5)], 10.0)
    assert core.step(10.1)[0]['state'] == 'CANCEL_CONFIRMED_STOPPED'
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 1, 1, 1, True,
                         ALIGNMENT_RAW)
    for now in (10.2, 12.0, 20.0, 70.0):
        core.update_alignment({
            'schema_version': 1, 'enabled': True,
            'state': 'TURNING', 'goal_uuid': UUID,
            'publisher_owned': True,
            'command': {'linear_x_m_s': 0.0,
                        'angular_z_rad_s': 0.0},
            'safety': {'pump_command_publisher': False}}, now)
        core.update_arm(True, now)
        core.update_cmd(ZERO, now)
        core.update_odom(ZERO, now)
        core.update_statuses([(UUID, 5)], now)
        report, send = core.step(now + 0.01)
        assert send is None
        assert report['state'] == 'ALIGNMENT_IN_PROGRESS'
        assert report['goal_uuid'] == UUID
        assert report['blockers'] == []


def test_guard_rejects_stale_alignment_report_even_with_raw_owner():
    core = ready_core()
    core.goal_uuid = UUID
    core.accepted_at = 10.0
    core.canceled_at = 10.0
    core.complete = True
    core.send_count = 1
    core.update_statuses([(UUID, 5)], 10.0)
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 1, 1, 1, True,
                         ALIGNMENT_RAW)
    core.update_arm(True, 10.2)
    core.update_cmd(ZERO, 10.2)
    core.update_odom(ZERO, 10.2)
    assert core.step(10.21)[0]['state'] == 'ALIGNMENT_IN_PROGRESS'
    core.update_arm(True, 11.0)
    core.update_cmd(ZERO, 11.0)
    core.update_odom(ZERO, 11.0)
    core.update_statuses([(UUID, 5)], 11.0)
    report, _ = core.step(11.01)
    assert report['state'] == 'FAILED'
    assert 'ALIGNMENT_STATE_STALE_OR_INVALID' in report['blockers']


def test_guard_rejects_alignment_speed_or_new_goal_during_long_wait():
    for unsafe_cmd, statuses in (
            ((0.7, 0, 0, 0, 0, 0), [(UUID, 5)]),
            (ZERO, [(UUID, 5), (OTHER, 2)])):
        core = ready_core()
        core.goal_uuid = UUID
        core.accepted_at = 10.0
        core.canceled_at = 10.0
        core.complete = True
        core.send_count = 1
        core.update_topology(1, 1, 1, SHADOW_SOURCES, 1, 1, 1, True,
                             ALIGNMENT_RAW)
        core.update_arm(True, 70.0)
        core.update_statuses(statuses, 70.0)
        core.update_cmd(unsafe_cmd, 70.0)
        core.update_odom(ZERO, 70.0)
        report, _ = core.step(70.01)
        assert report['state'] == 'FAILED'
        assert 'ALIGNMENT_MOTION_OR_CANCEL_PROOF_LOST' in report['blockers']


def test_unknown_raw_owner_after_cancel_still_fails_closed():
    core = ready_core()
    core.goal_uuid = UUID
    core.complete = True
    core.update_statuses([(UUID, 5)], 10.0)
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 1, 1, 1, True,
                         [('/', 'rogue', 'geometry_msgs/msg/Twist',
                           ReliabilityPolicy.BEST_EFFORT)])
    assert core.step(10.1)[0]['state'] == 'FAILED'


def test_pre_spray_clear_retires_failed_alignment_only_when_stopped():
    core = ready_core()
    core.goal_uuid = UUID
    core.complete = True
    core.canceled_at = 10.0
    core.failure = 'ALIGNMENT_STOP_PROOF_INVALID'
    core.update_statuses([(UUID, 5)], 10.0)
    proof = {'schema_version': 1, 'state': 'IDLE',
             'perception': {'detected_now': False, 'confirmed': False},
             'lifecycle': {'pre_spray_clear_goal_uuid': UUID}}
    core.update_mission(proof, 10.0, source_ok=False)
    assert core.step(10.1)[0]['state'] == 'FAILED'
    core.update_mission(proof, 10.2, source_ok=True)
    core.update_odom((0.1, 0, 0, 0, 0, 0), 10.2)
    assert core.step(10.21)[0]['state'] == 'FAILED'
    core.update_odom(ZERO, 10.3)
    core.update_cmd(ZERO, 10.3)
    core.update_statuses([(UUID, 5)], 10.3)
    core.pump_publishers = 1
    assert core.step(10.31)[0]['state'] == 'FAILED'
    core.pump_publishers = 0
    assert core.step(10.31)[0]['goal_uuid'] is None
    assert UUID in core.used_goal_uuids


def test_pre_spray_clear_accepts_unique_early_next_goal_but_not_new_fire():
    core = ready_core()
    core.goal_uuid = UUID
    core.complete = True
    core.canceled_at = 10.0
    core.failure = 'ALIGNMENT_STOP_PROOF_INVALID'
    core.update_statuses([(UUID, 5), (OTHER, 2)], 10.0)
    proof = {'schema_version': 1, 'state': 'NAV2_NAVIGATING',
             'perception': {'detected_now': False, 'confirmed': False},
             'lifecycle': {'pre_spray_clear_goal_uuid': UUID}}
    core.update_mission(proof, 10.0)
    report, _ = core.step(10.1)
    assert report['goal_uuid'] is None
    assert UUID in core.used_goal_uuids

    core = ready_core()
    core.goal_uuid = UUID
    core.complete = True
    core.canceled_at = 10.0
    core.failure = 'ALIGNMENT_STOP_PROOF_INVALID'
    core.update_statuses([(UUID, 5), (OTHER, 2)], 10.0)
    proof['perception'] = {'detected_now': True, 'confirmed': True}
    core.update_mission(proof, 10.0)
    assert core.step(10.1)[0]['goal_uuid'] == UUID


@pytest.mark.parametrize('statuses', [
    [(UUID, 2)],
    [(UUID, 5), (OTHER, 2), ('ef' * 16, 2)],
])
def test_pre_spray_clear_never_retires_with_old_or_ambiguous_active_goal(
        statuses):
    core = ready_core()
    core.goal_uuid = UUID
    core.complete = True
    core.canceled_at = 10.0
    core.failure = 'ALIGNMENT_STOP_PROOF_INVALID'
    core.update_statuses(statuses, 10.0)
    core.update_mission({
        'schema_version': 1, 'state': 'NAV2_NAVIGATING',
        'perception': {'detected_now': False, 'confirmed': False},
        'lifecycle': {'pre_spray_clear_goal_uuid': UUID},
    }, 10.0)
    assert core.step(10.1)[0]['goal_uuid'] == UUID


def ir_report(state='MOVING', goal_uuid=UUID, owned=True,
              reason='', linear=0.05, continuous=False):
    return {
        'schema_version': 1, 'enabled': True, 'one_shot': True,
        'integrated_handoff': True,
        'continuous_approach': continuous,
        'state': state, 'goal_uuid': goal_uuid,
        'publisher_owned': owned, 'direction': 'FORWARD',
        'stop_reason': reason,
        'command': {'linear_x_m_s': linear, 'angular_z_rad_s': 0.0},
        'safety': {'pump_command_publisher': False},
    }


def completed_alignment(core, now=10.0):
    core.goal_uuid = UUID
    core.accepted_at = now
    core.canceled_at = now
    core.complete = True
    core.send_count = 1
    core.update_statuses([(UUID, 5)], now)
    core.update_alignment({
        'state': 'COMPLETE', 'goal_uuid': UUID,
        'stop_reason': 'TARGET_CENTERED', 'publisher_owned': False,
        'command': {'angular_z_rad_s': 0.0},
    }, now)


def test_exact_ir_owner_after_alignment_temporarily_revokes_cancel_proof():
    core = ready_core()
    completed_alignment(core)
    assert core.step(10.1)[0]['state'] == 'CANCEL_CONFIRMED_STOPPED'
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 1, 1, 1, True, IR_RAW,
                         ir_state_source_ok=True)
    core.update_arm(True, 10.2)
    core.update_alignment(core.alignment, 10.2)
    core.update_ir_approach(ir_report(), 10.2)
    core.update_cmd((0.05, 0, 0, 0, 0, 0), 10.2)
    core.update_odom((0.04, 0, 0, 0, 0, 0), 10.2)
    assert core.step(10.21)[0]['state'] == 'IR_APPROACH_IN_PROGRESS'
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 0, 1, 1, True,
                         ir_state_source_ok=True)
    core.update_ir_approach(ir_report(
        state='COMPLETE', owned=False,
        reason='STEP_PROGRESS_TIME_LIMIT', linear=0.0), 10.3)
    core.update_cmd(ZERO, 10.3)
    core.update_odom(ZERO, 10.3)
    assert core.step(10.31)[0]['state'] == 'WAITING_FOR_IR_STOP'
    core.update_ir_approach(core.ir_approach, 10.85)
    core.update_arm(True, 10.85)
    core.update_cmd(ZERO, 10.85)
    core.update_odom(ZERO, 10.85)
    assert core.step(10.86)[0]['state'] == 'CANCEL_CONFIRMED_STOPPED'


def test_continuous_ir_eight_cm_per_second_and_stationary_stop_proof():
    core = ready_core()
    completed_alignment(core)
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 1, 1, 1, True, IR_RAW,
                         ir_state_source_ok=True)
    core.update_arm(True, 10.2)
    core.update_alignment(core.alignment, 10.2)
    core.update_ir_approach(
        ir_report(linear=0.08, continuous=True), 10.2)
    core.update_cmd((0.08, 0, 0, 0, 0, 0), 10.2)
    core.update_odom((0.07, 0, 0, 0, 0, 0), 10.2)
    assert core.step(10.21)[0]['state'] == 'IR_APPROACH_IN_PROGRESS'
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 0, 1, 1, True,
                         ir_state_source_ok=True)
    core.update_ir_approach(
        ir_report(state='COMPLETE', owned=False,
                  reason='SPRAY_RANGE_REACHED', linear=0.0,
                  continuous=True), 16.5)
    core.update_arm(True, 16.5)
    core.update_cmd(ZERO, 16.5)
    core.update_odom(ZERO, 16.5)
    assert core.step(16.51)[0]['state'] == 'WAITING_FOR_IR_STOP'
    core.update_ir_approach(core.ir_approach, 17.02)
    core.update_arm(True, 17.02)
    core.update_cmd(ZERO, 17.02)
    core.update_odom(ZERO, 17.02)
    assert core.step(17.03)[0]['state'] == 'CANCEL_CONFIRMED_STOPPED'


def test_realign_stop_retains_cancel_proof_only_after_zero_hold():
    core = ready_core()
    completed_alignment(core)
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 1, 1, 1, True, IR_RAW,
                         ir_state_source_ok=True)
    core.update_arm(True, 10.2)
    core.update_alignment(core.alignment, 10.2)
    core.update_ir_approach(
        ir_report(linear=0.08, continuous=True), 10.2)
    core.update_cmd((0.08, 0, 0, 0, 0, 0), 10.2)
    core.update_odom((0.07, 0, 0, 0, 0, 0), 10.2)
    assert core.step(10.21)[0]['state'] == 'IR_APPROACH_IN_PROGRESS'
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 0, 1, 1, True,
                         ir_state_source_ok=True)
    core.update_ir_approach(
        ir_report(state='COMPLETE', owned=False,
                  reason='REALIGN_REQUIRED', linear=0.0,
                  continuous=True), 10.3)
    core.update_cmd(ZERO, 10.3)
    core.update_odom(ZERO, 10.3)
    assert core.step(10.31)[0]['state'] == 'WAITING_FOR_IR_STOP'
    core.update_ir_approach(core.ir_approach, 10.85)
    core.update_arm(True, 10.85)
    core.update_cmd(ZERO, 10.85)
    core.update_odom(ZERO, 10.85)
    report, _ = core.step(10.86)
    assert report['state'] == 'CANCEL_CONFIRMED_STOPPED'
    assert report['send_count'] == 1
    assert report['goal_uuid'] == UUID


def test_continuous_ir_over_speed_fails_guard_contract():
    core = ready_core()
    completed_alignment(core)
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 1, 1, 1, True, IR_RAW,
                         ir_state_source_ok=True)
    core.update_arm(True, 10.4)
    core.update_alignment(core.alignment, 10.4)
    core.update_ir_approach(
        ir_report(linear=0.09, continuous=True), 10.4)
    core.update_cmd((0.09, 0, 0, 0, 0, 0), 10.4)
    core.update_odom(ZERO, 10.4)
    assert core.step(10.41)[0]['state'] == 'IR_APPROACH_WAITING_FOR_PROOF'
    core.update_arm(True, 10.69)
    core.update_alignment(core.alignment, 10.69)
    core.update_ir_approach(core.ir_approach, 10.69)
    core.update_cmd((0.09, 0, 0, 0, 0, 0), 10.69)
    core.update_odom(ZERO, 10.69)
    assert core.step(10.70)[0]['state'] == 'FAILED'


def test_ir_release_waits_briefly_for_owner_false_telemetry():
    core = ready_core()
    completed_alignment(core)
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 1, 1, 1, True, IR_RAW,
                         ir_state_source_ok=True)
    core.update_arm(True, 10.2)
    core.update_alignment(core.alignment, 10.2)
    core.update_ir_approach(
        ir_report(linear=0.0, continuous=True), 10.2)
    core.update_cmd(ZERO, 10.2)
    core.update_odom(ZERO, 10.2)
    assert core.step(10.21)[0]['state'] == 'IR_APPROACH_IN_PROGRESS'
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 0, 1, 1, True,
                         ir_state_source_ok=True)
    core.update_ir_approach(
        ir_report(state='COMPLETE', reason='SPRAY_RANGE_REACHED',
                  linear=0.0, continuous=True), 10.22)
    core.update_arm(True, 10.22)
    core.update_cmd(ZERO, 10.22)
    core.update_odom(ZERO, 10.22)
    assert core.step(10.23)[0]['state'] == 'WAITING_FOR_IR_STOP'
    core.update_ir_approach(
        ir_report(state='COMPLETE', owned=False,
                  reason='SPRAY_RANGE_REACHED', linear=0.0,
                  continuous=True), 10.3)
    core.update_cmd(ZERO, 10.3)
    core.update_odom(ZERO, 10.3)
    core.update_arm(True, 10.3)
    assert core.step(10.31)[0]['state'] == 'WAITING_FOR_IR_STOP'
    core.update_ir_approach(core.ir_approach, 10.83)
    core.update_cmd(ZERO, 10.83)
    core.update_odom(ZERO, 10.83)
    core.update_arm(True, 10.83)
    assert core.step(10.84)[0]['state'] == 'CANCEL_CONFIRMED_STOPPED'


def test_ir_owner_wrong_uuid_or_motion_above_bound_fails_closed():
    for report, command in (
            (ir_report(goal_uuid=OTHER), (0.05, 0, 0, 0, 0, 0)),
            (ir_report(), (0.2, 0, 0, 0, 0, 0))):
        core = ready_core()
        completed_alignment(core)
        core.update_topology(1, 1, 1, SHADOW_SOURCES, 1, 1, 1, True,
                             IR_RAW, ir_state_source_ok=True)
        core.update_arm(True, 10.4)
        core.update_alignment(core.alignment, 10.4)
        core.update_ir_approach(report, 10.4)
        core.update_cmd(command, 10.4)
        core.update_odom(ZERO, 10.4)
        assert core.step(10.41)[0]['state'] == \
            'IR_APPROACH_WAITING_FOR_PROOF'
        assert core.step(10.7)[0]['state'] == 'FAILED'


def test_ir_raw_owner_without_verified_ir_state_source_fails_closed():
    core = ready_core()
    completed_alignment(core)
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 1, 1, 1, True,
                         IR_RAW, ir_state_source_ok=False)
    assert core.step(10.1)[0]['state'] == 'FAILED'


@pytest.mark.parametrize('sources', [
    SHADOW_SOURCES[:2],
    SHADOW_SOURCES + [
        ('/', 'unknown', 'geometry_msgs/msg/Twist',
         ReliabilityPolicy.RELIABLE)],
    SHADOW_SOURCES[:2] + [
        ('/', 'unknown', 'geometry_msgs/msg/Twist',
         ReliabilityPolicy.RELIABLE)],
    SHADOW_SOURCES[:2] + [SHADOW_SOURCES[0]],
    SHADOW_SOURCES[:2] + [
        ('/', 'behavior_server', 'geometry_msgs/msg/Twist',
         ReliabilityPolicy.BEST_EFFORT)],
])
def test_unexpected_shadow_endpoint_blocks_cancel(sources):
    core = ready_core()
    core.update_topology(1, 1, 1, sources, 0, 1, 1, True)
    payload, send = core.step(10.1)
    assert send is None
    assert payload['nav_shadow_fingerprint_ok'] is False
    assert 'SHADOW_MOTION_ISOLATION_NOT_PROVEN' in payload['blockers']


def test_spoofed_uuid_or_multiple_guard_blocks():
    core = ready_core()
    core.update_intent(intent(OTHER), 10.0)
    payload, send = core.step(10.1)
    assert send is None
    assert 'CANCEL_INTENT_GOAL_MISMATCH' in payload['blockers']
    core.update_intent(intent(), 10.1)
    core.update_topology(1, 1, 1, SHADOW_SOURCES, 0, 1, 2, True)
    payload, send = core.step(10.2)
    assert send is None
    assert 'MULTIPLE_OR_MISSING_CANCEL_GUARDS' in payload['blockers']


def test_lost_fire_from_old_session_cannot_cancel_new_goal():
    core = ready_core()
    stale_fire = intent()
    stale_fire['mission'] = {
        'state': 'TARGET_LOST', 'intent': 'REQUEST_SAFE_STOP'}
    core.update_intent(stale_fire, 10.0)
    payload, send = core.step(10.1)
    assert send is None
    assert 'CANCEL_INTENT_CONTRACT_INVALID' in payload['blockers']


def test_one_shot_even_with_repeated_intent():
    core = ready_core()
    payload, send = core.step(10.1)
    assert payload['state'] == 'REQUEST_SENT'
    assert send == UUID
    assert payload['send_count'] == 1
    assert payload['nav_shadow_endpoints'] == 3
    assert payload['nav_shadow_fingerprint_ok'] is True
    assert payload['raw_cmd_publishers'] == 0
    core.update_intent(intent(), 10.2)
    payload, send = core.step(10.2)
    assert send is None
    assert payload['send_count'] == 1


def test_wrong_cancel_response_fails_without_retry():
    core = ready_core()
    core.step(10.1)
    core.on_cancel_response(accepted_response(OTHER), 10.2)
    payload, send = core.step(10.3)
    assert payload['state'] == 'FAILED'
    assert send is None
    assert payload['send_count'] == 1


def test_accept_cancel_status_and_stationary_hold():
    core = ready_core()
    core.step(10.1)
    core.on_cancel_response(accepted_response(), 10.2)
    core.update_statuses([(UUID, 3)], 10.2)
    assert core.step(10.3)[0]['state'] == 'WAITING_FOR_CANCEL'
    core.update_statuses([(UUID, 5)], 10.4)
    for now in (10.4, 10.7, 10.95):
        core.update_arm(True, now)
        core.update_cmd(ZERO, now)
        core.update_odom(ZERO, now)
        payload, send = core.step(now)
        assert send is None
    assert payload['state'] == 'CANCEL_CONFIRMED_STOPPED'
    assert payload['cancel_service_accepted'] is True
    assert payload['cmd_vel_fresh_zero'] is True
    assert payload['odom_fresh_stationary'] is True


def test_terminal_not_canceled_fails():
    core = ready_core()
    core.step(10.1)
    core.on_cancel_response(accepted_response(), 10.2)
    core.update_statuses([(UUID, 4)], 10.2)
    core.update_arm(True, 10.2)
    assert core.step(10.3)[0]['state'] == 'FAILED'


def test_arm_loss_after_send_fails_without_another_request():
    core = ready_core()
    core.step(10.1)
    payload, send = core.step(10.5)
    assert send is None
    assert payload['state'] == 'FAILED'
    assert payload['blockers'] == ['OPERATOR_ARM_LOST']


def test_response_timeout_and_status_publisher_loss_fail():
    core = ready_core()
    core.step(10.1)
    core.update_arm(True, 12.2)
    assert core.step(12.2)[0]['state'] == 'FAILED'
    other = ready_core()
    other.step(10.1)
    other.update_topology(0, 1, 1, SHADOW_SOURCES, 0, 1, 1, True)
    other.update_arm(True, 10.2)
    assert other.step(10.2)[0]['state'] == 'FAILED'


def test_complete_state_rearms_for_distinct_goal_with_zero_motion():
    core = ready_core()
    core.step(10.1)
    core.on_cancel_response(accepted_response(), 10.2)
    core.update_statuses([(UUID, 5)], 10.4)
    for now in (10.4, 11.0):
        core.update_arm(True, now)
        core.update_cmd(ZERO, now)
        core.update_odom(ZERO, now)
        core.step(now)
    assert core.complete is True
    core.update_statuses([(UUID, 5), (OTHER, 2)], 11.1)
    core.update_arm(True, 11.1)
    core.update_cmd(ZERO, 11.1)
    core.update_odom(ZERO, 11.1)
    report, send = core.step(11.1)
    assert report['state'] == 'WAITING_FOR_ADMISSION'
    assert report['goal_uuid'] is None and send is None


def test_completed_cancel_survives_nav2_removing_terminal_status():
    core = ready_core()
    core.step(10.1)
    core.on_cancel_response(accepted_response(), 10.2)
    core.update_statuses([(UUID, 5)], 10.4)
    for now in (10.4, 11.0):
        core.update_arm(True, now)
        core.update_cmd(ZERO, now)
        core.update_odom(ZERO, now)
        core.step(now)
    core.update_statuses([], 11.1)
    core.update_arm(True, 11.1)
    core.update_cmd(ZERO, 11.1)
    core.update_odom(ZERO, 11.1)
    payload, send = core.step(11.1)
    assert send is None
    assert payload['state'] == 'CANCEL_CONFIRMED_STOPPED'
    assert payload['goal_status'] == 5  # Exact canceled goal is latched.
    assert payload['cancel_status_seen'] is True


def test_arm_publisher_must_be_unique():
    core = ready_core()
    core.update_topology(1, 1, 2, SHADOW_SOURCES, 0, 1, 1, True)
    payload, send = core.step(10.1)
    assert send is None
    assert 'OPERATOR_ARM_PUBLISHER_UNAVAILABLE' in payload['blockers']


def test_shadow_source_change_after_cancel_request_fails_closed():
    core = ready_core()
    assert core.step(10.1)[1] == UUID
    core.update_topology(1, 1, 1, SHADOW_SOURCES[:2], 0, 1, 1, True)
    core.update_arm(True, 10.2)
    payload, send = core.step(10.2)
    assert send is None
    assert payload['state'] == 'FAILED'
    assert payload['blockers'] == ['SHADOW_TOPOLOGY_LOST']
