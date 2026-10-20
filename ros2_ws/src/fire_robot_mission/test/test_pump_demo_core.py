"""The demo pump stays off until fresh proof for a distinct goal."""

from fire_robot_mission.pump_demo_core import PumpDemoCore


COUNTS = {key: 1 for key in (
    'ir', 'mission', 'alignment', 'gateway', 'guard', 'status',
    'odom', 'cmd', 'arm', 'pump',
    'pump_subscribers')}
COUNTS['raw'] = 0
ZERO = (0.0,) * 6


def ready_core(pulse_s=5.0):
    core = PumpDemoCore(
        enabled=True, allow_provisional_ir=True, pulse_s=pulse_s)
    core.update('ir', {
        'schema_version': 1, 'mode': 'OBSERVE_ONLY', 'observe_only': True,
        'zone': 'SPRAY_RANGE', 'blockers': [],
        'calibration': 'PROVISIONAL_SINGLE_FLAME',
        'validated_for_actuation': False,
        'safety': {'actuator_command_publisher': False,
                   'motion_command_publisher': False}}, 1.0)
    core.update('mission', {
        'schema_version': 1, 'observe_only': True,
        'state': 'ALIGNMENT_READY',
        'perception': {'detected_now': True, 'confirmed': True,
                       'alignment_state': 'CENTERED'},
        'navigation': {'goal_active': False, 'goal_state': 'CANCELED'},
        'robot': {'odom_fresh': True, 'stationary': True},
        'health': {'status': 'OK', 'blockers': []},
        'decision': {'eligible_for_future_alignment_handoff': True,
                     'readiness_blockers': []},
        'safety': {'motion_command_publisher': False,
                   'actuator_command_publisher': False,
                   'collision_check_bypassed': False}}, 1.0)
    core.update('alignment', {
        'schema_version': 1, 'state': 'COMPLETE',
        'enabled': True, 'one_shot': True,
        'stop_reason': 'TARGET_CENTERED', 'publisher_owned': False,
        'goal_uuid': 'a' * 32,
        'command': {'angular_z_rad_s': 0.0},
        'safety': {'pump_command_publisher': False,
                   'linear_motion': False}}, 1.0)
    core.update('gateway', {
        'schema_version': 1, 'state': 'HANDOFF_READY', 'enabled': True,
        'fire_latched': True, 'ever_forwarded': True, 'released': True,
        'stop_reason': 'FIRE_ACQUIRED', 'handoff_ready': True,
        'raw_publishers': 0, 'nav_shadow_fingerprint_ok': True,
        'safety_gate_fingerprint_ok': True,
        'cmd_vel_fresh_zero': True, 'odom_stationary': True,
        'goal_active': False,
        'safety': {'raw_command_publisher': False}}, 1.0)
    core.update('guard', {
        'schema_version': 1, 'state': 'CANCEL_CONFIRMED_STOPPED',
        'enabled': True, 'goal_uuid': 'a' * 32, 'send_count': 1,
        'cancel_service_accepted': True, 'cancel_status_seen': True,
        'goal_status': 5, 'cmd_vel_fresh_zero': True,
        'odom_fresh_stationary': True, 'arm_fresh': True,
        'nav_shadow_fingerprint_ok': True, 'raw_cmd_publishers': 0,
        'blockers': [],
        'safety': {'motion_command_publisher': False,
                   'actuator_command_publisher': False}}, 1.0)
    core.update_statuses([('a' * 32, 5, 100, 0)], 1.0)
    core.update_arm(False, 0.9)
    core.update_arm(True, 1.0)
    core.update_odom(ZERO, 1.0)
    core.update_cmd(ZERO, 1.0)
    return core


def refresh(core, now):
    for name in ('ir', 'mission', 'alignment', 'gateway', 'guard'):
        core.update(name, core.inputs[name][0], now)
    core.update_arm(True, now)
    core.update_odom(ZERO, now)
    core.update_cmd(ZERO, now)


def test_disabled_and_provisional_requires_explicit_override():
    core = PumpDemoCore()
    assert core.step(1.0, COUNTS)[0] is False
    core = PumpDemoCore(enabled=True)
    assert core.step(1.0, COUNTS)[0] is False


def test_stale_high_arm_after_restart_is_not_permission():
    core = PumpDemoCore(enabled=True, allow_provisional_ir=True)
    core.update_arm(True, 1.0)
    assert 'OPERATOR_ARM_MISSING_OR_STALE' in core.blockers(1.0, COUNTS)
    core.update_arm(False, 1.1)
    core.update_arm(True, 1.2)
    assert 'OPERATOR_ARM_MISSING_OR_STALE' not in core.blockers(1.2, COUNTS)


def test_pruned_empty_nav_status_uses_fresh_exact_guard_but_no_active_goal():
    core = ready_core()
    core.update_statuses([], 1.0)
    assert 'EXACT_CANCEL_PROOF_INVALID' not in core.blockers(1.0, COUNTS)
    assert core.step(1.0, COUNTS)[0] is False
    refresh(core, 1.5)
    core.update_statuses([], 1.5)
    assert core.step(1.5, COUNTS)[0] is True
    core.update_statuses([('b' * 32, 2, 101, 0)], 1.6)
    refresh(core, 1.6)
    on, report = core.step(1.6, COUNTS)
    assert on is False and report['state'] == 'ABORTED'


def test_requires_stable_ready_then_one_bounded_pulse():
    core = ready_core()
    assert core.pulse_s == 5.0
    assert core.step(1.0, COUNTS)[0] is False
    refresh(core, 1.3)
    assert core.step(1.3, COUNTS)[0] is False
    refresh(core, 1.41)
    assert core.step(1.41, COUNTS)[0] is True
    refresh(core, 6.40)
    assert core.step(6.40, COUNTS)[0] is True
    refresh(core, 6.42)
    assert core.step(6.42, COUNTS)[0] is False
    assert core.state == 'COMPLETE'
    assert core.goal_uuid == 'a' * 32
    refresh(core, 7.0)
    assert core.step(7.0, COUNTS)[0] is False


def test_completed_pulse_rearms_off_for_new_active_goal_only():
    core = ready_core()
    core.step(1.0, COUNTS)
    refresh(core, 1.41)
    assert core.step(1.41, COUNTS)[0] is True
    refresh(core, 6.42)
    assert core.step(6.42, COUNTS)[0] is False
    assert core.state == 'COMPLETE'
    core.update_statuses([('a' * 32, 5, 100, 0),
                          ('b' * 32, 2, 101, 0)], 6.5)
    on, report = core.step(6.51, COUNTS)
    assert on is False
    assert report['state'] == 'WAITING_FOR_ADMISSION'
    assert report['goal_uuid'] is None
    assert 'a' * 32 in core.used_goal_uuids


def test_aborted_pulse_cannot_rearm_for_new_goal():
    core = ready_core()
    core.state = 'ABORTED'
    core.goal_uuid = 'a' * 32
    core.update_statuses([('a' * 32, 5, 100, 0),
                          ('b' * 32, 2, 101, 0)], 2.0)
    assert core.step(2.01, COUNTS)[1]['state'] == 'ABORTED'


def test_early_fire_loss_can_rearm_only_for_distinct_goal():
    core = ready_core()
    core.step(1.0, COUNTS)
    refresh(core, 1.41)
    assert core.step(1.41, COUNTS)[0] is True
    refresh(core, 1.5)
    ir = dict(core.inputs['ir'][0])
    ir['zone'] = 'UNKNOWN'
    core.update('ir', ir, 1.5)
    on, report = core.step(1.5, COUNTS)
    assert not on and report['state'] == 'ABORTED'
    assert report['stop_reason'] == 'IR_NOT_CONFIRMED_SPRAY_RANGE'
    refresh(core, 1.6)
    assert core.step(1.6, COUNTS)[1]['state'] == 'ABORTED'
    core.update_statuses([('a' * 32, 5, 100, 0),
                          ('b' * 32, 2, 101, 0)], 1.7)
    on, report = core.step(1.71, COUNTS)
    assert not on and report['state'] == 'WAITING_FOR_ADMISSION'
    assert report['goal_uuid'] is None


def test_shorter_pulse_can_be_selected_without_changing_safety_cap():
    core = ready_core(pulse_s=1.0)
    core.step(1.0, COUNTS)
    refresh(core, 1.41)
    assert core.step(1.41, COUNTS)[0] is True
    refresh(core, 2.40)
    assert core.step(2.40, COUNTS)[0] is True
    refresh(core, 2.42)
    assert core.step(2.42, COUNTS)[0] is False
    assert core.stop_reason == 'PULSE_TIME_LIMIT'


def test_unknown_during_pulse_aborts_without_auto_restart():
    core = ready_core()
    core.step(1.0, COUNTS)
    refresh(core, 1.41)
    assert core.step(1.41, COUNTS)[0] is True
    refresh(core, 1.51)
    ir = dict(core.inputs['ir'][0])
    ir['zone'] = 'UNKNOWN'
    core.update('ir', ir, 1.51)
    on, report = core.step(1.51, COUNTS)
    assert on is False
    assert report['state'] == 'ABORTED'
    ir['zone'] = 'SPRAY_RANGE'
    core.update('ir', ir, 1.6)
    refresh(core, 1.6)
    assert core.step(1.6, COUNTS)[0] is False


def test_missing_or_stale_live_cancel_proof_never_starts_pump():
    for key, reason in (
            ('gateway', 'GATEWAY_HANDOFF_PROOF_INVALID'),
            ('guard', 'CANCEL_GUARD_PROOF_INVALID')):
        core = ready_core()
        core.inputs.pop(key)
        assert reason in core.blockers(1.0, COUNTS)
        core = ready_core()
        core.update(key, core.inputs[key][0], 0.5)
        assert reason in core.blockers(1.0, COUNTS)
    core = ready_core()
    counts = COUNTS.copy()
    counts['guard'] = 0
    assert 'GUARD_PUBLISHER_INVALID' in core.blockers(1.0, counts)


def test_guard_mismatch_or_gateway_release_loss_aborts_pulse():
    for key, field, value in (
            ('guard', 'goal_uuid', 'b' * 32),
            ('guard', 'cancel_status_seen', False),
            ('gateway', 'handoff_ready', False),
            ('gateway', 'raw_publishers', 1)):
        core = ready_core()
        core.step(1.0, COUNTS)
        refresh(core, 1.41)
        assert core.step(1.41, COUNTS)[0] is True
        refresh(core, 1.5)
        payload = dict(core.inputs[key][0])
        payload[field] = value
        core.update(key, payload, 1.5)
        on, report = core.step(1.5, COUNTS)
        assert on is False
        assert report['state'] == 'ABORTED'
        refresh(core, 1.6)
        assert core.step(1.6, COUNTS)[0] is False


def test_motion_or_arm_loss_or_owner_conflict_stops():
    for case in ('odom', 'cmd', 'arm', 'owner', 'raw'):
        core = ready_core()
        core.step(1.0, COUNTS)
        refresh(core, 1.41)
        assert core.step(1.41, COUNTS)[0] is True
        refresh(core, 1.5)
        counts = COUNTS.copy()
        if case == 'odom':
            core.update_odom((0.1, 0, 0, 0, 0, 0), 1.5)
        elif case == 'cmd':
            core.update_cmd((0, 0, 0, 0, 0, 0.1), 1.5)
        elif case == 'arm':
            core.update_arm(False, 1.5)
        elif case == 'owner':
            counts['pump'] = 2
        else:
            counts['raw'] = 1
        assert core.step(1.5, counts)[0] is False
        assert core.state == 'ABORTED'


def test_stale_vision_and_pump_limit_validation():
    core = ready_core()
    assert core.step(1.0, COUNTS)[0] is False
    assert core.step(2.0, COUNTS)[0] is False
    assert 'IR_NOT_CONFIRMED_SPRAY_RANGE' in core.blockers(2.0, COUNTS)
    try:
        PumpDemoCore(enabled=True, pulse_s=5.1)
        assert False
    except ValueError:
        pass


def test_adjacent_ir_bands_do_not_pump_and_cancel_uuid_must_match():
    for zone in ('TOO_CLOSE', 'APPROACH', 'TOO_FAR', 'UNKNOWN'):
        core = ready_core()
        ir = dict(core.inputs['ir'][0])
        ir['zone'] = zone
        core.update('ir', ir, 1.0)
        core.step(1.0, COUNTS)
        refresh(core, 1.41)
        assert core.step(1.41, COUNTS)[0] is False
    core = ready_core()
    core.update_statuses([('b' * 32, 5, 101, 0)], 1.0)
    assert 'EXACT_CANCEL_PROOF_INVALID' in core.blockers(1.0, COUNTS)


def test_old_canceled_goal_cannot_authorize_pump_after_new_goal():
    core = ready_core()
    core.update_statuses([
        ('a' * 32, 5, 100, 0),
        ('b' * 32, 5, 101, 0),
    ], 1.0)
    assert 'EXACT_CANCEL_PROOF_INVALID' in core.blockers(1.0, COUNTS)
    core.update_statuses([
        ('a' * 32, 5, 100, 0),
        ('b' * 32, 2, 101, 0),
    ], 1.0)
    assert 'EXACT_CANCEL_PROOF_INVALID' in core.blockers(1.0, COUNTS)
    assert 'EXACT_CANCEL_PROOF_INVALID' in core.blockers(17.0, COUNTS)


def test_event_driven_canceled_status_remains_valid_with_fresh_live_proof():
    core = ready_core()
    refresh(core, 30.0)
    assert core.status_at == 1.0
    assert 'EXACT_CANCEL_PROOF_INVALID' not in core.blockers(30.0, COUNTS)
    core.step(30.0, COUNTS)
    refresh(core, 30.41)
    assert core.step(30.41, COUNTS)[0] is True


def test_rejected_before_hold_resets_stability_clock():
    core = ready_core()
    core.step(1.0, COUNTS)
    refresh(core, 1.2)
    core.update_arm(False, 1.2)
    assert core.step(1.2, COUNTS)[0] is False
    refresh(core, 1.3)
    assert core.step(1.3, COUNTS)[0] is False
    refresh(core, 1.69)
    assert core.step(1.69, COUNTS)[0] is False
    refresh(core, 1.71)
    assert core.step(1.71, COUNTS)[0] is True
