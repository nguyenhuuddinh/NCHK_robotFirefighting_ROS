"""One-shot IR motion admission, termination and output safety tests."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from fire_robot_mission.ir_approach_live_core import (
    IrApproachLiveCore, VerifiedEndpointPins, provisional_spray_candidate,
    realign_candidate, target_reacquire_candidate,
    verified_raw_gate,
)


ROOT = Path(__file__).resolve().parents[1]
COUNTS = {
    'ir': 1, 'scan': 1, 'odom': 1, 'cmd': 1, 'arm': 1,
    'raw': 0, 'raw_owned': False, 'raw_gate': True,
    'ir_owner': True, 'scan_owner': True, 'odom_owner': True,
    'safety_owner': True, 'nav': 0, 'nav_cmd': 0, 'pump': 0,
}


def ir(band='APPROACH'):
    return {
        'schema_version': 1, 'mode': 'OBSERVE_ONLY',
        'observe_only': True, 'zone': 'UNKNOWN', 'ir_band': band,
        'ir_receive_age_ms': 20.0, 'stable_samples': 5,
        'ir_sample_seq': 10,
        'filtered_counts': {'left': 14000, 'center': 19000,
                            'right': 1650},
        'blockers': ['FIRE_NOT_CENTERED'],
        'calibration': 'PROVISIONAL_SINGLE_FLAME',
        'validated_for_actuation': False,
        'safety': {'motion_command_publisher': False,
                   'actuator_command_publisher': False},
    }


def centered_ir(band='APPROACH'):
    payload = ir(band)
    payload['zone'] = band
    payload['blockers'] = []
    return payload


def scan(front=1.0, rear=1.0):
    ranges = [2.0] * 400
    for index in range(400):
        angle = index * 2 * math.pi / 400
        if abs(math.atan2(math.sin(angle), math.cos(angle))) <= 0.35:
            ranges[index] = front
        if abs(math.atan2(math.sin(angle - math.pi),
                          math.cos(angle - math.pi))) <= 0.35:
            ranges[index] = rear
    return (0.0, 2 * math.pi / 400, 0.08, 8.0, tuple(ranges))


def odom(x=0.0, vx=0.0):
    return ((x, 0.0), (vx, 0.0, 0.0, 0.0, 0.0, 0.0), 'odom')


def ready(band='APPROACH', *, enabled=True, override=True,
          reverse=False):
    core = IrApproachLiveCore(
        enabled=enabled, allow_provisional_ir=override,
        enable_reverse_jog=reverse)
    core.update('ir', ir(band), 0.1)
    core.update('scan', scan(), 0.1)
    core.update('odom', odom(), 0.1)
    core.update('cmd', (0.0,) * 6, 0.1)
    core.update_arm(False, 0.0)
    core.update_arm(True, 0.1)
    return core


def refresh(core, now, *, x=0.0, cmd=0.0, band='APPROACH'):
    payload = (centered_ir(band) if core.integrated_handoff else ir(band))
    core.update('ir', payload, now)
    core.update('scan', scan(), now)
    core.update('odom', odom(x, cmd), now)
    core.update('cmd', (cmd, 0.0, 0.0, 0.0, 0.0, 0.0), now)
    core.update_arm(True, now)


def admit(core, band='APPROACH'):
    assert core.step(0.2, COUNTS, False)[2] is None
    refresh(core, 0.5, band=band)
    assert core.step(0.51, COUNTS, False)[2] == 'CREATE'


def test_default_off_and_unvalidated_override_off_never_claims_raw():
    for enabled, override in ((False, False), (True, False)):
        core = ready(enabled=enabled, override=override)
        speed, state, action = core.step(0.2, COUNTS, False)
        assert speed == 0.0 and action is None
        assert state['state'] in ('DISABLED', 'WAITING_FOR_ADMISSION')


@pytest.mark.parametrize('band', ['SPRAY_RANGE', 'UNKNOWN'])
def test_spray_or_unknown_never_moves(band):
    core = ready(band)
    speed, state, action = core.step(0.2, COUNTS, False)
    assert (speed, action) == (0.0, None)
    assert 'IR_BAND_HOLD' in state['blockers']


def test_forward_one_step_claims_and_releases_only_after_zero_hold():
    core = ready()
    admit(core)
    assert core.step(0.55, COUNTS, True)[0] == 0.0
    owned = dict(COUNTS, raw=1, raw_owned=True)
    speed, state, action = core.step(0.6, owned, True)
    assert speed == 0.05 and state['state'] == 'MOVING'
    assert action is None
    refresh(core, 0.8, x=0.05, cmd=0.05)
    speed, state, _ = core.step(0.81, owned, True)
    assert speed == 0.0 and state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'STEP_DISTANCE_REACHED'
    core.update('odom', odom(0.05), 0.9)
    core.update('cmd', (0.0,) * 6, 0.9)
    assert core.step(0.91, owned, True)[2] is None
    core.update('odom', odom(0.05), 1.42)
    core.update('cmd', (0.0,) * 6, 1.42)
    speed, state, action = core.step(1.43, owned, True)
    assert speed == 0.0 and action == 'RELEASE'
    assert state['state'] == 'COMPLETE'
    assert core.step(1.5, dict(COUNTS), False)[2] is None


def test_reverse_requires_rear_scan_clearance():
    core = ready('TOO_CLOSE', reverse=True)
    core.update('scan', scan(rear=0.3), 0.1)
    assert core.step(0.2, COUNTS, False)[2] is None
    core.update('scan', scan(rear=1.0), 0.2)
    assert core.step(0.21, COUNTS, False)[2] is None
    refresh(core, 0.52, band='TOO_CLOSE')
    assert core.step(0.53, COUNTS, False)[2] == 'CREATE'
    owned = dict(COUNTS, raw=1, raw_owned=True)
    speed, state, _ = core.step(0.54, owned, True)
    assert speed == -0.05
    assert state['direction'] == 'REVERSE'


def test_reverse_is_separately_disabled_by_default():
    core = ready('TOO_CLOSE')
    speed, state, action = core.step(0.2, COUNTS, False)
    assert speed == 0.0 and action is None
    assert 'REVERSE_JOG_DISABLED' in state['blockers']


def test_arm_drop_ir_loss_and_raw_owner_conflict_stop():
    for change in ('arm', 'ir', 'raw'):
        core = ready()
        admit(core)
        owned = dict(COUNTS, raw=1, raw_owned=True)
        core.step(0.6, owned, True)
        refresh(core, 0.7, cmd=0.05)
        if change == 'arm':
            core.update_arm(False, 0.7)
        elif change == 'ir':
            core.update('ir', None, 0.7)
        else:
            owned['raw'] = 2
            owned['raw_owned'] = False
        speed, state, _ = core.step(0.71, owned, True)
        assert speed == 0.0
        assert state['state'] == 'STOPPING'
        assert state['stop_reason']


def test_nav2_or_pump_owner_blocks_admission():
    for name in ('nav', 'pump', 'raw'):
        core = ready()
        counts = dict(COUNTS, **{name: 1})
        assert core.step(0.2, counts, False)[2] is None


def integrated_counts(**changes):
    values = dict(COUNTS, nav=1, nav_cmd=1,
                  nav_shadow_ok=True, status_source_ok=True,
                  goal_uuid='ab' * 16, handoff_ok=True,
                  runtime_ok=True, guard_owner_ack=True)
    values.update(changes)
    return values


def continuous_core():
    core = IrApproachLiveCore(
        enabled=True, allow_provisional_ir=True,
        integrated_handoff=True, continuous_approach=True,
        speed_m_s=0.08)
    core.update('ir', centered_ir(), 0.1)
    core.update('scan', scan(), 0.1)
    core.update('odom', odom(), 0.1)
    core.update('cmd', (0.0,) * 6, 0.1)
    core.update_arm(False, 0.0)
    core.update_arm(True, 0.1)
    return core


def moving_continuous():
    core = continuous_core()
    assert core.step(0.2, integrated_counts(), False)[2] is None
    refresh(core, 0.51)
    assert core.step(0.52, integrated_counts(), False)[2] == 'CREATE'
    owned = integrated_counts(raw=1, raw_owned=True)
    assert core.step(0.6, owned, True)[0] == 0.08
    return core, owned


def spray_candidate():
    payload = ir('SPRAY_RANGE')
    payload['stable_samples'] = 1
    payload['blockers'] = ['IR_BAND_NOT_STABLE']
    payload['filtered_counts']['right'] = 1020
    return payload


def confirmed_spray():
    payload = spray_candidate()
    payload['zone'] = 'SPRAY_RANGE'
    payload['stable_samples'] = 5
    payload['blockers'] = []
    return payload


def test_continuous_approach_moves_without_intermediate_step_release():
    core, owned = moving_continuous()
    for now, x in ((0.8, 0.012), (1.0, 0.028), (1.2, 0.045)):
        refresh(core, now, x=x, cmd=0.08)
        speed, state, action = core.step(now + 0.01, owned, True)
        assert speed == 0.08 and state['state'] == 'MOVING'
        assert state['step_count'] == 0 and action is None
        assert state['continuous_approach'] is True


def test_continuous_approach_can_run_past_old_time_cap_with_fresh_progress():
    core, owned = moving_continuous()
    for index in range(1, 18):
        now = 0.6 + index * 0.4
        refresh(core, now, x=index * 0.012, cmd=0.08)
        speed, state, action = core.step(now + 0.01, owned, True)
        assert speed == 0.08 and action is None
        assert state['state'] == 'MOVING'
    assert now > 5.0


def test_first_spray_candidate_stops_then_requires_stationary_confirmation():
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.015, cmd=0.08)
    core.update('ir', spray_candidate(), 0.8)
    speed, state, _ = core.step(
        0.81, integrated_counts(raw=1, raw_owned=True,
                                runtime_ok=False), True)
    assert speed == 0 and state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'SPRAY_CANDIDATE_STOP'
    core.update('odom', odom(0.02), 0.9)
    core.update('cmd', (0.0,) * 6, 0.9)
    core.step(0.91, owned, True)
    core.update('ir', confirmed_spray(), 1.42)
    core.update('odom', odom(0.02), 1.42)
    core.update('cmd', (0.0,) * 6, 1.42)
    _, state, action = core.step(1.43, owned, True)
    assert state['state'] == 'COMPLETE' and action == 'RELEASE'
    assert state['stop_reason'] == 'SPRAY_RANGE_REACHED'
    assert state['total_progress_m'] == 0.02
    assert core.step(1.44, integrated_counts(), False)[1][
        'state'] == 'COMPLETE'


def test_transitional_or_untrusted_ir_never_authorizes_a_pump():
    payload = spray_candidate()
    assert provisional_spray_candidate(payload)
    for edit in (
            {'blockers': ['FIRE_NOT_CENTERED']},
            {'blockers': ['IR_SAMPLE_INVALID']},
            {'ir_receive_age_ms': 501},
            {'validated_for_actuation': True}):
        bad = dict(payload, **edit)
        assert not provisional_spray_candidate(bad)
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.015, cmd=0.08)
    core.update('ir', payload, 0.8)
    core.step(0.81, integrated_counts(raw=1, raw_owned=True,
                                      runtime_ok=False), True)
    core.update('odom', odom(0.02), 0.9)
    core.update('cmd', (0.0,) * 6, 0.9)
    core.step(0.91, owned, True)
    core.update('ir', payload, 2.31)
    core.update('odom', odom(0.02), 2.31)
    core.update('cmd', (0.0,) * 6, 2.31)
    _, state, action = core.step(2.32, owned, True)
    assert state['state'] == 'COMPLETE' and action == 'RELEASE'
    assert state['stop_reason'] == 'REALIGN_REQUIRED'
    assert 'SPRAY_NOT_CONFIRMED' in state['stop_trigger_blockers']
    assert state['recovery_ready'] is False


@pytest.mark.parametrize('fault,reason', [
    ('distance', 'APPROACH_DISTANCE_LIMIT'),
    ('no_progress', 'ODOM_PROGRESS_LOST'),
    ('arm', 'OPERATOR_ARM_MISSING_OR_STALE'),
    ('scan', 'SCAN_OBSTACLE_OR_INSUFFICIENT_MARGIN'),
])
def test_continuous_approach_fails_closed_on_limit_or_lost_proof(fault, reason):
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.01, cmd=0.08)
    if fault == 'distance':
        core.update('odom', odom(0.30, 0.08), 0.8)
    elif fault == 'no_progress':
        refresh(core, 1.6, x=0.0, cmd=0.08)
    elif fault == 'arm':
        core.update_arm(False, 0.8)
    else:
        core.update('scan', scan(front=0.52), 0.8)
    now = 1.61 if fault == 'no_progress' else 0.81
    speed, state, _ = core.step(now, owned, True)
    assert speed == 0.0 and state['state'] == 'STOPPING'
    assert state['stop_reason'] == reason


def test_lost_ir_trigger_is_latched_after_raw_owner_releases():
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.01, cmd=0.08)
    core.update('ir', None, 0.8)
    speed, state, _ = core.step(
        0.81, integrated_counts(raw=1, raw_owned=True,
                                runtime_ok=False), True)
    assert speed == 0.0 and state['state'] == 'STOPPING'
    assert 'IR_BAND_INVALID_OR_STALE' in state['stop_trigger_blockers']
    core.update('odom', odom(0.01), 0.9)
    core.update('cmd', (0.0,) * 6, 0.9)
    core.step(0.91, owned, True)
    core.update('odom', odom(0.01), 1.42)
    core.update('cmd', (0.0,) * 6, 1.42)
    _, state, action = core.step(1.43, owned, True)
    assert state['state'] == 'COMPLETE' and action == 'RELEASE'
    assert state['stop_reason'] == 'REALIGN_REQUIRED'
    assert 'IR_BAND_INVALID_OR_STALE' in state['stop_trigger_blockers']


def test_stale_ir_waits_for_two_new_samples_and_same_goal_handoff():
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.01, cmd=0.08)
    old = ir()
    old['ir_receive_age_ms'] = 590.0
    core.update('ir', old, 0.8)
    speed, state, _ = core.step(0.81, owned, True)
    assert speed == 0.0 and state['stop_reason'] == 'REALIGN_REQUIRED'
    for now in (0.9, 1.42):
        core.update('odom', odom(0.02), now)
        core.update('cmd', (0.0,) * 6, now)
        _, state, action = core.step(now + 0.01, owned, True)
    assert state['state'] == 'COMPLETE' and action == 'RELEASE'
    assert state['total_progress_m'] == 0.02
    assert core.step(1.44, integrated_counts(), False)[1][
        'recovery_ready'] is False
    fresh = dict(ir(), zone='APPROACH', blockers=[], ir_sample_seq=11)
    core.update('ir', fresh, 1.5)
    assert core.step(1.51, integrated_counts(), False)[1][
        'state'] == 'COMPLETE'
    core.update('ir', dict(fresh, ir_sample_seq=12), 1.7)
    _, state, action = core.step(
        1.71, integrated_counts(handoff_ok=False), False)
    assert state['state'] == 'COMPLETE' and action is None
    assert state['recovery_ready'] is True
    assert core.step(1.72, integrated_counts(
        goal_uuid='cd' * 16, handoff_ok=False), False)[1][
            'state'] == 'COMPLETE'
    _, state, action = core.step(1.73, integrated_counts(), False)
    assert state['state'] == 'WAITING_FOR_ADMISSION'
    assert state['total_progress_m'] == 0.02 and action is None


def test_fresh_ir_off_center_can_recenter_but_cannot_resume_forward():
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.01, cmd=0.08)
    core.update('ir', dict(ir(), ir_receive_age_ms=590.0), 0.8)
    core.step(0.81, owned, True)
    for now in (0.9, 1.42):
        core.update('odom', odom(0.02), now)
        core.update('cmd', (0.0,) * 6, now)
        _, state, action = core.step(now + 0.01, owned, True)
    assert state['state'] == 'COMPLETE' and action == 'RELEASE'
    core.update('ir', dict(ir(), ir_sample_seq=11), 1.5)
    assert core.step(1.51, integrated_counts(handoff_ok=False), False)[1][
        'recovery_ready'] is False
    core.update('ir', dict(ir(), ir_sample_seq=12), 1.7)
    _, state, action = core.step(
        1.71, integrated_counts(handoff_ok=False), False)
    assert state['recovery_ready'] is True
    assert state['state'] == 'COMPLETE' and action is None
    # A valid IR-only band with an off-center vision zone cannot rearm wheels.
    _, state, action = core.step(1.72, integrated_counts(), False)
    assert state['state'] == 'COMPLETE' and action is None


def test_two_new_ir_samples_are_not_enough_if_band_is_unstable():
    core = continuous_core()
    core.phase = 'COMPLETE'
    core.goal_uuid = 'ab' * 16
    core.stop_reason = 'REALIGN_REQUIRED'
    core.sensor_recovery_active = True
    core.recovery_sample_seq = 10
    unstable = dict(ir(), ir_sample_seq=12, stable_samples=2,
                    blockers=['IR_BAND_NOT_STABLE'])
    core.update('ir', unstable, 1.0)
    _, state, action = core.step(1.01, integrated_counts(), False)
    assert state['state'] == 'COMPLETE' and action is None
    assert state['recovery_ready'] is False
    core.update('ir', dict(ir(), ir_sample_seq=13), 1.2)
    _, state, action = core.step(1.21, integrated_counts(
        handoff_ok=False), False)
    assert state['state'] == 'COMPLETE' and action is None
    assert state['recovery_ready'] is True
    _, state, action = core.step(1.22, integrated_counts(), False)
    assert state['state'] == 'COMPLETE' and action is None


def test_ir_sample_counter_restart_needs_two_new_samples_in_new_epoch():
    core = continuous_core()
    core.phase = 'COMPLETE'
    core.goal_uuid = 'ab' * 16
    core.stop_reason = 'REALIGN_REQUIRED'
    core.sensor_recovery_active = True
    core.recovery_sample_seq = 10000
    for seq, now in ((1, 1.0), (2, 1.2)):
        core.update('ir', dict(ir(), ir_sample_seq=seq), now)
        _, state, action = core.step(now + 0.01, integrated_counts(), False)
        assert state['state'] == 'COMPLETE' and action is None
        assert state['recovery_ready'] is False
    core.update('ir', dict(ir(), ir_sample_seq=3), 1.4)
    _, state, action = core.step(1.41, integrated_counts(
        handoff_ok=False), False)
    assert state['recovery_ready'] is True
    assert state['state'] == 'COMPLETE' and action is None


def test_simultaneous_camera_candidate_and_ir_stale_stays_recoverable():
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.01, cmd=0.08)
    stale = dict(ir(), ir_receive_age_ms=590.0)
    core.update('ir', stale, 0.8)
    counts = integrated_counts(raw=1, raw_owned=True, runtime_ok=False)
    speed, state, _ = core.step(0.81, counts, True)
    assert speed == 0.0 and state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'REALIGN_REQUIRED'
    assert state['recovery_required'] is True
    for now in (0.9, 1.42):
        core.update('odom', odom(0.02), now)
        core.update('cmd', (0.0,) * 6, now)
        _, state, action = core.step(now + 0.01, counts, True)
    assert state['state'] == 'COMPLETE' and action == 'RELEASE'
    assert state['total_progress_m'] == 0.02


def test_persistent_ir_loss_waits_stopped_without_timer_rearming_motion():
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.01, cmd=0.08)
    core.update('ir', dict(ir(), ir_receive_age_ms=590.0), 0.8)
    core.step(0.81, owned, True)
    for now in (0.9, 1.42):
        core.update('odom', odom(0.02), now)
        core.update('cmd', (0.0,) * 6, now)
        _, state, action = core.step(now + 0.01, owned, True)
    assert state['state'] == 'COMPLETE' and action == 'RELEASE'
    for now in (10.0, 40.0, 120.0):
        speed, state, action = core.step(now, integrated_counts(), False)
        assert speed == 0.0 and action is None
        assert state['state'] == 'COMPLETE'
        assert state['recovery_ready'] is False
        assert state['safety']['pump_command_publisher'] is False


def test_ir_stale_while_acquiring_releases_without_moving_then_recovers():
    core = continuous_core()
    core.step(0.2, integrated_counts(), False)
    refresh(core, 0.51)
    assert core.step(0.52, integrated_counts(), False)[2] == 'CREATE'
    stale = dict(ir(), ir_receive_age_ms=590.0)
    core.update('ir', stale, 0.55)
    owned = integrated_counts(raw=1, raw_owned=True, runtime_ok=False)
    speed, state, _ = core.step(0.56, owned, True)
    assert speed == 0.0 and state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'REALIGN_REQUIRED'
    assert state['total_progress_m'] == 0.0
    for now in (0.65, 1.17):
        core.update('odom', odom(), now)
        core.update('cmd', (0.0,) * 6, now)
        _, state, action = core.step(now + 0.01, owned, True)
    assert state['state'] == 'COMPLETE' and action == 'RELEASE'
    assert state['total_progress_m'] == 0.0


@pytest.mark.parametrize('unsafe_counts', [
    {'pump': 1},
    {'raw': 2, 'raw_owned': False},
    {'goal_uuid': 'cd' * 16, 'runtime_ok': False},
])
def test_stale_ir_with_other_safety_fault_must_not_auto_recover(
        unsafe_counts):
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.01, cmd=0.08)
    core.update('ir', dict(ir(), ir_receive_age_ms=590.0), 0.8)
    counts = dict(owned, **unsafe_counts)
    speed, state, _ = core.step(0.81, counts, True)
    assert speed == 0.0 and state['state'] == 'STOPPING'
    assert state['stop_reason'] != 'REALIGN_REQUIRED'
    for now in (0.9, 1.42):
        core.update('odom', odom(0.02), now)
        core.update('cmd', (0.0,) * 6, now)
        _, state, action = core.step(now + 0.01, owned, True)
    assert state['state'] == 'ABORTED' and action == 'RELEASE'
    core.update('ir', centered_ir(), 1.5)
    _, state, action = core.step(1.51, integrated_counts(), False)
    assert state['state'] == 'ABORTED' and action is None


def test_unconfirmed_spray_recovers_only_after_new_samples_not_pump():
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.02, cmd=0.08)
    core.update('ir', spray_candidate(), 0.8)
    assert core.step(0.81, owned, True)[1][
        'stop_reason'] == 'SPRAY_CANDIDATE_STOP'
    for now in (0.9, 2.31):
        core.update('odom', odom(0.02), now)
        core.update('cmd', (0.0,) * 6, now)
        core.update('ir', spray_candidate(), now)
        _, state, action = core.step(now + 0.01, owned, True)
    assert state['state'] == 'COMPLETE' and action == 'RELEASE'
    assert state['stop_reason'] == 'REALIGN_REQUIRED'
    assert state['recovery_ready'] is False
    core.update('ir', dict(ir(), zone='APPROACH', blockers=[],
                           ir_sample_seq=11), 2.4)
    assert core.step(2.41, integrated_counts(), False)[1][
        'state'] == 'COMPLETE'
    core.update('ir', dict(ir(), zone='APPROACH', blockers=[],
                           ir_sample_seq=12), 2.6)
    _, state, action = core.step(2.61, integrated_counts(
        handoff_ok=False), False)
    assert state['recovery_ready'] is True and action is None
    assert core.step(2.62, integrated_counts(), False)[1][
        'state'] == 'WAITING_FOR_ADMISSION'


@pytest.mark.parametrize('stopped_x', [0.02, 0.30])
def test_off_center_fire_stops_forward_and_realigns_only_after_handoff(
        stopped_x):
    core, owned = moving_continuous()
    refresh(core, 0.8, x=stopped_x, cmd=0.08)
    shifted = ir()
    assert realign_candidate(shifted)
    core.update('ir', shifted, 0.8)
    speed, state, _ = core.step(
        0.81, integrated_counts(raw=1, raw_owned=True,
                                runtime_ok=False), True)
    assert speed == 0.0 and state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'REALIGN_REQUIRED'
    core.update('odom', odom(stopped_x), 0.9)
    core.update('cmd', (0.0,) * 6, 0.9)
    core.step(0.91, owned, True)
    core.update('odom', odom(stopped_x), 1.42)
    core.update('cmd', (0.0,) * 6, 1.42)
    _, state, action = core.step(1.43, owned, True)
    assert state['state'] == 'COMPLETE' and action == 'RELEASE'
    assert state['stop_reason'] == 'REALIGN_REQUIRED'
    assert state['total_progress_m'] == stopped_x
    assert core.step(1.44, integrated_counts(handoff_ok=False), False)[1][
        'state'] == 'COMPLETE'
    core.update('ir', dict(ir(), zone='APPROACH', blockers=[]), 1.50)
    _, state, action = core.step(
        1.51, integrated_counts(), False)
    assert state['state'] == 'WAITING_FOR_ADMISSION'
    assert action is None and state['total_progress_m'] == stopped_x


def test_fresh_unconfirmed_target_can_request_safe_realign():
    for edit in (
            {'blockers': ['FIRE_NOT_CONFIRMED', 'FIRE_NOT_CENTERED']},
            {'ir_receive_age_ms': 501},
            {'ir_band': 'NO_FLAME_SIGNATURE'}):
        assert not realign_candidate(dict(ir(), **edit))
    assert target_reacquire_candidate(dict(
        ir(), blockers=['FIRE_NOT_CONFIRMED', 'FIRE_NOT_CENTERED']))
    assert not target_reacquire_candidate(dict(
        ir(), blockers=['FIRE_NOT_CONFIRMED', 'SCAN_UNSAFE']))
    assert not target_reacquire_candidate(dict(
        ir(), ir_receive_age_ms=501,
        blockers=['FIRE_NOT_CONFIRMED']))
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.02, cmd=0.08)
    core.update('ir', dict(ir(), blockers=['FIRE_NOT_CONFIRMED',
                                           'FIRE_NOT_CENTERED']), 0.8)
    _, state, _ = core.step(
        0.81, integrated_counts(raw=1, raw_owned=True,
                                runtime_ok=False), True)
    assert state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'REALIGN_REQUIRED'


def test_target_lost_while_ir_moves_waits_for_new_centered_handoff():
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.02, cmd=0.08)
    lost = dict(ir(), zone='UNKNOWN', ir_band='NO_FLAME_SIGNATURE',
                blockers=['NO_IR_FLAME_SIGNATURE', 'FIRE_NOT_CONFIRMED',
                          'FIRE_NOT_CENTERED'])
    core.update('ir', lost, 0.8)
    speed, state, _ = core.step(
        0.81, integrated_counts(raw=1, raw_owned=True,
                                runtime_ok=False), True)
    assert speed == 0.0 and state['stop_reason'] == 'REALIGN_REQUIRED'
    for now in (0.9, 1.42):
        core.update('odom', odom(0.02), now)
        core.update('cmd', (0.0,) * 6, now)
        _, state, action = core.step(now + 0.01, owned, True)
    assert state['state'] == 'COMPLETE' and action == 'RELEASE'
    assert state['total_progress_m'] == 0.02
    assert core.step(1.44, integrated_counts(handoff_ok=False), False)[1][
        'state'] == 'COMPLETE'
    core.update('ir', dict(ir(), zone='APPROACH', blockers=[]), 1.5)
    _, state, action = core.step(1.51, integrated_counts(), False)
    assert state['state'] == 'WAITING_FOR_ADMISSION'
    assert action is None and state['total_progress_m'] == 0.02


def test_continuous_spray_coast_past_segment_budget_aborts():
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.29, cmd=0.08)
    core.update('ir', spray_candidate(), 0.8)
    core.step(0.81, integrated_counts(raw=1, raw_owned=True,
                                      runtime_ok=False), True)
    core.update('odom', odom(0.34), 0.9)
    core.update('cmd', (0.0,) * 6, 0.9)
    core.step(0.91, owned, True)
    core.update('ir', confirmed_spray(), 1.42)
    core.update('odom', odom(0.34), 1.42)
    core.update('cmd', (0.0,) * 6, 1.42)
    _, state, action = core.step(1.43, owned, True)
    assert state['state'] == 'ABORTED' and action == 'RELEASE'
    assert state['stop_reason'] == 'APPROACH_SEGMENT_OVERRUN'


def test_continuous_segment_restarts_same_goal_after_stationary_handoff():
    core, owned = moving_continuous()
    refresh(core, 0.8, x=0.30, cmd=0.08)
    speed, state, _ = core.step(0.81, owned, True)
    assert speed == 0.0 and state['stop_reason'] == 'APPROACH_DISTANCE_LIMIT'
    core.update('odom', odom(0.30), 0.9)
    core.update('cmd', (0.0,) * 6, 0.9)
    core.step(0.91, owned, True)
    core.update('odom', odom(0.30), 1.42)
    core.update('cmd', (0.0,) * 6, 1.42)
    _, state, action = core.step(1.43, owned, True)
    assert state['state'] == 'COMPLETE' and action == 'RELEASE'
    assert state['total_progress_m'] == 0.30
    core.update('ir', dict(ir(), zone='APPROACH', blockers=[]), 1.50)
    _, state, action = core.step(1.51, integrated_counts(
        handoff_ok=False), False)
    assert state['state'] == 'COMPLETE' and action is None
    _, state, action = core.step(1.52, integrated_counts(), False)
    assert state['state'] == 'WAITING_FOR_ADMISSION'
    assert action is None and state['total_progress_m'] == 0.30


@pytest.mark.parametrize('reason', [
    'REALIGN_REQUIRED', 'APPROACH_DISTANCE_LIMIT'])
def test_recentered_flame_now_in_spray_range_needs_no_more_ir_motion(reason):
    core = continuous_core()
    core.phase = 'COMPLETE'
    core.goal_uuid = 'ab' * 16
    core.stop_reason = reason
    core.step_count = 2
    core.total_progress_m = 0.30
    core.update('ir', confirmed_spray(), 0.2)
    _, state, action = core.step(0.21, integrated_counts(
        handoff_ok=False, range_handoff_ok=False), False)
    assert state['stop_reason'] == reason and action is None
    _, state, action = core.step(0.22, integrated_counts(
        handoff_ok=False, range_handoff_ok=True), False)
    assert state['state'] == 'COMPLETE'
    assert state['stop_reason'] == 'SPRAY_RANGE_REACHED'
    assert state['total_progress_m'] == 0.30
    assert action is None


def test_post_spray_approach_requires_review_and_actuator_off():
    core = continuous_core()
    core.phase = 'COMPLETE'
    core.goal_uuid = 'ab' * 16
    core.stop_reason = 'SPRAY_RANGE_REACHED'
    core.step_count = 1
    core.total_progress_m = 0.10
    refresh(core, 0.2)
    core.update('ir', dict(ir(), zone='APPROACH', blockers=[]), 0.2)
    core.update('review', {
        'schema_version': 1, 'mode': 'SHADOW_ONLY',
        'state': 'APPROACH_REQUIRED', 'goal_uuid': 'ab' * 16,
        'pulse_index': 1, 'blockers': []}, 0.2)
    core.update('actuator', {
        'pump_pwm_commanded_on': False,
        'servo_pwm_commanded_on': False,
        'servo_phase': 'IDLE', 'receive_age_ms': 20.0}, 0.2)
    counts = integrated_counts(pump=1, review_source_ok=True,
                               actuator_source_ok=True)
    _, state, action = core.step(0.21, counts, False)
    assert state['state'] == 'COMPLETE' and action is None
    assert core.pending_review[0] == 1
    counts['pump'] = 0
    counts['actuator_source_ok'] = False
    assert core.step(0.22, counts, False)[1]['state'] == 'COMPLETE'
    counts['actuator_source_ok'] = True
    _, state, action = core.step(0.23, counts, False)
    assert state['state'] == 'WAITING_FOR_ADMISSION'
    assert action is None
    assert core.last_review_pulse == 1
    assert state['total_progress_m'] == 0.10


def test_continuous_mode_requires_integrated_forward_only_and_hard_bounds():
    for options in (
            {'integrated_handoff': False},
            {'enable_reverse_jog': True},
            {'speed_m_s': 0.081},
            {'continuous_distance_m': 0.31}):
        with pytest.raises(ValueError):
            IrApproachLiveCore(
                enabled=True, allow_provisional_ir=True,
                continuous_approach=True,
                **dict({'integrated_handoff': True}, **options))


def test_integrated_ir_requires_exact_handoff_and_same_uuid():
    core = IrApproachLiveCore(
        enabled=True, allow_provisional_ir=True,
        integrated_handoff=True)
    core.update('ir', centered_ir(), 0.1)
    core.update('scan', scan(), 0.1)
    core.update('odom', odom(), 0.1)
    core.update('cmd', (0.0,) * 6, 0.1)
    core.update_arm(False, 0.0)
    core.update_arm(True, 0.1)
    assert core.step(0.2, integrated_counts(handoff_ok=False), False)[2] \
        is None
    assert core.step(0.21, integrated_counts(), False)[2] is None
    refresh(core, 0.53)
    assert core.step(0.54, integrated_counts(), False)[2] == 'CREATE'
    assert core.goal_uuid == 'ab' * 16
    owned = integrated_counts(raw=1, raw_owned=True,
                              goal_uuid='cd' * 16)
    speed, state, _ = core.step(0.55, owned, True)
    assert speed == 0.0
    assert state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'EXACT_HANDOFF_PROOF_INVALID'


def test_integrated_ir_stops_at_spray_and_releases_after_zero_hold():
    core = IrApproachLiveCore(
        enabled=True, allow_provisional_ir=True,
        integrated_handoff=True)
    core.update('ir', centered_ir(), 0.1)
    core.update('scan', scan(), 0.1)
    core.update('odom', odom(), 0.1)
    core.update('cmd', (0.0,) * 6, 0.1)
    core.update_arm(False, 0.0)
    core.update_arm(True, 0.1)
    assert core.step(0.2, integrated_counts(), False)[2] is None
    refresh(core, 0.53)
    assert core.step(0.54, integrated_counts(), False)[2] == 'CREATE'
    owned = integrated_counts(raw=1, raw_owned=True)
    core.step(0.6, owned, True)
    refresh(core, 0.7, x=0.015, cmd=0.05, band='SPRAY_RANGE')
    speed, state, _ = core.step(0.71, owned, True)
    assert speed == 0.0 and state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'SPRAY_RANGE_REACHED'
    core.update('odom', odom(0.015), 0.8)
    core.update('cmd', (0.0,) * 6, 0.8)
    core.step(0.81, owned, True)
    core.update('odom', odom(0.015), 1.33)
    core.update('cmd', (0.0,) * 6, 1.33)
    _, state, action = core.step(1.34, owned, True)
    assert action == 'RELEASE' and state['state'] == 'COMPLETE'
    assert state['step_count'] == 1
    assert state['total_progress_m'] == 0.015
    assert core.step(1.35, integrated_counts(), False)[2] is None


def test_integrated_ir_sends_zero_until_guard_acknowledges_raw_owner():
    core = IrApproachLiveCore(
        enabled=True, allow_provisional_ir=True,
        integrated_handoff=True)
    core.update('ir', centered_ir(), 0.1)
    core.update('scan', scan(), 0.1)
    core.update('odom', odom(), 0.1)
    core.update('cmd', (0.0,) * 6, 0.1)
    core.update_arm(False, 0.0)
    core.update_arm(True, 0.1)
    assert core.step(0.2, integrated_counts(), False)[2] is None
    refresh(core, 0.53)
    assert core.step(0.54, integrated_counts(), False)[2] == 'CREATE'
    owned = integrated_counts(raw=1, raw_owned=True,
                              guard_owner_ack=False)
    assert core.step(0.6, owned, True)[0] == 0.0
    assert core.phase == 'ACQUIRING'
    refresh(core, 0.65)
    owned['guard_owner_ack'] = True
    assert core.step(0.66, owned, True)[0] == 0.05


def test_integrated_ir_repeats_only_after_new_handoff_proof():
    core = IrApproachLiveCore(
        enabled=True, allow_provisional_ir=True,
        integrated_handoff=True)
    core.phase = 'COMPLETE'
    core.goal_uuid = 'ab' * 16
    core.stop_reason = 'STEP_PROGRESS_TIME_LIMIT'
    core.step_count = 1
    core.total_progress_m = 0.015
    refresh(core, 1.0)
    assert core.step(1.01, integrated_counts(handoff_ok=False), False)[1][
        'state'] == 'COMPLETE'
    assert core.step(1.02, integrated_counts(), False)[1][
        'state'] == 'WAITING_FOR_ADMISSION'
    core.phase = 'COMPLETE'
    core.step_count = 20
    assert core.step(1.03, integrated_counts(), False)[1][
        'approach_budget_exhausted'] is True
    assert core.phase == 'COMPLETE'


def test_integrated_ir_rearms_only_for_distinct_proven_goal():
    core = IrApproachLiveCore(
        enabled=True, allow_provisional_ir=True,
        integrated_handoff=True)
    core.phase = 'COMPLETE'
    core.goal_uuid = 'ab' * 16
    core.stop_reason = 'SPRAY_RANGE_REACHED'
    core.step_count = 3
    refresh(core, 1.0)
    assert core.step(1.01, integrated_counts(
        goal_uuid='cd' * 16, handoff_ok=False), False)[1][
        'goal_uuid'] == 'ab' * 16
    state = core.step(1.02, integrated_counts(
        goal_uuid='cd' * 16), False)[1]
    assert state['goal_uuid'] is None
    assert state['state'] == 'WAITING_FOR_ADMISSION'
    assert state['step_count'] == 0
    assert 'ab' * 16 in core.used_goal_uuids


@pytest.mark.parametrize('missing', [
    {'nav_shadow_ok': False}, {'status_source_ok': False},
    {'pump': 1}, {'raw': 1}, {'goal_uuid': None},
])
def test_integrated_ir_never_claims_with_missing_owner_or_proof(missing):
    core = IrApproachLiveCore(
        enabled=True, allow_provisional_ir=True,
        integrated_handoff=True)
    core.update('ir', ir(), 0.1)
    core.update('scan', scan(), 0.1)
    core.update('odom', odom(), 0.1)
    core.update('cmd', (0.0,) * 6, 0.1)
    core.update_arm(False, 0.0)
    core.update_arm(True, 0.1)
    counts = integrated_counts(**missing)
    assert core.step(0.2, counts, False)[2] is None
    refresh(core, 0.6)
    assert core.step(0.61, counts, False)[2] is None


def test_wrong_ir_or_scan_source_blocks_admission():
    for name in ('ir_owner', 'scan_owner', 'odom_owner'):
        core = ready()
        counts = dict(COUNTS, **{name: False})
        assert core.step(0.2, counts, False)[2] is None


def endpoint(*, gid=1, name='safety_watchdog', namespace='/',
             message_type='geometry_msgs/msg/Twist', reliability=2):
    return SimpleNamespace(
        endpoint_gid=[gid] + [0] * 23,
        node_name=name, node_namespace=namespace,
        topic_type=message_type,
        qos_profile=SimpleNamespace(reliability=reliability),
    )


def gate_matches(pins, info):
    return pins.matches(
        'raw_gate', info, name='safety_watchdog',
        message_type='geometry_msgs/msg/Twist', reliability=2)


def test_pi_endpoint_unknown_name_requires_prior_exact_gid():
    pins = VerifiedEndpointPins()
    unknown = endpoint(name='_NODE_NAME_UNKNOWN_',
                       namespace='_NODE_NAMESPACE_UNKNOWN_')
    assert not gate_matches(pins, unknown)
    assert gate_matches(pins, endpoint())
    assert gate_matches(pins, unknown)
    assert gate_matches(pins, endpoint())


def test_pi_endpoint_gid_change_or_other_name_fails_closed():
    pins = VerifiedEndpointPins()
    assert gate_matches(pins, endpoint())
    assert not gate_matches(pins, endpoint(
        gid=2, name='_NODE_NAME_UNKNOWN_',
        namespace='_NODE_NAMESPACE_UNKNOWN_'))
    assert not gate_matches(pins, endpoint(gid=2))
    assert not gate_matches(pins, endpoint(name='other_node'))
    assert gate_matches(pins, endpoint())


def test_pi_endpoint_qos_type_or_invalid_gid_fails_closed():
    pins = VerifiedEndpointPins()
    assert not gate_matches(pins, endpoint(reliability=1))
    assert not gate_matches(pins, endpoint(
        message_type='std_msgs/msg/String'))


@pytest.mark.parametrize('recorder_reliability', [1, 2])
def test_raw_gate_allows_exactly_one_verified_bag_observer(
        recorder_reliability):
    pins = VerifiedEndpointPins()
    gate = endpoint()
    recorder = endpoint(gid=2, name='rosbag2_recorder',
                        reliability=recorder_reliability)
    assert verified_raw_gate(pins, [gate], 2, (1, 2))
    assert verified_raw_gate(pins, [gate, recorder], 2, (1, 2))
    assert verified_raw_gate(pins, [recorder, gate], 2, (1, 2))
    unknown = endpoint(gid=2, name='_NODE_NAME_UNKNOWN_',
                       namespace='_NODE_NAMESPACE_UNKNOWN_',
                       reliability=recorder_reliability)
    assert verified_raw_gate(pins, [gate, unknown], 2, (1, 2))


def test_raw_gate_rejects_recorder_without_safety_gate():
    pins = VerifiedEndpointPins()
    recorder = endpoint(gid=2, name='rosbag2_recorder', reliability=1)
    assert not verified_raw_gate(pins, [recorder], 2, (1, 2))
    assert not verified_raw_gate(pins, [], 2, (1, 2))


def test_raw_gate_rejects_unknown_extra_subscriber_or_third_endpoint():
    pins = VerifiedEndpointPins()
    gate = endpoint()
    recorder = endpoint(gid=2, name='rosbag2_recorder', reliability=1)
    other = endpoint(gid=3, name='other_node')
    assert verified_raw_gate(pins, [gate], 2, (1, 2))
    assert not verified_raw_gate(pins, [gate, other], 2, (1, 2))
    assert not verified_raw_gate(pins, [gate, recorder, other], 2, (1, 2))


def test_integrated_raw_gate_requires_exact_supervisor_observer():
    pins = VerifiedEndpointPins()
    gate = endpoint()
    supervisor = endpoint(gid=2, name='fire_mission_supervisor')
    recorder = endpoint(gid=3, name='rosbag2_recorder', reliability=1)
    assert verified_raw_gate(pins, [gate, supervisor], 2, (1, 2),
                             require_supervisor=True)
    assert verified_raw_gate(pins, [supervisor, gate, recorder], 2, (1, 2),
                             require_supervisor=True)
    assert not verified_raw_gate(pins, [gate], 2, (1, 2),
                                 require_supervisor=True)
    assert not verified_raw_gate(pins, [gate, recorder], 2, (1, 2),
                                 require_supervisor=True)
    rogue = endpoint(gid=4, name='other_node')
    assert not verified_raw_gate(pins, [gate, supervisor, rogue],
                                 2, (1, 2), require_supervisor=True)


def test_raw_gate_rejects_recorder_gid_type_or_qos_change():
    pins = VerifiedEndpointPins()
    gate = endpoint()
    recorder = endpoint(gid=2, name='rosbag2_recorder', reliability=1)
    assert verified_raw_gate(pins, [gate, recorder], 2, (1, 2))
    assert not verified_raw_gate(pins, [gate, endpoint(
        gid=3, name='rosbag2_recorder', reliability=1)], 2, (1, 2))
    assert not verified_raw_gate(pins, [gate, endpoint(
        gid=2, name='rosbag2_recorder', reliability=1,
        message_type='std_msgs/msg/String')], 2, (1, 2))
    assert not verified_raw_gate(pins, [gate, endpoint(
        gid=2, name='rosbag2_recorder', reliability=3)], 2, (1, 2))
    assert not verified_raw_gate(pins, [gate, endpoint(
        gid=2, name='rosbag2_recorder', reliability=2)], 2, (1, 2))
    assert not gate_matches(pins, endpoint(gid=0))
    assert gate_matches(pins, endpoint())
    assert not gate_matches(pins, endpoint(reliability=1))
    assert not gate_matches(pins, endpoint(
        message_type='std_msgs/msg/String'))


def test_safety_gate_discovery_blip_after_claim_waits_with_zero():
    core = ready()
    admit(core)
    counts = dict(COUNTS, raw=1, raw_owned=True, raw_gate=False)
    speed, state, action = core.step(0.55, counts, True)
    assert speed == 0.0 and action is None
    assert state['state'] == 'ACQUIRING'
    assert state['blockers'] == ['SAFETY_GATE_TOPOLOGY_INVALID']
    recovered = dict(COUNTS, raw=1, raw_owned=True)
    speed, state, _ = core.step(0.60, recovered, True)
    assert speed == 0.05 and state['state'] == 'MOVING'


def test_safety_gate_discovery_timeout_never_moves():
    core = ready()
    admit(core)
    counts = dict(COUNTS, raw=1, raw_owned=True, raw_gate=False)
    core.step(0.55, counts, True)
    refresh(core, 1.27)
    speed, state, _ = core.step(1.28, counts, True)
    assert speed == 0.0 and state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'SAFETY_GATE_TOPOLOGY_INVALID'


def test_safety_gate_loss_during_motion_stops_immediately():
    core = ready()
    admit(core)
    owned = dict(COUNTS, raw=1, raw_owned=True)
    assert core.step(0.60, owned, True)[0] == 0.05
    refresh(core, 0.70, cmd=0.05)
    owned['raw_gate'] = False
    speed, state, _ = core.step(0.71, owned, True)
    assert speed == 0.0 and state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'SAFETY_GATE_TOPOLOGY_INVALID'


def test_second_raw_publisher_during_acquiring_fails_closed():
    core = ready()
    admit(core)
    counts = dict(COUNTS, raw=2, raw_owned=False)
    speed, state, _ = core.step(0.55, counts, True)
    assert speed == 0.0 and state['state'] == 'STOPPING'
    assert state['stop_reason'] == 'RAW_OWNER_CONFLICT'


def test_hard_limits_and_static_launch_opt_in():
    with pytest.raises(ValueError):
        IrApproachLiveCore(enabled=True, allow_provisional_ir=True,
                           speed_m_s=0.2)
    with pytest.raises(ValueError):
        IrApproachLiveCore(enabled=True, allow_provisional_ir=True,
                           step_m=0.02, speed_m_s=0.05)
    launch = (ROOT / 'launch' / 'ir_approach_live.launch.py').read_text()
    assert "default_value='false'" in launch
    assert "executable='ir_approach_live'" in launch
    assert 'pump_demo_controller' not in launch
    tree = ast.parse((ROOT / 'fire_robot_mission' /
                      'ir_approach_live.py').read_text())
    publishers = [call for call in ast.walk(tree)
                  if isinstance(call, ast.Call) and
                  isinstance(call.func, ast.Attribute) and
                  call.func.attr == 'create_publisher']
    types = {call.args[0].id for call in publishers}
    assert types == {'String', 'Twist'}
    assert all('/pump_cmd' not in ast.unparse(call) for call in publishers)
