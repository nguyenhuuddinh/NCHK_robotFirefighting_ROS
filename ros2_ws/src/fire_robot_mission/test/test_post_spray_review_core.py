"""Pure, non-actuating post-spray review tests."""

import pytest

from fire_robot_mission.post_spray_review_core import PostSprayReviewCore


def inputs(core, now, *, fire=True, band='SPRAY_RANGE', centered=True):
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'COMPLETE',
                         'goal_uuid': 'a' * 32,
                         'pump_on': False,
                         'stop_reason': 'PULSE_TIME_LIMIT'}, now)
    core.update('vision', {'schema_version': 1, 'observe_only': True,
                           'state': 'ok', 'source': {'age_ms': 25},
                           'fire': {'detected_now': fire,
                                    'confirmed': fire}}, now)
    ir_blockers = ['NO_IR_FLAME_SIGNATURE', 'FIRE_NOT_CONFIRMED'] \
        if band == 'NO_FLAME_SIGNATURE' else []
    core.update('ir', {'schema_version': 1, 'mode': 'OBSERVE_ONLY',
                       'observe_only': True, 'ir_band': band,
                       'zone': 'UNKNOWN' if ir_blockers else band,
                       'ir_receive_age_ms': 30, 'stable_samples': 8,
                       'blockers': ir_blockers,
                       'calibration': 'PROVISIONAL_SINGLE_FLAME',
                       'validated_for_actuation': False}, now)
    core.update('alignment', {'schema_version': 1, 'observe_only': True,
                              'state': 'CENTERED' if centered else
                              'ALIGNING_LEFT'}, now)
    core.update('mission', {'schema_version': 1, 'observe_only': True,
                            'robot': {'odom_fresh': True, 'stationary': True},
                            'navigation': {'goal_active': False},
                            'health': {'status': 'OK', 'blockers': []},
                            'safety': {'motion_command_publisher': False,
                                       'actuator_command_publisher': False}},
                now)


def complete_pulse(core):
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'FIRING',
                         'goal_uuid': 'a' * 32,
                         'pump_on': True}, 0.0)
    assert core.step(0.0)['state'] == 'FIRING'
    inputs(core, 0.2)
    assert core.step(0.2)['state'] == 'SETTLING'


def review(core, *, fire=True, band='SPRAY_RANGE', centered=True):
    inputs(core, 1.3, fire=fire, band=band, centered=centered)
    assert core.step(1.3)['state'] == 'REVIEWING'
    inputs(core, 2.4, fire=fire, band=band, centered=centered)
    return core.step(2.4)


def test_no_preexisting_complete_can_fake_a_review():
    core = PostSprayReviewCore()
    inputs(core, 0.0, fire=False, band='NO_FLAME_SIGNATURE')
    assert core.step(0.0)['state'] == 'WAITING_FOR_PULSE'
    assert core.step(10.0)['state'] == 'WAITING_FOR_PULSE'


def test_fire_still_centered_and_spray_range_is_shadow_respray():
    core = PostSprayReviewCore()
    complete_pulse(core)
    report = review(core)
    assert report['state'] == 'RESPRAY_READY_SHADOW'
    assert report['pulse_index'] == 1
    assert report['safety']['pump_command_publisher'] is False
    assert report['safety']['motion_command_publisher'] is False


def test_off_center_requests_realignment_before_respray():
    core = PostSprayReviewCore()
    complete_pulse(core)
    assert review(core, centered=False)['state'] == 'REALIGN_REQUIRED'


def test_no_fire_in_both_sensors_is_extinguished_candidate():
    core = PostSprayReviewCore()
    complete_pulse(core)
    assert review(core, fire=False,
                  band='NO_FLAME_SIGNATURE')['state'] == 'NO_FIRE_EVIDENCE'


@pytest.mark.parametrize('reason', [
    'IR_NOT_CONFIRMED_SPRAY_RANGE', 'MISSION_NOT_SAFE_AND_CENTERED',
])
def test_early_sensor_stop_is_reviewed_but_not_auto_declared_clear(reason):
    core = PostSprayReviewCore()
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'FIRING',
                         'goal_uuid': 'a' * 32, 'pump_on': True}, 0.0)
    assert core.step(0.0)['state'] == 'FIRING'
    inputs(core, 0.2, fire=False, band='NO_FLAME_SIGNATURE')
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'ABORTED',
                         'goal_uuid': 'a' * 32, 'pump_on': False,
                         'stop_reason': reason}, 0.2)
    assert core.step(0.2)['state'] == 'SETTLING'
    assert review(core, fire=False,
                  band='NO_FLAME_SIGNATURE')['state'] == 'NO_FIRE_EVIDENCE'


def test_early_sensor_stop_with_fire_still_visible_does_not_clear():
    core = PostSprayReviewCore()
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'FIRING',
                         'goal_uuid': 'a' * 32, 'pump_on': True}, 0.0)
    assert core.step(0.0)['state'] == 'FIRING'
    inputs(core, 0.2)
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'ABORTED',
                         'goal_uuid': 'a' * 32, 'pump_on': False,
                         'stop_reason': 'IR_NOT_CONFIRMED_SPRAY_RANGE'}, 0.2)
    assert core.step(0.2)['state'] == 'SETTLING'
    assert review(core)['state'] == 'RESPRAY_READY_SHADOW'


def test_confirmed_extinguished_review_resets_for_next_pump_cycle():
    core = PostSprayReviewCore()
    complete_pulse(core)
    report = review(core, fire=False, band='NO_FLAME_SIGNATURE')
    assert report['goal_uuid'] == 'a' * 32
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'WAITING_FOR_ADMISSION',
                         'goal_uuid': None, 'pump_on': False}, 2.5)
    report = core.step(2.5)
    assert report['state'] == 'WAITING_FOR_PULSE'
    assert report['pulse_index'] == 0 and report['goal_uuid'] is None


def test_fire_far_requests_approach_but_never_moves():
    core = PostSprayReviewCore()
    complete_pulse(core)
    report = review(core, band='APPROACH')
    assert report['state'] == 'APPROACH_REQUIRED'
    assert report['mode'] == 'SHADOW_ONLY'


def test_far_fire_can_return_to_respray_after_pump_owner_handoff():
    core = PostSprayReviewCore()
    complete_pulse(core)
    assert review(core, band='APPROACH')['state'] == 'APPROACH_REQUIRED'
    core.update('pump', None, 2.5)
    assert core.step(2.5)['state'] == 'REVIEWING'
    # After motion has stopped and IR reports spray range, the next pump
    # controller must first publish OFF; review remains read-only.
    inputs(core, 3.0)
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'WAITING_FOR_ADMISSION',
                         'goal_uuid': None, 'pump_on': False}, 3.0)
    assert core.step(3.0)['state'] == 'REVIEWING'
    inputs(core, 4.1)
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'WAITING_FOR_ADMISSION',
                         'goal_uuid': None, 'pump_on': False}, 4.1)
    report = core.step(4.1)
    assert report['state'] == 'RESPRAY_READY_SHADOW'
    assert report['pulse_index'] == 1
    assert report['goal_uuid'] == 'a' * 32
    assert report['safety']['actuation_authorized'] is False


def test_camera_ir_disagreement_does_not_decide():
    core = PostSprayReviewCore()
    complete_pulse(core)
    inputs(core, 1.3, fire=False, band='SPRAY_RANGE')
    report = core.step(1.3)
    assert report['state'] == 'REVIEWING'
    assert 'VISION_IR_DISAGREE' in report['blockers']


def test_stale_evidence_cannot_confirm_after_timer():
    core = PostSprayReviewCore()
    complete_pulse(core)
    inputs(core, 1.3)
    assert core.step(1.3)['state'] == 'REVIEWING'
    report = core.step(2.4)
    assert report['state'] == 'REVIEWING'
    assert report['blockers']


def test_confirmed_result_retracts_when_sources_go_stale():
    core = PostSprayReviewCore()
    complete_pulse(core)
    assert review(core, fire=False,
                  band='NO_FLAME_SIGNATURE')['state'] == 'NO_FIRE_EVIDENCE'
    report = core.step(3.1)
    assert report['state'] == 'REVIEWING'
    assert report['blockers']


def test_abort_is_not_extinguished_and_malformed_inputs_do_not_crash():
    core = PostSprayReviewCore()
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'FIRING',
                         'pump_on': True}, 0.0)
    core.step(0.0)
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'ABORTED',
                         'pump_on': False}, 0.2)
    assert core.step(0.2)['state'] == 'ABORTED'
    core.update('vision', {'source': 'bad'}, 0.3)
    assert core.step(0.3)['state'] == 'ABORTED'


def test_second_pulse_is_only_observed_after_respray_ready_and_off_gap():
    core = PostSprayReviewCore(max_pulses=2)
    complete_pulse(core)
    assert review(core)['state'] == 'RESPRAY_READY_SHADOW'
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'FIRING',
                         'pump_on': True}, 2.5)
    report = core.step(2.5)
    assert report['state'] == 'FIRING'
    assert report['pulse_index'] == 2
    assert report['safety']['actuation_authorized'] is False
    inputs(core, 2.6)
    assert core.step(2.6)['state'] == 'SETTLING'
    inputs(core, 3.7)
    assert core.step(3.7)['state'] == 'REVIEWING'
    inputs(core, 4.8)
    report = core.step(4.8)
    assert report['state'] == 'RETRY_LIMIT_REACHED'
    assert report['max_pulses'] == 2


def test_pump_restart_before_review_is_reported_not_endorsed():
    core = PostSprayReviewCore()
    complete_pulse(core)
    inputs(core, 1.3)
    assert core.step(1.3)['state'] == 'REVIEWING'
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'FIRING',
                         'pump_on': True}, 1.4)
    report = core.step(1.4)
    assert report['state'] == 'ABORTED'
    assert 'UNEXPECTED_PUMP_RESTART' in report['blockers']
    assert core.step(1.5)['pulse_index'] == 2


def test_respray_label_is_not_authority_after_evidence_goes_stale():
    core = PostSprayReviewCore()
    complete_pulse(core)
    assert review(core)['state'] == 'RESPRAY_READY_SHADOW'
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'FIRING',
                         'pump_on': True}, 3.2)
    report = core.step(3.2)
    assert report['state'] == 'ABORTED'
    assert 'UNEXPECTED_PUMP_RESTART' in report['blockers']
    assert 'VISION_MISSING_OR_INVALID' in report['blockers']


def test_pulse_after_limit_is_reported_not_endorsed():
    core = PostSprayReviewCore(max_pulses=1)
    complete_pulse(core)
    assert review(core)['state'] == 'RETRY_LIMIT_REACHED'
    core.update('pump', {'schema_version': 1, 'mode': 'SUPERVISED_DEMO',
                         'one_shot': True, 'state': 'FIRING',
                         'pump_on': True}, 2.5)
    report = core.step(2.5)
    assert report['state'] == 'ABORTED'
    assert 'UNEXPECTED_PUMP_RESTART' in report['blockers']
    assert core.step(2.6)['pulse_index'] == 2


@pytest.mark.parametrize('limit', [0, 6, 1.0, True])
def test_invalid_pulse_limit_rejected(limit):
    with pytest.raises(ValueError):
        PostSprayReviewCore(max_pulses=limit)
