"""OFF causality and source transitions; no ROS context or pump hardware."""

from copy import deepcopy

import pytest

from fire_robot_mission.pump_release import PumpReleaseProof


ORIGIN = 100_000_000_000
GOAL = 'ab' * 16


def actor(stamp_ns, seq, *, epoch=1, instance='ab' * 16, on=False, idle=True):
    sec, nanosec = divmod(stamp_ns, 1_000_000_000)
    return {
        'schema_version': 1, 'mode': 'COMMAND_TELEMETRY_ONLY',
        'pump_pwm_commanded_on': on, 'servo_pwm_commanded_on': on or not idle,
        'servo_phase': 'SWEEPING' if on else 'IDLE' if idle else 'RETURNING_HOME',
        'physical_pump_or_nozzle_verified': False,
        'receive_age_ms': 10.0, 'esp_ms': 1000 + seq,
        'source': {'stamp': {'sec': sec, 'nanosec': nanosec},
                   'bridge_instance_id': instance, 'serial_epoch': epoch,
                   'frame_seq': seq},
    }


def waiting():
    proof = PumpReleaseProof()
    proof.owner_created()
    proof.update(actor(ORIGIN - 10_000_000, 1), ORIGIN, 1.0, source_ok=True)
    assert proof.request_after_off_publish(GOAL, ORIGIN, 1.0)
    return proof


def after_off(proof, **overrides):
    proof.update(actor(ORIGIN + 90_000_000, 2, **overrides),
                 ORIGIN + 100_000_000, 1.1, source_ok=True)


@pytest.mark.parametrize('offset', [-10_000_000, 0, 19_000_000, 20_000_000])
def test_old_or_skew_ambiguous_off_is_not_after_the_request(offset):
    proof = waiting()
    # A later local receipt or sequence must not re-date the ACT stamp.
    proof.update(actor(ORIGIN + offset, 2), ORIGIN + 25_000_000,
                 1.025, source_ok=True)
    assert proof.ready(1.025) is False
    assert proof.receipt is None


def test_fresh_post_off_sample_can_form_one_historical_owner_receipt():
    proof = waiting()
    token = proof.owner_token
    after_off(proof)
    assert proof.ready(1.1) is True
    proof.destroyed(ros_now_ns=ORIGIN + 101_000_000, checked_at=1.1)
    receipt = deepcopy(proof.receipt)
    assert receipt['owner_token'] == token
    assert receipt['goal_uuid'] == GOAL
    assert receipt['session_id'] is None
    assert receipt['physical_pump_or_nozzle_verified'] is False
    assert proof.pending is False
    # Heartbeat snapshots must not make the release event younger.
    assert proof.snapshot(owned=False)['receipt'] == receipt
    proof.update(actor(ORIGIN + 200_000_000, 3), ORIGIN + 210_000_000,
                 1.21, source_ok=True)
    assert proof.snapshot(owned=False)['receipt'] == receipt
    proof.owner_created()
    assert proof.owner_token != token
    assert proof.receipt is None
    assert proof.request_ros_ns is None


@pytest.mark.parametrize('kwargs', [{'on': True}, {'idle': False}])
def test_pump_on_or_servo_active_cannot_release(kwargs):
    proof = waiting()
    after_off(proof, **kwargs)
    assert proof.ready(1.1) is False
    assert proof.blocker == 'ACTUATOR_NOT_OFF_AND_IDLE'


def test_total_age_and_invalid_source_revoke_readiness():
    proof = waiting()
    after_off(proof)
    assert not proof.ready(1.45)  # 10ms at receipt + 350ms local cache.
    proof.update(actor(ORIGIN + 400_000_000, 3), ORIGIN + 410_000_000,
                 1.41, source_ok=False)
    assert not proof.ready(1.41)
    assert proof.blocker == 'ACTUATOR_PUBLISHER_INVALID'


@pytest.mark.parametrize('changed', [{'epoch': 2}, {'instance': 'cd' * 16}])
def test_source_transition_requires_another_off_barrier_and_newer_sample(changed):
    proof = waiting()
    after_off(proof, **changed)
    assert proof.request_ros_ns is None
    assert not proof.ready(1.1)
    assert proof.request_after_off_publish(GOAL, ORIGIN + 110_000_000, 1.11)
    assert not proof.ready(1.11)  # Transition sample predates the new OFF.
    proof.update(actor(ORIGIN + 190_000_000, 3, **changed),
                 ORIGIN + 200_000_000, 1.2, source_ok=True)
    assert proof.ready(1.2)


def test_first_act_arriving_after_missing_source_still_needs_a_new_off_barrier():
    proof = PumpReleaseProof()
    proof.owner_created()
    assert not proof.request_after_off_publish(GOAL, ORIGIN, 1.0)
    after_off(proof)
    assert proof.request_ros_ns is None
    assert not proof.ready(1.1)
    proof.request_after_off_publish(GOAL, ORIGIN + 110_000_000, 1.11)
    assert not proof.ready(1.11)


def test_clock_rollback_cannot_reuse_the_cached_sample():
    proof = waiting()
    after_off(proof)
    assert not proof.request_after_off_publish(GOAL, ORIGIN - 100_000_000, 1.1)
    assert proof.sample is None
    assert not proof.ready(1.1)
    assert proof.blocker == 'ACTUATOR_CLOCK_BACKWARD'


def test_clock_jump_cannot_reuse_a_sample_across_the_off_barrier():
    proof = waiting()
    after_off(proof)
    assert not proof.request_after_off_publish(GOAL, ORIGIN + 1_100_000_000, 1.1)
    assert proof.sample is None
    assert proof.request_ros_ns is None
    assert proof.request_mono is None
    assert not proof.ready(1.1)
    assert proof.blocker == 'ACTUATOR_CLOCK_DISCONTINUITY'


def test_reported_queue_age_cannot_make_an_old_sample_postdate_off():
    proof = waiting()
    payload = actor(ORIGIN + 50_000_000, 2)
    payload['receive_age_ms'] = 150.0
    proof.update(payload, ORIGIN + 100_000_000, 1.1, source_ok=True)
    assert proof.sample is not None  # Fresh overall, but predates this OFF.
    assert not proof.ready(1.1)
    assert proof.blocker == 'ACTUATOR_SAMPLE_NOT_AFTER_OFF'


def test_changed_goal_and_missing_owner_never_produce_a_receipt():
    proof = waiting()
    after_off(proof)
    assert not proof.request_after_off_publish('cd' * 16, ORIGIN + 100_000_000, 1.1)
    assert proof.goal_uuid == GOAL
    assert proof.receipt is None
    orphan = PumpReleaseProof()
    orphan.update(actor(ORIGIN, 1), ORIGIN, 1.0, source_ok=True)
    orphan.request_after_off_publish(GOAL, ORIGIN, 1.0)
    after_off(orphan)
    assert not orphan.ready(1.1)
    with pytest.raises(RuntimeError, match='PUMP_RELEASE_EVIDENCE_INVALID'):
        orphan.destroyed(ros_now_ns=ORIGIN + 100_000_000, checked_at=1.1)


@pytest.mark.parametrize('timeout', [True, 0, -0.1, 0.351, float('nan')])
def test_cleanup_cannot_raise_the_existing_350ms_age_limit(timeout):
    with pytest.raises(ValueError):
        PumpReleaseProof(timeout_s=timeout)
