"""Stamped ACT admission, including serial counters and clock transitions."""

from copy import deepcopy

import pytest

from fire_robot_mission.actuator_freshness import ActuatorStampGuard


ORIGIN = 100_000_000_000


def report(stamp_ns=ORIGIN, *, seq=10, epoch=1, instance='ab' * 16,
           esp_ms=1000, age_ms=10.0):
    sec, nanosec = divmod(stamp_ns, 1_000_000_000)
    return {
        'schema_version': 1, 'mode': 'COMMAND_TELEMETRY_ONLY',
        'pump_pwm_commanded_on': False, 'servo_pwm_commanded_on': False,
        'servo_phase': 'IDLE', 'physical_pump_or_nozzle_verified': False,
        'receive_age_ms': age_ms, 'esp_ms': esp_ms,
        'source': {'stamp': {'sec': sec, 'nanosec': nanosec},
                   'bridge_instance_id': instance, 'serial_epoch': epoch,
                   'frame_seq': seq},
    }


def guard():
    return ActuatorStampGuard(max_age_ms=350.0, max_future_skew_ms=20.0)


def test_charge_transit_without_mutation_and_never_make_reported_age_younger():
    checker = guard()
    payload = report()
    before = deepcopy(payload)
    assert checker.admit(payload, ORIGIN + 200_000_000) == 200.0
    assert payload == before
    assert checker.context_changed is False
    assert checker.admit(report(ORIGIN + 100_000_000, seq=11, age_ms=300),
                         ORIGIN + 210_000_000) == 300.0


@pytest.mark.parametrize('age_ms,now_offset,reason', [
    (10.0, 350_000_001, 'ACTUATOR_SOURCE_STALE'),
    (350.1, 10_000_000, 'ACTUATOR_SOURCE_STALE'),
    (10.0, -20_000_001, 'ACTUATOR_SOURCE_IN_FUTURE'),
    (-0.1, 10_000_000, 'ACTUATOR_SOURCE_AGE_INVALID'),
    (True, 10_000_000, 'ACTUATOR_SOURCE_AGE_INVALID'),
    (float('nan'), 10_000_000, 'ACTUATOR_SOURCE_AGE_INVALID'),
    (float('inf'), 10_000_000, 'ACTUATOR_SOURCE_AGE_INVALID'),
])
def test_invalid_age_never_advances_watermark(age_ms, now_offset, reason):
    checker = guard()
    assert checker.admit(report(age_ms=age_ms), ORIGIN + now_offset) is None
    assert checker.reason == reason
    assert checker.last_stamp_ns is None


@pytest.mark.parametrize('offset,expected', [(-20_000_000, 10.0), (350_000_000, 350.0)])
def test_existing_age_and_skew_boundaries_are_preserved(offset, expected):
    assert guard().admit(report(), ORIGIN + offset) == expected


@pytest.mark.parametrize('field,value', [
    ('bridge_instance_id', '0' * 32), ('bridge_instance_id', 'AB' * 16),
    ('bridge_instance_id', None), ('serial_epoch', 0), ('serial_epoch', True),
    ('frame_seq', -1), ('frame_seq', True), ('frame_seq', 0x1_00000000),
    ('stamp', {'sec': True, 'nanosec': 0}),
    ('stamp', {'sec': 0, 'nanosec': 0}),
    ('stamp', {'sec': 1, 'nanosec': 1_000_000_000}),
])
def test_source_metadata_must_be_complete_and_strictly_typed(field, value):
    checker = guard()
    payload = report()
    payload['source'][field] = value
    assert checker.admit(payload, ORIGIN) is None
    assert checker.reason == 'ACTUATOR_SOURCE_METADATA_INVALID'


@pytest.mark.parametrize('field,value', [
    ('schema_version', True), ('mode', 'ACTUATION_AUTHORIZED'),
    ('pump_pwm_commanded_on', 0), ('servo_phase', 'UNKNOWN'),
    ('physical_pump_or_nozzle_verified', True), ('source', None),
    ('esp_ms', True), ('esp_ms', -1),
])
def test_legacy_or_invalid_reports_cannot_be_admitted(field, value):
    checker = guard()
    payload = report()
    payload[field] = value
    assert checker.admit(payload, ORIGIN) is None
    assert checker.last_stamp_ns is None


def test_duplicate_stamp_and_relabelled_duplicate_sequence_are_both_rejected():
    checker = guard()
    assert checker.admit(report(), ORIGIN + 10_000_000) == 10.0
    assert checker.admit(report(seq=11), ORIGIN + 20_000_000) is None
    assert checker.reason == 'ACTUATOR_STAMP_NON_INCREASING'
    assert checker.admit(report(ORIGIN + 10_000_000), ORIGIN + 20_000_000) is None
    assert checker.reason == 'ACTUATOR_SEQUENCE_NON_INCREASING'
    assert checker.last_seq == 10
    assert checker.admit(report(ORIGIN + 10_000_000, seq=12),
                         ORIGIN + 20_000_000) == 10.0


def test_uint32_wrap_and_interleaved_global_serial_gaps_are_valid():
    checker = guard()
    assert checker.admit(report(seq=0xFFFFFFFE, esp_ms=0xFFFFFFF0), ORIGIN) == 10.0
    assert checker.admit(report(ORIGIN + 10_000_000, seq=2, esp_ms=5),
                         ORIGIN + 10_000_000) == 10.0
    assert checker.context_changed is False


@pytest.mark.parametrize('seq,esp_ms,reason', [
    (9, 1100, 'ACTUATOR_SEQUENCE_NON_INCREASING'),
    (10 + 0x80000000, 1100, 'ACTUATOR_SEQUENCE_NON_INCREASING'),
    (11, 999, 'ACTUATOR_UPTIME_BACKWARD'),
])
def test_out_of_order_or_ambiguous_counters_fail_closed(seq, esp_ms, reason):
    checker = guard()
    checker.admit(report(), ORIGIN)
    assert checker.admit(report(ORIGIN + 1_000_000, seq=seq, esp_ms=esp_ms),
                         ORIGIN + 1_000_000) is None
    assert checker.reason == reason


def test_serial_epoch_and_bridge_change_are_explicit_context_transitions():
    checker = guard()
    checker.admit(report(epoch=2), ORIGIN)
    assert checker.admit(report(ORIGIN + 10_000_000, epoch=1, seq=11),
                         ORIGIN + 10_000_000) is None
    assert checker.reason == 'ACTUATOR_EPOCH_BACKWARD'
    assert checker.admit(report(ORIGIN + 20_000_000, epoch=3, seq=1, esp_ms=5),
                         ORIGIN + 20_000_000) == 10.0
    assert checker.context_changed is True
    assert checker.admit(report(ORIGIN + 30_000_000, instance='cd' * 16, seq=1),
                         ORIGIN + 30_000_000) == 10.0
    assert checker.context_changed is True
    # A delayed old-instance frame is not made current by its later receipt.
    assert checker.admit(report(ORIGIN + 25_000_000, epoch=3, seq=2),
                         ORIGIN + 40_000_000) is None
    assert checker.reason == 'ACTUATOR_STAMP_NON_INCREASING'


def test_clock_rollback_rejects_this_call_and_keeps_sequence_replay_protection():
    checker = guard()
    checker.admit(report(), ORIGIN + 10_000_000)
    assert checker.admit(report(), ORIGIN - 1_000_000_000) is None
    assert checker.reason == 'ACTUATOR_CLOCK_BACKWARD'
    lower = ORIGIN - 900_000_000
    assert checker.admit(report(lower), lower + 10_000_000) is None
    assert checker.reason == 'ACTUATOR_SEQUENCE_NON_INCREASING'
    assert checker.admit(report(lower, seq=11), lower + 20_000_000) == 20.0
    assert checker.context_changed is True


@pytest.mark.parametrize('now', [None, True, 0])
def test_receiver_clock_must_be_a_positive_integer(now):
    checker = guard()
    assert checker.admit(report(), now) is None
    assert checker.reason == 'ACTUATOR_CLOCK_INVALID'
