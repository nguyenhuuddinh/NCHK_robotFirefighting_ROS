"""Check detection admission and the actual ROS callback without ROS nodes."""

import copy
import json
from types import SimpleNamespace

import pytest

from fire_robot_mission.detection_freshness import DetectionStampGuard
from fire_robot_mission.mission_supervisor import (
    FireMissionSupervisor, MissionSupervisorCore,
)


def observation(stamp_ns=9_800_000_000, age_ms=10.0):
    sec, nanosec = divmod(stamp_ns, 1_000_000_000)
    return {
        'schema_version': 1, 'observe_only': True, 'state': 'ok',
        'source': {'stamp': {'sec': sec, 'nanosec': nanosec},
                   'age_ms': age_ms, 'frame_id': 'camera_frame'},
        'fire': {'detected_now': True, 'confirmed': True},
    }


def test_receipt_clock_accounts_for_transit_without_mutating_json():
    guard = DetectionStampGuard()
    payload = observation(age_ms=10.0)
    original = copy.deepcopy(payload)
    admitted = guard.admit(payload, 10_000_000_000)
    assert admitted['source']['age_ms'] == pytest.approx(200.0)
    assert payload == original
    assert admitted is not payload and admitted['source'] is not payload['source']
    assert guard.last_stamp_ns == 9_800_000_000
    assert guard.reason is None


def test_reported_age_is_never_made_younger_by_clock_offset():
    admitted = DetectionStampGuard().admit(
        observation(age_ms=250.0), 10_000_000_000)
    assert admitted['source']['age_ms'] == 250.0


def test_invalid_observer_contract_cannot_advance_stamp_watermark():
    guard = DetectionStampGuard()
    bad = observation(9_950_000_000)
    bad['observe_only'] = False
    assert guard.admit(bad, 10_000_000_000) is None
    assert guard.reason == 'DETECTION_NOT_OBSERVE_ONLY'
    assert guard.last_stamp_ns is None
    assert guard.admit(observation(9_900_000_000), 10_000_000_000)


@pytest.mark.parametrize('stamp_ns,age_ms,reason', [
    (9_699_000_000, 10.0, 'SOURCE_TIMESTAMP_STALE'),
    (9_900_000_000, 301.0, 'SOURCE_TIMESTAMP_STALE'),
    (10_020_000_001, 10.0, 'SOURCE_TIMESTAMP_IN_FUTURE'),
    (9_900_000_000, -20.1, 'SOURCE_TIMESTAMP_IN_FUTURE'),
    (0, 10.0, 'DETECTION_SOURCE_STAMP_INVALID'),
])
def test_old_future_and_zero_stamp_do_not_advance_accepted_epoch(
        stamp_ns, age_ms, reason):
    guard = DetectionStampGuard()
    assert guard.admit(observation(stamp_ns, age_ms), 10_000_000_000) is None
    assert guard.reason == reason
    assert guard.last_stamp_ns is None
    assert guard.admit(observation(9_900_000_000), 10_000_000_000) is not None


@pytest.mark.parametrize('stamp', [None, {}, {'sec': True, 'nanosec': 0},
                                  {'sec': 9, 'nanosec': 1_000_000_000},
                                  {'sec': -1, 'nanosec': 0}])
def test_malformed_json_stamp_is_not_an_exception_or_valid_input(stamp):
    payload = observation()
    payload['source']['stamp'] = stamp
    guard = DetectionStampGuard()
    assert guard.admit(payload, 10_000_000_000) is None
    assert guard.reason == 'DETECTION_SOURCE_STAMP_INVALID'


@pytest.mark.parametrize('reported_age', [None, True, float('nan'), float('inf')])
def test_missing_or_invalid_reported_age_still_blocks(reported_age):
    guard = DetectionStampGuard()
    assert guard.admit(observation(age_ms=reported_age), 10_000_000_000) is None
    assert guard.reason == 'SOURCE_AGE_MISSING'


def test_duplicate_and_out_of_order_cannot_refresh_one_frame():
    guard = DetectionStampGuard()
    assert guard.admit(observation(9_900_000_000), 10_000_000_000)
    for stamp in (9_900_000_000, 9_899_000_000):
        assert guard.admit(observation(stamp), 10_010_000_000) is None
        assert guard.reason == 'DETECTION_SOURCE_STAMP_NON_INCREASING'
        assert guard.last_stamp_ns == 9_900_000_000
    assert guard.admit(observation(9_950_000_000), 10_020_000_000)


def test_tolerated_future_skew_keeps_warning_without_extra_budget():
    admitted = DetectionStampGuard().admit(
        observation(10_008_000_000, -8.0), 10_000_000_000)
    assert admitted['source']['age_ms'] == -8.0
    core = MissionSupervisorCore()
    core.update_detection(admitted, 50.0)
    result = core.evaluate(50.301)
    assert 'SOURCE_TIMESTAMP_STALE' in result['health']['blockers']


def test_receiver_clock_rollback_fails_one_call_then_requires_fresh_epoch():
    guard = DetectionStampGuard()
    assert guard.admit(observation(19_900_000_000), 20_000_000_000)
    assert guard.admit(observation(9_900_000_000), 10_000_000_000) is None
    assert guard.reason == 'DETECTION_CLOCK_BACKWARD'
    assert guard.admit(observation(10_000_000_000), 10_010_000_000)


class FakeNode:
    """Bind the real callback without creating a ROS context or publisher."""

    _decode_json = FireMissionSupervisor._decode_json
    _on_detection = FireMissionSupervisor._on_detection
    _store_detection = FireMissionSupervisor._store_detection

    def __init__(self):
        self._core = MissionSupervisorCore(fire_acquire_hold_s=0.1)
        self._detection_stamp_guard = DetectionStampGuard()
        self.ros_ns = 10_000_000_000
        self.mono = 50.0

    def _now(self):
        return self.mono

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.ros_ns))

    def send(self, payload):
        self._on_detection(SimpleNamespace(data=json.dumps(payload)))

    def alignment(self):
        self._core.update_alignment({
            'observe_only': True, 'state': 'ALIGNING_LEFT',
            'preview': {'direction': 'LEFT', 'target_angular_z_rad_s': 0.3},
            'safety': {'motion_command_publisher': False,
                       'actuator_command_publisher': False},
        }, self.mono)
        self._core.update_odom(0.0, 0.0, self.mono)


def test_actual_callback_charges_transit_then_core_charges_local_hold():
    node = FakeNode()
    node.alignment()
    node.send(observation(age_ms=10.0))  # Actual age at receipt is 200ms.
    assert node._core.evaluate(50.0)['perception']['source_age_ms'] == 200.0
    result = node._core.evaluate(50.11)
    assert result['state'] == 'FAULT'
    assert 'SOURCE_TIMESTAMP_STALE' in result['health']['blockers']
    assert 'DETECTION_TIMEOUT' not in result['health']['blockers']
    assert result['decision']['intent'] == 'REQUEST_SAFE_STOP'
    assert result['decision']['command_published'] is False


def test_actual_callback_rejects_cached_json_with_optimistic_reported_age():
    node = FakeNode()
    node.send(observation(9_000_000_000, 5.0))
    result = node._core.evaluate(node.mono)
    assert result['state'] == 'FAULT'
    assert 'SOURCE_TIMESTAMP_STALE' in result['health']['blockers']
    assert 'DETECTION_INVALID_JSON' not in result['health']['blockers']


def test_duplicate_fault_resets_candidate_before_next_timer_tick():
    node = FakeNode()
    node.alignment()
    frame = observation(9_950_000_000)
    node.send(frame)
    assert node._core.evaluate(50.0)['state'] == 'FIRE_CANDIDATE'
    node.ros_ns, node.mono = 10_010_000_000, 50.01
    node.send(frame)  # Duplicate cannot become the second confirmation.
    node.ros_ns, node.mono = 10_120_000_000, 50.12
    node.alignment()
    node.send(observation(10_110_000_000))
    assert node._core.evaluate(node.mono)['state'] == 'FIRE_CANDIDATE'
    assert node._core._candidate_since == node.mono


def test_camera_stale_health_message_stays_explicit_and_does_not_reset_guard():
    node = FakeNode()
    node.send(observation())
    node.send({'observe_only': True, 'state': 'camera_stale',
               'source': {'stamp': {'sec': 0, 'nanosec': 0}, 'age_ms': None}})
    result = node._core.evaluate(node.mono)
    assert 'DETECTION_STATE_CAMERA_STALE' in result['health']['blockers']
    assert node._detection_stamp_guard.last_stamp_ns == 9_800_000_000
    node.send(observation())
    assert 'DETECTION_SOURCE_STAMP_NON_INCREASING' in node._core.evaluate(
        node.mono)['health']['blockers']


def test_malformed_json_still_uses_existing_parse_error_blocker():
    node = FakeNode()
    node._on_detection(SimpleNamespace(data='{'))
    result = node._core.evaluate(node.mono)
    assert 'DETECTION_INVALID_JSON' in result['health']['blockers']
