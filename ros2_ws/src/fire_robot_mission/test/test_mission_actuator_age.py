"""ACT transit/cache age and proof invalidation through the real ROS callback."""

import json
from types import SimpleNamespace

import pytest

from fire_robot_mission.actuator_freshness import ActuatorStampGuard
from fire_robot_mission.mission_supervisor import (
    FireMissionSupervisor, MissionSupervisorCore,
)


GOAL = 'ab' * 16
ORIGIN = 100_000_000_000


def no_fire(core, now):
    core.update_detection({
        'state': 'ok', 'observe_only': True, 'source': {'age_ms': 10.0},
        'fire': {'detected_now': False, 'confirmed': False},
    }, now)
    core.update_odom(0.0, 0.0, now)


def prepared_core():
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1)
    for now in (0.0, 0.11):
        core.update_goal_statuses([(GOAL, 2)], now)
        core.update_detection({
            'state': 'ok', 'observe_only': True, 'source': {'age_ms': 10.0},
            'fire': {'detected_now': True, 'confirmed': True},
        }, now)
        core.update_alignment({
            'observe_only': True, 'state': 'ALIGNING_LEFT',
            'preview': {'direction': 'LEFT', 'target_angular_z_rad_s': 0.3},
            'safety': {'motion_command_publisher': False,
                       'actuator_command_publisher': False},
        }, now)
        core.evaluate(now)
    core.update_goal_statuses([(GOAL, 5)], 0.12)
    no_fire(core, 0.12)
    assert core.evaluate(0.12)['state'] == 'TARGET_LOST'
    return core


def off():
    return {'schema_version': 1, 'mode': 'COMMAND_TELEMETRY_ONLY',
            'pump_pwm_commanded_on': False, 'servo_pwm_commanded_on': False,
            'servo_phase': 'IDLE', 'physical_pump_or_nozzle_verified': False}


def review(core, now):
    core.update_review({
        'schema_version': 1, 'mode': 'SHADOW_ONLY',
        'state': 'NO_FIRE_EVIDENCE', 'goal_uuid': GOAL,
        'pulse_index': 1, 'blockers': [],
        'safety': {'motion_command_publisher': False,
                   'pump_command_publisher': False, 'nav2_action_client': False,
                   'actuation_authorized': False},
    }, now)


def ir_absent(core, now):
    core.update_ir({
        'schema_version': 1, 'mode': 'OBSERVE_ONLY', 'observe_only': True,
        'zone': 'UNKNOWN', 'ir_band': 'NO_FLAME_SIGNATURE',
        'stable_samples': 5, 'ir_receive_age_ms': 20.0,
        'blockers': ['NO_IR_FLAME_SIGNATURE', 'FIRE_NOT_CONFIRMED'],
    }, now)


class CallbackHarness:
    """Bind actual callback without constructing a Node or ROS context."""

    _on_actuator = FireMissionSupervisor._on_actuator

    def __init__(self):
        self._core = prepared_core()
        self._actuator_stamp_guard = ActuatorStampGuard(
            max_age_ms=350, max_future_skew_ms=20)
        self.actuator_state_topic = '/fire_sensors/actuator_state'
        self.now = 1.0
        self.source_ok = True

    def _now(self):
        return self.now

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=ORIGIN + round(self.now * 1e9)))

    def _single_expected_publisher(self, topic, node_name):
        assert topic == '/fire_sensors/actuator_state'
        assert node_name == 'serial_bridge_node'
        return self.source_ok

    def receive(self, *, age_ms=10.0, seq=10, epoch=1, instance='ab' * 16):
        stamp = ORIGIN + round(self.now * 1e9) - round(age_ms * 1e6)
        sec, nanosec = divmod(stamp, 1_000_000_000)
        payload = {**off(), 'receive_age_ms': 10.0, 'esp_ms': 1000 + seq,
                   'source': {'stamp': {'sec': sec, 'nanosec': nanosec},
                              'bridge_instance_id': instance,
                              'serial_epoch': epoch, 'frame_seq': seq}}
        self._on_actuator(SimpleNamespace(data=json.dumps(payload)))


def test_transit_plus_cache_expires_review_proof_before_receipt_ttl():
    node = CallbackHarness()
    node.receive(age_ms=340)
    no_fire(node._core, 1.0)
    review(node._core, 1.0)
    assert node._core._reviewed_clear(1.0, False) is True
    result = node._core.evaluate(1.02)
    assert result['state'] == 'TARGET_LOST'  # Total ACT age is now 360ms.
    assert result['lifecycle']['actuator_source_blocker'] == 'ACTUATOR_SOURCE_STALE'
    node.now = 1.03
    node.receive(seq=11)
    no_fire(node._core, 1.03)
    assert node._core.evaluate(1.03)['state'] == 'IDLE'


def test_already_old_off_at_receipt_cannot_clear_even_with_a_fresh_review():
    node = CallbackHarness()
    node.receive(age_ms=351)
    no_fire(node._core, node.now)
    review(node._core, node.now)
    result = node._core.evaluate(node.now)
    assert result['state'] == 'TARGET_LOST'
    assert result['lifecycle']['actuator_source_blocker'] == 'ACTUATOR_SOURCE_STALE'
    assert node._actuator_stamp_guard.last_stamp_ns is None


def test_invalid_between_timer_ticks_revokes_old_hold_and_review():
    node = CallbackHarness()
    node.receive()
    no_fire(node._core, node.now)
    ir_absent(node._core, node.now)
    assert node._core.evaluate(node.now)['state'] == 'TARGET_LOST'
    assert node._core._pre_spray_clear_since == 1.0
    review(node._core, 1.0)
    node.now = 1.01
    node._on_actuator(SimpleNamespace(data='{invalid json'))
    assert node._core._pre_spray_clear_since is None
    assert node._core._review is None
    node.now = 1.02
    node.receive(seq=11)
    no_fire(node._core, node.now)
    assert node._core.evaluate(node.now)['state'] == 'TARGET_LOST'
    assert node._core._pre_spray_clear_since == 1.02


@pytest.mark.parametrize('changed', [{'epoch': 2}, {'instance': 'cd' * 16}])
def test_new_source_context_needs_new_absence_and_review_proofs(changed):
    node = CallbackHarness()
    node.receive()
    no_fire(node._core, node.now)
    ir_absent(node._core, node.now)
    node._core.evaluate(node.now)
    review(node._core, node.now)
    node.now = 1.02
    node.receive(seq=11, **changed)
    assert node._core._review is None
    assert node._core._pre_spray_clear_since is None
    no_fire(node._core, node.now)
    assert node._core.evaluate(node.now)['state'] == 'TARGET_LOST'
    assert node._core._pre_spray_clear_since == 1.02


def test_source_change_does_not_reset_pump_history_or_goal_cancel_binding():
    node = CallbackHarness()
    node._core.update_pump({'state': 'FIRING', 'pump_on': True,
                            'goal_uuid': GOAL}, node.now)
    node.receive()
    node.now = 1.02
    node.receive(seq=11, epoch=2)
    assert node._core._pump_started_uuid == GOAL
    assert node._core._acquired_goal_uuid == GOAL
    assert node._core._acquired_goal_canceled is True


def test_wrong_publisher_revokes_cache_without_advancing_source_watermark():
    node = CallbackHarness()
    node.source_ok = False
    node.receive()
    assert node._core._actuator is None
    assert node._actuator_stamp_guard.last_stamp_ns is None
    result = node._core.evaluate(node.now)
    assert result['lifecycle']['actuator_source_blocker'] == 'ACTUATOR_PUBLISHER_INVALID'


@pytest.mark.parametrize('data', [json.dumps(off()), '[]', 'null'])
def test_legacy_missing_metadata_and_non_object_json_have_no_fallback(data):
    node = CallbackHarness()
    node._on_actuator(SimpleNamespace(data=data))
    assert node._core._actuator is None
    assert node._core.evaluate(node.now)['state'] != 'IDLE'


def test_relabelled_duplicate_sequence_cannot_refresh_off_cache():
    node = CallbackHarness()
    node.receive()
    node.now = 1.02
    node.receive()  # New stamp, same ACT sequence.
    no_fire(node._core, node.now)
    review(node._core, node.now)
    result = node._core.evaluate(node.now)
    assert result['state'] == 'TARGET_LOST'
    assert result['lifecycle']['actuator_source_blocker'] == 'ACTUATOR_SEQUENCE_NON_INCREASING'


def test_pre_spray_hold_survives_off_report_gaps_but_requires_fresh_total_age_at_end():
    core = prepared_core()
    for now in (1.0, 1.8):
        no_fire(core, now)
        ir_absent(core, now)
        if now == 1.0:
            core.update_actuator(off(), now, source_age_ms=340.0)
        assert core.evaluate(now)['state'] == 'TARGET_LOST'
    core.update_actuator(off(), 2.51, source_age_ms=340.0)
    no_fire(core, 2.53)
    ir_absent(core, 2.53)
    assert core.evaluate(2.53)['state'] == 'TARGET_LOST'
    assert core._pre_spray_clear_since == 1.0
    core.update_actuator(off(), 2.54, source_age_ms=10.0)
    no_fire(core, 2.54)
    assert core.evaluate(2.54)['state'] == 'IDLE'


def test_invalid_or_on_report_before_valid_off_cannot_reuse_cached_review():
    core = prepared_core()
    review(core, 1.0)
    core.update_actuator({**off(), 'pump_pwm_commanded_on': True}, 1.01)
    core.update_actuator(off(), 1.02)
    no_fire(core, 1.02)
    assert core.evaluate(1.02)['state'] == 'TARGET_LOST'
    review(core, 1.03)
    assert core.evaluate(1.03)['state'] == 'IDLE'


def test_future_local_receipt_is_not_a_fresh_off_proof():
    core = prepared_core()
    core.update_actuator(off(), 1.1)
    review(core, 1.0)
    no_fire(core, 1.0)
    assert core.evaluate(1.0)['state'] == 'TARGET_LOST'
