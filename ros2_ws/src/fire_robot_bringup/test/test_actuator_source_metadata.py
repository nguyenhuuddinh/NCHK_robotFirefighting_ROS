"""ACT provenance from the real publish callback, without ROS/serial init."""

import json
import threading
from types import SimpleNamespace

import pytest

from fire_robot_bringup import serial_bridge_node as bridge


ROS_ORIGIN = 1_000_000_000_000


def frame(**overrides):
    return {
        'seq': 23, 'esp_ms': 4567, 'pump_pwm': 0, 'pump_ready': 1,
        'servo_phase': 0, 'servo_angle_deg': 255, 'servo_pwm': 0,
        'home_count': 4, **overrides,
    }


def harness(monkeypatch, *, age=0.125):
    clock = SimpleNamespace(mono=10.0 + age, ros_ns=ROS_ORIGIN + int(age * 1e9))
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: clock.mono)
    messages, results, warnings = [], [], []
    node = SimpleNamespace(
        enable_diagnostics=False, node_destroyed=False,
        state_lock=threading.Lock(), publish_lock=threading.Lock(),
        latest_state=None, latest_env=None, latest_ir=None,
        latest_actuator=(frame(), 7, 10.0),
        running=True, session_ready=True, telemetry_healthy=True, session_epoch=7,
        _actuator_source_instance='ab' * 16,
        actuator_pub=SimpleNamespace(publish=messages.append),
        _record_publish_result=lambda **item: results.append(item),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=clock.ros_ns)),
        get_logger=lambda: SimpleNamespace(warn=warnings.append),
    )
    return node, clock, messages, results, warnings


def publish(node):
    bridge.SerialBridgeNode.telemetry_publish_callback(node)


def stamp_ns(payload):
    stamp = payload['source']['stamp']
    return stamp['sec'] * 1_000_000_000 + stamp['nanosec']


def test_stamp_is_backdated_to_receipt_with_identity_and_existing_flags_preserved(monkeypatch):
    node, _, messages, results, warnings = harness(monkeypatch)
    publish(node)
    payload = json.loads(messages[0].data)
    assert stamp_ns(payload) == ROS_ORIGIN
    assert payload['source']['bridge_instance_id'] == 'ab' * 16
    assert payload['source']['serial_epoch'] == 7
    assert payload['source']['frame_seq'] == 23
    assert payload['schema_version'] == 1
    assert payload['mode'] == 'COMMAND_TELEMETRY_ONLY'
    assert payload['esp_ms'] == 4567  # ESP uptime is not a ROS timestamp.
    assert payload['receive_age_ms'] == 125.0
    assert payload['pump_pwm_commanded_on'] is False
    assert payload['servo_pwm_commanded_on'] is False
    assert payload['servo_phase'] == 'IDLE'
    assert payload['physical_pump_or_nozzle_verified'] is False
    assert node.latest_actuator is None
    assert results == [{'published': 1}]
    assert warnings == []


def test_cached_frame_cannot_get_a_younger_stamp_on_a_later_publish(monkeypatch):
    node, clock, messages, _, _ = harness(monkeypatch)
    publish(node)
    first = json.loads(messages[-1].data)
    clock.mono += 0.125
    clock.ros_ns += 125_000_000
    node.latest_actuator = (frame(), 7, 10.0)  # Same original frame/receipt.
    publish(node)
    second = json.loads(messages[-1].data)
    assert stamp_ns(first) == stamp_ns(second) == ROS_ORIGIN
    assert first['source'] == second['source']
    assert second['receive_age_ms'] == 250.0


def test_new_receipt_advances_stamp_and_exposes_sequence(monkeypatch):
    node, clock, messages, _, _ = harness(monkeypatch)
    publish(node)
    clock.mono = 10.25
    clock.ros_ns = ROS_ORIGIN + 250_000_000
    node.latest_actuator = (frame(seq=26, esp_ms=4692), 7, 10.125)
    publish(node)
    first, second = [json.loads(msg.data) for msg in messages]
    assert stamp_ns(second) - stamp_ns(first) == 125_000_000
    assert second['source']['frame_seq'] == 26
    assert second['source']['bridge_instance_id'] == first['source']['bridge_instance_id']


def test_serial_reconnect_and_bridge_restart_have_distinct_source_identities(monkeypatch):
    node, _, messages, _, _ = harness(monkeypatch)
    publish(node)
    node.session_epoch = 8
    node.latest_actuator = (frame(seq=1), 8, 10.0)
    publish(node)
    node._actuator_source_instance = 'cd' * 16
    node.session_epoch = 7  # Counter may repeat in a new bridge instance.
    node.latest_actuator = (frame(seq=23), 7, 10.0)
    publish(node)
    sources = [json.loads(msg.data)['source'] for msg in messages]
    assert sources[0]['bridge_instance_id'] == sources[1]['bridge_instance_id']
    assert sources[0]['serial_epoch'] != sources[1]['serial_epoch']
    assert sources[0]['serial_epoch'] == sources[2]['serial_epoch']
    assert sources[0]['bridge_instance_id'] != sources[2]['bridge_instance_id']


@pytest.mark.parametrize('age', [0.5, 0.625, -0.125])
def test_expired_or_future_monotonic_receipt_never_publishes_actuator_state(monkeypatch, age):
    node, _, messages, results, warnings = harness(monkeypatch, age=age)
    publish(node)
    assert messages == []
    assert results == [{'dropped': 1}]
    assert warnings == []


@pytest.mark.parametrize('changed', [
    {'running': False}, {'session_ready': False},
    {'telemetry_healthy': False}, {'session_epoch': 8},
])
def test_existing_link_health_and_epoch_guards_still_block(monkeypatch, changed):
    node, _, messages, results, _ = harness(monkeypatch)
    for name, value in changed.items():
        setattr(node, name, value)
    publish(node)
    assert messages == []
    assert results == [{'dropped': 1}]


@pytest.mark.parametrize('ros_ns', [0, 100_000_000])
def test_unready_ros_clock_never_creates_zero_or_negative_source_stamp(monkeypatch, ros_ns):
    node, clock, messages, results, warnings = harness(monkeypatch)
    clock.ros_ns = ros_ns
    publish(node)
    assert messages == []
    assert results == [{'failed': 1}]
    assert len(warnings) == 1
    assert 'ACT source clock is not ready' in warnings[0]


def test_pump_on_and_servo_phase_remain_command_telemetry(monkeypatch):
    node, _, messages, _, _ = harness(monkeypatch)
    node.latest_actuator = (frame(pump_pwm=1, servo_phase=1,
                                  servo_angle_deg=90, servo_pwm=1), 7, 10.0)
    publish(node)
    payload = json.loads(messages[0].data)
    assert payload['pump_pwm_commanded_on'] is True
    assert payload['servo_phase'] == 'SWEEPING'
    assert payload['servo_commanded_angle_deg'] == 90
    assert payload['physical_pump_or_nozzle_verified'] is False


def test_no_actuator_frame_means_no_clock_read_or_new_work(monkeypatch):
    node, _, messages, results, _ = harness(monkeypatch)
    node.latest_actuator = None

    def unexpected_clock():
        raise AssertionError('No ACT frame: do not read its clock')

    node.get_clock = unexpected_clock
    publish(node)
    assert messages == []
    assert results == []
