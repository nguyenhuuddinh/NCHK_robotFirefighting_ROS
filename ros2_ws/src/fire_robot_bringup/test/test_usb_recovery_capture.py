"""Focused gates for the optional ESP USB recovery capture."""

import time
import threading
from types import SimpleNamespace

import pytest

import fire_robot_bringup.serial_bridge_node as serial_bridge_node
from fire_robot_bringup.serial_bridge_node import (
    SerialBridgeNode, UsbRecoveryCapture)


@pytest.mark.parametrize('duration', [-1, 181, 0.5, True, '10'])
def test_capture_rejects_invalid_duration(duration):
    with pytest.raises(ValueError, match='usb_recovery_capture_s'):
        UsbRecoveryCapture(duration)


def test_capture_off_does_not_record_or_read_clock(monkeypatch):
    capture = UsbRecoveryCapture(0)

    def unexpected_clock():
        raise AssertionError('capture OFF read clock')

    monkeypatch.setattr(serial_bridge_node, 'time', SimpleNamespace(
        monotonic=unexpected_clock, time_ns=unexpected_clock))
    assert capture.record({'type': 'USB'}, 3, 1.0) is None
    assert capture.finish_if_due() is None


def test_capture_retains_esp_evidence_and_stops_at_duration():
    capture = UsbRecoveryCapture(10)
    frame = {'type': 'USB', 'esp_ms': 12345, 'recoveries': 2,
             'reason': 1, 'task_gap_ms': 40, 'tx_event_age_ms': 700}
    event = capture.record(frame, 7, time.monotonic())
    assert event['epoch'] == 7
    assert event['usb'] == frame
    assert event['pi_wall_ns'] > 0
    assert event['receive_mono_ns'] > 0
    assert capture.count == 1
    capture.deadline = 0
    assert capture.finish_if_due() == {
        'reason': 'duration_elapsed', 'frames': 1}
    assert capture.record(frame, 7, time.monotonic()) is None
    assert capture.finish_if_due() is None


def test_capture_frame_limit_is_bounded():
    capture = UsbRecoveryCapture(180)
    frame = {'type': 'USB', 'recoveries': 1}
    for _ in range(capture.MAX_FRAMES):
        assert capture.record(frame, 1, 0.0) is not None
    assert capture.record(frame, 1, 0.0) is None
    assert capture.finish_if_due() == {
        'reason': 'frame_limit', 'frames': capture.MAX_FRAMES}


def test_capture_logs_only_current_epoch_without_refreshing_state():
    messages = []
    frame = {'type': 'USB', 'esp_ms': 200, 'recoveries': 1, 'reason': 1}
    node = SimpleNamespace(
        transport_process=None,
        enable_diagnostics=False,
        _usb_capture=UsbRecoveryCapture(30),
        publish_lock=threading.Lock(),
        state_lock=threading.Lock(),
        node_destroyed=False,
        running=True,
        session_ready=True,
        telemetry_healthy=False,
        session_epoch=4,
        last_state_time_mono=0.0,
        latest_state=None,
        latest_env=None,
        latest_ir=None,
        latest_actuator=None,
        latest_usb_diagnostic=(frame, 3, time.monotonic()),
        get_logger=lambda: SimpleNamespace(info=messages.append),
    )
    SerialBridgeNode.telemetry_publish_callback(node)
    assert not messages
    assert node.last_state_time_mono == 0.0
    assert not node.telemetry_healthy

    node.latest_usb_diagnostic = (frame, 4, time.monotonic())
    SerialBridgeNode.telemetry_publish_callback(node)
    assert len(messages) == 1
    assert messages[0].startswith('esp_usb_capture ')
    assert node.last_state_time_mono == 0.0
    assert not node.telemetry_healthy

    node._usb_capture.deadline = 0
    SerialBridgeNode.telemetry_publish_callback(node)
    assert messages[-1].startswith('esp_usb_capture_end ')
