"""Offline checks for opt-in USB fault evidence; never opens a real port."""

import json
import time
from types import SimpleNamespace

import pytest

from fire_robot_bringup.serial_fault_trace import classify_esp_uptime
from fire_robot_bringup.serial_process import ProcessTransport
from fire_robot_bringup import serial_transport_core as core


def _sample(esp_ms, mono_ms):
    return {'esp_ms': esp_ms, 'receive_mono_ns': mono_ms * 1_000_000}


@pytest.mark.parametrize('before,after,label', [
    (_sample(100_000, 100_000), _sample(101_000, 101_000),
     'continuous_consistent'),
    (_sample(0xffffffff - 200, 100_000), _sample(299, 100_500),
     'wrap_consistent'),
    (_sample(100_000, 100_000), _sample(400, 101_000),
     'reset_suspected'),
    (_sample(100_000, 100_000), _sample(100_100, 110_000),
     'uptime_inconsistent'),
    (None, _sample(100, 100), 'missing_sample'),
    (_sample(True, 100), _sample(101, 101), 'invalid_sample'),
])
def test_uptime_evidence_is_cautious(before, after, label):
    assert classify_esp_uptime(before, after) == label


class _CaptureLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)

    def error(self, message):
        self.messages.append(message)


class _FakeSerial:
    closed = False

    def cancel_read(self):
        pass

    def cancel_write(self):
        pass

    def reset_output_buffer(self):
        pass

    def close(self):
        self.closed = True


def _transport(trace_events):
    config = {
        'port': '/tmp/qa_no_hardware', 'baudrate': 115200,
        'first_state_timeout_s': 8.0, 'pump_command_timeout_s': 0.7,
        'usb_fault_trace_events': trace_events,
    }
    worker = ProcessTransport(config, lambda *args, **kwargs: None)
    worker.logger = _CaptureLogger()
    worker.session_ready = True
    worker.session_epoch = 2
    worker._session_state = 'WAIT_FIRST_STATE'
    return worker


def test_first_state_records_epoch_pair_without_changing_health_rule():
    worker = _transport(1)
    before = _sample(100_000, 100_000)
    worker._fault_trace_pending = {
        'id': 1, 'fault_epoch': 1, 'last_state': before,
    }
    worker.handle_frame({'type': 'USB', 'esp_ms': 200})
    assert not worker.telemetry_healthy
    worker.handle_frame({'type': 'STATE', 'esp_ms': 300})
    assert worker.telemetry_healthy
    assert worker._session_state == 'HEALTHY'
    assert worker._fault_trace_pending is None
    events = [json.loads(line.split('serial_fault_trace ', 1)[1])
              for line in worker.logger.messages]
    assert len(events) == 1
    assert events[0]['phase'] == 'first_state'
    assert events[0]['before'] == before
    assert events[0]['after']['esp_ms'] == 300
    assert events[0]['uptime_evidence'] == classify_esp_uptime(
        before, events[0]['after'])


def test_fault_records_close_stages_after_failing_closed():
    worker = _transport(1)
    worker._session_state = 'HEALTHY'
    worker.telemetry_healthy = True
    worker._fault_trace_last_state = _sample(100_000, 100_000)
    handle = _FakeSerial()
    worker.ser = handle

    worker._quarantine_handle(handle, 'test:fake')
    task = worker.close_ownership['task']
    assert task is not None
    task.join(timeout=1.0)
    assert not task.is_alive()
    assert handle.closed
    assert not worker.telemetry_healthy
    assert worker._session_state == 'CLOSING'

    events = [json.loads(line.split('serial_fault_trace ', 1)[1])
              for line in worker.logger.messages
              if line.startswith('serial_fault_trace ')]
    by_phase = {event['phase']: event for event in events}
    assert set(by_phase) == {'fault', 'close_begin', 'close_end'}
    assert {event['id'] for event in events} == {1}
    assert by_phase['fault']['last_state']['esp_ms'] == 100_000
    assert by_phase['close_end']['result'] == 'ok'
    assert by_phase['close_end']['close_call_ms'] is not None


def test_default_off_records_no_state_samples_or_fault_events(monkeypatch):
    worker = _transport(0)

    def unexpected_clock():
        raise AssertionError('fault trace OFF sampled wall clock')

    monkeypatch.setattr(core, 'time', SimpleNamespace(
        monotonic=time.monotonic, time_ns=unexpected_clock))
    worker.handle_frame({'type': 'STATE', 'esp_ms': 100})
    assert worker.telemetry_healthy
    assert worker._fault_trace_last_state is None
    assert worker.logger.messages == []


def test_boot_identity_logs_without_admitting_telemetry():
    worker = _transport(0)
    boot = {
        'type': 'BOOT', 'esp_ms': 12345, 'reset_reason': 9,
        'elf_sha256': 'a' * 64,
    }
    worker.handle_frame(boot)
    worker.handle_frame(boot)
    assert not worker.telemetry_healthy
    assert worker.latest_state is None
    assert worker._session_state == 'WAIT_FIRST_STATE'
    events = [json.loads(line.split('esp_boot_identity ', 1)[1])
              for line in worker.logger.messages]
    assert len(events) == 1
    assert events[0]['reset_reason'] == 9
    assert events[0]['elf_sha256'] == 'a' * 64


@pytest.mark.parametrize('events', [-1, 17, True, 1.5])
def test_trace_limit_rejects_invalid_values(events):
    with pytest.raises(ValueError, match='fault_trace_events'):
        _transport(events)
