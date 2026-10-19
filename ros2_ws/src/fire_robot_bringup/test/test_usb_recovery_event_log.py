"""Bounded USB recovery evidence without periodic diagnostic logging."""

import json

from fire_robot_bringup.serial_transport_core import SerialTransportCore


class RecordingLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class RecordingCore(SerialTransportCore):
    def __init__(self):
        self.logger = RecordingLogger()
        self.init_transport('/dev/null', 115200, 8.0, 1.0, None,
                            diagnostics_enabled=False)
        self.session_ready = True
        self.session_epoch = 1

    def get_logger(self):
        return self.logger


def _usb(recoveries, snapshot=True):
    frame = {'type': 'USB', 'recoveries': recoveries, 'esp_ms': 12345}
    if snapshot:
        frame.update(fault_snapshot_valid=1, fault_cdc_connected=1,
                     fault_cdc_line_state=3,
                     fault_raw_write_available=0, fault_cached_dtr=1)
    return frame


def _events(core):
    return [json.loads(item.split('esp_usb_recovery_event ', 1)[1])
            for item in core.logger.warnings]


def test_only_recovery_counter_changes_are_logged_when_diagnostics_off():
    core = RecordingCore()
    core.handle_frame({'type': 'USB', 'esp_ms': 1})
    core.handle_frame(_usb(0, snapshot=False))
    core.handle_frame(_usb(0, snapshot=False))
    assert _events(core) == []

    core.handle_frame(_usb(1))
    core.handle_frame(_usb(1))
    event, = _events(core)
    assert event['kind'] == 'counter_advanced'
    assert event['previous_recoveries'] == 0
    assert event['usb']['fault_raw_write_available'] == 0
    assert core.latest_usb_diagnostic[0]['recoveries'] == 1
    assert core.telemetry_healthy is False


def test_initial_recovery_is_labeled_and_log_count_is_bounded():
    core = RecordingCore()
    core.handle_frame(_usb(4))
    assert _events(core)[0]['kind'] == 'before_first_usb_frame'
    assert _events(core)[0]['previous_recoveries'] is None
    for counter in range(5, 20):
        core.handle_frame(_usb(counter))
    assert len(_events(core)) == 8


def test_counter_reset_allows_next_recovery_without_false_reset_event():
    core = RecordingCore()
    core.handle_frame(_usb(3))
    core.handle_frame(_usb(0, snapshot=False))
    assert len(_events(core)) == 1
    core.session_epoch = 2
    core.handle_frame(_usb(1))
    assert _events(core)[1]['kind'] == 'counter_advanced'
    assert _events(core)[1]['epoch'] == 2
