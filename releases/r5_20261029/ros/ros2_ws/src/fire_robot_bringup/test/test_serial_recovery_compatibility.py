"""N8R2 BOOT/USB compatibility; fake serial only, never actuates hardware."""

import json
import threading
from types import SimpleNamespace

import pytest

from fire_robot_bringup import serial_bridge_node as bridge
from fire_robot_bringup.serial_protocol import SerialProtocolV2, crc16_ccitt_false
from fire_robot_bringup.serial_recovery_diagnostics import SerialRecoveryDiagnostics


def wire(payload):
    raw = payload.encode('ascii')
    return b'@' + raw + ('*%04X\n' % crc16_ccitt_false(raw)).encode('ascii')


def boot(seq=1, digest='ab' * 32):
    return f'BOOT,2,{seq},1000,1,{digest}'


def usb(seq=2, recoveries=1, snapshot=True):
    value = f'USB,2,{seq},1100,{recoveries},900,1,0,66,0,12,20,1200,1,2,3,0'
    return value + (',1,1,3,0,1' if snapshot else '')


def parsed(payload):
    return list(SerialProtocolV2().parse_chunk(wire(payload)))[0]


@pytest.mark.parametrize('payload', [boot(), boot(digest='unknown'),
                                    usb(snapshot=False), usb(snapshot=True)])
def test_n8r2_diagnostic_frames_decode(payload):
    parser = SerialProtocolV2()
    frame, = parser.parse_chunk(wire(payload))
    assert frame['type'] in ('BOOT', 'USB')
    assert parser.parse_fail_count == parser.gap_count == 0
    if frame['type'] == 'USB' and len(payload.split(',')) == 22:
        assert frame['fault_snapshot_valid'] == 1
        assert frame['fault_raw_write_available'] == 0


@pytest.mark.parametrize('payload', [
    boot(digest='ab' * 31), boot(digest='g' * 64),
    boot().replace(',1000,1,', ',4294967296,1,'),
    boot().replace(',1000,1,', ',1000,-1,'),
    usb() + ',0', usb().rsplit(',', 1)[0],
    usb().replace(',900,1,0,66,', ',900,3,0,66,'),
    usb().replace(',0,66,0,', ',67,66,0,'),
    usb().replace(',0,66,0,', ',0,193,0,'),
    usb().replace(',20,1200,1,', ',20,1200,2,'),
    usb().replace(',1100,1,', ',4294967296,1,'),
    usb().replace(',1,1,3,0,1', ',0,1,3,0,1'),
    usb().replace(',1,1,3,0,1', ',1,2,3,0,1'),
    usb().replace(',1,1,3,0,1', ',1,1,4,0,1'),
    usb().replace(',1,1,3,0,1', ',1,1,3,0,2'),
    usb().replace(',1,1,3,0,1', ',1,1,3,4294967296,1'),
    usb().replace(',1100,1,', ',1100,+1,'),
])
def test_invalid_diagnostics_do_not_commit_sequence(payload):
    parser = SerialProtocolV2()
    assert list(parser.parse_chunk(wire(payload))) == []
    assert parser.rx_seq == -1
    assert parser.parse_fail_count + parser.overflow_count > 0
    assert len(list(parser.parse_chunk(wire(boot())))) == 1


def test_shared_sequence_crc_fragmentation_and_reconnect():
    parser = SerialProtocolV2()
    messages = [boot(0xffffffff), usb(0),
                'STATE,2,1,1200,0.000,0.000,0.000,0.000,0.000,0.000',
                'ACT,2,2,1200,0,0,255,0,1,1',
                'IR,2,3,0,100,200,300,1', 'ENV,2,4,0,0.0,0.0,0.0,0']
    chunk = b''.join(wire(item) for item in messages)
    frames = []
    for index in range(0, len(chunk), 7):
        frames.extend(parser.parse_chunk(chunk[index:index + 7]))
    assert [frame['type'] for frame in frames] == [
        'BOOT', 'USB', 'STATE', 'ACT', 'IR', 'ENV']
    assert parser.gap_count == 0
    assert list(parser.parse_chunk(wire(usb(4)))) == []
    assert parser.dup_count == 1
    corrupted = wire(boot(5)).replace(b'1000', b'1001')
    assert list(parser.parse_chunk(corrupted)) == []
    assert parser.crc_fail_count == 1
    assert parser.rx_seq == 4
    parser.reset_parser()
    assert list(parser.parse_chunk(wire(boot(0))))[0]['seq'] == 0
    oversized = boot(1) + ',0' * 100
    assert list(parser.parse_chunk(wire(oversized))) == []
    assert parser.overflow_count == 1


def test_recovery_counter_is_event_only_and_bounded_across_epochs():
    collector = SerialRecoveryDiagnostics()
    collector.record(parsed(usb(recoveries=0)), 1, 1.0)
    assert collector.drain(1, True) == ()
    collector.record(parsed(usb(recoveries=1)), 1, 2.0)
    event, = collector.drain(1, True)
    assert event['previous_recoveries'] == 0
    assert event['kind'] == 'counter_advanced'
    collector.record(parsed(usb(recoveries=1)), 2, 3.0)
    assert collector.drain(2, True) == ()
    for epoch in range(3, 100):
        collector.record(parsed(boot()), epoch, float(epoch))
        collector.record(parsed(usb(recoveries=epoch)), epoch, float(epoch))
    assert collector.captured == collector.EVENT_LIMIT == 8
    assert len(collector.drain(99, True)) <= 2
    assert collector.drain(99, True) == ()


def test_burst_coalesces_to_latest_recovery_without_a_queue():
    collector = SerialRecoveryDiagnostics()
    collector.record(parsed(boot()), 1, 0.5)
    for counter in range(1, 6):
        collector.record(parsed(usb(recoveries=counter)), 1, float(counter))
    events = collector.drain(1, True)
    assert len(events) == 2
    assert events[1]['frame']['recoveries'] == 5
    assert events[1]['receive_mono_ns'] == 5_000_000_000
    assert collector.drain(1, True) == ()


def test_counter_reset_and_first_positive_observation_are_not_reboots():
    collector = SerialRecoveryDiagnostics()
    collector.record(parsed(usb(recoveries=5)), 1, 1.0)
    event, = collector.drain(1, True)
    assert event['kind'] == 'before_first_usb_frame'
    collector.record(parsed(usb(recoveries=0)), 2, 2.0)
    assert collector.drain(2, True) == ()
    collector.record(parsed(usb(recoveries=1)), 2, 3.0)
    event, = collector.drain(2, True)
    assert event['previous_recoveries'] == 0
    assert 'reboot' not in event


def test_boot_coalesces_per_epoch_and_old_or_closed_events_are_consumed():
    collector = SerialRecoveryDiagnostics()
    collector.record(parsed(boot()), 1, 1.0)
    collector.record(parsed(boot(2)), 1, 1.1)
    assert collector.captured == 1
    assert collector.drain(2, True) == ()
    collector.record(parsed(boot()), 2, 2.0)
    assert collector.drain(2, False) == ()
    assert collector.drain(2, True) == ()


class Harness:
    handle_frame = bridge.SerialBridgeNode.handle_frame
    telemetry_publish_callback = bridge.SerialBridgeNode.telemetry_publish_callback
    cmd_vel_callback = bridge.SerialBridgeNode.cmd_vel_callback
    fire_target_callback = bridge.SerialBridgeNode.fire_target_callback
    pump_cmd_callback = bridge.SerialBridgeNode.pump_cmd_callback
    _transition_state = bridge.SerialBridgeNode._transition_state

    def __init__(self, enabled):
        self.state_lock = threading.Lock()
        self.publish_lock = threading.Lock()
        self._recovery_diagnostics = SerialRecoveryDiagnostics() if enabled else None
        self.node_destroyed = False
        self.running = self.session_ready = True
        self.session_epoch = 1
        self._session_state = 'WAIT_FIRST_STATE'
        self.telemetry_healthy = False
        self.last_state_time_mono = 10.0
        self.reconnect_backoff = 0.5
        self.next_reconnect_mono = 12.0
        self.latest_state = self.latest_env = self.latest_ir = self.latest_actuator = None
        self.pending_cmd = self.pending_fire = self.pending_pump = None
        self.cmd_drop_count = 0
        self.messages = []

    def get_logger(self):
        return self

    def info(self, message):
        assert not self.state_lock.locked()
        self.messages.append(message)


@pytest.mark.parametrize('enabled', [False, True])
def test_diagnostics_cannot_refresh_health_or_admit_active_commands(enabled):
    node = Harness(enabled)
    node.handle_frame(parsed(boot()))
    node.handle_frame(parsed(usb()))
    assert node.telemetry_healthy is False
    assert node.last_state_time_mono == 10.0
    assert node.reconnect_backoff == 0.5
    assert node.next_reconnect_mono == 12.0
    assert node._session_state == 'WAIT_FIRST_STATE'
    node.cmd_vel_callback(SimpleNamespace(
        linear=SimpleNamespace(x=0.1), angular=SimpleNamespace(z=0.0)))
    node.fire_target_callback(SimpleNamespace(x=0.1, y=0.0))
    node.pump_cmd_callback(SimpleNamespace(data=True))
    assert node.cmd_drop_count == 3
    assert node.pending_cmd is node.pending_fire is node.pending_pump is None
    node.telemetry_publish_callback()
    assert len(node.messages) == (2 if enabled else 0)
    if enabled:
        assert {json.loads(msg.split(' ', 1)[1])['event'] for msg in node.messages} == {
            'esp_boot_identity', 'esp_usb_recovery_event'}


def test_only_state_recovers_health_after_diagnostics_saturate(monkeypatch):
    node = Harness(True)
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: 20.0)
    node.telemetry_healthy = True
    node._session_state = 'HEALTHY'
    for count in range(1, 30):
        node.handle_frame(parsed(usb(recoveries=count)))
    assert node.last_state_time_mono == 10.0
    assert node._recovery_diagnostics.captured == 8
    node.telemetry_healthy = False
    node._session_state = 'WAIT_FIRST_STATE'
    frame = {'type': 'STATE', 'esp_ms': 999, 'seq': 99}
    node.handle_frame(frame)
    assert node.telemetry_healthy is True
    assert node.last_state_time_mono == 20.0
    assert node.latest_state == (frame, 1, 20.0)
    assert node._session_state == 'HEALTHY'
    assert node.reconnect_backoff == node.next_reconnect_mono == 0.0


def test_bridge_drops_closed_session_and_old_epoch_diagnostics():
    node = Harness(True)
    node.session_ready = False
    node.handle_frame(parsed(boot()))
    assert node._recovery_diagnostics.captured == 0
    node.session_ready = True
    node.handle_frame(parsed(boot()))
    node.session_epoch = 2
    node.telemetry_publish_callback()
    assert node.messages == []


def test_destroyed_node_does_not_emit_diagnostics():
    node = Harness(True)
    node.handle_frame(parsed(boot()))
    node.node_destroyed = True
    node.telemetry_publish_callback()
    assert node.messages == []


@pytest.mark.parametrize('enabled', [False, True])
def test_constructor_does_not_add_threads_or_timers_for_diagnostics(monkeypatch, enabled):
    import rclpy

    constructed = []
    original_collector = bridge.SerialRecoveryDiagnostics

    def factory():
        constructed.append(True)
        return original_collector()

    def forbidden_serial(*args, **kwargs):
        pytest.fail('A compatibility test must never open hardware serial')

    monkeypatch.setattr(bridge, 'SerialRecoveryDiagnostics', factory)
    monkeypatch.setattr(bridge.SerialBridgeNode, 'serial_worker', lambda self: None)
    rclpy.init(args=['--ros-args', '-p',
                     'enable_serial_recovery_diagnostics:=' + str(enabled).lower()])
    node = None
    try:
        node = bridge.SerialBridgeNode(serial_cls=forbidden_serial)
        assert len(constructed) == int(enabled)
        assert len(list(node.timers)) == 2  # Existing telemetry + summary only.
        assert (node._recovery_diagnostics is not None) == enabled
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
