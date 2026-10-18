"""R2 gates for disabled instrumentation and the bounded optional observer."""

import queue
import json
import time
from types import SimpleNamespace

import pytest

from fire_robot_bringup import timing_diagnostics
from fire_robot_bringup.camera_qos_relay import ByteRateLimiter, CameraQosRelay
from fire_robot_bringup.native_mjpeg_camera import MjpegReader
from fire_robot_bringup import runtime_diagnostics
from fire_robot_bringup.runtime_diagnostics import BoundedJsonl, parse_args
from fire_robot_bringup.serial_bridge_node import SerialBridgeNode
from fire_robot_bringup import serial_transport_core


def test_disabled_timing_never_reads_clock_or_allocates_lock(monkeypatch):
    def forbidden_clock():
        raise AssertionError('diagnostics OFF read the clock')

    monkeypatch.setattr(timing_diagnostics, 'time',
                        SimpleNamespace(monotonic=forbidden_clock))
    stats = timing_diagnostics.TimingDiagnostics(enabled=False)
    stats.mark('timer')
    stats.observe('age', 100.0)
    stats.increment('drop')
    assert stats._lock is None
    assert stats.snapshot() == {}
    assert stats.format_snapshot() == ''


def test_bridge_does_not_take_stats_lock_when_off():
    node = SimpleNamespace(enable_diagnostics=False)
    SerialBridgeNode._record_publish_result(
        node, published=1, failed=1, dropped=1, state_age=0.5)
    assert not hasattr(node, 'state_lock')


def test_serial_io_skips_timing_clock_when_off(monkeypatch):
    def forbidden_clock():
        raise AssertionError('serial I/O diagnostics OFF read the clock')

    monkeypatch.setattr(serial_transport_core, 'time',
                        SimpleNamespace(monotonic=forbidden_clock))
    transport = serial_transport_core.SerialTransportCore.__new__(
        serial_transport_core.SerialTransportCore)
    transport.diagnostics_enabled = False
    handle = SimpleNamespace(read=lambda size: b'x' * size,
                             write=lambda payload: len(payload))
    assert transport._timed_read(handle, 2) == b'xx'
    assert transport._timed_write(handle, b'abc') == 3
    assert not hasattr(transport, 'state_lock')


def test_camera_keeps_latest_frame_without_collecting_stats():
    output = queue.Queue(maxsize=1)
    reader = MjpegReader(None, output, None, lambda: None,
                         collect_stats=False)
    reader._offer_latest(b'old')
    reader._offer_latest(b'new')
    assert output.get_nowait()[1] == b'new'
    assert reader.queue_drops == 0


def test_camera_relay_off_keeps_publish_and_skips_counters():
    published = []
    node = SimpleNamespace(
        enable_diagnostics=False,
        _budget=ByteRateLimiter(750000),
        _pub=SimpleNamespace(publish=published.append),
        _dropped_frames=0,
        _published_frames=0,
    )
    message = SimpleNamespace(data=b'jpeg')
    CameraQosRelay._relay(node, message)
    assert published == [message]
    assert node._published_frames == 0
    assert node._dropped_frames == 0


def test_optional_observer_stops_at_file_budget(tmp_path):
    path = tmp_path / 'trace.jsonl'
    writer = BoundedJsonl(path, max_bytes=1024)
    try:
        assert writer.write({'kind': 'event', 'data': 'a' * 100})
        assert not writer.write({'kind': 'event', 'data': 'b' * 1024})
        assert writer.full
    finally:
        writer.close()
    assert path.stat().st_size <= 1024


@pytest.mark.parametrize('duration', ['0', '61'])
def test_observer_rejects_unbounded_duration(tmp_path, duration):
    with pytest.raises(SystemExit):
        parse_args(['--role', 'pi', '--mode', 'trace',
                    '--duration-s', duration,
                    '--output', str(tmp_path / 'events.jsonl')])


def test_observer_finishes_and_closes_resources_without_ros_graph(
        tmp_path, monkeypatch):
    events = []
    active = SimpleNamespace(value=False)

    def init():
        active.value = True
        events.append('init')

    def shutdown():
        active.value = False
        events.append('shutdown')

    def spin_once(_node, timeout_sec):
        time.sleep(min(timeout_sec, 0.003))

    class FakeNode:
        def __init__(self, role, mode, output, include_image=False):
            assert (role, mode, include_image) == ('pi', 'summary', False)
            self.output = output
            events.append('node')

        def flush_summary(self):
            self.output.write({'kind': 'summary'})

        def destroy_node(self):
            events.append('destroy')

    monkeypatch.setattr(runtime_diagnostics, 'rclpy', SimpleNamespace(
        init=init, ok=lambda: active.value,
        spin_once=spin_once, shutdown=shutdown))
    monkeypatch.setattr(runtime_diagnostics, 'RuntimeDiagnostics', FakeNode)
    output = tmp_path / 'summary.jsonl'
    runtime_diagnostics.main([
        '--role', 'pi', '--mode', 'summary', '--duration-s', '0.002',
        '--output', str(output)])
    assert events == ['init', 'node', 'destroy', 'shutdown']
    assert json.loads(output.read_text().splitlines()[0])['kind'] == 'summary'
    receipt = json.loads((tmp_path / 'summary.jsonl.receipt.json').read_text())
    assert receipt['reason'] == 'duration_elapsed'
    assert receipt['include_image'] is False
