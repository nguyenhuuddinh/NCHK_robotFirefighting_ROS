"""Exercise real node methods with fake clocks/publishers and no ROS context."""
import threading
from types import MethodType, SimpleNamespace

import pytest
from nav_msgs.msg import Odometry

from fire_robot_bringup import serial_bridge_node as bridge
from fire_robot_bringup import timing_diagnostics as timing
from fire_robot_bringup import serial_transport_core as transport_core
from fire_robot_bringup.odom_to_tf_broadcaster import OdomToTfBroadcaster
from fire_robot_bringup import odom_to_tf_broadcaster as otf
from fire_robot_safety import odom_freshness


def bind(node, cls, names):
    for name in names:
        setattr(node, name, MethodType(getattr(cls, name), node))


@pytest.fixture
def clock(monkeypatch):
    state = SimpleNamespace(now=10.0)
    fake_time = SimpleNamespace(monotonic=lambda: state.now)
    monkeypatch.setattr(bridge, 'time', fake_time)
    monkeypatch.setattr(transport_core, 'time', fake_time)
    monkeypatch.setattr(timing, 'time', fake_time)
    monkeypatch.setattr(otf, 'time', fake_time)
    monkeypatch.setattr(odom_freshness, 'time', fake_time)
    return state


def bridge_stub(clock, publish_delay=0.0, fail=False):
    node = SimpleNamespace(
        state_lock=threading.Lock(), publish_lock=threading.Lock(),
        odom_timing=timing.TimingDiagnostics(), session_ready=True,
        running=True, telemetry_healthy=True, session_epoch=1,
        node_destroyed=False, records=[], logs=[],
        _transition_state=lambda state: None)
    for name in ('state_gap_max_ms', 'last_state_time_mono',
                 'telemetry_publish_count', 'telemetry_publish_fail_count',
                 'telemetry_drop_count', 'state_publish_max_age',
                 'env_publish_max_age'):
        setattr(node, name, 0)
    for name in ('latest_state', 'latest_env', 'latest_ir',
                 'latest_actuator', 'latest_usb_diagnostic'):
        setattr(node, name, None)
    node.get_logger = lambda: SimpleNamespace(
        info=node.logs.append, warn=node.logs.append)

    def ros_now():
        stamp = Odometry().header.stamp
        stamp.sec = int(clock.now)
        stamp.nanosec = int((clock.now - stamp.sec) * 1e9)
        return SimpleNamespace(to_msg=lambda: stamp)

    node.get_clock = lambda: SimpleNamespace(now=ros_now)

    def publish(msg):
        node.records.append(msg)
        clock.now += publish_delay
        if fail:
            raise RuntimeError('simulated publish failure')

    node.odom_pub = SimpleNamespace(publish=publish)
    node.imu_pub = SimpleNamespace(publish=lambda msg: None)
    bind(node, bridge.SerialBridgeNode, (
        'handle_frame', 'telemetry_publish_callback', '_record_publish_result',
        'log_timer_callback'))
    return node


def state_frame(i):
    return dict(type='STATE', x=float(i), y=0.0, yaw=0.0,
                vx=0.0, wz=0.0, gyro_z=0.0)


def test_timer_pause_exposes_gap_and_slot_overwrites(clock):
    node = bridge_stub(clock)
    for i in range(5):
        clock.now = 10 + i * 0.1
        node.handle_frame(state_frame(i))
        if i in (0, 4):
            node.telemetry_publish_callback()
    node.log_timer_callback()
    log = node.logs[-1]
    assert 'telemetry_timer_gap_max_ms=400.000' in log
    assert 'odom_publish_start_gap_max_ms=400.000' in log
    assert 'odom_publish_success_gap_max_ms=400.000' in log
    assert 'state_slot_overwrites=3' in log
    assert [m.pose.pose.position.x for m in node.records] == [0.0, 4.0]
    assert node.state_publish_max_age == 0.0
    assert node.telemetry_drop_count == 0


@pytest.mark.parametrize('fail', [False, True])
def test_slow_publish_is_measured_even_on_exception(clock, fail):
    node = bridge_stub(clock, publish_delay=0.35, fail=fail)
    node.handle_frame(state_frame(0))
    node.telemetry_publish_callback()
    stats = node.odom_timing.snapshot()
    assert stats['odom_publish_call_max_ms'] == pytest.approx(350)
    assert stats['odom_publish_call_over_300ms'] == 1
    assert stats['odom_publish_start_events'] == 1
    assert stats.get('odom_publish_success_events', 0) == (0 if fail else 1)
    assert node.telemetry_publish_fail_count == (1 if fail else 0)
    assert node.state_publish_max_age == 0.0


def test_window_reset_preserves_gap_and_signed_header_age(clock):
    stats = timing.TimingDiagnostics()
    stats.mark('rx')
    assert stats.snapshot() == {'rx_events': 1}
    assert stats.snapshot() == {}
    clock.now += 0.4
    stats.mark('rx')
    stats.observe('header', -25.0)
    stats.observe('header', 40.0)
    result = stats.snapshot()
    assert result['rx_gap_max_ms'] == pytest.approx(400)
    assert result['rx_gap_over_300ms'] == 1
    assert result['header_min_ms'] == -25.0
    assert result['header_max_ms'] == 40.0


def tf_stub(clock):
    node = SimpleNamespace(
        _timing=timing.TimingDiagnostics(), _odom_received=False,
        _stale_warned=False, _odom_count=0, _tf_count=0,
        _stale_timeout_ns=300_000_000, _stamp_offset_ns=-100_000_000,
        _stamp_source=OdomToTfBroadcaster.STAMP_PI_RECEIVE,
        STAMP_PI_RECEIVE=OdomToTfBroadcaster.STAMP_PI_RECEIVE,
        STAMP_ODOM_HEADER=OdomToTfBroadcaster.STAMP_ODOM_HEADER,
        _odom_stamp_guard=odom_freshness.OdomStampGuard(),
        _odom_sample_mono=None,
        sent=[], logs=[])
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round(clock.now * 1e9)))
    node.get_logger = lambda: SimpleNamespace(
        info=node.logs.append, warn=node.logs.append)
    node._tf_broadcaster = SimpleNamespace(sendTransform=node.sent.append)
    bind(node, OdomToTfBroadcaster, (
        '_odom_callback', '_timer_callback', '_get_tf_stamp'))
    return node


def test_tf_still_stops_on_stale_and_resumes_only_after_new_odom(clock):
    node = tf_stub(clock)
    msg = Odometry()
    msg.header.stamp.sec = 10
    node._odom_callback(msg)
    node._timer_callback()
    assert len(node.sent) == 1
    assert node.sent[0].header.stamp.sec == 10
    assert node.sent[0].header.stamp.nanosec == 100_000_000
    clock.now = 10.4
    node._timer_callback()
    node._timer_callback()
    assert len(node.sent) == 1  # No stale TF, even with diagnostics enabled.
    warnings = [s for s in node.logs if '/odom stale' in s]
    assert len(warnings) == 1
    assert 'tf_timer_gap_max_ms=400.000' in warnings[0]
    msg.header.stamp.nanosec = 350_000_000
    node._odom_callback(msg)
    stats = node._timing.snapshot()
    assert stats['odom_callback_gap_max_ms'] == pytest.approx(400)
    assert stats['odom_header_age_max_ms'] == pytest.approx(50)
    node._timer_callback()
    assert len(node.sent) == 2
    assert not node._stale_warned


def test_callback_reports_negative_header_age_and_rejects_future_stamp(clock):
    node = tf_stub(clock)
    msg = Odometry()
    msg.header.stamp.sec = 11
    node._odom_callback(msg)
    assert node._timing.snapshot()['odom_header_age_min_ms'] == -1000
    node._timer_callback()
    assert len(node.sent) == 0
    clock.now += 0.4
    node._timer_callback()
    assert len(node.sent) == 0


def test_old_sample_cannot_restart_tf_after_outage(clock):
    node = tf_stub(clock)
    msg = Odometry()
    msg.header.stamp.sec = 10
    node._odom_callback(msg)
    node._timer_callback()
    clock.now = 14.3
    node._timer_callback()
    node._odom_callback(msg)
    node._timer_callback()
    assert len(node.sent) == 1
    assert node._odom_stamp_guard.reason == 'STALE_STAMP'
    msg.header.stamp.sec = 14
    msg.header.stamp.nanosec = 250_000_000
    node._odom_callback(msg)
    node._timer_callback()
    assert len(node.sent) == 2


def test_transport_delay_consumes_tf_lifetime(clock):
    node = tf_stub(clock)
    msg = Odometry()
    msg.header.stamp.sec = 9
    msg.header.stamp.nanosec = 750_000_000
    node._odom_callback(msg)  # Already 250ms old on arrival.
    node._timer_callback()
    assert len(node.sent) == 1
    clock.now = 10.051
    node._timer_callback()
    assert len(node.sent) == 1  # Source age301ms despite callback gap51ms.


def test_tf_clock_backwards_stops_then_recovers_in_new_epoch(clock):
    node = tf_stub(clock)
    msg = Odometry()
    msg.header.stamp.sec = 10
    node._odom_callback(msg)
    node._timer_callback()
    clock.now = 5.0
    node._timer_callback()
    assert len(node.sent) == 1 and not node._odom_received
    msg.header.stamp.sec = 5
    node._odom_callback(msg)  # Detect epoch, require another sample.
    node._timer_callback()
    assert len(node.sent) == 1
    clock.now = 5.1
    msg.header.stamp.nanosec = 100_000_000
    node._odom_callback(msg)
    node._timer_callback()
    assert len(node.sent) == 2


def test_frozen_ros_clock_cannot_extend_tf_lifetime(clock):
    node = tf_stub(clock)
    # ROS time remains at10s while the independent monotonic clock advances.
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    msg = Odometry()
    msg.header.stamp.sec = 10
    node._odom_callback(msg)
    node._timer_callback()
    clock.now = 10.301
    node._timer_callback()
    assert len(node.sent) == 1
