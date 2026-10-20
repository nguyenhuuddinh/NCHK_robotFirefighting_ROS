"""Exercise every control/proof odom callback without nodes or hardware."""
import importlib
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry

from fire_robot_safety import odom_freshness


NODES = [
    ('live_nav_stop_gateway', 'LiveNavStopGateway', '_on_odom'),
    ('live_alignment_takeover', 'LiveAlignmentTakeover', '_on_odom'),
    ('nav2_cancel_guard', 'Nav2CancelGuard', '_on_odom'),
    ('pump_demo_controller', 'PumpDemoController', '_on_odom'),
    ('ir_approach_live', 'IrApproachLiveNode', '_on_odom'),
    ('ir_approach_shadow', 'IrApproachShadowNode', '_odom'),
    ('mission_supervisor', 'FireMissionSupervisor', '_on_odom'),
]


@pytest.mark.parametrize('module,class_name,callback', NODES)
def test_actual_callback_rejects_bad_stamps_and_keeps_age_budget(
        monkeypatch, module, class_name, callback):
    cls = getattr(importlib.import_module('fire_robot_mission.' + module), class_name)
    calls = []
    core = SimpleNamespace(
        odom_timeout=.35, input_timeout=.35, input_timeout_s=.35,
        telemetry_timeout=.35, timeout_s=.35,
        update_odom=lambda *args: calls.append(args),
        update=lambda *args: calls.append(args))
    clock = SimpleNamespace(ros=10_000_000_000, mono=50.)
    monkeypatch.setattr(odom_freshness, 'time', SimpleNamespace(
        monotonic=lambda: clock.mono))
    logs = []
    node = SimpleNamespace(
        core=core, _core=core, _odom_stamp_guard=odom_freshness.OdomStampGuard(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock.ros)),
        get_logger=lambda: SimpleNamespace(warn=logs.append, info=logs.append))
    for name in ('_values', '_twist_values'):
        if hasattr(cls, name):
            setattr(node, name, getattr(cls, name))
    msg = Odometry()
    for sec, ns in ((0, 0), (9, 0), (11, 0)):
        msg.header.stamp.sec, msg.header.stamp.nanosec = sec, ns
        getattr(cls, callback)(node, msg)
    assert calls == []
    msg.header.stamp.sec, msg.header.stamp.nanosec = 9, 800_000_000
    getattr(cls, callback)(node, msg)
    assert calls[-1][-1] == pytest.approx(49.8)
    # Same or older payload must not reset admission/hold timing.
    getattr(cls, callback)(node, msg)
    msg.header.stamp.nanosec = 750_000_000
    getattr(cls, callback)(node, msg)
    assert len(calls) == 1
    clock.ros, clock.mono = 14_300_000_000, 54.3
    getattr(cls, callback)(node, msg)
    assert len(calls) == 1  # Outage then old sample cannot restore freshness.
    msg.header.stamp.sec, msg.header.stamp.nanosec = 14, 200_000_000
    getattr(cls, callback)(node, msg)
    assert len(calls) == 2 and calls[-1][-1] == pytest.approx(54.2)


def test_gateway_stationary_proof_expires_by_source_age():
    from fire_robot_mission.live_nav_stop_gateway import LiveNavStopCore
    c = LiveNavStopCore()
    g = odom_freshness.OdomStampGuard()
    msg = Odometry()
    msg.header.stamp.sec, msg.header.stamp.nanosec = 9, 700_000_000
    received = g.admit(msg.header.stamp, 10_000_000_000, 50., .35)
    c.update_odom((0.,) * 6, received)
    assert c._stationary(50.)[0:2] == (True, True)
    assert c._stationary(50.051)[0:2] == (False, False)
