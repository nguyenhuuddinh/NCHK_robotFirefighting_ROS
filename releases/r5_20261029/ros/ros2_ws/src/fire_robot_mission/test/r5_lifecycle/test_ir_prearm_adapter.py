"""Actual IR preview and live operator with six consumers, asynchronous ADC cadence."""
import json
import pytest
from harness import Robot
from fire_robot_mission.ir_range_preview import IrRangePreviewNode
from fire_robot_mission.operational_intents import ARM_TOPICS
from fire_robot_mission.operational_view import status_lines
from test_operator_live_delta import operator_fixture


@pytest.mark.parametrize('period_ticks', [4,8])
def test_real_preview_node_and_operator_are_ready_with_200_or_400ms_raw(monkeypatch,period_ticks):
    r=Robot(monkeypatch);b=r.bus
    # Replace the fixture preview publisher with the real node. It still uses
    # the 5 Hz periodic report clock, independent of raw ADC arrival.
    b.pubs.remove(r.external['/fire_mission/ir_range_preview'])
    preview=IrRangePreviewNode();r.nodes.append(preview)
    emit=r.emit;publish=preview.test_timers[0]
    def emit_cadence(topic,value):
        tick=round((b.now-1.)/.05)
        if topic=='/fire_mission/ir_range_preview':return
        if topic=='/fire_sensors/ir_raw' and tick%period_ticks:return
        emit(topic,value)
    r.emit=emit_cadence
    preview.test_timers=[lambda:publish() if round((b.now-1.)/.05)%4==3 else None]
    report=r.until(lambda v:v['startup_readiness']['ready'],limit=110)
    assert b.now<6. and not report['armed']
    assert 'ARM SAN SANG' in '\n'.join(status_lines(report))
    report=r.step('A')
    if not report['armed']:report=r.until(lambda v:v['armed'],limit=10)
    assert report['operator_feedback']['reason']=='ARM_ACCEPTED'
    assert report['intents']['navigation'] and not report['intents']['pulse']
    assert not b.max_raw and not r.pulses


def test_real_operator_pending_A_is_cancelled_on_adapter_loss(monkeypatch):
    a=operator_fixture(monkeypatch)
    for _ in range(35):a.tick()
    # Retain the old real raw receipt while other inputs continue arriving.
    original=a.node.on_json
    a.node.on_json=lambda key,msg:None if key=='ir_raw' else original(key,msg)
    for _ in range(3):a.tick()
    r=a.tick('A');assert r['startup_readiness']['arm_request_pending'] and not r['armed']
    assert 'DA NHAN A' in '\n'.join(status_lines(r))
    r=a.tick(safety=False)
    assert not r['armed'] and not r['startup_readiness']['arm_request_pending']
    a.node.on_json=original
    for _ in range(40):r=a.tick()
    assert r['startup_readiness']['ready'] and not r['armed']
    assert not any(a.sent[t][-1] for t in ARM_TOPICS)


def test_raw_endpoint_change_is_a_fault_and_never_grants_A(monkeypatch):
    a=operator_fixture(monkeypatch)
    for _ in range(35):a.tick()
    a.pubs['/fire_sensors/ir_raw'][0].endpoint_gid=[99]*24
    r=a.tick('A')
    assert not r['armed'] and 'INPUT_GID_CHANGED:/fire_sensors/ir_raw' in r['fault']
    assert not any(a.sent[t][-1] for t in ARM_TOPICS)
