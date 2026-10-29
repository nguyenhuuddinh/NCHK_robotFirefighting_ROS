"""Delta recovery: no hardware/context; actual R5 peers and sensor clocks."""
import copy
import json
from pathlib import Path

import pytest
from action_msgs.msg import GoalStatusArray

from test_automatic_065731 import robot


def wait(h,predicate,limit=400):
    for _ in range(limit):
        r=h.step()
        assert not r['fault'],(r['phase'],r['fault'])
        if predicate(r):return r
    raise AssertionError((r['phase'],r['reason'],r['handback']))


def approach(h):
    h.fire=True;h.ir_flame=True
    wait(h,lambda r:abs(h.yaw)>.025)
    h.centered=True;wait(h,lambda r:h.x>.025)


@pytest.mark.parametrize('goal',[False,True])
@pytest.mark.parametrize('when',['midpulse','after_home_IR_only'])
def test_interrupted_pulse_retires_with_OFF_new_home_clear_and_no_replay(monkeypatch,tmp_path,goal,when):
    h,_=robot(monkeypatch,tmp_path,goal=goal)
    approach(h);h.right=950
    wait(h,lambda r:h.device_on if when=='midpulse' else h.home==1)
    h.fire=False;h.ir_flame=when!='midpulse'
    wait(h,lambda r:(h.op.core.get('actor',h.bus.now) or {}).get('state')=='ABORTED')
    if h.ir_flame:
        for _ in range(30):
            r=h.step();assert not r['handback']['consumed'] and not h.device_on
    h.ir_flame=False
    r=wait(h,lambda r:r['handback']['consumed'])
    assert r['reason']=='NAV2_READY_NEW_GOAL' and h.home==1 and len(h.pulses)==1
    assert r['handback']['release']['pulse_outcome']=='INTERRUPTED'
    assert h.op.core.interruption_reason.startswith('PULSE_INTERRUPTED:')
    h.status=[('56'*16,2)]
    wait(h,lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)


@pytest.mark.parametrize('bad',['ON','old_ACT','missing_home'])
def test_aborted_pulse_cannot_release_from_invalid_device_proof(monkeypatch,tmp_path,bad):
    h,_=robot(monkeypatch,tmp_path)
    approach(h);h.right=950;wait(h,lambda r:h.device_on)
    h.fire=False;h.ir_flame=False
    # Install contradiction before physical home can be reported to peers.
    send=h.emit;invalid=[True]
    def changed(topic,value):
        if invalid[0] and topic=='/fire_sensors/actuator_state':
            value=copy.deepcopy(value)
            if bad=='ON':value['pump_pwm_commanded_on']=True
            if bad=='old_ACT':value['receive_age_ms']=1000.
            if bad=='missing_home':value['home_command_completed_count']=0
        send(topic,value)
    h.emit=changed
    for _ in range(110):
        r=h.step()
        assert not r['handback']['consumed']
    assert len(h.pulses)==1 and not h.device_on
    invalid[0]=False
    wait(h,lambda r:r['handback']['consumed'])


def reentry(h):
    approach(h)
    for _ in range(3):
        h.fire=False
        for _ in range(3):h.step()
        h.fire=True
        for _ in range(5):h.step()
    h.right=950
    wait(h,lambda r:h.op.core.pending_stationary and h.op.core.last_ir_step>0)
    node=h.by_name['fire_pulse_controller']
    wait(h,lambda r:node.session.stationary.state=='CONFIRMING')
    return node


@pytest.mark.parametrize('bad',['step','progress','budget','context','raw_owner'])
def test_reentry_stationary_request_cannot_replace_original_I_binding(monkeypatch,tmp_path,bad):
    from fire_robot_mission.stationary_confirmation import common_errors
    h,_=robot(monkeypatch,tmp_path);node=reentry(h)
    session=node.session;now=h.bus.now
    counts=node.transport.fresh_counts(now,node.counts())
    original=session.inputs['stationary_request']
    request=copy.deepcopy(original[0]);re=request['ir_reentry']
    if bad=='step':re['step_count']+=1
    if bad=='progress':re['total_progress_m']+=.01
    if bad=='budget':re['budget_started_at']=now-20.01
    if bad=='context':request['cycle_context']['cycle_id']='98'*16
    if bad=='raw_owner':counts['raw']=1
    session.inputs['stationary_request']=(request,original[1])
    assert common_errors(session,now,counts,require_idle=True)
    assert not h.pulses
    session.inputs['stationary_request']=original
    wait(h,lambda r:h.home==1)
    assert len(h.pulses)==1 and h.x<=.30+1e-6


@pytest.mark.parametrize('goal,band',[(True,950),(True,750),(False,750)])
def test_range_at_reentry_keeps_real_or_missing_goal_and_close_policy(monkeypatch,tmp_path,goal,band):
    h,_=robot(monkeypatch,tmp_path,goal=goal)
    approach(h);cycle=h.op.core.cycles.current;requested=h.op.core.cycles.budget.requested_at
    for _ in range(3):
        h.fire=False
        for _ in range(3):h.step()
        h.fire=True
        for _ in range(5):h.step()
    h.right=band
    try:wait(h,lambda r:h.home==1)
    except AssertionError:
        data=[];old=None
        for r,s in zip(h.reports,h.snapshots):
            actor=s['fire_pulse_controller'];confirm=actor.get('stationary_confirmation') or {}
            tag=(r['phase'],confirm.get('state'),confirm.get('reason'))
            if tag!=old:
                data.append(dict(tag=tag,ir_live=s['ir_approach_live'],actor=actor));old=tag
        dest=Path(__file__).resolve().parents[6]/'evidence_r5_5_f01_f02_f03_fix_20261027'
        (dest/f'real_goal_{goal}_{band}.json').write_text(json.dumps(data,indent=2))
        raise
    assert h.op.core.cycles.current==cycle
    assert h.op.core.cycles.budget.requested_at==requested
    assert len(h.pulses)==1 and h.x<=.30+1e-6 and h.bus.max_raw==1


@pytest.mark.parametrize('goal,bad',[(True,'goal_uuid'),(True,'r5_cycle_context'),(False,'cycle_id')])
def test_reentry_never_admits_another_goal_or_cycle(monkeypatch,tmp_path,goal,bad):
    from fire_robot_mission.stationary_confirmation import common_errors
    h,_=robot(monkeypatch,tmp_path,goal=goal);node=reentry(h)
    session=node.session;now=h.bus.now
    counts=node.transport.fresh_counts(now,node.counts())
    original=session.inputs['ir_live'];live=copy.deepcopy(original[0])
    if bad=='r5_cycle_context':live[bad]['cycle_id']='98'*16
    else:live[bad]='98'*16
    session.inputs['ir_live']=(live,original[1])
    assert 'STATIONARY_IR_OWNER_NOT_IDLE' in common_errors(session,now,counts,require_idle=True)
    assert not h.pulses
    session.inputs['ir_live']=original
    wait(h,lambda r:h.home==1)
    assert len(h.pulses)==1


def test_initial_status_loss_holds_until_real_server_snapshot_then_fire_runs(monkeypatch,tmp_path):
    from harness import Bus
    original=Bus.publisher;sends=[];drop=[True]
    def missing(bus,name,typ,topic,qos):
        p=original(bus,name,typ,topic,qos)
        if topic=='/navigate_to_pose/_action/status':
            send=p.publish;sends.append(send)
            p.publish=lambda msg:None if drop[0] else send(msg)
        return p
    monkeypatch.setattr(Bus,'publisher',missing)
    h,_=robot(monkeypatch,tmp_path)
    h.fire=True;h.ir_flame=True
    for _ in range(12):r=h.step()
    assert r['reason']=='WAIT_FRESH_NAV_STATUS' and not h.pulses
    # The separate C++ test obtains this empty array from the real action
    # server's own goal table. Here model its next 200 ms refresh delivery.
    drop[0]=False;sends[0](GoalStatusArray());h.bus.flush()
    wait(h,lambda r:abs(h.yaw)>.025)
    h.centered=True;h.right=950
    wait(h,lambda r:h.home==1)
    assert len(h.pulses)==1 and not h.op.core.closed
