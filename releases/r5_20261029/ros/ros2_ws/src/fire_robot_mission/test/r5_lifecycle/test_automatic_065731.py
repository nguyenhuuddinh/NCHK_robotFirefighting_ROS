"""Automatic requests through actual R5 adapters. No ROS init or hardware."""
import copy
import json
from pathlib import Path
import pytest
from test_runtime_parity_042557 import scheduled_robot
from fire_robot_mission.automatic_policy import AutomaticStartGate

SESSION='12345678-1234-4567-8234-123456789abc'
PACKET='ab'*32


def marker():
    return dict(schema_version=1,kind='R5_AUTOMATIC_START',session=SESSION,
        packet=PACKET,stage='READY_GATE2',ready=True)


def robot(monkeypatch,tmp_path,*,goal=False):
    path=tmp_path/'automatic_start.json'
    h=scheduled_robot(monkeypatch,goal=goal,phase=1,operator_parameters=dict(
        automatic_mode=True,automatic_start_file=str(path),
        automatic_runner_session=SESSION,automatic_packet=PACKET))
    for _ in range(90):
        r=h.step()
        assert not r['armed'] and not any(r['intents'].values())
    path.write_text(json.dumps(marker()));path.chmod(0o600)
    h.until(lambda r:r['armed'])
    h.until(lambda r:h.op.core.fire_permission)
    return h,path


@pytest.mark.parametrize('scenario',['goal','no_goal','stationary_range','stationary_close','recenter','lost_target','budget_expired'])
def test_automatic_fire_chain_without_A_F_T_I_P(monkeypatch,tmp_path,scenario):
    h,path=robot(monkeypatch,tmp_path)
    if scenario=='goal':
        h.status=[('a'*32,2)]
        h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    h.right=750 if scenario=='stationary_close' else 950 if scenario=='stationary_range' else 5000
    h.fire=True
    h.until(lambda r:abs(h.yaw)>.025)
    h.centered=True
    if scenario not in ('stationary_range','stationary_close'):
        h.until(lambda r:h.op.core.cycles.budget.requested_at is not None)
        requested=h.op.core.cycles.budget.requested_at
        h.until(lambda r:h.x>.025)
        if scenario=='lost_target':
            h.ir_flame=True;h.fire=False
            h.until(lambda r:r['phase']=='WAIT_TARGET')
            x=h.x;yaw=h.yaw
            for _ in range(12):h.step()
            assert h.x==x and h.on_at is None
            h.fire=True;h.centered=False
            h.until(lambda r:h.yaw>yaw+.025)
            h.centered=True
            h.until(lambda r:h.x>x+.01)
        h.right=950
        if scenario in ('recenter','budget_expired'):
            h.until(lambda r:h.by_name['ir_approach_live'].core.phase=='COMPLETE')
            x=h.x;yaw=h.yaw;h.centered=False
            if scenario=='budget_expired':
                # Keep YOLO fire visible while testing the longer I budget.
                # Losing YOLO for >3 s is a separate recovery timeout.
                h.ir_flame=True
                h.until(lambda r:r['budget']['exhausted'],limit=340)
            h.until(lambda r:h.yaw>yaw+.025)
            assert h.x==x
            h.centered=True
    h.until(lambda r:h.home==1,limit=300)
    assert len(h.pulses)==1 and h.off_at-h.on_at<=5.1+1e-6
    # Remaining fire cannot authorize a second automatic P.
    for _ in range(20):h.step()
    assert len(h.pulses)==1
    cycle=h.op.core.cycles.current
    if scenario not in ('stationary_range','stationary_close'):
        assert h.op.core.cycles.budget.requested_at==requested
    h.fire=False;h.ir_flame=False
    r=h.until(lambda r:r['handback']['consumed'])
    assert r['reason']=='NAV2_READY_NEW_GOAL' and not r['closed']
    h.status=[('56'*16,2)]
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    events=[a for _,who,a in h.bus.logs if who=='fire_robot_operator' and a and 'AUTOMATIC_INTENT ' in a[0]]
    assert events and h.bus.max_raw==1
    assert all(json.loads(a[0].split('AUTOMATIC_INTENT ',1)[1])['feedback']['origin']=='AUTOMATIC' for a in events)


@pytest.mark.parametrize('failure',['remove_start','malformed_start','identity_epoch','Q','X'])
def test_automatic_revokes_or_stops_no_queued_motion(monkeypatch,tmp_path,failure):
    h,path=robot(monkeypatch,tmp_path)
    h.fire=True;h.until(lambda r:abs(h.yaw)>.025)
    if failure=='remove_start':path.unlink()
    if failure=='malformed_start':path.write_text('{}')
    if failure=='identity_epoch':
        from std_msgs.msg import String
        h.silence_serial=True
        identity=copy.deepcopy(h.op.core.inputs['identity'][0])
        identity.update(connection_epoch=2,valid=False,session_ready=False,telemetry_healthy=False)
        h.external['/fire_sensors/serial_session_identity'].publish(String(data=json.dumps(identity)))
        h.bus.flush()
    r=h.step(failure if failure in ('Q','X') else None)
    for _ in range(30):r=h.step()
    assert h.on_at is None
    if failure!='identity_epoch':assert not any(p.topic=='/cmd_vel_raw' for p in h.bus.pubs)
    else:
        assert not any(r['intents'].values())
        assert h.bus.last['/cmd_vel_raw'].linear.x==h.bus.last['/cmd_vel_raw'].angular.z==0.
    if failure=='X':
        assert h.op.core.cancel_requested and not h.op.core.closed
    else:assert r['fault'] or r['closed']


@pytest.mark.parametrize('bad',['missing','wrong_session','wrong_packet','before_ready','extra','symlink','writable','malformed','too_big','boolean_schema','integer_ready'])
def test_automatic_start_bound_to_private_run_packet_and_READY(tmp_path,bad):
    path=tmp_path/'start';data=marker()
    if bad=='wrong_session':data['session']='22345678-1234-4567-8234-123456789abc'
    if bad=='wrong_packet':data['packet']='cd'*32
    if bad=='before_ready':data['stage']='CONSUMERS'
    if bad=='extra':data['goal']='arbitrary'
    if bad=='boolean_schema':data['schema_version']=True
    if bad=='integer_ready':data['ready']=1
    if bad!='missing':path.write_text('{' if bad=='malformed' else 'x'*8193 if bad=='too_big' else json.dumps(data))
    if bad=='symlink':
        real=tmp_path/'real';path.rename(real);path.symlink_to(real)
    if bad=='writable':path.chmod(0o666)
    g=AutomaticStartGate(path,SESSION,PACKET)
    assert not g.ready() and not g.seen


@pytest.mark.parametrize('rejects',[1,3])
def test_automatic_actor_negative_admission_receipt_bounded_retry_one_pulse(monkeypatch,tmp_path,rejects):
    h,path=robot(monkeypatch,tmp_path)
    actor=h.by_name['fire_pulse_controller'].session
    original=actor.step;seen=[]
    def race(now,counts,*,request=False,stop=False):
        if request:
            seen.append(now)
            if len(seen)<=rejects:counts=dict(counts,raw=1)
        return original(now,counts,request=request,stop=stop)
    monkeypatch.setattr(actor,'step',race)
    h.fire=True;h.right=950
    h.until(lambda r:abs(h.yaw)>.025);h.centered=True
    if rejects==1:
        h.until(lambda r:h.home==1,limit=400)
        assert len(seen)==2 and len(h.pulses)==1
    else:
        h.until(lambda r:len(seen)==3)
        for _ in range(50):r=h.step()
        assert len(seen)==3 and h.on_at is None and not r['closed']
        assert r['automatic']['blocker']=='AUTOMATIC_P_WAIT_RECEIPT_NO_REPLAY'


def test_automatic_missing_P_delivery_does_not_retry_uncertain_request(monkeypatch,tmp_path):
    h,path=robot(monkeypatch,tmp_path)
    h.bus.drop=lambda p,s,msg:p.topic=='/fire_mission/pump_demo_arm' and getattr(msg,'data',None) is True
    h.fire=True;h.right=950
    h.until(lambda r:abs(h.yaw)>.025);h.centered=True
    h.until(lambda r:h.op.core.p_used)
    for _ in range(80):r=h.step()
    assert h.on_at is None and h.op.automatic.p_attempts==1
    assert not r['closed'] and 'WAIT_ACTOR_RECEIPT' in r['reason']
