"""Real R5 adapters, navigation-only holds and operator X; no ROS/devices."""
import copy
import json
import pytest
from test_ir_freshness_hold_115816 import robot


def driving(monkeypatch,tmp_path):
    h=robot(monkeypatch,tmp_path,goal=True)
    for p in h.nav:
        original=p.publish
        def publish(msg,send=original):
            msg=copy.deepcopy(msg)
            if any(s in (1,2,3) for u,s in h.status):msg.linear.x=.06
            send(msg)
        monkeypatch.setattr(p,'publish',publish)
    h.until(lambda r:h.x>.03)
    return h


@pytest.mark.parametrize('only_operator',[False,True])
def test_navigation_without_fire_resumes_same_goal_after_identity_hold(monkeypatch,tmp_path,only_operator):
    h=driving(monkeypatch,tmp_path);held=[True]
    if only_operator:
        h.bus.drop=lambda p,s,m:held[0] and p.topic=='/fire_sensors/serial_session_identity' and s.info.node_name=='fire_robot_operator'
    else:h.identity_gap=True
    h.until(lambda r:r['identity_hold'])
    for _ in range(12):h.step()
    x=h.x;held[0]=False;h.identity_gap=False
    h.until(lambda r:h.x>x+.01)
    assert h.op.core.cycles.current is None and h.op.core.armed and h.bus.max_raw==1
    h.status=[('a'*32,4)]
    h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL')
    x=h.x;h.status.append(('56'*16,2))
    h.until(lambda r:h.x>x+.01)


def test_X_cancels_exact_navigation_goal_twice_and_newgoal_runs(monkeypatch,tmp_path):
    h=driving(monkeypatch,tmp_path)
    for i in range(2):
        goal=h.status[-1][0]
        r=h.step('X');assert r['operator_feedback']['accepted']
        h.step('X')
        h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL',limit=180)
        assert sum(u==goal for at,u in h.bus.cancelled)==1
        assert not h.pulses and h.op.core.cycles.current is None
        x=h.x;h.status.append((('56' if i==0 else '78')*16,2))
        h.until(lambda r:h.x>x+.01)
    h.step('Q');r=h.until(lambda r:r.get('shutdown',{}).get('done'))
    assert r['shutdown']['confirmed']


def test_navigation_epoch_change_never_resumes_or_restores_NAVIGATION_label(monkeypatch,tmp_path):
    from std_msgs.msg import String
    h=driving(monkeypatch,tmp_path);h.silence_serial=True
    identity=copy.deepcopy(h.op.core.inputs['identity'][0]);identity.update(connection_epoch=2,valid=False,session_ready=False,telemetry_healthy=False)
    h.external['/fire_sensors/serial_session_identity'].publish(String(data=json.dumps(identity)));h.bus.flush()
    for _ in range(30):
        r=h.step();assert r['fault']=='BOOT_OR_SERIAL_EPOCH_CHANGED' and r['phase']=='FAULT'
        assert not any(r['intents'].values())


@pytest.mark.parametrize('bad',['epoch','scope','closed','stale','source','moving','ACT_ON'])
def test_navigation_X_boundary_never_sends_on_invalid_session_or_STOP(monkeypatch,tmp_path,bad):
    h=driving(monkeypatch,tmp_path);node=h.by_name['nav2_cancel_guard'];ep=node._cycle;p=ep.participant
    s=copy.deepcopy(h.op.core.report(h.bus.now));s['navigation_cancel_uuid']='a'*32
    p.update('operator_session',s,h.bus.now)
    if bad=='epoch':s['armed_identity'][1]+=1;p.update('operator_session',s,h.bus.now)
    if bad=='scope':s['scope']='WRONG';p.update('operator_session',s,h.bus.now)
    if bad=='closed':s['closed']=True;p.update('operator_session',s,h.bus.now)
    if bad=='stale':p.inputs['operator_session']=(s,h.bus.now-1.)
    if bad=='source':monkeypatch.setattr(ep,'_source',lambda t:False)
    if bad=='moving':p.inputs['odom']=((.1,0,0,0,0,0),h.bus.now)
    if bad=='ACT_ON':p.inputs['actuator'][0]['pump_pwm_commanded_on']=True
    ep.before_step(h.bus.now);ep.pause_tick(h.bus.now)
    assert not h.bus.cancelled
