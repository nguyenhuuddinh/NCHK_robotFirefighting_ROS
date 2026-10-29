"""Only new cancel-review deltas: retained status, request boundaries and races."""
import copy
import json
import pytest
from std_msgs.msg import String, Bool
from action_msgs.msg import GoalStatusArray
from harness import BE, REL
from test_navigation_no_fire_125512 import driving


def waiting(monkeypatch,tmp_path):
    h=driving(monkeypatch,tmp_path)
    ep=h.by_name['nav2_cancel_guard']._cycle
    monkeypatch.setattr(ep.node.cancel_client,'service_is_ready',lambda:False)
    r=h.step('X');assert r['operator_feedback']['accepted']
    h.until(lambda r:not any(p.topic=='/cmd_vel_raw' for p in h.bus.pubs))
    for _ in range(4):h.step()
    assert not h.bus.cancelled
    return h,ep


def run_cancel(ep,now):
    ep.before_step(now);ep.pause_tick(now)


@pytest.mark.parametrize('bad',['future_status','invalid_uuid','wrong_uuid','duplicate','multiple_active',
    'missing_status','future_session','held_identity','stale_ACT','schema_ACT','mode_ACT','pwm_ACT',
    'moving_cmd','stale_cmd','stale_odom','pump_owner','raw_owner','operator_gid','status_gid'])
def test_new_cancel_review_boundaries_never_send(monkeypatch,tmp_path,bad):
    h,ep=waiting(monkeypatch,tmp_path);p=ep.participant;now=h.bus.now
    monkeypatch.setattr(ep.node.cancel_client,'service_is_ready',lambda:True)
    if bad=='future_status':p.inputs['status']=(p.inputs['status'][0],now+.01)
    if bad=='invalid_uuid':p.inputs['status']=([('z'*32,2)],now)
    if bad=='wrong_uuid':p.inputs['status']=([('56'*16,2)],now)
    if bad=='duplicate':p.inputs['status']=([('a'*32,2),('a'*32,2)],now)
    if bad=='multiple_active':p.inputs['status']=([('a'*32,2),('56'*16,2)],now)
    if bad=='missing_status':p.inputs.pop('status')
    if bad=='future_session':p.inputs['operator_session']=(p.inputs['operator_session'][0],now+.01)
    if bad=='held_identity':ep.continuity.held=True;ep.continuity.since=now
    if bad=='stale_ACT':p.inputs['actuator']=(p.inputs['actuator'][0],now-1.)
    if bad in ('schema_ACT','mode_ACT','pwm_ACT'):
        a=copy.deepcopy(p.inputs['actuator'][0]);key,value={'schema_ACT':('schema_version',2),'mode_ACT':('mode','WRONG'),'pwm_ACT':('pump_pwm_ready',False)}[bad]
        a[key]=value;p.inputs['actuator']=(a,now)
    if bad=='moving_cmd':p.inputs['cmd']=((.1,0,0,0,0,0),now)
    if bad=='stale_cmd':p.inputs['cmd']=(p.inputs['cmd'][0],now-1.)
    if bad=='stale_odom':p.inputs['odom']=(p.inputs['odom'][0],now-1.)
    if bad=='pump_owner':h.bus.publisher('unexpected_pump',Bool,'/pump_cmd',REL)
    if bad=='raw_owner':
        from geometry_msgs.msg import Twist
        h.bus.publisher('unexpected_driver',Twist,'/cmd_vel_raw',BE)
    if bad in ('operator_gid','status_gid'):
        topic='/fire_mission/operator_state' if bad=='operator_gid' else '/navigate_to_pose/_action/status'
        pub=next(p for p in h.bus.pubs if p.topic==topic)
        pub.info.endpoint_gid=list((999).to_bytes(24,'big'))
    run_cancel(ep,now)
    assert not h.bus.cancelled


def test_retained_status_from_pinned_source_can_cancel_but_replay_cannot_repeat(monkeypatch,tmp_path):
    h,ep=waiting(monkeypatch,tmp_path);p=ep.participant;now=h.bus.now
    monkeypatch.setattr(ep.node.cancel_client,'service_is_ready',lambda:True)
    p.inputs['status']=([('a'*32,2)],now-12.)
    run_cancel(ep,now)
    assert [u for at,u in h.bus.cancelled]==['a'*32]
    # Replaying a valid request/status cannot re-send even after terminal was seen.
    p.inputs['status']=([('a'*32,5)],now);run_cancel(ep,now)
    p.inputs['status']=([('a'*32,2)],now);run_cancel(ep,now)
    assert len(h.bus.cancelled)==1
    assert ep.nav_cancel_report['status_policy']=='PINNED_EVENT_STREAM'


@pytest.mark.parametrize('only_operator',[False,True])
def test_X_during_identity_hold_drains_then_cancels_and_next_goal_runs(monkeypatch,tmp_path,only_operator):
    h=driving(monkeypatch,tmp_path);held=[True]
    if only_operator:
        h.bus.drop=lambda p,s,m:held[0] and p.topic=='/fire_sensors/serial_session_identity' and s.info.node_name=='fire_robot_operator'
    else:h.identity_gap=True
    h.until(lambda r:r['identity_hold']);h.step('X')
    h.until(lambda r:not any(p.topic=='/cmd_vel_raw' for p in h.bus.pubs))
    assert not h.bus.cancelled
    held[0]=False;h.identity_gap=False
    h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL')
    assert [u for at,u in h.bus.cancelled]==['a'*32]
    x=h.x;h.status.append(('56'*16,2));h.until(lambda r:h.x>x+.01)


@pytest.mark.parametrize('terminal',[4,6])
def test_natural_terminal_wins_X_race_without_action_retry(monkeypatch,tmp_path,terminal):
    h,ep=waiting(monkeypatch,tmp_path);h.status=[('a'*32,terminal)]
    h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL')
    assert not h.bus.cancelled
    h.status.append(('56'*16,2));x=h.x;h.until(lambda r:h.x>x+.01)


def test_Q_while_cancel_service_unavailable_stops_without_actions(monkeypatch,tmp_path):
    h,ep=waiting(monkeypatch,tmp_path);h.step('Q')
    r=h.until(lambda r:r.get('shutdown',{}).get('done'))
    assert r['shutdown']['confirmed'] and not h.bus.cancelled and not h.pulses


@pytest.mark.parametrize('failure',['send','response','reject'])
def test_cancel_failure_visible_without_retry_and_Q_still_works(monkeypatch,tmp_path,failure):
    from types import SimpleNamespace as N
    h,ep=waiting(monkeypatch,tmp_path);calls=[]
    def call(request):
        calls.append(bytes(request.goal_info.goal_id.uuid).hex())
        if failure=='send':raise RuntimeError('synthetic transport failure')
        class F:
            def done(self):return True
            def result(self):
                if failure=='response':raise RuntimeError('synthetic response failure')
                return N(return_code=1,goals_canceling=[])
        return F()
    monkeypatch.setattr(ep.node.cancel_client,'service_is_ready',lambda:True)
    monkeypatch.setattr(ep.node.cancel_client,'call_async',call)
    for _ in range(90):h.step()
    assert calls==['a'*32] and h.op.core.navigation_cancel_uuid=='a'*32
    assert ep.nav_cancel_report['state'] in ('CANCEL_SEND_FAILED','CANCEL_REJECTED_WAIT_TERMINAL')
    h.step('Q');assert h.until(lambda r:r.get('shutdown',{}).get('done'))['shutdown']['confirmed']


def test_goal_arriving_before_cancel_release_is_drained_not_permanently_locked(monkeypatch,tmp_path):
    h,ep=waiting(monkeypatch,tmp_path)
    h.status=[('a'*32,5),('56'*16,2)]
    monkeypatch.setattr(ep.node.cancel_client,'service_is_ready',lambda:True)
    h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL')
    assert [u for at,u in h.bus.cancelled]==['56'*16]
    h.status.append(('78'*16,2));x=h.x;h.until(lambda r:h.x>x+.01)


def test_navigation_cancel_does_not_poison_next_automatic_fire_cycle(monkeypatch,tmp_path):
    h,ep=waiting(monkeypatch,tmp_path)
    monkeypatch.setattr(ep.node.cancel_client,'service_is_ready',lambda:True)
    h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL')
    h.fire=True;h.centered=False
    h.until(lambda r:abs(h.yaw)>.025)
    assert h.op.core.cycles.current is not None and h.op.core.cycles.current.nav_goal_uuid is None
    assert [u for at,u in h.bus.cancelled]==['a'*32] and h.bus.max_raw==1


def test_status_and_operator_callbacks_reject_changed_source_before_storage(monkeypatch,tmp_path):
    h,ep=waiting(monkeypatch,tmp_path);p=ep.participant
    before=copy.deepcopy(p.inputs)
    op=next(p for p in h.bus.pubs if p.topic=='/fire_mission/operator_state')
    op.info.endpoint_gid=list((999).to_bytes(24,'big'))
    ep.on_json('operator_session',String(data=json.dumps(dict(navigation_cancel_uuid='56'*16))))
    assert p.inputs['operator_session']==before['operator_session']
    status=h.external['/navigate_to_pose/_action/status'];status.info.endpoint_gid=list((998).to_bytes(24,'big'))
    ep.on_status(GoalStatusArray())
    assert p.inputs['status']==before['status']


@pytest.mark.parametrize('bad',['epoch','request_stale','request_phase','raw_owner','terminal_missing','active_other'])
def test_no_goal_rebind_keeps_history_binding_without_all_current_proofs(monkeypatch,tmp_path,bad):
    from fire_robot_mission.cycle_release import ReleaseContext
    from geometry_msgs.msg import Twist
    h,guard=waiting(monkeypatch,tmp_path)
    monkeypatch.setattr(guard.node.cancel_client,'service_is_ready',lambda:True)
    h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL')
    ep=h.by_name['live_nav_stop_gateway']._cycle;p=ep.participant;core=ep.node.core;now=h.bus.now
    assert core.goal_uuid=='a'*32 and core.normal_terminal==5 and core.released
    ctx=ReleaseContext('cd'*16,None,*ep.continuity.bound)
    p.context=ctx;p.phase='ACTIVE';p.request={'phase':'ACTIVE','context':ctx.as_dict()};p.request_at=now
    if bad=='epoch':p.inputs['identity'][0]['connection_epoch']+=1
    if bad=='request_stale':p.request_at=now-1.
    if bad=='request_phase':p.request['phase']='PREPARE'
    if bad=='raw_owner':h.bus.publisher('unexpected_driver',Twist,'/cmd_vel_raw',BE)
    if bad=='terminal_missing':core.update_statuses([],now)
    if bad=='active_other':
        p.inputs['status']=([('a'*32,5),('56'*16,2)],now)
        core.update_statuses(p.inputs['status'][0],now)
    ep.before_step(now)
    assert core.goal_uuid=='a'*32


def test_cancel_only_path_requires_explicit_real_cancel_optin(monkeypatch,tmp_path):
    h,ep=waiting(monkeypatch,tmp_path)
    ep.node.core.enabled=False
    monkeypatch.setattr(ep.node.cancel_client,'service_is_ready',lambda:True)
    run_cancel(ep,h.bus.now)
    assert not h.bus.cancelled


@pytest.mark.parametrize('identity_gap',[False,True])
def test_fire_at_navigation_terminal_waits_for_real_drain_before_no_goal_rebind(monkeypatch,tmp_path,identity_gap):
    h=driving(monkeypatch,tmp_path);h.status=[('a'*32,4)];h.fire=True
    if identity_gap:
        h.identity_gap=True
        for _ in range(12):h.step()
        h.identity_gap=False
    h.until(lambda r:abs(h.yaw)>.025)
    assert h.op.core.cycles.current.nav_goal_uuid is None and h.bus.max_raw==1
    assert 'a'*32 in h.by_name['live_nav_stop_gateway'].core.used_goal_uuids
