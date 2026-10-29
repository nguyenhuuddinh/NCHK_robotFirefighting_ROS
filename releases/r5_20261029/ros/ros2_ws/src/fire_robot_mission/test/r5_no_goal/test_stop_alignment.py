import copy
import pytest
from fixtures import StopPair,fixture_module
from fire_robot_mission.no_goal_cycle import stopped_pair
from fire_robot_mission.operational_cycle import valid_uuid


def test_actual_gateway_guard_empty_status_hold_stop_and_send_no_cancel():
    p=StopPair();g,c=p.tick()
    assert not g['no_goal_proof']['valid'] and not c['no_goal_proof']['valid']
    g,c=p.qualify()
    assert stopped_pair(g,c,p.context)
    assert g['goal_uuid'] is None and g['cycle_goal_uuid'] is None and c['goal_uuid'] is None
    assert c['send_count']==0 and c['cancel_service_accepted'] is False and c['cancel_status_seen'] is False
    assert c['goal_status'] is None and p.guard.sent_at is None


@pytest.mark.parametrize('fault',['active_goal','graph','gap','different_cycle'])
def test_stop_witness_does_not_synthesize_release_on_missing_or_changed_proof(fault):
    p=StopPair();p.qualify()
    if fault=='active_goal':g,c=p.tick(active='12'*16)
    elif fault=='graph':g,c=p.tick(graph=False)
    elif fault=='gap':
        p.now+=1.;g,c=p.tick(moving=True)
        assert not g['no_goal_proof']['stationary_released'];return
    else:
        p.guard.r5_cycle_context=dict(p.context,cycle_id='12'*16);g,c=p.tick()
    assert not stopped_pair(g,c,p.context)


def alignment():
    p=StopPair();p.qualify();m=fixture_module('test_live_alignment_takeover.py');core=m.ready()
    core.r5_cycle_context=p.context;core.update_statuses([])
    core.update('gateway',p.g,10.);core.update('guard',p.c,10.)
    # No shadow cancel proof is supplied to the no-goal path.
    core.inputs.pop('handoff',None)
    return core,p,m


def test_real_alignment_no_goal_scanned_turn_then_center_zero_release_preserves_None():
    core,p,m=alignment();angular,r,action=m.step(core,10.01,counts=dict(m.COUNTS,handoff=0))
    assert action=='CREATE' and r['goal_uuid'] is None and r['cycle_id']==p.context['cycle_id']
    angular,r,action=m.step(core,10.06,owned=True,raw=m.RAW,counts=dict(m.COUNTS,handoff=0))
    assert angular>0 and r['state']=='TURNING' and r['goal_uuid'] is None
    for now in (10.1,10.2):
        core.update('preview',m.preview('CENTERED',0.,.01),now);core.update('mission',m.mission(),now)
        core.update_odom(m.ZERO,now);core.update_cmd(m.ZERO,now);core.update_scan(-3.14,3.14,[1.]*360,now);core.update_arm(True,now)
        angular,r,_=m.step(core,now,owned=True,raw=m.RAW,counts=m.COUNTS)
    assert r['state']=='STOPPING' and angular==0.
    for i in range(8):
        now=10.3+i*.1;core.update_odom(m.ZERO,now);core.update_cmd(m.ZERO,now)
        angular,r,action=m.step(core,now,owned=True,raw=m.RAW)
        if action=='RELEASE':break
    assert action=='RELEASE' and r['state']=='COMPLETE' and r['goal_uuid'] is None


@pytest.mark.parametrize('fault',['other_cycle','active_goal','unsafe_scan','raw_owner','missing_guard','fake_cancel','stale'])
def test_no_goal_alignment_keeps_all_remaining_gates(fault):
    core,p,m=alignment();kwargs={};now=10.01
    if fault=='other_cycle':core.inputs['guard'][0]['r5_cycle_context']=dict(p.context,cycle_id='12'*16)
    if fault=='active_goal':core.update_statuses([('12'*16,2)])
    if fault=='unsafe_scan':core.update_scan(-3.14,3.14,[.1]*360,10.)
    if fault=='raw_owner':kwargs['raw']=m.RAW
    if fault=='missing_guard':core.inputs.pop('guard')
    if fault=='fake_cancel':core.inputs['guard'][0].update(send_count=1,cancel_status_seen=True,goal_status=5)
    if fault=='stale':now=10.5
    angular,r,action=m.step(core,now,**kwargs)
    assert angular==0. and action is None and r['blockers']


def test_registered_no_goal_context_does_not_replace_an_existing_real_goal_binding():
    core,p,m=alignment();core.goal_uuid='34'*16
    # An endpoint must reject this stale bound core before admission.
    from fire_robot_mission.cycle_endpoint import CycleEndpoint
    from test_cycle_endpoint import adapter
    a=adapter('alignment',core=core);a.endpoint.participant.context=__import__('fire_robot_mission.cycle_release',fromlist=['ReleaseContext']).ReleaseContext.parse(p.context)
    a.endpoint.decorate({},10.)
    assert a.endpoint.participant.error=='LOCAL_CORE_GOAL_MISMATCH'
