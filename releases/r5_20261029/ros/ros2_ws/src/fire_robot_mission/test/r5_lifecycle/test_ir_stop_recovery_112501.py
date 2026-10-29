"""Regression112501: intentional IR STOP must not become an arm fault."""
import pytest
import sys
from pathlib import Path
from test_automatic_065731 import robot

sys.path.insert(0,str(Path(__file__).parents[1]/'r5_delta'))


@pytest.mark.parametrize('goal,variant',[(False,'brief_loss'),(True,'brief_loss'),(False,'center_shift'),(True,'center_shift'),(False,'expire'),(True,'expire')])
def test_automatic_IR_stop_recenter_resume_or_confirm_range(monkeypatch,tmp_path,goal,variant):
    h,path=robot(monkeypatch,tmp_path,goal=goal)
    h.fire=True;h.ir_flame=True
    h.until(lambda r:abs(h.yaw)>.025);h.centered=True
    h.until(lambda r:h.by_name['ir_approach_live'].core.phase=='MOVING')
    h.until(lambda r:h.x>.03)
    cycle=h.op.core.cycles.current;requested=h.op.core.cycles.budget.requested_at
    x=h.x;yaw=h.yaw
    if variant=='brief_loss':
        h.fire=False
        for _ in range(2):h.step()
        h.fire=True;h.centered=False
    elif variant=='center_shift':h.centered=False
    else:
        h.fire=False
        h.until(lambda r:r['budget']['exhausted'],limit=340)
        h.fire=True;h.centered=False;h.right=950
    h.until(lambda r:h.yaw>yaw+.025,limit=150)
    stopped=h.x
    assert h.by_name['ir_approach_live'].core.phase!='ABORTED'
    assert h.op.core.cycles.current==cycle and h.op.core.cycles.budget.requested_at==requested
    assert h.op.core.cycles.budget.distance_m>=x-1e-6
    h.centered=True
    if variant!='expire':h.until(lambda r:h.x>stopped+.01,limit=150)
    else:
        for _ in range(10):h.step()
        assert h.x==stopped
    h.right=950
    h.until(lambda r:h.home==1,limit=350)
    assert len(h.pulses)==1 and h.bus.max_raw==1
    assert h.x<=.30+1e-6 and h.off_at-h.on_at<=5.1+1e-6
    h.fire=False;h.ir_flame=False
    h.until(lambda r:r['handback']['consumed'])
    h.status=[('56'*16,2)]
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)


@pytest.mark.parametrize('delay_ticks',[0,2,5])
def test_false_IR_arm_before_delayed_stop_request_still_zero_then_recover(monkeypatch,tmp_path,delay_ticks):
    import json
    h,path=robot(monkeypatch,tmp_path,goal=True)
    h.fire=True;h.ir_flame=True
    h.until(lambda r:abs(h.yaw)>.025);h.centered=True
    h.until(lambda r:h.by_name['ir_approach_live'].core.phase=='MOVING');h.until(lambda r:h.x>.03)
    core=h.by_name['ir_approach_live'].core;budget=h.op.core.cycles.budget.requested_at
    dropping=[True]
    def lag(p,s,msg):
        return (dropping[0] and s.info.node_name=='ir_approach_live' and
            (p.topic in ('/fire_mission/state','/fire_mission/ir_range_preview') or
             p.topic=='/fire_mission/cycle_control' and 'ir_stop_request' in json.loads(msg.data)))
    h.bus.drop=lag;h.fire=False
    for _ in range(max(delay_ticks,1)):h.step()
    dropping[0]=False
    # Preserve enough actual target loss for the slow preview to catch up.
    for _ in range(5):h.step()
    h.fire=True;h.centered=False;yaw=h.yaw
    h.until(lambda r:h.yaw>yaw+.025,limit=150)
    stopped=h.x;h.centered=True
    h.until(lambda r:h.x>stopped+.01,limit=150);h.right=950
    h.until(lambda r:h.home==1,limit=350)
    assert core.phase!='ABORTED' and h.op.core.cycles.budget.requested_at==budget
    assert h.bus.max_raw==1 and len(h.pulses)==1


@pytest.mark.parametrize('goal,stage',[(False,'ALIGN'),(True,'ALIGN'),(False,'IR'),(True,'IR')])
def test_true_fire_clear_before_spray_returns_newgoal_at_each_motion_stage(monkeypatch,tmp_path,goal,stage):
    h,path=robot(monkeypatch,tmp_path,goal=goal)
    h.fire=True;h.ir_flame=True
    h.until(lambda r:abs(h.yaw)>.025)
    if stage=='IR':
        h.centered=True;h.until(lambda r:h.by_name['ir_approach_live'].core.phase=='MOVING');h.until(lambda r:h.x>.03)
    h.fire=False;h.ir_flame=False
    h.until(lambda r:r['handback']['consumed'],limit=250)
    assert h.on_at is None and not h.op.core.closed and h.bus.max_raw==1
    h.status=[('56'*16,2)]
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)


@pytest.mark.parametrize('bad',['no_request','wrong_step','invalid_cause','owner_fault','scan_fault','odom_fault'])
def test_planned_IR_stop_never_masks_other_faults(bad):
    from fire_robot_mission.ir_approach_live_core import IrApproachLiveCore
    c=IrApproachLiveCore()
    c.r5_stop_request=dict(schema_version=1,step_count=c.step_count,cause='PERCEPTION_HOLD')
    blockers=['OPERATOR_ARM_MISSING_OR_STALE']
    if bad=='no_request':c.r5_stop_request=None
    if bad=='wrong_step':c.r5_stop_request['step_count']+=1
    if bad=='invalid_cause':c.r5_stop_request['cause']='UNRECOGNIZED'
    if bad=='owner_fault':blockers+=['RAW_OWNER_CONFLICT']
    if bad=='scan_fault':blockers+=['SCAN_PUBLISHER_INVALID']
    if bad=='odom_fault':blockers+=['ODOM_MISSING_OR_STALE']
    assert not c._planned_stop(blockers)


@pytest.mark.parametrize('fault',[None,'no_receipt','wrong_cycle','epoch','graph','raw','pump','moving','cmd',
    'guard_other','mixed_guard','active_goal','not_accepted','no_status','duplicate_cancel'])
def test_alignment_clear_requires_exact_cancel_history_and_current_STOP(fault):
    import copy
    from test_release_core import Chain
    from fire_robot_mission.cycle_release import ReleaseInputs,ReleaseContext
    c=Chain();c.tick()
    p=ReleaseInputs();p.inputs=copy.deepcopy(c.master.inputs)
    ctx=ReleaseContext.parse(c.context)
    p.stop_errors(ctx,c.now,True,0,0)
    assert (ctx.cycle_id,ctx.nav_goal_uuid,ctx.identity) in p.cancel_seen
    g,at=p.inputs['guard'];g.update(state='FAILED',blockers=['ALIGNMENT_STOP_PROOF_INVALID'])
    if fault=='no_receipt':p.cancel_seen.clear()
    if fault=='wrong_cycle':ctx=ReleaseContext.parse(dict(c.context,cycle_id='98'*16))
    if fault=='epoch':p.inputs['identity'][0]['connection_epoch']=2
    if fault=='moving':p.inputs['odom']=([.03,0,0,0,0,0],c.now)
    if fault=='cmd':p.inputs['cmd']=([.01,0,0,0,0,0],c.now)
    if fault=='guard_other':g['blockers']=['CANCEL_SERVICE_FAILED']
    if fault=='mixed_guard':g['blockers'].append('CANCEL_SERVICE_FAILED')
    if fault=='active_goal':p.inputs['status']=([('98'*16,2)],c.now)
    if fault=='not_accepted':g['cancel_service_accepted']=False
    if fault=='no_status':g['cancel_status_seen']=False
    if fault=='duplicate_cancel':g['send_count']=2
    errors=p.stop_errors(ctx,c.now,fault!='graph',1 if fault=='raw' else 0,1 if fault=='pump' else 0)
    assert bool(errors)==(fault is not None),errors


@pytest.mark.parametrize('fault',[None,'bool_schema','wrong_step','invalid_cause','invalid_budget',
    'expired_control','wrong_source','changed_epoch','closing'])
def test_IR_stop_request_boundary_cannot_grant_unbound_recovery(monkeypatch,tmp_path,fault):
    import copy
    h,path=robot(monkeypatch,tmp_path,goal=True)
    h.fire=True;h.ir_flame=True
    h.until(lambda r:abs(h.yaw)>.025);h.centered=True
    h.until(lambda r:h.by_name['ir_approach_live'].core.phase=='MOVING')
    node=h.by_name['ir_approach_live'];ep=node._cycle;c=node.core;p=ep.participant
    p.request=copy.deepcopy(p.request)
    stop=dict(schema_version=1,step_count=c.step_count,cause='PERCEPTION_HOLD',
        budget_started_at=h.op.core.cycles.budget.requested_at)
    p.request['ir_stop_request']=stop
    if fault=='bool_schema':stop['schema_version']=True
    if fault=='wrong_step':stop['step_count']+=1
    if fault=='invalid_cause':stop['cause']='arbitrary'
    if fault=='invalid_budget':stop['budget_started_at']=float('nan')
    if fault=='expired_control':p.request_at=h.bus.now-.351
    if fault=='wrong_source':monkeypatch.setattr(ep,'_source',lambda topic:False)
    if fault=='changed_epoch':p.inputs['identity'][0]['connection_epoch']+=1
    if fault=='closing':p.request['phase']='PREPARE'
    ep.before_step(h.bus.now)
    assert (c.r5_stop_request is not None)==(fault is None)
