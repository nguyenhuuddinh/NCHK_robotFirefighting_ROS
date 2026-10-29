import copy
import pytest
from fixtures import StopPair
from test_session_core import data,feed,actor_ready
from test_release_core import Chain
from test_ir_pulse import no_goal_data
from fire_robot_mission.operational_session_core import OperationalSessionCore
from fire_robot_mission.operational_intents import IntentMapper,ARM_TOPICS
from fire_robot_mission.stationary_confirmation import proof_errors
from fire_robot_mission.pulse_session import PulseSession
from fire_robot_mission.cycle_release import ROLES


def session():
    p,d=no_goal_data();f=Chain(no_goal=True);c=OperationalSessionCore(lambda:p.context['cycle_id'])
    d['ir_raw']=data()['ir_raw']
    d.update(cmd=[0.]*6,odom=[0.]*6,graph=dict(valid=True,raw_count=0,raw_owner=None,nav_status_event_stream=True))
    d['preview']['preview']=dict(error_x_normalized=.01,target_angular_z_rad_s=0.)
    d['vision']['fire']=dict(detected_now=False,confirmed=False)
    for i in range(32):
        f.tick();d['identity']=f.master.inputs['identity'][0];feed(c,d,f.now)
    assert feed(c,d,f.now,['A'])['armed']
    d['vision']['fire']=dict(detected_now=True,confirmed=True)
    r=feed(c,d,f.now+.01,['F'])
    return c,d,p,f


def test_no_goal_session_I_confirm_separate_P_and_mapper_never_fabricates_status():
    c,d,p,f=session();now=f.now+.01
    assert c.cycles.current.nav_goal_uuid is None and c.phase=='WAIT_IR_I'
    # Context is retained independently of earlier navigation terminal statuses.
    assert c.inputs['status'][0]==[]
    # The operator consumes the actor's explicit confirmation receipt; fresh
    # ADC sequences alone no longer fabricate that independent receipt.
    actor_ready(c,d)
    d['actor']['pump_publisher_registered']=False
    r=feed(c,d,now+.01,['I']);assert r['operator_feedback']['accepted']
    for i in range(8):
        d['ir']['ir_sample_seq']+=1;r=feed(c,d,now+.1*(i+1))
    assert r['phase']=='IR_CONFIRM' and not c.p_used
    d['actor']['stationary_confirmation']=dict(state='COMPLETE',goal_uuid=None)
    r=feed(c,d,now+.85)
    assert r['phase']=='WAIT_P' and not c.p_used
    actor_ready(c,d)
    d['actor']['stationary_confirmation']=dict(state='COMPLETE',goal_uuid=None)
    r=feed(c,d,now+.9,['P']);assert r['operator_feedback']['accepted'] and c.p_used
    for role in ROLES:
        value=d.setdefault(role,{})
        value.update(cycle_protocol_enabled=True,no_goal_supported=True,cycle_release_ack=dict(
            protocol='CYCLE_RELEASE_V1',role=role,phase='ACTIVE',blockers=[],context=p.context))
        c.update(role,value,now+.9)
    h=dict(control=dict(protocol='CYCLE_RELEASE_V1',phase='ACTIVE',context=p.context),release=None)
    m=IntentMapper(lambda:'34'*16);plan=m.step(c,r,h,now+.9,safety_ok=True)
    assert plan['no_goal_motion_supported'] and plan['arms'][ARM_TOPICS[-1]]
    assert plan['stationary_request']['goal_uuid'] is None
    assert plan['stationary_request']['cycle_context']==p.context
    assert plan['cycle_control']['context']['nav_goal_uuid'] is None


@pytest.mark.parametrize('fault',['different_cycle','active_goal','no_proof','no_identity'])
def test_session_no_goal_cannot_admit_I_from_None_equality_or_retained_old_proof(fault):
    c,d,p,f=session();now=f.now+.02
    if fault=='different_cycle':d['guard']['r5_cycle_context']=dict(p.context,cycle_id='12'*16)
    if fault=='active_goal':d['status']=[('12'*16,2)]
    if fault=='no_proof':d['gateway'].pop('no_goal_proof')
    if fault=='no_identity':d['identity']['valid']=False
    r=feed(c,d,now,['I'])
    assert not r['operator_feedback']['accepted'] and c.cycles.budget.requested_at is None
    assert not r['closed']


@pytest.mark.parametrize('band',['SPRAY_RANGE','TOO_CLOSE'])
def test_stationary_no_goal_I_collects_five_actual_samples_and_proof_replay_uses_cycle(band):
    p,d=no_goal_data();ctx=p.context;s=PulseSession(enabled=True,allow_provisional_ir=True,allow_close_stationary=True)
    s.r5_cycle_context=ctx;s.pump.r5_cycle_context=ctx
    d['ir_live'].update(state='WAITING_FOR_ADMISSION',cycle_id=None,goal_uuid=None,one_shot=True,
        step_count=0,total_progress_m=0.,stop_reason='',recovery_required=False,
        blockers=['OPERATOR_ARM_MISSING_OR_STALE'],safety=dict(pump_command_publisher=False))
    d['ir'].update(ir_band=band,zone=band,filtered_counts=dict(left=1400,center=5000,right=750 if band=='TOO_CLOSE' else 950))
    counts={k:1 for k in ('ir','ir_live','mission','alignment','gateway','guard','actuator','odom','cmd','status','vision','preview','stationary_request','arm','pump_subscribers')}
    counts.update(pump=0,pump_total=0,raw=0)
    request=dict(schema_version=1,active=False,request_id='34'*16,goal_uuid=None,cycle_context=ctx,range_policy='CLOSE_STATIONARY_TEST_V1')
    def tick(now):
        for key,value in d.items():s.update(key,copy.deepcopy(value),now)
        s.update('cmd',(0.,)*6,now);s.update('odom',(0.,)*6,now)
        s.update('stationary_request',copy.deepcopy(request),now);s.update('operator',False,now)
        s.feed_pump(now);s.stationary.step(s,now,counts)
    tick(2.);request['active']=True;tick(2.1)
    for i in range(5):d['ir']['ir_sample_seq']+=1;tick(2.2+i*.1)
    assert s.stationary.state=='COMPLETE',s.stationary.report()
    proof=s.stationary.proof
    assert proof['goal_uuid'] is None and proof['cycle_context']==ctx
    assert proof_errors(proof)==[]
    bad=copy.deepcopy(proof);bad['cycle_context']['cycle_id']='12'*16
    assert proof_errors(bad)
