"""Opt-in intent boundary: real heartbeat mapping and asynchronous proof client."""
import copy
import json
from types import SimpleNamespace as N
import pytest
from std_msgs.msg import Bool,String
from fire_robot_mission.operational_intents import IntentMapper,ARM_TOPICS
from fire_robot_mission.operational_executor import OperationalExecutor
from fire_robot_mission.operational_session_core import OperationalSessionCore
from fire_robot_mission.cycle_release import ROLES,CONTROL_TOPIC
from test_release_core import Chain,CYCLE,OLD


def session(*,no_goal=False,cycle=True):
    f=Chain(no_goal=no_goal);f.tick();c=OperationalSessionCore(lambda:CYCLE);c.armed=True
    if cycle:c.cycles.begin([] if no_goal else [(OLD,2)],1.,1.,armed=True,fire_confirmed=True)
    for key,(v,at) in f.master.inputs.items():
        if key in c.INPUTS:c.update(key,v,at)
    c.update('graph',dict(valid=True,raw_count=0,raw_owner=None),f.now)
    for role in ROLES:
        value=c.get(role,f.now) or {};value.update(cycle_protocol_enabled=True,
            cycle_release_ack=dict(protocol='CYCLE_RELEASE_V1',role=role,context=f.context,
                                   phase='ACTIVE',blockers=[]))
        c.update(role,value,f.now)
    h=dict(control=dict(protocol='CYCLE_RELEASE_V1',phase='ACTIVE',context=f.context) if cycle else None,
           release=None,consumed=False)
    return c,f,h


def report(**intents):return dict(intents=intents)


@pytest.mark.parametrize('intent,suffix', [('alignment','live_alignment_arm'),('ir','ir_approach_arm'),('pulse','pump_demo_arm')])
def test_exact_registered_goal_maps_only_requested_stage_heartbeat(intent,suffix):
    c,f,h=session();m=IntentMapper();p=m.step(c,report(**{intent:True}),h,f.now,safety_ok=True)
    assert p['arms']['/fire_mission/'+suffix]
    assert p['arms'][ARM_TOPICS[1]] and not p['physical_commands_published']
    assert all(not v for k,v in p['arms'].items() if k in (ARM_TOPICS[3],ARM_TOPICS[4],ARM_TOPICS[5]) and k!='/fire_mission/'+suffix)


@pytest.mark.parametrize('fault',['BOOT','safety','graph','consumer','fault','closed','unarmed'])
def test_lost_gate_revokes_all_intents_and_drops_control_heartbeat(fault):
    c,f,h=session()
    if fault=='BOOT':c.inputs['identity'][0]['state_age_ms']=351
    if fault=='graph':c.inputs['graph'][0]['valid']=False
    if fault=='consumer':c.inputs['ir_live'][0]['cycle_protocol_enabled']=False
    if fault=='fault':c.stop_fault('TEST')
    if fault=='closed':c.closed=True
    if fault=='unarmed':c.armed=False
    p=IntentMapper().step(c,report(alignment=True,ir=True,pulse=True),h,f.now,safety_ok=fault!='safety')
    assert not any(p['arms'].values()) and p['cycle_control'] is None
    assert not p['stationary_request']['active']


@pytest.mark.parametrize('fault',['missing','wrong_context','wrong_phase','blocker'])
def test_cycle_binding_waits_before_stage_but_continues_ACTIVE_registration(fault):
    c,f,h=session();ack=c.inputs['actor'][0]['cycle_release_ack']
    if fault=='missing':c.inputs['actor'][0].pop('cycle_release_ack')
    if fault=='wrong_context':ack['context']=dict(ack['context'],cycle_id='12'*16)
    if fault=='wrong_phase':ack['phase']='COMMITTED'
    if fault=='blocker':ack['blockers']=['WAIT_LOCAL_RESET']
    p=IntentMapper().step(c,report(pulse=True,ir=True,alignment=True),h,f.now,safety_ok=True)
    assert p['cycle_control']['phase']=='ACTIVE' and 'WAIT_ACTOR_CYCLE_BINDING' in p['blockers']
    assert not any(p['arms'][x] for x in ARM_TOPICS[2:])


def test_no_goal_context_registers_without_fake_cancel_or_motion_and_blocks_new_goal_forwarding():
    c,f,h=session(no_goal=True)
    p=IntentMapper().step(c,report(alignment=True,ir=True,pulse=True),h,f.now,safety_ok=True)
    assert p['cycle_control']['context']['nav_goal_uuid'] is None
    assert not any(p['arms'].values()) and not p['no_goal_motion_supported']


def test_normal_navigation_needs_no_fire_cycle_but_closing_withdraws_motion_keeps_exact_cancel():
    c,f,h=session(cycle=False);m=IntentMapper();p=m.step(c,report(),h,f.now,safety_ok=True)
    assert p['arms'][ARM_TOPICS[0]] and not any(p['arms'][x] for x in ARM_TOPICS[1:])
    c,f,h=session();h['release']=dict(phase='PREPARE');h['control']['phase']='PREPARE'
    p=m.step(c,report(pulse=True,ir=True,alignment=True),h,f.now,safety_ok=True)
    assert p['arms'][ARM_TOPICS[1]] and not any(p['arms'][x] for x in ARM_TOPICS if x!=ARM_TOPICS[1])


def test_stationary_I_token_stays_same_through_WAIT_P_and_P_until_release():
    c,f,h=session();c.pending_stationary=True;m=IntentMapper(lambda:'12'*16)
    first=m.step(c,report(),h,f.now,safety_ok=True)['stationary_request']
    second=m.step(c,report(pulse=True),h,f.now,safety_ok=True)['stationary_request']
    assert first==second and first['active'] and first['request_id']=='12'*16 and first['goal_uuid']==OLD
    h['release']=dict(phase='PREPARE');h['control']['phase']='PREPARE'
    assert not m.step(c,report(pulse=True),h,f.now,safety_ok=True)['stationary_request']['active']


def executor(enabled):
    params={};pubs={};created=[];pending=N(done=lambda:False,cancel=lambda:None)
    client=N(service_is_ready=lambda:True,call_async=lambda req:pending,remove_pending_request=lambda f:created.append('removed'))
    node=N(declare_parameter=lambda k,v:params.setdefault(k,v),get_parameter=lambda k:N(value=enabled),
           resolve_topic_name=lambda t:t,create_client=lambda *a:client)
    def pub(typ,t,q):
        created.append((typ,t));pubs[t]=[]
        return N(publish=lambda m:pubs[t].append(m.data))
    node.create_publisher=pub
    return N(executor=OperationalExecutor(node),pubs=pubs,created=created,pending=pending,client=client)


def test_disabled_executor_creates_no_publishers_client_or_output():
    a=executor(False);c,f,h=session()
    assert not a.created
    assert a.executor.step(c,report(pulse=True),h,f.now)==dict(
        enabled=False,live_intents_published=False,live_intents_active=False)


def test_enabled_unarmed_false_heartbeats_are_not_reported_as_active_live_intents():
    a=executor(True);c,f,h=session(cycle=False);c.armed=False;a.executor.safety_ok_at=f.now
    r=a.executor.step(c,report(),h,f.now)
    assert r['enabled'] and r['live_intents_published'] and not r['live_intents_active']
    assert not any(r['plan']['arms'].values())


def test_enabled_boundary_has_only_Bool_arms_String_requests_no_raw_or_pump_commands():
    a=executor(True);c,f,h=session();a.executor.safety_ok_at=f.now
    r=a.executor.step(c,report(pulse=True),h,f.now)
    assert r['enabled'] and r['live_intents_active'] and set(a.pubs)==set(ARM_TOPICS)|{CONTROL_TOPIC,'/fire_mission/r4d_stationary_request'}
    assert '/cmd_vel_raw' not in a.pubs and '/pump_cmd' not in a.pubs
    assert a.pubs[ARM_TOPICS[-1]]==[True]
    assert json.loads(a.pubs[CONTROL_TOPIC][-1])['context']['nav_goal_uuid']==OLD
    a.executor.close();a.executor.close()
    assert all(a.pubs[t][-1] is False for t in ARM_TOPICS)


@pytest.mark.parametrize('result',['good','wrong_timeout','exception','timeout'])
def test_safety_probe_is_asynchronous_and_rejects_wrong_or_expired_parameters(result):
    a=executor(True);e=a.executor;e.poll(1.)
    assert e.pending is not None and not e.safety_ok(1.)
    if result=='timeout':
        e.poll(4.1);assert e.pending is None and 'removed' in a.created and not e.safety_ok(4.1);return
    values=[N(type=4,string_value='/cmd_vel_raw'),N(type=4,string_value='/cmd_vel'),
            N(type=2,integer_value=1000 if result!='wrong_timeout' else 500),N(type=3,double_value=10.)]
    a.pending.done=lambda:True
    if result=='exception':
        def fail():raise RuntimeError('read failed')
        a.pending.result=fail
    else:a.pending.result=lambda:N(values=values)
    e.poll(1.1);assert e.safety_ok(1.1)==(result=='good')
    assert not e.safety_ok(11.2)


def test_target_wait_keeps_alignment_heartbeat_only_for_actual_alignment_owner():
    c,f,h=session();c.turn_permission=True;c.phase='WAIT_TARGET'
    c.inputs['alignment'][0].update(enabled=True,goal_uuid=OLD,publisher_owned=True,state='TARGET_HOLD')
    c.inputs['graph'][0].update(raw_count=1,raw_owner='live_alignment_takeover')
    r=c.report(f.now);assert r['intents']['alignment']
    c.cancel_requested=True;r=c.report(f.now);assert not r['intents']['alignment']
