import copy
import json
from types import SimpleNamespace as N
import pytest
from std_msgs.msg import String
from fire_robot_mission.cycle_release import ReleaseContext,CONTROL_TOPIC,ROLES
from fire_robot_mission.operational_discovery import Discovery
from fire_robot_mission.operational_intents import IntentMapper,ARM_TOPICS
from fire_robot_mission.operational_cycle import CycleRegistry
from fire_robot_mission.operational_review import OperationalReview
from fire_robot_mission.operational_identity import IDENTITY_TOPIC
from fire_robot_mission import cycle_endpoint as ep
from fixtures import StopPair,fixture_module
from test_ir_pulse import IR,no_goal_data
from test_session_stationary import session
from test_session_core import feed
from test_release_core import Chain
from test_cycle_endpoint import adapter
from test_installed_pulse import adapter as pulse_adapter,endpoint


@pytest.mark.parametrize('terminal',[4,5,6])
def test_no_goal_context_after_old_terminal_is_not_a_fake_cancel_of_old_UUID(terminal):
    p=StopPair();old='12'*16
    for _ in range(8):
        p.now+=.1;n=p.now;p.gw.update_statuses([(old,terminal)],n);p.guard.update_statuses([(old,terminal)],n)
        for core in (p.gw,p.guard):core.update_cmd((0.,)*6,n);core.update_odom((0.,)*6,n)
        p.gw.update_nav((0.,)*6,n);p.gw.update_arm(True,n)
        p.guard.update_topology(1,1,1,p.nav.NAV_SOURCES,0,1,1,False)
        _,_,g,_=p.gw.evaluate(n,False,[],p.nav.NAV_SOURCES,p.nav.SAFETY_SOURCE,1,1,1,1,1)
        c,uuid=p.guard.step(n)
        assert uuid is None
    assert c['no_goal_proof']['valid'] and c['send_count']==0 and c['goal_uuid'] is None
    assert p.guard.statuses==[(old,terminal)] and c['goal_status'] is None


def test_stop_hold_must_requalify_after_motion_or_sampling_gap():
    p=StopPair();p.qualify();p.tick(moving=True);g,c=p.tick()
    assert not g['no_goal_proof']['stationary_released'] and not c['no_goal_proof']['stationary_released']
    p.qualify();p.now+=.4;g,c=p.tick()
    assert not g['no_goal_proof']['stationary_released'] and not c['no_goal_proof']['stationary_released']


def test_no_goal_reentry_after_REALIGN_uses_same_IR_cycle_and_distance():
    a=IR();a.step(2.);a.step(2.31);a.step(2.4,owned=True)
    a.step(2.5,owned=True,x=.01,speed=.08,fault='source')
    # Source failures are hard motion stops, not blindly relabeled realign.
    assert a.core.stop_reason!='REALIGN_REQUIRED'
    # Exercise the legitimate recoverable state made by the IR policy.
    a.core.phase='COMPLETE';a.core.stop_reason='REALIGN_REQUIRED';a.core.total_progress_m=.04
    a.core.step_count=1;a.core.cycle_id=a.pair.context['cycle_id']
    speed,r,action=a.step(3.)
    assert r['state']=='WAITING_FOR_ADMISSION' and r['total_progress_m']==.04 and action is None
    speed,r,action=a.step(3.31)
    assert action=='CREATE' and r['cycle_id']==a.pair.context['cycle_id'] and r['goal_uuid'] is None
    assert r['total_progress_m']==.04 and r['step_count']==1


@pytest.mark.parametrize('report_fault',['cycle','context','missing'])
def test_review_will_not_accept_None_goal_from_another_cycle(report_fault):
    p,d=no_goal_data();r=OperationalReview();pulse=dict(schema_version=1,mode='SUPERVISED_DEMO',one_shot=True,
        state='FIRING',pump_on=True,goal_uuid=None,cycle_id=p.context['cycle_id'],r5_cycle_context=p.context)
    actor=dict(request_used=True,goal_uuid=None,cycle_id=p.context['cycle_id'],pulse_core=pulse,
        cycle_release_ack=dict(protocol='CYCLE_RELEASE_V1',role='actor',context=p.context,phase='ACTIVE'))
    if report_fault=='cycle':actor['cycle_id']='12'*16
    if report_fault=='context':pulse['r5_cycle_context']=dict(p.context,cycle_id='12'*16)
    if report_fault=='missing':pulse.pop('cycle_id')
    r.update('actor',actor,1.);result=r.step(1.,graph_ok=True)
    assert result['state']=='WAITING_FOR_PULSE' and result['blockers']


def test_discovery_no_goal_key_keeps_goal_uuid_None_and_scopes_the_cycle():
    p=dict(node='_NODE_NAME_UNKNOWN_',namespace='_NODE_NAMESPACE_UNKNOWN_',
        type='geometry_msgs/msg/Twist',reliability='ReliabilityPolicy.BEST_EFFORT',gid='01'+'00'*23)
    d=Discovery();ctx=dict(valid=True,goal_uuid=None,cycle_id='cd'*16)
    r=d.observe(1.,[p],ctx)
    assert r['status']=='PENDING' and r['goal_uuid'] is None and r['cycle_id']=='cd'*16
    p.update(node='ir_approach_live',namespace='/')
    r=d.observe(1.1,[p],dict(ctx,cycle_id='12'*16))
    assert r['status']=='INVALID' and r['reason']=='RAW_DISCOVERY_IDENTITY_CHANGED'


def test_endpoint_rejects_bound_cycle_before_any_step_and_pin_change_latches(monkeypatch):
    a=adapter('ir_live');p=StopPair();a.clock.now=2.;monkeypatch.setattr(ep.time,'monotonic',lambda:a.clock.now)
    a.endpoint.on_control(String(data=json.dumps(dict(protocol='CYCLE_RELEASE_V1',phase='ACTIVE',context=p.context))))
    a.node.core.cycle_id='12'*16;a.node.core.update_arm(True,2.)
    a.endpoint.before_step(2.)
    assert a.endpoint.participant.error=='LOCAL_CORE_CYCLE_MISMATCH' and not a.node.core.arm


def test_actor_no_goal_real_tick_proof_loss_during_pulse_requests_OFF_not_pulse_replay(monkeypatch):
    a=pulse_adapter(monkeypatch,cycle_protocol=True);p,d=no_goal_data();ctx=p.context;f=Chain(no_goal=True)
    for t,name in [('/fire_mission/state','fire_mission_supervisor'),('/fire_mission/live_nav_stop_state','live_nav_stop_gateway'),
                   ('/fire_mission/live_alignment_state','live_alignment_takeover'),('/fire_mission/ir_approach_live_state','ir_approach_live')]:a.pubs[t]=[endpoint(name)]
    a.pubs[CONTROL_TOPIC]=[endpoint('fire_robot_operator','std_msgs/msg/String',__import__('rclpy.qos',fromlist=['ReliabilityPolicy']).ReliabilityPolicy.RELIABLE,90)]
    a.pubs[IDENTITY_TOPIC]=[endpoint('serial_bridge_node',seed=91)]
    monkeypatch.setattr(a.node,'destroy_publisher',lambda pub:a.pubs.pop('/pump_cmd',None))
    def tick(arm,stale=False):
        f.tick();n=f.now;a.clock.now=n
        a.node._cycle.on_control(String(data=json.dumps(dict(protocol='CYCLE_RELEASE_V1',phase='ACTIVE',context=ctx))))
        a.feed(d,n,arm=arm);a.node.session.update('status',[],n)
        for k,(v,at) in f.master.inputs.items():a.node._cycle.participant.update(k,v,at)
        a.node._cycle.participant.update('actuator',a.node.session.actuator(n),n)
        if stale:a.node._cycle.participant.inputs['identity'][0]['state_age_ms']=351
        a.node.tick()
        return json.loads(a.sent['/fire_mission/r4d_state'][-1])
    tick(False)
    for _ in range(7):r=tick(True)
    assert r['proposed_pump_on']
    d['actuator'].update(pump_pwm_commanded_on=True,servo_pwm_commanded_on=True,servo_phase='SWEEPING',servo_commanded_angle_deg=60.)
    r=tick(True);assert r['pump_on_feedback_seen']
    r=tick(True,stale=True)
    assert not a.sent['/pump_cmd'][-1] and a.node.session.used and a.node.session.phase=='VERIFY_OFF'
    # Fresh proof cannot clear the used P flag or resume ON.
    for _ in range(8):r=tick(True)
    assert not r['proposed_pump_on'] and a.node.session.used


def test_expired_I_during_identity_hold_releases_IR_owner_after_bounded_arm_wait(monkeypatch):
    a=adapter('ir_live');f=Chain();f.tick();ctx=f.context;now=f.now
    a.node.core.phase='MOVING';a.node.core.goal_uuid='a'*32;a.node.core.arm=False
    a.node._raw_publisher=N(publish=lambda msg:None);a.node.state_publisher=N(publish=lambda msg:None)
    a.endpoint.participant.context=ReleaseContext.parse(ctx);a.endpoint.resume_wait=True
    a.endpoint.graph_ok=True;a.endpoint.last_payload=dict(schema_version=1,enabled=True,state='MOVING',goal_uuid='a'*32)
    a.endpoint.participant.request_at=now
    assert a.endpoint.pause_tick(now)
    a.endpoint.participant.request_at=now+.2
    assert a.endpoint.pause_tick(now+.2)
    a.endpoint.participant.request_at=now+.41
    assert not a.endpoint.pause_tick(now+.41)
    assert a.node.core.r5_identity_release


@pytest.mark.parametrize('lost',['control','graph'])
def test_identity_resume_pause_cannot_hide_expired_control_or_graph_loss(lost):
    a=adapter('alignment');f=Chain();f.tick();now=f.now
    a.endpoint.participant.context=ReleaseContext.parse(f.context)
    a.endpoint.participant.request_at=now
    a.endpoint.graph_ok=lost!='graph';a.endpoint.resume_wait=True
    a.node.core.arm=False;a.node._raw_publisher=N(publish=lambda msg:pytest.fail('pause must fall through'))
    if lost=='control':now+=.36
    assert not a.endpoint.pause_tick(now) and not a.endpoint.resume_wait


def test_actual_IR_recovery_release_reason_is_not_a_new_motion_budget():
    a=IR();a.step(2.);a.step(2.31);a.step(2.4,owned=True,x=.01)
    a.core.r5_identity_release=True
    speed,r,_=a.step(2.5,owned=True,x=.02,fault='arm')
    assert speed==0. and r['stop_reason']=='REALIGN_REQUIRED' and r['state']=='STOPPING'
    assert a.core.sensor_recovery_active and a.core.cycle_id==a.pair.context['cycle_id']


def test_actual_no_goal_alignment_target_loss_keeps_zero_then_same_cycle_reacquires():
    m=fixture_module('test_live_alignment_takeover.py');p=StopPair();p.qualify();core=m.ready()
    core.r5_cycle_context=p.context;core.update_statuses([])
    core.update('gateway',p.g,10.);core.update('guard',p.c,10.)
    assert m.step(core,10.01)[2]=='CREATE'
    assert m.step(core,10.06,owned=True,raw=m.RAW)[0]>0
    missing=m.mission();missing['state']='TARGET_LOST';missing['perception']=dict(detected_now=False,confirmed=False)
    core.update('mission',missing,10.1);core.update('preview',m.preview('NO_TARGET',0.),10.1)
    core.update_scan(-3.14,3.14,[1.]*360,10.1);core.update_odom(m.ZERO,10.1);core.update_cmd(m.ZERO,10.1);core.update_arm(True,10.1)
    angular,r,_=m.step(core,10.11,owned=True,raw=m.RAW)
    assert angular==0. and r['target_reacquiring'] and r['cycle_id']==p.context['cycle_id']
    core.update('mission',m.mission(),10.2);core.update('preview',m.preview(),10.2)
    core.update_scan(-3.14,3.14,[1.]*360,10.2);core.update_odom(m.ZERO,10.2);core.update_cmd(m.ZERO,10.2);core.update_arm(True,10.2)
    angular,r,_=m.step(core,10.21,owned=True,raw=m.RAW)
    assert angular>0 and r['goal_uuid'] is None and r['cycle_id']==p.context['cycle_id']


def test_discovery_can_reinitialize_after_typed_retirement_without_faking_goal():
    from fire_robot_mission.operational_graph import SessionGraph
    from test_session_adapter import node
    # Pure coordinator retirement, then exercise actual graph adapter metadata.
    from fire_robot_mission.operational_session_core import OperationalSessionCore
    g=SessionGraph();c=OperationalSessionCore(lambda:'cd'*16)
    c.cycles.begin([],1.,1.,armed=True,fire_confirmed=True)
    chain=Chain(no_goal=True);chain.finish();c.cycles.retire(chain.master.receipt,chain.now,expected_context=chain.context)
    g.discovery_cycle='cd'*16;old=g.discovery;g.discovery.resolved=True
    # No DDS sources required to exercise the reset branch; graph remains WAIT.
    class Fake:
        state_qos=N(reliability=__import__('rclpy.qos',fromlist=['ReliabilityPolicy']).ReliabilityPolicy.BEST_EFFORT)
        def get_publishers_info_by_topic(self,t):return []
    now=chain.now
    for key,value in [('cmd',[0.]*6),('odom',[0.]*6),('actuator',chain.master.inputs['actuator'][0]),
                      ('graph',dict(raw_count=0,raw_owner=None,valid=True))]:c.update(key,value,now)
    g.check(Fake(),c,now,[],__import__('rclpy.qos',fromlist=['ReliabilityPolicy']).ReliabilityPolicy.RELIABLE)
    assert g.discovery is not old and g.discovery_cycle is None and not g.discovery.resolved
