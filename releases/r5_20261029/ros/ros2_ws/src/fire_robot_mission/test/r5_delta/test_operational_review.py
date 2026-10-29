"""Canonical review delta; consume actual installed actor pump diagnostics."""
import copy
import json
from types import SimpleNamespace as N
import pytest
from std_msgs.msg import String
from fire_robot_mission.operational_review import OperationalReview
from fire_robot_mission import post_spray_review as ros
from fire_robot_mission.operational_contract import PULSE_STATE_TOPIC,PULSE_NODE
from test_installed_pulse import adapter,ready,endpoint
from test_release_core import Chain,OLD


def ack(context,phase='ACTIVE'):
    return dict(protocol='CYCLE_RELEASE_V1',role='actor',context=context,
                phase=phase,blockers=[],reset_completed=phase=='COMMITTED')


class Flow:
    def __init__(self,monkeypatch):
        self.actor=adapter(monkeypatch);self.review=OperationalReview();self.d=ready();self.chain=Chain()
        self.seq=100;self.report={};self.actor_report={}
    def step(self,now,*,arm=False,clear=False):
        a=self.actor;d=self.d
        a.feed(d,now,arm=arm);a.node.tick()
        self.actor_report=json.loads(a.sent[PULSE_STATE_TOPIC][-1])
        self.actor_report['cycle_release_ack']=ack(self.chain.context)
        self.seq+=1
        for key in ('vision','ir','mission'):
            value=copy.deepcopy(d[key])
            if clear:
                if key=='vision':value['fire']=dict(detected_now=False,confirmed=False)
                if key=='ir':
                    value.update(ir_band='NO_FLAME_SIGNATURE',zone='UNKNOWN',
                        blockers=['NO_IR_FLAME_SIGNATURE'],ir_sample_seq=self.seq,
                        filtered_counts=dict(left=26000,center=26000,right=26000))
            self.review.update(key,value,now)
        self.review.update('alignment',d['preview'],now)
        self.review.update('actor',self.actor_report,now)
        self.report=self.review.step(now,graph_ok=True)
        return self.report

    def pulse(self):
        self.step(1.);self.step(1.1)
        for i in range(101):
            at=1.2+i*.1
            if at>=1.9:self.d['actuator'].update(pump_pwm_commanded_on=True,servo_pwm_commanded_on=True,
                servo_phase='SWEEPING',servo_commanded_angle_deg=60.)
            if at>=6.9:self.d['actuator'].update(pump_pwm_commanded_on=False,servo_phase='RETURNING_HOME')
            if at>=8.1:self.d['actuator'].update(servo_phase='IDLE',servo_pwm_commanded_on=False,
                servo_commanded_angle_deg=None,home_command_completed_count=1)
            report=self.step(at,arm=True,clear=at>=8.1)
            if report['state']=='NO_FIRE_EVIDENCE' and self.actor_report['state']=='DONE':return at
        raise AssertionError((self.actor_report,self.report))


def test_real_pulse_diagnostics_feed_canonical_NO_FIRE_without_actor_DONE_substitution(monkeypatch):
    f=Flow(monkeypatch);f.pulse()
    assert f.actor_report['pulse_core']['state']=='COMPLETE'
    assert f.actor_report['pulse_core']['stop_reason']=='PULSE_TIME_LIMIT'
    assert f.report['state']=='NO_FIRE_EVIDENCE' and f.report['blockers']==[]
    assert f.report['pulse_index']==1 and f.report['goal_uuid']==OLD
    assert f.report['safety']['actuation_authorized'] is False


def test_COMMITTED_reset_preserves_actual_terminal_history_only_in_same_cycle(monkeypatch):
    f=Flow(monkeypatch);now=f.pulse();old=copy.deepcopy(f.actor_report)
    reset=dict(old,state='WAIT_P',goal_uuid=None,request_used=False,pulse_core=None,
               cycle_release_ack=ack(f.chain.context,'COMMITTED'))
    now+=.1;f.review.update('actor',reset,now)
    for key in ('vision','ir','alignment','mission'):
        value,at=f.review.inputs[key];f.review.update(key,value,now)
    r=f.review.step(now,graph_ok=True)
    assert r['state']=='NO_FIRE_EVIDENCE' and r['historical_pulse_retained']
    assert r['pulse_index']==1
    # A new registered cycle resets the review instead of inheriting pulse1.
    next_ctx=dict(f.chain.context,cycle_id='23'*16,nav_goal_uuid='45'*16)
    reset['cycle_release_ack']=ack(next_ctx)
    now+=.1;f.review.update('actor',reset,now);r=f.review.step(now,graph_ok=True)
    assert r['state']=='WAITING_FOR_PULSE' and r['pulse_index']==0


@pytest.mark.parametrize('fault',['actor_stale','vision_age','vision_receive','ir_age','graph','new_epoch','reappears','fake_DONE'])
def test_canonical_review_never_keeps_READY_on_lost_evidence(fault,monkeypatch):
    f=Flow(monkeypatch);now=f.pulse()+.1
    for key,(value,_) in list(f.review.inputs.items()):f.review.update(key,value,now)
    if fault=='actor_stale':f.review.inputs['actor']=(f.review.inputs['actor'][0],now-.351)
    if fault=='vision_age':f.review.inputs['vision'][0]['source']['age_ms']=301
    if fault=='vision_receive':f.review.inputs['vision']=(f.review.inputs['vision'][0],now-.3)
    if fault=='ir_age':f.review.inputs['ir'][0]['ir_receive_age_ms']=501
    if fault=='new_epoch':f.review.inputs['actor'][0]['cycle_release_ack']['context']['connection_epoch']=2
    if fault=='reappears':f.review.inputs['vision'][0]['fire']=dict(detected_now=True,confirmed=True)
    if fault=='fake_DONE':f.review.inputs['actor'][0]['pulse_core']=None
    r=f.review.step(now,graph_ok=fault!='graph')
    assert r['state']!='NO_FIRE_EVIDENCE' and r['blockers']


def test_actual_review_node_opt_in_sources_are_installed_actor_with_GID_pins(monkeypatch):
    params={};callbacks={};sent=[];clock=N(now=1.);cls=ros.PostSprayReviewNode
    monkeypatch.setattr(ros.Node,'__init__',lambda *a:None)
    monkeypatch.setattr(cls,'declare_parameter',lambda self,k,v:params.setdefault(k,v))
    monkeypatch.setattr(cls,'get_parameter',lambda self,k:N(value=True if k=='use_operational_actor' else params[k]))
    monkeypatch.setattr(cls,'resolve_topic_name',lambda self,t:t)
    monkeypatch.setattr(cls,'create_subscription',lambda self,typ,t,fn,q:callbacks.setdefault(t,fn))
    monkeypatch.setattr(cls,'create_publisher',lambda self,typ,t,q:N(publish=lambda m:sent.append(json.loads(m.data))))
    monkeypatch.setattr(cls,'create_timer',lambda *a:None)
    monkeypatch.setattr(cls,'get_logger',lambda self:N(warning=lambda *a:None))
    monkeypatch.setattr(ros.time,'monotonic',lambda:clock.now)
    n=cls();pubs={topic:[endpoint(name,seed=i+1)] for i,(topic,name) in enumerate(n.sources.values())}
    n.get_publishers_info_by_topic=lambda t:pubs.get(t,[])
    assert PULSE_STATE_TOPIC in callbacks and '/fire_mission/pump_demo_state' not in callbacks
    assert n.core.core.max_pulses==1 and n._sources_current()
    pubs[PULSE_STATE_TOPIC][0].endpoint_gid=[111]+[0]*23
    n._tick();assert n.source_fault=='REVIEW_SOURCE_GID_CHANGED:'+PULSE_STATE_TOPIC
    assert sent[-1]['blockers'] and not sent[-1]['safety']['actuation_authorized']
