"""Actual opt-in operator/intent transport with peer protocol and fake I/O."""
import copy
import json
from types import SimpleNamespace as N
import pytest
from std_msgs.msg import String,Bool
from fire_robot_mission import operational_session as ros
from fire_robot_mission.operational_intents import ARM_TOPICS
from fire_robot_mission.cycle_release import ROLES,CONTROL_TOPIC,CycleReleaseParticipant
from test_installed_pulse import endpoint
from test_session_core import data
from test_release_core import Chain


def operator_fixture(monkeypatch, actor_source=None, review_source=None, fact_source=None):
    cls=ros.OperationalSessionNode;params={};pubs={};sent={};callbacks={};created=[];clock=N(now=1.)
    monkeypatch.setattr(ros.Node,'__init__',lambda *a:None)
    monkeypatch.setattr(cls,'declare_parameter',lambda self,k,v:params.setdefault(k,v))
    monkeypatch.setattr(cls,'get_parameter',lambda self,k:N(value=True if k=='enable_live_intents' else params[k]))
    monkeypatch.setattr(cls,'resolve_topic_name',lambda self,t:t)
    def publisher(self,typ,t,q):
        created.append((typ,t));sent[t]=[]
        pubs[t]=[endpoint(ros.OPERATOR_NODE,'std_msgs/msg/'+typ.__name__,q.reliability,len(created)+40)]
        return N(publish=lambda msg:sent[t].append(json.loads(msg.data) if typ is String else msg.data))
    monkeypatch.setattr(cls,'create_publisher',publisher)
    monkeypatch.setattr(cls,'create_subscription',lambda self,typ,t,fn,q:callbacks.setdefault(t,fn))
    monkeypatch.setattr(cls,'create_client',lambda *a:N(service_is_ready=lambda:False))
    monkeypatch.setattr(cls,'get_logger',lambda self:N(info=lambda *a:None,warning=lambda *a:None,error=lambda *a:None))
    monkeypatch.setattr(cls,'get_clock',lambda self:N(now=lambda:N(nanoseconds=int(clock.now*1e9))))
    n=cls()
    for i,(t,name,typ,rel) in enumerate(ros.EXPECTED):pubs[t]=[endpoint(name,typ,rel,i+1)]
    n.get_publishers_info_by_topic=lambda t:pubs.get(t,[])
    monkeypatch.setattr(ros.time,'monotonic',lambda:clock.now)
    f=Chain();peers={r:CycleReleaseParticipant(r) for r in ROLES};d=data();resets={r:0 for r in ROLES}
    d['vision']['fire']=dict(detected_now=False,confirmed=False)
    d['alignment'].update(state='WAITING_FOR_ADMISSION',goal_uuid=None,publisher_owned=False)
    def tick(key=None,*,clear=False,safety=True,peer_ack=True):
        f.tick();now=f.now;clock.now=now
        # The live operator has a decoded annotated-camera pre-arm gate. This
        # adapter test supplies that external proof without opening a device.
        n.frame_proof.received=now;n.frame_proof.source_age_ms=0.
        n.frame_proof.valid_frames+=1;n.frame_proof.reason='FRESH_DECODED_FRAME'
        d['identity']=copy.deepcopy(f.master.inputs['identity'][0]);d['review']=copy.deepcopy(f.master.inputs['review'][0])
        d['actor']=(actor_source(now,d) if actor_source else
                    dict(state='WAIT_P',start_home_count=None,request_used=False,pulse_core=None))
        if review_source:d['review']=review_source(now,d)
        if clear:
            d['ir']=copy.deepcopy(f.master.inputs['ir'][0]);d['vision']['fire']=dict(detected_now=False,confirmed=False)
        if fact_source:fact_source(now,d)
        n.intent_executor.safety_ok_at=now if safety else now-11.
        control=sent[CONTROL_TOPIC][-1] if sent[CONTROL_TOPIC] else None
        for role in ROLES:
            value=copy.deepcopy(d[role]);value['cycle_protocol_enabled']=True
            if control and peer_ack and not (role=='actor' and actor_source):
                p=peers[role];assert p.receive(control,now),p.error
                for name,v in d.items():p.update(name,v,now)
                def reset(ctx,r=role):resets[r]+=1
                value['cycle_release_ack']=p.step(now,graph_ok=True,raw_count=0,pump_count=0,reset=reset)
            if role=='actor':callbacks[ros.PULSE_STATE_TOPIC](String(data=json.dumps(value)))
            else:
                topic=ros.JSON_INPUTS[role][0];callbacks[topic](String(data=json.dumps(value)))
        for k,(t,_) in ros.JSON_INPUTS.items():
            if k not in ROLES:callbacks[t](String(data=json.dumps(d[k])))
        n.core.update('status',d['status'],now);n.core.update('cmd',[0.]*6,now);n.core.update('odom',[0.]*6,now)
        return n.tick(key)
    return N(node=n,d=d,tick=tick,sent=sent,created=created,clock=clock,resets=resets,pubs=pubs,facts=f)


def test_actual_live_operator_A_F_T_X_release_and_new_goal_stays_open(monkeypatch):
    a=operator_fixture(monkeypatch)
    for _ in range(35):prearm=a.tick()
    assert prearm['live_intents_published'] and not prearm['live_intents_active']
    assert prearm['phase']=='WAIT_ARM' and not prearm['closed']
    assert a.tick('A')['armed'] and a.sent[ARM_TOPICS[0]][-1]
    a.d['vision']['fire']=dict(detected_now=True,confirmed=True)
    r=a.tick('F');assert r['cycle'] and a.sent[CONTROL_TOPIC][-1]['phase']=='ACTIVE'
    a.d['status']=[('a'*32,5)]
    r=a.tick();assert r['phase']=='WAIT_T',r
    r=a.tick('T');assert r['phase']=='ALIGNING' and a.sent[ARM_TOPICS[3]][-1]
    r=a.tick('X');assert r['phase']=='CANCEL_WAIT_CLEAR'
    assert a.sent[ARM_TOPICS[1]][-1] and not any(a.sent[t][-1] for t in ARM_TOPICS[2:])
    for _ in range(70):
        r=a.tick(clear=True)
        if r['handback']['consumed']:break
    assert r['handback']['consumed'] and r['reason']=='NAV2_READY_NEW_GOAL',r
    assert not r['closed'] and set(a.resets.values())=={1}
    a.d['status']=[('12'*16,2)];r=a.tick(clear=True)
    assert r['phase']=='NAVIGATION' and a.sent[ARM_TOPICS[0]][-1]
    assert not any(t in ('/cmd_vel_raw','/pump_cmd') for typ,t in a.created)
    a.node.close_session();assert all(a.sent[t][-1] is False for t in ARM_TOPICS)


def test_no_key_or_early_A_F_T_I_P_ever_closes_prearm_session(monkeypatch):
    a=operator_fixture(monkeypatch)
    for key in (None,'A','F','T','I','P'):
        r=a.tick(key)
        assert r['phase']=='WAIT_ARM' and not r['closed'] and r['fault'] is None
        assert not r['armed'] and not r['live_intents_active']
        assert not any(a.sent[t][-1] for t in ARM_TOPICS)


def test_actual_live_operator_missing_protocol_rejects_A_and_safety_loss_keeps_UI_open(monkeypatch):
    a=operator_fixture(monkeypatch)
    for _ in range(35):a.tick()
    assert a.tick('A')['armed']
    before=len(a.sent[CONTROL_TOPIC]);r=a.tick(safety=False)
    assert r['fault'] is None and r['phase']=='WAIT_INPUT' and r['input_hold']
    assert not any(r['intents'].values())
    assert not r['closed'] and not any(a.sent[t][-1] for t in ARM_TOPICS)
    assert len(a.sent[CONTROL_TOPIC])==before
    blocked=a.tick('A',safety=False)
    assert not blocked['operator_feedback']['accepted'] and blocked['input_hold']
    for _ in range(16):restored=a.tick()
    assert restored['armed'] and not restored['input_hold'] and restored['fault'] is None


def test_default_observer_after_executor_port_still_has_only_state_publisher(monkeypatch):
    from test_session_adapter import node
    a=node(monkeypatch)
    assert not a.node.intent_executor.enabled
    assert a.created==[(String,ros.STATE_TOPIC)]
    a.node.tick()
    assert not a.sent[-1]['execution']['enabled'] and not a.sent[-1]['live_intents_published']


def test_repeated_A_while_live_proof_missing_keeps_recoverable_hold(monkeypatch):
    a=operator_fixture(monkeypatch)
    for _ in range(35):a.tick()
    assert a.tick('A')['armed']
    first=a.tick(safety=False)['fault']
    r=a.tick('A',safety=False)
    assert r['phase']=='WAIT_INPUT' and r['fault']==first is None and r['input_hold']
    assert not any(r['intents'].values())
    assert not r['closed'] and not r['operator_feedback']['accepted']
