"""Exercise the changed constructors/ticks, not just protocol helpers."""
import copy
import json
from types import SimpleNamespace as N
import pytest
from rclpy.node import Node
from std_msgs.msg import String
from fire_robot_mission import operational_session as op
from fire_robot_mission.live_nav_stop_gateway import LiveNavStopGateway
from fire_robot_mission.nav2_cancel_guard import Nav2CancelGuard
from fire_robot_mission.live_alignment_takeover import LiveAlignmentTakeover
from fire_robot_mission.ir_approach_live import IrApproachLiveNode
from fire_robot_mission.mission_supervisor import FireMissionSupervisor
from fire_robot_mission.pulse_controller import PulseController
from test_session_adapter import node as observer
from test_session_core import data
from test_release_core import Chain
from test_cycle_endpoint import adapter as peer_adapter
from fire_robot_mission import cycle_endpoint as ep


@pytest.mark.parametrize('cls,method,role',[
    (LiveNavStopGateway,'_tick','gateway'),(Nav2CancelGuard,'_tick','guard'),
    (LiveAlignmentTakeover,'_tick','alignment'),(IrApproachLiveNode,'_tick','ir_live'),
    (FireMissionSupervisor,'_publish_state','mission'),(PulseController,'tick','actor')])
@pytest.mark.parametrize('protocol',[False,True])
def test_real_nodes_initialize_protocol_before_timer_and_emit_safe_wait(monkeypatch,cls,method,role,protocol):
    pubs=[];subscriptions=[];params={};states=[];timers=[]
    monkeypatch.setattr(Node,'__init__',lambda *a:None)
    monkeypatch.setattr(Node,'declare_parameter',lambda self,k,v:params.setdefault(k,v))
    monkeypatch.setattr(Node,'get_parameter',lambda self,k:N(value=protocol if k=='enable_cycle_protocol' else params[k]))
    monkeypatch.setattr(Node,'resolve_topic_name',lambda self,t:t)
    def publisher(self,typ,t,q):
        pubs.append((typ,t));return N(publish=lambda msg:states.append(json.loads(msg.data)))
    monkeypatch.setattr(Node,'create_publisher',publisher)
    monkeypatch.setattr(Node,'create_subscription',lambda self,typ,t,fn,q:subscriptions.append(t))
    monkeypatch.setattr(Node,'create_client',lambda *a:N(service_is_ready=lambda:False))
    def timer(self,period,cb):
        assert hasattr(self,'_cycle');timers.append(cb)
    monkeypatch.setattr(Node,'create_timer',timer)
    monkeypatch.setattr(Node,'get_logger',lambda self:N(info=lambda *a:None,warning=lambda *a:None,error=lambda *a:None))
    monkeypatch.setattr(Node,'get_publishers_info_by_topic',lambda *a:[])
    monkeypatch.setattr(Node,'get_subscriptions_info_by_topic',lambda *a:[])
    monkeypatch.setattr(Node,'count_publishers',lambda *a:0)
    n=cls();getattr(n,method)()
    assert n._cycle.enabled==protocol and n._cycle.role==role
    assert all(typ is String for typ,t in pubs) and not any(t in ('/cmd_vel_raw','/pump_cmd') for typ,t in pubs)
    assert bool(ep.CONTROL_TOPIC in subscriptions)==protocol
    if protocol:
        assert states[-1]['cycle_release_ack']['blockers']==['WAIT_ACTIVE_CYCLE_REGISTRATION']
        assert not states[-1]['cycle_release_ack']['reset_completed']
    else:assert 'cycle_release_ack' not in states[-1]


def test_actual_observer_BOOT_bound_ARM_and_handback_proposal_have_no_control_publishers(monkeypatch):
    a=observer(monkeypatch);d=data();f=Chain()
    d['vision']['fire']=dict(detected_now=False,confirmed=False)
    for i in range(36):
        f.tick();d['identity']=f.master.inputs['identity'][0];d['review']=f.master.inputs['review'][0]
        a.feed(d,f.now);a.subscriptions[op.PULSE_STATE_TOPIC](String(data=json.dumps(f.master.inputs['actor'][0])))
        r=a.node.tick('A' if i==35 else None)
    assert r['armed'] and r['handback']['control'] is None
    f.tick();d['vision']['fire']=dict(detected_now=True,confirmed=True);d['identity']=f.master.inputs['identity'][0]
    a.feed(d,f.now);r=a.node.tick('F')
    assert r['cycle'] and r['handback']['control']['phase']=='ACTIVE'
    assert r['handback']['control']['context']['nav_goal_uuid']=='a'*32
    assert a.created==[(String,op.STATE_TOPIC)] and not r['live_intents_published']
    assert 'BOOT_EPOCH_GATE_PENDING' not in r['integration_blockers']
    # Identity loss faults intent only, leaving persistent UI alive for Q.
    f.tick();d['identity']=dict(f.master.inputs['identity'][0],connection_epoch=2)
    a.feed(d,f.now);r=a.node.tick()
    assert r['fault']=='BOOT_OR_SERIAL_EPOCH_CHANGED' and not r['closed']
    assert r['handback']['control'] is None


@pytest.mark.parametrize('bad',['epoch','stale','wrong_elf'])
def test_local_actor_stops_and_cannot_register_again_when_BOOT_proof_changes(monkeypatch,bad):
    from fire_robot_mission.pulse_session import PulseSession
    a=peer_adapter('actor',core=PulseSession(enabled=True,allow_provisional_ir=True),attr='session')
    c=Chain();c.tick();a.clock.now=c.now;monkeypatch.setattr(ep.time,'monotonic',lambda:a.clock.now)
    a.endpoint.on_control(String(data=json.dumps(dict(protocol='CYCLE_RELEASE_V1',phase='ACTIVE',context=c.context))))
    for key,(value,at) in c.master.inputs.items():a.endpoint.participant.update(key,value,at)
    a.endpoint.before_step(c.now);assert a.endpoint.stage_allowed(c.now)
    value,at=a.endpoint.participant.inputs['identity']
    if bad=='epoch':value['connection_epoch']=2
    if bad=='stale':value['state_age_ms']=351
    if bad=='wrong_elf':value['elf_sha256']='0'*64
    a.node.session.update('operator',True,c.now);a.node.transport.pending_p=c.now
    a.endpoint.before_step(c.now)
    assert not a.endpoint.stage_allowed(c.now) and a.node.transport.pending_p is None
    assert a.node.session.get('operator',c.now) is False


def test_current_epoch_mismatch_latches_locally_even_if_previous_payload_returns(monkeypatch):
    a=peer_adapter();c=Chain();c.tick();a.clock.now=c.now
    monkeypatch.setattr(ep.time,'monotonic',lambda:a.clock.now)
    a.endpoint.on_control(String(data=json.dumps(dict(protocol='CYCLE_RELEASE_V1',phase='ACTIVE',context=c.context))))
    for key,(value,at) in c.master.inputs.items():a.endpoint.participant.update(key,value,at)
    original=copy.deepcopy(a.endpoint.participant.inputs['identity'][0])
    a.endpoint.participant.inputs['identity'][0]['connection_epoch']=2
    a.endpoint.before_step(c.now)
    assert a.endpoint.participant.error=='CYCLE_BOOT_EPOCH_CHANGED'
    a.endpoint.participant.update('identity',original,c.now);a.node.core.update_arm(True,c.now)
    a.endpoint.before_step(c.now)
    assert not a.node.core.arm and not a.endpoint.stage_allowed(c.now)


def test_COMMITTED_gateway_reset_allows_new_normal_goal_without_rebinding_old_fire_cycle(monkeypatch):
    from fire_robot_mission.live_nav_stop_gateway import LiveNavStopCore
    a=peer_adapter('gateway',core=LiveNavStopCore(enabled=True));c=Chain();c.finish()
    a.endpoint.participant=c.peers['gateway'];a.clock.now=c.now
    a.node.core.goal_uuid='34'*16;a.node.core.update_arm(True,c.now)
    monkeypatch.setattr(ep.time,'monotonic',lambda:a.clock.now)
    a.endpoint.before_step(c.now)
    r=a.endpoint.decorate({'cycle_goal_uuid':'34'*16},c.now)
    assert a.node.core.arm and a.endpoint.participant.error is None
    assert r['cycle_release_ack']['phase']=='COMMITTED'
    assert r['cycle_release_ack']['context']['nav_goal_uuid']=='a'*32


def test_guard_COMMITTED_cannot_cancel_another_goal_before_new_ACTIVE_registration(monkeypatch):
    from fire_robot_mission.nav2_cancel_guard import Nav2CancelGuardCore
    a=peer_adapter('guard',core=Nav2CancelGuardCore(enabled=True));c=Chain();c.finish()
    a.endpoint.participant=c.peers['guard'];a.clock.now=c.now
    a.node.core.update_arm(True,c.now);monkeypatch.setattr(ep.time,'monotonic',lambda:a.clock.now)
    a.endpoint.before_step(c.now)
    assert not a.node.core.arm
