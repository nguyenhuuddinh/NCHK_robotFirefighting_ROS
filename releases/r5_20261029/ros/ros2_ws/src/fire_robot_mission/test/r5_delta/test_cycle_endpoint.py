"""R5.3 adapter delta: actual core reconstruction; no DDS or device I/O."""
import copy
import json
from types import SimpleNamespace as N
import pytest
from std_msgs.msg import String
from fire_robot_mission import cycle_endpoint as ep
from fire_robot_mission.cycle_release import ReleaseContext,CycleReleaseCoordinator
from fire_robot_mission.operational_cycle import CycleRegistry
from fire_robot_mission.live_nav_stop_gateway import LiveNavStopCore
from fire_robot_mission.nav2_cancel_guard import Nav2CancelGuardCore
from fire_robot_mission.live_alignment_takeover import LiveAlignmentCore
from fire_robot_mission.ir_approach_live_core import IrApproachLiveCore
from fire_robot_mission.mission_supervisor import MissionSupervisorCore
from fire_robot_mission.pulse_session import PulseSession
from test_installed_pulse import endpoint
from test_release_core import Chain,OLD,CYCLE,TOKEN,BRIDGE

CORES=[('gateway',LiveNavStopCore,'core',dict(enabled=True,max_nav_linear_m_s=.09)),
       ('guard',Nav2CancelGuardCore,'core',dict(enabled=True,response_timeout_s=1.9)),
       ('alignment',LiveAlignmentCore,'core',dict(enabled=True,max_angular_speed_rad_s=.5)),
       ('ir_live',IrApproachLiveCore,'core',dict(enabled=True,integrated_handoff=True,continuous_approach=True)),
       ('mission',MissionSupervisorCore,'_core',dict(fire_acquire_hold_s=.4)),
       ('actor',PulseSession,'session',dict(enabled=True,allow_provisional_ir=True,allow_close_stationary=True))]


def adapter(role='ir_live',enabled=True,core=None,attr='core'):
    callbacks={};destroyed=[];sent=[];clock=N(now=1.);params={}
    node=N(declare_parameter=lambda k,v:params.setdefault(k,v),
           get_parameter=lambda k:N(value=enabled),resolve_topic_name=lambda t:t,
           create_subscription=lambda typ,t,fn,q:callbacks.setdefault(t,fn),
           get_clock=lambda:N(now=lambda:N(nanoseconds=int(clock.now*1e9))),
           destroy_publisher=lambda pub:destroyed.append(pub),
           _raw_publisher=None,pub=None,cycle_pending_cancel=False)
    setattr(node,attr,core or IrApproachLiveCore(enabled=True))
    if role=='actor':
        from fire_robot_mission.pulse_transport import Transport
        node.transport=Transport(node.session);node.stop_requested=False
    a=ep.CycleEndpoint(node,role,attr)
    pubs={t:[endpoint(name,typ,rel,i+1)] for i,(t,(name,typ,rel)) in enumerate(a.expected.items())}
    node.get_publishers_info_by_topic=lambda t:pubs.get(t,[])
    return N(endpoint=a,node=node,pubs=pubs,callbacks=callbacks,destroyed=destroyed,sent=sent,clock=clock)


@pytest.mark.parametrize('role,cls,attr,kwargs',CORES)
def test_commit_reconstructs_real_core_with_original_configuration_and_tombstones(role,cls,attr,kwargs):
    core=cls(**kwargs);a=adapter(role,core=core,attr=attr);chain=Chain()
    if hasattr(core,'used_goal_uuids'):core.used_goal_uuids.add('be'*16)
    if hasattr(core,'goal_uuid'):core.goal_uuid=OLD
    if role=='actor':core.used=True;core.phase='DONE'
    if role=='mission':core.prespray_enabled=True
    a.endpoint._reset(ReleaseContext.parse(chain.context))
    new=getattr(a.node,attr)
    assert new is not core and new.r5_initialization==core.r5_initialization
    if hasattr(new,'used_goal_uuids'):assert {OLD,'be'*16}<=new.used_goal_uuids
    if role=='actor':
        assert not new.used and new.phase=='WAIT_P' and OLD in new.pump.used_goal_uuids
        assert a.node.transport.session is new and a.node.transport.pending_p is None
    if role=='mission':assert new.prespray_enabled
    if hasattr(new,'goal_uuid'):assert new.goal_uuid is None


@pytest.mark.parametrize('owner',['_raw_publisher','pub'])
def test_actual_local_publisher_must_be_destroyed_before_core_replacement(owner):
    a=adapter();old=a.node.core;setattr(a.node,owner,object())
    with pytest.raises(RuntimeError,match='LOCAL_PUBLISHER_STILL_OWNED'):
        a.endpoint._reset(ReleaseContext.parse(Chain().context))
    assert a.node.core is old


def test_default_protocol_creates_no_subscriptions_or_control_output():
    a=adapter(enabled=False)
    assert not a.callbacks and a.endpoint.stage_allowed(1.)
    a.node.core.update_arm(True,1.);a.endpoint.before_step(1.)
    assert a.node.core.arm and a.endpoint.decorate({'state':'legacy'},1.)=={'state':'legacy'}


@pytest.mark.parametrize('failure',['missing_active','expired','changed_gid','unknown_changed_gid'])
def test_protocol_requires_live_registered_context_and_pinned_control_source(failure,monkeypatch):
    a=adapter();c=Chain();monkeypatch.setattr(ep.time,'monotonic',lambda:a.clock.now)
    if failure!='missing_active':
        a.endpoint.on_control(String(data=json.dumps(dict(protocol='CYCLE_RELEASE_V1',phase='ACTIVE',context=c.context))))
    a.node.core.update_arm(True,1.)
    if failure=='expired':a.clock.now=1.36
    if failure in ('changed_gid','unknown_changed_gid'):
        i=a.pubs[ep.CONTROL_TOPIC][0];i.endpoint_gid=[100]+[0]*23
        if failure=='unknown_changed_gid':
            i.node_name='_NODE_NAME_UNKNOWN_';i.node_namespace='_NODE_NAMESPACE_UNKNOWN_'
    a.endpoint.before_step(a.clock.now)
    assert not a.node.core.arm and not a.endpoint.stage_allowed(a.clock.now)
    if 'gid' in failure:assert a.endpoint.participant.error.startswith('CYCLE_SOURCE_GID_CHANGED:')


def test_endpoint_COMMIT_ACK_follows_real_core_reset_and_pending_cancel_blocks_it(monkeypatch):
    a=adapter();chain=Chain();monkeypatch.setattr(ep.time,'monotonic',lambda:a.clock.now)
    for k in range(25):
        chain.tick();a.clock.now=chain.now
        if a.endpoint.participant.context is None:
            a.endpoint.on_control(String(data=json.dumps(dict(protocol='CYCLE_RELEASE_V1',phase='ACTIVE',context=chain.context))))
        control=chain.master.control();control['phase']='PREPARE' if k<15 else 'COMMIT'
        a.endpoint.on_control(String(data=json.dumps(control)))
        for key,(value,at) in chain.master.inputs.items():a.endpoint.participant.update(key,value,at)
        a.node.cycle_pending_cancel=15<=k<20
        a.endpoint.before_step(chain.now);old=a.node.core
        report=a.endpoint.decorate({'goal_uuid':OLD},chain.now)
        if 15<=k<20:
            assert 'WAIT_CANCEL_RESPONSE_SETTLED' in report['cycle_release_ack']['blockers']
            assert a.node.core is old
    # 0.5 s hold must restart after pending response cleared.
    for _ in range(10):
        chain.tick();a.clock.now=chain.now;a.endpoint.on_control(String(data=json.dumps(control)))
        for key,(value,at) in chain.master.inputs.items():a.endpoint.participant.update(key,value,at)
        a.endpoint.before_step(chain.now);report=a.endpoint.decorate({},chain.now)
    assert report['cycle_release_ack']['phase']=='COMMITTED'
    assert OLD in a.node.core.used_goal_uuids and not a.endpoint.stage_allowed(chain.now)


@pytest.mark.parametrize('mutation',[{'after_pulse':False,'home_before':None},{'home_before':1},{'after_pulse':1}])
def test_commit_cannot_remove_or_change_postpulse_home_policy(mutation):
    c=Chain(after_pulse=True);c.tick();p=c.peers['actor'];message=c.master.control()
    message.update(mutation);message['phase']='COMMIT'
    assert not p.receive(message,c.now)
    assert p.error in ('RELEASE_POLICY_CHANGED','RELEASE_POLICY_INVALID')


@pytest.mark.parametrize('bad', ['time','token','policy','home'])
def test_invalid_begin_is_transactional(bad):
    c=Chain();m=CycleReleaseCoordinator(lambda:TOKEN if bad!='token' else 'bad')
    kwargs=dict(after_pulse=False,home_before=None);now=1.
    if bad=='time':now=float('nan')
    if bad=='policy':kwargs['after_pulse']=1
    if bad=='home':kwargs.update(after_pulse=True,home_before=None)
    with pytest.raises(ValueError):m.begin(c.context,now,**kwargs)
    assert m.context is None and m.phase=='IDLE'


def test_postpulse_historical_ACK_survives_real_actor_reset_but_new_OFF_home_still_required():
    c=Chain(after_pulse=True)
    for _ in range(40):
        c.tick()
        if c.master.phase=='COMMIT':break
    assert c.master.pulse_proven
    # Real actor reset reports WAIT_P, not a fabricated perpetual DONE.
    for _ in range(30):
        report=c.tick(mutate=lambda d:d['actor'].update(state='WAIT_P',request_used=False,
             pump_on_feedback_seen=False,servo_sweep_feedback_seen=False))
        if report['ready']:break
    assert report['ready']
    assert not c.tick(fault='ACT_ON')['ready']
    assert not c.tick(fault='no_home')['ready']


@pytest.mark.parametrize('bad',['json','stale','context','roles','wrong_goal','replay','valid'])
def test_registry_retires_only_exact_typed_current_all_peer_receipt(bad):
    from dataclasses import replace
    c=Chain();c.finish();receipt=c.master.receipt
    reg=CycleRegistry(lambda:CYCLE);reg.begin([(OLD,2)],1.,1.,armed=True,fire_confirmed=True)
    context=copy.deepcopy(c.context);now=c.now
    if bad=='json':receipt=receipt.__dict__
    if bad=='stale':now+=.351
    if bad=='context':context['connection_epoch']=2
    if bad=='roles':receipt=replace(receipt,roles=('gateway',))
    if bad=='wrong_goal':receipt=replace(receipt,context=replace(receipt.context,nav_goal_uuid=None))
    if bad in ('valid','replay'):
        reg.retire(receipt,now,expected_context=context)
        assert reg.current is None and reg.budget is None and CYCLE in reg.retired
        if bad=='valid':return
    with pytest.raises(ValueError,match='CURRENT_EXACT_RELEASE_RECEIPT_REQUIRED'):
        reg.retire(receipt,now,expected_context=context)
    if bad!='replay':assert reg.current is not None
