import copy
import json
from types import SimpleNamespace as N
import pytest
from std_msgs.msg import String
from fire_robot_mission.operational_identity import IdentityContinuity,identity_current
from fire_robot_mission.operational_session_core import OperationalSessionCore
from fire_robot_mission.cycle_release import CONTROL_TOPIC
from fire_robot_mission import cycle_endpoint as ep
from test_cycle_endpoint import adapter
from test_release_core import Chain
from test_session_core import data,feed
from test_operator_live_delta import operator_fixture
from fire_robot_mission.operational_intents import ARM_TOPICS


def evidence():
    f=Chain();f.tick();return f,copy.deepcopy(f.master.inputs['identity'][0])


@pytest.mark.parametrize('fault',['epoch','instance','ELF','clock','bridge_clock','bridge_frame'])
def test_identity_changes_remain_latched_hard_faults(fault):
    f,p=evidence();lease=IdentityContinuity();assert lease.observe(p,f.now,f.now,armed=True,stopped=True)
    q=copy.deepcopy(p);now=f.now+.1
    if fault=='epoch':q['connection_epoch']+=1
    if fault=='instance':q['bridge_instance_id']='12'*16
    if fault=='ELF':q['elf_sha256']='0'*64
    if fault=='clock':now=f.now-.1
    if fault=='bridge_clock':q.update(valid=False,reason='MCU_CLOCK_REGRESSION')
    if fault=='bridge_frame':q.update(valid=False,reason='IDENTITY_FRAME_INVALID')
    assert not lease.observe(q,now,now,armed=True,stopped=True) and lease.fault
    assert not lease.observe(p,f.now+.2,f.now+.2,armed=True,stopped=True)


@pytest.mark.parametrize('stale',['receipt','state_age','bridge_wait'])
def test_stale_identity_does_not_extend_500ms_or_reset_epoch_and_recovers_only_fresh_stop_hold(stale):
    f,p=evidence();lease=IdentityContinuity();assert lease.observe(p,f.now,f.now,armed=True,stopped=True)
    at=f.now+.6;q=copy.deepcopy(p);received=at
    if stale=='receipt':received=f.now
    if stale=='state_age':q['state_age_ms']=501
    if stale=='bridge_wait':q.update(valid=False,telemetry_healthy=False,reason='WAIT_FRESH_STATE')
    assert not lease.observe(q,received,at,armed=True,stopped=True)
    assert lease.held and lease.fault is None and not identity_current(q,received,at)
    bound=lease.bound
    for i in range(10):
        now=at+.1*(i+1);assert not lease.observe(p,now,now,armed=True,stopped=False)
    for i in range(6):
        now+=.1;ready=lease.observe(p,now,now,armed=True,stopped=True)
        if i<5:assert not ready
    # Roundoff-safe final point beyond the hold boundary.
    now+=.1;assert lease.observe(p,now,now,armed=True,stopped=True)
    assert not lease.held and lease.fault is None and lease.bound==bound


def test_recovery_gap_resets_hold_not_source_freshness():
    f,p=evidence();lease=IdentityContinuity();lease.observe(p,f.now,f.now,armed=True,stopped=True)
    now=f.now+.4;lease.observe(None,None,now,armed=True,stopped=True)
    for dt in (.1,.2,.6,.7,.8,.9):assert not lease.observe(p,now+dt,now+dt,armed=True,stopped=True)
    assert lease.observe(p,now+1.2,now+1.2,armed=True,stopped=True)


def test_session_identity_HOLD_keeps_original_I_budget_and_discards_repeated_I():
    f,p=evidence();c=OperationalSessionCore(lambda:'cd'*16,require_boot_identity=True);d=data()
    d['identity']=p;d['vision']['fire']=dict(detected_now=False,confirmed=False)
    for i in range(35):
        f.tick();d['identity']=f.master.inputs['identity'][0];feed(c,d,f.now)
    assert feed(c,d,f.now,['A'])['armed']
    d['vision']['fire']=dict(detected_now=True,confirmed=True);f.tick();d['identity']=f.master.inputs['identity'][0]
    feed(c,d,f.now,['F']);d['status']=[('a'*32,5)]
    f.tick();d['identity']=f.master.inputs['identity'][0];feed(c,d,f.now,['I'])
    assert c.cycles.budget.requested_at is not None
    requested=c.cycles.budget.requested_at;cycle=c.cycles.current
    f.tick();d['identity']=dict(f.master.inputs['identity'][0],state_age_ms=501)
    r=feed(c,d,f.now,['I'])
    assert r['phase']=='WAIT_IDENTITY' and not r['fault'] and r['armed'] and not any(r['intents'].values())
    assert not r['operator_feedback']['accepted'] and not r['closed']
    # If a gap consumes all original15s the budget remains exhausted; fresh
    # proof does not create a second movement request.
    for _ in range(160):
        f.tick();d['identity']=dict(f.master.inputs['identity'][0],state_age_ms=501);feed(c,d,f.now)
    for _ in range(8):
        f.tick();d['identity']=f.master.inputs['identity'][0];r=feed(c,d,f.now)
    assert not r['identity_hold'] and c.cycles.current==cycle
    assert c.cycles.budget.requested_at==requested and r['budget']['exhausted']
    assert not r['intents']['ir']


def test_actual_motion_endpoint_holds_existing_owner_at_zero_without_reset_then_waits_for_fresh_arm(monkeypatch):
    from fire_robot_mission.live_alignment_takeover import LiveAlignmentCore
    a=adapter('alignment',core=LiveAlignmentCore(enabled=True));f,p=evidence();a.clock.now=f.now;now=f.now
    monkeypatch.setattr(ep.time,'monotonic',lambda:a.clock.now)
    a.endpoint.on_control(String(data=json.dumps(dict(protocol='CYCLE_RELEASE_V1',phase='ACTIVE',context=f.context))))
    sent=[];states=[];a.node._raw_publisher=N(publish=lambda msg:sent.append((msg.linear.x,msg.angular.z)))
    a.node._state_publisher=N(publish=lambda msg:states.append(json.loads(msg.data)))
    a.node.core.phase='TURNING';a.node.core.update_arm(True,now)
    for k,(v,at) in f.master.inputs.items():a.endpoint.participant.update(k,v,at)
    a.endpoint.before_step(now);assert not a.endpoint.pause_tick(now)
    old=a.node.core;a.endpoint.last_payload=dict(schema_version=1,enabled=True,state='TURNING',goal_uuid='a'*32,
        publisher_owned=True,command=dict(linear_x_m_s=0.,angular_z_rad_s=.3),blockers=[])
    for i in range(8):
        f.tick();a.clock.now=f.now;now=f.now
        a.endpoint.on_control(String(data=json.dumps(dict(protocol='CYCLE_RELEASE_V1',phase='ACTIVE',context=f.context))))
        for k,(v,at) in f.master.inputs.items():a.endpoint.participant.update(k,v,at)
        if i==0:a.endpoint.participant.inputs['identity'][0]['state_age_ms']=501
        a.node.core.update_arm(False,now)
        a.endpoint.before_step(now)
        assert a.endpoint.pause_tick(now)
        assert a.node.core is old and old.phase=='TURNING' and a.node._raw_publisher is not None
    assert sent and all(v==(0.,0.) for v in sent) and states[-1]['r5_identity_hold']
    old.update_arm(True,now);a.endpoint.before_step(now)
    assert not a.endpoint.pause_tick(now) and old.phase=='TURNING' and a.endpoint.participant.error is None


def test_operator_live_stale_identity_waits_with_false_intents_and_recovers_same_ARM(monkeypatch):
    a=operator_fixture(monkeypatch)
    for _ in range(35):a.tick()
    assert a.tick('A')['armed']
    # Callback gap, exact endpoint still present. Feed all other inputs now.
    a.clock.now+=.6;now=a.clock.now
    for key,(value,_) in list(a.node.core.inputs.items()):
        if key!='identity':a.node.core.update(key,value,now)
    a.node.intent_executor.safety_ok_at=now
    r=a.node.tick()
    assert r['identity_hold'] and not r['fault'] and r['armed'] and not r['closed']
    assert not any(a.sent[t][-1] for t in ARM_TOPICS)
    # Advance the fake source clock too; no backwards callback is permitted.
    a.facts.now=now
    r=None
    for _ in range(20):
        result=a.tick()
        r=result
        if r['fault']:break
    assert r['fault'] is None and not r['identity_hold'] and r['armed']


@pytest.mark.parametrize('fault',['LOCKED','unexpected_ON'])
def test_identity_HOLD_does_not_hide_fresh_hard_actuator_fault(fault):
    c=OperationalSessionCore(require_boot_identity=True);c.armed=True;d=data();f,p=evidence()
    d['identity']=p
    for k,v in d.items():c.update(k,v,f.now)
    c.step(f.now)
    f.tick();d['identity']=dict(f.master.inputs['identity'][0],state_age_ms=501)
    if fault=='LOCKED':d['actuator']['servo_phase']='LOCKED'
    else:d['actuator']['pump_pwm_commanded_on']=True
    r=feed(c,d,f.now)
    assert r['fault']==('ACTUATOR_LOCKED' if fault=='LOCKED' else 'UNEXPECTED_PUMP_ON')
    assert not r['armed'] and not r['closed'] and not any(r['intents'].values())
