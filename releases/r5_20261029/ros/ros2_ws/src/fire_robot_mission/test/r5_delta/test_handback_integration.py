"""R5.3 session + peer protocol + BOOT boundary; no ROS context or hardware."""
import copy
import importlib.util
from pathlib import Path
import pytest
from fire_robot_mission.operational_handback import OperationalHandback
from fire_robot_mission.operational_session_core import OperationalSessionCore
from fire_robot_mission.cycle_release import CycleReleaseParticipant,ROLES
from test_release_core import Chain,CYCLE,OLD
from test_session_core import data,feed
from test_installed_pulse import ready


class Harness:
    def __init__(self,*,no_goal=False,after=False,explicit=True):
        self.facts=Chain(no_goal=no_goal,after_pulse=after);self.session=OperationalSessionCore(lambda:CYCLE)
        c=self.session;c.armed=True;c.cycles.begin([] if no_goal else [(OLD,2)],1.,1.,armed=True,fire_confirmed=True)
        c.cycles.budget.request(1.,admission_ready=True)
        # This fixture constructs an already-armed session directly. Supply
        # the BOOT identity lease that the real ARM path establishes.
        ctx=self.facts.context
        c.identity_lease.bound=(ctx['bridge_instance_id'],ctx['connection_epoch'],ctx['elf_sha256'])
        c.p_used=after;c.turn_permission=True;c.cancel_requested=explicit and not after
        self.handback=OperationalHandback(lambda:'ef'*16)
        self.peers={r:CycleReleaseParticipant(r) for r in ROLES};self.resets={r:0 for r in ROLES}
        self.proposal=None
    def tick(self,*,ir_clear=True,yolo_clear=True,skip=None,fault=None):
        self.facts.tick(fault=fault);now=self.facts.now
        values={k:copy.deepcopy(v) for k,(v,at) in self.facts.master.inputs.items()}
        values['actor']['start_home_count']=0 if self.facts.after else None
        values['ir_live'].update(total_progress_m=0.,step_count=1)
        values['guard']=dict(ready()['guard'],goal_uuid=None if self.facts.context['nav_goal_uuid'] is None else OLD)
        if not ir_clear:values['ir'].update(ir_band='APPROACH',filtered_counts=dict(left=25000,center=25000,right=12000))
        if not yolo_clear:values['vision']['fire']=dict(detected_now=True,confirmed=True)
        for key,value in values.items():
            if key in self.session.INPUTS:self.session.update(key,value,now)
        self.session.update('graph',dict(valid=True,raw_count=0,raw_owner=None,nav_status_event_stream=True),now)
        if self.proposal and self.proposal['control']:
            for role,p in self.peers.items():
                if role==skip:continue
                assert p.receive(self.proposal['control'],now),p.error
                for key,value in values.items():p.update(key,value,now)
                def reset(ctx,r=role):self.resets[r]+=1
                ack=p.step(now,graph_ok=True,raw_count=0,pump_count=0,reset=reset)
                report=dict(values.get(role,{}),cycle_release_ack=ack)
                if role=='actor' and p.phase=='COMMITTED':report.update(state='WAIT_P',request_used=False,pump_on_feedback_seen=False,servo_sweep_feedback_seen=False)
                self.session.update(role,report,now)
        # Evaluate normal persistent core too, not just direct receipt bookkeeping.
        self.session.step(now)
        self.proposal=self.handback.step(self.session,now,graph_ok=True,raw_count=0,pump_count=0)
        return self.proposal
    def finish(self):
        for _ in range(90):
            r=self.tick()
            if r['consumed']:return r
        raise AssertionError((r,self.session.report(self.facts.now)))


@pytest.mark.parametrize('no_goal,after',[(False,False),(True,False),(False,True),(True,True)])
def test_exact_release_consumed_once_and_new_goal_keeps_same_session(no_goal,after):
    h=Harness(no_goal=no_goal,after=after);h.finish();c=h.session
    assert c.cycles.current is None and c.cycles.budget is None and CYCLE in c.cycles.retired
    assert c.armed and not c.closed and not c.p_used and not c.turn_permission and not c.cancel_requested
    assert c.reason=='NAV2_READY_NEW_GOAL' and set(h.resets.values())=={1}
    for _ in range(3):h.tick()
    assert set(h.resets.values())=={1} and h.handback.consumed
    d=data();d['vision']['fire']=dict(detected_now=False,confirmed=False)
    d['status']=[('34'*16,2)]
    r=feed(c,d,h.facts.now+.1)
    assert r['phase']=='NAVIGATION' and r['cycle'] is None and not r['closed']


def test_ACTIVE_registration_is_broadcast_before_first_PREPARE_even_for_immediate_X():
    h=Harness();r=h.tick()
    assert r['control']['phase']=='ACTIVE' and 'ACTIVE_REGISTRATION' in r['blockers'][0]
    assert h.handback.coordinator is None
    h.tick();assert h.handback.coordinator is not None


@pytest.mark.parametrize('explicit',[False,True])
def test_flame_outside_camera_waits_then_true_clear_retires_before_P(explicit):
    h=Harness(explicit=explicit)
    for _ in range(15):r=h.tick(ir_clear=False)
    assert not r['consumed'] and h.session.cycles.current is not None
    assert set(h.resets.values())=={0}
    assert h.finish()['consumed'] and not h.session.p_used


def test_missing_peer_registration_blocks_PREPARE_without_fault_or_UI_exit():
    h=Harness()
    for _ in range(20):r=h.tick(skip='ir_live')
    assert not r['consumed'] and h.handback.coordinator is None
    assert h.session.closed is False and h.session.armed
    assert h.finish()['consumed']


@pytest.mark.parametrize('state',['fault','closed','unarmed'])
def test_fault_or_close_cannot_consume_receipt_or_leave_ACTIVE_control(state):
    h=Harness();h.tick()
    if state=='fault':h.session.stop_fault('LOST_PROOF')
    if state=='closed':h.session.closed=True
    if state=='unarmed':h.session.armed=False
    r=h.handback.step(h.session,h.facts.now,graph_ok=True,raw_count=0,pump_count=0)
    assert r['control'] is None and not r['consumed'] and h.session.cycles.current is not None


def test_postpulse_review_from_other_cycle_cannot_authorize_release():
    h=Harness(after=True)
    for _ in range(30):
        h.tick();c=h.handback.coordinator
        if c is not None:
            value,at=h.session.inputs['review'];value['cycle_context']=dict(value['cycle_context'],cycle_id='45'*16)
            r=h.handback.step(h.session,h.facts.now,graph_ok=True,raw_count=0,pump_count=0)
            assert not r['consumed'] and 'WAIT_POSTSPRAY_REVIEW' in r['release']['blockers']
            break
    else:raise AssertionError('coordinator not begun')


def test_boot_required_at_actual_ARM_and_epoch_change_stays_STOP_without_Q_exit():
    c=OperationalSessionCore(require_boot_identity=True);d=data();f=Chain()
    for i in range(35):
        f.tick();d['vision']['fire']=dict(detected_now=False,confirmed=False)
        r=feed(c,d,f.now,['A'] if i==34 else ())
    assert not c.armed
    for i in range(35):
        f.tick();d['identity']=f.master.inputs['identity'][0]
        r=feed(c,d,f.now,['A'] if i==34 else ())
    assert c.armed
    f.tick();d['identity']=dict(f.master.inputs['identity'][0],connection_epoch=2)
    r=feed(c,d,f.now)
    assert not r['armed'] and r['fault']=='BOOT_OR_SERIAL_EPOCH_CHANGED' and not r['closed']
    f.tick();d['identity']=f.master.inputs['identity'][0];r=feed(c,d,f.now,['A'])
    assert not r['armed'] and r['fault']=='BOOT_OR_SERIAL_EPOCH_CHANGED'


def gateway_fixture():
    # Import fixture helpers only; no historical tests are collected/run.
    path=Path(__file__).parents[1]/'test_live_nav_stop_gateway.py'
    spec=importlib.util.spec_from_file_location('old_gateway_fixture',path);m=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m);return m


@pytest.mark.parametrize('earlier_fault',[None,'stale','foreign_raw'])
def test_actual_gateway_Nav2_ABORTED_zero_release_then_new_UUID_only_when_healthy(earlier_fault):
    m=gateway_fixture();c=m.ready_core();m.navigate(c)
    old='ab'*16;new='cd'*16
    if earlier_fault:
        if earlier_fault=='stale':m.step(c,10.6,owned=True,raw=m.RAW_SOURCE)
        else:m.step(c,10.12,owned=True,raw=[('/', 'foreign', 'geometry_msgs/msg/Twist',m.ReliabilityPolicy.BEST_EFFORT)])
    idle=m.mission('IDLE','HOLD_STOP',goal=False);idle['perception']=dict(detected_now=False,confirmed=False)
    owned=True
    for i in range(30):
        now=10.2+i*.1
        c.update_statuses([(old,6)],now);c.update_mission(idle,now);c.update_nav(m.ZERO,now)
        c.update_cmd(m.ZERO,now);c.update_odom(m.ZERO,now);c.update_arm(True,now)
        c.update_actuator(dict(pump_pwm_commanded_on=False,servo_pwm_commanded_on=False,servo_phase='IDLE'),now)
        _,_,r,action=m.step(c,now+.001,owned=owned,raw=m.RAW_SOURCE if owned else [])
        if action=='RELEASE':c.publisher_released();owned=False
    assert not owned
    if earlier_fault:assert not r['normal_rearm_ready'];return
    assert r['normal_rearm_ready'] and r['stop_reason']=='NAV_GOAL_ABORTED'
    now+=.1;c.update_statuses([(old,6),(new,2)],now);nav=m.mission();nav['perception']=dict(detected_now=False,confirmed=False)
    c.update_mission(nav,now);c.update_nav(m.MOVING,now);c.update_cmd(m.ZERO,now);c.update_odom(m.ZERO,now);c.update_arm(True,now)
    _,_,r,action=m.step(c,now+.001)
    assert r['cycle_goal_uuid']==new and action is None
    _,_,r,action=m.step(c,now+.002)
    assert action=='CREATE' and r['state']=='ARMING'
