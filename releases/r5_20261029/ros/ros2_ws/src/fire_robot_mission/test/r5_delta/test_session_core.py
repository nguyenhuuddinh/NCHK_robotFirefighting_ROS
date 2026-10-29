"""New persistent orchestration boundary with real R4-shaped reports."""
import copy
import pytest
from test_installed_pulse import ready
from fire_robot_mission.operational_session_core import OperationalSessionCore

UID = 'a'*32
CYCLE = 'cd'*16
ZERO = [0.]*6


def data():
    d=ready()
    d['ir_raw']=dict(schema_version=1,valid=True,left=26000,center=26000,right=26000,
                     sample_age_ms=20.,receive_age_ms=20.)
    d['graph']=dict(valid=True,raw_count=0,raw_owner=None)
    d['status']=[(UID,2)]
    d['cmd']=ZERO;d['odom']=ZERO
    d['gateway'].update(odom_fresh=True)
    d['alignment']['command']={'linear_x_m_s':0.,'angular_z_rad_s':0.}
    d['alignment']['blockers']=[]
    d['preview']['preview']={'error_x_normalized':.01}
    d['ir_live'].update(total_progress_m=0.,step_count=0,approach_budget_exhausted=False)
    return d


def actor_ready(c,d):
    # Explicit actor receipt is now part of P admission, even in pure session
    # fixtures. Actual callback/actor behavior is covered by QA042557.
    from fire_robot_mission.operational_contract import FIRMWARE_ELF_SHA256
    c.identity_lease.bound=('ab'*16,1,FIRMWARE_ELF_SHA256)
    d['actor']=dict(state='WAIT_P',request_used=False,pump_publisher_registered=True,
        pulse_arm_active=False,pulse_request_pending=False,blockers=['OPERATOR_HEARTBEAT_MISSING'],
        pulse_request_feedback=dict(index=0,accepted=None,reason='NO_P_REQUEST'),
        r5_cycle_context=dict(cycle_id=c.cycles.current.cycle_id,nav_goal_uuid=c.cycles.current.nav_goal_uuid,
            bridge_instance_id='ab'*16,connection_epoch=1,elf_sha256=FIRMWARE_ELF_SHA256))


def feed(c,d,at,keys=()):
    for name,value in d.items():
        if name in c.INPUTS:c.update(name,copy.deepcopy(value),at)
    return c.step(at,keys)


def armed(*,rows=None,fire=False):
    c=OperationalSessionCore(lambda:CYCLE);d=data()
    d['vision']['fire'].update(confirmed=fire,detected_now=fire)
    if rows is not None:d['status']=rows
    for at in (1.+n*.1 for n in range(31)):feed(c,d,at)
    r=feed(c,d,4.,['A'])
    assert c.armed,r
    return c,d


def cycle(*,I=False):
    c,d=armed(fire=True)
    r=feed(c,d,4.1,['F']);assert c.cycles.current.nav_goal_uuid==UID,r
    d['status']=[(UID,5)]
    r=feed(c,d,4.2)
    assert r['phase']=='WAIT_IR_I',r
    if I:
        d['ir']['zone']=d['ir']['ir_band']='APPROACH'
        d['ir']['filtered_counts']['right']=5000
        d['ir_live'].update(state='MOVING',stop_reason='',publisher_owned=True)
        r=feed(c,d,4.3,['I']);assert r['operator_feedback']['accepted'],r
    return c,d


@pytest.mark.parametrize('status',[4,5,6])
def test_terminal_navigation_preserves_session_and_new_goal_identity(status):
    c,d=armed()
    d['status']=[(UID,status)]
    d['gateway']['normal_rearm_ready']=True
    r=feed(c,d,5.)
    assert r['phase']=='ARMED_IDLE' and r['reason']=='NAV2_READY_NEW_GOAL'
    d['status']=[(UID,status),('ef'*16,2)]
    r=feed(c,d,5.1)
    assert r['phase']=='NAVIGATION' and r['cycle'] is None and not r['closed']


@pytest.mark.parametrize('rows',[[],[(UID,4)],[(UID,5)],[(UID,6)]])
def test_fire_without_active_goal_latches_context_and_visible_missing_handshake(rows):
    c,d=armed(rows=rows,fire=True)
    r=feed(c,d,4.1,['F'])
    assert r['cycle']==dict(cycle_id=CYCLE,nav_goal_uuid=None)
    assert r['phase']=='WAIT_NO_GOAL_HANDSHAKE'
    assert not any(r['intents'][k] for k in ('navigation','alignment','ir','pulse'))
    assert r['closed'] is False and c.inputs['status'][0]==rows


def test_no_goal_new_goal_is_blocked_without_overwriting_fire_context():
    c,d=armed(rows=[],fire=True);feed(c,d,4.1,['F'])
    d['status']=[(UID,2)]
    r=feed(c,d,4.2)
    assert r['phase']=='GOAL_BLOCKED' and r['cycle']['nav_goal_uuid'] is None
    assert not r['intents']['navigation'] and not r['closed']


def test_I_early_key_is_discarded_then_admitted_once_without_new_budget():
    c,d=armed(fire=True)
    r=feed(c,d,4.1,['I']);assert not r['operator_feedback']['accepted']
    feed(c,d,4.2,['F']);d['status']=[(UID,5)]
    r=feed(c,d,4.3);assert r['phase']=='WAIT_IR_I'
    assert r['budget']['requested_at'] is None
    r=feed(c,d,4.4,['I']);assert r['operator_feedback']['accepted']
    first=r['budget']['requested_at']
    r=feed(c,d,4.5,['I']);assert not r['operator_feedback']['accepted']
    assert r['budget']['requested_at']==first


def test_stationary_I_five_distinct_samples_then_separate_P():
    c,d=cycle();r=feed(c,d,4.3,['P']);assert not r['operator_feedback']['accepted']
    r=feed(c,d,4.4,['I']);assert r['phase']=='IR_CONFIRM'
    for n in range(4):
        d['ir']['ir_sample_seq']+=1
        r=feed(c,d,4.6+n*.11)
        assert r['phase']=='IR_CONFIRM' and not r['intents']['pulse']
    d['ir']['ir_sample_seq']+=1
    # Outer IR counts cannot replace the actual actor's stationary proof.
    r=feed(c,d,5.05);assert r['phase']=='IR_CONFIRM',r
    actor_ready(c,d)
    d['actor']['stationary_confirmation']=dict(state='COMPLETE',goal_uuid=UID,sample_count=5)
    r=feed(c,d,5.06,['P']);assert r['operator_feedback']['accepted'] and r['intents']['pulse']
    r=feed(c,d,5.07,['P']);assert not r['operator_feedback']['accepted'] and not r['closed']


def test_stationary_repeated_same_IR_sample_cannot_confirm_P():
    c,d=cycle();feed(c,d,4.3,['I'])
    for n in range(20):r=feed(c,d,4.4+n*.1)
    assert r['phase']=='IR_CONFIRM' and not r['intents']['pulse']


def test_target_loss_during_IR_STOPPING_step0_never_arms_alignment_or_IR():
    c,d=cycle(I=True)
    d['vision']['fire'].update(confirmed=False,detected_now=False)
    d['ir_live'].update(state='STOPPING',stop_reason='REALIGN_REQUIRED',step_count=0,total_progress_m=.08)
    r=feed(c,d,4.4)
    assert r['phase']=='IR_RELEASE' and not r['intents']['alignment'] and not r['intents']['ir']
    assert r['budget']['distance_m']==.08
    r=feed(c,d,80.)
    assert not r['closed'] and not r['intents']['ir'] and r['budget']['exhausted']


def test_release_wait_recenter_and_resume_keep_same_I_time_distance():
    c,d=cycle(I=True);start=c.cycles.budget.requested_at
    c.turn_permission=True
    d['ir_live'].update(state='STOPPING',stop_reason='REALIGN_REQUIRED',step_count=0,total_progress_m=.08)
    feed(c,d,4.5)
    d['ir_live'].update(state='COMPLETE',step_count=1,publisher_owned=False,command={'linear_x_m_s':0.,'angular_z_rad_s':0.})
    d['vision']['fire'].update(confirmed=False,detected_now=False)
    r=feed(c,d,4.6);assert r['phase']=='IR_RELEASE'
    feed(c,d,4.9)
    r=feed(c,d,5.2);assert r['phase']=='WAIT_TARGET' and not r['intents']['ir']
    d['vision']['fire'].update(confirmed=True,detected_now=True)
    d['preview']['state']='TARGET_LEFT';d['alignment']['state']='WAITING_FOR_ADMISSION'
    r=feed(c,d,5.3);assert r['phase']=='REALIGNING' and r['intents']['alignment']
    d['graph'].update(raw_count=1,raw_owner='live_alignment_takeover')
    d['alignment'].update(state='TURNING',publisher_owned=True)
    r=feed(c,d,5.4);assert r['phase']=='REALIGNING' and not r['intents']['ir']
    d['graph'].update(raw_count=0,raw_owner=None)
    d['preview']['state']='CENTERED';d['alignment'].update(state='COMPLETE',publisher_owned=False)
    d['ir_live'].update(state='WAITING_FOR_ADMISSION',stop_reason='',recovery_required=False)
    r=feed(c,d,5.5);assert r['phase']=='IR_RESUME' and r['intents']['ir'],r
    assert r['budget']['requested_at']==start and r['budget']['distance_m']==.08
    assert r['budget']['remaining_s']==pytest.approx(15.-(5.5-start))


def test_wrong_step_after_STOPPING_zero_is_visible_hard_fault():
    c,d=cycle(I=True)
    d['ir_live'].update(state='STOPPING',stop_reason='REALIGN_REQUIRED',step_count=0)
    feed(c,d,4.4)
    d['ir_live'].update(state='COMPLETE',step_count=2,publisher_owned=False)
    r=feed(c,d,4.5)
    assert r['fault']=='IR_STEP_REGRESSION_OR_SKIP' and not any(r['intents'].values())


def test_ir_progress_reset_or_wrong_goal_fault_does_not_restart():
    c,d=cycle(I=True)
    d['ir_live']['total_progress_m']=.12;feed(c,d,4.4)
    d['ir_live']['total_progress_m']=.01
    r=feed(c,d,4.5)
    assert r['fault']=='IR_DISTANCE_INVALID_OR_RESET' and not r['closed']
    r=feed(c,d,4.6,['A']);assert not r['armed']


def test_alignment_owns_raw_can_keep_turn_intent_without_granting_IR():
    c,d=cycle()
    c.turn_permission=True;d['alignment'].update(state='TURNING',publisher_owned=True)
    d['graph'].update(raw_count=1,raw_owner='live_alignment_takeover')
    r=feed(c,d,4.3)
    assert r['phase']=='ALIGNING' and r['intents']['alignment'] and not r['intents']['ir']
    d['vision']['fire'].update(confirmed=False,detected_now=False)
    r=feed(c,d,4.4)
    # Keep the existing owner armed to execute zero/release on target loss;
    # its live controller checks the missing target before commanding motion.
    assert r['phase']=='WAIT_TARGET' and r['intents']['alignment'] and not r['closed']
    assert not r['intents']['ir'] and not r['intents']['pulse']


@pytest.mark.parametrize('key',['X','Q','ESC','WINDOW_CLOSED'])
def test_explicit_cancel_or_stop_never_forces_automatic_new_goal(key):
    c,d=cycle();r=feed(c,d,4.3,[key])
    if key=='X':
        assert r['phase']=='CANCEL_WAIT_CLEAR' and r['intents']['cancel_cycle'] and not r['closed']
    else:
        assert r['closed'] and not any(r['intents'].values())


@pytest.mark.parametrize('fault',['graph','locked','unexpected_ON','clock'])
def test_hard_fault_stays_observable_and_does_not_auto_rearm(fault):
    c,d=cycle()
    at=4.3
    if fault=='graph':d['graph'].update(valid=False,fault='FOREIGN_OWNER')
    if fault=='locked':d['actuator']['servo_phase']='LOCKED'
    if fault=='unexpected_ON':d['actuator']['pump_pwm_commanded_on']=True
    if fault=='clock':at=4.1
    r=feed(c,d,at)
    assert r['fault'] and not r['closed'] and not any(r['intents'].values())


def test_complete_step0_cannot_reenter_or_admit_P():
    c,d=cycle(I=True)
    d['ir_live'].update(state='COMPLETE',step_count=0,stop_reason='SPRAY_RANGE_REACHED',publisher_owned=False)
    r=feed(c,d,4.4)
    assert r['fault']=='IR_COMPLETE_WITHOUT_RELEASED_STEP' and not r['intents']['pulse']


def test_cap_recenter_five_samples_P_but_never_another_forward_segment():
    c,d=cycle(I=True);c.turn_permission=True
    live=d['ir_live']
    live.update(state='STOPPING',step_count=0,total_progress_m=.305,stop_reason='APPROACH_TOTAL_DISTANCE_LIMIT',
        one_shot=True,cap_stop_policy='ZERO_RECENTER_SPRAY_ONLY_V1',
        forward_motion_inhibited=True,approach_budget_exhausted=True,
        approach_distance_limit_m=.30,approach_distance_m=.305,
        command={'linear_x_m_s':0.,'angular_z_rad_s':0.})
    feed(c,d,4.4)
    live.update(state='COMPLETE',step_count=1,publisher_owned=False)
    d['alignment']['state']='WAITING_FOR_ADMISSION';d['preview']['state']='TARGET_LEFT'
    feed(c,d,4.5);feed(c,d,4.8);r=feed(c,d,5.1)
    assert r['phase']=='REALIGNING' and r['intents']['alignment'] and not r['intents']['ir']
    d['alignment']['state']='COMPLETE';d['preview']['state']='CENTERED'
    r=feed(c,d,5.2)
    assert r['phase']=='IR_CAP_HOLD' and not r['intents']['ir']
    d['ir']['zone']=d['ir']['ir_band']='SPRAY_RANGE';d['ir']['filtered_counts']['right']=950
    for n in range(5):
        d['ir']['ir_sample_seq']+=1;r=feed(c,d,5.3+n*.11)
        assert not r['intents']['ir']
    assert r['phase']=='WAIT_P',r
    actor_ready(c,d)
    r=feed(c,d,5.85,['P']);assert r['operator_feedback']['accepted']
    assert r['intents']['pulse'] and not r['intents']['ir']


def test_time_cap_during_target_wait_cannot_resume_forward_on_reacquisition():
    c,d=cycle(I=True)
    d['ir_live'].update(state='STOPPING',step_count=0,stop_reason='REALIGN_REQUIRED',total_progress_m=.08)
    feed(c,d,4.4);d['ir_live'].update(state='COMPLETE',step_count=1,publisher_owned=False)
    d['vision']['fire'].update(confirmed=False,detected_now=False)
    feed(c,d,4.5);feed(c,d,4.8);feed(c,d,5.1)
    d['vision']['fire'].update(confirmed=True,detected_now=True)
    d['ir_live'].update(state='WAITING_FOR_ADMISSION',stop_reason='',recovery_required=False)
    r=feed(c,d,20.)
    assert r['phase']=='IR_CAP_HOLD' and r['budget']['exhausted'] and not r['intents']['ir']


@pytest.mark.parametrize('source',['vision','preview','ir'])
def test_stale_source_data_does_not_become_ready_P_or_close_session(source):
    c,d=cycle();feed(c,d,4.3,['I'])
    if source=='ir':d[source]['ir_receive_age_ms']=501
    else:d[source]['source']['age_ms']=301
    r=feed(c,d,4.4,['P'])
    assert not r['intents']['pulse'] and not r['closed'] and not r['operator_feedback']['accepted']


def test_nonfinite_IR_payload_cannot_kill_state_serialization():
    import json
    c,d=cycle();d['ir']['filtered_counts']['right']=float('nan')
    r=feed(c,d,4.3)
    assert r['ir_counts'] is None and r['ir_band'] is None
    json.dumps(r,allow_nan=False)


def test_missing_ticks_do_not_count_as_stable_standby_hold():
    c=OperationalSessionCore(lambda:CYCLE);d=data()
    feed(c,d,1.);r=feed(c,d,4.1,['A'])
    assert not r['armed'] and not r['operator_feedback']['accepted']


def test_invalid_status_keeps_FAULT_phase_and_no_permission():
    c,d=armed();d['status']=[('0'*32,2)]
    r=feed(c,d,4.1)
    assert r['fault']=='NAV_STATUS_INVALID' and r['phase']=='FAULT'
    assert not any(r['intents'].values())


def test_initial_I_can_propose_zero_only_admission_before_live_UUID_binding():
    from fire_robot_mission.ir_approach_live_core import IrApproachLiveCore
    c,d=cycle()
    d['ir']['zone']=d['ir']['ir_band']='APPROACH';d['ir']['filtered_counts']['right']=5000
    # The real live core starts unbound. This is not a fabricated ready report.
    actual=IrApproachLiveCore(enabled=True,allow_provisional_ir=True,integrated_handoff=True,
        continuous_approach=True,speed_m_s=.08,max_total_approach_m=.30)
    actual.update_arm(False,4.2)
    _,report,_=actual.step(4.2,{},False)
    assert report['goal_uuid'] is None
    d['ir_live']=report
    r=feed(c,d,4.3,['I'])
    assert r['operator_feedback']['accepted'] and r['intents']['ir'],r
    assert r['reason']=='I_ACCEPTED_WAIT_FIRST_IR_UUID_BIND'
    assert r['budget']['distance_m']==0. and not r['motion_authorized']


def test_bound_IR_UUID_change_is_not_new_budget_or_recoverable_no_goal():
    c,d=cycle(I=True);d['ir_live']['goal_uuid']='ef'*16
    r=feed(c,d,4.4)
    assert r['fault']=='IR_REPORT_UUID_CHANGED' and not any(r['intents'].values())
