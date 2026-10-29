"""R4 continuity at real 5 Hz cadence; no ROS or device initialization."""
import copy
import pytest
from test_session_core import data, feed
from fire_robot_mission.operational_session_core import OperationalSessionCore


def raw():
    return dict(schema_version=1,valid=True,left=26000,center=26000,right=26000,
                sample_age_ms=30.,receive_age_ms=5.)


def run(c, d, now, *, raw_at=None, raw_value=None, preview_at=None, preview_age=0., key=None):
    # Independent receipt times, not refreshed by every operator/UI tick.
    for name,value in d.items():
        if name not in ('ir','ir_raw') and name in c.INPUTS:c.update(name,value,now)
    if raw_at is not None and 'ir_raw' in c.INPUTS:
        c.update('ir_raw',raw() if raw_value is None else raw_value,raw_at)
    if preview_at is not None:
        value=copy.deepcopy(d['ir']);value['ir_receive_age_ms']=preview_age
        c.update('ir',value,preview_at)
    return c.step(now,[key] if key else ())


@pytest.mark.parametrize('phase_ms', [110,150,190])
def test_five_hz_preview_phase_does_not_prevent_three_second_standby(phase_ms):
    c=OperationalSessionCore();d=data();last_raw=0.;last_preview=-1.
    for ms in range(0,4101,10):
        now=1.+ms/1000
        raw_due=ms%200==0
        if raw_due:last_raw=now
        preview_due=ms>=phase_ms and (ms-phase_ms)%200==0
        r=run(c,d,now,raw_at=last_raw if raw_due else None,
              preview_at=now if preview_due else None,preview_age=(now-last_raw)*1000)
    assert r['reason']=='READY_PRESS_A',r['startup_readiness']
    assert not r['armed']
    assert run(c,d,5.11,raw_at=5.1,key='A')['armed']


@pytest.mark.parametrize('field,value',[('valid',False),('left',-1),('right',True),
    ('sample_age_ms',301.),('receive_age_ms',float('nan')),('schema_version',True)])
def test_invalid_raw_cannot_arm_even_when_preview_is_fresh(field,value):
    c=OperationalSessionCore();d=data();bad=raw();bad[field]=value
    for i in range(41):r=run(c,d,1+i*.1,raw_at=1+i*.1,raw_value=bad,preview_at=1+i*.1)
    assert not run(c,d,5.1,raw_at=5.1,raw_value=bad,preview_at=5.1,key='A')['armed']
    assert 'ir_continuity' in r['startup_readiness']['blockers']


def qualified():
    c=OperationalSessionCore();d=data()
    for i in range(41):run(c,d,1+i*.1,raw_at=1+i*.1,preview_at=1+i*.1)
    return c,d


def test_four_hundred_ms_gap_keeps_qualification_but_A_waits_for_next_raw():
    c,d=qualified()
    r=run(c,d,5.2,preview_at=5.2)
    r=run(c,d,5.4,preview_at=5.4,key='A')
    assert not r['armed'] and r['startup_readiness']['arm_request_pending']
    assert r['startup_readiness']['stable_s']>3.
    assert not r['startup_readiness']['ir']['grant_fresh']
    r=run(c,d,5.45,raw_at=5.45,preview_at=5.45)
    assert r['armed'] and r['operator_feedback']['reason']=='ARM_ACCEPTED'
    assert r['operator_feedback']['requested_at']==5.4


@pytest.mark.parametrize('break_proof', ['ir_missing','ir_invalid','cmd','odom','actuator','graph','loop_gap'])
def test_lost_proof_discards_pending_A_no_automatic_rearm(break_proof):
    c,d=qualified();run(c,d,5.2,preview_at=5.2);run(c,d,5.4,preview_at=5.4,key='A')
    assert c.arm_requested_at==5.4
    if break_proof=='ir_invalid':c.update('ir_raw',dict(raw(),valid=False),5.45)
    elif break_proof in ('cmd','odom'):d[break_proof]=[1.]*6
    elif break_proof=='actuator':d['actuator']['pump_pwm_commanded_on']=True
    elif break_proof=='graph':d['graph']['valid']=False
    end=5.8 if break_proof=='loop_gap' else 5.65 if break_proof=='ir_missing' else 5.45
    r=run(c,d,end,preview_at=end)
    assert not r['armed'] and c.arm_requested_at is None
    d=data()
    for i in range(40):r=run(c,d,6+i*.1,raw_at=6+i*.1,preview_at=6+i*.1)
    assert r['reason']=='READY_PRESS_A' and not r['armed']


def test_pending_A_expires_and_repeats_do_not_extend_deadline():
    c,d=qualified();run(c,d,5.2,preview_at=5.2);run(c,d,5.4,preview_at=5.4,key='A')
    for i in range(1,21):
        now=5.4+i*.1
        r=run(c,d,now,raw_at=now-.4,preview_at=now,key='A' if i<20 else None)
    assert not r['armed'] and c.arm_requested_at is None
    assert r['operator_feedback']['reason']=='ARM_REQUEST_EXPIRED_PRESS_A_AGAIN'
    assert not run(c,d,7.5,raw_at=7.5,preview_at=7.5)['armed']


@pytest.mark.parametrize('key',['Q','ESC','WINDOW_CLOSED'])
def test_stop_cancels_pending_A(key):
    c,d=qualified();run(c,d,5.2,preview_at=5.2);run(c,d,5.4,preview_at=5.4,key='A')
    r=run(c,d,5.45,raw_at=5.45,preview_at=5.45,key=key)
    assert r['closed'] and not r['armed'] and c.arm_requested_at is None


def test_early_A_is_not_remembered_and_motion_IR_still_expires_at_500ms():
    c=OperationalSessionCore();d=data()
    run(c,d,1.,raw_at=1.,preview_at=1.,key='A')
    for i in range(1,41):r=run(c,d,1+i*.1,raw_at=1+i*.1,preview_at=1+i*.1)
    assert r['reason']=='READY_PRESS_A' and not r['armed'] and c.arm_requested_at is None
    assert c.get('ir',5.501) is None
    assert c._standby_proof(5.301)['ir']['continuity']


def test_other_keys_cannot_mislabel_deferred_A_grant():
    c,d=qualified();run(c,d,5.2,preview_at=5.2);run(c,d,5.4,preview_at=5.4,key='A')
    index=c.feedback['index']
    run(c,d,5.42,preview_at=5.42,key='P')
    r=run(c,d,5.45,raw_at=5.45,preview_at=5.45)
    assert r['armed'] and r['operator_feedback']['key']=='A'
    assert r['operator_feedback']['index']==index and not r['intents']['pulse']
