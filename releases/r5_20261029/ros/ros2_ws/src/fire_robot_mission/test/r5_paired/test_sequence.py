import pytest
from operational_entry import StartupSequence


def sequence_to(stage):
    s=StartupSequence(0);now=0
    for key in ('clock_ok','io','camera','pi_sealed','pose','yolo','operator_present'):
        if s.stage==stage:return s,now
        now+=1;s.step(now,{k:True for k in ('clock_ok','io','camera','pi_sealed','pose','yolo','operator_present')[:('clock_ok','io','camera','pi_sealed','pose','yolo','operator_present').index(key)+1]})
    return s,now


def test_exact_start_order_no_operator_before_pose_yolo_and_no_live_arm():
    s=StartupSequence(0);events=[]
    for now,key in enumerate(('clock_ok','io','camera','pi_sealed','pose','yolo','operator_present'),1):events+=s.step(now,{k:True for k in ('clock_ok','io','camera','pi_sealed','pose','yolo','operator_present')[:('clock_ok','io','camera','pi_sealed','pose','yolo','operator_present').index(key)+1]})
    assert events==['START_PI','START_HEALTH','SEAL_PI','START_NAV','START_RVIZ','START_YOLO','START_CONSUMERS','START_OPERATOR']
    assert s.report()==dict(stage='READY_OBSERVE',done=True,error=None,retries=0,live=False)


def test_gate2_uses_same_start_order_but_reports_live_profile_truthfully():
    s=StartupSequence(0,live=True);events=[]
    keys=('clock_ok','io','camera','pi_sealed','pose','yolo','operator_present')
    for now,key in enumerate(keys,1):events+=s.step(now,{k:True for k in keys[:keys.index(key)+1]})
    assert events==['START_PI','START_HEALTH','SEAL_PI','START_NAV','START_RVIZ','START_YOLO','START_CONSUMERS','START_OPERATOR']
    assert s.report()==dict(stage='READY_GATE2',done=True,error=None,retries=0,live=True)


def test_startup_live_flag_must_be_literal_bool():
    with pytest.raises(ValueError):StartupSequence(0,live=1)


@pytest.mark.parametrize('stage',['PI_IO','SEAL_PI','POSE','YOLO','CONSUMERS'])
def test_stage_timeout_blocked_no_next_node(stage):
    s,now=sequence_to(stage);effects=s.step(now+s.LIMITS[stage],{})
    assert effects==['STOP'] and s.error=='STARTUP_TIMEOUT:'+stage


def test_camera_retry_exactly_once_before_nav_operator_bounded_wait():
    s,now=sequence_to('CAMERA');assert s.step(now+30,{})==['RETRY_PI','RESTART_HEALTH']
    assert s.retry==1 and s.stage=='RETRY_PI'
    s.step(now+31,{'retry_ready':True});s.step(now+32,{'io':True})
    assert s.stage=='CAMERA';assert s.step(now+62,{})==['STOP'] and s.error=='STARTUP_TIMEOUT:CAMERA'


def test_retry_late_reply_cannot_extend_wait_forever():
    s,now=sequence_to('CAMERA');s.step(now+30,{})
    assert s.step(now+50,{})==['STOP'] and s.done


@pytest.mark.parametrize('when',['CLOCK','PI_IO','CAMERA','SEAL_PI','POSE','YOLO','CONSUMERS'])
def test_child_failure_at_each_startup_stage_stops_without_next_effect(when):
    s,now=sequence_to(when);assert s.step(now+.1,{'fatal':'child1'})==['STOP'];assert s.error=='child1'


def test_monotonic_discontinuity_never_ready():
    s=StartupSequence(10);assert s.step(9,{'clock_ok':True})==['STOP']


@pytest.mark.parametrize('stage,missing',[('CAMERA','io'),('POSE','camera'),('YOLO','pose'),('YOLO','io'),('CONSUMERS','camera')])
def test_readiness_lost_during_next_stage_cannot_start_more_consumers(stage,missing):
    s,now=sequence_to(stage);observation={k:True for k in ('io','camera','pi_sealed','pose','yolo','operator_present')};observation[missing]=False
    assert s.step(now+.1,observation)==[] and s.stage==stage and not s.done


@pytest.mark.parametrize('stage',['PI_IO','SEAL_PI','POSE','YOLO','CONSUMERS'])
def test_late_readiness_cannot_override_expired_startup_deadline(stage):
    s,now=sequence_to(stage);obs={k:True for k in ('io','camera','pi_sealed','pose','yolo','operator_present')}
    assert s.step(now+s.LIMITS[stage],obs)==['STOP'] and s.done and s.error=='STARTUP_TIMEOUT:'+stage
