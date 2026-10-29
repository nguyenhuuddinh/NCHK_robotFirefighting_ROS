"""Interrupted attempts keep all STOP/home/clear gates and immutable policy."""
import copy
import pytest
from test_release_core import Chain,TOKEN
from fire_robot_mission.cycle_release import CycleReleaseCoordinator


def chain():
    c=Chain(after_pulse=True)
    c.master=CycleReleaseCoordinator(lambda:TOKEN)
    c.master.begin(c.context,c.now,after_pulse=True,home_before=0,pulse_interrupted=True)
    return c


def aborted(c,d):
    d['actor'].update(state='ABORTED',request_used=True,start_home_count=0,
        r5_cycle_context=copy.deepcopy(c.context),pump_on_feedback_seen=False,servo_sweep_feedback_seen=False)
    d['review']=None  # No fictitious successful pulse or canonical review.


@pytest.mark.parametrize('bad',[None,'used','context','ON','home','bool_home','DONE'])
def test_interrupted_terminal_requires_bound_attempt_and_real_home_baseline(bad):
    c=chain()
    def altered(d):
        aborted(c,d);a=d['actor']
        if bad=='used':a['request_used']=False
        if bad=='context':a['r5_cycle_context']['cycle_id']='98'*16
        if bad=='ON':a['proposed_pump_on']=True
        if bad=='home':a['start_home_count']=1
        if bad=='bool_home':a['start_home_count']=False
        if bad=='DONE':a['state']='DONE'
    for _ in range(50):r=c.tick(mutate=altered)
    assert r['ready']==(bad is None)
    assert r['pulse_outcome']=='INTERRUPTED'


@pytest.mark.parametrize('bad',['missing_home','stale_ACT','epoch','raw','IR_fire'])
def test_interrupted_attempt_never_bypasses_fresh_STOP_clear_or_epoch(bad):
    c=chain()
    def altered(d):
        aborted(c,d)
        if bad=='missing_home':d['actuator']['home_command_completed_count']=0
        if bad=='stale_ACT':d['actuator']['receive_age_ms']=1000.
        if bad=='epoch':d['identity']['connection_epoch']=2
        if bad=='IR_fire':d['ir']['filtered_counts']['right']=950
    for _ in range(40):r=c.tick(mutate=altered,fault='raw' if bad=='raw' else None)
    assert not r['ready'] and c.master.receipt is None


def test_interrupted_policy_cannot_downgrade_at_COMMIT():
    c=chain();c.tick(mutate=lambda d:aborted(c,d))
    message=c.master.control();message['phase']='COMMIT';message.pop('pulse_interrupted')
    assert not c.peers['actor'].receive(message,c.now)
    assert c.peers['actor'].error=='RELEASE_POLICY_CHANGED'
