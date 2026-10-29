import copy
import pytest
from fire_robot_mission.operational_stop import StopVerification
from fire_robot_mission.operational_contract import FIRMWARE_ELF_SHA256 as ELF


def inputs(n):
    act=dict(schema_version=1,mode='COMMAND_TELEMETRY_ONLY',pump_pwm_ready=True,
        pump_pwm_commanded_on=False,servo_pwm_commanded_on=False,servo_phase='IDLE',
        receive_age_ms=20,esp_ms=round(n*1000),home_command_completed_count=0)
    identity=dict(schema_version=1,frame_protocol=2,source='SERIAL_V2_BOOT_WITH_LIVE_STATE',valid=True,
        session_ready=True,telemetry_healthy=True,state_age_ms=20,state_esp_ms=round(n*1000),
        bridge_instance_id='ab'*16,connection_epoch=1,elf_sha256=ELF)
    return {k:(v,n) for k,v in dict(cmd=[0.]*6,odom=[0.]*6,actuator=act,identity=identity).items()}


def step(c,n,change=None,**kw):
    d=inputs(n)
    if change:change(d)
    return c.step(n,d,sources_ok=kw.get('sources_ok',True),raw_count=kw.get('raw_count',0),pump_count=kw.get('pump_count',0))


def test_stop_needs_continuous_hold_and_reports_no_motion_authority():
    c=StopVerification();c.begin(1.)
    for i in range(5):assert not step(c,1.+i*.1)['done']
    r=step(c,1.51)
    assert r['done'] and r['confirmed'] and not r['motion_authorized'] and not r['pulse_replayed']


@pytest.mark.parametrize('bad',['raw','pump','cmd','odom','ACT_ON','ACT_stale','identity_stale','sources'])
def test_missing_stop_proof_times_out_without_fake_confirmation(bad):
    c=StopVerification();c.begin(1.)
    def change(d):
        if bad=='cmd':d['cmd'][0][0]=.01
        if bad=='odom':d['odom'][0][-1]=.03
        if bad=='ACT_ON':d['actuator'][0]['pump_pwm_commanded_on']=True
        if bad=='ACT_stale':d['actuator'][0]['receive_age_ms']=351
        if bad=='identity_stale':d['identity'][0]['state_age_ms']=351
    for i in range(42):
        r=step(c,1.+i*.1,change,raw_count=int(bad=='raw'),pump_count=int(bad=='pump'),sources_ok=bad!='sources')
    assert r['done'] and not r['confirmed'] and r['reason']=='STOP_UNCONFIRMED' and r['blockers']


def test_actor_ON_requires_incremented_home_before_context_can_close():
    c=StopVerification();c.begin(1.)
    def change(d):d['actor']=(dict(pump_on_feedback_seen=True,start_home_count=0),d['actuator'][1])
    for i in range(7):assert not step(c,1.+i*.1,change)['confirmed']
    def home(d):change(d);d['actuator'][0]['home_command_completed_count']=1
    for i in range(7):r=step(c,1.7+i*.1,home)
    assert r['confirmed'] and r['home_required'] and r['home_before']==0


def test_duplicate_ACT_samples_with_reset_receive_age_cannot_hold_confirmed():
    c=StopVerification();c.begin(1.)
    for i in range(42):r=step(c,1.+i*.1,lambda d:d['actuator'][0].update(esp_ms=1000))
    assert r['done'] and not r['confirmed'] and 'WAIT_NEW_ACT_SAMPLE' in r['blockers']


def test_sampling_gap_resets_hold_and_repeated_Q_does_not_extend_deadline():
    c=StopVerification();c.begin(1.)
    step(c,1.);step(c,1.1)
    c.begin(1.5,cause='Q');assert c.start==1. and c.cause=='Q'
    assert not step(c,1.5)['confirmed']
    assert step(c,1.91)['confirmed'] is False  # another >350ms sampling gap
    for i in range(7):r=step(c,2.+i*.1)
    assert r['confirmed']


@pytest.mark.parametrize('fault',['clock','ACT_clock','epoch'])
def test_hard_stop_proof_changes_never_auto_confirm(fault):
    c=StopVerification();c.begin(1.);step(c,1.)
    if fault=='clock':r=step(c,.9)
    else:
        def change(d):
            if fault=='ACT_clock':d['actuator'][0]['esp_ms']=900
            else:d['identity'][0]['connection_epoch']=2
        r=step(c,1.1,change)
        for i in range(41):r=step(c,1.2+i*.1)
    assert r['fault'] and not r['confirmed']


@pytest.mark.parametrize('fault',['invalid_epoch','wrong_ELF','MCU_fault'])
def test_invalid_new_identity_cannot_be_hidden_by_old_healthy_payload_returning(fault):
    c=StopVerification();c.begin(1.);step(c,1.)
    def change(d):
        value=d['identity'][0];value['valid']=False
        if fault=='invalid_epoch':value['connection_epoch']=2
        if fault=='wrong_ELF':value['elf_sha256']='0'*64
        if fault=='MCU_fault':value['reason']='MCU_CLOCK_REGRESSION'
    step(c,1.1,change)
    for i in range(41):r=step(c,1.2+i*.1)
    assert r['done'] and not r['confirmed'] and r['fault']
